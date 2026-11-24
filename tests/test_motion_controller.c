/** @file test_motion_controller.c
 * @brief Host tests for the motion controller fault policy: an encoder fault
 *        brakes the car, then clears once the wheels have stood still, so the
 *        car can drive again. A command fault still latches. The run permit
 *        lease lapses on the same clock the motion task holds the wheels by.
 */

#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "motion_inbox.h"
#include "platform/log.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define TICK_US     10000U
#define TTL_US      200000U
#define SPEED_UM_S  100000
#define EDGES_TICK  12U /* about 100 mm/s at 2530 edges per 204 mm */

static int  g_applies;
static bool g_last_latch;

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

static motion_apply_result_t fake_apply (void * p_ctx, int16_t left, int16_t right)
{
    int16_t * const p_duty = p_ctx;

    if (p_duty != NULL)
    {
        p_duty[ENCODER_LEFT]  = left;
        p_duty[ENCODER_RIGHT] = right;
    }
    if ((left != 0) || (right != 0))
    {
        g_applies++;
    }
    return MOTION_APPLIED;
}

static bool fake_brake (void * p_ctx, bool latch)
{
    (void)p_ctx;
    g_last_latch = latch;
    return true;
}

static controller_config_t config (void)
{
    controller_config_t cfg = {0};
    unsigned int        wheel;

    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        cfg.sampling.wheel[wheel] = (motion_scale_t){204204U, 2530U, 1};
        cfg.speed[wheel]          = (pid_config_t){256U, 256U, 2U, 300000U, 100000U, 600000U, 400U, 0U, 0U, 0U};
    }
    cfg.sampling.max_interval_us = 100000U;
    cfg.max_command_ttl_us       = TTL_US;
    cfg.max_control_gap_us       = 100000U;
    cfg.reversal_brake_us        = 100000U;
    return cfg;
}

static motion_command_t drive (uint32_t now_us, int32_t speed)
{
    motion_command_t cmd = {1U, now_us, TTL_US, {speed, speed}, false};

    return cmd;
}

/* Advance the encoder counts by one tick of forward travel, or of invalid
 * steps when b_bad. */
static void advance (motion_input_t * p_in, bool b_bad)
{
    unsigned int wheel;

    p_in->timestamp_us += TICK_US;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (b_bad)
        {
            p_in->wheel[wheel].invalid += 1U;
        }
        else
        {
            p_in->wheel[wheel].positive += EDGES_TICK;
        }
    }
}

/* Drive with good feedback, then with invalid steps until FEEDBACK. */
static void to_feedback_fault (controller_t * p_c, motion_input_t * p_in)
{
    controller_config_t const cfg = config();
    motion_output_t const     out = {NULL, fake_apply, fake_brake};
    motion_command_t          cmd;
    int                       tick;

    *p_in = (motion_input_t){0};
    assert(controller_init(p_c, &cfg, p_in, &out));
    for (tick = 0; tick < 20; tick++)
    {
        advance(p_in, false);
        cmd = drive(p_in->timestamp_us, SPEED_UM_S);
        controller_tick(p_c, p_in, p_in->timestamp_us, &cmd, false);
    }
    assert(MOTION_RUNNING == p_c->status.state);
    for (tick = 0; (tick < 10) && (MOTION_FAULT != p_c->status.state); tick++)
    {
        advance(p_in, true);
        cmd = drive(p_in->timestamp_us, SPEED_UM_S);
        controller_tick(p_c, p_in, p_in->timestamp_us, &cmd, false);
    }
    assert(MOTION_FAULT == p_c->status.state);
    assert(MOTION_FAULT_FEEDBACK == p_c->status.fault);
}

static void test_invalid_feedback_respects_current_target (int32_t target_um_s)
{
    controller_t              c;
    motion_input_t            in  = {0};
    int16_t                   duty[ENCODER_WHEELS] = {0};
    controller_config_t const cfg = config();
    motion_output_t const     out = {duty, fake_apply, fake_brake};
    motion_command_t          cmd;
    int16_t                   previous;

    assert(controller_init(&c, &cfg, &in, &out));
    for (int tick = 0; tick < 20; tick++)
    {
        advance(&in, false);
        cmd = drive(in.timestamp_us, SPEED_UM_S);
        controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    }
    previous = duty[ENCODER_LEFT];
    assert(previous > 0);

    advance(&in, false);
    in.wheel[ENCODER_LEFT].invalid++;
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    cmd.target_um_s[ENCODER_LEFT] = target_um_s;
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);

    assert(MOTION_RUNNING == c.status.state);
    assert(!c.status.sample.wheel[ENCODER_LEFT].speed_valid);
    assert(duty[ENCODER_RIGHT] > 0);
    assert(duty[ENCODER_LEFT] == ((target_um_s > 0) ? previous : 0));
}

static void test_feedback_fault_clears_after_rest (void)
{
    controller_t     c;
    motion_input_t   in;
    motion_command_t cmd;
    uint32_t         waited = 0U;
    int              before = 0;

    to_feedback_fault(&c, &in);
    /* Braked, not latched: the PWM can drive again. */
    assert(!g_last_latch);
    motion_wheel_sample_t const before_fault = c.status.sample.wheel[ENCODER_LEFT];
    assert(before_fault.displacement_um > 0 && before_fault.invalid_samples > 0U);

    /* Count changes during the fault must not be added to reported travel. */
    while ((waited + TICK_US) < MOTION_FAULT_CLEAR_US)
    {
        advance(&in, false);
        waited += TICK_US;
        controller_tick(&c, &in, in.timestamp_us, NULL, false);
        assert(MOTION_FAULT == c.status.state);
    }

    /* After the rest time the fault clears and a new command drives. */
    advance(&in, false);
    controller_tick(&c, &in, in.timestamp_us, NULL, false);
    assert(MOTION_IDLE == c.status.state);
    assert(MOTION_FAULT_NONE == c.status.fault);

    in.timestamp_us += TICK_US;
    controller_tick(&c, &in, in.timestamp_us, NULL, false);
    assert(c.status.sample_available && c.status.sample.wheel[ENCODER_LEFT].odometry_valid);
    assert(c.status.sample.wheel[ENCODER_LEFT].displacement_um == before_fault.displacement_um);
    assert(c.status.sample.wheel[ENCODER_LEFT].velocity_um_s == 0);
    assert(c.status.sample.wheel[ENCODER_LEFT].invalid_samples == before_fault.invalid_samples);

    before = g_applies;
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_RUNNING == c.status.state);
    assert(g_applies > before);
    assert(c.status.sample.wheel[ENCODER_LEFT].displacement_um > before_fault.displacement_um);
}

static void test_glitch_tolerance_requires_fresh_capture (void)
{
    controller_config_t cfg = config();
    controller_t control;
    motion_input_t in = {0};
    motion_output_t const out = {NULL, fake_apply, fake_brake};

    cfg.speed[ENCODER_LEFT].max_sample_age_us = TICK_US;
    cfg.speed[ENCODER_RIGHT].max_sample_age_us = TICK_US;
    assert(controller_init(&control, &cfg, &in, &out));
    advance(&in, false);
    motion_command_t const cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&control, &in, in.timestamp_us, &cmd, false);
    assert(control.status.duty[0] > 0 && control.status.duty[1] > 0);
    advance(&in, true);
    controller_tick(&control, &in, in.timestamp_us + TICK_US + 1U, NULL, false);
    assert(MOTION_FAULT_FEEDBACK == control.status.fault);
    assert(control.status.duty[0] == 0 && control.status.duty[1] == 0);
}

/* One transient overspeed brakes that wheel without erasing its learned load.
 * Sustained overspeed may unwind the load only as far as zero drive. */
static void test_overspeed_memory (int8_t direction)
{
    controller_config_t cfg = config();
    controller_t control;
    motion_input_t in = {0};
    motion_output_t const out = {NULL, fake_apply, fake_brake};
    for (unsigned wheel = 0; wheel < ENCODER_WHEELS; ++wheel)
    {
        cfg.sampling.wheel[wheel] = (motion_scale_t){1000U, 100U, direction};
    }
    assert(controller_init(&control, &cfg, &in, &out));
    for (unsigned tick = 0; tick < 50U; ++tick)
    {
        in.timestamp_us += TICK_US;
        in.wheel[ENCODER_LEFT].positive += tick < 20U ? 90U : 120U;
        in.wheel[ENCODER_RIGHT].positive += 90U;
        motion_command_t cmd = drive(in.timestamp_us, direction * SPEED_UM_S);
        controller_tick(&control, &in, in.timestamp_us, &cmd, false);
        assert(control.status.state == MOTION_RUNNING);
        assert(direction * control.status.duty[ENCODER_RIGHT] > 0);
        if (tick >= 20U)
        {
            pid_t const * pid = &control.pid[ENCODER_LEFT];
            assert(control.status.duty[ENCODER_LEFT] == 0);
            assert(pid->has_previous && pid->previous_measurement == direction * 120000);
            assert(tick == 20U ? direction * pid->integral_q16 > 0 : direction * pid->integral_q16 >= 0);
        }
    }
    assert(control.pid[ENCODER_LEFT].integral_q16 == 0);
    in.timestamp_us += TICK_US;
    motion_command_t const stop = drive(in.timestamp_us, 0);
    controller_tick(&control, &in, in.timestamp_us, &stop, false);
    assert(control.status.state == MOTION_IDLE && control.status.duty[0] == 0 && control.status.duty[1] == 0);
    assert(!control.pid[0].has_previous && !control.pid[1].has_previous);
    assert(control.pid[0].integral_q16 == 0 && control.pid[1].integral_q16 == 0);
}

static void test_old_command_does_not_resume (void)
{
    controller_t   c;
    motion_input_t in;
    uint32_t       waited = 0U;

    to_feedback_fault(&c, &in);
    while (waited <= MOTION_FAULT_CLEAR_US)
    {
        advance(&in, false);
        waited += TICK_US;
        controller_tick(&c, &in, in.timestamp_us, NULL, false);
    }
    /* No new command: the car stays idle, not driving the old one. */
    advance(&in, false);
    controller_tick(&c, &in, in.timestamp_us, NULL, false);
    assert(MOTION_IDLE == c.status.state);
    assert((0 == c.status.duty[0]) && (0 == c.status.duty[1]));
}

static void test_persistent_encoder_fault (unsigned int bad_wheel)
{
    controller_t c;
    motion_input_t in;
    motion_command_t cmd;

    to_feedback_fault(&c, &in);
    int const before = g_applies;
    for (uint32_t waited = 0U; waited < 2U * MOTION_FAULT_CLEAR_US; waited += TICK_US)
    {
        advance(&in, false);
        in.wheel[bad_wheel].invalid++;
        controller_tick(&c, &in, in.timestamp_us, NULL, false);
        assert(c.status.state == MOTION_FAULT && c.status.fault == MOTION_FAULT_FEEDBACK);
        assert(c.status.duty[0] == 0 && c.status.duty[1] == 0 && !c.status.sample_available);
    }

    /* An outstanding command from the faulted period cannot restart the wheels. */
    advance(&in, false);
    cmd = drive(in.timestamp_us - TICK_US, SPEED_UM_S);
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    assert(c.status.state == MOTION_IDLE && c.status.fault == MOTION_FAULT_NONE);
    assert(c.status.target_um_s[0] == 0 && c.status.target_um_s[1] == 0);
    assert(g_applies == before);
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    assert(c.status.state == MOTION_RUNNING && g_applies > before);
}

static void test_fault_recovery_needs_fresh_captures (void)
{
    controller_t c;
    motion_input_t in;
    uint32_t now_us;

    to_feedback_fault(&c, &in);
    now_us = in.timestamp_us + MOTION_FAULT_CLEAR_US;
    controller_tick(&c, NULL, now_us, NULL, false);
    assert(c.status.state == MOTION_FAULT);
    in.timestamp_us = now_us;
    controller_tick(&c, &in, now_us, NULL, false);
    assert(c.status.state == MOTION_FAULT);
    controller_tick(&c, &in, now_us + TICK_US, NULL, false);
    assert(c.status.state == MOTION_FAULT);

    /* Even consecutive timestamps can be stale when a capture is delivered late. */
    advance(&in, false);
    now_us = in.timestamp_us + c.config.speed[0].max_sample_age_us + 1U;
    controller_tick(&c, &in, now_us, NULL, false);
    assert(c.status.state == MOTION_FAULT);
    in.timestamp_us = now_us;
    controller_tick(&c, &in, now_us, NULL, false);
    assert(c.status.state == MOTION_FAULT);
    advance(&in, false);
    controller_tick(&c, &in, in.timestamp_us, NULL, false);
    assert(c.status.state == MOTION_IDLE && c.status.fault == MOTION_FAULT_NONE);
}

/* A lapsed lease or a late control period brakes without the output latch and
 * clears in place like an encoder fault. Driving needs a new command. */
/* Ticks to reach RUNNING, and the most ticks a lapsed command takes to fault. */
#define RUN_UP_TICKS     (20)
#define FAULT_WAIT_TICKS (30)

static void test_timing_fault_clears_in_place (motion_fault_t reason)
{
    controller_t              c;
    motion_input_t            in  = {0};
    controller_config_t const cfg = config();
    motion_output_t const     out = {NULL, fake_apply, fake_brake};
    motion_command_t          cmd;
    int                       tick;

    assert(controller_init(&c, &cfg, &in, &out));
    for (tick = 0; tick < RUN_UP_TICKS; tick++)
    {
        advance(&in, false);
        cmd = drive(in.timestamp_us, SPEED_UM_S);
        controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    }
    assert(MOTION_RUNNING == c.status.state);
    if (MOTION_FAULT_GAP == reason)
    {
        in.timestamp_us += cfg.max_control_gap_us;
        advance(&in, false);
        controller_tick(&c, &in, in.timestamp_us, NULL, false);
    }
    else
    {
        for (tick = 0; (tick < FAULT_WAIT_TICKS) && (MOTION_FAULT != c.status.state); tick++)
        {
            advance(&in, false);
            controller_tick(&c, &in, in.timestamp_us, NULL, false);
        }
    }
    assert(MOTION_FAULT == c.status.state);
    assert(reason == c.status.fault);
    assert(!g_last_latch);
    assert((0 == c.status.duty[0]) && (0 == c.status.duty[1]));

    for (uint32_t waited = 0U; waited <= MOTION_FAULT_CLEAR_US; waited += TICK_US)
    {
        advance(&in, false);
        controller_tick(&c, &in, in.timestamp_us, NULL, false);
    }
    assert(MOTION_IDLE == c.status.state);
    assert(MOTION_FAULT_NONE == c.status.fault);
    assert((0 == c.status.target_um_s[0]) && (0 == c.status.target_um_s[1]));

    int const before = g_applies;
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_RUNNING == c.status.state);
    assert(g_applies > before);
}

static void test_command_fault_stays_latched (void)
{
    controller_t              c;
    motion_input_t            in  = {0};
    controller_config_t const cfg = config();
    motion_output_t const     out = {NULL, fake_apply, fake_brake};
    motion_command_t          cmd;
    int                       tick;

    assert(controller_init(&c, &cfg, &in, &out));
    advance(&in, false);
    cmd = drive(in.timestamp_us, 400000); /* above max_target_um_s */
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_FAULT_COMMAND == c.status.fault);
    assert(g_last_latch);
    for (tick = 0; tick < 100; tick++)
    {
        advance(&in, false);
        controller_tick(&c, &in, in.timestamp_us, NULL, false);
    }
    assert(MOTION_FAULT == c.status.state);
}

/* A wheel with no edges is no fault of the controller: the pilot judges it.
 * The output stays inside the duty limit of the speed loop. */
static void test_no_edges_is_no_fault_and_output_is_bounded (void)
{
    controller_t              c;
    motion_input_t            in  = {0};
    controller_config_t const cfg = config();
    motion_output_t const     out = {NULL, fake_apply, fake_brake};
    motion_command_t          cmd;
    int                       tick;

    assert(controller_init(&c, &cfg, &in, &out));
    for (tick = 0; tick < 400; tick++) /* 4 s, twice the old timeout */
    {
        in.timestamp_us += TICK_US;
        cmd = drive(in.timestamp_us, SPEED_UM_S);
        controller_tick(&c, &in, in.timestamp_us, &cmd, false);
        assert(MOTION_FAULT != c.status.state);
        assert(MOTION_FAULT_NONE == c.status.fault);
        assert(c.status.duty[0] <= (int16_t)cfg.speed[0].max_duty_permille);
        assert(c.status.duty[1] <= (int16_t)cfg.speed[1].max_duty_permille);
    }
    assert(MOTION_RUNNING == c.status.state);
    assert(c.status.duty[0] > 0);
    assert(c.status.sample_available);
}

/* A failed encoder read reaches the controller as no input: FEEDBACK. */
static void test_missing_capture_raises_feedback (void)
{
    controller_t              c;
    motion_input_t            in  = {0};
    controller_config_t const cfg = config();
    motion_output_t const     out = {NULL, fake_apply, fake_brake};
    motion_command_t          cmd;
    int                       tick;

    assert(controller_init(&c, &cfg, &in, &out));
    for (tick = 0; tick < 20; tick++)
    {
        advance(&in, false);
        cmd = drive(in.timestamp_us, SPEED_UM_S);
        controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    }
    assert(MOTION_RUNNING == c.status.state);
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&c, NULL, in.timestamp_us, &cmd, false);
    assert(MOTION_FAULT_FEEDBACK == c.status.fault);
}

/* Validity follows the current sample: a bad sample is invalid, the next
 * good one is valid again. */
static void test_odometry_valid_recovers_per_sample (void)
{
    odometry_t              odo;
    motion_input_t          in  = {0};
    motion_sample_t         out;
    controller_config_t const cfg = config();
    unsigned int            wheel;

    assert(odometry_init(&odo, &cfg.sampling, &in));
    advance(&in, false);
    assert(odometry_update(&odo, &in, &out));
    advance(&in, true);
    assert(odometry_update(&odo, &in, &out));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        assert(!out.wheel[wheel].odometry_valid);
    }
    advance(&in, false);
    assert(odometry_update(&odo, &in, &out));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        assert(out.wheel[wheel].odometry_valid);
    }
    in.timestamp_us += cfg.sampling.max_interval_us + 1U;
    assert(odometry_update(&odo, &in, &out));
    assert(!out.wheel[0].odometry_valid);
    advance(&in, false);
    assert(odometry_update(&odo, &in, &out));
    assert(out.wheel[0].odometry_valid);
}

/* A reader that polls every other sample sees an invalid one in the count. */
static void test_invalid_samples_count_survives_a_clean_sample (void)
{
    odometry_t              odo;
    motion_input_t          in  = {0};
    motion_sample_t         out;
    controller_config_t const cfg = config();
    uint8_t                 seen[ENCODER_WHEELS];
    unsigned int            wheel;

    assert(odometry_init(&odo, &cfg.sampling, &in));
    advance(&in, false);
    assert(odometry_update(&odo, &in, &out));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        seen[wheel] = out.wheel[wheel].invalid_samples;
    }
    advance(&in, true);
    assert(odometry_update(&odo, &in, &out));
    advance(&in, false);
    assert(odometry_update(&odo, &in, &out));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        assert(out.wheel[wheel].odometry_valid);
        assert(out.wheel[wheel].invalid_samples != seen[wheel]);
        seen[wheel] = out.wheel[wheel].invalid_samples;
    }
    advance(&in, false);
    assert(odometry_update(&odo, &in, &out));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        assert(out.wheel[wheel].invalid_samples == seen[wheel]);
    }
    in.timestamp_us += cfg.sampling.max_interval_us + 1U;
    assert(odometry_update(&odo, &in, &out));
    assert(out.wheel[0].invalid_samples != seen[0]);
}

/* The permit lapses once more than its lease has passed since the last renewal,
 * and at once when it was never renewed. Its age in whole ms, rounded up, is
 * over the lease exactly when it has lapsed, so the supervisor and the motion
 * task agree at every instant, across the 2^32-us wrap too. */
static void test_permit_lapse_and_age (void)
{
    uint32_t const lease_us = 200000U;
    uint64_t const renewed  = UINT64_C(0xffffff00);
    uint64_t const edge     = renewed + lease_us;
    uint32_t       offset   = 0U;

    assert(motion_permit_lapsed(false, renewed, lease_us, renewed));
    assert(motion_permit_elapsed_ms(false, renewed, renewed) == UINT32_MAX);

    assert(!motion_permit_lapsed(true, renewed, lease_us, renewed));
    assert(motion_permit_elapsed_ms(true, renewed, renewed) == 0U);
    assert(!motion_permit_lapsed(true, renewed, lease_us, edge));
    assert(motion_permit_elapsed_ms(true, renewed, edge) == 200U);
    assert(motion_permit_lapsed(true, renewed, lease_us, edge + 1U));
    assert(motion_permit_elapsed_ms(true, renewed, edge + 1U) == 201U);
    assert(motion_permit_elapsed_ms(true, renewed, edge + 500U) == 201U);
    assert(motion_permit_lapsed(true, renewed, lease_us, edge + UINT32_C(0x80000000)));

    for (offset = 0U; offset < 40U; offset++)
    {
        uint64_t const now = edge - 20U + offset;

        assert(motion_permit_lapsed(true, renewed, lease_us, now) ==
               (motion_permit_elapsed_ms(true, renewed, now) > 200U));
    }
}

static void test_single_stop_sample_brakes_and_latches (void)
{
    controller_t                  control;
    controller_config_t const     cfg = config();
    motion_output_t const         out = {NULL, fake_apply, fake_brake};
    motion_input_t                in = {0};
    motion_command_t              cmd;
    int                           applies;

    assert(controller_init(&control, &cfg, &in, &out));
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&control, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_RUNNING == control.status.state);

    applies = g_applies;
    advance(&in, false);
    controller_tick(&control, NULL, in.timestamp_us, NULL, true);
    assert(MOTION_STOPPED == control.status.state);
    assert(g_last_latch);

    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&control, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_STOPPED == control.status.state);
    assert(g_applies == applies);
}

static void test_stop_sample_during_feedback_fault (void)
{
    controller_t     control;
    motion_input_t   in;
    motion_command_t cmd;
    uint32_t         waited = 0U;
    int              applies;

    to_feedback_fault(&control, &in);
    assert(!g_last_latch);
    applies = g_applies;

    advance(&in, false);
    controller_tick(&control, NULL, in.timestamp_us, NULL, true);
    assert(MOTION_STOPPED == control.status.state);
    assert(g_last_latch);

    while (waited < MOTION_FAULT_CLEAR_US)
    {
        advance(&in, false);
        waited += TICK_US;
        controller_tick(&control, &in, in.timestamp_us, NULL, false);
    }
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&control, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_STOPPED == control.status.state);
    assert(g_applies == applies);
}

int main (int argc, char ** argv)
{
    if ((2 == argc) && (0 == strcmp(argv[1], "single_stop_sample")))
    {
        test_single_stop_sample_brakes_and_latches();
        return 0;
    }
    if ((2 == argc) && (0 == strcmp(argv[1], "stop_during_feedback_fault")))
    {
        test_stop_sample_during_feedback_fault();
        return 0;
    }
    if ((2 == argc) && (0 == strcmp(argv[1], "permit_lease")))
    {
        test_permit_lapse_and_age();
        return 0;
    }
    if (1 != argc)
    {
        (void)fprintf(stderr,
                      "Usage: %s [single_stop_sample|stop_during_feedback_fault|permit_lease]\n",
                      argv[0]);
        return 2;
    }
    test_no_edges_is_no_fault_and_output_is_bounded();
    test_overspeed_memory(1);
    test_overspeed_memory(-1);
    test_invalid_feedback_respects_current_target(SPEED_UM_S / 2);
    test_invalid_feedback_respects_current_target(0);
    test_invalid_feedback_respects_current_target(-SPEED_UM_S);
    test_glitch_tolerance_requires_fresh_capture();
    test_feedback_fault_clears_after_rest();
    test_old_command_does_not_resume();
    test_persistent_encoder_fault(ENCODER_LEFT);
    test_persistent_encoder_fault(ENCODER_RIGHT);
    test_fault_recovery_needs_fresh_captures();
    test_timing_fault_clears_in_place(MOTION_FAULT_EXPIRED);
    test_timing_fault_clears_in_place(MOTION_FAULT_GAP);
    test_command_fault_stays_latched();
    test_missing_capture_raises_feedback();
    test_odometry_valid_recovers_per_sample();
    test_invalid_samples_count_survives_a_clean_sample();
    test_permit_lapse_and_age();
    test_single_stop_sample_brakes_and_latches();
    test_stop_sample_during_feedback_fault();
    puts("test_motion_controller: all cases passed");
    return 0;
}
