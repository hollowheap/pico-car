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
    (void)p_ctx;
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
        cfg.speed[wheel]          = (pid_config_t){256U, 256U, 2U, 300000U, 100000U, 600000U, 400U};
    }
    cfg.sampling.max_interval_us = 100000U;
    cfg.max_command_ttl_us       = TTL_US;
    cfg.max_control_gap_us       = 100000U;
    cfg.reversal_brake_us        = 100000U;
    return cfg;
}

static motion_command_t drive (uint32_t now_us, int32_t speed)
{
    motion_command_t cmd = {1U, now_us, TTL_US, {speed, speed}};

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

    /* Still faulted before the rest time, with good feedback. */
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

    before = g_applies;
    advance(&in, false);
    cmd = drive(in.timestamp_us, SPEED_UM_S);
    controller_tick(&c, &in, in.timestamp_us, &cmd, false);
    assert(MOTION_RUNNING == c.status.state);
    assert(g_applies > before);
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
    uint32_t const renewed  = 0xFFFFFF00U;
    uint32_t const edge     = renewed + lease_us;
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

    for (offset = 0U; offset < 40U; offset++)
    {
        uint32_t const now = edge - 20U + offset;

        assert(motion_permit_lapsed(true, renewed, lease_us, now) ==
               (motion_permit_elapsed_ms(true, renewed, now) > 200U));
    }
}

int main (void)
{
    test_no_edges_is_no_fault_and_output_is_bounded();
    test_feedback_fault_clears_after_rest();
    test_old_command_does_not_resume();
    test_command_fault_stays_latched();
    test_missing_capture_raises_feedback();
    test_odometry_valid_recovers_per_sample();
    test_invalid_samples_count_survives_a_clean_sample();
    test_permit_lapse_and_age();
    puts("test_motion_controller: 8 tests passed");
    return 0;
}
