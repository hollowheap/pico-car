/** @file test_motion_controller.c
 * @brief Host tests for the motion controller fault policy: an encoder fault
 *        brakes the car, then clears once the wheels have stood still, so the
 *        car can drive again. A command fault still latches.
 */

#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "controller.h"
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
    cfg.feedback_timeout_us      = 2000000U;
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

int main (void)
{
    test_feedback_fault_clears_after_rest();
    test_old_command_does_not_resume();
    test_command_fault_stays_latched();
    puts("test_motion_controller: 3 tests passed");
    return 0;
}
