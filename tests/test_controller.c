/** @file test_controller.c
 * @brief Capture -> sampler -> two PIDs -> fake motor driver integration.
 */
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include "controller.h"
#include "guidance.h"
#include "command_slot.h"

typedef struct
{
    unsigned int apply_calls;
    unsigned int brake_calls;
    int16_t duty[2];
    controller_apply_result_t next_result;
    bool latched;
    bool brake_failure;
} fake_output_t;

static controller_t control;
static odometry_input_t input;
static fake_output_t output;
static uint32_t now;

static controller_apply_result_t apply (void * context, int16_t left, int16_t right)
{
    fake_output_t * p_fake = context;
    controller_apply_result_t result = p_fake->next_result;

    assert(!p_fake->latched);
    p_fake->apply_calls++;
    p_fake->next_result = CONTROLLER_APPLIED;
    if (result == CONTROLLER_APPLIED)
    {
        p_fake->duty[0] = left;
        p_fake->duty[1] = right;
    }
    else
    {
        p_fake->duty[0] = 0;
        p_fake->duty[1] = 0;
    }
    return result;
}

static bool brake (void * context, bool latch)
{
    fake_output_t * p_fake = context;

    p_fake->brake_calls++;
    p_fake->latched = p_fake->latched || latch;
    p_fake->duty[0] = 0;
    p_fake->duty[1] = 0;
    return !p_fake->brake_failure;
}

static controller_config_t configuration (void)
{
    /* Synthetic units/gains/timing, deliberately not car tuning values. */
    controller_config_t config = {0};
    unsigned int wheel;

    for (wheel = 0U; wheel < 2U; wheel++)
    {
        config.sampling.wheel[wheel] = (odometry_scale_t){1000U, 1000U,
            wheel == 0U ? -1 : 1};
        config.speed[wheel] = (pid_config_t){16384U, 0U, 0U,
            10000U, 100000U, 40000U, 300U};
    }
    config.sampling.max_interval_us = 100000U;
    config.max_control_gap_us = 50000U;
    config.max_command_ttl_us = 1000000U;
    config.feedback_timeout_us = 100000U;
    config.reversal_brake_us = 60000U;
    return config;
}

static void initialise (void)
{
    controller_config_t config = configuration();
    controller_output_t callbacks = {&output, apply, brake};

    (void)memset(&output, 0, sizeof(output));
    (void)memset(&input, 0, sizeof(input));
    now = UINT32_MAX - 10000U;
    input.timestamp_us = now;
    assert(controller_init(&control, &config, &input, &callbacks));
    assert(output.brake_calls == 1U && !output.latched);
}

static motion_command_t command (int32_t left, int32_t right, uint32_t ttl)
{
    motion_command_t next = {1U, now + 20000U, ttl, {left, right}};

    return next;
}

static void tick (const motion_command_t * p_command, bool stop)
{
    now += 20000U;
    input.timestamp_us = now;
    controller_tick(&control, &input, now, p_command, stop);
}

static void assert_fault (motion_fault_t expected)
{
    assert(control.status.state == MOTION_FAULT);
    assert(control.status.fault == expected);
    assert(output.latched && output.duty[0] == 0 && output.duty[1] == 0);
}

int main (void)
{
    motion_command_t next;
    unsigned int calls;
    unsigned int i;

    initialise();
    tick(NULL, false);
    assert(control.status.state == MOTION_IDLE && output.apply_calls == 0U);
    next = command(1000, 2000, 1000000U);
    tick(&next, false);
    assert(control.status.state == MOTION_RUNNING);
    assert(output.duty[0] == 250 && output.duty[1] == 300);
    input.wheel[0].negative += 10U;
    input.wheel[1].positive += 10U;
    tick(NULL, false);
    assert(output.duty[0] == 125 && output.duty[1] == 300);
    calls = output.apply_calls;
    controller_tick(&control, NULL, now, NULL, true);
    assert(control.status.state == MOTION_STOPPED && output.latched);
    assert(!control.status.sample_available);
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    assert(output.apply_calls == calls);

    initialise();
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    next = command(0, 0, 1000000U);
    tick(&next, false);
    assert(control.status.state == MOTION_IDLE && !output.latched);
    assert(output.duty[0] == 0 && output.duty[1] == 0);
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    assert(control.status.state == MOTION_RUNNING && output.duty[0] == 250);

    initialise();
    for (i = 0U; i < 6U; i++)
    {
        next = command(1000, 1000, 1000000U);
        next.id = i + 1U;
        tick(&next, false);
    }
    assert_fault(MOTION_FAULT_NO_EDGES); /* Refresh cannot hide missing edges. */
    calls = output.apply_calls;
    tick(&next, false);
    assert(output.apply_calls == calls);

    initialise();
    next = command(1000, 1000, 60000U);
    tick(&next, false);
    tick(NULL, false);
    tick(NULL, false);
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    assert_fault(MOTION_FAULT_EXPIRED); /* New arrivals cannot mask expiry. */

    initialise();
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    next = command(-1000, -1000, 1000000U);
    tick(&next, false);
    assert(control.status.state == MOTION_REVERSING && output.duty[0] == 0);
    calls = output.apply_calls;
    tick(NULL, false);
    tick(NULL, false);
    assert(output.apply_calls == calls);
    tick(NULL, false);
    assert(control.status.state == MOTION_RUNNING && output.duty[0] == -250);

    initialise();
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    next = command(0, 0, 1000000U);
    tick(&next, false);
    next = command(-1000, -1000, 1000000U);
    tick(&next, false);
    assert(control.status.state == MOTION_REVERSING); /* Zero cannot bypass dwell. */
    tick(NULL, false);
    tick(NULL, false);
    assert(output.duty[0] == -250);

    initialise();
    output.next_result = CONTROLLER_BRAKED_RETRY;
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    assert(control.status.state == MOTION_REVERSING);
    tick(NULL, false);
    tick(NULL, false);
    assert(output.apply_calls == 1U);
    tick(NULL, false);
    assert(output.apply_calls == 2U && output.duty[0] == 250);

    initialise();
    output.next_result = CONTROLLER_OUTPUT_FAILED;
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    assert_fault(MOTION_FAULT_OUTPUT);

    initialise();
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    input.wheel[1].invalid++;
    calls = output.apply_calls;
    tick(NULL, false);
    assert_fault(MOTION_FAULT_FEEDBACK);
    assert(output.apply_calls == calls); /* No partial left-wheel apply. */

    initialise();
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    calls = output.apply_calls;
    input.wheel[0].negative += 60U;
    input.wheel[1].positive += 60U;
    tick(NULL, false);
    assert(control.status.state == MOTION_RUNNING);
    assert(output.duty[0] == 0 && output.duty[1] == 0);
    assert(output.apply_calls == calls); /* Overspeed brakes, never counter-drives. */

    initialise();
    for (i = 0U; i < 100U; i++)
    {
        input.wheel[0].negative += 10U;
        next = command(1000, 0, 100000U);
        tick(&next, false);
        assert(control.status.state == MOTION_RUNNING);
        assert(output.duty[0] == 125 && output.duty[1] == 0 && !output.latched);
    } /* Healthy refresh/progress stream; idle right wheel is not a stall. */

    initialise();
    next = command(1000, 1000, 1000000U);
    tick(&next, false);
    now += 60000U;
    input.timestamp_us = now;
    controller_tick(&control, &input, now, NULL, false);
    assert_fault(MOTION_FAULT_GAP);

    initialise();
    next = command(1000, 1000, 1000000U);
    now += 20000U;
    input.timestamp_us = now;
    controller_tick(&control, &input, now + 40001U, &next, false);
    assert_fault(MOTION_FAULT_FEEDBACK);

    initialise();
    next = command(10001, 1000, 1000000U);
    tick(&next, false);
    assert_fault(MOTION_FAULT_COMMAND);

    initialise();
    output.brake_failure = true;
    tick(NULL, true);
    assert_fault(MOTION_FAULT_OUTPUT);
    assert(control.status.brake_failed);
    {
        controller_config_t config = configuration();
        controller_output_t callbacks = {&output, apply, brake};

        initialise();
        config.speed[0].ki_q16 = 65536U;
        assert(controller_init(&control, &config, &input, &callbacks));
        next = command(1000, 1000, 1000000U);
        tick(&next, false);
        assert(output.duty[0] == 270);
        next = command(900, 1000, 1000000U);
        tick(&next, false);
        assert(output.duty[0] == 263); /* I=20+18 persists through steering update. */
        assert(!controller_config_valid(NULL));
        config.speed[0].kp_q16 = 0U;
        config.speed[0].ki_q16 = 0U;
        assert(!controller_config_valid(&config)); /* No D-only startup. */
        output.brake_failure = true;
        next = command(0, 0, 1000000U);
        tick(&next, false);
        assert_fault(MOTION_FAULT_OUTPUT);
        assert(control.status.brake_failed);
        initialise();
        config = configuration();
        assert(controller_init(&control, &config, &input, &callbacks));
        next = command(1000, 1000, 1000000U);
        input.wheel[0].invalid++;
        tick(&next, false);
        assert_fault(MOTION_FAULT_FEEDBACK);
        initialise();
        assert(controller_init(&control, &config, &input, &callbacks));
        next = command(1000, 1000, 1000000U);
        tick(&next, false);
        assert(control.status.state == MOTION_RUNNING);
    }
    controller_tick(NULL, NULL, 0U, NULL, true);
    {
        command_slot_t inbox;
        const uint32_t limits[2] = {10000U, 10000U};
        guidance_request_t guidance = {GUIDANCE_STRAIGHT, 1000, 400, 0U, 100000U};
        motion_command_t plan;
        bool stop;

        initialise();
        assert(command_slot_init(&inbox, 1000000U));
        for (i = 0U; i < 3U; i++)
        {
            guidance.action = (guidance_action_t)i;
            guidance.sensed_us = now;
            assert(guidance_plan(&guidance, limits, 1000000U, now, &plan));
            assert(command_slot_post(&inbox, &plan, now) == COMMAND_SLOT_ACCEPTED);
            assert(command_slot_take(&inbox, now + 20000U, &next, &stop) && !stop);
            tick(&next, false);
            assert(output.duty[0] == (i == 1U ? 100 : 250));
            assert(output.duty[1] == (i == 2U ? 100 : 250));
        }
        assert(command_slot_hold(&inbox, MOTION_HOLD_OBSTACLE, true, now));
        assert(command_slot_take(&inbox, now + 20000U, &next, &stop));
        tick(&next, false);
        assert(control.status.state == MOTION_IDLE && !output.latched);
        guidance.sensed_us = now;
        assert(guidance_plan(&guidance, limits, 1000000U, now, &plan));
        assert(command_slot_post(&inbox, &plan, now) == COMMAND_SLOT_HELD);
        assert(command_slot_hold(&inbox, MOTION_HOLD_OBSTACLE, false, now));
        guidance.action = GUIDANCE_STRAIGHT;
        assert(guidance_plan(&guidance, limits, 1000000U, now, &plan));
        assert(command_slot_post(&inbox, &plan, now) == COMMAND_SLOT_ACCEPTED);
        assert(command_slot_take(&inbox, now + 20000U, &next, &stop));
        tick(&next, false);
        assert(output.duty[0] == 0 && output.duty[1] == 0);
        assert(command_slot_take(&inbox, now + 20000U, &next, &stop));
        tick(&next, false);
        assert(output.duty[0] == 250 && output.duty[1] == 250);
        command_slot_stop(&inbox);
        assert(!command_slot_take(&inbox, now, &next, &stop) && stop);
        tick(NULL, stop);
        assert(control.status.state == MOTION_STOPPED && output.latched);
    }
    return 0;
}
