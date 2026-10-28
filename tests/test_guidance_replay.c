/** @file test_guidance_replay.c
 * @brief Replay through the real planner/inbox/controller, with fake output.
 */
#include <assert.h>
#include <stddef.h>
#include "guidance_replay.h"
#include "speed_probe.h"

typedef struct
{
    int16_t duty[2];
    bool latched;
} fake_output_t;

static motion_apply_result_t apply (void * context, int16_t left, int16_t right)
{
    fake_output_t * output = context;

    assert(!output->latched);
    assert(left >= 0 && left <= (int16_t)SPEED_PROBE_DUTY_CAP);
    assert(right >= 0 && right <= (int16_t)SPEED_PROBE_DUTY_CAP);
    output->duty[0] = left;
    output->duty[1] = right;
    return MOTION_APPLIED;
}

static bool brake (void * context, bool latch)
{
    fake_output_t * output = context;

    output->duty[0] = 0;
    output->duty[1] = 0;
    output->latched = output->latched || latch;
    return true;
}

int main (void)
{
    motion_control_config_t config = speed_probe_configuration();
    const uint32_t limits[2] = {50000U, 50000U};
    const uint32_t reasons[2] = {MOTION_HOLD_OBSTACLE, MOTION_HOLD_LINE_LOST};
    const uint32_t boundaries[GUIDANCE_REPLAY_STAGES] = {
        0U, 600000U, 1200000U, 1800000U, 2000000U, 2200000U, 2400000U
    };
    motion_control_t control;
    motion_input_t input = {0};
    motion_inbox_t inbox;
    fake_output_t fake = {{0, 0}, false};
    motion_output_t output = {&fake, apply, brake};
    speed_probe_t probe;
    guidance_replay_frame_t frame = {MOTION_STEER_RIGHT, 99U, 99U};
    uint32_t started;
    uint32_t now = UINT32_MAX - 100000U;
    unsigned int stage;
    unsigned int ticks;
    unsigned int seen = 0U;
    bool resumed = false;
    bool stop;

    for (stage = 0U; stage < GUIDANCE_REPLAY_STAGES; stage++)
    {
        assert(guidance_replay_frame(boundaries[stage], &frame));
        assert(frame.stage == stage);
        if (stage != 0U)
        {
            assert(guidance_replay_frame(boundaries[stage] - 1U, &frame));
            assert(frame.stage == stage - 1U);
        }
    }
    frame.stage = 99U;
    assert(!guidance_replay_frame(SPEED_PROBE_DURATION_US, &frame));
    assert(frame.stage == 99U);
    assert(!guidance_replay_frame(UINT32_MAX, &frame));
    assert(!guidance_replay_frame(0U, NULL));

    speed_probe_init(&probe);
    assert(speed_probe_update(&probe, now, false, false) == SPEED_PROBE_WAIT);
    now += MOTOR_RELEASE_US;
    assert(speed_probe_update(&probe, now, false, false) == SPEED_PROBE_WAIT);
    now++;
    assert(speed_probe_update(&probe, now, true, false) == SPEED_PROBE_WAIT);
    now += MOTOR_PRESS_US;
    assert(speed_probe_update(&probe, now, true, false) == SPEED_PROBE_RUN);
    started = now;
    input.timestamp_us = now - 10000U;
    assert(motion_control_init(&control, &config, &input, &output));
    assert(motion_inbox_init(&inbox, config.max_command_ttl_us));

    for (ticks = 0U; ticks < 300U; ticks++)
    {
        motion_drive_request_t request;
        motion_command_t planned;
        motion_command_t taken;
        bool received;
        unsigned int reason;
        uint32_t old_holds = inbox.hold_mask;

        assert(speed_probe_update(&probe, now, true, false) == SPEED_PROBE_RUN);
        assert(guidance_replay_frame(now - started, &frame));
        seen |= 1U << frame.stage;
        for (reason = 0U; reason < 2U; reason++)
        {
            if ((frame.hold_mask & reasons[reason]) != 0U)
            {
                assert(motion_inbox_hold(&inbox, reasons[reason], true, now));
            }
        }
        for (reason = 0U; reason < 2U; reason++)
        {
            if ((frame.hold_mask & reasons[reason]) == 0U)
            {
                assert(motion_inbox_hold(&inbox, reasons[reason], false, now));
            }
        }
        request = (motion_drive_request_t){frame.action, 50000, 25000,
            now, SPEED_PROBE_TTL_US};
        assert(motion_request_plan(&request, limits, SPEED_PROBE_TTL_US, now, &planned));
        planned.id = ticks + 1U;
        assert(motion_inbox_post(&inbox, &planned, now) ==
            (frame.hold_mask != 0U ? MOTION_INBOX_HELD : MOTION_INBOX_ACCEPTED));
        received = motion_inbox_take(&inbox, now, &taken, &stop);
        assert(!stop);
        /* Artificial progress exercises signs/quality only; not a plant model. */
        input.wheel[0].negative += fake.duty[0] > 0 ? 1U : 0U;
        input.wheel[1].positive += fake.duty[1] > 0 ? 1U : 0U;
        input.timestamp_us = now;
        motion_control_tick(&control, &input, now, received ? &taken : NULL, stop);
        assert(control.status.fault == MOTION_FAULT_NONE);
        if (frame.hold_mask != 0U || old_holds != 0U)
        {
            assert(fake.duty[0] == 0 && fake.duty[1] == 0);
            assert(control.status.state == MOTION_IDLE && !fake.latched);
        }
        else
        {
            assert(control.status.target_um_s[0] == planned.target_um_s[0]);
            assert(control.status.target_um_s[1] == planned.target_um_s[1]);
            if (frame.stage == 6U)
            {
                resumed = true;
                assert(fake.duty[0] > 0 && fake.duty[1] > 0);
            }
        }
        now += 10000U;
    }
    assert(seen == (1U << GUIDANCE_REPLAY_STAGES) - 1U && resumed);
    assert(speed_probe_update(&probe, now, true, false) == SPEED_PROBE_FINISHED);
    assert(!probe.interrupted && !probe.gap_fault);
    motion_inbox_stop(&inbox);
    motion_control_tick(&control, NULL, now, NULL, true);
    assert(control.status.state == MOTION_STOPPED && fake.latched);
    assert(fake.duty[0] == 0 && fake.duty[1] == 0);
    assert(speed_probe_update(&probe, now + 1U, false, false) == SPEED_PROBE_FINISHED);
    assert(speed_probe_update(&probe, now + 100000U, true, false) == SPEED_PROBE_FINISHED);
    return 0;
}
