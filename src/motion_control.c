/** @file motion_control.c
 * @brief Command expiry, feedback faults, PID ownership and braked reversal.
 */
#include <limits.h>
#include <stddef.h>
#include "motion_control.h"

static int8_t sign (int32_t value)
{
    return value > 0 ? 1 : (value < 0 ? -1 : 0);
}

static void reset_pid (motion_control_t * p_control)
{
    unsigned int wheel;

    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        wheel_pid_reset(&p_control->pid[wheel]);
    }
}

static void brake (motion_control_t * p_control, uint32_t now_us, bool latch)
{
    if (!p_control->braked)
    {
        p_control->brake_started_us = now_us;
    }
    p_control->braked = true;
    p_control->status.duty[0] = 0;
    p_control->status.duty[1] = 0;
    if (!p_control->output.brake(p_control->output.context, latch))
    {
        p_control->status.brake_failed = true;
        p_control->status.state = MOTION_FAULT;
        p_control->status.fault = MOTION_FAULT_OUTPUT;
        if (!latch)
        {
            (void)p_control->output.brake(p_control->output.context, true);
        }
    }
}

static void fault (motion_control_t * p_control, uint32_t now_us,
                   motion_fault_t reason)
{
    p_control->status.state = MOTION_FAULT;
    p_control->status.fault = reason;
    reset_pid(p_control);
    brake(p_control, now_us, true);
}

bool motion_control_config_valid (const motion_control_config_t * p_config)
{
    motion_sampler_t sampler;
    motion_input_t baseline = {0};
    wheel_pid_t pid;
    unsigned int wheel;

    if ((p_config == NULL) ||
        (p_config->max_command_ttl_us == 0U) ||
        (p_config->max_command_ttl_us > (uint32_t)INT32_MAX) ||
        (p_config->max_control_gap_us == 0U) ||
        (p_config->max_control_gap_us > p_config->sampling.max_interval_us) ||
        (p_config->feedback_timeout_us <= p_config->max_control_gap_us) ||
        (p_config->feedback_timeout_us > (uint32_t)INT32_MAX) ||
        (p_config->reversal_brake_us == 0U) ||
        (p_config->reversal_brake_us > (uint32_t)INT32_MAX) ||
        !motion_sampler_init(&sampler, &p_config->sampling, &baseline))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (!wheel_pid_init(&pid, &p_config->speed[wheel]) ||
            ((p_config->speed[wheel].kp_q16 == 0U) &&
             (p_config->speed[wheel].ki_q16 == 0U)) ||
            (p_config->max_control_gap_us > p_config->speed[wheel].max_interval_us))
        {
            return false;
        }
    }
    return true;
}

bool motion_control_init (motion_control_t * p_control,
    const motion_control_config_t * p_config, const motion_input_t * p_baseline,
    const motion_output_t * p_output)
{
    motion_control_t next = {0};
    unsigned int wheel;

    if ((p_control == NULL) || (p_baseline == NULL) || (p_output == NULL) ||
        (p_output->apply == NULL) || (p_output->brake == NULL) ||
        !motion_control_config_valid(p_config) ||
        !motion_sampler_init(&next.sampler, &p_config->sampling, p_baseline))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        (void)wheel_pid_init(&next.pid[wheel], &p_config->speed[wheel]);
        next.progress_us[wheel] = p_baseline->timestamp_us;
    }
    if (!p_output->brake(p_output->context, false))
    {
        return false;
    }
    next.config = *p_config;
    next.output = *p_output;
    next.previous_tick_us = p_baseline->timestamp_us;
    next.status.timestamp_us = p_baseline->timestamp_us;
    next.brake_started_us = p_baseline->timestamp_us;
    next.braked = true;
    next.initialised = true;
    *p_control = next;
    return true;
}

static bool valid_command (const motion_control_t * p_control,
                           const motion_command_t * p_command, uint32_t now_us)
{
    unsigned int wheel;

    if ((p_command->ttl_us == 0U) ||
        (p_command->ttl_us > p_control->config.max_command_ttl_us) ||
        ((uint32_t)(now_us - p_command->issued_us) >= p_command->ttl_us))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t target = p_command->target_um_s[wheel];
        int64_t limit = p_control->config.speed[wheel].max_target_um_s;

        if ((target > limit) || (target < -limit))
        {
            return false;
        }
    }
    return true;
}

void motion_control_tick (motion_control_t * p_control,
    const motion_input_t * p_input, uint32_t now_us,
    const motion_command_t * p_command, bool stop_requested)
{
    bool active;
    bool reversal = false;
    unsigned int wheel;
    int16_t requested[ENCODER_WHEELS] = {0, 0};
    motion_apply_result_t applied;

    if ((p_control == NULL) || !p_control->initialised)
    {
        return;
    }
    p_control->status.timestamp_us = now_us;
    p_control->status.sample_available = false;
    if ((p_control->status.state == MOTION_STOPPED) ||
        (p_control->status.state == MOTION_FAULT))
    {
        return;
    }
    if (stop_requested)
    {
        p_control->status.state = MOTION_STOPPED;
        reset_pid(p_control);
        brake(p_control, now_us, true);
        return;
    }
    active = p_control->status.state != MOTION_IDLE;
    if (active && ((uint32_t)(now_us - p_control->previous_tick_us) >
                   p_control->config.max_control_gap_us))
    {
        fault(p_control, now_us, MOTION_FAULT_GAP);
        return;
    }
    p_control->previous_tick_us = now_us;
    if (active && ((uint32_t)(now_us - p_control->command.issued_us) >=
                   p_control->command.ttl_us))
    {
        /* A fresh arrival cannot conceal an already expired active lease. */
        fault(p_control, now_us, MOTION_FAULT_EXPIRED);
        return;
    }
    if (p_command != NULL)
    {
        if (!valid_command(p_control, p_command, now_us))
        {
            fault(p_control, now_us, MOTION_FAULT_COMMAND);
            return;
        }
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            /* Keep integral/history across same-direction steering/speed
             * adjustments; reset only the wheel whose target direction
             * changes or becomes/leaves zero. */
            if (sign(p_command->target_um_s[wheel]) !=
                sign(p_control->command.target_um_s[wheel]))
            {
                wheel_pid_reset(&p_control->pid[wheel]);
            }
        }
        p_control->command = *p_command;
        p_control->status.command_id = p_command->id;
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_control->status.target_um_s[wheel] = p_command->target_um_s[wheel];
        }
    }
    active = (p_control->command.target_um_s[0] != 0) ||
             (p_control->command.target_um_s[1] != 0);
    if (active && ((uint32_t)(now_us - p_control->command.issued_us) >=
                   p_control->command.ttl_us))
    {
        fault(p_control, now_us, MOTION_FAULT_EXPIRED);
        return;
    }
    p_control->status.sample_available = motion_sampler_update(
        &p_control->sampler, p_input, &p_control->status.sample);
    if (!active)
    {
        p_control->status.state = MOTION_IDLE;
        reset_pid(p_control);
        brake(p_control, now_us, false);
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_control->progress_us[wheel] = now_us;
            if (p_control->status.sample_available)
            {
                p_control->last_edges[wheel] = p_control->status.sample.wheel[wheel].net_edges;
            }
        }
        return;
    }
    if (!p_control->status.sample_available)
    {
        fault(p_control, now_us, MOTION_FAULT_FEEDBACK);
        return;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t edges = p_control->status.sample.wheel[wheel].net_edges;

        if ((p_control->status.duty[wheel] == 0) ||
            (edges != p_control->last_edges[wheel]))
        {
            p_control->progress_us[wheel] = now_us;
        }
        else if ((uint32_t)(now_us - p_control->progress_us[wheel]) >=
                 p_control->config.feedback_timeout_us)
        {
            fault(p_control, now_us, MOTION_FAULT_NO_EDGES);
            return;
        }
        p_control->last_edges[wheel] = edges;
        if (!wheel_pid_update(&p_control->pid[wheel],
            p_control->command.target_um_s[wheel], &p_control->status.sample,
            wheel, now_us, &requested[wheel]))
        {
            fault(p_control, now_us, MOTION_FAULT_FEEDBACK);
            return;
        }
        /* Decelerate by brake rather than applying reverse drive when the
         * signed speed controller asks to oppose the current target. */
        if (sign(requested[wheel]) != sign(p_control->command.target_um_s[wheel]))
        {
            requested[wheel] = 0;
            wheel_pid_reset(&p_control->pid[wheel]);
        }
        if ((requested[wheel] != 0) && (p_control->drive_sign[wheel] != 0) &&
            (sign(requested[wheel]) != p_control->drive_sign[wheel]))
        {
            reversal = true;
        }
    }
    if ((p_control->status.state == MOTION_REVERSING || reversal) &&
        (!p_control->braked || ((uint32_t)(now_us - p_control->brake_started_us) <
                               p_control->config.reversal_brake_us)))
    {
        p_control->status.state = MOTION_REVERSING;
        reset_pid(p_control);
        brake(p_control, now_us, false);
        return;
    }
    if ((requested[0] == 0) && (requested[1] == 0))
    {
        p_control->status.state = MOTION_RUNNING;
        brake(p_control, now_us, false);
        return;
    }
    applied = p_control->output.apply(p_control->output.context, requested[0], requested[1]);
    if (applied == MOTION_OUTPUT_FAILED)
    {
        fault(p_control, now_us, MOTION_FAULT_OUTPUT);
    }
    else if (applied == MOTION_BRAKED_RETRY)
    {
        p_control->status.state = MOTION_REVERSING;
        reset_pid(p_control);
        /* RETRY promises that braking began at this call, even if the last
         * published output was already zero. Restart the full hold interval. */
        p_control->braked = false;
        brake(p_control, now_us, false);
    }
    else if (applied == MOTION_APPLIED)
    {
        p_control->status.state = MOTION_RUNNING;
        p_control->braked = false;
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_control->status.duty[wheel] = requested[wheel];
            if (requested[wheel] != 0)
            {
                p_control->drive_sign[wheel] = sign(requested[wheel]);
            }
        }
    }
    else
    {
        fault(p_control, now_us, MOTION_FAULT_OUTPUT);
    }
}
