/** @file controller.c
 * @brief Command expiry, feedback faults, PID ownership and braked reversal.
 */
#include <limits.h>
#include <stddef.h>
#include "controller.h"
#include "tm/tmonitor.h"

/* Consecutive invalid speed samples tolerated per wheel before FEEDBACK.
 * One sample is 10 ms, so a single encoder glitch (e.g. a motor switching
 * spike) is ridden through, while a dead encoder still faults in ~30 ms. */
#define MAX_BAD_SAMPLES 3U

static int8_t sign (int32_t value)
{
    return value > 0 ? 1 : (value < 0 ? -1 : 0);
}

static void reset_pid (controller_t * p_control)
{
    unsigned int wheel = 0U;

    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        pid_reset(&p_control->pid[wheel]);
    }
}

static void brake (controller_t * p_control, uint32_t now_us, bool latch)
{
    if (!p_control->braked)
    {
        p_control->brake_started_us = now_us;
    }
    p_control->braked         = true;
    p_control->status.duty[0] = 0;
    p_control->status.duty[1] = 0;
    if (!p_control->output.brake(p_control->output.context, latch))
    {
        p_control->status.brake_failed = true;
        p_control->status.state        = MOTION_FAULT;
        p_control->status.fault        = MOTION_FAULT_OUTPUT;
        if (!latch)
        {
            (void)p_control->output.brake(p_control->output.context, true);
        }
    }
}

static void fault (controller_t * p_control, uint32_t now_us,
                   motion_fault_t reason)
{
    p_control->status.state = MOTION_FAULT;
    p_control->status.fault = reason;
    reset_pid(p_control);
    brake(p_control, now_us, true);
#ifndef NDEBUG
    tm_printf((UB *)"MOTION FAULT, reason: %d\n", p_control->status.fault);
#endif
}

bool controller_config_valid (const controller_config_t * p_config)
{
    odometry_t sampler;
    motion_input_t   baseline = {0};
    pid_t      pid;
    unsigned int     wheel = 0U;

    if ((p_config == NULL) || (p_config->max_command_ttl_us == 0U) ||
        (p_config->max_command_ttl_us > (uint32_t)INT32_MAX) ||
        (p_config->max_control_gap_us == 0U) ||
        (p_config->max_control_gap_us > p_config->sampling.max_interval_us) ||
        (p_config->feedback_timeout_us <= p_config->max_control_gap_us) ||
        (p_config->feedback_timeout_us > (uint32_t)INT32_MAX) ||
        (p_config->reversal_brake_us == 0U) ||
        (p_config->reversal_brake_us > (uint32_t)INT32_MAX) ||
        !odometry_init(&sampler, &p_config->sampling, &baseline))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (!pid_init(&pid, &p_config->speed[wheel]) ||
            ((p_config->speed[wheel].kp_q16 == 0U) &&
             (p_config->speed[wheel].ki_q16 == 0U)) ||
            (p_config->max_control_gap_us >
             p_config->speed[wheel].max_interval_us))
        {
            return false;
        }
    }
    return true;
}

bool controller_init (controller_t *              p_control,
                          const controller_config_t * p_config,
                          const motion_input_t *          p_baseline,
                          const motion_output_t *         p_output)
{
    controller_t next = {0};
    unsigned int     wheel = 0U;

    if ((p_control == NULL) || (p_baseline == NULL) || (p_output == NULL) ||
        (p_output->apply == NULL) || (p_output->brake == NULL) ||
        !controller_config_valid(p_config) ||
        !odometry_init(&next.sampler, &p_config->sampling, p_baseline))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        (void)pid_init(&next.pid[wheel], &p_config->speed[wheel]);
        next.progress_us[wheel] = p_baseline->timestamp_us;
    }
    if (!p_output->brake(p_output->context, false))
    {
        return false;
    }
    next.config              = *p_config;
    next.output              = *p_output;
    next.previous_tick_us    = p_baseline->timestamp_us;
    next.status.timestamp_us = p_baseline->timestamp_us;
    next.brake_started_us    = p_baseline->timestamp_us;
    next.braked              = true;
    next.initialised         = true;
    *p_control               = next;
    return true;
}

static bool valid_command (const controller_t * p_control,
                           const motion_command_t * p_command, uint32_t now_us)
{
    unsigned int wheel = 0U;

    if ((p_command->ttl_us == 0U) ||
        (p_command->ttl_us > p_control->config.max_command_ttl_us) ||
        ((uint32_t)(now_us - p_command->issued_us) >= p_command->ttl_us))
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t target = p_command->target_um_s[wheel];
        int64_t limit  = p_control->config.speed[wheel].max_target_um_s;

        if ((target > limit) || (target < -limit))
        {
            return false;
        }
    }
    return true;
}

/* Stopped and faulted states are terminal, and a stop request has priority
 * over everything else. Also faults on a control gap or an expired lease.
 * Returns false when the tick has already finished. */
static bool tick_admit (controller_t * p_control, uint32_t now_us,
                        bool stop_requested)
{
    bool active = false;

    p_control->status.timestamp_us     = now_us;
    p_control->status.sample_available = false;
    if ((p_control->status.state == MOTION_STOPPED) ||
        (p_control->status.state == MOTION_FAULT))
    {
        return false;
    }
    if (stop_requested)
    {
        p_control->status.state = MOTION_STOPPED;
        reset_pid(p_control);
        brake(p_control, now_us, true);
        return false;
    }
    active = p_control->status.state != MOTION_IDLE;
    if (active && ((uint32_t)(now_us - p_control->previous_tick_us) >
                   p_control->config.max_control_gap_us))
    {
        fault(p_control, now_us, MOTION_FAULT_GAP);
        return false;
    }
    p_control->previous_tick_us = now_us;
    if (active && ((uint32_t)(now_us - p_control->command.issued_us) >=
                   p_control->command.ttl_us))
    {
        /* A fresh arrival cannot conceal an already expired active lease. */
        fault(p_control, now_us, MOTION_FAULT_EXPIRED);
        return false;
    }
    return true;
}

/* Validate and latch a new command. Returns false after a COMMAND fault. */
static bool accept_command (controller_t * p_control,
                            const motion_command_t * p_command,
                            uint32_t now_us)
{
    unsigned int wheel = 0U;

    if (!valid_command(p_control, p_command, now_us))
    {
        fault(p_control, now_us, MOTION_FAULT_COMMAND);
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        /* Keep integral/history across same-direction steering/speed
         * adjustments; reset only the wheel whose target direction
         * changes or becomes/leaves zero. */
        if (sign(p_command->target_um_s[wheel]) !=
            sign(p_control->command.target_um_s[wheel]))
        {
            pid_reset(&p_control->pid[wheel]);
        }
    }
    p_control->command           = *p_command;
    p_control->status.command_id = p_command->id;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        p_control->status.target_um_s[wheel] = p_command->target_um_s[wheel];
    }
    return true;
}

/* Both targets are zero: brake without latching and resynchronise progress. */
static void tick_idle (controller_t * p_control, uint32_t now_us)
{
    unsigned int wheel = 0U;

    p_control->status.state = MOTION_IDLE;
    reset_pid(p_control);
    brake(p_control, now_us, false);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        p_control->progress_us[wheel] = now_us;
        p_control->bad_samples[wheel] = 0U;
        if (p_control->status.sample_available)
        {
            p_control->last_edges[wheel] =
                p_control->status.sample.wheel[wheel].net_edges;
        }
    }
}

/* Run the per-wheel progress watchdog and speed PID. Fills p_requested and
 * sets *p_reversal when a wheel asks to reverse its drive direction.
 * Returns false after a fault. */
static bool speed_loop (controller_t * p_control, uint32_t now_us,
                        int16_t * p_requested, bool * p_reversal)
{
    unsigned int wheel = 0U;

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
            return false;
        }
        p_control->last_edges[wheel] = edges;
        if (!p_control->status.sample.wheel[wheel].speed_valid &&
            (p_control->bad_samples[wheel] < MAX_BAD_SAMPLES))
        {
            /* Isolated encoder glitch: keep the duty already applied and
             * skip this controller update instead of latching FEEDBACK. */
            p_control->bad_samples[wheel]++;
            p_requested[wheel] = p_control->status.duty[wheel];
            continue;
        }
        p_control->bad_samples[wheel] = 0U;
        if (!pid_update(
                &p_control->pid[wheel], p_control->command.target_um_s[wheel],
                &p_control->status.sample, wheel, now_us, &p_requested[wheel]))
        {
            fault(p_control, now_us, MOTION_FAULT_FEEDBACK);
            return false;
        }
        /* Decelerate by brake rather than applying reverse drive when the
         * signed speed controller asks to oppose the current target. */
        if (sign(p_requested[wheel]) !=
            sign(p_control->command.target_um_s[wheel]))
        {
            p_requested[wheel] = 0;
            pid_reset(&p_control->pid[wheel]);
        }
        if ((p_requested[wheel] != 0) && (p_control->drive_sign[wheel] != 0) &&
            (sign(p_requested[wheel]) != p_control->drive_sign[wheel]))
        {
            *p_reversal = true;
        }
    }
    return true;
}

/* Hold the brake for the reversal interval. Returns true when the tick has
 * been consumed by braking. */
static bool reversal_hold (controller_t * p_control, uint32_t now_us,
                           bool reversal)
{
    if ((p_control->status.state == MOTION_REVERSING || reversal) &&
        (!p_control->braked ||
         ((uint32_t)(now_us - p_control->brake_started_us) <
          p_control->config.reversal_brake_us)))
    {
        p_control->status.state = MOTION_REVERSING;
        reset_pid(p_control);
        brake(p_control, now_us, false);
        return true;
    }
    return false;
}

/* Publish the requested duty cycles and act on the driver's answer. */
static void apply_requested (controller_t * p_control, uint32_t now_us,
                             const int16_t * p_requested)
{
    unsigned int          wheel   = 0U;
    motion_apply_result_t applied = MOTION_OUTPUT_FAILED;

    applied = p_control->output.apply(p_control->output.context,
                                      p_requested[0], p_requested[1]);
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
        p_control->braked       = false;
        for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
        {
            p_control->status.duty[wheel] = p_requested[wheel];
            if (p_requested[wheel] != 0)
            {
                p_control->drive_sign[wheel] = sign(p_requested[wheel]);
            }
        }
    }
    else
    {
        fault(p_control, now_us, MOTION_FAULT_OUTPUT);
    }
}

void controller_tick (controller_t *     p_control,
                          const motion_input_t * p_input, uint32_t now_us,
                          const motion_command_t * p_command,
                          bool                     stop_requested)
{
    bool    active                    = false;
    bool    reversal                  = false;
    int16_t requested[ENCODER_WHEELS] = {0, 0};

    if ((p_control == NULL) || !p_control->initialised)
    {
        return;
    }
    if (!tick_admit(p_control, now_us, stop_requested))
    {
        return;
    }
    if ((p_command != NULL) && !accept_command(p_control, p_command, now_us))
    {
        return;
    }
    active = (p_control->command.target_um_s[0] != 0) ||
             (p_control->command.target_um_s[1] != 0);
    if (active && ((uint32_t)(now_us - p_control->command.issued_us) >=
                   p_control->command.ttl_us))
    {
        fault(p_control, now_us, MOTION_FAULT_EXPIRED);
        return;
    }

    p_control->status.sample_available = odometry_update(
        &p_control->sampler, p_input, &p_control->status.sample);
    if (!active)
    {
        tick_idle(p_control, now_us);
        return;
    }
    if (!p_control->status.sample_available)
    {
        fault(p_control, now_us, MOTION_FAULT_FEEDBACK);
        return;
    }

    if (!speed_loop(p_control, now_us, requested, &reversal))
    {
        return;
    }
    if (reversal_hold(p_control, now_us, reversal))
    {
        return;
    }
    if ((requested[0] == 0) && (requested[1] == 0))
    {
        p_control->status.state = MOTION_RUNNING;
        brake(p_control, now_us, false);
        return;
    }
    apply_requested(p_control, now_us, requested);
}
