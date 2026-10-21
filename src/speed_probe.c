/** @file speed_probe.c
 * @brief Bounded initial PI experiment, without automatic gain/duty escalation.
 */
#include <stddef.h>
#include "speed_probe.h"

motion_control_config_t speed_probe_configuration (void)
{
    motion_control_config_t config = {0};
    unsigned int wheel;

    config.sampling.wheel[0] = (motion_scale_t){210487U, 2541U, -1, false};
    config.sampling.wheel[1] = (motion_scale_t){210487U, 2530U, 1, false};
    config.sampling.max_interval_us = SPEED_PROBE_MAX_GAP_US;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        /* Kp=64/65536; Ki=256/65536. At zero feedback P initially
         * requests about49 permille. I can ramp only up to the duty cap.
         * These are experimental startup settings, not measured gains. */
        config.speed[wheel] = (wheel_pid_config_t){64U, 256U, 0U,
            (uint32_t)SPEED_PROBE_TARGET_UM_S, SPEED_PROBE_MAX_GAP_US,
            60000U, SPEED_PROBE_DUTY_CAP, true};
    }
    config.max_command_ttl_us = SPEED_PROBE_TTL_US;
    config.max_control_gap_us = SPEED_PROBE_MAX_GAP_US;
    config.feedback_timeout_us = UINT32_C(2000000);
    config.reversal_brake_us = UINT32_C(100000);
    return config;
}

void speed_probe_init (speed_probe_t * p_probe)
{
    if (p_probe != NULL)
    {
        *p_probe = (speed_probe_t){0};
    }
}

speed_probe_phase_t speed_probe_update (speed_probe_t * p_probe,
    uint32_t now_us, bool start_pressed, bool stop_or_fault)
{
    if (p_probe == NULL)
    {
        return SPEED_PROBE_FINISHED;
    }
    if (stop_or_fault)
    {
        p_probe->interrupted = true;
        p_probe->phase = SPEED_PROBE_FINISHED;
    }
    if (p_probe->phase == SPEED_PROBE_RUN)
    {
        uint32_t gap = now_us - p_probe->previous_us;

        if (gap > p_probe->max_gap_us)
        {
            p_probe->max_gap_us = gap;
        }
        if (gap > SPEED_PROBE_MAX_GAP_US)
        {
            p_probe->gap_fault = true;
            p_probe->phase = SPEED_PROBE_FINISHED;
        }
        else if ((uint32_t)(now_us - p_probe->started_us) >= SPEED_PROBE_DURATION_US)
        {
            p_probe->phase = SPEED_PROBE_FINISHED;
        }
    }
    else if (p_probe->phase == SPEED_PROBE_WAIT)
    {
        if (motor_pulse_update(&p_probe->gate, now_us, start_pressed, false) !=
            MOTOR_PULSE_OFF)
        {
            p_probe->phase = SPEED_PROBE_RUN;
            p_probe->started_us = now_us;
        }
    }
    p_probe->previous_us = now_us;
    return p_probe->phase;
}
