/** @file motor_pulse.c
 * @brief Release-to-arm, press-to-run diagnostic pulse policy.
 */
#include <stddef.h>
#include "motor_pulse.h"

void motor_pulse_init (motor_pulse_t * p_pulse)
{
    if (p_pulse != NULL)
    {
        *p_pulse = (motor_pulse_t){0};
    }
}

motor_pulse_phase_t motor_pulse_update (motor_pulse_t * p_pulse,
    uint32_t now_us, bool start_pressed, bool stop_pressed)
{
    if (p_pulse == NULL)
    {
        return MOTOR_PULSE_OFF;
    }
    if (stop_pressed)
    {
        p_pulse->stopped = true;
        p_pulse->active = MOTOR_PULSE_OFF;
        p_pulse->ready = false;
    }
    if (p_pulse->stopped)
    {
        return MOTOR_PULSE_OFF;
    }
    if (p_pulse->active != MOTOR_PULSE_OFF)
    {
        if ((uint32_t)(now_us - p_pulse->started_us) >= MOTOR_PULSE_US)
        {
            p_pulse->active = MOTOR_PULSE_OFF;
            p_pulse->release_tracking = false;
            p_pulse->press_tracking = false;
            p_pulse->ready = false;
        }
        return p_pulse->active;
    }
    if (p_pulse->next >= 4U)
    {
        return MOTOR_PULSE_OFF;
    }
    if (!start_pressed)
    {
        p_pulse->press_tracking = false;
        if (!p_pulse->release_tracking)
        {
            p_pulse->released_us = now_us;
            p_pulse->release_tracking = true;
        }
        if ((uint32_t)(now_us - p_pulse->released_us) >= MOTOR_RELEASE_US)
        {
            p_pulse->ready = true;
        }
    }
    else
    {
        p_pulse->release_tracking = false;
        if (p_pulse->ready)
        {
            if (!p_pulse->press_tracking)
            {
                p_pulse->pressed_us = now_us;
                p_pulse->press_tracking = true;
            }
            if ((uint32_t)(now_us - p_pulse->pressed_us) >= MOTOR_PRESS_US)
            {
                p_pulse->active = (motor_pulse_phase_t)(p_pulse->next + 1U);
                p_pulse->next++;
                p_pulse->started_us = now_us;
                p_pulse->ready = false;
                p_pulse->press_tracking = false;
            }
        }
    }
    return p_pulse->active;
}
