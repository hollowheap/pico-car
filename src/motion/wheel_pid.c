/** @file wheel_pid.c
 * @brief Checked fixed-point PID; no floating point or hardware access.
 */
#include <limits.h>
#include <stddef.h>
#include "wheel_pid.h"

#define PID_Q16 INT64_C(65536)

static bool multiply (int64_t value, uint32_t factor, int64_t * p_result)
{
    if ((factor != 0U) && ((value > INT64_MAX / (int64_t)factor) ||
                           (value < INT64_MIN / (int64_t)factor)))
    {
        return false;
    }
    *p_result = value * (int64_t)factor;
    return true;
}

static bool add (int64_t left, int64_t right, int64_t * p_result)
{
    if (((right > 0) && (left > INT64_MAX - right)) ||
        ((right < 0) && (left < INT64_MIN - right)))
    {
        return false;
    }
    *p_result = left + right;
    return true;
}

static int64_t clamp (int64_t value, int64_t limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

bool wheel_pid_init (wheel_pid_t * p_pid, const wheel_pid_config_t * p_config)
{
    if ((p_pid == NULL) || (p_config == NULL) ||
        (p_config->max_target_um_s == 0U) ||
        (p_config->max_target_um_s > (uint32_t)INT32_MAX) ||
        (p_config->max_interval_us == 0U) ||
        (p_config->max_interval_us > (uint32_t)INT32_MAX) ||
        (p_config->max_sample_age_us == 0U) ||
        (p_config->max_sample_age_us > (uint32_t)INT32_MAX) ||
        (p_config->max_duty_permille == 0U) ||
        (p_config->max_duty_permille > 1000U))
    {
        return false;
    }
    *p_pid             = (wheel_pid_t){0};
    p_pid->config      = *p_config;
    p_pid->initialised = true;
    return true;
}

void wheel_pid_reset (wheel_pid_t * p_pid)
{
    if (p_pid != NULL)
    {
        p_pid->integral_q16         = 0;
        p_pid->previous_measurement = 0;
        p_pid->previous_us          = 0U;
        p_pid->has_previous         = false;
    }
}

bool wheel_pid_update (wheel_pid_t * p_pid, int32_t target_um_s,
                       const motion_sample_t * p_sample, unsigned int wheel,
                       uint32_t now_us, int16_t * p_duty)
{
    wheel_pid_t                   next;
    const motion_wheel_sample_t * p_feedback;
    uint32_t                      interval;
    int64_t                       error;
    int64_t                       proportional;
    int64_t                       integral_delta;
    int64_t                       integral;
    int64_t                       derivative = 0;
    int64_t                       limit;
    int64_t                       sum;

    if (p_duty != NULL)
    {
        *p_duty = 0;
    }
    if ((p_pid == NULL) || (p_duty == NULL) || !p_pid->initialised ||
        (p_sample == NULL) || (wheel >= ENCODER_WHEELS) ||
        !motion_sample_fresh(p_sample, now_us, p_pid->config.max_sample_age_us))
    {
        return false;
    }
    next       = *p_pid;
    p_feedback = &p_sample->wheel[wheel];
    interval   = next.has_previous ? p_sample->timestamp_us - next.previous_us
                                   : p_sample->elapsed_us;
    if (!p_feedback->speed_valid ||
        (interval == 0U) || (interval > next.config.max_interval_us) ||
        (p_sample->elapsed_us == 0U) ||
        (p_sample->elapsed_us > next.config.max_interval_us) ||
        ((int64_t)target_um_s > (int64_t)next.config.max_target_um_s) ||
        ((int64_t)target_um_s < -(int64_t)next.config.max_target_um_s))
    {
        return false;
    }
    error = (int64_t)target_um_s - (int64_t)p_feedback->velocity_um_s;
    if (!multiply(error, next.config.kp_q16, &proportional) ||
        !multiply(error, next.config.ki_q16, &integral_delta) ||
        !multiply(integral_delta, interval, &integral_delta))
    {
        return false;
    }
    integral_delta /= INT64_C(1000000);
    limit = (int64_t)next.config.max_duty_permille * PID_Q16;
    if (!add(next.integral_q16, integral_delta, &integral))
    {
        return false;
    }
    integral = clamp(integral, limit);
    if (next.has_previous)
    {
        int64_t change = (int64_t)p_feedback->velocity_um_s -
                         (int64_t)next.previous_measurement;

        if (!multiply(change, UINT32_C(1000000), &derivative))
        {
            return false;
        }
        derivative /= (int64_t)interval;
        if (!multiply(derivative, next.config.kd_q16, &derivative))
        {
            return false;
        }
    }
    /* Derivative on measurement: P + I - D. Refuse INT64_MIN before negate. */
    if ((derivative == INT64_MIN) || !add(proportional, -derivative, &sum) ||
        !add(sum, integral, &sum))
    {
        return false;
    }
    if (((sum > limit) && (error > 0)) || ((sum < -limit) && (error < 0)))
    {
        /* Reject integration pushing further into saturation. */
        integral = next.integral_q16;
        if (!add(proportional, -derivative, &sum) || !add(sum, integral, &sum))
        {
            return false;
        }
    }
    next.integral_q16         = integral;
    next.previous_measurement = p_feedback->velocity_um_s;
    next.previous_us          = p_sample->timestamp_us;
    next.has_previous         = true;
    if (target_um_s == 0)
    {
        next.integral_q16 = 0;
        next.has_previous = false;
        sum               = 0;
    }
    *p_duty = (int16_t)(clamp(sum, limit) / PID_Q16);
    *p_pid  = next;
    return true;
}
