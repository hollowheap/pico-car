/** @file wheel_pid.h
 * @brief Task-owned integer PID with timestamp/quality checks.
 */
#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include "motion_sampler.h"

typedef struct
{
    /* Q16 gains: real coefficient = stored value / 65536.
     * P: permille / (um/s); I: permille / um;
     * D: permille / (um/s^2). Nonnegative; zero disables a term. */
    uint32_t kp_q16;
    uint32_t ki_q16;
    uint32_t kd_q16;
    uint32_t max_target_um_s; /* Application-selected, not a motor rating. */
    uint32_t max_interval_us;
    uint32_t max_sample_age_us;
    uint16_t max_duty_permille; /* 1..1000. */
} wheel_pid_config_t;

typedef struct
{
    wheel_pid_config_t config;
    int64_t            integral_q16;
    int32_t            previous_measurement;
    uint32_t           previous_us;
    bool               has_previous;
    bool               initialised;
} wheel_pid_t;

bool wheel_pid_init (wheel_pid_t * p_pid, const wheel_pid_config_t * p_config);
void wheel_pid_reset (wheel_pid_t * p_pid);

/* Supply a new coherent sample for the selected wheel each call. Rejects
 * invalid/stale/unverified feedback, duplicate/excessively spaced samples,
 * targets outside configuration, and unrepresentable arithmetic. On failure
 * state is unchanged and *p_duty is set to zero when nonnull. The motor owner
 * must apply brake/latch its fault; do not continue a previous PWM command.
 * Uses derivative on measurement and conditional integration anti-windup.
 * Output is signed permille. It can request reversal; the owner must enforce
 * the motor driver's braked reversal interval. A zero target resets integral
 * and returns zero. It commands brake rather than holding position.
 * Not an RTOS task, mailbox or tuned controller. Calls must be serialised by
 * one owner. No default gains or physical speed limits are supplied. */
bool wheel_pid_update (wheel_pid_t * p_pid, int32_t target_um_s,
                       const motion_sample_t * p_sample, unsigned int wheel,
                       uint32_t now_us, int16_t * p_duty);

#endif /* WHEEL_PID_H */
