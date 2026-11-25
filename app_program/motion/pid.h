/** @file pid.h
 * @brief Task-owned integer PID with timestamp/quality checks.
 */
#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include "odometry.h"

#define PID_DUTY_PERMILLE_FULL (1000U)

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
    uint16_t base_duty_permille; /* Zero uses the fixed maximum. */
    uint16_t ks_permille; /* Static friction, signed with a nonzero target. */
    uint32_t kv_q16; /* Feed-forward: permille / (um/s), same scale as kp_q16. */
} pid_config_t;

typedef struct
{
    pid_config_t config;
    int64_t            integral_q16;
    int32_t            previous_measurement;
    uint32_t           previous_us;
    bool               has_previous;
    bool               initialised;
} pid_t;

bool pid_init (pid_t * p_pid, const pid_config_t * p_config);
void pid_reset (pid_t * p_pid);

/* Supply a new coherent sample for the selected wheel each call. Rejects
 * invalid/stale/unverified feedback, duplicate/excessively spaced samples,
 * targets outside configuration, and unrepresentable arithmetic. On failure
 * state is unchanged and *p_duty is set to zero when nonnull. The motor owner
 * must apply brake/latch its fault; do not continue a previous PWM command.
 * Uses derivative on measurement and conditional integration anti-windup.
 * The feed-forward kv_q16 * target and the signed ks_permille are summed with
 * P, I and D under the same duty limit.
 * With a base duty limit, grants extra duty in proportion to the speed
 * shortfall while feedback shows travel in the target direction.
 * Output is signed permille. It can request reversal; the owner must enforce
 * the motor driver's braked reversal interval. A zero target resets integral
 * and returns zero. It commands brake rather than holding position.
 * Not an RTOS task, mailbox or tuned controller. Calls must be serialised by
 * one owner. No default gains or physical speed limits are supplied. */
bool pid_update (pid_t * p_pid, int32_t target_um_s,
                       const motion_sample_t * p_sample, unsigned int wheel,
                       uint32_t now_us, int16_t * p_duty);

#endif /* WHEEL_PID_H */
