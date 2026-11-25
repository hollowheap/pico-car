/** @file setup.c
 * @brief Apply the original obstacle-extras wheel tuning to the motion service.
 */

#include "setup.h"

#include "motion/motion_service.h"
#include "platform/chassis.h"

#define EXTRAS_WHEEL_INTERVAL_US  (100000U)
#define EXTRAS_PID_KP_Q16          (256U)
#define EXTRAS_PID_KI_Q16          (256U)
#define EXTRAS_PID_KD_Q16          (2U)
#define EXTRAS_MAX_TARGET_UM_S     (200000U)
#define EXTRAS_PID_SAMPLE_AGE_US   (600000U)
#define EXTRAS_MAX_DUTY_PERMILLE   (250U)
#define EXTRAS_COMMAND_TTL_US      (200000U)
#define EXTRAS_CONTROL_GAP_US      (100000U)
#define EXTRAS_REVERSAL_BRAKE_US   (100000U)
#define EXTRAS_MOTION_PERIOD_MS    (10U)

ER extras_motion_setup (ID task_id)
{
    pid_config_t const wheel_pid = {
        .kp_q16            = EXTRAS_PID_KP_Q16,
        .ki_q16            = EXTRAS_PID_KI_Q16,
        .kd_q16            = EXTRAS_PID_KD_Q16,
        .max_target_um_s   = EXTRAS_MAX_TARGET_UM_S,
        .max_interval_us   = EXTRAS_WHEEL_INTERVAL_US,
        .max_sample_age_us = EXTRAS_PID_SAMPLE_AGE_US,
        .max_duty_permille = EXTRAS_MAX_DUTY_PERMILLE,
    };

    motion_service_config_t const config = {
        .control = {
            .sampling = {
                .wheel = {
                    {CHASSIS_WHEEL_TRAVEL_UM, CHASSIS_EDGES_PER_REV, -1},
                    {CHASSIS_WHEEL_TRAVEL_UM, CHASSIS_EDGES_PER_REV, 1},
                },
                .max_interval_us = EXTRAS_WHEEL_INTERVAL_US,
            },
            .speed                 = {wheel_pid, wheel_pid},
            .max_command_ttl_us    = EXTRAS_COMMAND_TTL_US,
            .max_control_gap_us    = EXTRAS_CONTROL_GAP_US,
            .reversal_brake_us     = EXTRAS_REVERSAL_BRAKE_US,
            .pivot_duty_permille   = EXTRAS_MAX_DUTY_PERMILLE,
        },
        .poll_delay_ms = EXTRAS_MOTION_PERIOD_MS,
    };

    return motion_service_init(&config, task_id);
}
