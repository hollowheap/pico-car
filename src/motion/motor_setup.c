/** @file motor_setup.c
 * @brief The motor service configuration of the car. See motor_setup.h.
 */

#include "motion/motor_setup.h"

#include "motion_service.h"

#define MOTOR_WHEEL_COUNT            (2U)
#define MOTOR_WHEEL_TRAVEL_UM        (204204U)
#define MOTOR_EDGES_PER_REVOLUTION   (2530U)
#define MOTOR_SAMPLING_INTERVAL_US   (100000U)
#define MOTOR_PID_KP_Q16             (256U)
#define MOTOR_PID_KI_Q16             (256U)
#define MOTOR_PID_KD_Q16             (2U)
#define MOTOR_PID_INTERVAL_US        (100000U)
#define MOTOR_PID_SAMPLE_AGE_US      (600000U)
#define MOTOR_PID_DUTY_PERMILLE      (400U)
#define MOTOR_COMMAND_TTL_US         (200000U)
#define MOTOR_CONTROL_GAP_US         (100000U)
#define MOTOR_FEEDBACK_TIMEOUT_US    (2000000U)
#define MOTOR_REVERSAL_BRAKE_US      (100000U)
#define MOTOR_POLL_DELAY_MS          (10U)

ER motion_setup (ID motion_tid)
{
    motion_service_config_t config   = {0};

    /* The third field is the direction sign of the encoder. The left wheel is
     * mounted mirrored. */
    config.control.sampling.wheel[0] =
        (motion_scale_t){MOTOR_WHEEL_TRAVEL_UM, MOTOR_EDGES_PER_REVOLUTION,
                         -1};
    config.control.sampling.wheel[1] =
        (motion_scale_t){MOTOR_WHEEL_TRAVEL_UM, MOTOR_EDGES_PER_REVOLUTION,
                         1};
    config.control.sampling.max_interval_us = MOTOR_SAMPLING_INTERVAL_US;
    for (uint8_t wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        config.control.speed[wheel] = (pid_config_t){
            .kp_q16            = MOTOR_PID_KP_Q16,
            .ki_q16            = MOTOR_PID_KI_Q16,
            .kd_q16            = MOTOR_PID_KD_Q16,
            .max_target_um_s   = MOTOR_MAX_TARGET_UM_S,
            .max_interval_us   = MOTOR_PID_INTERVAL_US,
            .max_sample_age_us = MOTOR_PID_SAMPLE_AGE_US,
            .max_duty_permille = MOTOR_PID_DUTY_PERMILLE,
        };
    }
    config.control.max_command_ttl_us  = MOTOR_COMMAND_TTL_US;
    config.control.max_control_gap_us  = MOTOR_CONTROL_GAP_US;
    config.control.feedback_timeout_us = MOTOR_FEEDBACK_TIMEOUT_US;
    config.control.reversal_brake_us   = MOTOR_REVERSAL_BRAKE_US;
    config.poll_delay_ms               = MOTOR_POLL_DELAY_MS;

    return motion_service_init(&config, motion_tid);
}

/*** end of file ***/
