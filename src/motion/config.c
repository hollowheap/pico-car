/** @file config.c
 * @brief Application motor defaults owned by the motion subsystem.
 */
#include <stddef.h>
#include "motion_config.h"

void motion_config_default (motion_config_t * p_config)
{
    unsigned int wheel;

    if (p_config == NULL)
    {
        return;
    }

    *p_config = (motion_config_t){0};
    p_config->control.sampling.wheel[0] =
        (odometry_scale_t){204204U, 2530U, -1};
    p_config->control.sampling.wheel[1] =
        (odometry_scale_t){204204U, 2530U, 1};
    p_config->control.sampling.max_interval_us = 100000U;
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        p_config->control.speed[wheel] = (pid_config_t){
            .kp_q16            = 256U,
            .ki_q16            = 256U,
            .kd_q16            = 16U,
            .max_target_um_s   = 100000U,
            .max_interval_us   = 100000U,
            .max_sample_age_us = 600000U,
            .max_duty_permille = 250U,
        };
    }
    p_config->control.max_command_ttl_us  = 200000U;
    p_config->control.max_control_gap_us  = 100000U;
    p_config->control.feedback_timeout_us = 2000000U;
    p_config->control.reversal_brake_us   = 100000U;
    p_config->poll_delay_ms               = 10U;
}

/*** end of file ***/
