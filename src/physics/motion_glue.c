/** @file motion_glue.c
 *
 * @brief Read the motion service into motion_reading_t (target only).
 *
 * Pass motion_glue_read as physics_task_config_t.read_motion once
 * motion_service_init() has been called.
 */

#include <tk/tkernel.h>

#include "motion/motion.h"

#include <stdbool.h>
#include <stdint.h>

#include "motion_adapter.h"
#include "motion_glue.h"

bool motion_glue_read (motion_reading_t * p_reading)
{
    motion_feedback_t feedback;
    uint32_t          wheel;

    if ((NULL == p_reading) || (E_OK != motion_feedback_snapshot(&feedback)))
    {
        return false;
    }

    p_reading->available    = feedback.fresh;
    p_reading->timestamp_us = feedback.timestamp_us;
    p_reading->velocity_um_s[0] = feedback.left_speed_um_s;
    p_reading->velocity_um_s[1] = feedback.right_speed_um_s;
    p_reading->displacement_um[0] = feedback.left_displacement_um;
    p_reading->displacement_um[1] = feedback.right_displacement_um;
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        p_reading->speed_valid[wheel]    = feedback.speed_valid[wheel];
        p_reading->odometry_valid[wheel] = feedback.odometry_valid[wheel];
    }

    return true;
}

/*** end of file ***/
