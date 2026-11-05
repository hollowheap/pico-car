/** @file b4_b2_glue.c
 *
 * @brief Read Buddy 2's motion feedback into b4_b2_reading_t (target only).
 *
 * Link with the team "motion" library. Pass b4_read_b2_motion_service as
 * b4_task_config_t.read_b2 once motion_init() has been called; before the
 * motor task publishes its first sample the flags read invalid.
 */

/* motion.h brings in the kernel headers, which define NULL and size_t;
 * <stddef.h> would conflict. */
#include "motion/motion.h"

#include <stdbool.h>
#include <stdint.h>

#include "b4_b2_adapter.h"
#include "b4_b2_glue.h"

bool b4_read_b2_motion_service (b4_b2_reading_t * p_reading)
{
    motion_feedback_t fb;

    if ((NULL == p_reading) || (E_OK != motion_feedback_snapshot (&fb)))
    {
        return false;
    }

    /* After STOP or FAULT, motion publishes zeros with both flags false
     * and timestamp 0, which b4_b2_convert treats as invalid. */
    p_reading->available = (fb.speed_valid || fb.odometry_valid);
    p_reading->timestamp_us = fb.timestamp_us;
    p_reading->velocity_um_s[B4_B2_LEFT] = fb.left_speed_um_s;
    p_reading->velocity_um_s[B4_B2_RIGHT] = fb.right_speed_um_s;
    p_reading->displacement_um[B4_B2_LEFT] = fb.left_displacement_um;
    p_reading->displacement_um[B4_B2_RIGHT] = fb.right_displacement_um;

    /* Motion only reports combined (both-wheel) validity. */
    p_reading->speed_valid[B4_B2_LEFT] = fb.speed_valid;
    p_reading->speed_valid[B4_B2_RIGHT] = fb.speed_valid;
    p_reading->odometry_valid[B4_B2_LEFT] = fb.odometry_valid;
    p_reading->odometry_valid[B4_B2_RIGHT] = fb.odometry_valid;

    return true;
}

/*** end of file ***/
