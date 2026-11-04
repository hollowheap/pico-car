/** @file b4_b2_glue.c
 *
 * @brief Read Buddy 2's motion service into b4_b2_reading_t (target only).
 *
 * Link with buddy2_motion_service. Pass b4_read_b2_motion_service as
 * b4_task_config_t.read_b2 once motion_service_start() has been called.
 */

/* motion_service.h brings in the kernel headers, which define NULL and
 * size_t; <stddef.h> would conflict. */
#include "motion_service.h"

#include <stdbool.h>
#include <stdint.h>

#include "b4_b2_adapter.h"
#include "b4_b2_glue.h"

bool b4_read_b2_motion_service (b4_b2_reading_t * p_reading)
{
    motion_service_status_t status;
    uint32_t wheel;

    if ((NULL == p_reading) || (E_OK != motion_service_read_status (&status)))
    {
        return false;
    }

    p_reading->available = status.control.sample_available;
    p_reading->timestamp_us = status.control.sample.timestamp_us;
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        const motion_wheel_sample_t * p_w = &status.control.sample.wheel[wheel];

        p_reading->velocity_um_s[wheel] = p_w->velocity_um_s;
        p_reading->displacement_um[wheel] = p_w->displacement_um;
        p_reading->speed_valid[wheel] = p_w->speed_valid;
        p_reading->odometry_valid[wheel] = p_w->odometry_valid;
    }

    return true;
}

/*** end of file ***/
