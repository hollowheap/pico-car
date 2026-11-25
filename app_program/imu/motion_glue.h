/** @file motion_glue.h
 *
 * @brief Read the motion service into motion_reading_t (target only).
 */

#ifndef MOTION_GLUE_H
#define MOTION_GLUE_H

#include <stdbool.h>

#include "motion_adapter.h"

/* Matches physics_task_config_t.read_motion. Returns false if the service is not
 * running or the status read failed. */
bool motion_glue_read (motion_reading_t * p_reading);

#endif /* MOTION_GLUE_H */

/*** end of file ***/
