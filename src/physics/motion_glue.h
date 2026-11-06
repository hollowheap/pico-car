/** @file motion_glue.h
 *
 * @brief Read Buddy 2's motion service into motion_reading_t (target only).
 */

#ifndef MOTION_GLUE_H
#define MOTION_GLUE_H

#include <stdbool.h>

#include "motion_adapter.h"

/* Matches physics_task_config_t.read_b2. Returns false if the service is not
 * running or the status read failed. */
bool motion_read_b2 (motion_reading_t * p_reading);

#endif /* MOTION_GLUE_H */

/*** end of file ***/
