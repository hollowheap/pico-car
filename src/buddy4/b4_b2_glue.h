/** @file b4_b2_glue.h
 *
 * @brief Read Buddy 2's motion feedback into b4_b2_reading_t (target only).
 */

#ifndef B4_B2_GLUE_H
#define B4_B2_GLUE_H

#include <stdbool.h>

#include "b4_b2_adapter.h"

/* Matches b4_task_config_t.read_b2. Returns false if motion_init() has not
 * run or the feedback read failed. */
bool b4_read_b2_motion_service (b4_b2_reading_t * p_reading);

#endif /* B4_B2_GLUE_H */

/*** end of file ***/
