/** @file comms.h
 *
 * @brief the obstacle task private communications: the command queue, the event
 *        outbox, the copies other tasks read without blocking the obstacle task, and
 *        the links to the motion service. Only obstacle.c and scan_task.c use it.
 */

#ifndef OBSTACLE_COMMS_H
#define OBSTACLE_COMMS_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "statechart.h"
#include "obstacle_internal.h"

ER   obstacle_comms_init (obstacle_links_t const * p_links);
void obstacle_comms_set_init_failed (void);
ID   obstacle_comms_flag (void);

ER   obstacle_comms_post_cmd (uint8_t cmd);
bool obstacle_comms_pop_cmd (uint8_t * p_cmd);

void     obstacle_comms_publish (obstacle_statechart_out_t const * p_out, obstacle_statechart_t const * p_fsm);
uint32_t obstacle_comms_odometer_mm (uint32_t now_ms);

ER       obstacle_comms_get_event (uint32_t * p_evt, TMO tmout);
uint32_t obstacle_comms_steer (void);
uint32_t obstacle_comms_status (void);
uint16_t obstacle_comms_faults (void);
uint8_t  obstacle_comms_drops (void);
ER       obstacle_comms_copy_profile (uint16_t * p_profile);
ER       obstacle_comms_copy_plan (obstacle_plan_summary_t * p_plan);
uint32_t obstacle_comms_aim (void);
uint8_t  obstacle_comms_flags (void);
void     obstacle_comms_count_class (uint8_t cls);

/* Navigation tells the scan task that the body is turning (a scan pivot or a
 * spin of the face follow). The sonar keeps pinging, but only the collision and health
 * checks use those pings: they point at the wrong angle for a scan slot or the
 * forward guard. */
void     obstacle_comms_set_pivoting (bool b_pivoting);
bool     obstacle_comms_is_pivoting (void);

/* Navigation asks for the servo angle in REACQUIRE: the face follow looks at
 * the face. Both tasks run on core 0; the value is one aligned
 * 16-bit word, so it is written and read whole. */
void     obstacle_comms_set_look (int16_t deci);
int16_t  obstacle_comms_look (void);
void     obstacle_comms_class_counts (uint32_t * p_counts);

#endif /* OBSTACLE_COMMS_H */

/*** end of file ***/

