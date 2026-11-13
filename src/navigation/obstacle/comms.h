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
int16_t  obstacle_comms_body_target (void);
uint8_t  obstacle_comms_flags (void);
void     obstacle_comms_count_class (uint8_t cls);
void     obstacle_comms_class_counts (uint32_t * p_counts);

/* Navigation owns this mailbox. A scan is enabled only by the full car;
 * the standalone sonar bench cannot turn the chassis. */
#define OBSTACLE_SONAR_FORWARD (0U)
#define OBSTACLE_SONAR_WAIT    (1U)
#define OBSTACLE_SONAR_SCAN    (2U)
#define OBSTACLE_SONAR_SIDE    (3U)

void obstacle_comms_enable_nav_sonar (void);
bool obstacle_comms_has_nav_sonar (void);
void obstacle_comms_set_nav_sonar (uint8_t mode, int16_t target_deci);
uint32_t obstacle_comms_nav_sonar (void);
uint8_t obstacle_comms_sonar_mode (uint32_t word);
int16_t obstacle_comms_sonar_target (uint32_t word);

#endif /* OBSTACLE_COMMS_H */

/*** end of file ***/

