/** @file comms.h
 *
 * @brief Buddy 5 private communications: the command queue, the event
 *        outbox, the copies other tasks read without blocking Buddy 5, and
 *        the links to Buddy 2. Only obstacle.c and scan_task.c use it.
 */

#ifndef OBSTACLE_COMMS_H
#define OBSTACLE_COMMS_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "statechart.h"
#include "obstacle.h"

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
uint8_t  obstacle_comms_faults (void);
uint8_t  obstacle_comms_drops (void);
ER       obstacle_comms_copy_profile (uint16_t * p_profile);

#endif /* OBSTACLE_COMMS_H */

/*** end of file ***/

