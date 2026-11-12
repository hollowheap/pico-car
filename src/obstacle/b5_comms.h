/** @file b5_comms.h
 *
 * @brief Buddy 5 private communications: the command queue, the event
 *        outbox, the copies other tasks read without blocking Buddy 5, and
 *        the links to Buddy 2. Only obstacle.c and scan_task.c use it.
 */

#ifndef B5_COMMS_H
#define B5_COMMS_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "b5_fsm.h"
#include "obstacle.h"

ER   b5_comms_init (obstacle_links_t const * p_links);
void b5_comms_set_init_failed (void);
ID   b5_comms_flag (void);

ER   b5_comms_post_cmd (uint8_t cmd);
bool b5_comms_pop_cmd (uint8_t * p_cmd);

void     b5_comms_publish (b5_fsm_out_t const * p_out, b5_fsm_t const * p_fsm);
uint32_t b5_comms_odometer_mm (uint32_t now_ms);

ER       b5_comms_get_event (uint32_t * p_evt, TMO tmout);
uint32_t b5_comms_steer (void);
void     b5_comms_set_look (int16_t deci);
int16_t  b5_comms_look (void);
uint32_t b5_comms_status (void);
uint8_t  b5_comms_faults (void);
uint8_t  b5_comms_drops (void);
ER       b5_comms_copy_profile (uint16_t * p_profile);

#endif /* B5_COMMS_H */

/*** end of file ***/

