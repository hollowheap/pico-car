/** @file comms.h
 *
 * @brief Buddy 5 private communications: the command queue, the event
 *        outbox, the copies other tasks read without blocking Buddy 5, and
 *        the links to Buddy 2. Only obstacle.c and scan_task.c use it.
 */

#ifndef EXTRAS_COMMS_H
#define EXTRAS_COMMS_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "statechart.h"
#include "service.h"
#include "obstacle/obstacle.h"

ER   extras_comms_init (obstacle_extras_links_t const * p_links);
void extras_comms_set_init_failed (void);
ID   extras_comms_flag (void);

ER   extras_comms_post_cmd (uint8_t cmd);
bool extras_comms_pop_cmd (uint8_t * p_cmd);

void     extras_comms_publish (extras_fsm_out_t const * p_out, extras_fsm_t const * p_fsm);
uint32_t extras_comms_odometer_mm (uint32_t now_ms);

ER       extras_comms_get_event (uint32_t * p_evt, TMO tmout);
uint32_t extras_comms_steer (void);
void     extras_comms_set_look (int16_t deci);
int16_t  extras_comms_look (void);
void     extras_comms_set_track (bool b_on);
bool     extras_comms_track (void);
void     extras_comms_set_sweep (bool b_on);
bool     extras_comms_sweep (void);
uint32_t extras_comms_status (void);
void     extras_comms_publish_ping (uint8_t seq, int16_t deci, uint8_t cls, uint16_t dist_mm,
                                   uint32_t trigger_us);
uint32_t extras_comms_ping_tag (void);
uint32_t extras_comms_ping (void);
uint32_t extras_comms_ping_recent (uint32_t now_us);
ER       extras_comms_forward_snapshot (obstacle_forward_sample_t * p_sample);
void     extras_comms_echo_counts (uint32_t * p_counts);
uint8_t  extras_comms_faults (void);
uint8_t  extras_comms_drops (void);
ER       extras_comms_copy_profile (uint16_t * p_profile);
ER       extras_comms_copy_plan (obstacle_plan_summary_t * p_plan);
void     extras_comms_bind_aim (obstacle_sonar_aim_fn_t p_observe);
void     extras_comms_aim (int16_t deci);

#endif /* EXTRAS_COMMS_H */

/*** end of file ***/
