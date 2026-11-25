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

#include "sonar_health.h"
#include "obstacle_internal.h"

/* The sonar task waits on this bit for a ping request (see
 * obstacle_ping_request()). */
#define OBSTACLE_FLG_PING (0x20U)

/* The status scalars, copied together so that they come from one step. */
typedef struct
{
    uint32_t status; /* Packed status word.                          */
    uint32_t aim;    /* Servo deci-degrees low, body deci-degrees high. */
    uint16_t faults; /* OBSTACLE_FAULT_* bits; give-up reason in 12-14. */
    uint8_t  flags;  /* OBSTACLE_FLAG_* bits.                        */
    uint8_t  drops;  /* Events and commands dropped, saturating.     */
} obstacle_comms_scalars_t;

ER   obstacle_comms_init (obstacle_links_t const * p_links);
void obstacle_comms_set_init_failed (void);
ID   obstacle_comms_flag (void);

ER   obstacle_comms_post_cmd (uint8_t cmd);
bool obstacle_comms_pop_cmd (uint8_t * p_cmd);

void     obstacle_comms_publish (sonar_health_t const * p_health);
bool     obstacle_comms_travel (obstacle_travel_t * p_travel);

ER       obstacle_comms_get_event (uint32_t * p_evt, TMO tmout);
uint32_t obstacle_comms_steer (void);
uint32_t obstacle_comms_status (void);
uint32_t obstacle_comms_ping (void);
uint32_t obstacle_comms_take_ping_request (void);
void     obstacle_comms_publish_ping_reply (uint32_t word);
/* Copy a centered free-running shot, or invalidate it at the start of an aim.
 */
void obstacle_comms_publish_forward (
    const obstacle_forward_sample_t * p_sample);
uint16_t obstacle_comms_faults (void);
uint8_t  obstacle_comms_drops (void);
void     obstacle_comms_copy_scalars (obstacle_comms_scalars_t * p_out);
ER       obstacle_comms_copy_profile (uint16_t * p_profile);
ER       obstacle_comms_copy_plan (obstacle_plan_summary_t * p_plan);
uint32_t obstacle_comms_aim (void);
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

