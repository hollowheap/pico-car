/** @file obstacle.h
 *
 * @brief Buddy 5 public API. This is the only Buddy 5 header that other
 *        buddies need to include.
 *
 * Start-up order (on processor 1, which is RP2040 core 0):
 *   1. gpio_irq_init()           shared dispatcher (owned by another buddy)
 *   2. obstacle_init(&links)     Buddy 5 pins, servo, ISR and task
 *   3. obstacle_post_cmd(RESET), then obstacle_post_cmd(START)
 *
 * Navigation then talks to Buddy 5 only through commands in and events out
 * (design review section 4.1). Buddy 5 never writes the motors.
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>
#include <tk/tkernel.h>

#include "b5_cfg.h"
#include "b5_pack.h"

/* Bits of Buddy 5's event flag (flg_obs_id, Tribunal). Navigation may wait
 * on the two NAV bits through obstacle_flag_id(). */
#define B5_FLG_ECHO      (0x01U) /* Echo ISR to the Buddy 5 task.        */
#define B5_FLG_CMD       (0x02U) /* A command is waiting in the queue.   */
#define B5_FLG_STOP      (0x04U) /* STOP command; it is never queued.    */
#define B5_FLG_NAV_EVT   (0x08U) /* An event is waiting for Navigation.  */
#define B5_FLG_NAV_STEER (0x10U) /* A new steering word is available.    */

/* Emergency stop supplied by Buddy 2. */
typedef ER (*obstacle_stop_fn_t)(void);

/* Distance travelled supplied by Buddy 2, in mm. */
typedef ER (*obstacle_distance_fn_t)(uint32_t * p_distance_mm);

/* Links handed to Buddy 5 at start-up. Any member may be 0 or NULL.
 * h_nav_mbf 0 means "use Buddy 5's own 4-slot outbox". */
typedef struct
{
    ID                     h_nav_mbf;
    obstacle_stop_fn_t     p_stop;
    obstacle_distance_fn_t p_get_distance_mm;
} obstacle_links_t;

ER obstacle_init (obstacle_links_t const * p_links);
ER obstacle_post_cmd (uint8_t cmd);
ER obstacle_get_event (uint32_t * p_evt, TMO tmout);

uint32_t obstacle_get_steer (void);
uint32_t obstacle_get_status (void);
uint8_t  obstacle_get_faults (void);
uint8_t  obstacle_dropped_events (void);

ER obstacle_copy_profile (uint16_t * p_profile);
ID obstacle_flag_id (void);

#endif /* OBSTACLE_H */

/*** end of file ***/

