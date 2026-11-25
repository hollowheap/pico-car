/** @file service.h
 * @brief Obstacle scanner lifecycle, commands and observations.
 *
 * Set up the shared GPIO dispatcher on core 0 before calling
 * obstacle_extras_setup(). Then send RESET and START to monitor the course.
 * The scanner reports steering and never drives the motors directly.
 */

#ifndef OBSTACLE_EXTRAS_SERVICE_H
#define OBSTACLE_EXTRAS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "config.h"
#include "packed.h"

/* Bit flags for scan inputs and navigation outputs. */
#define EXTRAS_FLG_ECHO      (0x01U) /* Echo ISR to the scanner task.        */
#define EXTRAS_FLG_CMD       (0x02U) /* A command is waiting in the queue.   */
#define EXTRAS_FLG_STOP      (0x04U) /* STOP command; it is never queued.    */
#define EXTRAS_FLG_NAV_EVT   (0x08U) /* An event is waiting for Navigation.  */
#define EXTRAS_FLG_NAV_STEER (0x10U) /* A new steering word is available.    */

/* Emergency stop supplied by motion. */
typedef ER (*obstacle_extras_stop_fn_t)(void);

/* Distance travelled supplied by motion, in mm. */
typedef ER (*obstacle_extras_distance_fn_t)(uint32_t * p_distance_mm);

/* Optional motion links. A zero h_nav_mbf uses the scanner's own outbox. */
typedef struct
{
    ID                            h_nav_mbf;
    obstacle_extras_stop_fn_t     p_stop;
    obstacle_extras_distance_fn_t p_get_distance_mm;
} obstacle_extras_links_t;

bool obstacle_extras_ready (void);
ER obstacle_extras_setup (obstacle_extras_links_t const * p_links);
ER obstacle_extras_post_cmd (uint8_t cmd);
ER obstacle_extras_get_event (uint32_t * p_evt, TMO tmout);

uint32_t obstacle_extras_get_steer (void);
void     obstacle_extras_set_look (int16_t deci);
void     obstacle_extras_set_track (bool b_on);
void     obstacle_extras_set_sweep (bool b_on);
uint32_t obstacle_extras_get_status (void);
uint32_t obstacle_extras_get_ping_tag (void);
uint8_t  obstacle_extras_get_faults (void);
uint8_t  obstacle_extras_dropped_events (void);

ER obstacle_extras_copy_profile (uint16_t * p_profile);
ID obstacle_extras_flag_id (void);

#endif /* OBSTACLE_EXTRAS_SERVICE_H */

/*** end of file ***/
