/** @file obstacle.h
 *
 * @brief Buddy 5 private interface. Never include this from outside
 *        src/navigation/obstacle/; use navigation/obstacle.h instead.
 *
 * The public facade is included here so every implementation file gets the
 * same types. The packed accessors and the event flag bits stay private:
 * only the statechart task and Navigation glue use them.
 */

#ifndef OBSTACLE_INTERNAL_H
#define OBSTACLE_INTERNAL_H

#include <stdint.h>

#include "navigation/obstacle.h"

/* Bits of Buddy 5's event flag. Navigation waits on the two NAV bits
 * through obstacle_flag_id(). */
#define OBSTACLE_FLG_ECHO      (0x01U) /* Echo ISR to the Buddy 5 task.      */
#define OBSTACLE_FLG_CMD       (0x02U) /* A command is waiting in the queue. */
#define OBSTACLE_FLG_STOP      (0x04U) /* STOP command; it is never queued.  */
#define OBSTACLE_FLG_NAV_EVT   (0x08U) /* An event waits for Navigation.     */
#define OBSTACLE_FLG_NAV_STEER (0x10U) /* A new steering word is available.  */

/* Packed status word, read in one 32-bit load. The layout is private. */
uint32_t obstacle_get_status (void);

/* Latest STEER word; its sequence field changes on every decision. */
uint32_t obstacle_get_steer (void);

/* Events dropped because an inbox or the outbox was full. */
uint8_t obstacle_dropped_events (void);

#endif /* OBSTACLE_INTERNAL_H */

/*** end of file ***/
