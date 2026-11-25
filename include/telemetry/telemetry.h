/** @file telemetry.h
 *
 * @brief Telemetry subsystem: frame queues, the telemetry task and the
 *        connection task.
 */

#ifndef TELEMETRY_TELEMETRY_H
#define TELEMETRY_TELEMETRY_H

#include <stdint.h>

/* Create the frame queues and prepare the connection. Telemetry is optional:
   the car still runs when either fails. */
void telemetry_init (void);

/* Create and start the telemetry and connection tasks. A failure is logged
   and the task is left out of telemetry_live_mask(). */
void telemetry_start (void);

/* Bits of LIVE_* for the telemetry tasks that were created. */
uint16_t telemetry_live_mask (void);

#endif /* TELEMETRY_TELEMETRY_H */

/*** end of file ***/
