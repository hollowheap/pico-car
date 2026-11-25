/** @file physics.h
 *
 * @brief Physics public facade.
 *
 * This is the only physics header another subsystem may include. The
 * LSM303 driver, the filters, the packed hump word and the task are
 * private to src/physics/.
 *
 * `physics_init()` sets up the 20 ms task and binds the motion service's motion
 * service as the speed and odometry source. Telemetry reads
 * `physics_snapshot()`; it does not unpack the wire word.
 */

#ifndef PHYSICS_PHYSICS_H
#define PHYSICS_PHYSICS_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "platform/static_assert.h"

/* Latest state in engineering units. The numeric codes for state, motion
 * and faults match the wire schema owned by protocol/ and the dashboard
 * decoder; this header does not fix their values. */
typedef struct
{
    uint32_t cycles;         /* Task cycles since start.                */
    uint32_t overruns;       /* Wake-ups missed while still working.    */
    uint32_t dropped;        /* Messages lost to a full buffer.         */
    int16_t  pitch_cdeg;     /* Pitch in centi-degrees.                 */
    uint16_t hump_peak_mm;   /* Peak height of the current hump, mm.    */
    uint16_t run_peak_mm;    /* Highest valid hump in the run, mm.      */
    uint8_t  state;          /* Hump state: idle, rising, falling, ...  */
    uint8_t  motion;         /* Motion class from the IMU.              */
    uint8_t  faults;         /* Latched B4 faults.                      */
    uint8_t  heading_deg;    /* Tilt-compensated magnetic heading.      */
    bool     ready;          /* The task has produced a message.        */
    bool     calibrated;     /* Hump detector has its rest reference.   */
    bool     heading_valid;  /* heading_deg is fresh and usable.        */
    bool     hump_active;    /* A hump is in progress.                  */
    bool     finished;       /* The last hump finished.                 */
    bool     height_valid;   /* hump_peak_mm is trustworthy.            */
} physics_status_t;

STATIC_ASSERT (sizeof(physics_status_t) == 28U, "physics_status_t size");

/* Set up the task's message buffers and cyclic handler. The composition
 * root creates the task named by task_id and calls physics_run() from it,
 * so create the task before calling this. Call once, after
 * motion_service_init(), because the default configuration reads the motion service's
 * motion service. */
ER physics_init (ID task_id);

/* Task body. Never returns. */
void physics_run (void);

/* Copy the latest state. Returns E_PAR for a null pointer, or E_OK with
 * ready == false before the first message. */
ER physics_snapshot (physics_status_t * p_status);

/* Request a new run at rest: recalibrate and reset the run peak. */
void physics_request_new_run (void);

/* Clear the latched I2C fault on the next cycle. */
void physics_request_clear_fault (void);

#endif /* PHYSICS_PHYSICS_H */

/*** end of file ***/
