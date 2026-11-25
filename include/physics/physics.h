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
    uint32_t reference_request; /* Latest new-run request applied to this sample. */
    uint32_t overruns;       /* Wake-ups missed while still working.    */
    uint32_t forward_sample_us; /* Hardware timer of independent accelerometer sample. */
    int16_t  pitch_cdeg;     /* Pitch in centi-degrees.                 */
    int16_t  yaw_rate_cdps;  /* Best available yaw rate, 0.01 deg/s.    */
    int16_t  a_fwd_mg;       /* Forward acceleration residual, mg.      */
    int16_t  forward_specific_mg; /* Signed forward accelerometer reading, including gravity. */
    int16_t  report_roll_cdeg; /* Smoothed telemetry roll in centidegrees. */
    int16_t  roll_cdeg;      /* Roll, left-down positive, 0.01 deg.     */
    int16_t  az_mg;          /* Board z reading, mg.                    */
    uint16_t amag_mg;        /* Acceleration magnitude, mg.             */
    uint16_t heading_ddeg;   /* Tilt-compensated heading, 0..3599 ddeg. */
    uint16_t hump_peak_mm;   /* Peak height of the current hump, mm.    */
    uint16_t run_peak_mm;    /* Highest valid hump in the run, mm.      */
    uint8_t  state;          /* Hump state: idle, rising, falling, ...  */
    uint8_t  motion;         /* Motion class from the IMU.              */
    uint8_t  faults;         /* Latched B4 faults.                      */
    uint8_t  attitude_state; /* Wire attitude_state: ok, candidate, lifted, overturned. */
    bool     attitude_sample_valid; /* Fresh calibrated tilt measurement. */
    bool     ready;          /* The task has produced a recent message. */
    bool     calibrated;     /* Hump detector has its rest reference.   */
    bool     heading_valid;  /* heading_ddeg is fresh and usable.       */
    bool     heading_gated;  /* Offset learnt, field in band and rates agree. */
    bool     hump_active;    /* A hump is in progress.                  */
    bool     finished;       /* The last hump finished.                 */
    bool     height_valid;   /* hump_peak_mm is trustworthy.            */
    bool     level_still;    /* Recent samples settled against the rest gravity vector. */
    bool     collecting_rest; /* Fresh qualifying sample in the gravity or settled-pose window. */
    uint8_t  rest_interruptions; /* Wraps when an ongoing rest window is interrupted. */
    uint8_t  pivot_scale_pct; /* Body over encoder yaw of a steady pivot on the spot,
                                 50..100 with the magnetometer offset learnt; 0 for none. */
    bool     attitude_unknown; /* No rest reference: at boot, after a new run
                                  or after an IMU loss. Attitude fields hold
                                  the last known values; do not act on them
                                  while set. */
} physics_status_t;

/* Fixed-size supervisor and telemetry snapshot. */
STATIC_ASSERT (sizeof(physics_status_t) == 56U, "physics_status_t size");

/* Set up the task's message buffers and cyclic handler. The composition
 * root creates the task named by task_id and calls physics_run() from it,
 * so create the task before calling this. Call once, after
 * motion_service_init(), because the default configuration reads the motion service's
 * motion service. */
ER physics_init (ID task_id);

/* Task body. Never returns. */
void physics_run (void);

/* Copy the latest state from a kernel task on either core. Returns E_PAR for
 * a null pointer, or E_OK with ready == false and unknown attitude when the
 * task is yet to publish or the last sample is past the IMU loss window. */
ER physics_snapshot (physics_status_t * p_status);

/* Return the new reference request ID; physics_snapshot reports its application. */
uint32_t physics_request_new_run (void);

/* The supervisor tells physics that the wheels turn on target with low duty.
 * It raises the attitude state CANDIDATE and never LIFTED. */
void physics_set_wheels_free (bool b_free);

/* Clear the latched I2C fault on the next cycle. */
void physics_request_clear_fault (void);

#endif /* PHYSICS_PHYSICS_H */

/*** end of file ***/
