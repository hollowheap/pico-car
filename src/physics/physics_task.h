/** @file physics_task.h
 *
 * @brief Physics Hump task for micro T-Kernel 3.0 (target only).
 *
 * Every 20 ms (cyclic handler wake-up):
 *   1. Read the accelerometer (and the magnetometer every mag_every cycles)
 *      over I2C1 GP2/GP3.
 *   2. Read the motion status through the optional read_motion callback.
 *   3. Run pipeline_step and send the packed 4-byte message to the Nav
 *      and Telemetry message buffers (TMO_POL, never blocks).
 *   4. Optionally print one CSV row over the USB console.
 *
 * I2C failures retry (max_attempts per transaction), then latch the I2C
 * fault, which is sent in the message; the task keeps polling. If the
 * sensor is missing at start-up, init is retried once a second.
 *
 * Full message buffer: the new message is dropped and counted (micro
 * T-Kernel has no drop-oldest mode; the team decision is still pending).
 */

#ifndef PHYSICS_TASK_H
#define PHYSICS_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "motion_adapter.h"
#include "hump_detect.h"
#include "imu_motion.h"
#include "lsm303.h"

typedef struct
{
    PRI      priority;           /* PostTri1: 4. See plan.md.              */
    UW       processor;          /* TP_PRC1 or TP_PRC2.                    */
    uint32_t period_ms;          /* 20.                                    */
    uint32_t mag_every;          /* Read the magnetometer every N cycles.  */
    uint32_t telem_every;        /* Periodic Telemetry message, cycles.    */
    uint32_t log_every;          /* CSV row every N cycles; 0 = off.       */
    uint8_t  i2c_attempts;       /* Attempts per I2C transaction.          */
    float    motion_accel_alpha;     /* dv/dt smoothing, see motion_adapter.h.  */
    /* Fill *p_reading from the motion service. NULL = the motion service is not used (bring-up). */
    bool   (*read_motion) (motion_reading_t * p_reading);
    hump_config_t       hump;
    imu_motion_config_t motion;
} physics_task_config_t;

typedef struct
{
    uint32_t cycles;
    uint32_t overruns;           /* Wake-ups missed while still working.   */
    uint32_t max_work_us;        /* Longest cycle, read to send.           */
    uint32_t sensor_inits;       /* Attempts to start the LSM303.          */
    lsm303_result_t last_init;
    lsm303_stats_t  i2c;
    packed_hump_t last_word;
    uint16_t heading_ddeg;       /* 0..3599 deci-degrees, tilt-compensated. */
    int16_t  yaw_rate_cdps;      /* Best available yaw rate, 0.01 deg/s.   */
    int16_t  a_fwd_mg;           /* Forward acceleration residual, mg.     */
    int16_t  roll_cdeg;          /* Roll, left-down positive, 0.01 deg.    */
    int16_t  az_mg;              /* Board z reading, mg.                   */
    uint16_t amag_mg;            /* Acceleration magnitude, mg.            */
    uint8_t  attitude_state;     /* attitude_state_t, holds on a failed read. */
    bool     heading_valid;      /* heading_ddeg is fresh and usable.      */
    bool     calibrated;         /* Hump detector has its rest reference.  */
} physics_task_stats_t;

void physics_task_default_config (physics_task_config_t * p_cfg);

/* Create the cyclic handler. The composition root
 * creates the task named by task_id and calls physics_task_run() from it. The
 * cyclic handler needs the task id, so create the task first. Returns E_OK
 * or the first kernel error. */
ER physics_task_init (const physics_task_config_t * p_cfg, ID task_id);

/* Task body. Never returns. */
void physics_task_run (void);

/* Consistent copy of the counters; safe from any task on either core. */
void physics_task_get_stats (physics_task_stats_t * p_stats);

/* Request a new run (recalibrate, reset run peak) on the next cycle.
 * Only while the car is at rest. */
void physics_task_request_new_run (void);

/* The car's settings: the mount axis, the hump limits and the motion reader.
 * physics_init() and the bring-up check both use them, so a bench sees what the
 * car sees. */
void physics_car_config (physics_task_config_t * p_cfg);

/* The supervisor sets the wheels-run-free cue (D161). It raises CANDIDATE only. */
void physics_task_set_wheels_free (bool b_free);

/* Clear the latched I2C fault on the next cycle. */
void physics_task_request_clear_fault (void);

#endif /* PHYSICS_TASK_H */

/*** end of file ***/
