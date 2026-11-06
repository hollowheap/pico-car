/** @file physics_task.h
 *
 * @brief Buddy 4 Hump task for micro T-Kernel 3.0 (target only).
 *
 * Every 20 ms (cyclic handler wake-up):
 *   1. Read the accelerometer (and the magnetometer every mag_every cycles)
 *      over I2C1 GP2/GP3.
 *   2. Read Buddy 2's motion status through the optional read_b2 callback.
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
    float    b2_accel_alpha;     /* dv/dt smoothing, see motion_adapter.h.  */
    /* Fill *p_reading from Buddy 2. NULL = Buddy 2 not used (bring-up). */
    bool   (*read_b2) (motion_reading_t * p_reading);
    /* Called with the 3 Buddy 4 fault bits whenever they change, so the
     * integrator can update the shared fault word. NULL = not used. */
    void   (*publish_faults) (uint8_t physics_faults);
    hump_config_t       hump;
    imu_motion_config_t motion;
} physics_task_config_t;

typedef struct
{
    uint32_t cycles;
    uint32_t overruns;           /* Wake-ups missed while still working.   */
    uint32_t max_work_us;        /* Longest cycle, read to send.           */
    uint32_t nav_sent;
    uint32_t nav_dropped;
    uint32_t telem_sent;
    uint32_t telem_dropped;
    uint32_t sensor_inits;       /* Attempts to start the LSM303.          */
    lsm303_result_t last_init;
    lsm303_stats_t  i2c;
    packed_hump_t last_word;
} physics_task_stats_t;

void physics_task_default_config (physics_task_config_t * p_cfg);

/* Create the message buffers and the cyclic handler. The composition root
 * creates the task named by task_id and calls physics_task_run() from it. The
 * cyclic handler needs the task id, so create the task first. Returns E_OK
 * or the first kernel error. */
ER physics_task_init (const physics_task_config_t * p_cfg, ID task_id);

/* Task body. Never returns. */
void physics_task_run (void);

/* Message buffer IDs for receivers (4-byte messages). Valid after start. */
ID physics_task_nav_mbf (void);
ID physics_task_telem_mbf (void);

/* Consistent copy of the counters; safe from any task on either core. */
void physics_task_get_stats (physics_task_stats_t * p_stats);

/* Request a new run (recalibrate, reset run peak) on the next cycle.
 * Only while the car is at rest. */
void physics_task_request_new_run (void);

/* Clear the latched I2C fault on the next cycle. */
void physics_task_request_clear_fault (void);

#endif /* PHYSICS_TASK_H */

/*** end of file ***/
