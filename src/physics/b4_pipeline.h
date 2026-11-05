/** @file b4_pipeline.h
 *
 * @brief One 20 ms Buddy 4 cycle: sensor values in, packed messages out.
 *
 * Portable glue between the LSM303 readings, Buddy 2's motion data and the
 * logic modules (hump_detect, imu_motion, hump_msg). The RTOS task only
 * reads the sensors, calls b4_pipeline_step() and sends what it returns.
 *
 * Sending rules:
 *   - Nav gets a message only when something it acts on changes: IMU
 *     calibration done, hump started or finished, motion class changed, or
 *     the fault bits changed.
 *   - Telemetry gets every Nav message plus one every telem_every cycles.
 *
 * A failed accelerometer read cannot be run through the filters. The cycle
 * then repeats the last message with B4_FAULT_I2C set. The distance and
 * time of the failed cycle are added to the next good sample; if Buddy 2's
 * data was invalid during a failed cycle, that next sample is marked
 * invalid so the hump height is not trusted.
 */

#ifndef B4_PIPELINE_H
#define B4_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "hump_detect.h"
#include "hump_msg.h"
#include "imu_motion.h"

/* Motion data from Buddy 2, already converted to SI units. */
typedef struct
{
    bool  valid;
    float age_s;                 /* Age at the time of this cycle.         */
    float speed_mps;             /* Forward, mean of both wheels.          */
    float ds_m;                  /* Distance since the previous cycle.     */
    float accel_mps2;            /* Forward dv/dt.                         */
    bool  wheels_valid;
    float v_left_mps;
    float v_right_mps;
} b4_motion_in_t;

typedef struct
{
    hump_ctx_t        hump;
    imu_motion_ctx_t  motion;
    b4_run_t          run;
    uint32_t          telem_every;
    uint32_t          cycle;
    bool              have_last;
    b4_packed_hump_t  last_word;
    uint8_t           last_faults;
    b4_motion_t       last_motion;
    /* Travel and time from cycles whose accelerometer read failed, added
     * to the next good sample so no distance is lost from the height sum. */
    float             carry_ds_m;
    float             carry_dt_s;
    bool              carry_invalid;
} b4_pipeline_t;

typedef struct
{
    b4_packed_hump_t    word;
    bool                to_nav;
    bool                to_telem;
    bool                sensors_ok;  /* False when accel read failed.      */
    hump_output_t       hump;        /* Valid only when sensors_ok.        */
    imu_motion_output_t motion;      /* Valid only when sensors_ok.        */
} b4_step_out_t;

/* telem_every >= 1. Returns false for a null pointer or a bad config. */
bool b4_pipeline_init (b4_pipeline_t * p_pipe,
                       const hump_config_t * p_hump_cfg,
                       const imu_motion_config_t * p_motion_cfg,
                       uint32_t telem_every);

/* p_acc_g: accelerometer X, Y, Z in g, or NULL if the read failed.
 * p_mag_g: magnetometer X, Y, Z in gauss, or NULL if not read this cycle.
 * i2c_fault: the driver's latched fault flag.
 * Returns false only for a null p_pipe, p_motion or p_out, or bad dt. */
bool b4_pipeline_step (b4_pipeline_t * p_pipe,
                       const float * p_acc_g,
                       const float * p_mag_g,
                       const b4_motion_in_t * p_motion,
                       float dt_s,
                       bool i2c_fault,
                       b4_step_out_t * p_out);

/* Start a new run at rest: recalibrate and reset the run peak. */
void b4_pipeline_new_run (b4_pipeline_t * p_pipe);

#endif /* B4_PIPELINE_H */

/*** end of file ***/
