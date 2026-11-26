/** @file pipeline.h
 *
 * @brief One 20 ms Buddy 4 cycle: sensor values in, packed messages out.
 *
 * Portable glue between the LSM303 readings, Buddy 2's motion data and the
 * logic modules (hump_detect, imu_motion, hump_msg). The RTOS task only
 * reads the sensors, calls pipeline_step() and sends what it returns.
 *
 * Sending rules:
 *   - Nav gets a message only when something it acts on changes: IMU
 *     calibration done, hump started or finished, motion class changed, or
 *     the fault bits changed.
 *   - Telemetry gets every Nav message plus one every telem_every cycles.
 *
 * A failed accelerometer read cannot be run through the filters. The cycle
 * then repeats the last message with PHYSICS_FAULT_I2C set. The distance and
 * time of the failed cycle are added to the next good sample; if Buddy 2's
 * data was invalid during a failed cycle, that next sample is marked
 * invalid so the hump height is not trusted.
 */

#ifndef PHYSICS_PIPELINE_H
#define PHYSICS_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "hump_detect.h"
#include "hump_msg.h"
#include "attitude.h"
#include "imu_motion.h"
#include "drv_lsm303.h"
#include "platform/chassis.h"

/* Accelerometer reads failed this many cycles in a row (0.5 s): the IMU is
 * lost. Unmeasured; a shorter burst rides through on the carried travel, as
 * a 10-cycle gap in a hump does in the host tests. */
#define PIPELINE_LOST_CYCLES (25U)

/* The stale-motion fault bit stays set this many clean cycles (0.2 s, one
 * status frame period) after the last stale one. Every change of the fault
 * bits is a telemetry event, and a marginal encoder would flip the bit each
 * cycle. */
#define PIPELINE_STALE_HOLD_CYCLES (10U)

/* Hump validity. The car drives over test humps up to 30 mm high; past a
 * pitch or height limit the reading is not a valid hump, the hump flag clears
 * at once and the lift and overturn checks and the line follower act as on
 * level ground. All values below are guesses, not measured on the car. */
#define PIPELINE_HUMP_MAX_MM           (30)
#define PIPELINE_HUMP_HEIGHT_BUDGET_MM (10)
/* Margin over the peak pitch of the tallest valid hump, asin (30 mm / axle to
 * caster) = 14.5 deg: the rest calibration may be off by up to the 2 deg
 * spread it accepts and vibration adds about half a degree. 14.5 + 2.5 = 17
 * deg stays 3 deg under the 20 deg tilt envelope of the lift check, so a
 * lift still reads as a lift. */
#define PIPELINE_HUMP_PITCH_MARGIN_DEG (2.5f)
/* The test hump rises over 100 mm and falls over 100 mm. The lift mask ends
 * after 1.5 times the span from the first wheel on the ramp to the last
 * wheel off it: 2 x 100 mm plus the axle to caster distance, 480 mm in all.
 * A 30 mm hump on the same 0.2 slope spans 420 mm and stays inside it. */
#define PIPELINE_HUMP_RAMP_MM          (100)
#define PIPELINE_HUMP_MASK_MM \
    ((3 * ((2 * PIPELINE_HUMP_RAMP_MM) + CHASSIS_AXLE_TO_CASTER_MM)) / 2)

/* Motion data from Buddy 2, already converted to SI units. */
typedef struct
{
    bool  valid;
    float age_s;                 /* Age at the time of this cycle.         */
    float speed_mps;             /* Forward, mean of both wheels.          */
    float ds_m;                  /* Distance since the previous cycle.     */
    float accel_mps2;            /* Forward dv/dt.                         */
    bool  wheels_valid;
    bool  wheels_moved;          /* Encoder activity between motion observations. */
    float v_left_mps;
    float v_right_mps;
} motion_input_t;

typedef struct
{
    hump_ctx_t        hump;
    imu_motion_ctx_t  motion;
    attitude_ctx_t    attitude;
    float             hump_pitch_max_deg; /* Pitch limit of a valid hump. */
    float             hump_travel_m; /* Travel since the hump began. */
    bool              hump_rejected; /* The hump in progress is not valid. */
    bool              wheels_free;   /* Second lift cue, set by the owner. */
    hump_run_t          run;
    uint32_t          telem_every;
    uint32_t          cycle;
    bool              have_last;
    packed_hump_t  last_word;
    uint8_t           last_faults;
    motion_class_t       last_motion;
    /* Travel and time from cycles whose accelerometer read failed, added
     * to the next good sample so no distance is lost from the height sum. */
    float             carry_ds_m;
    float             carry_dt_s;
    bool              carry_invalid;
    uint16_t          failed_cycles; /* Cycles in a row without accel data. */
    uint8_t           stale_hold;    /* Clean cycles left to hold the stale
                                        fault. */
} pipeline_t;

typedef struct
{
    packed_hump_t    word;
    bool                to_nav;
    bool                to_telem;
    bool                sensors_ok;  /* False when accel read failed.      */
    bool                imu_lost;    /* PIPELINE_LOST_CYCLES failed reads:
                                        attitude is the not-valid default. */
    bool                imu_lost_started; /* First cycle of a loss. */
    bool                imu_recovered;    /* First good read after a loss. */
    bool                calibrated;      /* Rest reference set; failed reads
                                            keep it. */
    bool                attitude_unknown; /* No rest reference: at boot, after
                                             a new run or after an IMU loss. */
    hump_output_t       hump;        /* Valid only when sensors_ok.        */
    imu_motion_output_t motion;      /* Valid only when sensors_ok.        */
    attitude_output_t   attitude;    /* Valid when sensors_ok and not
                                        attitude_unknown. */
    bool                hump_rejected; /* The hump was judged not valid on
                                          this cycle; set once per hump.    */
} pipeline_step_t;

/* telem_every >= 1. Returns false for a null pointer or a bad config. */
bool pipeline_init (pipeline_t * p_pipe,
                       const hump_config_t * p_hump_cfg,
                       const imu_motion_config_t * p_motion_cfg,
                       uint32_t telem_every);

/* p_acc_g: accelerometer X, Y, Z in g, or NULL if the read failed.
 * p_mag_g: magnetometer X, Y, Z in gauss, or NULL if not read this cycle.
 * i2c_fault: the driver's latched fault flag.
 * Returns false only for a null p_pipe, p_motion or p_out, or bad dt. */
bool pipeline_step (pipeline_t * p_pipe,
                       const float * p_acc_g,
                       const float * p_mag_g,
                       const motion_input_t * p_motion,
                       float dt_s,
                       bool i2c_fault,
                       pipeline_step_t * p_out);

/* Start a new run at rest: recalibrate and reset the run peak. */
void pipeline_new_run (pipeline_t * p_pipe);

/* Read the accelerometer for this cycle, capping retries at RETRY_ATTEMPTS
 * after a failed cycle. Returns false when the sensor is not ready. */
bool pipeline_read_accel (const pipeline_t * p_pipe, lsm303_t * p_imu,
                          uint8_t attempts, lsm303_vec_t * p_acc);

/* Read the magnetometer for this cycle, capping retries at RETRY_ATTEMPTS
 * after a failed cycle. Returns false when the sensor is not ready. */
bool pipeline_read_mag (const pipeline_t * p_pipe, lsm303_t * p_imu,
                        uint8_t attempts, lsm303_vec_t * p_mag);

/* One cycle's reads: the accelerometer, then the magnetometer when b_mag_due.
 * attempts is the configured attempts per transaction, capped at
 * RETRY_ATTEMPTS after a failed cycle. Sets *p_acc_ok and *p_mag_ok; leaves
 * both false when the sensor is not ready. */
void pipeline_read (const pipeline_t * p_pipe, lsm303_t * p_imu,
                    uint8_t attempts, bool b_mag_due,
                    lsm303_vec_t * p_acc, lsm303_vec_t * p_mag,
                    bool * p_acc_ok, bool * p_mag_ok);

/* The configuration readback for this cycle, when it is due: every every
 * cycles, on a ready sensor. Returns true when the configuration does not
 * match, so the sensor must be initialised again. */
bool pipeline_config_readback (const pipeline_t * p_pipe, lsm303_t * p_imu,
                               uint32_t cycle, uint32_t every,
                               uint8_t attempts);

/* True when this cycle's attitude fields are current: the sensors were read
 * and a rest reference is set. */
bool pipeline_attitude_known (const pipeline_step_t * p_step);

#endif /* PHYSICS_PIPELINE_H */

/*** end of file ***/
