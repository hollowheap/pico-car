/** @file imu_motion.h
 *
 * @brief Motion classification and turn rate.
 *
 * Pure logic with no hardware access. Runs after hump_update() on the same
 * 20 ms sample and uses its output (calibration, pitch, hump state).
 *
 * Turn rate (the LSM303DLHC has no gyroscope):
 *   - Encoder yaw rate = (v_right - v_left) / track_width. Primary source.
 *   - Magnetometer yaw rate = change in tilt-compensated heading over time.
 *     Optional cross-check and fallback, because motor current can disturb
 *     the field. A sample is rejected when |m| is outside the expected
 *     Earth-field band.
 *   - The hard-iron offset is learnt on the car: while it pivots on the spot
 *     on level ground (a line search), the horizontal field samples lie on a
 *     circle round the offset. A least-squares circle fit over a sweep of at
 *     least mag_learn_span_deg of encoder yaw replaces the offset of the two
 *     horizontal board axes, if the radius is a plausible horizontal field.
 *     The offset includes the motor field at pivot duty; at other duty it
 *     is only approximate.
 *   - When both are valid and disagree for several samples, yaw_disagree is
 *     set (wheel slip, or magnetic interference). It does not say which;
 *     the encoder rate stays the yaw source and heading_valid is cleared.
 *   Sign: positive yaw rate = turning left (counter-clockwise from above).
 *   Heading: 0 to 360 degrees, clockwise from magnetic north.
 *
 * Forward residual: measured forward reading minus sin(filtered pitch) minus
 * dv/dt / g. It is only reported for telemetry. The IMU does not detect
 * collisions; the sonar guard in the obstacle code does.
 *
 * Classes, highest priority first: UNKNOWN (not calibrated), CLIMBING /
 * DESCENDING (hump state), STATIONARY, TURNING, ACCELERATING, MOVING,
 * UNKNOWN (no usable motion data). A new class must persist class_samples
 * samples before it is reported, except calibration. LIFTED is set by the
 * pipeline from the attitude module, not by this module.
 *
 * No threshold is measured on the car; imu_motion.c says what sets the scale
 * of each. The track width is the chassis value (CHASSIS_WHEEL_TRACK_MM).
 */

#ifndef IMU_MOTION_H
#define IMU_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "hump_detect.h"

/* Motion classes. 8 values: fits the 3-bit field of packed_hump_t. */
typedef enum
{
    MOTION_CLASS_UNKNOWN = 0,       /* Not calibrated, or cannot classify.    */
    MOTION_CLASS_STATIONARY,
    MOTION_CLASS_MOVING,            /* Moving at near-constant speed, level.  */
    MOTION_CLASS_ACCELERATING,      /* Speeding up or braking.                */
    MOTION_CLASS_TURNING,
    MOTION_CLASS_CLIMBING,
    MOTION_CLASS_DESCENDING,
    MOTION_CLASS_LIFTED             /* Set by the pipeline from the attitude. */
} motion_class_t;

#define MOTION_CLASS_COUNT (8U)

/* Board axes of the 3-vectors in this module. */
#define IMU_AXIS_X     (0U)
#define IMU_AXIS_Y     (1U)
#define IMU_AXIS_Z     (2U)
#define IMU_AXIS_COUNT (3U)

/* Sums of the pivot circle fit, and the horizontal axes it fits. */
#define IMU_FIT_SUMS   (9U)
#define IMU_FIT_AXES   (2U)

typedef enum
{
    IMU_YAW_NONE = 0,
    IMU_YAW_ENCODER,
    IMU_YAW_MAG
} imu_yaw_src_t;

typedef struct
{
    /* Classification. */
    uint16_t class_samples;      /* Debounce for a class change.           */
    float    stat_speed_mps;     /* Both wheels below this: stationary.    */
    float    stat_dev_g;         /* And | |a| - 1 g | at most this.        */
    float    accel_mps2;         /* |dv/dt| above this: accelerating.      */
    float    turn_dps;           /* |yaw rate| above this: turning.        */

    /* Encoder turn rate. */
    float    track_width_m;      /* Distance between wheel contact lines.  */

    /* Magnetometer turn rate. */
    bool     mag_enabled;
    float    mag_offset_g[IMU_AXIS_COUNT]; /* Hard-iron offset per board axis. */
    int8_t   mag_yaw_sign;       /* +1, or -1 if the board frame is mirrored. */
    float    mag_min_gauss;      /* Accept |m| inside this band.           */
    float    mag_max_gauss;
    float    heading_dev_g;      /* Tilt compensation needs | |a| - 1 g |
                                    at most this.                          */
    float    mag_alpha;          /* EMA weight of a new mag yaw rate.      */
    float    mag_timeout_s;      /* No new mag sample this long: invalid.  */
    float    disagree_dps;       /* Encoder vs mag yaw difference limit.   */
    uint16_t disagree_samples;   /* Consecutive samples over the limit.    */

    /* Hard-iron learning from pivots on the spot. */
    float    mag_learn_turn_dps; /* |encoder yaw| at least this.           */
    float    mag_learn_speed_mps; /* |mean wheel speed| at most this.       */
    float    mag_learn_span_deg; /* Encoder yaw range a fit needs.         */
    float    mag_learn_min_gauss; /* Fitted radius at least this.          */

    /* Pivot witness: share of the encoder yaw the field shows. */
    float    witness_settle_s;   /* Steady one-way pivot before reporting. */
    float    witness_min_scale;  /* Lowest reported share, in (0, 1).      */
} imu_motion_config_t;

/* Per-sample inputs not already in hump_sample_t. */
typedef struct
{
    float dt_s;                  /* Time since the previous call, > 0.     */
    bool  wheels_valid;          /* Per-wheel speeds usable this sample.   */
    float v_left_mps;            /* Signed, forward positive.              */
    float v_right_mps;
    bool  mag_new;               /* mx..mz hold a new magnetometer sample. */
    float mx_g;                  /* Board-axis field in gauss.             */
    float my_g;
    float mz_g;
} imu_motion_input_t;

typedef struct
{
    motion_class_t motion;
    bool  changed;               /* motion differs from the previous call. */
    float fwd_residual_g;
    float yaw_rate_dps;          /* Best available, see yaw_src.           */
    imu_yaw_src_t yaw_src;
    float enc_yaw_dps;
    bool  enc_yaw_valid;
    float mag_yaw_dps;
    bool  mag_yaw_valid;
    float heading_deg;
    bool  heading_valid;
    bool  yaw_disagree;
    bool  mag_offset_learnt;     /* A pivot fit has set the offset.        */
    bool  heading_gated;         /* Offset learnt, field in band and rates agree. */
    float pivot_scale;           /* Body over encoder yaw in a steady pivot,
                                    witness_min_scale..1; 0 for none.      */
} imu_motion_output_t;

typedef struct
{
    imu_motion_config_t cfg;
    uint8_t     fwd_axis;        /* Copied from hump_config_t.             */
    int8_t      fwd_sign;
    motion_class_t motion;
    motion_class_t candidate;
    uint16_t    candidate_count;
    bool        have_heading;
    float       last_heading_deg;
    float       since_mag_s;
    bool        mag_rate_valid;
    float       mag_rate_dps;
    uint16_t    disagree_count;
    /* Circle fit of the horizontal field during one pivot. */
    float       fit_sum[IMU_FIT_SUMS]; /* see learn_offset()             */
    uint16_t    fit_n;
    uint8_t     fit_axis[IMU_FIT_AXES]; /* The horizontal board axes.    */
    float       fit_yaw_deg;     /* Encoder yaw since the pivot began.     */
    float       fit_yaw_lo;
    float       fit_yaw_hi;
    bool        offset_learnt;
    float       witness_s;       /* Time in the current one-way pivot.     */
    int8_t      witness_dir;     /* Its direction, 0 when not pivoting.    */
} imu_motion_ctx_t;

/* Fill cfg with the unmeasured defaults and the chassis track width. */
void imu_motion_default_config (imu_motion_config_t * p_cfg);

/* Check cfg, copy the forward axis from the hump config and reset. Returns
 * false and leaves the context unchanged for a null pointer or a setting
 * out of range. */
bool imu_motion_init (imu_motion_ctx_t * p_ctx,
                      const imu_motion_config_t * p_cfg,
                      const hump_config_t * p_hump_cfg);

/* Process one sample after hump_update() succeeded on the same p_sample.
 * Returns false and leaves the context and output unchanged for a null
 * pointer, non-positive or non-finite dt, or a non-finite wheel speed
 * (when wheels_valid) or field value (when mag_new). */
bool imu_motion_update (imu_motion_ctx_t * p_ctx,
                        const hump_sample_t * p_sample,
                        const hump_output_t * p_hump,
                        const imu_motion_input_t * p_in,
                        imu_motion_output_t * p_out);

#endif /* IMU_MOTION_H */

/*** end of file ***/
