/** @file hump_detect.h
 *
 * @brief Hump detection and height estimation from accelerometer tilt.
 *
 * Pure logic with no hardware access, following the SE01 PostTri1 design.
 * The caller supplies one accelerometer sample (in g, board axes) and the
 * latest motion data from Buddy 2 per call, every 20 ms (50 Hz).
 *
 * Processing per sample:
 *   1. Forward axis minus the car's own acceleration (dv/dt / 9.80665).
 *   2. Raw pitch = atan2(fwd, sqrt(other1^2 + other2^2)), nose-up positive.
 *   3. Median of the last median_len raw pitches, then an EMA.
 *   4. Subtract the rest pitch theta_0 measured during calibration.
 *   5. State machine FLAT -> CLIMB -> CREST -> DESCEND -> FLAT.
 *   6. Height = sum of ds * sin(pitch - theta_0), ds from encoder distance.
 *
 * Calibration: after init (or hump_recalibrate) the first cal_samples samples
 * taken at rest set theta_0, the rest gravity direction g0 and the rest
 * magnitude |g0|. Any sample that is not at rest restarts the count. No hump
 * is detected until calibration is done.
 *
 * Two detection modes (hump_config_t.mode):
 *   HUMP_MODE_PITCH: steps 5 and 6 above. For humps that both drive wheels
 *     climb, which tilt the car nose-up then nose-down.
 *   HUMP_MODE_TILT: for a hump under one wheel. Tilt = angle between the
 *     current gravity vector and g0, in any direction: median, EMA, then a
 *     tilt_avg_len moving average, because pushes and hand contact give
 *     0.1-0.3 s tilt spikes that would otherwise set the peak.
 *     FLAT -> CLIMB when tilt exceeds enter_deg, CLIMB -> DESCEND when tilt
 *     falls below half its peak, FINISHED after exit_samples inside the exit
 *     band. Height = lever_m * sin(tilt). lever_m is the distance from the
 *     lifted wheel to the line through the other two contact points, so
 *     height needs no Buddy 2 data and height_valid stays true. CREST is not
 *     used. A hump under both wheels is still detected, but its height is
 *     wrong because the car pivots about a different line.
 *
 * Entry gate (both modes): a sample whose dv/dt-corrected magnitude differs
 * from |g0| by more than entry_max_dev_g is a shock. It and the next
 * median_len + enter_samples samples cannot start a hump or raise the tilt
 * peak, so a knock does not look like a slope. With require_motion set, a
 * hump can also only start while Buddy 2 reports the car moving, so lifting
 * or tilting the car by hand is ignored.
 *
 * Limits: the accelerometer sees vibration and bumps as well as tilt. The
 * dv/dt correction only removes acceleration along the direction of travel.
 * theta_0 combines the mounting tilt and the sensor's zero-g offset on the
 * forward axis; it does not separate them. All thresholds are placeholders
 * until tuned on logged data from the real car.
 */

#ifndef HUMP_DETECT_H
#define HUMP_DETECT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    HUMP_STATE_FLAT = 0,
    HUMP_STATE_CLIMB,
    HUMP_STATE_CREST,
    HUMP_STATE_DESCEND
} hump_state_t;

typedef enum
{
    HUMP_EVENT_NONE = 0,
    HUMP_EVENT_CAL_DONE,         /* theta_0 measured; detection now active.  */
    HUMP_EVENT_STARTED,
    HUMP_EVENT_FINISHED
} hump_event_t;

/* Board axis indices for hump_config_t.fwd_axis. */
#define HUMP_AXIS_X (0U)
#define HUMP_AXIS_Y (1U)
#define HUMP_AXIS_Z (2U)

/* Values for hump_config_t.mode. */
#define HUMP_MODE_PITCH (0U)
#define HUMP_MODE_TILT  (1U)

/* Largest supported median window (odd). */
#define HUMP_MEDIAN_MAX (5U)

/* Largest tilt moving-average window, samples. */
#define HUMP_TILT_AVG_MAX (16U)

typedef struct
{
    uint8_t  mode;               /* HUMP_MODE_PITCH or HUMP_MODE_TILT.         */
    uint8_t  fwd_axis;           /* HUMP_AXIS_X, _Y or _Z: points along travel. */
    int8_t   fwd_sign;           /* +1 or -1: sign that makes nose-up positive. */
    uint8_t  median_len;         /* Median window: 1, 3 or 5 samples.          */
    float    alpha;              /* EMA weight of the new value, 0 < a <= 1.   */
    float    enter_deg;          /* Pitch (or tilt) that starts a slope.       */
    float    exit_band_deg;      /* Pitch (or tilt) magnitude counted as flat. */
    uint16_t enter_samples;      /* Consecutive samples above enter_deg.       */
    uint16_t exit_samples;       /* Consecutive samples inside exit band.      */
    uint32_t max_hump_samples;   /* Give up on a hump that lasts longer.       */
    float    impact_g;           /* | |a| - 1 g | above this sets impact.      */
    float    min_speed_mps;      /* Below this, height is marked invalid.      */
    float    max_motion_age_s;   /* Older motion data counts as stale.         */
    uint16_t cal_samples;        /* Rest samples needed to set theta_0.        */
    float    cal_max_dev_g;      /* At rest: | |a| - 1 g | at most this.       */
    float    cal_max_spread_deg; /* At rest: max - min raw pitch at most this. */
    float    lever_m;            /* Tilt mode: wheel to pivot line, metres.    */
    uint8_t  tilt_avg_len;       /* Tilt mode: moving average after the EMA,
                                    1..HUMP_TILT_AVG_MAX samples.             */
    float    entry_max_dev_g;    /* Entry gate: | |a| - |g0| | at most this.   */
    bool     require_motion;     /* Entry also needs fresh Buddy 2 data with
                                    |speed| >= min_speed_mps. Off when Buddy 2
                                    is not running (bring-up logger).        */
} hump_config_t;

typedef struct
{
    float ax_g;                  /* Board-axis acceleration in g.              */
    float ay_g;
    float az_g;
    /* Motion data from Buddy 2. Ignored when motion_valid is false. */
    bool  motion_valid;          /* False when Buddy 2 has no usable data.     */
    float motion_age_s;          /* Age of the motion data at this sample.     */
    float speed_mps;             /* Forward speed, signed.                     */
    float ds_m;                  /* Distance since the previous sample, signed. */
    float accel_mps2;            /* Forward dv/dt, signed.                     */
} hump_sample_t;

typedef struct
{
    hump_state_t state;
    hump_event_t event;          /* Set only on the call where it occurs.      */
    bool  calibrated;            /* theta_0 is set; detection is active.       */
    float theta0_deg;            /* Rest pitch subtracted from every sample.   */
    float pitch_deg;             /* Filtered pitch relative to theta_0.        */
    float tilt_deg;              /* Filtered tilt from g0, >= 0. 0 before
                                    calibration.                              */
    float peak_tilt_deg;         /* Tilt mode: highest tilt in this hump.      */
    float height_m;              /* Running height of the current hump. On
                                    FINISHED this is the end height, which
                                    should be near zero for a full hump.       */
    float peak_m;                /* Highest height_m reached in this hump.     */
    float peak_pitch_deg;        /* Highest pitch_deg reached in this hump.    */
    bool  height_valid;          /* False if any motion sample was unusable.   */
    bool  complete;              /* On FINISHED: full climb and descent seen.  */
    bool  motion_stale;          /* Motion data invalid or too old this call.  */
    bool  impact;                /* Shock detected on this sample.             */
} hump_output_t;

typedef struct
{
    float    buf[HUMP_MEDIAN_MAX];
    uint8_t  count;
    uint8_t  next;
} hump_median_t;

typedef struct
{
    hump_config_t cfg;

    /* Pitch filter. */
    hump_median_t pitch_med;
    bool     have_pitch;
    float    filt_deg;           /* Filtered absolute pitch (before theta_0). */

    /* Tilt filter (runs once calibrated). */
    hump_median_t tilt_med;
    bool     have_tilt;
    float    tilt_ema_deg;
    float    tilt_avg_buf[HUMP_TILT_AVG_MAX];
    uint8_t  tilt_avg_count;
    uint8_t  tilt_avg_next;
    float    tilt_deg;           /* Moving average of tilt_ema_deg.           */

    /* Calibration. */
    bool     calibrated;
    uint16_t cal_count;
    float    cal_sum[3];
    float    cal_min_deg;
    float    cal_max_deg;
    float    theta0_deg;
    float    g0_unit[3];         /* Rest gravity direction.                   */
    float    rest_mag_g;         /* Rest |a|.                                 */

    /* Entry gate. */
    uint16_t entry_block;        /* Samples left before entry is allowed.     */

    /* Hump tracking. */
    hump_state_t state;
    uint16_t up_count;
    uint16_t down_count;
    uint16_t flat_count;
    uint32_t hump_samples;
    float    pending_height_m;
    float    pending_peak_pitch_deg;
    bool     pending_valid;
    float    height_m;
    float    peak_m;
    float    peak_pitch_deg;
    float    pending_peak_tilt_deg;
    float    peak_tilt_deg;
    bool     height_valid;
} hump_ctx_t;

/* Fill cfg with the placeholder defaults from the design notes. */
void hump_default_config (hump_config_t * p_cfg);

/* Check cfg and reset the context, including calibration. Returns false and
 * leaves the context unchanged when an argument is null or a setting is out
 * of range. */
bool hump_init (hump_ctx_t * p_ctx, const hump_config_t * p_cfg);

/* Drop theta_0, abandon any hump in progress and start calibration again.
 * Call only while the car is at rest, e.g. at the start of each run. */
void hump_recalibrate (hump_ctx_t * p_ctx);

/* Process one sample. Returns false and leaves the context and output
 * unchanged for a null argument, a non-finite acceleration, or (when
 * motion_valid is true) a non-finite or negative-age motion field. */
bool hump_update (hump_ctx_t * p_ctx,
                  const hump_sample_t * p_sample,
                  hump_output_t * p_out);

#endif /* HUMP_DETECT_H */

/*** end of file ***/
