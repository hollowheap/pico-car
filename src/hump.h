/** @file hump.h
 *
 * @brief B4 hump detection + terrain monitor (gyro-less v1).
 *
 * Method (fits LSM303DLHC, no gyro): tilt pitch from accel low-pass
 * plus forward speed from B2. Height = integral(v * sin(pitch)) over
 * the hump window. This avoids double-integration drift and meets
 * NFR5 on small humps when pitch exceeds ~5 deg and speed is known
 * to ~0.03 m/s.
 *
 * Task: 20 ms periodic. Inputs accel-g + speed snapshot. Outputs
 * hump events + motion events, no blocking on WiFi.
 *
 * Test targets (Task 4): detection >=95% on 20/30/40 mm humps at
 * 0.15 m/s, false positives 0 on 5 m flat, peak error <=10 mm.
 */

#ifndef HUMP_H
#define HUMP_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_lsm303.h"

#define HUMP_PERIOD_MS            20u
#define HUMP_PITCH_ENTER_DEG      6.0f
#define HUMP_PITCH_EXIT_DEG       2.0f
#define HUMP_PITCH_DESCEND_DEG    -4.0f
#define HUMP_MIN_SPEED_MPS        0.05f
#define HUMP_DEBOUNCE_SAMPLES     3u
#define HUMP_FLAT_SETTLE_SAMPLES  10u
#define HUMP_LPF_ALPHA            0.25f
#define HUMP_IMPACT_DELTA_G       0.7f

typedef enum
{
    HUMP_FLAT = 0,
    HUMP_CLIMB,
    HUMP_CREST,
    HUMP_DESCEND
} hump_state_t;

typedef enum
{
    MOTION_UNKNOWN = 0,
    MOTION_STATIONARY,
    MOTION_MOVING_FLAT,
    MOTION_CLIMBING,
    MOTION_DESCENDING,
    MOTION_IMPACT
} motion_event_t;

typedef enum
{
    HUMP_FAULT_NONE = 0,
    HUMP_FAULT_I2C_NACK,
    HUMP_FAULT_STALE_DATA,
    HUMP_FAULT_INVALID_TILT
} hump_fault_t;

/* Minimal speed snapshot owned by B2; B4 holds a local copy. */
typedef struct
{
    float    forward_mps;
    float    distance_m;
    uint32_t timestamp_ms;
    uint32_t seq;
} speed_snapshot_t;

typedef struct
{
    bool          hump_active;
    bool          hump_finished;
    float         peak_height_mm;
    float         peak_pitch_deg;
    float         run_peak_height_mm;
    hump_state_t  state;
    motion_event_t motion;
    hump_fault_t  fault;
    uint32_t      fault_count;
} hump_result_t;

typedef struct
{
    float          filt_pitch_deg;
    float          window_height_mm;
    float          peak_pitch_deg;
    float          run_peak_mm;
    uint8_t        debounce;
    uint8_t        settle;
    hump_state_t   state;
    uint32_t       last_speed_seq;
    uint32_t       stale_count;
    hump_fault_t   fault;
    uint32_t       fault_count;
    imu_calib_t    calib;
} hump_ctx_t;

/*!
 * @brief Reset detector state for a new run (keeps calibration).
 *
 * @param[in] p_ctx Detector context.
 */
void hump_reset (hump_ctx_t * p_ctx);

/*!
 * @brief 20 ms update: filter pitch, run state machine, integrate height.
 *
 * Height math: dh = v * sin(pitch) * dt, accumulated while not FLAT.
 * Peak latched at window exit; run peak keeps highest hump this run.
 *
 * @param[in]  p_ctx   Detector context (holds filter + calibration).
 * @param[in]  p_accel Calibrated accel in g.
 * @param[in]  p_speed Latest B2 speed snapshot copy.
 * @param[in]  now_ms  Current time in ms.
 * @param[out] p_out   Result for Navigation + Telemetry copies.
 * @return true if inputs valid (false = fault latched, outputs safe).
 */
bool hump_update (hump_ctx_t * p_ctx, const imu_accel_g_t * p_accel,
                  const speed_snapshot_t * p_speed, uint32_t now_ms,
                  hump_result_t * p_out);

#endif /* HUMP_H */

/*** end of file ***/
