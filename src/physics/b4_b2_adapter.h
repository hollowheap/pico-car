/** @file b4_b2_adapter.h
 *
 * @brief Convert Buddy 2's motion status into Buddy 4's SI inputs.
 *
 * b4_b2_reading_t mirrors the fields of Buddy 2's motion_sample_t
 * (pico-car/src/odometry.h), filled by b4_b2_glue.c on the car. Keeping
 * the conversion here makes it testable on the laptop.
 *
 * Rules:
 *   - valid needs a sample, both wheel speeds valid and both odometries
 *     valid, and a timestamp that is not in the future.
 *   - ds is the change in mean wheel displacement since the previous call,
 *     so it is exact however often Buddy 2 updates. It is zero on the first
 *     call and after odometry was invalid.
 *   - Buddy 2 does not publish dv/dt yet (PostTri1 planned accel_mms2).
 *     Until it does, dv/dt is the change in mean speed between Buddy 2
 *     sample timestamps, smoothed with an EMA. Encoder speeds are interval
 *     averages, so this estimate is noisy and lags; replace it with Buddy
 *     2's value once available.
 */

#ifndef B4_B2_ADAPTER_H
#define B4_B2_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "b4_pipeline.h"

#define B4_B2_LEFT  (0U)   /* Same as Buddy 2's ENCODER_LEFT.  */
#define B4_B2_RIGHT (1U)   /* Same as Buddy 2's ENCODER_RIGHT. */

typedef struct
{
    bool     available;          /* Buddy 2 has published a sample.        */
    uint32_t timestamp_us;
    int32_t  velocity_um_s[2];   /* Signed, positive car-forward.          */
    int64_t  displacement_um[2]; /* Signed, since Buddy 2's sampler init.  */
    bool     speed_valid[2];
    bool     odometry_valid[2];
} b4_b2_reading_t;

typedef struct
{
    float    accel_alpha;        /* EMA weight of a new dv/dt value.       */
    bool     have_disp;
    int64_t  prev_disp_um[2];
    bool     have_speed;
    uint32_t prev_ts_us;
    float    prev_speed_mps;
    bool     accel_valid;
    float    accel_mps2;
} b4_b2_adapter_t;

/* accel_alpha in (0, 1]. Returns false for a null pointer or bad alpha. */
bool b4_b2_adapter_init (b4_b2_adapter_t * p_ad, float accel_alpha);

/* p_reading may be NULL when Buddy 2 is not running; out is then invalid.
 * Returns false only for a null p_ad or p_out. */
bool b4_b2_convert (b4_b2_adapter_t * p_ad, const b4_b2_reading_t * p_reading,
                    uint32_t now_us, b4_motion_in_t * p_out);

#endif /* B4_B2_ADAPTER_H */

/*** end of file ***/
