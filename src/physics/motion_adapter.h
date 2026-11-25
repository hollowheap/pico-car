/** @file motion_adapter.h
 *
 * @brief Convert the motion service status into the physics SI inputs.
 *
 * motion_reading_t mirrors the fields of the motion sample
 * (pico-car/src/odometry.h), filled by motion_glue.c on the car. Keeping
 * the conversion here makes it testable on the laptop.
 *
 * Rules:
 *   - valid needs a sample, both wheel speeds valid and both odometries
 *     valid, and a timestamp that is not in the future.
 *   - ds is the change in mean wheel displacement since the previous call,
 *     so it is exact however often the motion service updates. It is zero on the first
 *     call and after odometry was invalid.
 *   - the motion service does not publish dv/dt yet (PostTri1 planned accel_mms2).
 *     Until it does, dv/dt is the change in mean speed between the motion service
 *     sample timestamps, smoothed with an EMA. Encoder speeds are interval
 *     averages, so this estimate is noisy and lags; replace it with Buddy
 *     2's value once available.
 */

#ifndef MOTION_ADAPTER_H
#define MOTION_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "pipeline.h"

#define PHYSICS_WHEEL_LEFT  (0U)   /* Same as the motion service's ENCODER_LEFT.  */
#define PHYSICS_WHEEL_RIGHT (1U)   /* Same as the motion service's ENCODER_RIGHT. */

typedef struct
{
    bool     available;          /* the motion service has published a sample.        */
    uint32_t timestamp_us;
    int32_t  velocity_um_s[2];   /* Signed, positive car-forward.          */
    int64_t  displacement_um[2]; /* Signed, since the motion service's sampler init.  */
    bool     speed_valid[2];
    bool     odometry_valid[2];
} motion_reading_t;

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
} motion_adapter_t;

/* accel_alpha in (0, 1]. Returns false for a null pointer or bad alpha. */
bool motion_adapter_init (motion_adapter_t * p_ad, float accel_alpha);

/* p_reading may be NULL when the motion service is not running; out is then invalid.
 * Returns false only for a null p_ad or p_out. */
bool motion_adapter_convert (motion_adapter_t * p_ad, const motion_reading_t * p_reading,
                    uint32_t now_us, motion_input_t * p_out);

#endif /* MOTION_ADAPTER_H */

/*** end of file ***/
