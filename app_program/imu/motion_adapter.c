/** @file motion_adapter.c
 *
 * @brief Convert the motion service status into physics's SI inputs.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "motion_adapter.h"
#include "pipeline.h"

#define UM_TO_M        (1.0e-6f)
#define US_TO_S        (1.0e-6f)
#define FUTURE_LIMIT      (0x80000000UL) /* now - ts beyond this: future. */
#define FUTURE_SLACK_US   (100U)         /* Allow a few us sampling jitter. */
#define WHEEL_MEAN_WEIGHT (0.5f)         /* Mean of the two wheels.       */

bool motion_adapter_init (motion_adapter_t * p_ad, float accel_alpha)
{
    if ((NULL == p_ad) || !isfinite (accel_alpha) || (accel_alpha <= 0.0f)
        || (accel_alpha > 1.0f))
    {
        return false;
    }

    (void)memset (p_ad, 0, sizeof (*p_ad));
    p_ad->accel_alpha = accel_alpha;
    return true;
}

/* The motion service samples twice per physics read, so a bad sample can
 * come and go between reads; its count of bad samples still moves. */
static bool invalid_samples_moved (motion_adapter_t * p_ad,
                                   const motion_reading_t * p_reading)
{
    bool moved = false;
    uint32_t wheel = 0U;

    for (wheel = 0U; wheel < PHYSICS_WHEEL_COUNT; wheel++)
    {
        moved = moved
                || (p_reading->invalid_samples[wheel] != p_ad->prev_invalid[wheel]);
        p_ad->prev_invalid[wheel] = p_reading->invalid_samples[wheel];
    }
    moved = moved && p_ad->have_invalid;
    p_ad->have_invalid = true;

    return moved;
}

/* Distance since the previous call; zero on the first call and after an
 * invalid one. */
static float distance_since_last (motion_adapter_t * p_ad,
                                  const motion_reading_t * p_reading, bool odo_ok)
{
    float ds_m = 0.0f;

    if (odo_ok)
    {
        if (p_ad->have_disp)
        {
            int64_t dl = p_reading->displacement_um[PHYSICS_WHEEL_LEFT]
                         - p_ad->prev_disp_um[PHYSICS_WHEEL_LEFT];
            int64_t dr = p_reading->displacement_um[PHYSICS_WHEEL_RIGHT]
                         - p_ad->prev_disp_um[PHYSICS_WHEEL_RIGHT];

            ds_m = WHEEL_MEAN_WEIGHT * ((float)dl + (float)dr) * UM_TO_M;
        }
        p_ad->prev_disp_um[PHYSICS_WHEEL_LEFT] = p_reading->displacement_um[PHYSICS_WHEEL_LEFT];
        p_ad->prev_disp_um[PHYSICS_WHEEL_RIGHT] =
            p_reading->displacement_um[PHYSICS_WHEEL_RIGHT];
        p_ad->have_disp = true;
    }
    else
    {
        p_ad->have_disp = false;
    }

    return ds_m;
}

/* Time bases: the age is this cycle's clock (now_us) minus the motion stamp;
 * dv/dt uses the motion stamps alone, because the speeds were measured then.
 * Both come from the same 1 MHz timer, and the hump detector drops a sample
 * older than its max_motion_age_s (25 ms), so speed and acceleration are never
 * paired with an accelerometer reading more than 25 ms apart. */
bool motion_adapter_convert (motion_adapter_t * p_ad, const motion_reading_t * p_reading,
                    uint32_t now_us, motion_input_t * p_out)
{
    uint32_t age_us = 0U;
    bool speeds_ok = false;
    bool odo_ok = false;
    bool glitch = false;
    float speed = 0.0f;

    if ((NULL == p_ad) || (NULL == p_out))
    {
        return false;
    }

    (void)memset (p_out, 0, sizeof (*p_out));

    if ((NULL == p_reading) || !p_reading->available)
    {
        p_ad->have_disp = false;
        p_ad->have_invalid = false;
        p_ad->have_speed = false;
        p_ad->accel_valid = false;
        return true;
    }

    age_us = now_us - p_reading->timestamp_us;   /* Wrap-safe. */
    if ((age_us > FUTURE_LIMIT) && ((0U - age_us) <= FUTURE_SLACK_US))
    {
        /* The motion task can sample a few microseconds after this cycle's
         * clock read. Treat that as fresh, not as a 71-minute-old sample. */
        age_us = 0U;
    }
    speeds_ok = p_reading->speed_valid[PHYSICS_WHEEL_LEFT]
                && p_reading->speed_valid[PHYSICS_WHEEL_RIGHT];
    glitch = invalid_samples_moved (p_ad, p_reading);
    odo_ok = p_reading->odometry_valid[PHYSICS_WHEEL_LEFT]
             && p_reading->odometry_valid[PHYSICS_WHEEL_RIGHT] && !glitch;

    p_out->v_left_mps = (float)p_reading->velocity_um_s[PHYSICS_WHEEL_LEFT] * UM_TO_M;
    p_out->v_right_mps = (float)p_reading->velocity_um_s[PHYSICS_WHEEL_RIGHT]
                         * UM_TO_M;
    p_out->wheels_valid = speeds_ok;
    /* Physics samples every other motion tick. A zero speed sample must
     * cover the interval back to the last observation. */
    p_out->wheels_moved = (0U == p_reading->stationary_span_us) || (p_ad->have_speed &&
        ((p_reading->timestamp_us - p_ad->prev_ts_us) > p_reading->stationary_span_us));
    speed = WHEEL_MEAN_WEIGHT * (p_out->v_left_mps + p_out->v_right_mps);
    p_out->speed_mps = speed;
    p_out->age_s = (float)age_us * US_TO_S;

    p_out->ds_m = distance_since_last (p_ad, p_reading, odo_ok);

    /* dv/dt between the motion service samples, on their own stamps. */
    if (speeds_ok)
    {
        if (p_ad->have_speed && (p_reading->timestamp_us != p_ad->prev_ts_us))
        {
            float dt = (float)(p_reading->timestamp_us - p_ad->prev_ts_us)
                       * US_TO_S;
            float raw = (speed - p_ad->prev_speed_mps) / dt;

            if (p_ad->accel_valid)
            {
                p_ad->accel_mps2 += p_ad->accel_alpha * (raw - p_ad->accel_mps2);
            }
            else
            {
                p_ad->accel_mps2 = raw;
                p_ad->accel_valid = true;
            }
        }
        if (!p_ad->have_speed || (p_reading->timestamp_us != p_ad->prev_ts_us))
        {
            p_ad->prev_ts_us = p_reading->timestamp_us;
            p_ad->prev_speed_mps = speed;
            p_ad->have_speed = true;
        }
    }
    else
    {
        p_ad->have_speed = false;
        p_ad->accel_valid = false;
    }
    p_out->accel_mps2 = p_ad->accel_valid ? p_ad->accel_mps2 : 0.0f;

    p_out->valid = speeds_ok && odo_ok && (age_us < FUTURE_LIMIT);

    return true;
}

/*** end of file ***/
