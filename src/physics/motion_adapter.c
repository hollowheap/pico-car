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

bool motion_adapter_convert (motion_adapter_t * p_ad, const motion_reading_t * p_reading,
                    uint32_t now_us, motion_input_t * p_out)
{
    uint32_t age_us = 0U;
    bool speeds_ok = false;
    bool odo_ok = false;
    float speed = 0.0f;

    if ((NULL == p_ad) || (NULL == p_out))
    {
        return false;
    }

    (void)memset (p_out, 0, sizeof (*p_out));

    if ((NULL == p_reading) || !p_reading->available)
    {
        p_ad->have_disp = false;
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
    odo_ok = p_reading->odometry_valid[PHYSICS_WHEEL_LEFT]
             && p_reading->odometry_valid[PHYSICS_WHEEL_RIGHT];

    p_out->v_left_mps = (float)p_reading->velocity_um_s[PHYSICS_WHEEL_LEFT] * UM_TO_M;
    p_out->v_right_mps = (float)p_reading->velocity_um_s[PHYSICS_WHEEL_RIGHT]
                         * UM_TO_M;
    p_out->wheels_valid = speeds_ok;
    speed = WHEEL_MEAN_WEIGHT * (p_out->v_left_mps + p_out->v_right_mps);
    p_out->speed_mps = speed;
    p_out->age_s = (float)age_us * US_TO_S;

    /* Distance since the previous call. */
    if (odo_ok)
    {
        if (p_ad->have_disp)
        {
            int64_t dl = p_reading->displacement_um[PHYSICS_WHEEL_LEFT]
                         - p_ad->prev_disp_um[PHYSICS_WHEEL_LEFT];
            int64_t dr = p_reading->displacement_um[PHYSICS_WHEEL_RIGHT]
                         - p_ad->prev_disp_um[PHYSICS_WHEEL_RIGHT];

            p_out->ds_m = WHEEL_MEAN_WEIGHT * ((float)dl + (float)dr) * UM_TO_M;
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

    /* dv/dt between the motion service samples. */
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
