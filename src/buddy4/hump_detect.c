/** @file hump_detect.c
 *
 * @brief Hump detection and height estimation from accelerometer tilt.
 *
 * State machine: FLAT -> CLIMB -> CREST -> DESCEND -> FLAT.
 *
 * While in FLAT, height from samples that already exceed the enter threshold
 * is accumulated as pending. When the enter debounce completes, the pending
 * height is kept, so the debounce delay does not cut off the start of the
 * climb.
 *
 * The pitch formula atan2(fwd, sqrt(other1^2 + other2^2)) does not change
 * with roll about the forward axis, so subtracting theta_0 as an angle
 * corrects a mounting tilt about the side axis exactly.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hump_detect.h"

#define RAD_TO_DEG   (57.2957795f)
#define DEG_TO_RAD   (0.0174532925f)
#define STD_GRAVITY  (9.80665f)

static bool is_finite (float value)
{
    return isfinite (value) && (value <= 3.0e38f) && (value >= -3.0e38f);
}

static float axis_value (const float axes[3], uint8_t axis)
{
    return axes[(axis <= HUMP_AXIS_Z) ? axis : HUMP_AXIS_Z];
}

/* Board axes into an array, with the forward axis corrected for the car's
 * own acceleration when the motion data is usable. The correction is applied
 * in board units, so it is subtracted with the axis sign. */
static void corrected_axes (const hump_config_t * p_cfg,
                            const hump_sample_t * p_sample,
                            bool motion_fresh,
                            float axes[3])
{
    axes[HUMP_AXIS_X] = p_sample->ax_g;
    axes[HUMP_AXIS_Y] = p_sample->ay_g;
    axes[HUMP_AXIS_Z] = p_sample->az_g;

    if (motion_fresh)
    {
        axes[p_cfg->fwd_axis] -= (float)p_cfg->fwd_sign
                                 * (p_sample->accel_mps2 / STD_GRAVITY);
    }
}

static float pitch_from_axes (const hump_config_t * p_cfg, const float axes[3])
{
    float fwd = axis_value (axes, p_cfg->fwd_axis) * (float)p_cfg->fwd_sign;
    float other_sq = 0.0f;
    uint8_t axis;

    for (axis = HUMP_AXIS_X; axis <= HUMP_AXIS_Z; axis++)
    {
        if (axis != p_cfg->fwd_axis)
        {
            float value = axis_value (axes, axis);
            other_sq += value * value;
        }
    }

    return atan2f (fwd, sqrtf (other_sq)) * RAD_TO_DEG;
}

/* Push a raw pitch into the median window and return the median of the
 * samples held. Before the window is full, the median of what is held is
 * used (upper middle value for an even count). */
static float median_push (hump_ctx_t * p_ctx, float raw_deg)
{
    float sorted[HUMP_MEDIAN_MAX];
    uint8_t n;
    uint8_t i;
    uint8_t j;

    p_ctx->median_buf[p_ctx->median_next] = raw_deg;
    p_ctx->median_next++;
    if (p_ctx->median_next >= p_ctx->cfg.median_len)
    {
        p_ctx->median_next = 0U;
    }
    if (p_ctx->median_count < p_ctx->cfg.median_len)
    {
        p_ctx->median_count++;
    }

    n = p_ctx->median_count;
    for (i = 0U; i < n; i++)
    {
        sorted[i] = p_ctx->median_buf[i];
    }

    /* Insertion sort: at most 5 values. */
    for (i = 1U; i < n; i++)
    {
        float key = sorted[i];

        j = i;
        while ((j > 0U) && (sorted[j - 1U] > key))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = key;
    }

    return sorted[n / 2U];
}

static void reset_hump (hump_ctx_t * p_ctx)
{
    p_ctx->state = HUMP_STATE_FLAT;
    p_ctx->up_count = 0U;
    p_ctx->down_count = 0U;
    p_ctx->flat_count = 0U;
    p_ctx->hump_samples = 0U;
    p_ctx->pending_height_m = 0.0f;
    p_ctx->pending_peak_pitch_deg = 0.0f;
    p_ctx->pending_valid = true;
    p_ctx->height_m = 0.0f;
    p_ctx->peak_m = 0.0f;
    p_ctx->peak_pitch_deg = 0.0f;
    p_ctx->height_valid = true;
}

static void cal_restart (hump_ctx_t * p_ctx)
{
    p_ctx->cal_count = 0U;
    p_ctx->cal_sum[HUMP_AXIS_X] = 0.0f;
    p_ctx->cal_sum[HUMP_AXIS_Y] = 0.0f;
    p_ctx->cal_sum[HUMP_AXIS_Z] = 0.0f;
    p_ctx->cal_min_deg = 0.0f;
    p_ctx->cal_max_deg = 0.0f;
}

/* Add one sample to calibration. Returns true on the sample that completes
 * it. A sample that is not at rest restarts the count; if it still passes
 * the per-sample checks it becomes the first sample of the new window. */
static bool cal_add (hump_ctx_t * p_ctx, const float axes[3], float raw_deg,
                     float magnitude_g, bool moving)
{
    const hump_config_t * p_cfg = &p_ctx->cfg;
    bool sample_ok = !moving
                     && (fabsf (magnitude_g - 1.0f) <= p_cfg->cal_max_dev_g);
    bool done = false;

    if (!sample_ok)
    {
        cal_restart (p_ctx);
    }
    else
    {
        if (0U == p_ctx->cal_count)
        {
            p_ctx->cal_min_deg = raw_deg;
            p_ctx->cal_max_deg = raw_deg;
        }
        else
        {
            float new_min = (raw_deg < p_ctx->cal_min_deg) ? raw_deg
                                                            : p_ctx->cal_min_deg;
            float new_max = (raw_deg > p_ctx->cal_max_deg) ? raw_deg
                                                            : p_ctx->cal_max_deg;

            if ((new_max - new_min) > p_cfg->cal_max_spread_deg)
            {
                /* Car moved or was bumped: start again from this sample. */
                cal_restart (p_ctx);
                new_min = raw_deg;
                new_max = raw_deg;
            }
            p_ctx->cal_min_deg = new_min;
            p_ctx->cal_max_deg = new_max;
        }

        p_ctx->cal_sum[HUMP_AXIS_X] += axes[HUMP_AXIS_X];
        p_ctx->cal_sum[HUMP_AXIS_Y] += axes[HUMP_AXIS_Y];
        p_ctx->cal_sum[HUMP_AXIS_Z] += axes[HUMP_AXIS_Z];
        p_ctx->cal_count++;

        if (p_ctx->cal_count >= p_cfg->cal_samples)
        {
            /* theta_0 from the mean vector, which averages out noise better
             * than the mean of per-sample angles. Scale does not matter. */
            p_ctx->theta0_deg = pitch_from_axes (p_cfg, p_ctx->cal_sum);
            p_ctx->calibrated = true;
            done = true;
        }
    }

    return done;
}

void hump_default_config (hump_config_t * p_cfg)
{
    if (NULL != p_cfg)
    {
        p_cfg->fwd_axis = HUMP_AXIS_X;
        p_cfg->fwd_sign = 1;
        p_cfg->median_len = 5U;
        p_cfg->alpha = 0.75f;
        p_cfg->enter_deg = 6.0f;
        p_cfg->exit_band_deg = 2.5f;
        p_cfg->enter_samples = 3U;
        p_cfg->exit_samples = 10U;
        p_cfg->max_hump_samples = 500U;
        p_cfg->impact_g = 0.7f;
        p_cfg->min_speed_mps = 0.08f;
        p_cfg->max_motion_age_s = 0.025f;
        p_cfg->cal_samples = 50U;
        p_cfg->cal_max_dev_g = 0.05f;
        p_cfg->cal_max_spread_deg = 2.0f;
    }
}

bool hump_init (hump_ctx_t * p_ctx, const hump_config_t * p_cfg)
{
    bool ok = (NULL != p_ctx) && (NULL != p_cfg);

    if (ok)
    {
        ok = (p_cfg->fwd_axis <= HUMP_AXIS_Z)
             && ((1 == p_cfg->fwd_sign) || (-1 == p_cfg->fwd_sign))
             && (p_cfg->median_len >= 1U)
             && (p_cfg->median_len <= HUMP_MEDIAN_MAX)
             && (1U == (p_cfg->median_len % 2U))
             && is_finite (p_cfg->alpha) && (p_cfg->alpha > 0.0f)
             && (p_cfg->alpha <= 1.0f)
             && is_finite (p_cfg->enter_deg) && is_finite (p_cfg->exit_band_deg)
             && (p_cfg->exit_band_deg > 0.0f)
             && (p_cfg->enter_deg > p_cfg->exit_band_deg)
             && (p_cfg->enter_samples >= 1U) && (p_cfg->exit_samples >= 1U)
             && (p_cfg->max_hump_samples >= 1U)
             && is_finite (p_cfg->impact_g) && (p_cfg->impact_g > 0.0f)
             && is_finite (p_cfg->min_speed_mps)
             && (p_cfg->min_speed_mps >= 0.0f)
             && is_finite (p_cfg->max_motion_age_s)
             && (p_cfg->max_motion_age_s > 0.0f)
             && (p_cfg->cal_samples >= 1U)
             && is_finite (p_cfg->cal_max_dev_g)
             && (p_cfg->cal_max_dev_g > 0.0f)
             && is_finite (p_cfg->cal_max_spread_deg)
             && (p_cfg->cal_max_spread_deg > 0.0f);
    }

    if (ok)
    {
        p_ctx->cfg = *p_cfg;
        p_ctx->median_count = 0U;
        p_ctx->median_next = 0U;
        p_ctx->have_pitch = false;
        p_ctx->filt_deg = 0.0f;
        hump_recalibrate (p_ctx);
    }

    return ok;
}

void hump_recalibrate (hump_ctx_t * p_ctx)
{
    if (NULL != p_ctx)
    {
        p_ctx->calibrated = false;
        p_ctx->theta0_deg = 0.0f;
        cal_restart (p_ctx);
        reset_hump (p_ctx);
    }
}

bool hump_update (hump_ctx_t * p_ctx,
                  const hump_sample_t * p_sample,
                  hump_output_t * p_out)
{
    const hump_config_t * p_cfg;
    float axes[3];
    float raw;
    float med;
    float pitch;
    float abs_pitch;
    float magnitude;
    float delta_m;
    bool motion_fresh;
    bool speed_ok;
    bool finished = false;
    bool complete = false;

    if ((NULL == p_ctx) || (NULL == p_sample) || (NULL == p_out))
    {
        return false;
    }

    if (!is_finite (p_sample->ax_g) || !is_finite (p_sample->ay_g)
        || !is_finite (p_sample->az_g))
    {
        return false;
    }

    if (p_sample->motion_valid
        && (!is_finite (p_sample->motion_age_s)
            || (p_sample->motion_age_s < 0.0f)
            || !is_finite (p_sample->speed_mps)
            || !is_finite (p_sample->ds_m)
            || !is_finite (p_sample->accel_mps2)))
    {
        return false;
    }

    p_cfg = &p_ctx->cfg;

    motion_fresh = p_sample->motion_valid
                   && (p_sample->motion_age_s <= p_cfg->max_motion_age_s);

    /* Pitch: dv/dt correction, median, then EMA seeded by the first value. */
    corrected_axes (p_cfg, p_sample, motion_fresh, axes);
    raw = pitch_from_axes (p_cfg, axes);
    med = median_push (p_ctx, raw);
    if (p_ctx->have_pitch)
    {
        p_ctx->filt_deg += p_cfg->alpha * (med - p_ctx->filt_deg);
    }
    else
    {
        p_ctx->filt_deg = med;
        p_ctx->have_pitch = true;
    }

    magnitude = sqrtf ((p_sample->ax_g * p_sample->ax_g)
                       + (p_sample->ay_g * p_sample->ay_g)
                       + (p_sample->az_g * p_sample->az_g));

    p_out->event = HUMP_EVENT_NONE;

    if (!p_ctx->calibrated)
    {
        /* Moving if Buddy 2 reports speed. Without fresh motion data the
         * accelerometer checks alone decide. */
        bool moving = motion_fresh
                      && (fabsf (p_sample->speed_mps) >= p_cfg->min_speed_mps)
                      && (p_cfg->min_speed_mps > 0.0f);

        if (cal_add (p_ctx, axes, raw, magnitude, moving))
        {
            p_out->event = HUMP_EVENT_CAL_DONE;
        }
    }

    pitch = p_ctx->filt_deg - p_ctx->theta0_deg;
    abs_pitch = fabsf (pitch);

    /* Height step for this sample, if the motion data is usable. */
    speed_ok = motion_fresh && (p_sample->speed_mps >= p_cfg->min_speed_mps);
    delta_m = 0.0f;
    if (speed_ok)
    {
        delta_m = p_sample->ds_m * sinf (pitch * DEG_TO_RAD);
    }

    if (!p_ctx->calibrated)
    {
        /* No detection until theta_0 is known. */
    }
    else if (HUMP_STATE_FLAT == p_ctx->state)
    {
        if (pitch > p_cfg->enter_deg)
        {
            p_ctx->up_count++;
            p_ctx->pending_height_m += delta_m;
            p_ctx->pending_valid = p_ctx->pending_valid && speed_ok;
            if (pitch > p_ctx->pending_peak_pitch_deg)
            {
                p_ctx->pending_peak_pitch_deg = pitch;
            }

            if (p_ctx->up_count >= p_cfg->enter_samples)
            {
                p_ctx->state = HUMP_STATE_CLIMB;
                p_ctx->height_m = p_ctx->pending_height_m;
                p_ctx->peak_m = (p_ctx->height_m > 0.0f) ? p_ctx->height_m
                                                         : 0.0f;
                p_ctx->peak_pitch_deg = p_ctx->pending_peak_pitch_deg;
                p_ctx->height_valid = p_ctx->pending_valid;
                p_ctx->hump_samples = p_ctx->up_count;
                p_ctx->up_count = 0U;
                p_ctx->down_count = 0U;
                p_ctx->flat_count = 0U;
                p_out->event = HUMP_EVENT_STARTED;
            }
        }
        else
        {
            p_ctx->up_count = 0U;
            p_ctx->pending_height_m = 0.0f;
            p_ctx->pending_peak_pitch_deg = 0.0f;
            p_ctx->pending_valid = true;
        }
    }
    else
    {
        p_ctx->height_m += delta_m;
        p_ctx->height_valid = p_ctx->height_valid && speed_ok;
        if (p_ctx->height_m > p_ctx->peak_m)
        {
            p_ctx->peak_m = p_ctx->height_m;
        }
        if (pitch > p_ctx->peak_pitch_deg)
        {
            p_ctx->peak_pitch_deg = pitch;
        }
        p_ctx->hump_samples++;

        /* Count samples inside the flat band and samples steeply downhill. */
        if (abs_pitch <= p_cfg->exit_band_deg)
        {
            p_ctx->flat_count++;
        }
        else
        {
            p_ctx->flat_count = 0U;
        }

        if (pitch < -p_cfg->enter_deg)
        {
            p_ctx->down_count++;
        }
        else
        {
            p_ctx->down_count = 0U;
        }

        switch (p_ctx->state)
        {
            case HUMP_STATE_CLIMB:
                if (p_ctx->down_count >= p_cfg->enter_samples)
                {
                    p_ctx->state = HUMP_STATE_DESCEND;
                    p_ctx->flat_count = 0U;
                }
                else if (p_ctx->flat_count >= p_cfg->exit_samples)
                {
                    p_ctx->state = HUMP_STATE_CREST;
                    p_ctx->flat_count = 0U;
                    p_ctx->down_count = 0U;
                }
                else
                {
                    /* Still climbing. */
                }
                break;

            case HUMP_STATE_CREST:
                if (p_ctx->down_count >= p_cfg->enter_samples)
                {
                    p_ctx->state = HUMP_STATE_DESCEND;
                    p_ctx->flat_count = 0U;
                }
                break;

            case HUMP_STATE_DESCEND:
                if (p_ctx->flat_count >= p_cfg->exit_samples)
                {
                    finished = true;
                    complete = true;
                }
                break;

            default:
                break;
        }

        if (!finished && (p_ctx->hump_samples >= p_cfg->max_hump_samples))
        {
            finished = true;
            complete = false;
        }
    }

    /* Fill the output before a finished hump is cleared. */
    p_out->calibrated = p_ctx->calibrated;
    p_out->theta0_deg = p_ctx->theta0_deg;
    p_out->pitch_deg = pitch;
    p_out->height_m = p_ctx->height_m;
    p_out->peak_m = p_ctx->peak_m;
    p_out->peak_pitch_deg = p_ctx->peak_pitch_deg;
    p_out->height_valid = p_ctx->height_valid;
    p_out->complete = complete;
    p_out->motion_stale = !motion_fresh;

    if (finished)
    {
        p_out->event = HUMP_EVENT_FINISHED;
        reset_hump (p_ctx);
    }
    p_out->state = p_ctx->state;
    p_out->impact = (fabsf (magnitude - 1.0f) > p_cfg->impact_g);

    return true;
}

/*** end of file ***/
