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

/* Push a value into a median window of len samples and return the median of
 * the samples held. Before the window is full, the median of what is held is
 * used (upper middle value for an even count). */
static float median_push (hump_median_t * p_med, uint8_t len, float value)
{
    float sorted[HUMP_MEDIAN_MAX];
    uint8_t n;
    uint8_t i;
    uint8_t j;

    p_med->buf[p_med->next] = value;
    p_med->next++;
    if (p_med->next >= len)
    {
        p_med->next = 0U;
    }
    if (p_med->count < len)
    {
        p_med->count++;
    }

    n = p_med->count;
    for (i = 0U; i < n; i++)
    {
        sorted[i] = p_med->buf[i];
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

static float vec_norm (const float v[3])
{
    return sqrtf ((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

/* Angle between axes and the rest direction g0_unit, in degrees, 0..180.
 * atan2 of |cross| and dot stays accurate at small angles. */
static float tilt_from_axes (const hump_ctx_t * p_ctx, const float axes[3])
{
    const float * g0 = p_ctx->g0_unit;
    float cross[3];

    cross[0] = (axes[1] * g0[2]) - (axes[2] * g0[1]);
    cross[1] = (axes[2] * g0[0]) - (axes[0] * g0[2]);
    cross[2] = (axes[0] * g0[1]) - (axes[1] * g0[0]);

    return atan2f (vec_norm (cross),
                   (axes[0] * g0[0]) + (axes[1] * g0[1]) + (axes[2] * g0[2]))
           * RAD_TO_DEG;
}

/* Median then EMA, seeded by the first value. */
static float filter_step (hump_median_t * p_med, bool * p_have, float * p_filt,
                          const hump_config_t * p_cfg, float raw)
{
    float med = median_push (p_med, p_cfg->median_len, raw);

    if (*p_have)
    {
        *p_filt += p_cfg->alpha * (med - *p_filt);
    }
    else
    {
        *p_filt = med;
        *p_have = true;
    }

    return *p_filt;
}

/* Moving average of the last tilt_avg_len EMA tilts. Summed fresh each call
 * (at most 16 adds), so no running-sum float error builds up. */
static float tilt_avg_push (hump_ctx_t * p_ctx, float value)
{
    uint8_t len = p_ctx->cfg.tilt_avg_len;
    float sum = 0.0f;
    uint8_t i;

    p_ctx->tilt_avg_buf[p_ctx->tilt_avg_next] = value;
    p_ctx->tilt_avg_next++;
    if (p_ctx->tilt_avg_count < len)
    {
        p_ctx->tilt_avg_count++;
    }
    if (p_ctx->tilt_avg_next >= len)
    {
        p_ctx->tilt_avg_next = 0U;
    }

    for (i = 0U; i < p_ctx->tilt_avg_count; i++)
    {
        sum += p_ctx->tilt_avg_buf[i];
    }

    return sum / (float)p_ctx->tilt_avg_count;
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
    p_ctx->pending_peak_tilt_deg = 0.0f;
    p_ctx->peak_tilt_deg = 0.0f;
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
            {
                float sum_norm = vec_norm (p_ctx->cal_sum);
                uint8_t axis;

                /* sum_norm > 0: every sample passed the |a| check and they
                 * point within cal_max_spread_deg of each other. */
                for (axis = HUMP_AXIS_X; axis <= HUMP_AXIS_Z; axis++)
                {
                    p_ctx->g0_unit[axis] = p_ctx->cal_sum[axis] / sum_norm;
                }
                p_ctx->rest_mag_g = sum_norm / (float)p_ctx->cal_count;
            }
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
        p_cfg->mode = HUMP_MODE_PITCH;
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
        /* 0.10 g: this LSM303 reads 1.03-1.06 g at rest (car days 1 and 2),
         * which is inside its zero-g offset spec. 0.05 g blocked calibration. */
        p_cfg->cal_max_dev_g = 0.10f;
        p_cfg->cal_max_spread_deg = 2.0f;
        /* Team car: W = 112 mm, rear axle to caster L = 82 mm, so the right
         * rear wheel is W*L / sqrt(L^2 + (W/2)^2) = 92.5 mm from the line
         * through the left wheel and the caster. */
        p_cfg->lever_m = 0.0925f;
        p_cfg->tilt_avg_len = 10U;
        p_cfg->entry_max_dev_g = 0.05f;
        p_cfg->require_motion = false;
    }
}

bool hump_init (hump_ctx_t * p_ctx, const hump_config_t * p_cfg)
{
    bool ok = (NULL != p_ctx) && (NULL != p_cfg);

    if (ok)
    {
        ok = (p_cfg->mode <= HUMP_MODE_TILT)
             && (p_cfg->fwd_axis <= HUMP_AXIS_Z)
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
             && (p_cfg->cal_max_spread_deg > 0.0f)
             && is_finite (p_cfg->lever_m) && (p_cfg->lever_m > 0.0f)
             && (p_cfg->tilt_avg_len >= 1U)
             && (p_cfg->tilt_avg_len <= HUMP_TILT_AVG_MAX)
             && is_finite (p_cfg->entry_max_dev_g)
             && (p_cfg->entry_max_dev_g > 0.0f);
    }

    if (ok)
    {
        p_ctx->cfg = *p_cfg;
        p_ctx->pitch_med.count = 0U;
        p_ctx->pitch_med.next = 0U;
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
        p_ctx->g0_unit[HUMP_AXIS_X] = 0.0f;
        p_ctx->g0_unit[HUMP_AXIS_Y] = 0.0f;
        p_ctx->g0_unit[HUMP_AXIS_Z] = 1.0f;
        p_ctx->rest_mag_g = 1.0f;
        p_ctx->tilt_med.count = 0U;
        p_ctx->tilt_med.next = 0U;
        p_ctx->have_tilt = false;
        p_ctx->tilt_ema_deg = 0.0f;
        p_ctx->tilt_avg_count = 0U;
        p_ctx->tilt_avg_next = 0U;
        p_ctx->tilt_deg = 0.0f;
        p_ctx->entry_block = 0U;
        cal_restart (p_ctx);
        reset_hump (p_ctx);
    }
}

/* One tilt-mode step after calibration. entry_ok allows a hump to start;
 * gate_ok is false for shock samples and the samples just after them, which
 * cannot raise the peak. Returns true when the hump finishes and sets
 * *p_complete. */
static bool tilt_step (hump_ctx_t * p_ctx, float tilt, bool entry_ok,
                       bool gate_ok, bool * p_complete, hump_event_t * p_event)
{
    const hump_config_t * p_cfg = &p_ctx->cfg;
    bool finished = false;

    if (HUMP_STATE_FLAT == p_ctx->state)
    {
        if (entry_ok && (tilt > p_cfg->enter_deg))
        {
            p_ctx->up_count++;
            if (tilt > p_ctx->pending_peak_tilt_deg)
            {
                p_ctx->pending_peak_tilt_deg = tilt;
            }
            if (p_ctx->up_count >= p_cfg->enter_samples)
            {
                p_ctx->state = HUMP_STATE_CLIMB;
                p_ctx->peak_tilt_deg = p_ctx->pending_peak_tilt_deg;
                p_ctx->height_valid = true;
                p_ctx->hump_samples = p_ctx->up_count;
                p_ctx->up_count = 0U;
                p_ctx->flat_count = 0U;
                *p_event = HUMP_EVENT_STARTED;
            }
        }
        else
        {
            p_ctx->up_count = 0U;
            p_ctx->pending_peak_tilt_deg = 0.0f;
        }
    }
    else
    {
        p_ctx->hump_samples++;
        if (gate_ok && (tilt > p_ctx->peak_tilt_deg))
        {
            p_ctx->peak_tilt_deg = tilt;
        }

        if (tilt <= p_cfg->exit_band_deg)
        {
            p_ctx->flat_count++;
        }
        else
        {
            p_ctx->flat_count = 0U;
        }

        if ((HUMP_STATE_CLIMB == p_ctx->state)
            && (tilt < (0.5f * p_ctx->peak_tilt_deg)))
        {
            p_ctx->state = HUMP_STATE_DESCEND;
        }

        if (p_ctx->flat_count >= p_cfg->exit_samples)
        {
            finished = true;
            *p_complete = true;
        }
        else if (p_ctx->hump_samples >= p_cfg->max_hump_samples)
        {
            finished = true;
            *p_complete = false;
        }
        else
        {
            /* Hump continues. */
        }
    }

    if (HUMP_STATE_FLAT != p_ctx->state)
    {
        p_ctx->height_m = p_cfg->lever_m * sinf (tilt * DEG_TO_RAD);
        p_ctx->peak_m = p_cfg->lever_m
                        * sinf (p_ctx->peak_tilt_deg * DEG_TO_RAD);
    }

    return finished;
}

bool hump_update (hump_ctx_t * p_ctx,
                  const hump_sample_t * p_sample,
                  hump_output_t * p_out)
{
    const hump_config_t * p_cfg;
    float axes[3];
    float raw;
    float pitch;
    float abs_pitch;
    float tilt = 0.0f;
    float magnitude;
    float delta_m;
    bool motion_fresh;
    bool speed_ok;
    bool gate_ok = false;
    bool entry_ok;
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
    (void)filter_step (&p_ctx->pitch_med, &p_ctx->have_pitch, &p_ctx->filt_deg,
                       p_cfg, raw);

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

    if (p_ctx->calibrated)
    {
        /* Tilt from g0, and the entry gate on the corrected magnitude. */
        bool quiet = fabsf (vec_norm (axes) - p_ctx->rest_mag_g)
                     <= p_cfg->entry_max_dev_g;

        tilt = filter_step (&p_ctx->tilt_med, &p_ctx->have_tilt,
                            &p_ctx->tilt_ema_deg, p_cfg,
                            tilt_from_axes (p_ctx, axes));
        tilt = tilt_avg_push (p_ctx, tilt);
        p_ctx->tilt_deg = tilt;

        gate_ok = quiet && (0U == p_ctx->entry_block);
        if (!quiet)
        {
            p_ctx->entry_block = (uint16_t)(p_cfg->median_len
                                            + p_cfg->enter_samples);
        }
        else if (p_ctx->entry_block > 0U)
        {
            p_ctx->entry_block--;
        }
        else
        {
            /* Gate open. */
        }
    }

    /* Height step for this sample, if the motion data is usable. */
    speed_ok = motion_fresh && (p_sample->speed_mps >= p_cfg->min_speed_mps);
    delta_m = 0.0f;
    if (speed_ok)
    {
        delta_m = p_sample->ds_m * sinf (pitch * DEG_TO_RAD);
    }

    /* Entry needs the shock gate open and, if configured, a moving car. */
    entry_ok = gate_ok
               && (!p_cfg->require_motion
                   || (motion_fresh
                       && (fabsf (p_sample->speed_mps)
                           >= p_cfg->min_speed_mps)));

    if (!p_ctx->calibrated)
    {
        /* No detection until theta_0 is known. */
    }
    else if (HUMP_MODE_TILT == p_cfg->mode)
    {
        finished = tilt_step (p_ctx, tilt, entry_ok, gate_ok, &complete,
                              &p_out->event);
        if ((HUMP_STATE_FLAT != p_ctx->state) || finished)
        {
            if (pitch > p_ctx->peak_pitch_deg)
            {
                p_ctx->peak_pitch_deg = pitch;
            }
        }
    }
    else if (HUMP_STATE_FLAT == p_ctx->state)
    {
        if (entry_ok && (pitch > p_cfg->enter_deg))
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
    p_out->tilt_deg = tilt;
    p_out->peak_tilt_deg = p_ctx->peak_tilt_deg;
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
