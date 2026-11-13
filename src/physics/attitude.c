/** @file attitude.c
 *
 * @brief Lift and overturn detection. See attitude.h.
 */

#include <math.h>
#include <stddef.h>

#include "attitude.h"

#define RAD_TO_DEG (57.2957795f)

#define DEFAULT_TILT_ENVELOPE_DEG (20.0f)
#define DEFAULT_LIFT_DEV_G        (0.12f)
#define DEFAULT_LIFT_DEV_SAMPLES  (3U)
#define DEFAULT_LIFT_HOLD_SAMPLES (10U)
#define DEFAULT_LIFT_SAMPLES      (15U)
#define DEFAULT_CANDIDATE_CLEAR   (10U)
#define DEFAULT_OVERTURN_TILT_DEG (100.0f)
#define DEFAULT_OVERTURN_ROLL_DEG (60.0f)
#define DEFAULT_OVERTURN_SAMPLES  (10U)
#define DEFAULT_CLEAR_TILT_DEG    (5.0f)
#define DEFAULT_CLEAR_DEV_G       (0.05f)
#define DEFAULT_CLEAR_SAMPLES     (50U)

static float dot3 (const float * p_a, const float * p_b)
{
    return (p_a[ATTITUDE_AXIS_X] * p_b[ATTITUDE_AXIS_X]) +
           (p_a[ATTITUDE_AXIS_Y] * p_b[ATTITUDE_AXIS_Y]) +
           (p_a[ATTITUDE_AXIS_Z] * p_b[ATTITUDE_AXIS_Z]);
}

static float clampf (float value, float low, float high)
{
    float result = value;

    if (result < low)
    {
        result = low;
    }
    else if (result > high)
    {
        result = high;
    }
    else
    {
        /* Within range. */
    }

    return result;
}

void attitude_default_config (attitude_config_t * p_cfg)
{
    if (NULL != p_cfg)
    {
        p_cfg->tilt_envelope_deg = DEFAULT_TILT_ENVELOPE_DEG;
        p_cfg->lift_dev_g        = DEFAULT_LIFT_DEV_G;
        p_cfg->lift_dev_samples  = DEFAULT_LIFT_DEV_SAMPLES;
        p_cfg->lift_hold_samples = DEFAULT_LIFT_HOLD_SAMPLES;
        p_cfg->lift_samples      = DEFAULT_LIFT_SAMPLES;
        p_cfg->candidate_clear   = DEFAULT_CANDIDATE_CLEAR;
        p_cfg->overturn_tilt_deg = DEFAULT_OVERTURN_TILT_DEG;
        p_cfg->overturn_roll_deg = DEFAULT_OVERTURN_ROLL_DEG;
        p_cfg->overturn_samples  = DEFAULT_OVERTURN_SAMPLES;
        p_cfg->clear_tilt_deg    = DEFAULT_CLEAR_TILT_DEG;
        p_cfg->clear_dev_g       = DEFAULT_CLEAR_DEV_G;
        p_cfg->clear_samples     = DEFAULT_CLEAR_SAMPLES;
    }
}

static bool config_ok (const attitude_config_t * p_cfg)
{
    return (p_cfg->tilt_envelope_deg > 0.0f) && (p_cfg->lift_dev_g > 0.0f) &&
           (p_cfg->lift_dev_samples >= 1U) && (p_cfg->lift_hold_samples >= 1U) &&
           (p_cfg->lift_samples >= 1U) && (p_cfg->candidate_clear >= 1U) &&
           (p_cfg->overturn_tilt_deg > p_cfg->tilt_envelope_deg) &&
           (p_cfg->overturn_roll_deg > 0.0f) && (p_cfg->overturn_samples >= 1U) &&
           (p_cfg->clear_tilt_deg > 0.0f) && (p_cfg->clear_dev_g > 0.0f) &&
           (p_cfg->clear_samples >= 1U);
}

void attitude_reset (attitude_ctx_t * p_ctx)
{
    if (NULL != p_ctx)
    {
        p_ctx->state          = ATTITUDE_OK;
        p_ctx->dev_run        = 0U;
        p_ctx->dev_hold       = 0U;
        p_ctx->cue_count      = 0U;
        p_ctx->no_cue_count   = 0U;
        p_ctx->overturn_count = 0U;
        p_ctx->clear_count    = 0U;
        p_ctx->was_calibrated = false;
    }
}

bool attitude_init (attitude_ctx_t * p_ctx, const attitude_config_t * p_cfg)
{
    bool b_ok = (NULL != p_ctx) && (NULL != p_cfg) && config_ok (p_cfg);

    if (b_ok)
    {
        p_ctx->cfg = *p_cfg;
        attitude_reset (p_ctx);
    }

    return b_ok;
}

static bool input_ok (const attitude_input_t * p_in)
{
    uint8_t idx = 0U;
    bool    b_ok = isfinite (p_in->ax_g) && isfinite (p_in->ay_g) && isfinite (p_in->az_g);

    for (idx = 0U; idx < ATTITUDE_AXES; idx++)
    {
        b_ok = b_ok && isfinite (p_in->g0[idx]) && isfinite (p_in->fwd[idx]);
    }

    return b_ok;
}

/* Tilt, roll and magnitude from the sample and the rest gravity vector. */
static void measure (const attitude_input_t * p_in, attitude_output_t * p_out)
{
    float const a[ATTITUDE_AXES] = { p_in->ax_g, p_in->ay_g, p_in->az_g };
    float const amag = sqrtf (dot3 (a, a));
    float const gmag = sqrtf (dot3 (p_in->g0, p_in->g0));

    p_out->amag_g    = amag;
    p_out->tilt_deg  = 0.0f;
    p_out->roll_deg  = 0.0f;

    if ((amag > 0.0f) && (gmag > 0.0f))
    {
        float g_hat[ATTITUDE_AXES];
        float lat[ATTITUDE_AXES];
        uint8_t axis = 0U;

        for (axis = 0U; axis < ATTITUDE_AXES; axis++)
        {
            g_hat[axis] = p_in->g0[axis] / gmag;
        }
        /* Lateral axis: g_hat x forward. */
        lat[ATTITUDE_AXIS_X] = (g_hat[ATTITUDE_AXIS_Y] * p_in->fwd[ATTITUDE_AXIS_Z]) -
                               (g_hat[ATTITUDE_AXIS_Z] * p_in->fwd[ATTITUDE_AXIS_Y]);
        lat[ATTITUDE_AXIS_Y] = (g_hat[ATTITUDE_AXIS_Z] * p_in->fwd[ATTITUDE_AXIS_X]) -
                               (g_hat[ATTITUDE_AXIS_X] * p_in->fwd[ATTITUDE_AXIS_Z]);
        lat[ATTITUDE_AXIS_Z] = (g_hat[ATTITUDE_AXIS_X] * p_in->fwd[ATTITUDE_AXIS_Y]) -
                               (g_hat[ATTITUDE_AXIS_Y] * p_in->fwd[ATTITUDE_AXIS_X]);
        float const lat_mag = sqrtf (dot3 (lat, lat));
        float const along   = dot3 (a, g_hat);

        p_out->tilt_deg = acosf (clampf (along / amag, -1.0f, 1.0f)) * RAD_TO_DEG;
        if (lat_mag > 0.0f)
        {
            for (axis = 0U; axis < ATTITUDE_AXES; axis++)
            {
                lat[axis] /= lat_mag;
            }
            p_out->roll_deg = atan2f (dot3 (a, lat), along) * RAD_TO_DEG;
        }
    }
}

static void count_overturn (attitude_ctx_t * p_ctx, const attitude_output_t * p_out)
{
    const attitude_config_t * p_cfg = &p_ctx->cfg;

    if ((p_out->tilt_deg > p_cfg->overturn_tilt_deg) ||
        (fabsf (p_out->roll_deg) > p_cfg->overturn_roll_deg))
    {
        if (p_ctx->overturn_count < UINT16_MAX)
        {
            p_ctx->overturn_count++;
        }
    }
    else
    {
        p_ctx->overturn_count = 0U;
    }
}

/* The unsettled cue: |amag - 1| over the limit for lift_dev_samples in a row,
 * held for lift_hold_samples after the last such sample. */
static bool unsettled_cue (attitude_ctx_t * p_ctx, const attitude_output_t * p_out)
{
    const attitude_config_t * p_cfg = &p_ctx->cfg;

    if (fabsf (p_out->amag_g - 1.0f) > p_cfg->lift_dev_g)
    {
        if (p_ctx->dev_run < UINT16_MAX)
        {
            p_ctx->dev_run++;
        }
    }
    else
    {
        p_ctx->dev_run = 0U;
    }
    if (p_ctx->dev_run >= p_cfg->lift_dev_samples)
    {
        p_ctx->dev_hold = p_cfg->lift_hold_samples;
    }
    else if (p_ctx->dev_hold > 0U)
    {
        p_ctx->dev_hold--;
    }
    else
    {
        /* Settled. */
    }

    return (p_ctx->dev_hold > 0U);
}

/* b_wheels_free is the second cue (D161): the wheels turn on target with little
 * duty. It raises CANDIDATE and keeps it, but only the unsettled cue or an
 * envelope breach can make LIFTED. */
static void step_lift (attitude_ctx_t * p_ctx, const attitude_output_t * p_out,
                       bool b_wheels_free)
{
    const attitude_config_t * p_cfg = &p_ctx->cfg;
    bool const b_breach = (p_out->tilt_deg > p_cfg->tilt_envelope_deg);
    bool const b_cue    = unsettled_cue (p_ctx, p_out) || b_breach;
    bool const b_level  = (p_out->tilt_deg < p_cfg->clear_tilt_deg) &&
                          (fabsf (p_out->amag_g - 1.0f) < p_cfg->clear_dev_g);

    switch (p_ctx->state)
    {
        case ATTITUDE_OK:
            if (b_cue || b_wheels_free)
            {
                p_ctx->state        = ATTITUDE_CANDIDATE;
                p_ctx->cue_count    = b_cue ? 1U : 0U;
                p_ctx->no_cue_count = 0U;
            }
        break;

        case ATTITUDE_CANDIDATE:
            if (b_cue)
            {
                p_ctx->no_cue_count = 0U;
                p_ctx->cue_count++;
                if (p_ctx->cue_count >= p_cfg->lift_samples)
                {
                    p_ctx->state       = ATTITUDE_LIFTED;
                    p_ctx->clear_count = 0U;
                }
            }
            else if (b_wheels_free)
            {
                p_ctx->no_cue_count = 0U;
            }
            else
            {
                p_ctx->no_cue_count++;
                if (p_ctx->no_cue_count >= p_cfg->candidate_clear)
                {
                    p_ctx->state     = ATTITUDE_OK;
                    p_ctx->cue_count = 0U;
                }
            }
        break;

        case ATTITUDE_LIFTED:
        case ATTITUDE_OVERTURNED:
            /* The supervisor keeps the motor hold latched after set-down. */
            if (b_level)
            {
                p_ctx->clear_count++;
                if (p_ctx->clear_count >= p_cfg->clear_samples)
                {
                    p_ctx->state     = ATTITUDE_OK;
                    p_ctx->cue_count = 0U;
                }
            }
            else
            {
                p_ctx->clear_count = 0U;
            }
        break;

        default:
        break;
    }
}

bool attitude_update (attitude_ctx_t * p_ctx, const attitude_input_t * p_in,
                      attitude_output_t * p_out)
{
    attitude_state_t previous = ATTITUDE_OK;

    if ((NULL == p_ctx) || (NULL == p_in) || (NULL == p_out) || !input_ok (p_in))
    {
        return false;
    }

    previous = p_ctx->state;
    p_out->calibrated = p_in->calibrated;
    p_out->tilt_deg   = 0.0f;
    p_out->roll_deg   = 0.0f;
    p_out->amag_g     = 0.0f;
    p_out->az_g       = p_in->az_g;

    if (!p_in->calibrated)
    {
        /* No reference vector: report the acceleration only. */
        float const a[ATTITUDE_AXES] = { p_in->ax_g, p_in->ay_g, p_in->az_g };

        attitude_reset (p_ctx);
        p_out->amag_g = sqrtf (dot3 (a, a));
    }
    else
    {
        measure (p_in, p_out);
        p_ctx->was_calibrated = true;
        count_overturn (p_ctx, p_out);
        if (p_ctx->overturn_count >= p_ctx->cfg.overturn_samples)
        {
            p_ctx->state = ATTITUDE_OVERTURNED;
            p_ctx->clear_count = 0U;
        }
        else
        {
            step_lift (p_ctx, p_out, p_in->b_wheels_free);
        }
    }

    p_out->state   = p_ctx->state;
    p_out->changed = (p_ctx->state != previous);

    return true;
}

/*** end of file ***/
