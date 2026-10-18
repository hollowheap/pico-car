/** @file hump.c
 *
 * @brief Gyro-less hump detector: pitch state machine + v*sin(pitch).
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hump.h"
#include "imu_lsm303.h"

#define DEG_TO_RAD 0.01745329f
#define MS_TO_S    0.001f

void hump_reset (hump_ctx_t * p_ctx)
{
    if (p_ctx == NULL)
    {
        return;
    }

    p_ctx->filt_pitch_deg  = 0.0f;
    p_ctx->window_height_mm = 0.0f;
    p_ctx->peak_pitch_deg  = 0.0f;
    p_ctx->debounce        = 0u;
    p_ctx->settle          = 0u;
    p_ctx->state           = HUMP_FLAT;
    p_ctx->stale_count     = 0u;
    p_ctx->fault           = HUMP_FAULT_NONE;
}

static float low_pass (float prev, float fresh)
{
    return prev + (HUMP_LPF_ALPHA * (fresh - prev));
}

bool hump_update (hump_ctx_t * p_ctx, const imu_accel_g_t * p_accel,
                  const speed_snapshot_t * p_speed, uint32_t now_ms,
                  hump_result_t * p_out)
{
    (void) now_ms;

    float raw_pitch = 0.0f;
    float mag       = 0.0f;
    float dh_mm     = 0.0f;

    if ((p_ctx == NULL) || (p_accel == NULL) || (p_speed == NULL) ||
        (p_out == NULL))
    {
        return false;
    }

    p_out->hump_finished      = false;
    p_out->hump_active        = false;
    p_out->peak_height_mm     = 0.0f;
    p_out->peak_pitch_deg     = p_ctx->peak_pitch_deg;
    p_out->run_peak_height_mm = p_ctx->run_peak_mm;
    p_out->state              = p_ctx->state;
    p_out->motion             = MOTION_UNKNOWN;
    p_out->fault              = p_ctx->fault;
    p_out->fault_count        = p_ctx->fault_count;

    /* Stale speed snapshot: same seq many ticks means B2 stalled. */
    if (p_speed->seq == p_ctx->last_speed_seq)
    {
        p_ctx->stale_count++;
    }
    else
    {
        p_ctx->stale_count     = 0u;
        p_ctx->last_speed_seq = p_speed->seq;
    }

    if (p_ctx->stale_count > 50u)
    {
        p_ctx->fault = HUMP_FAULT_STALE_DATA;
        p_ctx->fault_count++;
        p_out->fault       = p_ctx->fault;
        p_out->fault_count = p_ctx->fault_count;

        return false;
    }

    raw_pitch = imu_pitch_deg(p_accel);

    /* Reject free-fall / unplugged sensor readings. */
    mag = sqrtf((p_accel->x_g * p_accel->x_g) +
                (p_accel->y_g * p_accel->y_g) + (p_accel->z_g * p_accel->z_g));

    if ((mag < 0.4f) || (mag > 2.5f))
    {
        p_ctx->fault = HUMP_FAULT_INVALID_TILT;
        p_ctx->fault_count++;
        p_out->fault       = p_ctx->fault;
        p_out->fault_count = p_ctx->fault_count;

        return false;
    }

    p_ctx->filt_pitch_deg = low_pass(p_ctx->filt_pitch_deg, raw_pitch);

    /* Impact: transient above 1g envelope, independent of hump state. */
    float delta = mag - 1.0f;

    if (delta < 0.0f)
    {
        delta = -delta;
    }

    if (delta > HUMP_IMPACT_DELTA_G)
    {
        p_out->motion = MOTION_IMPACT;
    }

    bool speed_ok = p_speed->forward_mps > HUMP_MIN_SPEED_MPS;

    switch (p_ctx->state)
    {
        case HUMP_FLAT:
            if ((p_ctx->filt_pitch_deg > HUMP_PITCH_ENTER_DEG) && speed_ok)
            {
                p_ctx->debounce++;

                if (p_ctx->debounce >= HUMP_DEBOUNCE_SAMPLES)
                {
                    p_ctx->state            = HUMP_CLIMB;
                    p_ctx->window_height_mm = 0.0f;
                    p_ctx->peak_pitch_deg   = p_ctx->filt_pitch_deg;
                    p_ctx->debounce         = 0u;
                }
            }
            else
            {
                p_ctx->debounce = 0u;
            }

            if (p_out->motion != MOTION_IMPACT)
            {
                if (p_speed->forward_mps < 0.02f)
                {
                    p_out->motion = MOTION_STATIONARY;
                }
                else
                {
                    p_out->motion = MOTION_MOVING_FLAT;
                }
            }
            break;

        case HUMP_CLIMB:
            p_out->hump_active = true;
            p_out->motion      = MOTION_CLIMBING;

            if (p_ctx->filt_pitch_deg > p_ctx->peak_pitch_deg)
            {
                p_ctx->peak_pitch_deg = p_ctx->filt_pitch_deg;
            }

            /* Integrate even in climb; sine handles sign. */
            dh_mm = p_speed->forward_mps *
                    sinf(p_ctx->filt_pitch_deg * DEG_TO_RAD) *
                    ((float) HUMP_PERIOD_MS * MS_TO_S) * 1000.0f;

            if (dh_mm > 0.0f)
            {
                p_ctx->window_height_mm += dh_mm;
            }

            if (p_ctx->filt_pitch_deg < HUMP_PITCH_EXIT_DEG)
            {
                p_ctx->state = HUMP_CREST;
            }
            break;

        case HUMP_CREST:
            p_out->hump_active = true;
            p_out->motion      = MOTION_CLIMBING;

            dh_mm = p_speed->forward_mps *
                    sinf(p_ctx->filt_pitch_deg * DEG_TO_RAD) *
                    ((float) HUMP_PERIOD_MS * MS_TO_S) * 1000.0f;
            p_ctx->window_height_mm += dh_mm;

            if (p_ctx->filt_pitch_deg < HUMP_PITCH_DESCEND_DEG)
            {
                p_ctx->state = HUMP_DESCEND;
            }
            else if (p_ctx->filt_pitch_deg > HUMP_PITCH_ENTER_DEG)
            {
                /* Double hump back-to-back. */
                p_ctx->state = HUMP_CLIMB;
            }
            else
            {
                /* Small hump with no clear descent: settle to flat. */
                p_ctx->settle++;

                if (p_ctx->settle >= HUMP_FLAT_SETTLE_SAMPLES)
                {
                    p_ctx->state = HUMP_FLAT;
                    p_ctx->settle = 0u;
                    p_out->hump_finished  = true;
                    p_out->peak_height_mm = p_ctx->window_height_mm;

                    if (p_ctx->window_height_mm > p_ctx->run_peak_mm)
                    {
                        p_ctx->run_peak_mm = p_ctx->window_height_mm;
                    }

                    p_out->run_peak_height_mm = p_ctx->run_peak_mm;
                }
            }
            break;

        case HUMP_DESCEND:
            p_out->hump_active = true;
            p_out->motion      = MOTION_DESCENDING;

            dh_mm = p_speed->forward_mps *
                    sinf(p_ctx->filt_pitch_deg * DEG_TO_RAD) *
                    ((float) HUMP_PERIOD_MS * MS_TO_S) * 1000.0f;
            p_ctx->window_height_mm += dh_mm;

            /* Wait for level for N samples before closing window. */
            if ((p_ctx->filt_pitch_deg > -HUMP_PITCH_EXIT_DEG) &&
                (p_ctx->filt_pitch_deg < HUMP_PITCH_EXIT_DEG))
            {
                p_ctx->settle++;

                if (p_ctx->settle >= HUMP_FLAT_SETTLE_SAMPLES)
                {
                    p_ctx->state  = HUMP_FLAT;
                    p_ctx->settle = 0u;

                    /* Clamp small negative drift from descent tail. */
                    if (p_ctx->window_height_mm < 0.0f)
                    {
                        p_ctx->window_height_mm = 0.0f;
                    }

                    p_out->hump_finished  = true;
                    p_out->peak_height_mm = p_ctx->window_height_mm;

                    if (p_ctx->window_height_mm > p_ctx->run_peak_mm)
                    {
                        p_ctx->run_peak_mm = p_ctx->window_height_mm;
                    }

                    p_out->run_peak_height_mm = p_ctx->run_peak_mm;
                }
            }
            else
            {
                p_ctx->settle = 0u;
            }
            break;

        default:
            p_ctx->state = HUMP_FLAT;
            break;
    }

    p_out->state          = p_ctx->state;
    p_out->peak_pitch_deg = p_ctx->peak_pitch_deg;

    if (p_out->hump_active)
    {
        /* Live estimate for telemetry while crossing. */
        p_out->peak_height_mm = p_ctx->window_height_mm;
    }

    return true;
}

/*** end of file ***/
