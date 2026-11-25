/** @file imu_motion.c
 *
 * @brief Motion classification, collision detection and turn rate.
 *
 * Heading uses vector tilt compensation, which needs only the forward axis:
 *   D = -a / |a|      (down, from the accelerometer)
 *   E = D x M         (east, perpendicular to down and the field)
 *   N = E x D         (north, horizontal)
 *   heading = atan2(f . E, f . N), f = unit vector along the forward axis.
 * This assumes the accelerometer and magnetometer axes are parallel and the
 * board frame is right-handed. If the real board is mirrored, the heading
 * turns the wrong way; set mag_yaw_sign to -1.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hump_detect.h"
#include "imu_motion.h"

#define RAD_TO_DEG   (57.2957795f)
#define DEG_TO_RAD   (0.0174532925f)
#define STD_GRAVITY  (9.80665f)
#define MIN_NORM     (1.0e-6f)

static bool is_pos (float value)
{
    return isfinite (value) && (value > 0.0f);
}

static void cross (const float p_a[3], const float p_b[3], float p_out[3])
{
    p_out[0] = (p_a[1] * p_b[2]) - (p_a[2] * p_b[1]);
    p_out[1] = (p_a[2] * p_b[0]) - (p_a[0] * p_b[2]);
    p_out[2] = (p_a[0] * p_b[1]) - (p_a[1] * p_b[0]);
}

static float norm (const float p_v[3])
{
    return sqrtf ((p_v[0] * p_v[0]) + (p_v[1] * p_v[1]) + (p_v[2] * p_v[2]));
}

/* Wrap an angle difference into [-180, 180). */
static float wrap180 (float deg)
{
    float wrapped = fmodf (deg + 180.0f, 360.0f);

    if (wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped - 180.0f;
}

/* Tilt-compensated heading in [0, 360). Returns false when the inputs do
 * not allow a heading (field out of band, car not near 1 g, degenerate). */
static bool compute_heading (const imu_motion_ctx_t * p_ctx,
                             const float p_acc[3],
                             const imu_motion_input_t * p_in,
                             float * p_heading_deg)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;
    float down[3];
    float mag[3];
    float east[3];
    float north[3];
    float acc_norm = norm (p_acc);
    float mag_norm;
    float east_norm;
    float fe;
    float fn;
    float heading;
    uint8_t i;

    if (fabsf (acc_norm - 1.0f) > p_cfg->heading_dev_g)
    {
        return false;
    }

    mag[0] = p_in->mx_g - p_cfg->mag_offset_g[0];
    mag[1] = p_in->my_g - p_cfg->mag_offset_g[1];
    mag[2] = p_in->mz_g - p_cfg->mag_offset_g[2];
    mag_norm = norm (mag);
    if ((mag_norm < p_cfg->mag_min_gauss) || (mag_norm > p_cfg->mag_max_gauss))
    {
        return false;
    }

    for (i = 0U; i < 3U; i++)
    {
        down[i] = -p_acc[i] / acc_norm;
    }

    cross (down, mag, east);
    east_norm = norm (east);
    if (east_norm < MIN_NORM)
    {
        return false;
    }
    for (i = 0U; i < 3U; i++)
    {
        east[i] /= east_norm;
    }
    cross (east, down, north);

    /* f has a single nonzero component, so f . v is one signed component. */
    fe = (float)p_ctx->fwd_sign * east[p_ctx->fwd_axis];
    fn = (float)p_ctx->fwd_sign * north[p_ctx->fwd_axis];
    if ((fabsf (fe) < MIN_NORM) && (fabsf (fn) < MIN_NORM))
    {
        /* Forward axis is vertical: heading undefined. */
        return false;
    }

    heading = atan2f (fe, fn) * RAD_TO_DEG;
    if (heading < 0.0f)
    {
        heading += 360.0f;
    }
    if (heading >= 360.0f)
    {
        heading -= 360.0f;
    }
    *p_heading_deg = heading;

    return true;
}

static void update_mag (imu_motion_ctx_t * p_ctx, const float p_acc[3],
                        const imu_motion_input_t * p_in)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;
    float heading = 0.0f;

    p_ctx->since_mag_s += p_in->dt_s;

    if (p_cfg->mag_enabled && p_in->mag_new)
    {
        if (compute_heading (p_ctx, p_acc, p_in, &heading))
        {
            if (p_ctx->have_heading)
            {
                /* Heading grows clockwise; yaw rate is positive left. */
                float rate = -(float)p_cfg->mag_yaw_sign
                             * wrap180 (heading - p_ctx->last_heading_deg)
                             / p_ctx->since_mag_s;

                if (p_ctx->mag_rate_valid)
                {
                    p_ctx->mag_rate_dps += p_cfg->mag_alpha
                                           * (rate - p_ctx->mag_rate_dps);
                }
                else
                {
                    p_ctx->mag_rate_dps = rate;
                    p_ctx->mag_rate_valid = true;
                }
            }
            p_ctx->last_heading_deg = heading;
            p_ctx->have_heading = true;
        }
        else
        {
            p_ctx->have_heading = false;
            p_ctx->mag_rate_valid = false;
        }
        p_ctx->since_mag_s = 0.0f;
    }

    if (p_ctx->since_mag_s > p_cfg->mag_timeout_s)
    {
        p_ctx->have_heading = false;
        p_ctx->mag_rate_valid = false;
    }
}

void imu_motion_default_config (imu_motion_config_t * p_cfg)
{
    if (NULL != p_cfg)
    {
        p_cfg->class_samples = 3U;
        p_cfg->impact_hold = 10U;
        p_cfg->stat_speed_mps = 0.01f;
        p_cfg->stat_dev_g = 0.05f;
        p_cfg->accel_mps2 = 0.3f;
        p_cfg->turn_dps = 20.0f;
        p_cfg->collision_g = 0.5f;
        p_cfg->track_width_m = 0.12f;
        p_cfg->mag_enabled = true;
        p_cfg->mag_offset_g[0] = 0.0f;
        p_cfg->mag_offset_g[1] = 0.0f;
        p_cfg->mag_offset_g[2] = 0.0f;
        p_cfg->mag_yaw_sign = 1;
        p_cfg->mag_min_gauss = 0.15f;
        p_cfg->mag_max_gauss = 1.0f;
        p_cfg->heading_dev_g = 0.1f;
        p_cfg->mag_alpha = 0.5f;
        p_cfg->mag_timeout_s = 0.2f;
        p_cfg->disagree_dps = 15.0f;
        p_cfg->disagree_samples = 5U;
    }
}

bool imu_motion_init (imu_motion_ctx_t * p_ctx,
                      const imu_motion_config_t * p_cfg,
                      const hump_config_t * p_hump_cfg)
{
    bool ok = (NULL != p_ctx) && (NULL != p_cfg) && (NULL != p_hump_cfg);

    if (ok)
    {
        ok = (p_cfg->class_samples >= 1U) && (p_cfg->impact_hold >= 1U)
             && is_pos (p_cfg->stat_speed_mps) && is_pos (p_cfg->stat_dev_g)
             && is_pos (p_cfg->accel_mps2) && is_pos (p_cfg->turn_dps)
             && is_pos (p_cfg->collision_g) && is_pos (p_cfg->track_width_m)
             && isfinite (p_cfg->mag_offset_g[0])
             && isfinite (p_cfg->mag_offset_g[1])
             && isfinite (p_cfg->mag_offset_g[2])
             && ((1 == p_cfg->mag_yaw_sign) || (-1 == p_cfg->mag_yaw_sign))
             && is_pos (p_cfg->mag_min_gauss)
             && isfinite (p_cfg->mag_max_gauss)
             && (p_cfg->mag_max_gauss > p_cfg->mag_min_gauss)
             && is_pos (p_cfg->heading_dev_g)
             && is_pos (p_cfg->mag_alpha) && (p_cfg->mag_alpha <= 1.0f)
             && is_pos (p_cfg->mag_timeout_s) && is_pos (p_cfg->disagree_dps)
             && (p_cfg->disagree_samples >= 1U)
             && (p_hump_cfg->fwd_axis <= HUMP_AXIS_Z)
             && ((1 == p_hump_cfg->fwd_sign) || (-1 == p_hump_cfg->fwd_sign));
    }

    if (ok)
    {
        p_ctx->cfg = *p_cfg;
        p_ctx->fwd_axis = p_hump_cfg->fwd_axis;
        p_ctx->fwd_sign = p_hump_cfg->fwd_sign;
        p_ctx->motion = MOTION_CLASS_UNKNOWN;
        p_ctx->candidate = MOTION_CLASS_UNKNOWN;
        p_ctx->candidate_count = 0U;
        p_ctx->impact_left = 0U;
        p_ctx->have_heading = false;
        p_ctx->last_heading_deg = 0.0f;
        p_ctx->since_mag_s = 0.0f;
        p_ctx->mag_rate_valid = false;
        p_ctx->mag_rate_dps = 0.0f;
        p_ctx->disagree_count = 0U;
    }

    return ok;
}

bool imu_motion_update (imu_motion_ctx_t * p_ctx,
                        const hump_sample_t * p_sample,
                        const hump_output_t * p_hump,
                        const imu_motion_input_t * p_in,
                        imu_motion_output_t * p_out)
{
    const imu_motion_config_t * p_cfg;
    float acc[3];
    float acc_norm;
    float residual;
    float enc_yaw = 0.0f;
    bool fresh;
    bool wheels_ok;
    bool enc_valid;
    bool collision;
    motion_class_t previous;
    motion_class_t cand;

    if ((NULL == p_ctx) || (NULL == p_sample) || (NULL == p_hump)
        || (NULL == p_in) || (NULL == p_out))
    {
        return false;
    }
    if (!is_pos (p_in->dt_s)
        || (p_in->wheels_valid
            && (!isfinite (p_in->v_left_mps) || !isfinite (p_in->v_right_mps)))
        || (p_in->mag_new
            && (!isfinite (p_in->mx_g) || !isfinite (p_in->my_g)
                || !isfinite (p_in->mz_g))))
    {
        return false;
    }

    p_cfg = &p_ctx->cfg;
    previous = p_ctx->motion;

    /* hump_update has already checked the sample and judged freshness. */
    fresh = !p_hump->motion_stale;
    wheels_ok = fresh && p_in->wheels_valid;

    acc[HUMP_AXIS_X] = p_sample->ax_g;
    acc[HUMP_AXIS_Y] = p_sample->ay_g;
    acc[HUMP_AXIS_Z] = p_sample->az_g;
    acc_norm = norm (acc);
    if (fresh)
    {
        /* Remove the car's own acceleration so gravity points down. */
        acc[p_ctx->fwd_axis] -= (float)p_ctx->fwd_sign
                                * (p_sample->accel_mps2 / STD_GRAVITY);
    }

    /* Collision: forward residual against the filtered pitch. */
    residual = ((float)p_ctx->fwd_sign * acc[p_ctx->fwd_axis])
               - sinf ((p_hump->pitch_deg + p_hump->theta0_deg) * DEG_TO_RAD);
    collision = p_hump->impact
                || (p_hump->calibrated && (fabsf (residual) > p_cfg->collision_g));

    /* Turn rate. */
    enc_valid = wheels_ok;
    if (enc_valid)
    {
        enc_yaw = ((p_in->v_right_mps - p_in->v_left_mps) / p_cfg->track_width_m)
                  * RAD_TO_DEG;
    }
    update_mag (p_ctx, acc, p_in);

    if (enc_valid && p_ctx->mag_rate_valid
        && (fabsf (enc_yaw - p_ctx->mag_rate_dps) > p_cfg->disagree_dps))
    {
        if (p_ctx->disagree_count < UINT16_MAX)
        {
            p_ctx->disagree_count++;
        }
    }
    else
    {
        p_ctx->disagree_count = 0U;
    }

    p_out->enc_yaw_dps = enc_yaw;
    p_out->enc_yaw_valid = enc_valid;
    p_out->mag_yaw_dps = p_ctx->mag_rate_valid ? p_ctx->mag_rate_dps : 0.0f;
    p_out->mag_yaw_valid = p_ctx->mag_rate_valid;
    p_out->heading_deg = p_ctx->have_heading ? p_ctx->last_heading_deg : 0.0f;
    p_out->heading_valid = p_ctx->have_heading;
    p_out->yaw_disagree = (p_ctx->disagree_count >= p_cfg->disagree_samples);
    if (enc_valid)
    {
        p_out->yaw_rate_dps = enc_yaw;
        p_out->yaw_src = IMU_YAW_ENCODER;
    }
    else if (p_ctx->mag_rate_valid)
    {
        p_out->yaw_rate_dps = p_ctx->mag_rate_dps;
        p_out->yaw_src = IMU_YAW_MAG;
    }
    else
    {
        p_out->yaw_rate_dps = 0.0f;
        p_out->yaw_src = IMU_YAW_NONE;
    }

    /* Classification. */
    if (!p_hump->calibrated)
    {
        p_ctx->motion = MOTION_CLASS_UNKNOWN;
        p_ctx->candidate_count = 0U;
        p_ctx->impact_left = 0U;
    }
    else if (collision)
    {
        p_ctx->motion = MOTION_CLASS_IMPACT;
        p_ctx->candidate_count = 0U;
        p_ctx->impact_left = p_cfg->impact_hold;
    }
    else if (p_ctx->impact_left > 1U)
    {
        p_ctx->impact_left--;
    }
    else
    {
        p_ctx->impact_left = 0U;

        if (HUMP_STATE_CLIMB == p_hump->state)
        {
            cand = MOTION_CLASS_CLIMBING;
        }
        else if (HUMP_STATE_DESCEND == p_hump->state)
        {
            cand = MOTION_CLASS_DESCENDING;
        }
        else if (wheels_ok
                 && (fabsf (p_in->v_left_mps) < p_cfg->stat_speed_mps)
                 && (fabsf (p_in->v_right_mps) < p_cfg->stat_speed_mps)
                 && (fabsf (acc_norm - 1.0f) <= p_cfg->stat_dev_g))
        {
            cand = MOTION_CLASS_STATIONARY;
        }
        else if ((IMU_YAW_NONE != p_out->yaw_src)
                 && (fabsf (p_out->yaw_rate_dps) > p_cfg->turn_dps))
        {
            cand = MOTION_CLASS_TURNING;
        }
        else if (fresh && (fabsf (p_sample->accel_mps2) > p_cfg->accel_mps2))
        {
            cand = MOTION_CLASS_ACCELERATING;
        }
        else if (fresh)
        {
            cand = MOTION_CLASS_MOVING;
        }
        else
        {
            cand = MOTION_CLASS_UNKNOWN;
        }

        if (cand == p_ctx->motion)
        {
            p_ctx->candidate_count = 0U;
        }
        else
        {
            if (cand == p_ctx->candidate)
            {
                p_ctx->candidate_count++;
            }
            else
            {
                p_ctx->candidate = cand;
                p_ctx->candidate_count = 1U;
            }
            if (p_ctx->candidate_count >= p_cfg->class_samples)
            {
                p_ctx->motion = cand;
                p_ctx->candidate_count = 0U;
            }
        }
    }

    p_out->motion = p_ctx->motion;
    p_out->changed = (p_ctx->motion != previous);
    p_out->collision = collision;
    p_out->fwd_residual_g = residual;

    return true;
}

/*** end of file ***/
