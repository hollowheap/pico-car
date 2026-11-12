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
#define DEG_HALF_TURN (180.0f)
#define DEG_FULL_TURN (360.0f)

/* imu_motion_default_config values; all are placeholders until tuned. */
#define DEFAULT_CLASS_SAMPLES    (3U)
#define DEFAULT_STAT_SPEED_MPS   (0.01f)
#define DEFAULT_STAT_DEV_G       (0.05f)
#define DEFAULT_ACCEL_MPS2       (0.3f)
#define DEFAULT_TURN_DPS         (20.0f)
#define DEFAULT_TRACK_WIDTH_M    (0.12f)
#define DEFAULT_MAG_MIN_GAUSS    (0.15f)
#define DEFAULT_MAG_MAX_GAUSS    (1.0f)
#define DEFAULT_HEADING_DEV_G    (0.1f)
#define DEFAULT_MAG_ALPHA        (0.5f)
#define DEFAULT_MAG_TIMEOUT_S    (0.2f)
#define DEFAULT_DISAGREE_DPS     (15.0f)
#define DEFAULT_DISAGREE_SAMPLES (5U)

static bool is_pos (float value)
{
    return isfinite (value) && (value > 0.0f);
}

static void cross (const float p_a[IMU_AXIS_COUNT],
                   const float p_b[IMU_AXIS_COUNT],
                   float p_out[IMU_AXIS_COUNT])
{
    p_out[IMU_AXIS_X] = (p_a[IMU_AXIS_Y] * p_b[IMU_AXIS_Z])
                         - (p_a[IMU_AXIS_Z] * p_b[IMU_AXIS_Y]);
    p_out[IMU_AXIS_Y] = (p_a[IMU_AXIS_Z] * p_b[IMU_AXIS_X])
                         - (p_a[IMU_AXIS_X] * p_b[IMU_AXIS_Z]);
    p_out[IMU_AXIS_Z] = (p_a[IMU_AXIS_X] * p_b[IMU_AXIS_Y])
                         - (p_a[IMU_AXIS_Y] * p_b[IMU_AXIS_X]);
}

static float norm (const float p_v[IMU_AXIS_COUNT])
{
    return sqrtf ((p_v[IMU_AXIS_X] * p_v[IMU_AXIS_X])
                  + (p_v[IMU_AXIS_Y] * p_v[IMU_AXIS_Y])
                  + (p_v[IMU_AXIS_Z] * p_v[IMU_AXIS_Z]));
}

/* Wrap an angle difference into [-180, 180). */
static float wrap180 (float deg)
{
    float wrapped = fmodf (deg + DEG_HALF_TURN, DEG_FULL_TURN);

    if (wrapped < 0.0f)
    {
        wrapped += DEG_FULL_TURN;
    }
    return wrapped - DEG_HALF_TURN;
}

/* Tilt-compensated heading in [0, 360). Returns false when the inputs do
 * not allow a heading (field out of band, car not near 1 g, degenerate). */
static bool compute_heading (const imu_motion_ctx_t * p_ctx,
                             const float p_acc[IMU_AXIS_COUNT],
                             const imu_motion_input_t * p_in,
                             float * p_heading_deg)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;
    float down[IMU_AXIS_COUNT];
    float mag[IMU_AXIS_COUNT];
    float east[IMU_AXIS_COUNT];
    float north[IMU_AXIS_COUNT];
    float acc_norm = norm (p_acc);
    float mag_norm = 0.0f;
    float east_norm = 0.0f;
    float fe = 0.0f;
    float fn = 0.0f;
    float heading = 0.0f;
    uint8_t i = 0U;

    if (fabsf (acc_norm - 1.0f) > p_cfg->heading_dev_g)
    {
        return false;
    }

    mag[IMU_AXIS_X] = p_in->mx_g - p_cfg->mag_offset_g[IMU_AXIS_X];
    mag[IMU_AXIS_Y] = p_in->my_g - p_cfg->mag_offset_g[IMU_AXIS_Y];
    mag[IMU_AXIS_Z] = p_in->mz_g - p_cfg->mag_offset_g[IMU_AXIS_Z];
    mag_norm = norm (mag);
    if ((mag_norm < p_cfg->mag_min_gauss) || (mag_norm > p_cfg->mag_max_gauss))
    {
        return false;
    }

    for (i = 0U; i < IMU_AXIS_COUNT; i++)
    {
        down[i] = -p_acc[i] / acc_norm;
    }

    cross (down, mag, east);
    east_norm = norm (east);
    if (east_norm < MIN_NORM)
    {
        return false;
    }
    for (i = 0U; i < IMU_AXIS_COUNT; i++)
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
        heading += DEG_FULL_TURN;
    }
    if (heading >= DEG_FULL_TURN)
    {
        heading -= DEG_FULL_TURN;
    }
    *p_heading_deg = heading;

    return true;
}

static void update_mag (imu_motion_ctx_t * p_ctx,
                        const float p_acc[IMU_AXIS_COUNT],
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
        p_cfg->class_samples = DEFAULT_CLASS_SAMPLES;
        p_cfg->stat_speed_mps = DEFAULT_STAT_SPEED_MPS;
        p_cfg->stat_dev_g = DEFAULT_STAT_DEV_G;
        p_cfg->accel_mps2 = DEFAULT_ACCEL_MPS2;
        p_cfg->turn_dps = DEFAULT_TURN_DPS;
        p_cfg->track_width_m = DEFAULT_TRACK_WIDTH_M;
        p_cfg->mag_enabled = true;
        p_cfg->mag_offset_g[IMU_AXIS_X] = 0.0f;
        p_cfg->mag_offset_g[IMU_AXIS_Y] = 0.0f;
        p_cfg->mag_offset_g[IMU_AXIS_Z] = 0.0f;
        p_cfg->mag_yaw_sign = 1;
        p_cfg->mag_min_gauss = DEFAULT_MAG_MIN_GAUSS;
        p_cfg->mag_max_gauss = DEFAULT_MAG_MAX_GAUSS;
        p_cfg->heading_dev_g = DEFAULT_HEADING_DEV_G;
        p_cfg->mag_alpha = DEFAULT_MAG_ALPHA;
        p_cfg->mag_timeout_s = DEFAULT_MAG_TIMEOUT_S;
        p_cfg->disagree_dps = DEFAULT_DISAGREE_DPS;
        p_cfg->disagree_samples = DEFAULT_DISAGREE_SAMPLES;
    }
}

bool imu_motion_init (imu_motion_ctx_t * p_ctx,
                      const imu_motion_config_t * p_cfg,
                      const hump_config_t * p_hump_cfg)
{
    bool ok = (NULL != p_ctx) && (NULL != p_cfg) && (NULL != p_hump_cfg);

    if (ok)
    {
        ok = (p_cfg->class_samples >= 1U)
             && is_pos (p_cfg->stat_speed_mps) && is_pos (p_cfg->stat_dev_g)
             && is_pos (p_cfg->accel_mps2) && is_pos (p_cfg->turn_dps)
             && is_pos (p_cfg->track_width_m)
             && isfinite (p_cfg->mag_offset_g[IMU_AXIS_X])
             && isfinite (p_cfg->mag_offset_g[IMU_AXIS_Y])
             && isfinite (p_cfg->mag_offset_g[IMU_AXIS_Z])
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
        p_ctx->have_heading = false;
        p_ctx->last_heading_deg = 0.0f;
        p_ctx->since_mag_s = 0.0f;
        p_ctx->mag_rate_valid = false;
        p_ctx->mag_rate_dps = 0.0f;
        p_ctx->disagree_count = 0U;
    }

    return ok;
}

/* Argument and input checks; the null checks come first. */
static bool update_args_ok (const imu_motion_ctx_t * p_ctx,
                            const hump_sample_t * p_sample,
                            const hump_output_t * p_hump,
                            const imu_motion_input_t * p_in,
                            const imu_motion_output_t * p_out)
{
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

    return true;
}

/* Count consecutive samples where the encoder and magnetometer yaw rates
 * differ by more than disagree_dps. */
static void update_disagree (imu_motion_ctx_t * p_ctx, bool enc_valid,
                             float enc_yaw)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;

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
}

/* Fill the turn-rate and heading fields of the output. */
static void fill_yaw_output (const imu_motion_ctx_t * p_ctx,
                             imu_motion_output_t * p_out, bool enc_valid,
                             float enc_yaw)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;

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
}

/* Pick the class this sample votes for (no collision, calibrated). */
static motion_class_t pick_candidate (const imu_motion_ctx_t * p_ctx,
                                      const hump_sample_t * p_sample,
                                      const hump_output_t * p_hump,
                                      const imu_motion_input_t * p_in,
                                      const imu_motion_output_t * p_out,
                                      float acc_norm, bool fresh,
                                      bool wheels_ok)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;
    motion_class_t cand = MOTION_CLASS_UNKNOWN;

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

    return cand;
}

/* A new class must persist class_samples samples before it is reported. */
static void debounce_class (imu_motion_ctx_t * p_ctx, motion_class_t cand)
{
    const imu_motion_config_t * p_cfg = &p_ctx->cfg;

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

/* Classification. */
static void classify (imu_motion_ctx_t * p_ctx,
                      const hump_sample_t * p_sample,
                      const hump_output_t * p_hump,
                      const imu_motion_input_t * p_in,
                      const imu_motion_output_t * p_out,
                      float acc_norm, bool fresh, bool wheels_ok)
{
    motion_class_t cand = MOTION_CLASS_UNKNOWN;

    if (!p_hump->calibrated)
    {
        p_ctx->motion = MOTION_CLASS_UNKNOWN;
        p_ctx->candidate_count = 0U;
    }
    else
    {
        cand = pick_candidate (p_ctx, p_sample, p_hump, p_in, p_out, acc_norm,
                               fresh, wheels_ok);
        debounce_class (p_ctx, cand);
    }
}

bool imu_motion_update (imu_motion_ctx_t * p_ctx,
                        const hump_sample_t * p_sample,
                        const hump_output_t * p_hump,
                        const imu_motion_input_t * p_in,
                        imu_motion_output_t * p_out)
{
    const imu_motion_config_t * p_cfg = NULL;
    float acc[IMU_AXIS_COUNT];
    float acc_norm = 0.0f;
    float residual = 0.0f;
    float enc_yaw = 0.0f;
    bool fresh = false;
    bool wheels_ok = false;
    bool enc_valid = false;
    motion_class_t previous = MOTION_CLASS_UNKNOWN;

    if (!update_args_ok (p_ctx, p_sample, p_hump, p_in, p_out))
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

    /* Forward residual against the filtered pitch, for telemetry. */
    residual = ((float)p_ctx->fwd_sign * acc[p_ctx->fwd_axis])
               - sinf ((p_hump->pitch_deg + p_hump->theta0_deg) * DEG_TO_RAD);

    /* Turn rate. */
    enc_valid = wheels_ok;
    if (enc_valid)
    {
        enc_yaw = ((p_in->v_right_mps - p_in->v_left_mps) / p_cfg->track_width_m)
                  * RAD_TO_DEG;
    }
    update_mag (p_ctx, acc, p_in);

    update_disagree (p_ctx, enc_valid, enc_yaw);
    fill_yaw_output (p_ctx, p_out, enc_valid, enc_yaw);

    classify (p_ctx, p_sample, p_hump, p_in, p_out, acc_norm, fresh,
              wheels_ok);

    p_out->motion = p_ctx->motion;
    p_out->changed = (p_ctx->motion != previous);
    p_out->fwd_residual_g = residual;

    return true;
}

/*** end of file ***/
