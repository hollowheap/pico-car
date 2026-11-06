/** @file test_imu_motion.c
 *
 * @brief Host tests for motion classification, collision and turn rate.
 *
 * World model: board X forward, Y left, Z up (right-handed), level mount.
 * World frame north-east-down. Heading psi is clockwise from north, pitch
 * theta nose-up. Earth field 0.3 G north, 0.4 G down (about Singapore's
 * total is lower; values only need to be inside the accepted band).
 * The accelerometer reads world-up (1 g) plus dv/dt on the forward axis.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hump_detect.h"
#include "imu_motion.h"

#define DT_S        (0.02f)
#define SPEED_MPS   (0.15f)
#define DEG_TO_RAD  (0.0174532925f)
#define RAD_TO_DEG  (57.2957795f)
#define STD_GRAVITY (9.80665f)
#define TRACK_M     (0.12f)
#define MAG_EVERY   (3U)     /* New mag sample every 3rd call: ~16.7 Hz. */

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            printf ("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

typedef struct
{
    float psi_deg;
    float theta_deg;
    float v_left;
    float v_right;
    float accel;           /* dv/dt reported by Buddy 2 and felt by IMU.  */
    float fwd_spike_g;     /* Extra forward reading (collision).          */
    float mag_scale;       /* 1 = clean Earth field.                      */
    bool  motion_valid;
    bool  wheels_valid;
    bool  mag_on;          /* Deliver mag samples at all.                 */
    bool  slip;            /* Wheels report speeds but heading is fixed.  */
} world_t;

typedef struct
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    hump_ctx_t hump;
    imu_motion_ctx_t motion;
    uint32_t n;
    hump_output_t hout;
    imu_motion_output_t mout;
} sim_t;

static void world_init (world_t * p_w)
{
    memset (p_w, 0, sizeof (*p_w));
    p_w->mag_scale = 1.0f;
    p_w->motion_valid = true;
    p_w->wheels_valid = true;
    p_w->mag_on = true;
}

static float dot (const float p_a[3], const float p_b[3])
{
    return (p_a[0] * p_b[0]) + (p_a[1] * p_b[1]) + (p_a[2] * p_b[2]);
}

static void sim_init (sim_t * p_s)
{
    memset (p_s, 0, sizeof (*p_s));
    hump_default_config (&p_s->hcfg);
    imu_motion_default_config (&p_s->mcfg);
    p_s->mcfg.track_width_m = TRACK_M;
    CHECK (hump_init (&p_s->hump, &p_s->hcfg));
    CHECK (imu_motion_init (&p_s->motion, &p_s->mcfg, &p_s->hcfg));
}

/* One 20 ms step. Advances psi from the wheel speeds unless slipping. */
static void sim_step (sim_t * p_s, world_t * p_w)
{
    float c_t = cosf (p_w->theta_deg * DEG_TO_RAD);
    float s_t = sinf (p_w->theta_deg * DEG_TO_RAD);
    float c_p = cosf (p_w->psi_deg * DEG_TO_RAD);
    float s_p = sinf (p_w->psi_deg * DEG_TO_RAD);
    float fwd[3];
    float left[3];
    float up[3];
    float world_up[3] = { 0.0f, 0.0f, -1.0f };
    float field[3];
    float speed = 0.5f * (p_w->v_left + p_w->v_right);
    hump_sample_t hs;
    imu_motion_input_t in;

    fwd[0] = c_t * c_p;
    fwd[1] = c_t * s_p;
    fwd[2] = -s_t;
    left[0] = s_p;
    left[1] = -c_p;
    left[2] = 0.0f;
    up[0] = (fwd[1] * left[2]) - (fwd[2] * left[1]);
    up[1] = (fwd[2] * left[0]) - (fwd[0] * left[2]);
    up[2] = (fwd[0] * left[1]) - (fwd[1] * left[0]);
    field[0] = 0.3f * p_w->mag_scale;
    field[1] = 0.0f;
    field[2] = 0.4f * p_w->mag_scale;

    memset (&hs, 0, sizeof (hs));
    hs.ax_g = dot (world_up, fwd) + (p_w->accel / STD_GRAVITY)
              + p_w->fwd_spike_g;
    hs.ay_g = dot (world_up, left);
    hs.az_g = dot (world_up, up);
    hs.motion_valid = p_w->motion_valid;
    hs.motion_age_s = 0.005f;
    hs.speed_mps = speed;
    hs.ds_m = speed * DT_S;
    hs.accel_mps2 = p_w->accel;

    memset (&in, 0, sizeof (in));
    in.dt_s = DT_S;
    in.wheels_valid = p_w->wheels_valid;
    in.v_left_mps = p_w->v_left;
    in.v_right_mps = p_w->v_right;
    in.mag_new = p_w->mag_on && (0U == (p_s->n % MAG_EVERY));
    in.mx_g = dot (field, fwd);
    in.my_g = dot (field, left);
    in.mz_g = dot (field, up);

    CHECK (hump_update (&p_s->hump, &hs, &p_s->hout));
    CHECK (imu_motion_update (&p_s->motion, &hs, &p_s->hout, &in,
                              &p_s->mout));
    p_s->n++;

    if (!p_w->slip)
    {
        /* Positive yaw (left) lowers the clockwise heading. */
        float yaw_dps = ((p_w->v_right - p_w->v_left) / TRACK_M) * RAD_TO_DEG;

        p_w->psi_deg -= yaw_dps * DT_S;
        if (p_w->psi_deg < 0.0f)
        {
            p_w->psi_deg += 360.0f;
        }
        if (p_w->psi_deg >= 360.0f)
        {
            p_w->psi_deg -= 360.0f;
        }
    }
}

static void sim_run (sim_t * p_s, world_t * p_w, uint32_t steps)
{
    uint32_t i;

    for (i = 0U; i < steps; i++)
    {
        sim_step (p_s, p_w);
    }
}

/* Calibrate at rest, then settle for a few samples. */
static void sim_calibrate (sim_t * p_s, world_t * p_w)
{
    world_t rest = *p_w;

    rest.v_left = 0.0f;
    rest.v_right = 0.0f;
    rest.accel = 0.0f;
    sim_run (p_s, &rest, 60U);
    CHECK (p_s->hout.calibrated);
}

static void cruise (world_t * p_w)
{
    p_w->v_left = SPEED_MPS;
    p_w->v_right = SPEED_MPS;
}

static void test_init_validation (void)
{
    imu_motion_ctx_t ctx;
    imu_motion_config_t cfg;
    hump_config_t hcfg;

    printf ("init_validation\n");
    hump_default_config (&hcfg);
    imu_motion_default_config (&cfg);
    CHECK (imu_motion_init (&ctx, &cfg, &hcfg));
    CHECK (!imu_motion_init (NULL, &cfg, &hcfg));
    CHECK (!imu_motion_init (&ctx, NULL, &hcfg));
    CHECK (!imu_motion_init (&ctx, &cfg, NULL));

    cfg.track_width_m = 0.0f;
    CHECK (!imu_motion_init (&ctx, &cfg, &hcfg));
    imu_motion_default_config (&cfg);
    cfg.mag_yaw_sign = 0;
    CHECK (!imu_motion_init (&ctx, &cfg, &hcfg));
    imu_motion_default_config (&cfg);
    cfg.mag_max_gauss = cfg.mag_min_gauss;
    CHECK (!imu_motion_init (&ctx, &cfg, &hcfg));
    imu_motion_default_config (&cfg);
    cfg.class_samples = 0U;
    CHECK (!imu_motion_init (&ctx, &cfg, &hcfg));
    imu_motion_default_config (&cfg);
    cfg.mag_alpha = 1.5f;
    CHECK (!imu_motion_init (&ctx, &cfg, &hcfg));
    imu_motion_default_config (&cfg);
    cfg.mag_offset_g[1] = NAN;
    CHECK (!imu_motion_init (&ctx, &cfg, &hcfg));
}

static void test_rejects_bad_input (void)
{
    sim_t s;
    hump_sample_t hs;
    hump_output_t ho;
    imu_motion_input_t in;
    imu_motion_output_t out;

    printf ("update_rejects_bad_input\n");
    sim_init (&s);
    memset (&hs, 0, sizeof (hs));
    memset (&ho, 0, sizeof (ho));
    memset (&in, 0, sizeof (in));
    hs.az_g = 1.0f;
    in.dt_s = DT_S;
    CHECK (imu_motion_update (&s.motion, &hs, &ho, &in, &out));
    in.dt_s = 0.0f;
    CHECK (!imu_motion_update (&s.motion, &hs, &ho, &in, &out));
    in.dt_s = DT_S;
    in.wheels_valid = true;
    in.v_left_mps = NAN;
    CHECK (!imu_motion_update (&s.motion, &hs, &ho, &in, &out));
    in.wheels_valid = false;
    CHECK (imu_motion_update (&s.motion, &hs, &ho, &in, &out));
    in.mag_new = true;
    in.mz_g = INFINITY;
    CHECK (!imu_motion_update (&s.motion, &hs, &ho, &in, &out));
    CHECK (!imu_motion_update (NULL, &hs, &ho, &in, &out));
    CHECK (!imu_motion_update (&s.motion, NULL, &ho, &in, &out));
    CHECK (!imu_motion_update (&s.motion, &hs, NULL, &in, &out));
    CHECK (!imu_motion_update (&s.motion, &hs, &ho, NULL, &out));
    CHECK (!imu_motion_update (&s.motion, &hs, &ho, &in, NULL));
}

static void test_unknown_then_stationary (void)
{
    sim_t s;
    world_t w;
    uint32_t i;
    bool unknown_ok = true;

    printf ("unknown_until_calibrated_then_stationary\n");
    sim_init (&s);
    world_init (&w);
    for (i = 0U; i < 49U; i++)
    {
        sim_step (&s, &w);
        unknown_ok = unknown_ok && (MOTION_CLASS_UNKNOWN == s.mout.motion);
    }
    CHECK (unknown_ok);
    sim_run (&s, &w, 5U);
    CHECK (MOTION_CLASS_STATIONARY == s.mout.motion);
}

static void test_moving_and_accelerating (void)
{
    sim_t s;
    world_t w;

    printf ("moving_accelerating_and_debounce\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    cruise (&w);
    sim_run (&s, &w, 10U);
    CHECK (MOTION_CLASS_MOVING == s.mout.motion);
    CHECK (!s.mout.collision);
    CHECK (fabsf (s.mout.yaw_rate_dps) < 0.01f);

    /* A one-sample dv/dt blip does not change the class. */
    w.accel = 0.5f;
    sim_step (&s, &w);
    w.accel = 0.0f;
    sim_run (&s, &w, 5U);
    CHECK (MOTION_CLASS_MOVING == s.mout.motion);

    /* Sustained acceleration does, and changed is set once. */
    w.accel = 0.5f;
    sim_run (&s, &w, 2U);
    CHECK (MOTION_CLASS_MOVING == s.mout.motion);
    sim_step (&s, &w);
    CHECK (MOTION_CLASS_ACCELERATING == s.mout.motion);
    CHECK (s.mout.changed);
    sim_step (&s, &w);
    CHECK (!s.mout.changed);
    CHECK (!s.mout.collision);

    /* Braking counts as accelerating too. */
    w.accel = -0.5f;
    sim_run (&s, &w, 5U);
    CHECK (MOTION_CLASS_ACCELERATING == s.mout.motion);
}

static void test_encoder_turn (void)
{
    sim_t s;
    world_t w;
    float expect = (0.1f / TRACK_M) * RAD_TO_DEG;

    printf ("encoder_turn_left_matches_magnetometer\n");
    sim_init (&s);
    world_init (&w);
    w.psi_deg = 90.0f;
    sim_calibrate (&s, &w);
    w.v_left = 0.10f;
    w.v_right = 0.20f;
    sim_run (&s, &w, 50U);
    printf ("  expected %.1f dps, encoder %.1f, mag %.1f\n", (double)expect,
            (double)s.mout.enc_yaw_dps, (double)s.mout.mag_yaw_dps);
    CHECK (MOTION_CLASS_TURNING == s.mout.motion);
    CHECK (IMU_YAW_ENCODER == s.mout.yaw_src);
    CHECK (fabsf (s.mout.yaw_rate_dps - expect) < 0.01f);
    CHECK (s.mout.mag_yaw_valid);
    CHECK (fabsf (s.mout.mag_yaw_dps - expect) < 3.0f);
    CHECK (!s.mout.yaw_disagree);
    CHECK (s.mout.heading_valid);
    CHECK (fabsf (s.mout.heading_deg - w.psi_deg) < 3.0f);
}

static void test_right_turn_through_north (void)
{
    sim_t s;
    world_t w;
    uint32_t i;
    float expect = -30.0f;
    float diff = (30.0f * DEG_TO_RAD) * TRACK_M;
    bool ok = true;

    printf ("right_turn_through_north_wraps\n");
    sim_init (&s);
    world_init (&w);
    w.psi_deg = 340.0f;
    sim_calibrate (&s, &w);
    w.v_left = SPEED_MPS + (0.5f * diff);
    w.v_right = SPEED_MPS - (0.5f * diff);
    sim_run (&s, &w, 20U);
    /* 1.6 s more at 30 dps carries psi from ~352 past 0 to ~40. */
    for (i = 0U; i < 80U; i++)
    {
        sim_step (&s, &w);
        ok = ok && s.mout.mag_yaw_valid
             && (fabsf (s.mout.mag_yaw_dps - expect) < 3.0f)
             && !s.mout.yaw_disagree;
    }
    printf ("  final heading %.1f (true %.1f), mag yaw %.1f dps\n",
            (double)s.mout.heading_deg, (double)w.psi_deg,
            (double)s.mout.mag_yaw_dps);
    CHECK (ok);
    CHECK (w.psi_deg > 20.0f);
    CHECK (w.psi_deg < 90.0f);
    CHECK (fabsf (s.mout.enc_yaw_dps - expect) < 0.01f);
    CHECK (MOTION_CLASS_TURNING == s.mout.motion);
}

static void test_pivot_turn (void)
{
    sim_t s;
    world_t w;

    printf ("pivot_turn_is_turning_not_stationary\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    w.v_left = -0.05f;
    w.v_right = 0.05f;
    sim_run (&s, &w, 10U);
    CHECK (MOTION_CLASS_TURNING == s.mout.motion);
}

static void test_tilt_compensation (void)
{
    static const float pitches[] = { -15.0f, 0.0f, 10.0f, 20.0f };
    uint32_t p;
    uint32_t h;
    float worst = 0.0f;

    printf ("heading_tilt_compensated\n");
    for (p = 0U; p < (sizeof (pitches) / sizeof (pitches[0])); p++)
    {
        for (h = 0U; h < 8U; h++)
        {
            sim_t s;
            world_t w;
            float err;

            sim_init (&s);
            world_init (&w);
            w.psi_deg = 45.0f * (float)h;
            w.theta_deg = pitches[p];
            sim_step (&s, &w);   /* n = 0 delivers a mag sample. */
            CHECK (s.mout.heading_valid);
            err = fabsf (fmodf (s.mout.heading_deg - w.psi_deg + 540.0f,
                                360.0f) - 180.0f);
            if (err > worst)
            {
                worst = err;
            }
        }
    }
    printf ("  worst heading error %.3f deg over pitch -15..20\n",
            (double)worst);
    CHECK (worst < 0.5f);
}

static void test_mag_disturbed (void)
{
    sim_t s;
    world_t w;

    printf ("disturbed_field_rejected\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    w.v_left = 0.10f;
    w.v_right = 0.20f;
    w.mag_scale = 5.0f;     /* |m| = 2.5 G, far outside the band. */
    sim_run (&s, &w, 20U);
    CHECK (!s.mout.heading_valid);
    CHECK (!s.mout.mag_yaw_valid);
    CHECK (IMU_YAW_ENCODER == s.mout.yaw_src);
    CHECK (MOTION_CLASS_TURNING == s.mout.motion);
}

static void test_wheel_slip (void)
{
    sim_t s;
    world_t w;

    printf ("encoder_vs_mag_disagreement\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    w.v_left = 0.10f;
    w.v_right = 0.20f;
    w.slip = true;          /* Wheels say turning, heading does not move. */
    sim_run (&s, &w, 20U);
    CHECK (s.mout.mag_yaw_valid);
    CHECK (s.mout.yaw_disagree);
}

static void test_mag_fallback (void)
{
    sim_t s;
    world_t w;

    printf ("mag_fallback_when_encoders_invalid\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    w.v_left = 0.10f;
    w.v_right = 0.20f;
    sim_run (&s, &w, 20U);
    w.wheels_valid = false;
    sim_run (&s, &w, 20U);
    CHECK (!s.mout.enc_yaw_valid);
    CHECK (IMU_YAW_MAG == s.mout.yaw_src);
    CHECK (MOTION_CLASS_TURNING == s.mout.motion);

    /* Mag samples stop: invalid after the 0.2 s timeout. */
    w.mag_on = false;
    sim_run (&s, &w, 12U);
    CHECK (!s.mout.mag_yaw_valid);
    CHECK (IMU_YAW_NONE == s.mout.yaw_src);
}

static void test_stale_motion (void)
{
    sim_t s;
    world_t w;

    printf ("stale_motion_gives_unknown\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    cruise (&w);
    sim_run (&s, &w, 10U);
    w.motion_valid = false;
    w.mag_on = false;
    sim_run (&s, &w, 20U);
    CHECK (MOTION_CLASS_UNKNOWN == s.mout.motion);
}

static void test_collision (void)
{
    sim_t s;
    world_t w;
    uint32_t i;
    uint32_t impact_samples = 0U;
    bool hump_impact = false;
    bool flagged = false;

    printf ("forward_collision_detected\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    cruise (&w);
    sim_run (&s, &w, 10U);

    /* Hitting an obstacle: -0.8 g for 2 samples on the forward axis. */
    w.fwd_spike_g = -0.8f;
    for (i = 0U; i < 2U; i++)
    {
        sim_step (&s, &w);
        flagged = flagged || s.mout.collision;
        hump_impact = hump_impact || s.hout.impact;
    }
    w.fwd_spike_g = 0.0f;
    CHECK (flagged);
    CHECK (!hump_impact);   /* | |a| - 1 g | alone would miss this hit. */
    CHECK (MOTION_CLASS_IMPACT == s.mout.motion);

    for (i = 0U; i < 30U; i++)
    {
        sim_step (&s, &w);
        if (MOTION_CLASS_IMPACT == s.mout.motion)
        {
            impact_samples++;
        }
    }
    CHECK (impact_samples >= 9U);
    CHECK (MOTION_CLASS_MOVING == s.mout.motion);
}

static void test_reported_braking (void)
{
    sim_t s;
    world_t w;
    uint32_t i;
    bool any_collision = false;

    printf ("hard_braking_reported_by_buddy2_is_not_a_collision\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    cruise (&w);
    sim_run (&s, &w, 10U);

    /* -6 m/s^2 (0.61 g) short-brake for 60 ms, reported as dv/dt. */
    w.accel = -6.0f;
    for (i = 0U; i < 3U; i++)
    {
        sim_step (&s, &w);
        any_collision = any_collision || s.mout.collision;
        CHECK (s.mout.heading_valid);
    }
    CHECK (!any_collision);
    CHECK (MOTION_CLASS_IMPACT != s.mout.motion);
}

static void test_hump_classes (void)
{
    sim_t s;
    world_t w;
    uint32_t i;
    bool saw_climb = false;
    bool saw_descend = false;
    bool any_collision = false;

    printf ("hump_gives_climbing_then_descending_no_collision\n");
    sim_init (&s);
    world_init (&w);
    sim_calibrate (&s, &w);
    cruise (&w);
    for (i = 0U; i < 200U; i++)
    {
        uint32_t t = (i >= 20U) ? (i - 20U) : 1000U;

        w.theta_deg = 0.0f;
        if (t < 39U)
        {
            w.theta_deg = 15.0f;
        }
        else if ((t >= 49U) && (t < 88U))
        {
            w.theta_deg = -15.0f;
        }
        else
        {
            /* Level. */
        }
        sim_step (&s, &w);
        saw_climb = saw_climb || (MOTION_CLASS_CLIMBING == s.mout.motion);
        saw_descend = saw_descend || (MOTION_CLASS_DESCENDING == s.mout.motion);
        any_collision = any_collision || s.mout.collision;
    }
    CHECK (saw_climb);
    CHECK (saw_descend);
    CHECK (!any_collision);
    CHECK (MOTION_CLASS_MOVING == s.mout.motion);
}

int main (void)
{
    test_init_validation ();
    test_rejects_bad_input ();
    test_unknown_then_stationary ();
    test_moving_and_accelerating ();
    test_encoder_turn ();
    test_right_turn_through_north ();
    test_pivot_turn ();
    test_tilt_compensation ();
    test_mag_disturbed ();
    test_wheel_slip ();
    test_mag_fallback ();
    test_stale_motion ();
    test_collision ();
    test_reported_braking ();
    test_hump_classes ();

    if (0 == g_failures)
    {
        printf ("\nALL TESTS PASSED\n");
    }
    else
    {
        printf ("\n%d CHECK(S) FAILED\n", g_failures);
    }
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/
