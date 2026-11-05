/** @file test_hump_detect.c
 *
 * @brief Host tests for hump_detect using synthetic accelerometer data.
 *
 * These tests show the code matches the design. They do not show that the
 * default thresholds work on the real car.
 *
 * Synthetic sensor model: the board's forward axis reads
 * sin(slope + mount_tilt) + a_lin / g and the up axis reads
 * cos(slope + mount_tilt), plus optional uniform noise on every axis.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hump_detect.h"

#define DT_S           (0.02f)
#define SPEED_MPS      (0.15f)
#define DEG_TO_RAD     (0.0174532925f)
#define STD_GRAVITY    (9.80665f)
#define HEIGHT_TOL_M   (0.010f)
#define CAL_SAMPLES    (50U)

static int g_failures = 0;
static uint32_t g_lcg = 12345U;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            printf ("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

/* Linear congruential generator, uniform in [-1, 1]. */
static float noise_unit (void)
{
    g_lcg = (g_lcg * 1664525U) + 1013904223U;
    return ((float)(g_lcg >> 8) / 8388608.0f) - 1.0f;
}

typedef struct
{
    float angle_deg;             /* Slope angle of the climb and descent. */
    uint32_t ramp_samples;       /* Samples on each ramp. */
    uint32_t crest_samples;      /* Level samples between the ramps. */
    float noise_g;               /* Uniform noise amplitude per axis. */
    float mount_tilt_deg;        /* Board tilt nose-up relative to the car. */
    float speed_mps;
    uint32_t bad_from;           /* First run sample with bad motion data. */
    uint32_t bad_to;             /* One past the last bad sample. */
    bool bad_is_stale;           /* Bad = valid but 30 ms old, else invalid. */
    uint32_t accel_from;         /* Car acceleration pulse, run samples. */
    uint32_t accel_to;
    float accel_mps2;
    bool report_accel;           /* False: Buddy 2 reports dv/dt as zero. */
} profile_t;

typedef struct
{
    uint32_t cal_done;
    uint32_t started;
    uint32_t finished;
    uint32_t impacts;
    uint32_t complete;
    float last_peak_m;
    float last_peak_pitch_deg;
    float theta0_deg;
    bool last_valid;
    hump_state_t final_state;
} result_t;

static void profile_init (profile_t * p_prof)
{
    memset (p_prof, 0, sizeof (*p_prof));
    p_prof->angle_deg = 15.0f;
    p_prof->ramp_samples = 39U;
    p_prof->crest_samples = 10U;
    p_prof->speed_mps = SPEED_MPS;
    p_prof->report_accel = true;
}

/* Slope for run sample index i of a profile that starts with lead_in flat
 * samples and ends flat. */
static float profile_slope (const profile_t * p_prof, uint32_t i,
                            uint32_t lead_in)
{
    float slope = 0.0f;
    uint32_t t = (i >= lead_in) ? (i - lead_in) : 0U;

    if (i >= lead_in)
    {
        if (t < p_prof->ramp_samples)
        {
            slope = p_prof->angle_deg;
        }
        else if (t < (p_prof->ramp_samples + p_prof->crest_samples))
        {
            slope = 0.0f;
        }
        else if (t < ((2U * p_prof->ramp_samples) + p_prof->crest_samples))
        {
            slope = -p_prof->angle_deg;
        }
        else
        {
            slope = 0.0f;
        }
    }
    return slope;
}

/* Build a sample with the forward component on the configured axis. */
static void make_sample (const hump_config_t * p_cfg, float body_pitch_deg,
                         float lin_accel_mps2, float noise_g,
                         hump_sample_t * p_sample)
{
    float rad = body_pitch_deg * DEG_TO_RAD;
    float fwd = sinf (rad) + (lin_accel_mps2 / STD_GRAVITY)
                + (noise_g * noise_unit ());
    float side = noise_g * noise_unit ();
    float up = cosf (rad) + (noise_g * noise_unit ());

    memset (p_sample, 0, sizeof (*p_sample));
    if (HUMP_AXIS_X == p_cfg->fwd_axis)
    {
        p_sample->ax_g = fwd * (float)p_cfg->fwd_sign;
        p_sample->ay_g = side;
        p_sample->az_g = up;
    }
    else if (HUMP_AXIS_Y == p_cfg->fwd_axis)
    {
        p_sample->ay_g = fwd * (float)p_cfg->fwd_sign;
        p_sample->ax_g = side;
        p_sample->az_g = up;
    }
    else
    {
        p_sample->az_g = fwd * (float)p_cfg->fwd_sign;
        p_sample->ax_g = side;
        p_sample->ay_g = up;
    }
}

static void tally (result_t * p_res, const hump_output_t * p_out)
{
    if (HUMP_EVENT_CAL_DONE == p_out->event)
    {
        p_res->cal_done++;
        p_res->theta0_deg = p_out->theta0_deg;
    }
    if (HUMP_EVENT_STARTED == p_out->event)
    {
        p_res->started++;
    }
    if (HUMP_EVENT_FINISHED == p_out->event)
    {
        p_res->finished++;
        p_res->last_peak_m = p_out->peak_m;
        p_res->last_peak_pitch_deg = p_out->peak_pitch_deg;
        p_res->last_valid = p_out->height_valid;
        if (p_out->complete)
        {
            p_res->complete++;
        }
    }
    if (p_out->impact)
    {
        p_res->impacts++;
    }
    p_res->final_state = p_out->state;
}

/* Calibrate at rest on the profile's mount tilt, then run n_samples of the
 * profile. The profile repeats every `period` samples when period is
 * nonzero (for back-to-back humps). */
static result_t run (const hump_config_t * p_cfg, const profile_t * p_prof,
                     uint32_t lead_in, uint32_t n_samples, uint32_t period)
{
    hump_ctx_t ctx;
    result_t res;
    uint32_t i;

    memset (&res, 0, sizeof (res));
    CHECK (hump_init (&ctx, p_cfg));

    for (i = 0U; i < p_cfg->cal_samples; i++)
    {
        hump_sample_t sample;
        hump_output_t out;

        make_sample (p_cfg, p_prof->mount_tilt_deg, 0.0f, p_prof->noise_g,
                     &sample);
        sample.motion_valid = true;
        CHECK (hump_update (&ctx, &sample, &out));
        tally (&res, &out);
    }
    CHECK (1U == res.cal_done);

    for (i = 0U; i < n_samples; i++)
    {
        hump_sample_t sample;
        hump_output_t out;
        uint32_t idx = (period > 0U) ? (i % period) : i;
        bool in_accel = (i >= p_prof->accel_from) && (i < p_prof->accel_to);
        float lin = in_accel ? p_prof->accel_mps2 : 0.0f;
        bool bad = (i >= p_prof->bad_from) && (i < p_prof->bad_to);

        make_sample (p_cfg,
                     profile_slope (p_prof, idx, lead_in)
                     + p_prof->mount_tilt_deg,
                     lin, p_prof->noise_g, &sample);
        sample.motion_valid = !(bad && !p_prof->bad_is_stale);
        sample.motion_age_s = (bad && p_prof->bad_is_stale) ? 0.030f : 0.005f;
        sample.speed_mps = p_prof->speed_mps;
        sample.ds_m = p_prof->speed_mps * DT_S;
        sample.accel_mps2 = p_prof->report_accel ? lin : 0.0f;

        CHECK (hump_update (&ctx, &sample, &out));
        tally (&res, &out);
    }
    return res;
}

/* Ideal peak height: climb samples * distance per sample * sin(angle). */
static float ideal_peak_m (const profile_t * p_prof)
{
    return (float)p_prof->ramp_samples * p_prof->speed_mps * DT_S
           * sinf (p_prof->angle_deg * DEG_TO_RAD);
}

static void test_init_validation (void)
{
    hump_ctx_t ctx;
    hump_config_t cfg;

    printf ("init_validation\n");
    hump_default_config (&cfg);
    CHECK (hump_init (&ctx, &cfg));
    CHECK (!hump_init (NULL, &cfg));
    CHECK (!hump_init (&ctx, NULL));

    hump_default_config (&cfg);
    cfg.alpha = 0.0f;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.alpha = 1.5f;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.exit_band_deg = cfg.enter_deg;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.enter_samples = 0U;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.fwd_axis = 3U;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.fwd_sign = 0;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.median_len = 0U;
    CHECK (!hump_init (&ctx, &cfg));
    cfg.median_len = 4U;
    CHECK (!hump_init (&ctx, &cfg));
    cfg.median_len = 7U;
    CHECK (!hump_init (&ctx, &cfg));
    cfg.median_len = 3U;
    CHECK (hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.max_motion_age_s = 0.0f;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.cal_samples = 0U;
    CHECK (!hump_init (&ctx, &cfg));
    hump_default_config (&cfg);
    cfg.cal_max_spread_deg = 0.0f;
    CHECK (!hump_init (&ctx, &cfg));
}

static void test_update_rejects_bad_input (void)
{
    hump_ctx_t ctx;
    hump_config_t cfg;
    hump_sample_t sample;
    hump_output_t out;

    printf ("update_rejects_bad_input\n");
    hump_default_config (&cfg);
    CHECK (hump_init (&ctx, &cfg));
    memset (&sample, 0, sizeof (sample));
    sample.az_g = 1.0f;
    sample.motion_valid = true;
    CHECK (hump_update (&ctx, &sample, &out));

    sample.ax_g = NAN;
    CHECK (!hump_update (&ctx, &sample, &out));
    sample.ax_g = 0.0f;
    sample.speed_mps = NAN;
    CHECK (!hump_update (&ctx, &sample, &out));
    sample.speed_mps = 0.0f;
    sample.motion_age_s = -0.001f;
    CHECK (!hump_update (&ctx, &sample, &out));
    sample.motion_age_s = 0.0f;
    sample.ds_m = INFINITY;
    CHECK (!hump_update (&ctx, &sample, &out));

    /* Motion fields are ignored when Buddy 2 marks them invalid. */
    sample.motion_valid = false;
    CHECK (hump_update (&ctx, &sample, &out));
    CHECK (out.motion_stale);

    sample.ds_m = 0.0f;
    CHECK (!hump_update (NULL, &sample, &out));
    CHECK (!hump_update (&ctx, NULL, &out));
    CHECK (!hump_update (&ctx, &sample, NULL));
}

static void test_calibration (void)
{
    hump_ctx_t ctx;
    hump_config_t cfg;
    hump_sample_t sample;
    hump_output_t out;
    uint32_t i;
    uint32_t done_at = 0U;

    printf ("calibration_restarts_when_moving_or_tilting\n");
    hump_default_config (&cfg);
    CHECK (hump_init (&ctx, &cfg));

    /* 30 rest samples, 1 with the wheels turning, then rest again: done
     * after 50 more rest samples, at sample index 80. */
    for (i = 0U; i < 200U; i++)
    {
        make_sample (&cfg, 0.0f, 0.0f, 0.0f, &sample);
        sample.motion_valid = true;
        sample.speed_mps = (30U == i) ? SPEED_MPS : 0.0f;
        CHECK (hump_update (&ctx, &sample, &out));
        if ((HUMP_EVENT_CAL_DONE == out.event) && (0U == done_at))
        {
            done_at = i;
        }
    }
    CHECK (80U == done_at);
    CHECK (out.calibrated);

    /* 30 samples level, then the car is tipped by 3 degrees: the spread
     * check restarts at index 30, so done at index 79 with theta_0 = 3. */
    CHECK (hump_init (&ctx, &cfg));
    done_at = 0U;
    for (i = 0U; i < 200U; i++)
    {
        make_sample (&cfg, (i < 30U) ? 0.0f : 3.0f, 0.0f, 0.0f, &sample);
        sample.motion_valid = true;
        CHECK (hump_update (&ctx, &sample, &out));
        if ((HUMP_EVENT_CAL_DONE == out.event) && (0U == done_at))
        {
            done_at = i;
        }
    }
    CHECK (79U == done_at);
    CHECK (fabsf (out.theta0_deg - 3.0f) < 0.01f);

    /* A shake (|a| far from 1 g) restarts calibration. */
    CHECK (hump_init (&ctx, &cfg));
    done_at = 0U;
    for (i = 0U; i < 200U; i++)
    {
        make_sample (&cfg, 0.0f, 0.0f, 0.0f, &sample);
        if (20U == i)
        {
            sample.az_g = 1.3f;
        }
        CHECK (hump_update (&ctx, &sample, &out));
        if ((HUMP_EVENT_CAL_DONE == out.event) && (0U == done_at))
        {
            done_at = i;
        }
    }
    CHECK (70U == done_at);

    /* Recalibrate clears theta_0 and needs a fresh window. */
    hump_recalibrate (&ctx);
    make_sample (&cfg, 0.0f, 0.0f, 0.0f, &sample);
    CHECK (hump_update (&ctx, &sample, &out));
    CHECK (!out.calibrated);
}

static void test_no_detection_before_calibration (void)
{
    hump_ctx_t ctx;
    hump_config_t cfg;
    uint32_t i;
    uint32_t started = 0U;

    printf ("no_detection_before_calibration\n");
    hump_default_config (&cfg);
    CHECK (hump_init (&ctx, &cfg));
    /* Moving up a slope without calibration: the wheels are turning, so
     * calibration never completes and no hump is reported. */
    for (i = 0U; i < 200U; i++)
    {
        hump_sample_t sample;
        hump_output_t out;

        make_sample (&cfg, 15.0f, 0.0f, 0.0f, &sample);
        sample.motion_valid = true;
        sample.speed_mps = SPEED_MPS;
        sample.ds_m = SPEED_MPS * DT_S;
        CHECK (hump_update (&ctx, &sample, &out));
        CHECK (!out.calibrated);
        if (HUMP_EVENT_STARTED == out.event)
        {
            started++;
        }
    }
    CHECK (0U == started);
}

static void test_flat_ground (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("flat_ground_with_noise\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.angle_deg = 0.0f;
    prof.noise_g = 0.01f;
    res = run (&cfg, &prof, 0U, 1000U, 0U);
    CHECK (0U == res.started);
    CHECK (0U == res.finished);
    CHECK (0U == res.impacts);
    CHECK (HUMP_STATE_FLAT == res.final_state);
}

static void test_single_hump (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;
    float ideal;

    printf ("single_hump_clean\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    res = run (&cfg, &prof, 20U, 250U, 0U);
    ideal = ideal_peak_m (&prof);
    printf ("  ideal peak %.1f mm, measured %.1f mm, peak pitch %.1f deg\n",
            (double)(ideal * 1000.0f), (double)(res.last_peak_m * 1000.0f),
            (double)res.last_peak_pitch_deg);
    CHECK (1U == res.started);
    CHECK (1U == res.finished);
    CHECK (1U == res.complete);
    CHECK (res.last_valid);
    CHECK (fabsf (res.last_peak_m - ideal) <= HEIGHT_TOL_M);
    CHECK (fabsf (res.last_peak_pitch_deg - prof.angle_deg) < 0.5f);
    CHECK (HUMP_STATE_FLAT == res.final_state);
}

static void test_single_hump_noise (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;
    float ideal;

    printf ("single_hump_noise\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.noise_g = 0.01f;
    res = run (&cfg, &prof, 20U, 250U, 0U);
    ideal = ideal_peak_m (&prof);
    printf ("  ideal peak %.1f mm, measured %.1f mm\n",
            (double)(ideal * 1000.0f), (double)(res.last_peak_m * 1000.0f));
    CHECK (1U == res.started);
    CHECK (1U == res.finished);
    CHECK (fabsf (res.last_peak_m - ideal) <= HEIGHT_TOL_M);
}

static void test_height_sweep (void)
{
    static const float heights_mm[] = { 20.0f, 30.0f, 40.0f, 50.0f };
    hump_config_t cfg;
    uint32_t k;

    printf ("height_sweep (angle 15 deg, 0.15 m/s)\n");
    hump_default_config (&cfg);
    for (k = 0U; k < (sizeof (heights_mm) / sizeof (heights_mm[0])); k++)
    {
        profile_t prof;
        result_t res;
        float ideal;
        float ramp_len_m = (heights_mm[k] / 1000.0f)
                           / sinf (15.0f * DEG_TO_RAD);

        profile_init (&prof);
        prof.ramp_samples = (uint32_t)((ramp_len_m / SPEED_MPS / DT_S) + 0.5f);
        res = run (&cfg, &prof, 20U, 400U, 0U);
        ideal = ideal_peak_m (&prof);
        printf ("  %.0f mm: ideal %.1f, measured %.1f mm\n",
                (double)heights_mm[k], (double)(ideal * 1000.0f),
                (double)(res.last_peak_m * 1000.0f));
        CHECK (1U == res.finished);
        CHECK (fabsf (res.last_peak_m - ideal) <= HEIGHT_TOL_M);
    }
}

static void test_mount_tilt (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;
    float ideal;

    printf ("mount_tilt_removed_by_calibration\n");
    hump_default_config (&cfg);

    /* Board tilted 7 degrees nose-up: above the 6 degree enter threshold,
     * so without theta_0 flat ground would look like a climb. */
    profile_init (&prof);
    prof.angle_deg = 0.0f;
    prof.mount_tilt_deg = 7.0f;
    prof.noise_g = 0.01f;
    res = run (&cfg, &prof, 0U, 500U, 0U);
    printf ("  theta_0 %.2f deg (true 7.00)\n", (double)res.theta0_deg);
    CHECK (fabsf (res.theta0_deg - 7.0f) < 0.3f);
    CHECK (0U == res.started);

    /* Same tilt with a real hump: height still matches. */
    profile_init (&prof);
    prof.mount_tilt_deg = 7.0f;
    res = run (&cfg, &prof, 20U, 250U, 0U);
    ideal = ideal_peak_m (&prof);
    printf ("  with hump: ideal %.1f mm, measured %.1f mm\n",
            (double)(ideal * 1000.0f), (double)(res.last_peak_m * 1000.0f));
    CHECK (1U == res.finished);
    CHECK (1U == res.complete);
    CHECK (fabsf (res.last_peak_m - ideal) <= HEIGHT_TOL_M);
}

static void test_accel_subtraction (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("dv_dt_subtraction_rejects_speed_up\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.angle_deg = 0.0f;
    /* +1.5 m/s^2 for 200 ms looks like about 8.8 degrees nose-up. */
    prof.accel_from = 50U;
    prof.accel_to = 60U;
    prof.accel_mps2 = 1.5f;

    res = run (&cfg, &prof, 0U, 200U, 0U);
    CHECK (0U == res.started);

    /* Without dv/dt from Buddy 2 the same run gives a false hump start. */
    prof.report_accel = false;
    res = run (&cfg, &prof, 0U, 200U, 0U);
    CHECK (1U == res.started);
}

static void test_median_rejects_spike (void)
{
    hump_config_t cfg;
    hump_ctx_t ctx;
    uint32_t pass;

    printf ("median_rejects_two_sample_spike\n");
    for (pass = 0U; pass < 2U; pass++)
    {
        uint32_t i;
        uint32_t started = 0U;

        hump_default_config (&cfg);
        cfg.median_len = (0U == pass) ? 5U : 1U; cfg.entry_max_dev_g = 1.0f;
        CHECK (hump_init (&ctx, &cfg));
        for (i = 0U; i < (CAL_SAMPLES + 50U); i++)
        {
            hump_sample_t sample;
            hump_output_t out;

            make_sample (&cfg, 0.0f, 0.0f, 0.0f, &sample);
            sample.motion_valid = true;
            if (i >= CAL_SAMPLES)
            {
                sample.speed_mps = SPEED_MPS;
                sample.ds_m = SPEED_MPS * DT_S;
            }
            if ((i == (CAL_SAMPLES + 20U)) || (i == (CAL_SAMPLES + 21U)))
            {
                sample.ax_g = 0.6f; /* Bump on the forward axis. */
            }
            CHECK (hump_update (&ctx, &sample, &out));
            if (HUMP_EVENT_STARTED == out.event)
            {
                started++;
            }
        }
        /* Median 5 removes it; with no median the spike starts a hump. */
        CHECK (((0U == pass) ? 0U : 1U) == started);
    }
}

static void test_axis_mapping (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t base;
    result_t res;
    uint8_t axis;
    int8_t sign;

    printf ("axis_mapping\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    base = run (&cfg, &prof, 20U, 250U, 0U);

    for (axis = HUMP_AXIS_X; axis <= HUMP_AXIS_Z; axis++)
    {
        for (sign = -1; sign <= 1; sign += 2)
        {
            hump_default_config (&cfg);
            cfg.fwd_axis = axis;
            cfg.fwd_sign = sign;
            res = run (&cfg, &prof, 20U, 250U, 0U);
            CHECK (1U == res.finished);
            CHECK (fabsf (res.last_peak_m - base.last_peak_m) < 0.0005f);
        }
    }

    /* dv/dt correction follows the configured axis and sign. */
    hump_default_config (&cfg);
    cfg.fwd_axis = HUMP_AXIS_Y;
    cfg.fwd_sign = -1;
    profile_init (&prof);
    prof.angle_deg = 0.0f;
    prof.accel_from = 50U;
    prof.accel_to = 60U;
    prof.accel_mps2 = 1.5f;
    res = run (&cfg, &prof, 0U, 200U, 0U);
    CHECK (0U == res.started);
}

static void test_downhill_only (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("downhill_only_is_ignored\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.angle_deg = -15.0f;
    prof.ramp_samples = 100U;
    prof.crest_samples = 0U;
    res = run (&cfg, &prof, 0U, 100U, 0U);
    CHECK (0U == res.started);
}

static void test_motion_invalid (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("invalid_motion_marks_height_invalid\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.bad_from = 40U;
    prof.bad_to = 45U;
    res = run (&cfg, &prof, 20U, 250U, 0U);
    CHECK (1U == res.finished);
    CHECK (!res.last_valid);
}

static void test_motion_stale (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("motion_older_than_25ms_marks_height_invalid\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.bad_from = 40U;
    prof.bad_to = 41U;
    prof.bad_is_stale = true;
    res = run (&cfg, &prof, 20U, 250U, 0U);
    CHECK (1U == res.finished);
    CHECK (!res.last_valid);
}

static void test_below_min_speed (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("below_min_speed_marks_height_invalid\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    prof.speed_mps = 0.05f;
    res = run (&cfg, &prof, 20U, 400U, 0U);
    CHECK (1U == res.finished);
    CHECK (!res.last_valid);
}

static void test_incomplete_hump (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("climb_without_descent_gives_up\n");
    hump_default_config (&cfg);
    cfg.max_hump_samples = 60U;
    profile_init (&prof);
    prof.ramp_samples = 39U;
    prof.crest_samples = 10000U;
    res = run (&cfg, &prof, 20U, 300U, 0U);
    CHECK (1U == res.started);
    CHECK (1U == res.finished);
    CHECK (0U == res.complete);
    CHECK (HUMP_STATE_FLAT == res.final_state);
}

static void test_two_humps (void)
{
    hump_config_t cfg;
    profile_t prof;
    result_t res;

    printf ("two_humps_back_to_back\n");
    hump_default_config (&cfg);
    profile_init (&prof);
    /* Period: 20 flat + 39 + 10 + 39 + 40 flat = 148 samples. */
    res = run (&cfg, &prof, 20U, 148U * 2U, 148U);
    CHECK (2U == res.started);
    CHECK (2U == res.finished);
    CHECK (2U == res.complete);
}

static void test_impact (void)
{
    hump_ctx_t ctx;
    hump_config_t cfg;
    hump_sample_t sample;
    hump_output_t out;

    printf ("impact_flag\n");
    hump_default_config (&cfg);
    CHECK (hump_init (&ctx, &cfg));
    memset (&sample, 0, sizeof (sample));
    sample.az_g = 1.0f;
    CHECK (hump_update (&ctx, &sample, &out));
    CHECK (!out.impact);
    sample.az_g = 1.8f;
    CHECK (hump_update (&ctx, &sample, &out));
    CHECK (out.impact);
}

int main (void)
{
    test_init_validation ();
    test_update_rejects_bad_input ();
    test_calibration ();
    test_no_detection_before_calibration ();
    test_flat_ground ();
    test_single_hump ();
    test_single_hump_noise ();
    test_height_sweep ();
    test_mount_tilt ();
    test_accel_subtraction ();
    test_median_rejects_spike ();
    test_axis_mapping ();
    test_downhill_only ();
    test_motion_invalid ();
    test_motion_stale ();
    test_below_min_speed ();
    test_incomplete_hump ();
    test_two_humps ();
    test_impact ();

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
