/*
 *	test_attitude.c
 *	Host tests for lift and overturn detection: the eight traces of
 *	artifacts/notes/imu-lift-overturn.md, and a bump that must not count.
 */

#include <math.h>
#include <stdio.h>

#include "attitude.h"

static unsigned g_fail;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            (void)printf ("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
            g_fail++;                                                        \
        }                                                                    \
    } while (0)

#define DEG_TO_RAD (0.01745329252f)

typedef struct
{
    attitude_ctx_t    ctx;
    attitude_output_t out;
    attitude_input_t  in;
    unsigned          seed;
} rig_t;

/* A tiny repeatable noise source, +-1. */
static float noise (rig_t * p_rig)
{
    p_rig->seed = (p_rig->seed * 1103515245U) + 12345U;
    return (((float)((p_rig->seed >> 16U) & 0x7FFFU)) / 16383.5f) - 1.0f;
}

static void rig_init (rig_t * p_rig)
{
    attitude_config_t cfg;

    attitude_default_config (&cfg);
    CHECK (attitude_init (&p_rig->ctx, &cfg));
    p_rig->seed        = 7U;
    p_rig->in.g0[0]    = 0.0f;
    p_rig->in.g0[1]    = 0.0f;
    p_rig->in.g0[2]    = 1.0f;
    p_rig->in.fwd[0]   = 1.0f;
    p_rig->in.fwd[1]   = 0.0f;
    p_rig->in.fwd[2]   = 0.0f;
    p_rig->in.calibrated = true;
    p_rig->in.b_wheels_free = false;
    p_rig->out.state   = ATTITUDE_OK;
}

/* One sample of a vector of size mag at pitch (nose up) and roll (left down). */
static void sample (rig_t * p_rig, float mag, float pitch_deg, float roll_deg, float jitter)
{
    float const p = pitch_deg * DEG_TO_RAD;
    float const r = roll_deg * DEG_TO_RAD;

    p_rig->in.ax_g = (mag * sinf (p)) + (jitter * noise (p_rig));
    p_rig->in.ay_g = (mag * sinf (r)) + (jitter * noise (p_rig));
    p_rig->in.az_g = (mag * cosf (p) * cosf (r)) + (jitter * noise (p_rig));
    CHECK (attitude_update (&p_rig->ctx, &p_rig->in, &p_rig->out));
}

static void run (rig_t * p_rig, unsigned count, float mag, float pitch, float roll, float jitter)
{
    unsigned idx = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        sample (p_rig, mag, pitch, roll, jitter);
    }
}

static void test_flat_run (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    for (idx = 0U; idx < 500U; idx++)
    {
        sample (&rig, 1.0f, 0.0f, 0.0f, 0.03f);
        CHECK (ATTITUDE_OK == rig.out.state);
    }
}

static void test_hump_is_not_a_lift (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    for (idx = 0U; idx < 150U; idx++)
    {
        float const pitch = (idx < 50U) ? 12.0f : ((idx < 100U) ? 0.0f : -12.0f);

        sample (&rig, 1.0f, pitch, 0.0f, 0.02f);
        CHECK (ATTITUDE_OK == rig.out.state);
    }
}

static void test_lift_carry_level (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    run (&rig, 100U, 1.0f, 0.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_OK == rig.out.state);

    /* A hand lifts and carries the car level for 1.2 s. The magnitude swings. */
    for (idx = 0U; idx < 60U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.2f : 0.82f, 2.0f, 1.0f, 0.02f);
    }
    CHECK (ATTITUDE_LIFTED == rig.out.state);

    /* Set down. Level and still for 1 s clears it, not before. */
    for (idx = 0U; idx < (rig.ctx.cfg.clear_samples - 1U); idx++)
    {
        sample (&rig, 1.0f, 0.0f, 0.0f, 0.01f);
        CHECK (ATTITUDE_LIFTED == rig.out.state);
    }
    sample (&rig, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OK == rig.out.state);
    CHECK (rig.out.changed);
}

static void test_lift_and_tilt (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    for (idx = 1U; idx < rig.ctx.cfg.lift_samples; idx++)
    {
        sample (&rig, 1.0f, 30.0f, 0.0f, 0.01f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
    sample (&rig, 1.0f, 30.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);
}

static void test_short_breach_is_ignored (void)
{
    rig_t rig;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    run (&rig, 8U, 1.0f, 30.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_CANDIDATE == rig.out.state);
    run (&rig, 20U, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OK == rig.out.state);
}

static void test_bump_is_not_a_lift (void)
{
    rig_t rig;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    run (&rig, 3U, 1.35f, 0.0f, 0.0f, 0.0f);
    run (&rig, 100U, 1.0f, 0.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_OK == rig.out.state);
}

static void test_tip_on_side (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    for (idx = 1U; idx < rig.ctx.cfg.overturn_samples; idx++)
    {
        sample (&rig, 1.0f, 0.0f, 90.0f, 0.0f);
        CHECK (ATTITUDE_OVERTURNED != rig.out.state);
    }
    sample (&rig, 1.0f, 0.0f, 90.0f, 0.0f);
    CHECK (ATTITUDE_OVERTURNED == rig.out.state);
    CHECK (fabsf (rig.out.roll_deg) > 80.0f);

    /* One unstable sample restarts the upright window. */
    run (&rig, rig.ctx.cfg.clear_samples - 1U, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OVERTURNED == rig.out.state);
    sample (&rig, 1.0f, 10.0f, 0.0f, 0.01f);
    run (&rig, rig.ctx.cfg.clear_samples - 1U, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OVERTURNED == rig.out.state);
    sample (&rig, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OK == rig.out.state);
    CHECK (rig.out.changed);
}

static void test_upside_down (void)
{
    rig_t rig;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    run (&rig, 10U, -1.0f, 0.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_OVERTURNED == rig.out.state);
    CHECK (rig.out.tilt_deg > 170.0f);
}

static void test_placement_by_hand (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    rig.in.calibrated = false;
    for (idx = 0U; idx < 150U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.4f : 0.6f, 25.0f, 40.0f, 0.1f);
        CHECK (ATTITUDE_OK == rig.out.state);
        CHECK (!rig.out.calibrated);
    }
    /* The rest calibration completes. No fault follows. */
    rig.in.calibrated = true;
    run (&rig, 100U, 1.0f, 0.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_OK == rig.out.state);
    CHECK (rig.out.calibrated);
}

static void test_failed_read_keeps_state (void)
{
    rig_t rig;
    attitude_output_t before;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    before = rig.out;

    /* A failed read is not fed. A non-finite value is refused and changes nothing. */
    rig.in.ax_g = NAN;
    CHECK (!attitude_update (&rig.ctx, &rig.in, &rig.out));
    CHECK (ATTITUDE_OK == rig.ctx.state);
    CHECK (before.state == ATTITUDE_OK);
    CHECK (!attitude_update (NULL, &rig.in, &rig.out));
}

static void test_config_checks (void)
{
    attitude_ctx_t    ctx;
    attitude_config_t cfg;

    attitude_default_config (&cfg);
    CHECK (attitude_init (&ctx, &cfg));
    cfg.overturn_tilt_deg = 10.0f; /* below the envelope */
    CHECK (!attitude_init (&ctx, &cfg));
    CHECK (!attitude_init (NULL, &cfg));
}

static void test_wheels_free_only_candidate (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    /* The wheels run free on a level car: CANDIDATE, held, never LIFTED. */
    rig_init (&rig);
    rig.in.b_wheels_free = true;
    for (idx = 0U; idx < 600U; idx++)
    {
        sample (&rig, 1.0f, 0.0f, 0.0f, 0.02f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
    CHECK (ATTITUDE_CANDIDATE == rig.out.state);

    /* The cue ends: the state falls back to OK. */
    rig.in.b_wheels_free = false;
    run (&rig, 20U, 1.0f, 0.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_OK == rig.out.state);

    /* The cue with a real breach still reaches LIFTED. */
    rig.in.b_wheels_free = true;
    run (&rig, 40U, 1.0f, 30.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);
}

int main (void)
{
    test_flat_run ();
    test_hump_is_not_a_lift ();
    test_lift_carry_level ();
    test_lift_and_tilt ();
    test_short_breach_is_ignored ();
    test_bump_is_not_a_lift ();
    test_tip_on_side ();
    test_upside_down ();
    test_placement_by_hand ();
    test_failed_read_keeps_state ();
    test_config_checks ();
    test_wheels_free_only_candidate ();

    if (g_fail != 0U)
    {
        (void)printf ("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf ("test_attitude: all checks passed\n");
    return 0;
}
