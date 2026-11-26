/*
 *	test_attitude.c
 *	Host tests for lift and overturn detection: eight traces,
 *	and a bump that must not count.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

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
    p_rig->in.b_wheels_still = false;
    p_rig->in.b_on_hump = false;
    p_rig->out.state   = ATTITUDE_OK;
}

/* One sample of a vector of size mag at pitch (nose up) and roll (left down). */
static void sample (rig_t * p_rig, float mag, float pitch_deg, float roll_deg, float jitter)
{
    float const p = pitch_deg * DEG_TO_RAD;
    float const r = roll_deg * DEG_TO_RAD;

    p_rig->in.ax_g = (mag * sinf (p)) + (jitter * noise (p_rig));
    p_rig->in.ay_g = -(mag * sinf (r)) + (jitter * noise (p_rig));
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

    /* An upward onset precedes the alternating loads during the carry. */
    run (&rig, 16U, 1.2f, 0.0f, 0.0f, 0.0f);
    for (idx = 0U; idx < 60U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.2f : 0.82f, 2.0f, 1.0f, 0.02f);
    }
    CHECK (ATTITUDE_LIFTED == rig.out.state);

    /* A level pose with wheel activity cannot clear a lift. */
    for (idx = 0U; idx < rig.ctx.cfg.clear_samples; idx++)
    {
        sample (&rig, 1.0f, 0.0f, 0.0f, 0.01f);
        CHECK (ATTITUDE_LIFTED == rig.out.state && !rig.out.level_still);
    }
    /* Set down. The full level and still window clears the lift. */
    rig.in.b_wheels_still = true;
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
    sample (&rig, 0.4f, 0.0f, 0.0f, 0.01f);
    for (idx = 1U; idx < rig.ctx.cfg.lift_samples; idx++)
    {
        sample (&rig, 1.08f, 30.0f, 0.0f, 0.01f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
    sample (&rig, 1.08f, 30.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);
}

/* Supported, a loaded tilted carry lifts. While a landing settles, only a
 * gravity-sized tilted sample counts. */
static void test_tilt_gate (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    run (&rig, rig.ctx.cfg.lift_samples, 1.2f, 37.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);

    rig_init (&rig);
    run (&rig, 50U, 1.0f, 0.0f, 0.0f, 0.02f);
    sample (&rig, 0.4f, 0.0f, 0.0f, 0.01f);
    for (idx = 0U; idx < rig.ctx.cfg.lift_samples; idx++)
    {
        sample (&rig, 1.3f, 30.0f, 0.0f, 0.01f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
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

static void test_interrupted_floor_tilt (void)
{
    rig_t rig;

    for (unsigned free_cue = 0U; free_cue < 2U; free_cue++)
    {
        rig_init (&rig);
        rig.in.b_wheels_free = (free_cue != 0U);
        /* The November floor recording contains both of these magnitudes
         * and a near-envelope roll. The intervening 50 Hz samples here
         * are synthetic. Ordinary driving need not meet set-down limits. */
        run (&rig, 8U, 1.044f, -6.5f, 19.73f, 0.0f);
        CHECK (ATTITUDE_CANDIDATE == rig.out.state && rig.out.tilt_deg > rig.ctx.cfg.tilt_envelope_deg);
        run (&rig, 4U, 1.070f, 2.0f, 4.67f, 0.0f);
        CHECK (ATTITUDE_CANDIDATE == rig.out.state && rig.out.tilt_deg < rig.ctx.cfg.tilt_envelope_deg);
        for (unsigned i = 1U; i < rig.ctx.cfg.lift_samples; i++)
        {
            sample (&rig, 1.044f, -6.5f, 19.73f, 0.0f);
            CHECK (ATTITUDE_CANDIDATE == rig.out.state);
        }
        sample (&rig, 1.044f, -6.5f, 19.73f, 0.0f);
        CHECK (ATTITUDE_LIFTED == rig.out.state);
    }
}

static void test_floor_bounce_ends_before_confirmation (void)
{
    rig_t rig;

    rig_init (&rig);
    /* A symmetric acceleration pulse returns a supported chassis to its
     * initial height and vertical speed. At 50 Hz it lasts 160 ms. */
    for (unsigned i = 0U; i < 8U; i++)
    {
        float const accel = (i < 2U || i >= 6U) ? 0.20f : -0.20f;
        sample (&rig, 1.044f + accel, 0.0f, 0.0f, 0.0f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
    for (unsigned i = 0U; i < rig.ctx.cfg.landing_settle_samples + rig.ctx.cfg.candidate_clear; i++)
    {
        sample (&rig, 1.044f, 0.0f, 0.0f, 0.0f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
    CHECK (ATTITUDE_OK == rig.out.state);
}

static void test_drop_and_landing (void)
{
    rig_t rig;
    rig_init (&rig);
    rig.in.b_wheels_still = true;
    for (unsigned i = 0; i < 100U; i++)
    {
        float const load = i < 20U ? 1.0f : i < 39U ? 0.35f : i < 55U ? 1.45f : 1.0f;
        sample (&rig, load, 0.0f, 0.0f, 0.0f);
        CHECK (rig.out.state != ATTITUDE_LIFTED);
    }
    /* Unloaded acceleration can point sideways. Its direction cannot
     * establish that the chassis is resting on its side. */
    for (unsigned i = 0U; i < 15U; i++)
    {
        sample (&rig, 0.04f, 0.0f, 85.0f, 0.0f);
        CHECK (rig.out.state != ATTITUDE_OVERTURNED && rig.out.state != ATTITUDE_LIFTED);
    }
    run (&rig, 16U, 1.0f, 0.0f, 0.0f, 0.0f);
    /* Pickup after the landing has settled, with the same stationary wheels. */
    run (&rig, 8U, 1.08f, 0.0f, 0.0f, 0.0f);
    CHECK (rig.out.state == ATTITUDE_LIFTED);
}

static void test_parked_vertical_onset (void)
{
    rig_t rig;
    rig_init (&rig);
    /* Calibrated rest may differ from 1 g and from board Z. */
    rig.in.g0[0] = 1.044f * 0.60f;
    rig.in.g0[2] = 1.044f * 0.80f;
    rig.in.b_wheels_still = true;
    for (unsigned i = 0U; i < 40U; i++)
    {
        float const load = i < 33U ? 1.054f : 1.124f;
        rig.in.ax_g = load * 0.60f;
        rig.in.ay_g = 0.0f;
        rig.in.az_g = load * 0.80f;
        CHECK (attitude_update (&rig.ctx, &rig.in, &rig.out));
        if (i < 33U) CHECK (ATTITUDE_LIFTED != rig.out.state);
    }
    CHECK (ATTITUDE_LIFTED == rig.out.state);
    rig_init (&rig);
    run (&rig, 40U, 1.08f, 0.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_LIFTED != rig.out.state);
}

static void test_level_hand_pickup (void)
{
    /* One smooth upward pickup ends level about 250 mm higher after 0.8 s.
     * A pickup during travel must also work before the wheels slow down. */
    for (unsigned parked = 0U; parked <= 1U; ++parked)
    {
        rig_t rig;
        rig_init (&rig);
        rig.in.b_wheels_still = (parked != 0U);
        for (unsigned i = 1U; i <= 40U; i++)
        {
            float const accel = 0.25f * sinf (360.0f * DEG_TO_RAD * (float)i / 40.0f);
            sample (&rig, 1.0f + accel, 0.0f, 0.0f, 0.0f);
        }
        CHECK (ATTITUDE_LIFTED == rig.out.state);
    }
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

    run (&rig, rig.ctx.cfg.clear_samples, 1.0f, 65.0f, 0.0f, 0.0f);
    CHECK (fabsf (rig.out.roll_deg) < 1.0f && !rig.out.level_still);
    run (&rig, rig.ctx.cfg.clear_samples, 1.2f, 0.0f, 0.0f, 0.0f);
    CHECK (!rig.out.level_still);
    rig.in.b_wheels_still = true;
    run (&rig, rig.ctx.cfg.clear_samples - 1U, 1.0f, 0.0f, 0.0f, 0.0f);
    CHECK (!rig.out.level_still);
    sample (&rig, 1.0f, 0.0f, 0.0f, 0.0f);
    CHECK (rig.out.level_still && ATTITUDE_OVERTURNED == rig.out.state);
    /* Upright again: it holds until a reset. */
    run (&rig, 200U, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OVERTURNED == rig.out.state);
    attitude_reset (&rig.ctx);
    run (&rig, 5U, 1.0f, 0.0f, 0.0f, 0.01f);
    CHECK (ATTITUDE_OK == rig.out.state);
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

static void test_zero_reference_keeps_lift (void)
{
    rig_t rig;
    bool rejected = true;

    rig_init (&rig);
    run (&rig, rig.ctx.cfg.lift_samples, 1.0f, 30.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);

    rig.in.g0[2] = 0.0f;
    rig.in.ax_g  = 0.0f;
    rig.in.ay_g  = 0.0f;
    rig.in.az_g  = 1.0f;
    for (unsigned i = 0U; i < rig.ctx.cfg.clear_samples; i++)
    {
        if (attitude_update (&rig.ctx, &rig.in, &rig.out))
        {
            rejected = false;
        }
    }
    CHECK (rejected);
    CHECK (ATTITUDE_LIFTED == rig.ctx.state && ATTITUDE_LIFTED == rig.out.state);

    rig.in.g0[2] = 1.0f;
    sample (&rig, 0.0f, 0.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_LIFTED == rig.out.state && rig.out.amag_g == 0.0f);
    rig.in.b_wheels_still = true;
    run (&rig, rig.ctx.cfg.clear_samples, 1.0f, 0.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_OK == rig.out.state);
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

/* A landing needs at least one fresh, supported observation to rearm. */
static void test_landing_settle_config (void)
{
    attitude_config_t cfg;
    attitude_ctx_t ctx;
    attitude_default_config (&cfg);
    cfg.landing_settle_samples = 0U;
    CHECK (!attitude_init (&ctx, &cfg));
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

/* During a hump the unsettled cue does not count; a tilt breach still does. */
static void test_hump_flag_mutes_unsettled_cue (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    run (&rig, 100U, 1.0f, 0.0f, 0.0f, 0.02f);
    rig.in.b_on_hump = true;
    for (idx = 0U; idx < 60U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.2f : 0.82f, 2.0f, 1.0f, 0.02f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }

    /* Alternating vibration alone has no sustained upward onset. */
    rig.in.b_on_hump = false;
    for (idx = 0U; idx < 60U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.2f : 0.82f, 2.0f, 1.0f, 0.02f);
    }
    CHECK (ATTITUDE_LIFTED != rig.out.state);
    run (&rig, 20U, 1.0f, 0.0f, 0.0f, 0.0f);
    run (&rig, 20U, 1.2f, 2.0f, 1.0f, 0.01f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);

    /* A tilt breach on a hump still lifts. */
    rig_init (&rig);
    rig.in.b_on_hump = true;
    run (&rig, 40U, 1.0f, 30.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);
}

/* Cue samples counted before a hump do not add to the hold after it. */
static void test_hump_ending_mid_candidate_is_not_a_lift (void)
{
    rig_t    rig;
    unsigned idx = 0U;

    rig_init (&rig);
    rig.in.b_wheels_free = true;
    run (&rig, 100U, 1.0f, 0.0f, 0.0f, 0.02f);
    for (idx = 0U; idx < 9U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.3f : 0.7f, 0.0f, 0.0f, 0.0f);
    }
    CHECK (ATTITUDE_CANDIDATE == rig.out.state);

    rig.in.b_on_hump = true;
    for (idx = 0U; idx < 20U; idx++)
    {
        sample (&rig, ((idx % 2U) == 0U) ? 1.3f : 0.7f, 2.0f, 0.0f, 0.0f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }

    /* The hump ends; the bumps stop. Only the hold of the unsettled cue is left. */
    rig.in.b_on_hump = false;
    for (idx = 0U; idx < 30U; idx++)
    {
        sample (&rig, 1.0f, 0.0f, 0.0f, 0.01f);
        CHECK (ATTITUDE_LIFTED != rig.out.state);
    }

    /* A real lift after the hump still counts in full. */
    run (&rig, 40U, 1.0f, 30.0f, 0.0f, 0.02f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);
}

static void test_hump_keeps_wheels_free_candidate (void)
{
    rig_t rig;

    rig_init (&rig);
    rig.in.b_on_hump = true;
    rig.in.b_wheels_free = true;
    run (&rig, 60U, 1.0f, 12.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_CANDIDATE == rig.out.state);
}

static void test_hump_keeps_tilt_breach (void)
{
    rig_t rig;

    rig_init (&rig);
    rig.in.b_on_hump = true;
    run (&rig, 40U, 1.0f, 30.0f, 0.0f, 0.0f);
    CHECK (ATTITUDE_LIFTED == rig.out.state);
}

int main (int argc, char * argv[])
{
    static const struct
    {
        char const * name;
        void (*test) (void);
    } selectable[] = {
        {"hump-flag", test_hump_flag_mutes_unsettled_cue},
        {"hump-ends-mid-candidate", test_hump_ending_mid_candidate_is_not_a_lift},
        {"hump-wheels-free", test_hump_keeps_wheels_free_candidate},
        {"hump-tilt", test_hump_keeps_tilt_breach},
        {"lift-set-down", test_lift_carry_level},
        {"tilt-gate", test_tilt_gate},
        {"interrupted-floor-tilt", test_interrupted_floor_tilt},
        {"floor-bounce-ends", test_floor_bounce_ends_before_confirmation},
        {"level-hand-pickup", test_level_hand_pickup},
        {"overturn-tilt", test_tip_on_side},
        {"uncalibrated-placement", test_placement_by_hand},
        {"invalid-acceleration", test_failed_read_keeps_state},
        {"zero-gravity-reference", test_zero_reference_keeps_lift},
    };
    size_t i;

    if (2 == argc)
    {
        for (i = 0U; i < (sizeof (selectable) / sizeof (selectable[0])); ++i)
        {
            if (0 == strcmp (argv[1], selectable[i].name))
            {
                selectable[i].test ();
                return (0U == g_fail) ? 0 : 1;
            }
        }
    }
    if (1 != argc)
    {
        (void)fprintf (stderr, "usage: %s [case]\n", argv[0]);
        return 2;
    }

    test_flat_run ();
    test_hump_is_not_a_lift ();
    test_lift_carry_level ();
    test_lift_and_tilt ();
    test_tilt_gate ();
    test_short_breach_is_ignored ();
    test_bump_is_not_a_lift ();
    test_interrupted_floor_tilt ();
    test_floor_bounce_ends_before_confirmation ();
    test_level_hand_pickup ();
    test_drop_and_landing ();
    test_parked_vertical_onset ();
    test_tip_on_side ();
    test_upside_down ();
    test_placement_by_hand ();
    test_failed_read_keeps_state ();
    test_zero_reference_keeps_lift ();
    test_config_checks ();
    test_landing_settle_config ();
    test_wheels_free_only_candidate ();
    test_hump_flag_mutes_unsettled_cue ();
    test_hump_ending_mid_candidate_is_not_a_lift ();
    test_hump_keeps_wheels_free_candidate ();
    test_hump_keeps_tilt_breach ();

    if (g_fail != 0U)
    {
        (void)printf ("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf ("test_attitude: all checks passed\n");
    return 0;
}
