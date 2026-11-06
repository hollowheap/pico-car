/** @file test_hump_msg.c
 *
 * @brief Host tests for the packed 32-bit Buddy 4 message.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hump_detect.h"
#include "hump_msg.h"

#define DT_S        (0.02f)
#define SPEED_MPS   (0.15f)
#define DEG_TO_RAD  (0.0174532925f)

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

static void base_fields (hump_fields_t * p_f)
{
    memset (p_f, 0, sizeof (*p_f));
    p_f->state = HUMP_STATE_FLAT;
    p_f->motion = MOTION_CLASS_MOVING;
}

static void test_size_and_layout (void)
{
    hump_fields_t f;
    packed_hump_t w = 0U;

    printf ("size_and_layout\n");
    CHECK (4U == sizeof (packed_hump_t));
    CHECK (32U == PHYSICS_MBF_BUFSZ);
    CHECK (0x00E0U == PHYSICS_SHARED_FAULT_MASK);

    /* All-zero fields: pitch 0 is offset code 64 at bits 8-14. */
    base_fields (&f);
    f.motion = MOTION_CLASS_UNKNOWN;
    CHECK (hump_pack (&f, &w));
    CHECK ((64UL << 8U) == w);

    /* Each field lands in its documented bits. */
    base_fields (&f);
    f.state = HUMP_STATE_DESCEND;            /* 3 at bits 0-1   */
    f.motion = MOTION_CLASS_IMPACT;             /* 7 at bits 2-4   */
    f.faults = PHYSICS_FAULT_I2C | PHYSICS_FAULT_STALE; /* 3 at bits 5-7  */
    f.pitch_deg = 31.5f;                     /* 127 at bits 8-14 */
    f.hump_peak_m = 0.126f;                  /* 126 at 15-21    */
    f.run_peak_m = 0.001f;                   /* 1 at 22-28      */
    f.hump_active = true;                    /* bit 29          */
    f.height_valid = true;                   /* bit 31          */
    CHECK (hump_pack (&f, &w));
    CHECK (0x3UL == (w & 0x3UL));
    CHECK ((0x7UL << 2U) == (w & (0x7UL << 2U)));
    CHECK ((0x3UL << 5U) == (w & (0x7UL << 5U)));
    CHECK ((127UL << 8U) == (w & (0x7FUL << 8U)));
    CHECK ((126UL << 15U) == (w & (0x7FUL << 15U)));
    CHECK ((1UL << 22U) == (w & (0x7FUL << 22U)));
    CHECK (0x20000000UL == (w & 0x20000000UL));
    CHECK (0UL == (w & 0x40000000UL));
    CHECK (0x80000000UL == (w & 0x80000000UL));
}

static void test_round_trip (void)
{
    static const float pitches[] = { -32.0f, -10.5f, -0.5f, 0.0f, 0.5f,
                                     6.0f, 15.0f, 31.5f };
    hump_fields_t f;
    hump_fields_t g;
    packed_hump_t w;
    uint32_t i;
    uint32_t mm;

    printf ("round_trip\n");
    for (i = 0U; i < (sizeof (pitches) / sizeof (pitches[0])); i++)
    {
        base_fields (&f);
        f.pitch_deg = pitches[i];
        CHECK (hump_pack (&f, &w));
        CHECK (hump_unpack (w, &g));
        CHECK (fabsf (g.pitch_deg - pitches[i]) < 0.001f);
        CHECK (0U == g.faults);
    }

    for (mm = 0U; mm <= HUMP_HEIGHT_MAX_MM; mm++)
    {
        base_fields (&f);
        f.hump_peak_m = (float)mm / 1000.0f;
        f.run_peak_m = (float)(HUMP_HEIGHT_MAX_MM - mm) / 1000.0f;
        f.height_valid = true;
        CHECK (hump_pack (&f, &w));
        CHECK (hump_unpack (w, &g));
        CHECK (fabsf (g.hump_peak_m - f.hump_peak_m) < 0.0001f);
        CHECK (fabsf (g.run_peak_m - f.run_peak_m) < 0.0001f);
        CHECK (g.height_valid);
    }

    base_fields (&f);
    f.state = HUMP_STATE_CREST;
    f.motion = MOTION_CLASS_CLIMBING;
    f.faults = PHYSICS_FAULT_STALE;
    f.finished = true;
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (HUMP_STATE_CREST == g.state);
    CHECK (MOTION_CLASS_CLIMBING == g.motion);
    CHECK (PHYSICS_FAULT_STALE == g.faults);
    CHECK (g.finished && !g.hump_active);
}

static void test_rounding (void)
{
    hump_fields_t f;
    hump_fields_t g;
    packed_hump_t w;

    printf ("rounding_to_nearest_step\n");
    base_fields (&f);
    f.pitch_deg = 6.24f;       /* -> 6.0 */
    f.hump_peak_m = 0.03049f;  /* -> 30 mm */
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (fabsf (g.pitch_deg - 6.0f) < 0.001f);
    CHECK (fabsf (g.hump_peak_m - 0.030f) < 0.0001f);

    f.pitch_deg = 6.26f;       /* -> 6.5 */
    f.hump_peak_m = 0.03051f;  /* -> 31 mm */
    f.run_peak_m = 0.0004f;    /* -> 0 mm */
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (fabsf (g.pitch_deg - 6.5f) < 0.001f);
    CHECK (fabsf (g.hump_peak_m - 0.031f) < 0.0001f);
    CHECK (fabsf (g.run_peak_m) < 0.0001f);

    f.pitch_deg = -6.26f;      /* -> -6.5 */
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (fabsf (g.pitch_deg + 6.5f) < 0.001f);
}

static void test_out_of_range (void)
{
    hump_fields_t f;
    hump_fields_t g;
    packed_hump_t w;

    printf ("out_of_range_is_flagged_not_hidden\n");

    /* Pitch beyond the 7-bit range saturates and raises the tilt fault. */
    base_fields (&f);
    f.pitch_deg = 40.0f;
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (fabsf (g.pitch_deg - HUMP_PITCH_MAX_DEG) < 0.001f);
    CHECK (PHYSICS_FAULT_TILT == g.faults);
    f.pitch_deg = -32.3f;
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (fabsf (g.pitch_deg - HUMP_PITCH_MIN_DEG) < 0.001f);
    CHECK (PHYSICS_FAULT_TILT == g.faults);
    f.pitch_deg = -32.2f;      /* Rounds to -32.0: still in range. */
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (0U == g.faults);

    /* Height above 126 mm: overflow code and never valid. */
    base_fields (&f);
    f.hump_peak_m = 0.1266f;   /* Rounds to 127: overflow. */
    f.height_valid = true;
    CHECK (hump_pack (&f, &w));
    CHECK (HUMP_HEIGHT_OVERFLOW == ((w >> 15U) & 0x7FUL));
    CHECK (hump_unpack (w, &g));
    CHECK (!g.height_valid);
    f.hump_peak_m = 0.1264f;   /* Rounds to 126: still valid. */
    CHECK (hump_pack (&f, &w));
    CHECK (hump_unpack (w, &g));
    CHECK (g.height_valid);
    f.run_peak_m = 0.5f;
    CHECK (hump_pack (&f, &w));
    CHECK (HUMP_HEIGHT_OVERFLOW == ((w >> 22U) & 0x7FUL));
}

static void test_rejects_bad_input (void)
{
    hump_fields_t f;
    hump_fields_t g;
    packed_hump_t w = 0xDEADBEEFUL;

    printf ("pack_unpack_reject_bad_input\n");
    base_fields (&f);
    CHECK (!hump_pack (NULL, &w));
    CHECK (!hump_pack (&f, NULL));

    f.hump_active = true;
    f.finished = true;
    CHECK (!hump_pack (&f, &w));
    base_fields (&f);
    f.faults = 0x8U;
    CHECK (!hump_pack (&f, &w));
    base_fields (&f);
    f.hump_peak_m = -0.001f;
    CHECK (!hump_pack (&f, &w));
    base_fields (&f);
    f.pitch_deg = NAN;
    CHECK (!hump_pack (&f, &w));
    base_fields (&f);
    f.run_peak_m = INFINITY;
    CHECK (!hump_pack (&f, &w));
    base_fields (&f);
    f.state = (hump_state_t)4;
    CHECK (!hump_pack (&f, &w));
    base_fields (&f);
    f.motion = (motion_class_t)8;
    CHECK (!hump_pack (&f, &w));
    CHECK (0xDEADBEEFUL == w);

    CHECK (!hump_unpack (0x60000000UL, &g));
    CHECK (!hump_unpack (0U, NULL));
}

static void test_bytes (void)
{
    uint8_t bytes[4];

    printf ("little_endian_bytes\n");
    hump_to_bytes (0x12345678UL, bytes);
    CHECK ((0x78U == bytes[0]) && (0x56U == bytes[1])
           && (0x34U == bytes[2]) && (0x12U == bytes[3]));
    CHECK (0x12345678UL == hump_from_bytes (bytes));
    CHECK (0U == hump_from_bytes (NULL));
}

static void test_run_peak (void)
{
    hump_run_t run;
    hump_output_t out;

    printf ("run_peak_counts_valid_complete_humps_only\n");
    hump_run_init (&run);
    memset (&out, 0, sizeof (out));

    out.event = HUMP_EVENT_FINISHED;
    out.complete = true;
    out.height_valid = true;
    out.peak_m = 0.030f;
    hump_run_update (&run, &out);
    CHECK (fabsf (run.run_peak_m - 0.030f) < 0.0001f);

    /* Higher but invalid: ignored, counted as rejected. */
    out.peak_m = 0.080f;
    out.height_valid = false;
    hump_run_update (&run, &out);
    /* Higher but incomplete: ignored. */
    out.height_valid = true;
    out.complete = false;
    hump_run_update (&run, &out);
    /* Not a FINISHED event: ignored entirely. */
    out.complete = true;
    out.event = HUMP_EVENT_STARTED;
    hump_run_update (&run, &out);
    CHECK (fabsf (run.run_peak_m - 0.030f) < 0.0001f);
    CHECK (1U == run.valid_humps);
    CHECK (2U == run.rejected_humps);

    out.event = HUMP_EVENT_FINISHED;
    out.peak_m = 0.045f;
    hump_run_update (&run, &out);
    out.peak_m = 0.020f;
    hump_run_update (&run, &out);
    CHECK (fabsf (run.run_peak_m - 0.045f) < 0.0001f);
    CHECK (3U == run.valid_humps);
}

/* Drive hump_detect through calibration and one 30 mm hump, packing every
 * output as the Hump task would. Checks the messages Nav would receive. */
static void test_end_to_end (void)
{
    hump_ctx_t ctx;
    hump_config_t cfg;
    hump_run_t run;
    uint32_t i;
    uint32_t cal_msgs = 0U;
    uint32_t start_msgs = 0U;
    uint32_t end_msgs = 0U;
    hump_fields_t end_fields;
    float ideal_m = 39.0f * SPEED_MPS * DT_S * sinf (15.0f * DEG_TO_RAD);

    printf ("end_to_end_with_hump_detect\n");
    memset (&end_fields, 0, sizeof (end_fields));
    hump_default_config (&cfg);
    CHECK (hump_init (&ctx, &cfg));
    hump_run_init (&run);

    for (i = 0U; i < (50U + 250U); i++)
    {
        hump_sample_t s;
        hump_output_t out;
        hump_fields_t f;
        hump_fields_t g;
        packed_hump_t w;
        uint8_t bytes[4];
        float slope = 0.0f;
        uint32_t t = (i >= 70U) ? (i - 70U) : 1000U;

        if (t < 39U)
        {
            slope = 15.0f;
        }
        else if ((t >= 49U) && (t < 88U))
        {
            slope = -15.0f;
        }
        else
        {
            /* Level. */
        }

        memset (&s, 0, sizeof (s));
        s.ax_g = sinf (slope * DEG_TO_RAD);
        s.az_g = cosf (slope * DEG_TO_RAD);
        s.motion_valid = true;
        if (i >= 50U)
        {
            s.speed_mps = SPEED_MPS;
            s.ds_m = SPEED_MPS * DT_S;
        }
        CHECK (hump_update (&ctx, &s, &out));
        hump_run_update (&run, &out);
        CHECK (hump_fields_from_output (&out, &run,
                                           MOTION_CLASS_MOVING, false, &f));
        CHECK (hump_pack (&f, &w));

        /* Through the MQTT byte form and back, as Telemetry would. */
        hump_to_bytes (w, bytes);
        CHECK (hump_unpack (hump_from_bytes (bytes), &g));

        if (HUMP_EVENT_CAL_DONE == out.event)
        {
            cal_msgs++;
            CHECK (MOTION_CLASS_MOVING == g.motion);
        }
        if (!out.calibrated)
        {
            CHECK (MOTION_CLASS_UNKNOWN == g.motion);
        }
        if (HUMP_EVENT_STARTED == out.event)
        {
            start_msgs++;
            CHECK (g.hump_active && !g.finished);
        }
        if (HUMP_EVENT_FINISHED == out.event)
        {
            end_msgs++;
            end_fields = g;
        }
    }

    printf ("  ideal %.1f mm, packed hump_peak %.0f mm, run_peak %.0f mm\n",
            (double)(ideal_m * 1000.0f),
            (double)(end_fields.hump_peak_m * 1000.0f),
            (double)(end_fields.run_peak_m * 1000.0f));
    CHECK (1U == cal_msgs);
    CHECK (1U == start_msgs);
    CHECK (1U == end_msgs);
    CHECK (end_fields.finished && !end_fields.hump_active);
    CHECK (end_fields.height_valid);
    CHECK (HUMP_STATE_FLAT == end_fields.state);
    CHECK (fabsf (end_fields.hump_peak_m - ideal_m) <= 0.0015f);
    CHECK (fabsf (end_fields.run_peak_m - end_fields.hump_peak_m) < 0.0001f);
}

int main (void)
{
    test_size_and_layout ();
    test_round_trip ();
    test_rounding ();
    test_out_of_range ();
    test_rejects_bad_input ();
    test_bytes ();
    test_run_peak ();
    test_end_to_end ();

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
