/** @file test_line_filter.c
 * @brief Host tests for the 3-sample line sensor debounce (S09).
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "line_filter.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define SAMPLE_PERIOD_MS 5U
#define LONG_RUN         1000U

/* Feed a string of raw samples ('1' on the line, '0' off) and return the
 * filtered level after the last one. */
static bool feed (line_debounce_t * p_f, char const * p_samples)
{
    bool b_out = p_f->b_level;

    for (; '\0' != *p_samples; p_samples++)
    {
        b_out = line_debounce_update (p_f, '1' == *p_samples);
    }

    return b_out;
}

static void test_init_holds_level (void)
{
    line_debounce_t f;

    printf ("init_holds_level\n");
    line_debounce_init (&f, true);
    assert (f.b_level && (0U == f.count));
    line_debounce_init (&f, false);
    assert ((!f.b_level) && (0U == f.count));
    assert (3U == LINE_DEBOUNCE_SAMPLES);
}

static void test_short_pulses_rejected (void)
{
    line_debounce_t f;

    printf ("short_pulses_rejected\n");
    line_debounce_init (&f, false);
    /* One or two black samples are not a line. */
    assert (!feed (&f, "1"));
    assert (!feed (&f, "0"));
    assert (!feed (&f, "11"));
    assert (!feed (&f, "0"));

    line_debounce_init (&f, true);
    /* One or two white samples are not a loss. */
    assert (feed (&f, "0"));
    assert (feed (&f, "1"));
    assert (feed (&f, "00"));
    assert (feed (&f, "1"));
}

static void test_third_sample_changes_output (void)
{
    line_debounce_t f;

    printf ("third_sample_changes_output\n");
    line_debounce_init (&f, false);
    assert (!line_debounce_update (&f, true));
    assert (!line_debounce_update (&f, true));
    assert (line_debounce_update (&f, true));
    /* 3 samples at 5 ms is the 15 ms of S09. */
    assert (15U == (LINE_DEBOUNCE_SAMPLES * SAMPLE_PERIOD_MS));

    assert (line_debounce_update (&f, false));
    assert (line_debounce_update (&f, false));
    assert (!line_debounce_update (&f, false));
}

static void test_flicker_never_flips (void)
{
    line_debounce_t f;
    uint32_t        i = 0U;

    printf ("flicker_never_flips\n");
    line_debounce_init (&f, false);
    for (i = 0U; i < LONG_RUN; i++)
    {
        assert (!line_debounce_update (&f, 0U != (i & 1U)));
    }

    line_debounce_init (&f, true);
    for (i = 0U; i < LONG_RUN; i++)
    {
        assert (line_debounce_update (&f, 0U == (i & 1U)));
    }

    /* Two on, one off, two on, one off ... never reaches three. */
    line_debounce_init (&f, false);
    for (i = 0U; i < LONG_RUN; i++)
    {
        assert (!line_debounce_update (&f, 2U != (i % 3U)));
    }
}

static void test_interrupted_run_restarts (void)
{
    line_debounce_t f;

    printf ("interrupted_run_restarts\n");
    line_debounce_init (&f, false);
    assert (!feed (&f, "11"));
    assert (!feed (&f, "0"));
    /* The run restarts: two more black samples are still not enough. */
    assert (!feed (&f, "11"));
    assert (feed (&f, "1"));
}

static void test_no_stuck_count (void)
{
    line_debounce_t f;
    uint32_t        i = 0U;

    printf ("no_stuck_count\n");
    line_debounce_init (&f, true);
    for (i = 0U; i < LONG_RUN; i++)
    {
        assert (line_debounce_update (&f, true));
        assert (0U == f.count);
    }
    assert (!feed (&f, "000"));
    for (i = 0U; i < LONG_RUN; i++)
    {
        assert (!line_debounce_update (&f, false));
        assert (0U == f.count);
    }
    /* The count never exceeds the limit while it runs. */
    assert (!line_debounce_update (&f, true));
    assert (f.count <= LINE_DEBOUNCE_SAMPLES);
}

static void test_sensors_are_independent (void)
{
    line_debounce_t left;
    line_debounce_t right;

    printf ("sensors_are_independent\n");
    line_debounce_init (&left, false);
    line_debounce_init (&right, false);
    (void)feed (&left, "11");
    (void)feed (&right, "1");
    assert (feed (&left, "1"));
    assert (!feed (&right, "1"));
    assert (feed (&right, "1"));
}

static void test_null_pointer (void)
{
    printf ("null_pointer\n");
    line_debounce_init (NULL, true);
    assert (!line_debounce_update (NULL, true));
}

int main (void)
{
    test_init_holds_level ();
    test_short_pulses_rejected ();
    test_third_sample_changes_output ();
    test_flicker_never_flips ();
    test_interrupted_run_restarts ();
    test_no_stuck_count ();
    test_sensors_are_independent ();
    test_null_pointer ();
    printf ("line_filter: all tests passed\n");
    return 0;
}
