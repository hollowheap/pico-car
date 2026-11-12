/** @file test_log_core.c
 * @brief Host tests for the pure parts of the log module (D100): the record
 *        ring, the prefix and the argument counter.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "log_core.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static log_record_t make (log_sev_t sev, int32_t first)
{
    log_record_t rec;

    memset (&rec, 0, sizeof (rec));
    rec.p_fmt  = "test %d";
    rec.sev    = (uint8_t)sev;
    rec.task   = (uint8_t)LOG_TASK_LINE;
    rec.nargs  = 1U;
    rec.arg[0] = first;

    return rec;
}

static void test_prefix (void)
{
    char         buf[LOG_LINE_MAX];
    log_record_t rec = make (LOG_SEV_WARN, 0);
    uint32_t     len = 0U;

    printf ("prefix\n");
    len = log_prefix (buf, (uint32_t)sizeof (buf), &rec);
    assert (0 == strcmp (buf, "[WARN](line) "));
    assert (len == strlen (buf));

    rec.mod = (uint8_t)LOG_MOD_NAV;
    (void)log_prefix (buf, (uint32_t)sizeof (buf), &rec);
    assert (0 == strcmp (buf, "[WARN](line:nav) "));

    rec.sev  = (uint8_t)LOG_SEV_ERROR;
    rec.task = (uint8_t)LOG_TASK_OBSTACLE;
    rec.mod  = (uint8_t)LOG_MOD_SCAN;
    (void)log_prefix (buf, (uint32_t)sizeof (buf), &rec);
    assert (0 == strcmp (buf, "[ERROR](obstacle:scan) "));

    rec.sev  = (uint8_t)LOG_SEV_INFO;
    rec.task = (uint8_t)LOG_TASK_CONNECTION;
    rec.mod  = (uint8_t)LOG_MOD_TCP;
    (void)log_prefix (buf, (uint32_t)sizeof (buf), &rec);
    assert (0 == strcmp (buf, "[INFO](connection:tcp) "));

    rec.sev  = (uint8_t)LOG_SEV_DEBUG;
    rec.task = (uint8_t)LOG_TASK_SUPERVISOR;
    rec.mod  = (uint8_t)LOG_MOD_NONE;
    (void)log_prefix (buf, (uint32_t)sizeof (buf), &rec);
    assert (0 == strcmp (buf, "[DEBUG](supervisor) "));

    /* Out-of-range codes do not crash and stay readable. */
    rec.sev  = 99U;
    rec.task = 99U;
    rec.mod  = 99U;
    (void)log_prefix (buf, (uint32_t)sizeof (buf), &rec);
    assert (0 == strcmp (buf, "[?](?) "));
}

static void test_prefix_truncates (void)
{
    char         small[8];
    log_record_t rec = make (LOG_SEV_ERROR, 0);

    printf ("prefix_truncates\n");
    memset (small, 'x', sizeof (small));
    (void)log_prefix (small, (uint32_t)sizeof (small), &rec);
    assert ('\0' == small[sizeof (small) - 1U] || strlen (small) < sizeof (small));
    assert (strlen (small) <= (sizeof (small) - 1U));

    small[0] = 'x';
    assert (0U == log_prefix (small, 0U, &rec));
    assert ('x' == small[0]);
    assert (0U == log_prefix (NULL, 4U, &rec));
    (void)log_prefix (small, (uint32_t)sizeof (small), NULL);
    assert ('\0' == small[0]);
}

static void test_count_args (void)
{
    printf ("count_args\n");
    assert (0U == log_count_args (NULL));
    assert (0U == log_count_args (""));
    assert (0U == log_count_args ("no conversion"));
    assert (1U == log_count_args ("n=%d"));
    assert (0U == log_count_args ("100%% sure"));
    assert (1U == log_count_args ("100%% of %d"));
    assert (2U == log_count_args ("%d and %u"));
    assert (3U == log_count_args ("%5d %-4s %lu"));
    assert (2U == log_count_args ("%08x,%+3d"));
    assert (1U == log_count_args ("%c"));
    assert (1U == log_count_args ("%ld"));
    assert (LOG_MAX_ARGS == log_count_args ("%d %d %d %d %d %d %d %d %d %d"));
    /* A lone percent at the end consumes nothing. */
    assert (0U == log_count_args ("trailing %"));
}

static void test_ring_fifo (void)
{
    log_ring_t   ring;
    log_record_t rec;
    uint32_t     i = 0U;

    printf ("ring_fifo\n");
    log_ring_init (&ring);
    assert (0U == log_ring_count (&ring));
    assert (!log_ring_pop (&ring, &rec));

    for (i = 0U; i < 5U; i++)
    {
        rec = make (LOG_SEV_INFO, (int32_t)i);
        assert (log_ring_push (&ring, &rec));
    }
    assert (5U == log_ring_count (&ring));
    for (i = 0U; i < 5U; i++)
    {
        assert (log_ring_pop (&ring, &rec));
        assert ((int32_t)i == rec.arg[0]);
    }
    assert (0U == log_ring_count (&ring));
    assert (0U == log_ring_dropped (&ring));
}

static void test_ring_wraps (void)
{
    log_ring_t   ring;
    log_record_t rec;
    uint32_t     i = 0U;

    printf ("ring_wraps\n");
    log_ring_init (&ring);
    /* Push and pop many more records than the ring holds. */
    for (i = 0U; i < (LOG_RING_LEN * 5U); i++)
    {
        rec = make (LOG_SEV_INFO, (int32_t)i);
        assert (log_ring_push (&ring, &rec));
        assert (log_ring_pop (&ring, &rec));
        assert ((int32_t)i == rec.arg[0]);
    }
    assert (0U == log_ring_dropped (&ring));
}

static void test_ring_full_drops_info (void)
{
    log_ring_t   ring;
    log_record_t rec;
    uint32_t     i = 0U;

    printf ("ring_full_drops_info\n");
    log_ring_init (&ring);
    for (i = 0U; i < LOG_RING_LEN; i++)
    {
        rec = make (LOG_SEV_INFO, (int32_t)i);
        assert (log_ring_push (&ring, &rec));
    }
    assert (LOG_RING_LEN == log_ring_count (&ring));

    rec = make (LOG_SEV_INFO, 1000);
    assert (!log_ring_push (&ring, &rec));
    rec = make (LOG_SEV_DEBUG, 1001);
    assert (!log_ring_push (&ring, &rec));
    assert (2U == log_ring_dropped (&ring));
    assert (LOG_RING_LEN == log_ring_count (&ring));

    /* The oldest record is still first. */
    assert (log_ring_pop (&ring, &rec));
    assert (0 == rec.arg[0]);
}

static void test_ring_full_keeps_errors (void)
{
    log_ring_t   ring;
    log_record_t rec;
    uint32_t     i = 0U;

    printf ("ring_full_keeps_errors\n");
    log_ring_init (&ring);
    for (i = 0U; i < LOG_RING_LEN; i++)
    {
        rec = make (LOG_SEV_INFO, (int32_t)i);
        assert (log_ring_push (&ring, &rec));
    }

    rec = make (LOG_SEV_ERROR, 2000);
    assert (log_ring_push (&ring, &rec));
    rec = make (LOG_SEV_WARN, 2001);
    assert (log_ring_push (&ring, &rec));
    assert (2U == log_ring_dropped (&ring));
    assert (LOG_RING_LEN == log_ring_count (&ring));

    /* Two oldest records are gone. The new ones sit last, in order. */
    assert (log_ring_pop (&ring, &rec));
    assert (2 == rec.arg[0]);
    for (i = 3U; i < LOG_RING_LEN; i++)
    {
        assert (log_ring_pop (&ring, &rec));
        assert ((int32_t)i == rec.arg[0]);
    }
    assert (log_ring_pop (&ring, &rec));
    assert (2000 == rec.arg[0]);
    assert (log_ring_pop (&ring, &rec));
    assert (2001 == rec.arg[0]);
    assert (!log_ring_pop (&ring, &rec));
}

static void test_same (void)
{
    log_record_t a = make (LOG_SEV_WARN, 5);
    log_record_t b = make (LOG_SEV_WARN, 5);

    printf ("same\n");
    assert (log_same (&a, &b));
    b.arg[0] = 6;
    assert (!log_same (&a, &b));
    b = a;
    b.p_fmt = "other %d";
    assert (!log_same (&a, &b));
    b = a;
    b.task = (uint8_t)LOG_TASK_MOTION;
    assert (!log_same (&a, &b));
    b = a;
    b.sev = (uint8_t)LOG_SEV_ERROR;
    assert (!log_same (&a, &b));
    /* Arguments beyond nargs do not count. */
    b = a;
    b.arg[3] = 77;
    assert (log_same (&a, &b));
    assert (!log_same (NULL, &b));
}

static void test_null_safety (void)
{
    log_record_t rec = make (LOG_SEV_INFO, 0);

    printf ("null_safety\n");
    log_ring_init (NULL);
    assert (!log_ring_push (NULL, &rec));
    assert (!log_ring_pop (NULL, &rec));
    assert (0U == log_ring_count (NULL));
    assert (0U == log_ring_dropped (NULL));
}

int main (void)
{
    test_prefix ();
    test_prefix_truncates ();
    test_count_args ();
    test_ring_fifo ();
    test_ring_wraps ();
    test_ring_full_drops_info ();
    test_ring_full_keeps_errors ();
    test_same ();
    test_null_safety ();
    printf ("log_core: all tests passed\n");
    return 0;
}
