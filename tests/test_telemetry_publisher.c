/** @file test_telemetry_publisher.c
 * @brief Host test for telemetry_publisher_run (src/telemetry/publisher.c).
 *
 * The publisher is driven with a scripted pull and a recording publish
 * function. A call guard aborts the run if the publisher loops past the
 * script, so a regression to an endless retry loop fails instead of hanging.
 *
 * S24: "a failed publish does not stop the connection loop". The publisher
 * meets that by returning to its caller on the first failed publish; the
 * caller (connection_task.c) calls it again on its next pass.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "telemetry/packet.h"
#include "telemetry/publisher.h"

#define FRAME_MAX  TELEMETRY_FRAME_MAX
#define SCRIPT_MAX 32U
#define CALL_GUARD 64U

static unsigned g_fail;
static unsigned g_checks;
static telemetry_publisher_t g_publisher;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        g_checks++;                                                          \
        if (!(cond))                                                         \
        {                                                                    \
            g_fail++;                                                        \
            (void)printf("  FAIL line %d: %s\n", __LINE__, #cond);           \
        }                                                                    \
    } while (0)

static void guard (unsigned calls, const char * who)
{
    if (calls > CALL_GUARD)
    {
        (void)printf("FAIL publisher kept calling %s (endless loop)\n", who);
        exit(2);
    }
}

/* ---- scripted pull: returns `ret`, sets *len, writes `len` bytes --------- */

typedef struct
{
    int      ret;
    uint16_t len;
    uint8_t  seed;
} step_t;

static step_t   g_script[SCRIPT_MAX];
static unsigned g_script_len;
static unsigned g_pull_calls;

static int fake_pull (uint8_t * frame, uint16_t * len)
{
    const unsigned call = g_pull_calls;

    g_pull_calls++;
    guard(g_pull_calls, "pull");
    if (call >= g_script_len)
    {
        return 0;
    }
    *len = g_script[call].len;
    for (uint16_t i = 0U; (i < g_script[call].len) && (i < FRAME_MAX); i++)
    {
        frame[i] = (uint8_t)(g_script[call].seed + i);
    }
    return g_script[call].ret;
}

/* ---- recording publish --------------------------------------------------- */

static unsigned g_pub_calls;
static uint16_t g_pub_len[CALL_GUARD + 1U];
static uint8_t  g_pub_data[CALL_GUARD + 1U][FRAME_MAX];
static int      g_pub_result[CALL_GUARD + 1U];

static int fake_publish (const uint8_t * frame, uint16_t len)
{
    const unsigned call = g_pub_calls;

    g_pub_calls++;
    guard(g_pub_calls, "publish");
    g_pub_len[call] = len;
    memcpy(g_pub_data[call], frame, (len <= FRAME_MAX) ? len : FRAME_MAX);
    return g_pub_result[call];
}

static void reset (void)
{
    memset(g_script, 0, sizeof g_script);
    memset(g_pub_len, 0, sizeof g_pub_len);
    memset(g_pub_data, 0, sizeof g_pub_data);
    memset(g_pub_result, 0, sizeof g_pub_result);
    g_script_len = 0U;
    g_pull_calls = 0U;
    g_pub_calls  = 0U;
    g_publisher.len = 0U;
}

static void add (int ret, uint16_t len, uint8_t seed)
{
    g_script[g_script_len].ret  = ret;
    g_script[g_script_len].len  = len;
    g_script[g_script_len].seed = seed;
    g_script_len++;
}

static int pub_got (unsigned call, uint16_t len, uint8_t seed)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        if (g_pub_data[call][i] != (uint8_t)(seed + i))
        {
            return 0;
        }
    }
    return g_pub_len[call] == len;
}

static void pass (const char * name, unsigned fail_before)
{
    (void)printf("%s %s\n", (g_fail == fail_before) ? "PASS" : "FAIL", name);
}

/* ---- tests --------------------------------------------------------------- */

static void test_publishes_until_empty (void)
{
    unsigned before = g_fail;

    reset();
    add(1, 8U, 10U);
    add(1, 4U, 20U);
    add(1, FRAME_MAX, 30U);
    add(1, 1U, 40U);
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 4U);
    CHECK(pub_got(0U, 8U, 10U));
    CHECK(pub_got(1U, 4U, 20U));
    CHECK(pub_got(2U, FRAME_MAX, 30U));
    CHECK(pub_got(3U, 1U, 40U));
    CHECK(g_pull_calls == 5U);
    pass("publisher_publishes_until_empty", before);
}

static void test_any_positive_pull_is_a_frame (void)
{
    unsigned before = g_fail;

    reset();
    add(5, 8U, 1U);
    add(100, 8U, 2U);
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 2U);
    pass("publisher_any_positive_pull_is_a_frame", before);
}

static void test_empty_pull_does_nothing (void)
{
    unsigned before = g_fail;

    reset();
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pull_calls == 1U);
    CHECK(g_pub_calls == 0U);
    pass("publisher_empty_pull_does_nothing", before);
}

static void test_stops_on_pull_end_or_error (void)
{
    static const int ends[] = {0, -1, -50, -42};

    unsigned before = g_fail;

    for (unsigned e = 0U; e < sizeof ends / sizeof ends[0]; e++)
    {
        reset();
        add(1, 8U, 1U);
        add(ends[e], 8U, 2U); /* frame data is ignored when ret <= 0 */
        add(1, 8U, 3U);       /* must not be reached */
        telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
        CHECK(g_pub_calls == 1U);
        CHECK(pub_got(0U, 8U, 1U));
        CHECK(g_pull_calls == 2U);
    }
    pass("publisher_stops_on_pull_end_or_error", before);
}

static void test_rejects_bad_lengths (void)
{
    static const uint16_t bad[] = {0U, FRAME_MAX + 1U, 200U, UINT16_MAX};

    unsigned before = g_fail;

    for (unsigned b = 0U; b < sizeof bad / sizeof bad[0]; b++)
    {
        reset();
        add(1, bad[b], 1U);
        add(1, 8U, 2U); /* must not be reached */
        telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
        CHECK(g_pub_calls == 0U);
        CHECK(g_pull_calls == 1U);
    }

    /* Boundaries 1 and FRAME_MAX are published. */
    reset();
    add(1, 1U, 1U);
    add(1, FRAME_MAX, 2U);
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 2U);
    pass("publisher_rejects_bad_lengths", before);
}

static void test_failed_publish_returns_without_retry (void)
{
    static const int failures[] = {-1, 7};

    unsigned before = g_fail;

    for (unsigned f = 0U; f < sizeof failures / sizeof failures[0]; f++)
    {
        reset();
        add(1, 8U, 1U);
        add(1, 8U, 2U);
        add(1, 8U, 3U);
        g_pub_result[1] = failures[f];
        telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
        /* The run ends at the failure: no retry of the same frame and no
           further pulls, so the caller's loop is not held up. */
        CHECK(g_pub_calls == 2U);
        CHECK(g_pull_calls == 2U);
        CHECK(pub_got(1U, 8U, 2U));
    }
    pass("publisher_failed_publish_returns_without_retry", before);
}

static void test_next_run_continues_after_failure (void)
{
    unsigned before = g_fail;

    reset();
    add(1, 8U, 1U);
    add(1, 8U, 2U);
    add(1, 8U, 3U);
    g_pub_result[0] = -1;
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 1U);
    /* The failed frame is gone (already pulled). The next pass publishes the
       rest: the publisher keeps no failed state. */
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 3U);
    CHECK(pub_got(1U, 8U, 2U));
    CHECK(pub_got(2U, 8U, 3U));
    pass("publisher_next_run_continues_after_failure", before);
}

static void test_busy_frame_retries_before_next_pull (void)
{
    unsigned before = g_fail;

    reset();
    add(1, 8U, 1U);
    add(1, 8U, 2U);
    g_pub_result[0] = TELEMETRY_PUBLISH_RETRY;
    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 1U);
    CHECK(g_pull_calls == 1U);

    telemetry_publisher_run(&g_publisher, fake_pull, fake_publish);
    CHECK(g_pub_calls == 3U);
    CHECK(pub_got(1U, 8U, 1U));
    CHECK(pub_got(2U, 8U, 2U));
    pass("publisher_retries_busy_frame_before_next_pull", before);
}

static void test_null_publish_discards (void)
{
    unsigned before = g_fail;

    reset();
    add(1, 8U, 1U);
    add(1, 8U, 2U);
    telemetry_publisher_run(&g_publisher, fake_pull, 0);
    CHECK(g_pull_calls == 3U);
    CHECK(g_pub_calls == 0U);
    pass("publisher_null_publish_discards", before);
}

int main (void)
{
    test_publishes_until_empty();
    test_any_positive_pull_is_a_frame();
    test_empty_pull_does_nothing();
    test_stops_on_pull_end_or_error();
    test_rejects_bad_lengths();
    test_failed_publish_returns_without_retry();
    test_next_run_continues_after_failure();
    test_busy_frame_retries_before_next_pull();
    test_null_publish_discards();

    (void)printf("%u checks, %u failed\n", g_checks, g_fail);
    return (g_fail == 0U) ? 0 : 1;
}
