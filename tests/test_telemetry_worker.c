/** @file test_telemetry_worker.c
 * @brief Host test for telemetry_worker_run (src/telemetry/worker.c, S24).
 *
 * The worker is driven with a scripted pop and a recording sink; no queue or
 * kernel object is involved. A call guard aborts the run if the worker loops
 * past the script, so a regression to an endless loop fails instead of hanging.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "telemetry/packet.h"
#include "telemetry/worker.h"

#define FRAME_MAX  TELEMETRY_FRAME_MAX
#define SCRIPT_MAX 32U
#define CALL_GUARD 64U
#define RLWAI_CODE (-49)

static unsigned g_fail;
static unsigned g_checks;

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
        (void)printf("FAIL worker kept calling %s (endless loop)\n", who);
        exit(2);
    }
}

/* ---- scripted pop: each step returns `ret` and writes `len` bytes -------- */

typedef struct
{
    INT     ret;
    uint8_t len;
    uint8_t seed;
} step_t;

static step_t    g_script[SCRIPT_MAX];
static unsigned  g_script_len;
static unsigned  g_pop_calls;
static TMO       g_pop_tmo[CALL_GUARD + 1U];
static uint8_t * g_pop_buf[CALL_GUARD + 1U];

static INT fake_pop (uint8_t * frame, TMO tmout)
{
    const unsigned call = g_pop_calls;

    g_pop_calls++;
    guard(g_pop_calls, "pop");
    g_pop_tmo[call] = tmout;
    g_pop_buf[call] = frame;
    if (call >= g_script_len)
    {
        return E_TMOUT;
    }
    for (uint8_t i = 0U; i < g_script[call].len; i++)
    {
        frame[i] = (uint8_t)(g_script[call].seed + i);
    }
    return g_script[call].ret;
}

/* ---- recording sink ------------------------------------------------------ */

static unsigned g_sink_calls;
static uint16_t g_sink_len[CALL_GUARD + 1U];
static uint8_t  g_sink_data[CALL_GUARD + 1U][FRAME_MAX];
static int      g_sink_result[CALL_GUARD + 1U];
static int      g_sink_default;

static int fake_sink (const uint8_t * frame, uint16_t len)
{
    const unsigned call = g_sink_calls;

    g_sink_calls++;
    guard(g_sink_calls, "sink");
    g_sink_len[call] = len;
    memcpy(g_sink_data[call], frame, (len <= FRAME_MAX) ? len : FRAME_MAX);
    return (call < CALL_GUARD) ? g_sink_result[call] : g_sink_default;
}

static void reset (void)
{
    memset(g_script, 0, sizeof g_script);
    memset(g_pop_tmo, 0, sizeof g_pop_tmo);
    memset(g_pop_buf, 0, sizeof g_pop_buf);
    memset(g_sink_len, 0, sizeof g_sink_len);
    memset(g_sink_data, 0, sizeof g_sink_data);
    memset(g_sink_result, 0, sizeof g_sink_result);
    g_script_len  = 0U;
    g_pop_calls   = 0U;
    g_sink_calls  = 0U;
    g_sink_default = 0;
}

static void add (INT ret, uint8_t len, uint8_t seed)
{
    g_script[g_script_len].ret  = ret;
    g_script[g_script_len].len  = len;
    g_script[g_script_len].seed = seed;
    g_script_len++;
}

static int sink_got (unsigned call, uint16_t len, uint8_t seed)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        if (g_sink_data[call][i] != (uint8_t)(seed + i))
        {
            return 0;
        }
    }
    return (g_sink_len[call] == len);
}

static void pass (const char * name, unsigned fail_before)
{
    (void)printf("%s %s\n", (g_fail == fail_before) ? "PASS" : "FAIL", name);
}

/* ---- tests --------------------------------------------------------------- */

static void test_drains_all_frames (void)
{
    unsigned before = g_fail;

    reset();
    add(9, 9U, 10U);
    add(5, 5U, 20U);
    add((INT)FRAME_MAX, FRAME_MAX, 30U);
    add(1, 1U, 40U);
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);

    CHECK(g_sink_calls == 4U);
    CHECK(sink_got(0U, 9U, 10U));
    CHECK(sink_got(1U, 5U, 20U));
    CHECK(sink_got(2U, FRAME_MAX, 30U));
    CHECK(sink_got(3U, 1U, 40U));
    CHECK(g_pop_calls == 5U); /* four frames, then the empty pop ends it */
    pass("worker_drains_all_frames", before);
}

static void test_length_is_the_pop_return (void)
{
    unsigned before = g_fail;

    reset();
    /* The returned size, not the bytes the pop wrote, is the sink length. */
    add(3, 8U, 1U);
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
    CHECK(g_sink_calls == 1U);
    CHECK(g_sink_len[0] == 3U);
    pass("worker_length_is_the_pop_return", before);
}

static void test_timeout_forwarded (void)
{
    static const TMO values[] = {TMO_POL, TMO_FEVR, 250};

    unsigned before = g_fail;

    for (unsigned v = 0U; v < sizeof values / sizeof values[0]; v++)
    {
        reset();
        add(4, 4U, 1U);
        add(4, 4U, 2U);
        telemetry_worker_run(fake_pop, fake_sink, values[v]);
        CHECK(g_pop_calls == 3U);
        for (unsigned c = 0U; c < g_pop_calls; c++)
        {
            CHECK(g_pop_tmo[c] == values[v]);
        }
    }
    pass("worker_timeout_forwarded", before);
}

static void test_same_buffer_for_pop_and_sink (void)
{
    unsigned before = g_fail;

    reset();
    add(8, 8U, 5U);
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
    CHECK(g_pop_buf[0] != 0);
    CHECK(g_pop_buf[0] == g_pop_buf[1]); /* one frame buffer is reused */
    pass("worker_same_buffer_for_pop_and_sink", before);
}

static void test_empty_queue_does_nothing (void)
{
    unsigned before = g_fail;

    reset();
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
    CHECK(g_pop_calls == 1U);
    CHECK(g_sink_calls == 0U);
    pass("worker_empty_queue_does_nothing", before);
}

static void test_stops_on_pop_error (void)
{
    static const INT errors[] = {E_TMOUT, E_NOEXS, RLWAI_CODE, E_PAR, 0};

    unsigned before = g_fail;

    for (unsigned e = 0U; e < sizeof errors / sizeof errors[0]; e++)
    {
        reset();
        add(8, 8U, 1U);
        add(errors[e], 0U, 0U);
        add(8, 8U, 2U); /* must not be reached */
        telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
        CHECK(g_sink_calls == 1U);
        CHECK(sink_got(0U, 8U, 1U));
        CHECK(g_pop_calls == 2U);
    }
    pass("worker_stops_on_pop_error", before);
}

static void test_stops_on_sink_failure (void)
{
    static const int failures[] = {1, -1, 7};

    unsigned before = g_fail;

    for (unsigned f = 0U; f < sizeof failures / sizeof failures[0]; f++)
    {
        reset();
        add(8, 8U, 1U);
        add(8, 8U, 2U);
        add(8, 8U, 3U);
        g_sink_result[1] = failures[f]; /* second frame fails */
        telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
        /* The failed frame was already popped and is not retried; the third
           frame stays in the queue for the next run. */
        CHECK(g_sink_calls == 2U);
        CHECK(g_pop_calls == 2U);
        CHECK(sink_got(1U, 8U, 2U));
    }

    reset();
    add(4, 4U, 1U);
    g_sink_result[0] = 1;
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
    CHECK(g_sink_calls == 1U);
    CHECK(g_pop_calls == 1U);
    pass("worker_stops_on_sink_failure", before);
}

static void test_null_sink_discards (void)
{
    unsigned before = g_fail;

    reset();
    add(8, 8U, 1U);
    add(8, 8U, 2U);
    add(8, 8U, 3U);
    telemetry_worker_run(fake_pop, 0, TMO_POL);
    CHECK(g_pop_calls == 4U); /* still drains the queue */
    CHECK(g_sink_calls == 0U);
    pass("worker_null_sink_discards", before);
}

static void test_runs_again_after_stop (void)
{
    unsigned before = g_fail;

    reset();
    add(8, 8U, 1U);
    add(E_TMOUT, 0U, 0U);
    add(8, 8U, 2U);
    add(8, 8U, 3U);
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
    CHECK(g_sink_calls == 1U);
    /* The worker keeps no state: a second call carries on with the script. */
    telemetry_worker_run(fake_pop, fake_sink, TMO_POL);
    CHECK(g_sink_calls == 3U);
    CHECK(sink_got(1U, 8U, 2U));
    CHECK(sink_got(2U, 8U, 3U));
    pass("worker_runs_again_after_stop", before);
}

int main (void)
{
    test_drains_all_frames();
    test_length_is_the_pop_return();
    test_timeout_forwarded();
    test_same_buffer_for_pop_and_sink();
    test_empty_queue_does_nothing();
    test_stops_on_pop_error();
    test_stops_on_sink_failure();
    test_null_sink_discards();
    test_runs_again_after_stop();

    (void)printf("%u checks, %u failed\n", g_checks, g_fail);
    return (g_fail == 0U) ? 0 : 1;
}
