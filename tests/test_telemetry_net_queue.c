/** @file test_telemetry_net_queue.c
 * @brief Host test for the Core 1 worker to connection-task queue.
 *
 * src/telemetry/net_queue.c is compiled unchanged against the kernel shim and
 * the message buffer fake in tests/shim/tk/mbf_fake.h (capacity and ring
 * rules copied from messagebuf.c). A frame is the topic byte plus the wire
 * packet, so the largest frame (the 82-byte profile) is an 83-byte message
 * here: 88 bytes in the buffer, 11 messages in 1024 bytes.
 *
 * Unlike the telemetry queue, a full net queue never evicts: the new frame is
 * refused with E_TMOUT and counted.
 *
 * NOTE: artifacts/SE01_PostTri1.md (sections 3 and 4.1) says overflow drops
 * the oldest QoS 0 report. The code drops the newest frame. These tests
 * assert the code.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tk/mbf_fake.h>
#include "telemetry/net_queue.h"

#define BUFFER_BYTES 1024U
#define MSG_MAX      83U
#define CAP_17       11U /* messages of MSG_MAX bytes (88 B each) */
#define CAP_9        64U /* 9 B costs 16 B */
#define CAP_2        128U /* 1 B costs 8 B */
#define FUZZ_STEPS   3000U
#define MODEL_MAX    160U
#define LCG_MUL      1103515245U
#define LCG_ADD      12345U

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

static void fill_frame (uint8_t * frame, uint16_t len, uint8_t seed)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        frame[i] = (uint8_t)(seed + i);
    }
}

static ER push_seed (uint16_t len, uint8_t seed)
{
    uint8_t frame[MSG_MAX];

    fill_frame(frame, len, seed);
    return telemetry_net_queue_push(frame, len);
}

static int pop_is (uint16_t len, uint8_t seed)
{
    uint8_t msg[MSG_MAX];
    uint8_t want[MSG_MAX];
    INT     got = telemetry_net_queue_pop(msg, TMO_POL);

    fill_frame(want, len, seed);
    return (got == (INT)len) && (memcmp(msg, want, len) == 0);
}

static int pop_empty (void)
{
    uint8_t msg[MSG_MAX];

    return telemetry_net_queue_pop(msg, TMO_POL) == E_TMOUT;
}

static unsigned fill_with (uint16_t len, unsigned count)
{
    unsigned stored = 0U;

    for (unsigned i = 0U; i < count; i++)
    {
        if (push_seed(len, (uint8_t)i) == E_OK)
        {
            stored++;
        }
    }
    return stored;
}

static void setup (void)
{
    mbf_fake_reset();
    g_mbf_net_tx_id = 0;
    CHECK(telemetry_net_queue_create() == E_OK);
}

static void pass (const char * name, unsigned fail_before)
{
    (void)printf("%s %s\n", (g_fail == fail_before) ? "PASS" : "FAIL", name);
}

static void test_create_failure_paths (void)
{
    static uint8_t spare[BUFFER_BYTES] __attribute__((aligned(4)));
    uint8_t        msg[MSG_MAX] = {0};
    T_CMBF         cfg = {0};
    unsigned       before = g_fail;

    mbf_fake_reset();
    g_mbf_net_tx_id = 0;

    CHECK(telemetry_net_queue_push(msg, 4U) == E_NOEXS);
    CHECK(telemetry_net_queue_pop(msg, TMO_POL) == E_NOEXS);
    CHECK(g_mbf_fake.send_calls == 0U);
    CHECK(g_mbf_fake.recv_calls == 0U);

    g_mbf_fake.create_error = E_NOMEM;
    CHECK(telemetry_net_queue_create() == E_NOMEM);
    CHECK(g_mbf_net_tx_id == 0);
    CHECK(telemetry_net_queue_push(msg, 4U) == E_NOEXS);

    cfg.mbfatr = TA_USERBUF;
    cfg.bufsz  = BUFFER_BYTES;
    cfg.maxmsz = MSG_MAX;
    cfg.bufptr = spare;
    CHECK(tk_cre_mbf(&cfg) > 0);
    CHECK(tk_cre_mbf(&cfg) > 0);
    CHECK(telemetry_net_queue_create() == E_LIMIT);
    CHECK(g_mbf_net_tx_id == 0);

    mbf_fake_reset();
    CHECK(telemetry_net_queue_create() == E_OK);
    CHECK(g_mbf_net_tx_id > 0);
    CHECK(g_mbf_fake.create_calls == 1U);
    CHECK(push_seed(4U, 5U) == E_OK);
    CHECK(telemetry_net_queue_create() == E_OK); /* second call is a no-op */
    CHECK(g_mbf_fake.create_calls == 1U);
    CHECK(pop_is(4U, 5U));
    pass("net_create_failure_paths", before);
}

static void test_create_parameters (void)
{
    unsigned before = g_fail;

    setup();
    CHECK((g_mbf_fake.last_cmbf.mbfatr & TA_USERBUF) != 0U);
    CHECK(g_mbf_fake.last_cmbf.bufsz == (SZ)TELEMETRY_NET_QUEUE_BYTES);
    CHECK(g_mbf_fake.last_cmbf.bufsz == (SZ)BUFFER_BYTES);
    CHECK(g_mbf_fake.last_cmbf.maxmsz == (INT)TELEMETRY_NET_QUEUE_MAX_MSG);
    CHECK(g_mbf_fake.last_cmbf.maxmsz == (INT)MSG_MAX);
    CHECK(g_mbf_fake.last_cmbf.bufptr != 0);
    CHECK((((uintptr_t)g_mbf_fake.last_cmbf.bufptr) % 4U) == 0U);
    pass("net_create_parameters", before);
}

static void test_arguments (void)
{
    uint8_t  frame[MSG_MAX + 1U] = {0};
    uint8_t  msg[MSG_MAX];
    unsigned before = g_fail;
    unsigned sent;

    setup();
    sent = g_mbf_fake.send_calls;
    CHECK(telemetry_net_queue_push(0, 4U) == E_PAR);
    CHECK(telemetry_net_queue_push(frame, 0U) == E_PAR);
    CHECK(telemetry_net_queue_push(frame, MSG_MAX + 1U) == E_PAR);
    CHECK(telemetry_net_queue_push(frame, UINT16_MAX) == E_PAR);
    CHECK(g_mbf_fake.send_calls == sent);
    CHECK(telemetry_net_queue_drop_count() == 0U); /* E_PAR is not a drop */

    CHECK(telemetry_net_queue_push(frame, 1U) == E_OK);
    CHECK(telemetry_net_queue_push(frame, MSG_MAX) == E_OK); /* no tag here */

    CHECK(telemetry_net_queue_pop(0, TMO_POL) == E_PAR);
    CHECK(telemetry_net_queue_pop(msg, 77) == 1); /* oldest: the 1 B frame */
    CHECK(g_mbf_fake.last_rcv_tmout == 77);

    g_mbf_net_tx_id = 0;
    CHECK(telemetry_net_queue_push(0, 4U) == E_PAR); /* args first */
    CHECK(telemetry_net_queue_pop(0, TMO_POL) == E_PAR);
    pass("net_arguments", before);
}

static void test_fifo_and_lengths (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(push_seed(4U, 1U) == E_OK);
    CHECK(push_seed(MSG_MAX, 2U) == E_OK);
    CHECK(push_seed(1U, 3U) == E_OK);
    CHECK(push_seed(8U, 4U) == E_OK);
    CHECK(pop_is(4U, 1U));
    CHECK(pop_is(MSG_MAX, 2U));
    CHECK(pop_is(1U, 3U));
    CHECK(pop_is(8U, 4U));
    CHECK(pop_empty());
    pass("net_fifo_and_lengths", before);
}

static void test_full_rejects_new_and_counts (void)
{
    unsigned before = g_fail;
    unsigned drops;

    setup();
    drops = telemetry_net_queue_drop_count();
    CHECK(fill_with(MSG_MAX, CAP_17) == CAP_17);
    CHECK(telemetry_net_queue_drop_count() == drops);

    CHECK(push_seed(MSG_MAX, 200U) == E_TMOUT);
    CHECK(telemetry_net_queue_drop_count() == drops + 1U);
    /* 968 of 1024 bytes are used; another full-size frame still does not fit. */
    CHECK(push_seed(MSG_MAX, 201U) == E_TMOUT);
    CHECK(telemetry_net_queue_drop_count() == drops + 2U);
    CHECK(g_mbf_fake.last_snd_tmout == TMO_POL);
    CHECK(g_mbf_fake.would_block == 0U);
    CHECK(g_mbf_fake.recv_calls == 0U); /* never evicts */

    /* The queue still holds the first frames, oldest first. */
    for (unsigned i = 0U; i < CAP_17; i++)
    {
        CHECK(pop_is(MSG_MAX, (uint8_t)i));
    }
    CHECK(pop_empty());

    /* Space again: pushes work and are not counted. */
    CHECK(push_seed(MSG_MAX, 9U) == E_OK);
    CHECK(telemetry_net_queue_drop_count() == drops + 2U);
    pass("net_full_rejects_new_and_counts", before);
}

static void test_capacity_by_size (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(fill_with(MSG_MAX, CAP_17 + 3U) == CAP_17);
    setup();
    CHECK(fill_with(9U, CAP_9 + 3U) == CAP_9); /* 9 B costs 16 B */
    setup();
    CHECK(fill_with(1U, CAP_2 + 3U) == CAP_2); /* 1 B costs 8 B */
    pass("net_capacity_by_size", before);
}

static void test_other_errors_not_counted (void)
{
    unsigned before = g_fail;
    unsigned drops;

    setup();
    drops = telemetry_net_queue_drop_count();
    g_mbf_fake.send_error = E_NOMEM;
    CHECK(push_seed(4U, 0U) == E_NOMEM);
    CHECK(telemetry_net_queue_drop_count() == drops);
    CHECK(pop_empty());
    pass("net_other_errors_not_counted", before);
}

typedef struct
{
    uint8_t len;
    uint8_t seed;
} model_t;

static void test_wraparound_fuzz (void)
{
    static const uint8_t lens[] = {1U, 4U, 5U, 8U, 9U, 12U, 16U, 17U, 40U, 83U};
    model_t              model[MODEL_MAX];
    unsigned             count = 0U;
    unsigned             used = 0U;
    unsigned             refused = 0U;
    uint32_t             rng = 7U;
    unsigned             before = g_fail;
    unsigned             drops;

    setup();
    drops = telemetry_net_queue_drop_count();
    for (unsigned step = 0U; step < FUZZ_STEPS; step++)
    {
        rng = (rng * LCG_MUL) + LCG_ADD;
        if (((rng >> 16) % 5U) < 3U)
        {
            const model_t  m = {lens[(rng >> 12) % sizeof lens],
                                (uint8_t)(rng >> 20)};
            const unsigned cost = MBF_FAKE_COST(m.len);
            const ER       r = push_seed(m.len, m.seed);

            if ((used + cost) <= BUFFER_BYTES)
            {
                CHECK(r == E_OK);
                model[count++] = m;
                used += cost;
            }
            else
            {
                CHECK(r == E_TMOUT);
                refused++;
            }
        }
        else if (count > 0U)
        {
            CHECK(pop_is(model[0].len, model[0].seed));
            used -= MBF_FAKE_COST(model[0].len);
            memmove(&model[0], &model[1], (count - 1U) * sizeof model[0]);
            count--;
        }
        else
        {
            CHECK(pop_empty());
        }
    }
    CHECK(refused > 0U);
    CHECK(telemetry_net_queue_drop_count() == drops + refused);
    pass("net_wraparound_fuzz", before);
}

int main (void)
{
    test_create_failure_paths();
    test_create_parameters();
    test_arguments();
    test_fifo_and_lengths();
    test_full_rejects_new_and_counts();
    test_capacity_by_size();
    test_other_errors_not_counted();
    test_wraparound_fuzz();

    (void)printf("%u checks, %u failed\n", g_checks, g_fail);
    return (g_fail == 0U) ? 0 : 1;
}
