/** @file test_telemetry_queue.c
 * @brief Host test for the Core 0 to Core 1 telemetry queue (S03).
 *
 * src/telemetry/queue.c is compiled unchanged against the kernel shim. The
 * message buffer is the fake in tests/shim/tk/mbf_fake.h, which copies the
 * capacity and ring rules of lib/mtk3smp-rp2040/kernel/tkernel/messagebuf.c.
 * Frames are stored without a topic byte (D87, S35) and the topic is inferred
 * from the stored length, so a 384-byte buffer holds:
 *   system    (16 B stored): 20 bytes each, 19 messages
 *   status     (8 B stored): 12 bytes each, 32 messages
 *   event      (7 B stored): 12 bytes each, 32 messages
 *   heartbeat  (4 B stored):  8 bytes each, 48 messages
 * pop still returns topic || wire frame (events with byte 7 restored as 0).
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tk/mbf_fake.h>
#include "telemetry/queue.h"

#define BUFFER_BYTES   384U
#define FRAME_MAX      16U
#define MSG_MAX        17U
#define STATUS_LEN     8U
#define SYSTEM_LEN     16U
#define HEARTBEAT_LEN  4U
#define EVENT_LEN      8U
#define CAP_SYSTEM     19U
#define CAP_STATUS     32U
#define CAP_EVENT      32U
#define CAP_HEARTBEAT  48U
#define COST_HEARTBEAT 8U
#define COST_EVENT     12U
#define COST_STATUS    12U
#define COST_SYSTEM    20U
#define CAP_17         16U /* kernel math only: 17 B message costs 24 B */
#define MODEL_MAX      64U
#define GUARD_BYTE     0xCCU
#define GUARD_LEN      4U
#define FUZZ_STEPS     20000U
#define LCG_MUL        1103515245U
#define LCG_ADD        12345U
#define BAD_ID         9

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

/* Wire length of a frame for `topic`, 0 for an unknown topic. */
static uint16_t wire_len (uint8_t topic)
{
    switch (topic)
    {
        case 'h':
            return HEARTBEAT_LEN;
        case 'e':
        case 's':
            return STATUS_LEN;
        case 't':
            return SYSTEM_LEN;
        default:
            return 0U;
    }
}

/* Bytes one message of `topic` takes in the 384-byte buffer (D87). */
static unsigned cost_of (uint8_t topic)
{
    switch (topic)
    {
        case 'h':
            return COST_HEARTBEAT;
        case 'e':
            return COST_EVENT;
        case 's':
            return COST_STATUS;
        default:
            return COST_SYSTEM;
    }
}

/* Wire frame for `topic` filled from `seed`; an event's byte 7 is 0. */
static void make_frame (uint8_t * frame, uint8_t topic, uint8_t seed)
{
    fill_frame(frame, wire_len(topic), seed);
    if (topic == (uint8_t)'e')
    {
        frame[EVENT_LEN - 1U] = 0U;
    }
}

/* Push a frame of `len` bytes filled from `seed` (len may be wrong for the
   topic, to test validation). */
static ER push_seed (uint16_t len, uint8_t topic, uint8_t qos, uint8_t seed)
{
    uint8_t frame[FRAME_MAX];

    fill_frame(frame, len, seed);
    if ((topic == (uint8_t)'e') && (len == EVENT_LEN))
    {
        len = TELEMETRY_EVENT_QUEUED_SIZE;
    }
    return telemetry_queue_push(frame, len, qos);
}

/* Push a valid frame with the queued length of `topic`. */
static ER push_t (uint8_t topic, uint8_t qos, uint8_t seed)
{
    uint8_t frame[FRAME_MAX];

    make_frame(frame, topic, seed);
    if (topic == (uint8_t)'e')
    {
        return telemetry_queue_push(frame, TELEMETRY_EVENT_QUEUED_SIZE, qos);
    }
    return telemetry_queue_push(frame, wire_len(topic), qos);
}

/* Pop one message and check it is `topic` + the frame made from `seed`, with
   nothing written past the returned length. */
static int pop_t (uint8_t topic, uint8_t seed)
{
    uint8_t  msg[MSG_MAX + GUARD_LEN];
    uint8_t  want[FRAME_MAX];
    uint16_t len = wire_len(topic);
    INT      got;
    int      ok;

    memset(msg, (int)GUARD_BYTE, sizeof msg);
    got = telemetry_queue_pop(msg, TMO_POL);
    make_frame(want, topic, seed);
    ok = (got == (INT)(len + 1U)) && (msg[0] == topic) &&
         (memcmp(&msg[1], want, len) == 0);
    for (unsigned i = (unsigned)len + 1U; i < sizeof msg; i++)
    {
        ok = ok && (msg[i] == GUARD_BYTE);
    }
    return ok;
}

static int queue_empty (void)
{
    uint8_t msg[MSG_MAX];

    return telemetry_queue_pop(msg, TMO_POL) == E_TMOUT;
}

static void setup (void)
{
    mbf_fake_reset();
    g_mbf_telem_id = 0;
    CHECK(telemetry_queue_create() == E_OK);
}

/* Push `count` valid frames of `topic` (qos 1 so none is evicted) with seeds
   0..count-1; returns how many were stored. */
static unsigned fill_with (uint8_t topic, unsigned count)
{
    unsigned stored = 0U;

    for (unsigned i = 0U; i < count; i++)
    {
        if (push_t(topic, 1U, (uint8_t)i) == E_OK)
        {
            stored++;
        }
    }
    return stored;
}

static void pass (const char * name, unsigned fail_before)
{
    if (g_fail == fail_before)
    {
        (void)printf("PASS %s\n", name);
    }
    else
    {
        (void)printf("FAIL %s\n", name);
    }
}

/* ---- fake sanity: its capacity math must equal the kernel's ------------- */

static void test_fake_matches_kernel (void)
{
    static uint8_t buf[BUFFER_BYTES] __attribute__((aligned(4)));
    uint8_t        msg[32];
    uint8_t        out[32];
    T_CMBF         cfg = {0};
    ID             id;
    unsigned       before = g_fail;

    mbf_fake_reset();
    cfg.mbfatr = TA_USERBUF | TA_TPRI;
    cfg.bufsz  = BUFFER_BYTES;
    cfg.maxmsz = MSG_MAX;
    cfg.bufptr = buf;
    id = tk_cre_mbf(&cfg);
    CHECK(id > 0);
    memset(msg, 0xA5, sizeof msg);

    CHECK(MBF_FAKE_COST(1U) == 8U);
    CHECK(MBF_FAKE_COST(4U) == 8U);
    CHECK(MBF_FAKE_COST(5U) == 12U);
    CHECK(MBF_FAKE_COST(9U) == 16U);
    CHECK(MBF_FAKE_COST(17U) == 24U);

    for (unsigned i = 0U; i < CAP_17; i++)
    {
        CHECK(tk_snd_mbf(id, msg, (INT)MSG_MAX, TMO_POL) == E_OK);
    }
    CHECK(mbf_fake_free(id) == 0);
    CHECK(tk_snd_mbf(id, msg, 1, TMO_POL) == E_TMOUT);
    CHECK(tk_rcv_mbf(id, out, TMO_POL) == (INT)MSG_MAX);
    CHECK(mbf_fake_free(id) == 24);
    CHECK(tk_snd_mbf(id, msg, (INT)MSG_MAX, TMO_POL) == E_OK);
    CHECK(tk_snd_mbf(id, msg, (INT)MSG_MAX, TMO_FEVR) == E_TMOUT);
    CHECK(g_mbf_fake.would_block == 1U);

    CHECK(tk_snd_mbf(id, msg, (INT)MSG_MAX + 1, TMO_POL) == E_PAR);
    CHECK(tk_snd_mbf(id, msg, 0, TMO_POL) == E_PAR);
    CHECK(tk_snd_mbf(BAD_ID, msg, 1, TMO_POL) == E_ID);
    CHECK(tk_rcv_mbf(BAD_ID, out, TMO_POL) == E_ID);
    CHECK(tk_snd_mbf(2, msg, 1, TMO_POL) == E_NOEXS);

    /* tk_ref_mbf: size of the oldest message, 0 when empty. */
    {
        T_RMBF info;

        mbf_fake_reset();
        id = tk_cre_mbf(&cfg);
        CHECK(tk_ref_mbf(id, &info) == E_OK);
        CHECK(info.msgsz == 0);
        CHECK(info.frbufsz == (SZ)BUFFER_BYTES);
        CHECK(info.maxmsz == (INT)MSG_MAX);
        CHECK(tk_snd_mbf(id, msg, 7, TMO_POL) == E_OK);
        CHECK(tk_snd_mbf(id, msg, 16, TMO_POL) == E_OK);
        CHECK(tk_ref_mbf(id, &info) == E_OK);
        CHECK(info.msgsz == 7); /* the oldest, not the rounded size */
        CHECK(info.frbufsz == (SZ)(BUFFER_BYTES - 12U - 20U));
        CHECK(tk_rcv_mbf(id, out, TMO_POL) == 7);
        CHECK(tk_ref_mbf(id, &info) == E_OK);
        CHECK(info.msgsz == 16);
        CHECK(tk_ref_mbf(BAD_ID, &info) == E_ID);
        CHECK(tk_ref_mbf(2, &info) == E_NOEXS);
        CHECK(g_mbf_fake.ref_calls == 5U);
    }

    /* A 9-byte message returns 9, not the rounded 12. */
    mbf_fake_reset();
    id = tk_cre_mbf(&cfg);
    CHECK(tk_snd_mbf(id, msg, 9, TMO_POL) == E_OK);
    CHECK(mbf_fake_free(id) == (int32_t)(BUFFER_BYTES - 16U));
    CHECK(tk_rcv_mbf(id, out, TMO_POL) == 9);
    CHECK(tk_rcv_mbf(id, out, TMO_POL) == E_TMOUT);

    /* Create-time validation. */
    mbf_fake_reset();
    cfg.maxmsz = 0;
    CHECK(tk_cre_mbf(&cfg) == E_PAR);
    cfg.maxmsz = MSG_MAX;
    cfg.bufsz  = BUFFER_BYTES - 1U;
    CHECK(tk_cre_mbf(&cfg) == E_PAR);
    cfg.bufsz = BUFFER_BYTES;
    CHECK(tk_cre_mbf(&cfg) > 0);
    CHECK(tk_cre_mbf(&cfg) > 0);
    CHECK(tk_cre_mbf(&cfg) == E_LIMIT);
    pass("fake_matches_kernel", before);
}

/* ---- create ------------------------------------------------------------- */

static void test_create_failure_paths (void)
{
    static uint8_t spare[BUFFER_BYTES] __attribute__((aligned(4)));
    uint8_t        msg[MSG_MAX];
    T_CMBF         cfg = {0};
    unsigned       before = g_fail;

    mbf_fake_reset();
    g_mbf_telem_id = 0;

    /* Nothing created yet. */
    CHECK(push_t((uint8_t)'s', 0U, 0U) == E_NOEXS);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == E_NOEXS);
    CHECK(g_mbf_fake.send_calls == 0U);
    CHECK(g_mbf_fake.recv_calls == 0U);

    /* Kernel refuses: the error is returned and the id stays unset. */
    g_mbf_fake.create_error = E_NOMEM;
    CHECK(telemetry_queue_create() == E_NOMEM);
    CHECK(g_mbf_telem_id == 0);
    CHECK(push_t((uint8_t)'s', 0U, 0U) == E_NOEXS);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == E_NOEXS);

    g_mbf_fake.create_error = E_LIMIT;
    CHECK(telemetry_queue_create() == E_LIMIT);
    CHECK(g_mbf_telem_id == 0);

    /* No free control block (all kernel slots taken by other buffers). */
    cfg.mbfatr = TA_USERBUF;
    cfg.bufsz  = BUFFER_BYTES;
    cfg.maxmsz = MSG_MAX;
    cfg.bufptr = spare;
    CHECK(tk_cre_mbf(&cfg) > 0);
    CHECK(tk_cre_mbf(&cfg) > 0);
    CHECK(telemetry_queue_create() == E_LIMIT);
    CHECK(g_mbf_telem_id == 0);

    /* Retry after the failure succeeds. */
    mbf_fake_reset();
    CHECK(telemetry_queue_create() == E_OK);
    CHECK(g_mbf_telem_id > 0);
    CHECK(g_mbf_fake.create_calls == 1U);
    CHECK(push_t((uint8_t)'s', 0U, 0U) == E_OK);

    /* A second create keeps the queue and its contents. */
    CHECK(telemetry_queue_create() == E_OK);
    CHECK(g_mbf_fake.create_calls == 1U);
    CHECK(pop_t((uint8_t)'s', 0U));
    pass("create_failure_paths", before);
}

static void test_create_parameters (void)
{
    unsigned before = g_fail;

    setup();
    /* S03: 384-byte user buffer, kernel-managed wait order irrelevant. */
    CHECK(g_mbf_fake.create_calls == 1U);
    CHECK((g_mbf_fake.last_cmbf.mbfatr & TA_USERBUF) != 0U);
    CHECK(g_mbf_fake.last_cmbf.bufsz == (SZ)TELEMETRY_QUEUE_BYTES);
    CHECK(g_mbf_fake.last_cmbf.bufsz == (SZ)BUFFER_BYTES);
    CHECK(g_mbf_fake.last_cmbf.maxmsz == (INT)TELEMETRY_QUEUE_MAX_MSG);
    CHECK(g_mbf_fake.last_cmbf.maxmsz == (INT)MSG_MAX);
    CHECK(g_mbf_fake.last_cmbf.bufptr != 0);
    CHECK((((uintptr_t)g_mbf_fake.last_cmbf.bufptr) % 4U) == 0U);
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)BUFFER_BYTES);
    pass("create_parameters", before);
}

static void test_pop_arguments (void)
{
    uint8_t  msg[MSG_MAX];
    unsigned before = g_fail;

    setup();
    CHECK(telemetry_queue_pop(0, TMO_POL) == E_PAR);
    CHECK(g_mbf_fake.recv_calls == 0U);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == E_TMOUT); /* empty */

    /* The timeout goes to the kernel unchanged. */
    CHECK(telemetry_queue_pop(msg, 123) == E_TMOUT);
    CHECK(g_mbf_fake.last_rcv_tmout == 123);
    CHECK(telemetry_queue_pop(msg, TMO_FEVR) == E_TMOUT);
    CHECK(g_mbf_fake.last_rcv_tmout == TMO_FEVR);

    /* NULL is rejected before a missing queue is. */
    g_mbf_telem_id = 0;
    CHECK(telemetry_queue_pop(0, TMO_POL) == E_PAR);
    pass("pop_arguments", before);
}

/* ---- argument validation ------------------------------------------------ */

static void test_push_arguments (void)
{
    static const uint8_t topics[] = {'h', 'e', 's', 't'};
    uint8_t              frame[FRAME_MAX + 1U] = {0};
    unsigned             before = g_fail;
    unsigned             sent;

    setup();
    sent = g_mbf_fake.send_calls;

    CHECK(telemetry_queue_push(0, STATUS_LEN, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 0U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, FRAME_MAX + 1U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, UINT16_MAX, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 1U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 2U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 3U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 5U, 0U) == E_PAR);
    CHECK(g_mbf_fake.send_calls == sent); /* nothing reached the kernel */
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)BUFFER_BYTES);

    /* Bad arguments are reported before a missing queue is. */
    g_mbf_telem_id = 0;
    CHECK(telemetry_queue_push(0, STATUS_LEN, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 0U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 15U, 0U) == E_PAR);
    CHECK(push_t((uint8_t)'s', 0U, 0U) == E_NOEXS);

    /* Every topic is accepted and round-trips as the first byte. */
    setup();
    for (unsigned t = 0U; t < sizeof topics; t++)
    {
        CHECK(push_t(topics[t], 0U, (uint8_t)t) == E_OK);
    }
    for (unsigned t = 0U; t < sizeof topics; t++)
    {
        CHECK(pop_t(topics[t], (uint8_t)t));
    }
    pass("push_arguments", before);
}

/* D87: only these queued lengths are valid: h 4, e 7, s 8, t 16. */
static void test_topic_length_validation (void)
{
    uint8_t  frame[FRAME_MAX + 1U] = {0};
    unsigned before = g_fail;
    unsigned drops;

    setup();
    drops = telemetry_queue_drop_count();
    for (uint16_t len = 1U; len <= FRAME_MAX + 1U; len++)
    {
        const unsigned sent = g_mbf_fake.send_calls;
        const ER       r    = telemetry_queue_push(frame, len, 0U);

        if ((len == 4U) || (len == 7U) || (len == 8U) || (len == 16U))
        {
            CHECK(r == E_OK);
            CHECK(g_mbf_fake.send_calls == (sent + 1U));
            CHECK(telemetry_queue_pop(frame, TMO_POL) > 0);
        }
        else
        {
            CHECK(r == E_PAR);
            CHECK(g_mbf_fake.send_calls == sent);
        }
        memset(frame, 0, sizeof frame);
    }
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)BUFFER_BYTES);
    CHECK(telemetry_queue_drop_count() == drops);
    pass("topic_length_validation", before);
}

static void test_event_reserved_byte (void)
{
    uint8_t  frame[FRAME_MAX];
    uint8_t  msg[MSG_MAX];
    unsigned before = g_fail;

    setup();
    /* Status (8 B) keeps byte 7 intact in the queue and on pop. */
    fill_frame(frame, STATUS_LEN, 0x10U);
    frame[STATUS_LEN - 1U] = 0x5AU;
    CHECK(telemetry_queue_push(frame, STATUS_LEN, 0U) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == (INT)(STATUS_LEN + 1U));
    CHECK(msg[0] == (uint8_t)'s');
    CHECK(msg[STATUS_LEN] == 0x5AU);

    /* Event is queued as 7 B and byte 7 is restored as 0 on publish/pop. */
    fill_frame(frame, TELEMETRY_EVENT_QUEUED_SIZE, 0x10U);
    CHECK(telemetry_queue_push(frame, TELEMETRY_EVENT_QUEUED_SIZE, 1U) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == (INT)(EVENT_LEN + 1U));
    CHECK(msg[0] == (uint8_t)'e');
    CHECK(msg[EVENT_LEN] == 0U);
    pass("event_reserved_byte", before);
}

/* ---- stored format and pop output --------------------------------------- */

/* The kernel sees 4, 7, 8 and 16 bytes with no topic byte; an event is its
   wire frame without byte 7. */
static void test_stored_lengths_and_content (void)
{
    static const uint8_t topics[] = {'h', 'e', 's', 't'};
    static const INT     stored[] = {4, 7, 8, 16};
    uint8_t              raw[MSG_MAX];
    uint8_t              frame[FRAME_MAX];
    T_RMBF               info;
    unsigned             before = g_fail;

    for (unsigned t = 0U; t < sizeof topics; t++)
    {
        int32_t free_before;

        setup();
        make_frame(frame, topics[t], 0x30U);
        free_before = mbf_fake_free(g_mbf_telem_id);
        CHECK(telemetry_queue_push(frame, (uint16_t)stored[t], 0U) == E_OK);
        CHECK(tk_ref_mbf(g_mbf_telem_id, &info) == E_OK);
        CHECK(info.msgsz == stored[t]);
        CHECK((free_before - mbf_fake_free(g_mbf_telem_id)) ==
              (int32_t)cost_of(topics[t]));
        CHECK(tk_rcv_mbf(g_mbf_telem_id, raw, TMO_POL) == stored[t]);
        CHECK(memcmp(raw, frame, (size_t)stored[t]) == 0);
    }
    CHECK(TELEMETRY_HEARTBEAT_QUEUED_SIZE == 4U);
    CHECK(TELEMETRY_EVENT_QUEUED_SIZE == 7U);
    CHECK(TELEMETRY_STATUS_QUEUED_SIZE == 8U);
    CHECK(TELEMETRY_SYSTEM_QUEUED_SIZE == 16U);
    pass("stored_lengths_and_content", before);
}

/* pop returns topic || frame, the same bytes and length as before D87. */
static void test_pop_output_unchanged (void)
{
    static const uint8_t topics[] = {'h', 'e', 's', 't'};
    uint8_t              msg[MSG_MAX];
    unsigned             before = g_fail;
    INT                  got;

    setup();
    CHECK(push_seed(HEARTBEAT_LEN, (uint8_t)'h', 0U, 0x10U) == E_OK);
    got = telemetry_queue_pop(msg, TMO_POL);
    CHECK(got == (INT)(HEARTBEAT_LEN + 1U));
    CHECK(msg[0] == (uint8_t)'h');
    CHECK(msg[1] == 0x10U);
    CHECK(msg[4] == 0x13U);

    CHECK(push_seed(SYSTEM_LEN, (uint8_t)'t', 0U, 0x20U) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == (INT)MSG_MAX);
    CHECK(msg[0] == (uint8_t)'t');
    CHECK(msg[SYSTEM_LEN] == 0x2FU);

    /* Event: 9 bytes, byte 8 (the reserved byte) is 0 again. */
    CHECK(push_seed(EVENT_LEN, (uint8_t)'e', 1U, 0x40U) == E_OK);
    memset(msg, 0xFF, sizeof msg);
    got = telemetry_queue_pop(msg, TMO_POL);
    CHECK(got == (INT)(EVENT_LEN + 1U));
    CHECK(msg[0] == (uint8_t)'e');
    CHECK(msg[1] == 0x40U);
    CHECK(msg[7] == 0x46U);
    CHECK(msg[8] == 0U);
    CHECK(msg[9] == 0xFFU); /* nothing written past the frame */

    CHECK(push_seed(STATUS_LEN, (uint8_t)'s', 0U, 0x50U) == E_OK);
    got = telemetry_queue_pop(msg, TMO_POL);
    CHECK(got == (INT)(STATUS_LEN + 1U));
    CHECK(msg[0] == (uint8_t)'s');
    CHECK(msg[8] == 0x57U);

    /* All four, back to back, in a full-width ring. */
    for (unsigned round = 0U; round < 40U; round++)
    {
        for (unsigned t = 0U; t < sizeof topics; t++)
        {
            CHECK(push_t(topics[t], 0U, (uint8_t)(round + t)) == E_OK);
        }
        for (unsigned t = 0U; t < sizeof topics; t++)
        {
            CHECK(pop_t(topics[t], (uint8_t)(round + t)));
        }
    }
    CHECK(queue_empty());
    pass("pop_output_unchanged", before);
}

/* A foreign message (length 5, not a telemetry frame) is dropped and counted
   on pop, and is evictable. */
static void test_foreign_length (void)
{
    uint8_t  raw[5] = {1, 2, 3, 4, 5};
    uint8_t  msg[MSG_MAX];
    unsigned drops;
    unsigned before = g_fail;

    setup();
    drops = telemetry_queue_drop_count();
    CHECK(tk_snd_mbf(g_mbf_telem_id, raw, 5, TMO_POL) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == E_OBJ);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    CHECK(queue_empty());
    pass("foreign_length", before);
}

static void test_fifo_order (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(push_t((uint8_t)'s', 0U, 1U) == E_OK);
    CHECK(push_t((uint8_t)'e', 1U, 2U) == E_OK);
    CHECK(push_t((uint8_t)'h', 0U, 3U) == E_OK);
    CHECK(push_t((uint8_t)'t', 0U, 4U) == E_OK);
    CHECK(pop_t((uint8_t)'s', 1U));
    CHECK(pop_t((uint8_t)'e', 2U));
    CHECK(pop_t((uint8_t)'h', 3U));
    CHECK(pop_t((uint8_t)'t', 4U));
    CHECK(queue_empty());
    pass("fifo_order", before);
}

/* ---- capacity and non-blocking ------------------------------------------ */

static void test_capacity_of_384_bytes (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM + 4U) == CAP_SYSTEM);
    setup();
    CHECK(fill_with((uint8_t)'s', CAP_STATUS + 4U) == CAP_STATUS);
    setup();
    CHECK(fill_with((uint8_t)'e', CAP_EVENT + 4U) == CAP_EVENT);
    setup();
    CHECK(fill_with((uint8_t)'h', CAP_HEARTBEAT + 4U) == CAP_HEARTBEAT);
    CHECK(mbf_fake_free(g_mbf_telem_id) == 0);
    CHECK((CAP_SYSTEM * COST_SYSTEM) <= BUFFER_BYTES);
    CHECK(((CAP_SYSTEM + 1U) * COST_SYSTEM) > BUFFER_BYTES);
    pass("capacity_of_384_bytes", before);
}

static void test_producers_never_block (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    CHECK(push_t((uint8_t)'t', 1U, 0U) == E_TMOUT);
    CHECK(push_t((uint8_t)'t', 0U, 0U) == E_OK);
    CHECK(g_mbf_fake.send_calls > CAP_SYSTEM);
    CHECK(g_mbf_fake.last_snd_tmout == TMO_POL);
    CHECK(g_mbf_fake.last_rcv_tmout == TMO_POL); /* eviction read */
    CHECK(g_mbf_fake.would_block == 0U);

    /* Also when the eviction ends in a rejection. */
    setup();
    CHECK(push_t((uint8_t)'e', 1U, 0U) == E_OK);
    CHECK(fill_with((uint8_t)'s', CAP_STATUS - 1U) == CAP_STATUS - 1U);
    CHECK(push_t((uint8_t)'s', 0U, 0U) == E_TMOUT);
    CHECK(g_mbf_fake.would_block == 0U);
    pass("producers_never_block", before);
}

static void test_nonzero_qos_rejected_when_full (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    drops = telemetry_queue_drop_count();

    CHECK(push_seed(STATUS_LEN, (uint8_t)'e', 1U, 99U) == E_TMOUT);
    CHECK(push_seed(STATUS_LEN, (uint8_t)'e', 2U, 99U) == E_TMOUT);
    CHECK(push_seed(STATUS_LEN, (uint8_t)'e', 255U, 99U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops); /* a rejection is no drop */
    CHECK(g_mbf_fake.recv_calls == 0U);           /* and nothing is evicted */
    CHECK(g_mbf_fake.ref_calls == 0U);
    CHECK(mbf_fake_free(g_mbf_telem_id) == 4);

    for (unsigned i = 0U; i < CAP_SYSTEM; i++)
    {
        CHECK(pop_t((uint8_t)'t', (uint8_t)i));
    }
    CHECK(telemetry_queue_drop_count() == drops);
    pass("nonzero_qos_rejected_when_full", before);
}

/* ---- QoS 0 eviction and the drop counter -------------------------------- */

static void test_qos0_evicts_oldest_and_counts (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    drops = telemetry_queue_drop_count();

    CHECK(push_t((uint8_t)'t', 0U, 100U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U);

    /* Message 0 is gone; 1..18 remain in order; the new one is last. */
    for (unsigned i = 1U; i < CAP_SYSTEM; i++)
    {
        CHECK(pop_t((uint8_t)'t', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'t', 100U));
    CHECK(queue_empty());

    /* Every further eviction adds exactly one. */
    setup();
    drops = telemetry_queue_drop_count();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    for (uint8_t n = 0U; n < 5U; n++)
    {
        CHECK(push_t((uint8_t)'t', 0U, (uint8_t)(50U + n)) == E_OK);
        CHECK(telemetry_queue_drop_count() == drops + n + 1U);
    }
    CHECK(pop_t((uint8_t)'t', 5U)); /* 0..4 evicted */
    pass("qos0_evicts_oldest_and_counts", before);
}

static void test_no_drop_while_space_remains (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    drops = telemetry_queue_drop_count();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM - 1U) == CAP_SYSTEM - 1U);
    CHECK(push_t((uint8_t)'t', 0U, 7U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops);
    CHECK(g_mbf_fake.recv_calls == 0U);
    CHECK(g_mbf_fake.ref_calls == 0U);
    pass("no_drop_while_space_remains", before);
}

static void test_small_frame_evicts_larger_oldest (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    drops = telemetry_queue_drop_count();

    /* Evicting one 20-byte message frees room for an 8-byte one. */
    CHECK(push_t((uint8_t)'h', 0U, 9U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)(24U - COST_HEARTBEAT));
    pass("small_frame_evicts_larger_oldest", before);
}

/* Spec S03: "a QoS 0 push drops the oldest frame and increments the drop
   counter". S35: eviction never removes a QoS 1 event. The oldest message is
   an event, so the QoS 0 push is rejected, counted, and evicts nothing. */
static void test_qos1_event_is_never_evicted (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(push_t((uint8_t)'e', 1U, 0xE0U) == E_OK);
    CHECK(fill_with((uint8_t)'s', CAP_STATUS - 1U) == CAP_STATUS - 1U);
    drops = telemetry_queue_drop_count();

    CHECK(push_t((uint8_t)'s', 0U, 0x50U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    CHECK(g_mbf_fake.recv_calls == 0U); /* nothing was evicted */

    CHECK(pop_t((uint8_t)'e', 0xE0U)); /* the event is still first */
    for (unsigned i = 0U; i < CAP_STATUS - 1U; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(queue_empty());
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    pass("qos1_event_is_never_evicted", before);
}

/* An event that becomes the oldest during eviction stops it. Frames evicted
   before that stay lost and are counted; the new frame is rejected and
   counted too. */
static void test_event_at_head_stops_eviction (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(push_t((uint8_t)'s', 1U, 0U) == E_OK);
    CHECK(push_t((uint8_t)'e', 1U, 0xE1U) == E_OK);
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(push_t((uint8_t)'s', 1U, (uint8_t)i) == E_OK);
    }
    CHECK(mbf_fake_free(g_mbf_telem_id) == 0);
    drops = telemetry_queue_drop_count();

    /* A 20-byte system frame needs two 12-byte slots. One status is evicted,
       then the event is at the head. */
    CHECK(push_t((uint8_t)'t', 0U, 0x70U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 2U); /* evicted + new */
    CHECK(g_mbf_fake.recv_calls == 1U);

    /* An 8-byte heartbeat fits in the 12 bytes that were freed. */
    CHECK(push_t((uint8_t)'h', 0U, 0x71U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 2U);

    CHECK(pop_t((uint8_t)'e', 0xE1U));
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'h', 0x71U));
    CHECK(queue_empty());
    pass("event_at_head_stops_eviction", before);
}

/* Realistic sizes from src/main.c: status is 8 B (12 B in the buffer),
   system is 16 B (20 B). A full queue of status frames takes a system frame
   by evicting two status frames; both are counted. */
static void test_qos0_evicts_several_to_fit (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'s', CAP_STATUS) == CAP_STATUS);
    drops = telemetry_queue_drop_count();

    CHECK(push_t((uint8_t)'t', 0U, 0x77U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 2U);
    CHECK(g_mbf_fake.recv_calls == 2U);
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)(24U - COST_SYSTEM));
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'t', 0x77U));
    CHECK(queue_empty());

    /* Heartbeats (8 B) evicted for a system frame (20 B): three go. */
    setup();
    CHECK(fill_with((uint8_t)'h', CAP_HEARTBEAT) == CAP_HEARTBEAT);
    drops = telemetry_queue_drop_count();
    CHECK(push_t((uint8_t)'t', 0U, 0x78U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 3U);
    pass("qos0_evicts_several_to_fit", before);
}

static void test_kernel_errors_pass_through (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    drops = telemetry_queue_drop_count();

    /* A send error other than E_TMOUT is returned and nothing is evicted. */
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    g_mbf_fake.send_error = E_NOMEM;
    CHECK(push_t((uint8_t)'t', 0U, 0U) == E_NOMEM);
    CHECK(g_mbf_fake.recv_calls == 0U);
    CHECK(telemetry_queue_drop_count() == drops);

    /* If the eviction read fails, the original E_TMOUT is returned and the
       rejected frame is counted. */
    g_mbf_fake.recv_error = E_NOMEM;
    CHECK(push_t((uint8_t)'t', 0U, 0U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 1U);

    /* Same when the size peek fails. */
    g_mbf_fake.ref_error = E_NOMEM;
    CHECK(push_t((uint8_t)'t', 0U, 0U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 2U);
    for (unsigned i = 0U; i < CAP_SYSTEM; i++)
    {
        CHECK(pop_t((uint8_t)'t', (uint8_t)i));
    }

    /* Same for a failed QoS 1 send. */
    g_mbf_fake.send_error = E_NOMEM;
    CHECK(push_t((uint8_t)'e', 1U, 0U) == E_NOMEM);
    CHECK(telemetry_queue_drop_count() == drops + 2U);
    pass("kernel_errors_pass_through", before);
}

/* ---- a consumer on the other core pops during eviction ------------------- */

static void consumer_pops_one (void)
{
    uint8_t msg[MSG_MAX];

    (void)telemetry_queue_pop(msg, TMO_POL);
}

/* The consumer pops between the producer's peek and its receive. The oldest
   message the producer sees is then the next one. */
static void test_consumer_races_ahead (void)
{
    unsigned drops;
    unsigned before = g_fail;

    /* The next message is a status: it is evicted instead, counted once. */
    setup();
    CHECK(fill_with((uint8_t)'s', CAP_STATUS) == CAP_STATUS);
    drops = telemetry_queue_drop_count();
    g_mbf_fake.recv_hook = consumer_pops_one; /* pops seed 0 */
    CHECK(push_t((uint8_t)'t', 0U, 0x60U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U); /* seed 1 */
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'t', 0x60U));
    CHECK(queue_empty());

    /* The next message is an event: the producer's receive returns it. It must
       not be lost; it is sent back (after the newer frames). The space the
       consumer and the receive freed now fits the new status, so nothing
       else is evicted and nothing is dropped. */
    setup();
    CHECK(push_t((uint8_t)'s', 1U, 0U) == E_OK);
    CHECK(push_t((uint8_t)'e', 1U, 0xE2U) == E_OK);
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(push_t((uint8_t)'s', 1U, (uint8_t)i) == E_OK);
    }
    drops = telemetry_queue_drop_count();
    g_mbf_fake.recv_hook = consumer_pops_one; /* pops seed 0 */
    CHECK(push_t((uint8_t)'s', 0U, 0x61U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops);
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'e', 0xE2U)); /* the event survived */
    CHECK(pop_t((uint8_t)'s', 0x61U));
    CHECK(queue_empty());
    CHECK(g_mbf_fake.would_block == 0U);

    /* Same race with a 16 B system frame (20 B): after the event is sent back
       the next oldest is a status, which is evicted (counted once) and the
       new frame fits. The loop does not stop at the event it sent back. */
    setup();
    CHECK(push_t((uint8_t)'s', 1U, 0U) == E_OK);
    CHECK(push_t((uint8_t)'e', 1U, 0xE2U) == E_OK);
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(push_t((uint8_t)'s', 1U, (uint8_t)i) == E_OK);
    }
    drops = telemetry_queue_drop_count();
    g_mbf_fake.recv_hook = consumer_pops_one; /* pops seed 0 */
    CHECK(push_t((uint8_t)'t', 0U, 0x63U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U); /* seed 2 */
    for (unsigned i = 3U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'e', 0xE2U));
    CHECK(pop_t((uint8_t)'t', 0x63U));
    CHECK(queue_empty());

    /* The consumer's pop frees a slot first: the producer still evicts one
       more (it only learns the size at the next peek) and succeeds. */
    setup();
    CHECK(fill_with((uint8_t)'t', CAP_SYSTEM) == CAP_SYSTEM);
    g_mbf_fake.recv_hook = consumer_pops_one;
    drops = telemetry_queue_drop_count();
    CHECK(push_t((uint8_t)'t', 0U, 0x62U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    pass("consumer_races_ahead", before);
}

/* ---- producer mutex ------------------------------------------------------
 * Producers serialise on a priority-inheritance mutex, not a spinlock:
 * tk_rcv_mbf returns E_CTX with interrupts or dispatch disabled, so eviction
 * cannot run under a spinlock. The fake counts message buffer calls made
 * outside the mutex. */

static void test_producer_mutex (void)
{
    uint8_t  msg[MSG_MAX];
    unsigned locks;
    unsigned sends;
    unsigned drops;
    unsigned before = g_fail;

    /* Creation: TA_INHERIT only (TA_INHERIT | TA_TPRI is TA_CEILING). */
    setup();
    CHECK(g_mbf_fake.mtx_create_calls == 1U);
    CHECK(g_mbf_fake.mtx_used == 1);
    CHECK(g_mbf_fake.mtx_atr == TA_INHERIT);
    CHECK(telemetry_queue_create() == E_OK); /* second create: no new mutex */
    CHECK(g_mbf_fake.mtx_create_calls == 1U);

    /* Mutex creation fails: error returned, no buffer created, retry works. */
    mbf_fake_reset();
    g_mbf_telem_id = 0;
    g_mbf_fake.mtx_create_error = E_LIMIT;
    CHECK(telemetry_queue_create() == E_LIMIT);
    CHECK(g_mbf_fake.create_calls == 0U);
    CHECK(g_mbf_telem_id == 0);
    CHECK(telemetry_queue_create() == E_OK);
    CHECK(g_mbf_fake.mtx_used == 1);

    /* Buffer creation fails: the mutex is deleted, retry creates both. */
    mbf_fake_reset();
    g_mbf_telem_id = 0;
    g_mbf_fake.create_error = E_NOMEM;
    CHECK(telemetry_queue_create() == E_NOMEM);
    CHECK(g_mbf_fake.mtx_used == 0);
    CHECK(g_mbf_fake.mtx_del_calls == 1U);
    CHECK(telemetry_queue_create() == E_OK);
    CHECK(g_mbf_fake.mtx_used == 1);

    /* A push takes the mutex once, waits for it, and releases it. */
    setup();
    CHECK(push_t((uint8_t)'s', 0U, 1U) == E_OK);
    CHECK(g_mbf_fake.mtx_lock_calls == 1U);
    CHECK(g_mbf_fake.mtx_unlock_calls == 1U);
    CHECK(g_mbf_fake.mtx_wait_forever == 1U);
    CHECK(g_mbf_fake.mtx_locked == 0);
    CHECK(g_mbf_fake.snd_ref_unlocked == 0U);

    /* Rejected arguments never reach the mutex. */
    locks = g_mbf_fake.mtx_lock_calls;
    CHECK(push_seed(15U, (uint8_t)'s', 0U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(0, STATUS_LEN, 0U) == E_PAR);
    CHECK(g_mbf_fake.mtx_lock_calls == locks);

    /* pop does not use it (it may block). */
    CHECK(telemetry_queue_pop(msg, TMO_POL) == (INT)(STATUS_LEN + 1U));
    CHECK(g_mbf_fake.mtx_lock_calls == locks);

    /* A lock error is returned; nothing is sent, counted or unlocked. */
    sends = g_mbf_fake.send_calls;
    drops = telemetry_queue_drop_count();
    g_mbf_fake.mtx_lock_error = E_CTX;
    CHECK(push_t((uint8_t)'s', 0U, 2U) == E_CTX);
    CHECK(g_mbf_fake.send_calls == sends);
    CHECK(g_mbf_fake.mtx_unlock_calls == g_mbf_fake.mtx_lock_calls - 1U);
    CHECK(telemetry_queue_drop_count() == drops);
    CHECK(queue_empty());

    /* Every path leaves the mutex free: store, evict, reject qos 1, and the
       reject at an event. Peeks and sends are inside it; the evicting
       receive is too. */
    setup();
    CHECK(fill_with((uint8_t)'s', CAP_STATUS) == CAP_STATUS);
    CHECK(push_t((uint8_t)'s', 1U, 0x70U) == E_TMOUT);
    CHECK(g_mbf_fake.mtx_locked == 0);
    CHECK(g_mbf_fake.rcv_locked == 0U);
    CHECK(push_t((uint8_t)'t', 0U, 0x71U) == E_OK); /* evicts two */
    CHECK(g_mbf_fake.mtx_locked == 0);
    CHECK(g_mbf_fake.rcv_locked >= 2U);
    CHECK(g_mbf_fake.snd_ref_unlocked == 0U);
    setup();
    CHECK(push_t((uint8_t)'e', 1U, 0xE3U) == E_OK);
    CHECK(fill_with((uint8_t)'s', CAP_STATUS - 1U) == CAP_STATUS - 1U);
    CHECK(push_t((uint8_t)'s', 0U, 0x72U) == E_TMOUT); /* event at head */
    CHECK(g_mbf_fake.mtx_locked == 0);
    CHECK(g_mbf_fake.snd_ref_unlocked == 0U);
    CHECK(g_mbf_fake.mtx_lock_calls == g_mbf_fake.mtx_unlock_calls);
    pass("producer_mutex", before);
}

/* ---- mixed traffic ------------------------------------------------------- */

typedef struct
{
    uint8_t topic;
    uint8_t seed;
} model_t;

static unsigned model_cost (const model_t * m, unsigned count)
{
    unsigned used = 0U;

    for (unsigned i = 0U; i < count; i++)
    {
        used += cost_of(m[i].topic);
    }
    return used;
}

/* Random pushes (qos 0 and 1, all topics) and pops against a model of the
   documented behavior: FIFO order, eviction only of qos 0 frames at the head,
   and pushed = popped + dropped + queued (+ rejected qos 1, which the caller
   sees as E_TMOUT and the queue does not count). */
static void test_mixed_traffic_fuzz (void)
{
    static const uint8_t topics[] = {'h', 'e', 's', 't', 's', 's', 'e', 't'};
    model_t              model[MODEL_MAX];
    unsigned             count = 0U;
    uint32_t             rng = 1U;
    unsigned             before = g_fail;
    unsigned             pushed = 0U;
    unsigned             popped = 0U;
    unsigned             dropped = 0U;
    unsigned             qos1_rejected = 0U;
    unsigned             stopped_by_event = 0U;
    unsigned             multi_evict = 0U;
    unsigned             base;

    setup();
    base = telemetry_queue_drop_count(); /* the counter outlives setup() */
    for (unsigned step = 0U; step < FUZZ_STEPS; step++)
    {
        rng = (rng * LCG_MUL) + LCG_ADD;
        if (((rng >> 16) % 5U) < 3U)
        {
            const uint8_t topic = topics[(rng >> 8) % sizeof topics];
            const uint8_t seed = (uint8_t)(rng >> 20);
            const uint8_t qos = (topic == (uint8_t)'e') ? 1U
                                : ((((rng >> 4) % 8U) == 0U) ? 1U : 0U);
            const unsigned cost = cost_of(topic);
            unsigned       evicted = 0U;
            ER             want = E_OK;
            const ER       r = push_t(topic, qos, seed);

            pushed++;
            if ((model_cost(model, count) + cost) > BUFFER_BYTES)
            {
                if (qos != 0U)
                {
                    want = E_TMOUT;
                    qos1_rejected++;
                }
                else
                {
                    while ((count > 0U) &&
                           (model[0].topic != (uint8_t)'e') &&
                           ((model_cost(model, count) + cost) > BUFFER_BYTES))
                    {
                        memmove(&model[0], &model[1],
                                (count - 1U) * sizeof model[0]);
                        count--;
                        dropped++;
                        evicted++;
                    }
                    if ((model_cost(model, count) + cost) > BUFFER_BYTES)
                    {
                        want = E_TMOUT;
                        dropped++;
                        stopped_by_event++;
                    }
                }
            }
            if (evicted > 1U)
            {
                multi_evict++;
            }
            if (want == E_OK)
            {
                model[count].topic = topic;
                model[count].seed  = seed;
                count++;
            }
            CHECK(r == want);
        }
        else if (count > 0U)
        {
            CHECK(pop_t(model[0].topic, model[0].seed));
            memmove(&model[0], &model[1], (count - 1U) * sizeof model[0]);
            count--;
            popped++;
        }
        else
        {
            CHECK(queue_empty());
        }
        CHECK(mbf_fake_free(g_mbf_telem_id) ==
              (int32_t)(BUFFER_BYTES - model_cost(model, count)));
        CHECK(telemetry_queue_drop_count() == (base + dropped));
        CHECK(pushed == (popped + dropped + count + qos1_rejected));
    }
    CHECK(stopped_by_event > 0U); /* an event at the head did stop eviction */
    CHECK(multi_evict > 0U);      /* and several frames were evicted at once */
    CHECK(qos1_rejected > 0U);
    CHECK(dropped > 0U);
    pass("mixed_traffic_fuzz", before);
}

int main (void)
{
    test_fake_matches_kernel();
    test_create_failure_paths();
    test_create_parameters();
    test_push_arguments();
    test_topic_length_validation();
    test_event_reserved_byte();
    test_stored_lengths_and_content();
    test_pop_output_unchanged();
    test_foreign_length();
    test_pop_arguments();
    test_fifo_order();
    test_capacity_of_384_bytes();
    test_producers_never_block();
    test_nonzero_qos_rejected_when_full();
    test_qos0_evicts_oldest_and_counts();
    test_no_drop_while_space_remains();
    test_small_frame_evicts_larger_oldest();
    test_qos1_event_is_never_evicted();
    test_event_at_head_stops_eviction();
    test_qos0_evicts_several_to_fit();
    test_kernel_errors_pass_through();
    test_consumer_races_ahead();
    test_producer_mutex();
    test_mixed_traffic_fuzz();

    (void)printf("%u checks, %u failed\n", g_checks, g_fail);
    return (g_fail == 0U) ? 0 : 1;
}
