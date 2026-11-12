/** @file test_telemetry_queue.c
 * @brief Host test for the Core 0 to Core 1 telemetry queue (S03, S46).
 *
 * src/telemetry/queue.c is compiled unchanged against the kernel shim. The
 * message buffer is the fake in tests/shim/tk/mbf_fake.h, which copies the
 * capacity and ring rules of lib/mtk3smp-rp2040/kernel/tkernel/messagebuf.c.
 * A queued frame is the topic byte plus the wire packet (D110), so a 1024-byte
 * buffer holds:
 *   physics   (25 B frame): 32 bytes each, 32 messages
 *   status    (17 B frame): 24 bytes each, 42 messages
 *   event     (13 B frame): 20 bytes each, 51 messages
 *   heartbeat ( 9 B frame): 16 bytes each, 64 messages
 *   profile   (83 B frame): 88 bytes each, 11 messages
 * pop returns the frame as it was pushed.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tk/mbf_fake.h>
#include "telemetry/queue.h"

#define BUFFER_BYTES   1024U
#define MSG_MAX        83U
#define FRAME_MAX      83U
#define FRAME_HEARTBEAT 9U
#define FRAME_EVENT    13U
#define FRAME_STATUS   17U
#define FRAME_PHYSICS  25U
#define FRAME_PROFILE  83U
#define CAP_PHYSICS    32U
#define CAP_STATUS     42U
#define CAP_EVENT      51U
#define CAP_HEARTBEAT  64U
#define COST_HEARTBEAT 16U
#define COST_EVENT     20U
#define COST_STATUS    24U
#define COST_PHYSICS   32U
#define COST_DIAG      40U
#define COST_PROFILE   88U
#define FILL_LEN       28U /* kernel math only: a 28 B message costs 32 B */
#define CAP_FILL       32U
#define EV_FILL_LAST   41U /* last status seed when an event is queued 2nd */
#define MODEL_MAX      80U
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

/* Frame length (topic byte + wire packet) for `topic`, 0 for an unknown one. */
static uint16_t frame_len (uint8_t topic)
{
    switch (topic)
    {
        case 'h':
            return FRAME_HEARTBEAT;
        case 'e':
            return FRAME_EVENT;
        case 's':
        case 'm':
            return FRAME_STATUS;
        case 'p':
        case 'o':
            return FRAME_PHYSICS;
        case 'd':
            return TELEMETRY_DIAG_WIRE_SIZE + 1U;
        case 'f':
            return FRAME_PROFILE;
        default:
            return 0U;
    }
}

/* Bytes one frame of `topic` takes in the 1024-byte buffer: 4 B header plus
   the length rounded up to 4 B. */
static unsigned cost_of (uint8_t topic)
{
    const unsigned len = frame_len(topic);

    return 4U + ((len + 3U) / 4U) * 4U;
}

/* A frame for `topic`: the topic byte, then bytes filled from `seed`. */
static void make_frame (uint8_t * frame, uint8_t topic, uint8_t seed)
{
    const uint16_t len = frame_len(topic);

    fill_frame(&frame[1], (uint16_t)(len - 1U), seed);
    frame[0] = topic;
}

/* Push a frame of `len` bytes whose first byte is `topic`, filled from `seed`
   (len may be wrong for the topic, to test validation). */
static ER push_seed (uint16_t len, uint8_t topic, uint8_t qos, uint8_t seed)
{
    uint8_t frame[FRAME_MAX + 1U];

    fill_frame(frame, len, seed);
    frame[0] = topic;
    return telemetry_queue_push(frame, len, qos);
}

/* Push a valid frame of `topic`. */
static ER push_t (uint8_t topic, uint8_t qos, uint8_t seed)
{
    uint8_t frame[FRAME_MAX];

    make_frame(frame, topic, seed);
    return telemetry_queue_push(frame, frame_len(topic), qos);
}

/* Pop one message and check it is the frame of `topic` made from `seed`, with
   nothing written past the returned length. */
static int pop_t (uint8_t topic, uint8_t seed)
{
    uint8_t  msg[MSG_MAX + GUARD_LEN];
    uint8_t  want[FRAME_MAX];
    uint16_t len = frame_len(topic);
    INT      got;
    int      ok;

    memset(msg, (int)GUARD_BYTE, sizeof msg);
    got = telemetry_queue_pop(msg, TMO_POL);
    make_frame(want, topic, seed);
    ok = (got == (INT)len) && (memcmp(msg, want, len) == 0);
    for (unsigned i = (unsigned)len; i < sizeof msg; i++)
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
    uint8_t        msg[FRAME_MAX];
    uint8_t        out[FRAME_MAX];
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
    CHECK(MBF_FAKE_COST(FILL_LEN) == 32U);
    CHECK(MBF_FAKE_COST(MSG_MAX) == COST_PROFILE);

    for (unsigned i = 0U; i < CAP_FILL; i++)
    {
        CHECK(tk_snd_mbf(id, msg, (INT)FILL_LEN, TMO_POL) == E_OK);
    }
    CHECK(mbf_fake_free(id) == 0);
    CHECK(tk_snd_mbf(id, msg, 1, TMO_POL) == E_TMOUT);
    CHECK(tk_rcv_mbf(id, out, TMO_POL) == (INT)FILL_LEN);
    CHECK(mbf_fake_free(id) == 32);
    CHECK(tk_snd_mbf(id, msg, (INT)FILL_LEN, TMO_POL) == E_OK);
    CHECK(tk_snd_mbf(id, msg, (INT)FILL_LEN, TMO_FEVR) == E_TMOUT);
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
    /* S03: 1024-byte user buffer, kernel-managed wait order irrelevant. */
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

static const uint8_t g_topics[] = {'h', 'e', 's', 'm', 'p', 'o', 'd', 'f'};

static void test_push_arguments (void)
{
    uint8_t  frame[FRAME_MAX + 1U] = {0};
    unsigned before = g_fail;
    unsigned sent;

    setup();
    sent = g_mbf_fake.send_calls;

    CHECK(telemetry_queue_push(0, FRAME_STATUS, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 0U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, FRAME_MAX + 1U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, UINT16_MAX, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 1U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 2U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, FRAME_STATUS, 0U) == E_PAR); /* topic 0 */
    CHECK(g_mbf_fake.send_calls == sent); /* nothing reached the kernel */
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)BUFFER_BYTES);

    /* The command topic is not a queue topic. */
    CHECK(push_seed(FRAME_STATUS, (uint8_t)'c', 0U, 1U) == E_PAR);
    /* The old system topic is gone. */
    CHECK(push_seed(FRAME_PHYSICS, (uint8_t)'t', 0U, 1U) == E_PAR);

    /* Bad arguments are reported before a missing queue is. */
    g_mbf_telem_id = 0;
    CHECK(telemetry_queue_push(0, FRAME_STATUS, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 0U, 0U) == E_PAR);
    CHECK(telemetry_queue_push(frame, 15U, 0U) == E_PAR);
    CHECK(push_t((uint8_t)'s', 0U, 0U) == E_NOEXS);

    /* Every topic is accepted and round-trips unchanged. */
    setup();
    for (unsigned t = 0U; t < sizeof g_topics; t++)
    {
        CHECK(push_t(g_topics[t], 0U, (uint8_t)t) == E_OK);
    }
    for (unsigned t = 0U; t < sizeof g_topics; t++)
    {
        CHECK(pop_t(g_topics[t], (uint8_t)t));
    }
    pass("push_arguments", before);
}

/* D110: a frame is valid only when its length matches its topic's wire size. */
static void test_topic_length_validation (void)
{
    uint8_t  frame[FRAME_MAX + 1U];
    unsigned before = g_fail;
    unsigned drops;

    setup();
    drops = telemetry_queue_drop_count();
    for (unsigned t = 0U; t < sizeof g_topics; t++)
    {
        for (uint16_t len = 1U; len <= FRAME_MAX + 1U; len++)
        {
            const unsigned sent = g_mbf_fake.send_calls;
            ER             r;

            fill_frame(frame, len, 0x11U);
            frame[0] = g_topics[t];
            r        = telemetry_queue_push(frame, len, 0U);
            if (len == frame_len(g_topics[t]))
            {
                CHECK(r == E_OK);
                CHECK(g_mbf_fake.send_calls == (sent + 1U));
                CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)len);
            }
            else
            {
                CHECK(r == E_PAR);
                CHECK(g_mbf_fake.send_calls == sent);
            }
        }
    }
    CHECK(mbf_fake_free(g_mbf_telem_id) == (int32_t)BUFFER_BYTES);
    CHECK(telemetry_queue_drop_count() == drops);
    pass("topic_length_validation", before);
}

/* ---- stored format and pop output --------------------------------------- */

/* The kernel sees the whole frame, topic byte first. */
static void test_stored_lengths_and_content (void)
{
    uint8_t  raw[MSG_MAX];
    uint8_t  frame[FRAME_MAX];
    T_RMBF   info;
    unsigned before = g_fail;

    for (unsigned t = 0U; t < sizeof g_topics; t++)
    {
        int32_t free_before;

        setup();
        make_frame(frame, g_topics[t], 0x30U);
        free_before = mbf_fake_free(g_mbf_telem_id);
        CHECK(telemetry_queue_push(frame, frame_len(g_topics[t]), 0U) == E_OK);
        CHECK(tk_ref_mbf(g_mbf_telem_id, &info) == E_OK);
        CHECK(info.msgsz == (INT)frame_len(g_topics[t]));
        CHECK((free_before - mbf_fake_free(g_mbf_telem_id)) ==
              (int32_t)cost_of(g_topics[t]));
        CHECK(tk_rcv_mbf(g_mbf_telem_id, raw, TMO_POL) ==
              (INT)frame_len(g_topics[t]));
        CHECK(memcmp(raw, frame, frame_len(g_topics[t])) == 0);
        CHECK(raw[0] == g_topics[t]);
    }
    CHECK(TELEMETRY_EVENT_QUEUED_SIZE == FRAME_EVENT);
    CHECK(TELEMETRY_QUEUE_MAX_MSG == FRAME_MAX);
    CHECK(cost_of((uint8_t)'h') == COST_HEARTBEAT);
    CHECK(cost_of((uint8_t)'e') == COST_EVENT);
    CHECK(cost_of((uint8_t)'s') == COST_STATUS);
    CHECK(cost_of((uint8_t)'p') == COST_PHYSICS);
    CHECK(cost_of((uint8_t)'d') == COST_DIAG);
    CHECK(cost_of((uint8_t)'f') == COST_PROFILE);
    pass("stored_lengths_and_content", before);
}

/* pop returns the frame as it was pushed, with nothing written past it. */
static void test_pop_output_is_the_frame (void)
{
    unsigned before = g_fail;

    setup();
    /* All topics, back to back, many times round the ring. */
    for (unsigned round = 0U; round < 40U; round++)
    {
        for (unsigned t = 0U; t < sizeof g_topics; t++)
        {
            CHECK(push_t(g_topics[t], 0U, (uint8_t)(round + t)) == E_OK);
        }
        for (unsigned t = 0U; t < sizeof g_topics; t++)
        {
            CHECK(pop_t(g_topics[t], (uint8_t)(round + t)));
        }
    }
    CHECK(queue_empty());
    pass("pop_output_is_the_frame", before);
}

/* A foreign message (first byte not a topic, or the wrong length) is dropped
   and counted on pop, and is evictable. */
static void test_foreign_length (void)
{
    uint8_t  raw[25] = {1, 2, 3, 4, 5};
    uint8_t  msg[MSG_MAX];
    unsigned drops;
    unsigned before = g_fail;

    setup();
    drops = telemetry_queue_drop_count();
    CHECK(tk_snd_mbf(g_mbf_telem_id, raw, 5, TMO_POL) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == E_OBJ);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    CHECK(queue_empty());

    /* A known topic with the wrong length is foreign too. */
    raw[0] = (uint8_t)'s';
    CHECK(tk_snd_mbf(g_mbf_telem_id, raw, 5, TMO_POL) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == E_OBJ);
    CHECK(telemetry_queue_drop_count() == drops + 2U);
    CHECK(queue_empty());

    /* An unknown message in the queue is evicted and counted. It costs 32
       bytes; 41 status frames cost 984. */
    raw[0] = 1U;
    CHECK(tk_snd_mbf(g_mbf_telem_id, raw, 25, TMO_POL) == E_OK);
    CHECK(fill_with((uint8_t)'s', CAP_STATUS - 1U) == CAP_STATUS - 1U);
    drops = telemetry_queue_drop_count();
    CHECK(push_t((uint8_t)'s', 0U, 0x55U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    for (unsigned i = 0U; i < CAP_STATUS - 1U; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'s', 0x55U));
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
    CHECK(push_t((uint8_t)'p', 0U, 4U) == E_OK);
    CHECK(pop_t((uint8_t)'s', 1U));
    CHECK(pop_t((uint8_t)'e', 2U));
    CHECK(pop_t((uint8_t)'h', 3U));
    CHECK(pop_t((uint8_t)'p', 4U));
    CHECK(queue_empty());
    pass("fifo_order", before);
}

/* telemetry_queue_push_packet builds topic || packet. */
static void test_push_packet (void)
{
    uint8_t  packet[TELEMETRY_WIRE_MAX];
    uint8_t  want[FRAME_MAX];
    uint8_t  msg[MSG_MAX];
    unsigned before = g_fail;

    setup();
    make_frame(want, (uint8_t)'m', 0x21U);
    memcpy(packet, &want[1], FRAME_STATUS - 1U);
    CHECK(telemetry_queue_push_packet((uint8_t)'m', packet,
                                      (uint16_t)(FRAME_STATUS - 1U), 0U) == E_OK);
    CHECK(telemetry_queue_pop(msg, TMO_POL) == (INT)FRAME_STATUS);
    CHECK(memcmp(msg, want, FRAME_STATUS) == 0);

    CHECK(telemetry_queue_push_packet((uint8_t)'m', 0, 16U, 0U) == E_PAR);
    CHECK(telemetry_queue_push_packet((uint8_t)'m', packet, 0U, 0U) == E_PAR);
    CHECK(telemetry_queue_push_packet((uint8_t)'m', packet,
                                      TELEMETRY_WIRE_MAX + 1U, 0U) == E_PAR);
    /* The queue still validates topic and length. */
    CHECK(telemetry_queue_push_packet((uint8_t)'m', packet, 15U, 0U) == E_PAR);
    CHECK(telemetry_queue_push_packet((uint8_t)'z', packet, 16U, 0U) == E_PAR);
    CHECK(queue_empty());
    pass("push_packet", before);
}

/* ---- capacity and non-blocking ------------------------------------------ */

static void test_capacity_of_1024_bytes (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS + 4U) == CAP_PHYSICS);
    CHECK(mbf_fake_free(g_mbf_telem_id) == 0);
    setup();
    CHECK(fill_with((uint8_t)'s', CAP_STATUS + 4U) == CAP_STATUS);
    setup();
    CHECK(fill_with((uint8_t)'e', CAP_EVENT + 4U) == CAP_EVENT);
    setup();
    CHECK(fill_with((uint8_t)'h', CAP_HEARTBEAT + 4U) == CAP_HEARTBEAT);
    CHECK(mbf_fake_free(g_mbf_telem_id) == 0);
    setup();
    CHECK(fill_with((uint8_t)'f', 20U) == (BUFFER_BYTES / COST_PROFILE));
    CHECK((CAP_STATUS * COST_STATUS) <= BUFFER_BYTES);
    CHECK(((CAP_STATUS + 1U) * COST_STATUS) > BUFFER_BYTES);
    pass("capacity_of_1024_bytes", before);
}

static void test_producers_never_block (void)
{
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    CHECK(push_t((uint8_t)'p', 1U, 0U) == E_TMOUT);
    CHECK(push_t((uint8_t)'p', 0U, 0U) == E_OK);
    CHECK(g_mbf_fake.send_calls > CAP_PHYSICS);
    CHECK(g_mbf_fake.last_snd_tmout == TMO_POL);
    CHECK(g_mbf_fake.last_rcv_tmout == TMO_POL); /* eviction read */
    CHECK(g_mbf_fake.would_block == 0U);

    /* Also when the eviction ends in a rejection: an event, then statuses
       leave 20 free bytes, and a status needs 24. */
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
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    drops = telemetry_queue_drop_count();

    CHECK(push_t((uint8_t)'e', 1U, 99U) == E_TMOUT);
    CHECK(push_t((uint8_t)'e', 2U, 99U) == E_TMOUT);
    CHECK(push_t((uint8_t)'e', 255U, 99U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops); /* a rejection is no drop */
    CHECK(g_mbf_fake.recv_calls == 0U);           /* and nothing is evicted */
    CHECK(g_mbf_fake.ref_calls == 0U);
    CHECK(mbf_fake_free(g_mbf_telem_id) == 0);

    for (unsigned i = 0U; i < CAP_PHYSICS; i++)
    {
        CHECK(pop_t((uint8_t)'p', (uint8_t)i));
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
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    drops = telemetry_queue_drop_count();

    CHECK(push_t((uint8_t)'p', 0U, 100U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U);

    /* Message 0 is gone; 1..31 remain in order; the new one is last. */
    for (unsigned i = 1U; i < CAP_PHYSICS; i++)
    {
        CHECK(pop_t((uint8_t)'p', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'p', 100U));
    CHECK(queue_empty());

    /* Every further eviction adds exactly one. */
    setup();
    drops = telemetry_queue_drop_count();
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    for (uint8_t n = 0U; n < 5U; n++)
    {
        CHECK(push_t((uint8_t)'p', 0U, (uint8_t)(50U + n)) == E_OK);
        CHECK(telemetry_queue_drop_count() == drops + n + 1U);
    }
    CHECK(pop_t((uint8_t)'p', 5U)); /* 0..4 evicted */
    pass("qos0_evicts_oldest_and_counts", before);
}

static void test_no_drop_while_space_remains (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    drops = telemetry_queue_drop_count();
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS - 1U) == CAP_PHYSICS - 1U);
    CHECK(push_t((uint8_t)'p', 0U, 7U) == E_OK);
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
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    drops = telemetry_queue_drop_count();

    /* Evicting one 32-byte message frees room for a 16-byte one. */
    CHECK(push_t((uint8_t)'h', 0U, 9U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    CHECK(mbf_fake_free(g_mbf_telem_id) ==
          (int32_t)(COST_PHYSICS - COST_HEARTBEAT));
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

/* Fill: status seed 0, an event, then statuses seed 2..EV_FILL_LAST. That is
   24 + 20 + 40 x 24 = 1004 bytes, 20 free. */
static void fill_status_event_statuses (uint8_t event_seed)
{
    CHECK(push_t((uint8_t)'s', 1U, 0U) == E_OK);
    CHECK(push_t((uint8_t)'e', 1U, event_seed) == E_OK);
    for (unsigned i = 2U; i <= EV_FILL_LAST; i++)
    {
        CHECK(push_t((uint8_t)'s', 1U, (uint8_t)i) == E_OK);
    }
    CHECK(mbf_fake_free(g_mbf_telem_id) == 20);
}

/* An event that becomes the oldest during eviction stops it. Frames evicted
   before that stay lost and are counted; the new frame is rejected and
   counted too. */
static void test_event_at_head_stops_eviction (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    fill_status_event_statuses(0xE1U);
    drops = telemetry_queue_drop_count();

    /* A profile frame (88 bytes) needs more than the 20 free plus one status.
       One status is evicted, then the event is at the head. */
    CHECK(push_t((uint8_t)'f', 0U, 0x70U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 2U); /* evicted + new */
    CHECK(g_mbf_fake.recv_calls == 1U);

    /* A 16-byte heartbeat fits in the 44 bytes that are free now. */
    CHECK(push_t((uint8_t)'h', 0U, 0x71U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 2U);

    CHECK(pop_t((uint8_t)'e', 0xE1U));
    for (unsigned i = 2U; i <= EV_FILL_LAST; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'h', 0x71U));
    CHECK(queue_empty());
    pass("event_at_head_stops_eviction", before);
}

/* Realistic sizes: status is 17 B as a frame (24 B in the buffer), profile is
   83 B (88 B). A full queue of status frames takes a profile frame by
   evicting three status frames; all three are counted. */
static void test_qos0_evicts_several_to_fit (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    CHECK(fill_with((uint8_t)'s', CAP_STATUS) == CAP_STATUS);
    drops = telemetry_queue_drop_count();

    CHECK(push_t((uint8_t)'f', 0U, 0x77U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 3U);
    CHECK(g_mbf_fake.recv_calls == 3U);
    CHECK(mbf_fake_free(g_mbf_telem_id) == 0);
    for (unsigned i = 3U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'f', 0x77U));
    CHECK(queue_empty());

    /* Heartbeats (16 B each) evicted for a profile frame (88 B): six go. */
    setup();
    CHECK(fill_with((uint8_t)'h', CAP_HEARTBEAT) == CAP_HEARTBEAT);
    drops = telemetry_queue_drop_count();
    CHECK(push_t((uint8_t)'f', 0U, 0x78U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 6U);
    pass("qos0_evicts_several_to_fit", before);
}

static void test_kernel_errors_pass_through (void)
{
    unsigned drops;
    unsigned before = g_fail;

    setup();
    drops = telemetry_queue_drop_count();

    /* A send error other than E_TMOUT is returned and nothing is evicted. */
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    g_mbf_fake.send_error = E_NOMEM;
    CHECK(push_t((uint8_t)'p', 0U, 0U) == E_NOMEM);
    CHECK(g_mbf_fake.recv_calls == 0U);
    CHECK(telemetry_queue_drop_count() == drops);

    /* If the eviction read fails, the original E_TMOUT is returned and the
       rejected frame is counted. */
    g_mbf_fake.recv_error = E_NOMEM;
    CHECK(push_t((uint8_t)'p', 0U, 0U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 1U);

    /* Same when the size peek fails. */
    g_mbf_fake.ref_error = E_NOMEM;
    CHECK(push_t((uint8_t)'p', 0U, 0U) == E_TMOUT);
    CHECK(telemetry_queue_drop_count() == drops + 2U);
    for (unsigned i = 0U; i < CAP_PHYSICS; i++)
    {
        CHECK(pop_t((uint8_t)'p', (uint8_t)i));
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
    CHECK(push_t((uint8_t)'p', 0U, 0x60U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 1U); /* seed 1 */
    for (unsigned i = 2U; i < CAP_STATUS; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'p', 0x60U));
    CHECK(queue_empty());

    /* The next message is an event: the producer's receive returns it. It must
       not be lost; it is sent back (after the newer frames). The space the
       consumer and the receive freed now fits the new status, so nothing
       else is evicted and nothing is dropped. */
    setup();
    fill_status_event_statuses(0xE2U);
    drops = telemetry_queue_drop_count();
    g_mbf_fake.recv_hook = consumer_pops_one; /* pops seed 0 */
    CHECK(push_t((uint8_t)'s', 0U, 0x61U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops);
    for (unsigned i = 2U; i <= EV_FILL_LAST; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'e', 0xE2U)); /* the event survived */
    CHECK(pop_t((uint8_t)'s', 0x61U));
    CHECK(queue_empty());
    CHECK(g_mbf_fake.would_block == 0U);

    /* Same race with a profile frame (88 B): after the event is sent back the
       next oldest is a status, which is evicted (counted), and again the next.
       The loop does not stop at the event it sent back. */
    setup();
    fill_status_event_statuses(0xE2U);
    drops = telemetry_queue_drop_count();
    g_mbf_fake.recv_hook = consumer_pops_one; /* pops seed 0 */
    CHECK(push_t((uint8_t)'f', 0U, 0x63U) == E_OK);
    CHECK(telemetry_queue_drop_count() == drops + 2U); /* seeds 2 and 3 */
    for (unsigned i = 4U; i <= EV_FILL_LAST; i++)
    {
        CHECK(pop_t((uint8_t)'s', (uint8_t)i));
    }
    CHECK(pop_t((uint8_t)'e', 0xE2U));
    CHECK(pop_t((uint8_t)'f', 0x63U));
    CHECK(queue_empty());

    /* The consumer's pop frees a slot first: the producer still evicts one
       more (it only learns the size at the next peek) and succeeds. */
    setup();
    CHECK(fill_with((uint8_t)'p', CAP_PHYSICS) == CAP_PHYSICS);
    g_mbf_fake.recv_hook = consumer_pops_one;
    drops = telemetry_queue_drop_count();
    CHECK(push_t((uint8_t)'p', 0U, 0x62U) == E_OK);
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
    CHECK(telemetry_queue_push(0, FRAME_STATUS, 0U) == E_PAR);
    CHECK(g_mbf_fake.mtx_lock_calls == locks);

    /* pop does not use it (it may block). */
    CHECK(telemetry_queue_pop(msg, TMO_POL) == (INT)FRAME_STATUS);
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
    CHECK(push_t((uint8_t)'f', 0U, 0x71U) == E_OK); /* evicts three */
    CHECK(g_mbf_fake.mtx_locked == 0);
    CHECK(g_mbf_fake.rcv_locked >= 3U);
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
    static const uint8_t topics[] = {'h', 'e', 's', 'p', 's', 's', 'e', 'p',
                                     'm', 'o', 'd', 'f'};
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
    test_stored_lengths_and_content();
    test_pop_output_is_the_frame();
    test_foreign_length();
    test_pop_arguments();
    test_fifo_order();
    test_push_packet();
    test_capacity_of_1024_bytes();
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
