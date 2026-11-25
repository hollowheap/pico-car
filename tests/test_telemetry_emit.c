/** @file test_telemetry_emit.c
 * @brief Host test for telemetry_emit_event (S46): ids, fields, QoS 1.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tk/mbf_fake.h>
#include "telemetry/emit.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"

static unsigned g_fail;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            g_fail++;                                                        \
            (void)printf("FAIL line %d: %s\n", __LINE__, #cond);             \
        }                                                                    \
    } while (0)

static uint16_t be16 (const uint8_t * p)
{
    return (uint16_t)(((unsigned)p[0] << 8) | p[1]);
}

static uint32_t be32 (const uint8_t * p)
{
    return ((uint32_t)be16(p) << 16) | be16(&p[2]);
}

static void setup (void)
{
    mbf_fake_reset();
    telemetry_queue_test_reset();
    telemetry_emit_test_reset();
    CHECK(telemetry_queue_create() == E_OK);
}

static void test_event_fields_and_ids (void)
{
    uint8_t  frame[TELEMETRY_FRAME_MAX];
    uint16_t first = 0U;

    setup();
    first = telemetry_event_count();
    CHECK(telemetry_emit_event(TELEMETRY_EVENT_SOURCE_BARCODE,
                               TELEMETRY_EVT_BARCODE, (int16_t)'L', 2U,
                               123456U) == E_OK);
    CHECK(telemetry_emit_event(TELEMETRY_EVENT_SOURCE_PHYSICS,
                               TELEMETRY_EVT_HUMP_END, -5, 41U, 7U) == E_OK);

    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)(TELEMETRY_EVENT_WIRE_SIZE + 1U));
    CHECK(frame[0] == (uint8_t)TELEMETRY_TOPIC_EVENT);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID]) == first);
    CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_BARCODE);
    CHECK(frame[1U + TELEMETRY_EVENT_OFF_SOURCE] == TELEMETRY_EVENT_SOURCE_BARCODE);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P1]) == (uint16_t)'L');
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == 2U);
    CHECK(be32(&frame[1U + TELEMETRY_EVENT_OFF_ODOMETER_MM]) == 123456U);

    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)(TELEMETRY_EVENT_WIRE_SIZE + 1U));
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID]) == (uint16_t)(first + 1U));
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P1]) == (uint16_t)-5);
    CHECK(telemetry_event_count() == (uint16_t)(first + 2U));
}

/* A full queue of status frames never loses an event: status is evicted. A
   full queue of events refuses the next one. */
static void test_event_is_qos1 (void)
{
    uint8_t  status[TELEMETRY_STATUS_PIVOT_WIRE_SIZE] = {0};
    uint8_t  frame[TELEMETRY_FRAME_MAX];
    unsigned stored = 0U;
    unsigned events = 0U;

    setup();
    while (telemetry_queue_push_packet((uint8_t)TELEMETRY_TOPIC_STATUS, status,
                                       TELEMETRY_STATUS_PIVOT_WIRE_SIZE, 0U) == E_OK)
    {
        stored++;
        if (stored > 200U)
        {
            break;
        }
    }
    CHECK(stored > 0U);
    /* Status frames are in the way: eviction is for qos 0 only, so a qos 1
       event on a full queue is refused. */
    CHECK(telemetry_emit_event(TELEMETRY_EVENT_SOURCE_LINE,
                               TELEMETRY_EVT_LINE_LOST, 0, 0U, 0U) == E_TMOUT);
    CHECK(telemetry_queue_pop(frame, TMO_POL) > 0);
    CHECK(telemetry_emit_event(TELEMETRY_EVENT_SOURCE_LINE,
                               TELEMETRY_EVT_LINE_LOST, 0, 0U, 0U) == E_OK);
    events++;
    /* Now a status push evicts a status, never the event. */
    for (unsigned i = 0U; i < stored + 5U; i++)
    {
        (void)telemetry_queue_push_packet((uint8_t)TELEMETRY_TOPIC_STATUS, status,
                                          TELEMETRY_STATUS_PIVOT_WIRE_SIZE, 0U);
    }
    {
        unsigned seen = 0U;

        while (telemetry_queue_pop(frame, TMO_POL) > 0)
        {
            if (frame[0] == (uint8_t)TELEMETRY_TOPIC_EVENT)
            {
                seen++;
            }
        }
        CHECK(seen == events);
    }
}

static void test_try_event_on_contended_queue (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    UW      drops = 0U;

    setup();
    CHECK(telemetry_emit_event(TELEMETRY_EVENT_SOURCE_PHYSICS,
                               TELEMETRY_EVT_OVERTURN, 0, 0U, 40U) == E_OK);
    drops = telemetry_queue_drop_count();

    /* The kernel returns E_TMOUT when another task owns a polled mutex. */
    g_mbf_fake.mtx_lock_error = E_TMOUT;
    CHECK(telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_OBSTACLE,
                                   TELEMETRY_EVT_SCAN_DONE, -120, 8U, 41U) == E_TMOUT);
    CHECK(g_mbf_fake.mtx_lock_calls == 2U);
    CHECK(g_mbf_fake.mtx_wait_forever == 1U);
    CHECK(g_mbf_fake.mtx_unlock_calls == 1U);
    CHECK(g_mbf_fake.send_calls == 1U);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
    CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_OVERTURN);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == E_TMOUT);
}

static void test_try_event_preserves_full_queue (void)
{
    /* Message buffer cost is 20 bytes including its header and padding. */
    const uint16_t capacity = (uint16_t)(TELEMETRY_QUEUE_BYTES / 20U);
    uint8_t        frame[TELEMETRY_FRAME_MAX];
    uint16_t       first = 0U;
    UW             drops = 0U;

    setup();
    first = telemetry_event_count();
    for (uint16_t i = 0U; i < capacity; i++)
    {
        CHECK(telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_OBSTACLE,
                                       TELEMETRY_EVT_SCAN_DONE, -120, i, 40U) == E_OK);
    }
    drops = telemetry_queue_drop_count();
    CHECK(telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_OBSTACLE,
                                   TELEMETRY_EVT_SCAN_DONE, -120, 99U, 41U) == E_TMOUT);
    CHECK(g_mbf_fake.mtx_wait_forever == 0U);
    CHECK(g_mbf_fake.last_snd_tmout == TMO_POL);
    CHECK(g_mbf_fake.ref_calls == 0U);
    CHECK(telemetry_queue_drop_count() == drops + 1U);
    for (uint16_t i = 0U; i < capacity; i++)
    {
        CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
        CHECK(frame[0] == TELEMETRY_TOPIC_EVENT);
        CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_SCAN_DONE);
        CHECK(frame[1U + TELEMETRY_EVENT_OFF_SOURCE] == TELEMETRY_EVENT_SOURCE_OBSTACLE);
        CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID]) == (uint16_t)(first + i));
        CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P1]) == (uint16_t)-120);
        CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == i);
        CHECK(be32(&frame[1U + TELEMETRY_EVENT_OFF_ODOMETER_MM]) == 40U);
    }
    CHECK(telemetry_queue_pop(frame, TMO_POL) == E_TMOUT);
}

static void test_cmd_ack_handoff_fifo_and_wrap (void)
{
    uint8_t  frame[TELEMETRY_FRAME_MAX];
    uint16_t first = 0U;

    setup();
    CHECK(telemetry_cmd_ack_overflow_count() == 0U);
    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_LINE_LOST, 0, 1U, 0U) == E_PAR);
    CHECK(telemetry_cmd_ack_overflow_count() == 0U);

    first = telemetry_event_count();
    for (uint16_t i = 0U; i < (uint16_t)(TELEMETRY_ACK_RING_CAPACITY + 5U); i++)
    {
        uint8_t const type = ((i % 2U) == 0U)
                                 ? (uint8_t)TELEMETRY_EVT_CMD_ACCEPT
                                 : (uint8_t)TELEMETRY_EVT_CMD_REJECT;
        int16_t const p1   = (int16_t)((((i % 2U) == 0U) ? 0x0100 : -0x2000) + (int16_t)i);

        CHECK(telemetry_post_cmd_ack(type, p1, (uint16_t)(100U + i), 500U + i) == E_OK);
        if ((i == 5U) || (i == (uint16_t)(TELEMETRY_ACK_RING_CAPACITY + 4U)))
        {
            uint32_t const want = (i == 5U) ? 6U : (uint32_t)(TELEMETRY_ACK_RING_CAPACITY - 1U);
            CHECK(telemetry_drain_cmd_acks() == want);
            uint16_t const base = (i == 5U) ? 0U : 6U;
            for (uint16_t k = 0U; k < want; k++)
            {
                uint16_t const idx = (uint16_t)(base + k);
                uint8_t const  exp_type = ((idx % 2U) == 0U)
                                              ? (uint8_t)TELEMETRY_EVT_CMD_ACCEPT
                                              : (uint8_t)TELEMETRY_EVT_CMD_REJECT;
                int16_t const  exp_p1   = (int16_t)((((idx % 2U) == 0U) ? 0x0100 : -0x2000) + (int16_t)idx);

                CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
                CHECK(frame[0] == (uint8_t)TELEMETRY_TOPIC_EVENT);
                CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID]) == (uint16_t)(first + idx));
                CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == exp_type);
                CHECK(frame[1U + TELEMETRY_EVENT_OFF_SOURCE] == TELEMETRY_EVENT_SOURCE_SUPERVISOR);
                CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P1]) == (uint16_t)exp_p1);
                CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == (uint16_t)(100U + idx));
                CHECK(be32(&frame[1U + TELEMETRY_EVENT_OFF_ODOMETER_MM]) == (uint32_t)(500U + idx));
            }
            CHECK(telemetry_queue_pop(frame, TMO_POL) == E_TMOUT);
        }
    }
    CHECK(telemetry_cmd_ack_overflow_count() == 0U);
}

static void test_cmd_ack_drain_under_contention_and_full_queue (void)
{
    uint8_t  frame[TELEMETRY_FRAME_MAX];
    uint8_t  status[TELEMETRY_STATUS_PIVOT_WIRE_SIZE] = {0};
    UW       drops = 0U;
    uint16_t first = 0U;

    /* Post before queue exists: handoff never touches queue or mutex. */
    mbf_fake_reset();
    telemetry_queue_test_reset();
    telemetry_emit_test_reset();
    first = telemetry_event_count();
    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_ACCEPT, 0x0106, 9U, 100U) == E_OK);
    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_REJECT, 0x2101, 10U, 101U) == E_OK);
    CHECK(g_mbf_fake.mtx_lock_calls == 0U);
    CHECK(g_mbf_fake.send_calls == 0U);
    CHECK(g_mbf_fake.recv_calls == 0U);
    CHECK(telemetry_drain_cmd_acks() == 0U);
    CHECK(telemetry_queue_create() == E_OK);
    drops = telemetry_queue_drop_count();

    /* Contended producer mutex: drain defers without dropping or burning IDs. */
    g_mbf_fake.mtx_lock_error = E_TMOUT;
    CHECK(telemetry_drain_cmd_acks() == 0U);
    CHECK(g_mbf_fake.mtx_wait_forever == 0U);
    CHECK(telemetry_queue_drop_count() == drops);
    CHECK(telemetry_event_count() == (uint16_t)(first + 1U));

    /* Full queue of QoS 0 status frames: drain defers without evicting or dropping. */
    while (telemetry_queue_push_packet((uint8_t)TELEMETRY_TOPIC_STATUS, status,
                                       TELEMETRY_STATUS_PIVOT_WIRE_SIZE, 0U) == E_OK)
    {
        if (g_mbf_fake.send_calls > 100U)
        {
            break;
        }
    }
    drops = telemetry_queue_drop_count();
    unsigned const recv_before = g_mbf_fake.recv_calls;
    CHECK(telemetry_drain_cmd_acks() == 0U);
    CHECK(telemetry_drain_cmd_acks() == 0U);
    CHECK(g_mbf_fake.recv_calls == recv_before);
    CHECK(telemetry_queue_drop_count() == drops);
    CHECK(telemetry_event_count() == (uint16_t)(first + 1U));

    /* Consumer frees queue space: all buffered ACKs drain in FIFO order with contiguous IDs. */
    while (telemetry_queue_pop(frame, TMO_POL) > 0)
    {
    }
    /* Partial drain: fill queue with events, free space for 1 event only, and
       verify slot 0 drains with held id `first` while slot 1 holds a single new
       id across a subsequent full-queue retry. */
    while (telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_LINE,
                                    TELEMETRY_EVT_LINE_LOST, 0, 0U, 0U) == E_OK)
    {
    }
    uint16_t const second_id = telemetry_event_count();
    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
    CHECK(telemetry_drain_cmd_acks() == 1U);
    CHECK(telemetry_event_count() == (uint16_t)(second_id + 1U));
    CHECK(telemetry_drain_cmd_acks() == 0U);
    CHECK(telemetry_event_count() == (uint16_t)(second_id + 1U));
    {
        unsigned seen_accept = 0U;

        while (telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE)
        {
            if (frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_CMD_ACCEPT)
            {
                CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID]) == first);
                CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P1]) == 0x0106U);
                CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == 9U);
                CHECK(be32(&frame[1U + TELEMETRY_EVENT_OFF_ODOMETER_MM]) == 100U);
                seen_accept++;
            }
        }
        CHECK(seen_accept == 1U);
    }

    CHECK(telemetry_drain_cmd_acks() == 1U);
    CHECK(telemetry_event_count() == (uint16_t)(second_id + 1U));
    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_EVENT_ID]) == second_id);
    CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_CMD_REJECT);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P1]) == 0x2101U);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == 10U);
    CHECK(be32(&frame[1U + TELEMETRY_EVENT_OFF_ODOMETER_MM]) == 101U);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == E_TMOUT);
}

static void test_cmd_ack_overflow_visibility (void)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];
    UW      drops = 0U;

    setup();
    drops = telemetry_queue_drop_count();
    for (uint16_t i = 0U; i < (uint16_t)TELEMETRY_ACK_RING_CAPACITY; i++)
    {
        CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_ACCEPT, 0x0100,
                                     (uint16_t)(200U + i), 1000U + i) == E_OK);
    }
    CHECK(telemetry_cmd_ack_overflow_count() == 0U);
    CHECK(telemetry_queue_drop_count() == drops);
    CHECK(g_mbf_fake.mtx_lock_calls == 0U);

    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_ACCEPT, 0x0106, 900U, 2000U) == E_TMOUT);
    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_REJECT, 0x2101, 901U, 2001U) == E_TMOUT);
    CHECK(telemetry_cmd_ack_overflow_count() == 2U);
    CHECK(telemetry_queue_drop_count() == drops + 2U);
    CHECK(g_mbf_fake.mtx_lock_calls == 0U);

    CHECK(telemetry_drain_cmd_acks() == TELEMETRY_ACK_RING_CAPACITY);
    for (uint16_t i = 0U; i < (uint16_t)TELEMETRY_ACK_RING_CAPACITY; i++)
    {
        CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
        CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_CMD_ACCEPT);
        CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == (uint16_t)(200U + i));
        CHECK(be32(&frame[1U + TELEMETRY_EVENT_OFF_ODOMETER_MM]) == (uint32_t)(1000U + i));
    }
    CHECK(telemetry_queue_pop(frame, TMO_POL) == E_TMOUT);
    CHECK(telemetry_cmd_ack_overflow_count() == 2U);

    /* Post-overflow recovery: ring accepts wrapped ACCEPT and REJECT cleanly. */
    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_REJECT, 0x2101, 910U, 3000U) == E_OK);
    CHECK(telemetry_post_cmd_ack(TELEMETRY_EVT_CMD_ACCEPT, 0x0106, 911U, 3001U) == E_OK);
    CHECK(telemetry_drain_cmd_acks() == 2U);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
    CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_CMD_REJECT);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == 910U);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)TELEMETRY_EVENT_QUEUED_SIZE);
    CHECK(frame[1U + TELEMETRY_EVENT_OFF_EVENT_TYPE] == TELEMETRY_EVT_CMD_ACCEPT);
    CHECK(be16(&frame[1U + TELEMETRY_EVENT_OFF_P2]) == 911U);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == E_TMOUT);
    CHECK(telemetry_cmd_ack_overflow_count() == 2U);
}

int main (void)
{
    setup();
    telemetry_barcode_sample_t sample = {
        .edge_us = 1234567U, .interval_us = 20000U, .odometer_mm = 522U,
        .width_um = 3000U, .seq = 255U, .level_high = 1U, .width_valid = 1U
    };
    uint8_t frame[TELEMETRY_FRAME_MAX];
    CHECK(telemetry_try_emit_barcode_sample(&sample) == E_OK);
    CHECK(telemetry_queue_pop(frame, TMO_POL) == (INT)(1U + TELEMETRY_BARCODE_SAMPLE_WIRE_SIZE));
    CHECK(frame[0] == 'b' && be32(&frame[1]) == 1234567U && be32(&frame[5]) == 20000U);
    CHECK(be32(&frame[9]) == 522U && be16(&frame[13]) == 3000U && frame[15] == 255U);
    CHECK(frame[16] == 1U && frame[17] == 1U && frame[18] == 0U);
    unsigned accepted = 0U;
    while (accepted < 100U && telemetry_try_emit_barcode_sample(&sample) == E_OK) ++accepted;
    CHECK(accepted > 0U && accepted < 100U);
    CHECK(telemetry_emit_event(TELEMETRY_EVENT_SOURCE_BARCODE, TELEMETRY_EVT_BARCODE,
                               'Z', 0, 522U) == E_OK);
    test_event_fields_and_ids();
    test_event_is_qos1();
    test_try_event_on_contended_queue();
    test_try_event_preserves_full_queue();
    test_cmd_ack_handoff_fifo_and_wrap();
    test_cmd_ack_drain_under_contention_and_full_queue();
    test_cmd_ack_overflow_visibility();
    if (g_fail != 0U)
    {
        (void)printf("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf("test_telemetry_emit: all checks passed\n");
    return 0;
}
