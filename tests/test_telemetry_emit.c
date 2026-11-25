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
    if (g_fail != 0U)
    {
        (void)printf("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf("test_telemetry_emit: all checks passed\n");
    return 0;
}
