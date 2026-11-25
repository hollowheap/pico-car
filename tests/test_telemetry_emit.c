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
    g_mbf_telem_id = 0;
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
    uint8_t  status[TELEMETRY_STATUS_WIRE_SIZE] = {0};
    uint8_t  frame[TELEMETRY_FRAME_MAX];
    unsigned stored = 0U;
    unsigned events = 0U;

    setup();
    while (telemetry_queue_push_packet((uint8_t)TELEMETRY_TOPIC_STATUS, status,
                                       TELEMETRY_STATUS_WIRE_SIZE, 0U) == E_OK)
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
                                          TELEMETRY_STATUS_WIRE_SIZE, 0U);
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

int main (void)
{
    test_event_fields_and_ids();
    test_event_is_qos1();
    if (g_fail != 0U)
    {
        (void)printf("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf("test_telemetry_emit: all checks passed\n");
    return 0;
}
