/*
 *	emit.c
 *	Queue one telemetry event. See emit.h.
 */

#include <stdbool.h>
#include <tk/smp.h>

#include "telemetry/emit.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"

#define EVENT_QOS 1u

/* The next event id. The kernel atomic keeps ids unique when several tasks
   emit at once. */
static UW g_next_event_id;

static ER
emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
            uint32_t odometer_mm, bool b_try)
{
    uint8_t           frame[TELEMETRY_EVENT_QUEUED_SIZE];
    telemetry_event_t event;

    event.odometer_mm = odometer_mm;
    event.event_id    = (uint16_t) atomic_inc(&g_next_event_id);
    event.p1          = p1;
    event.p2          = p2;
    event.event_type  = type;
    event.source      = source;
    frame[0] = (uint8_t) TELEMETRY_TOPIC_EVENT;
    (void) telemetry_pack_event(&frame[1], &event);
    return b_try ? telemetry_queue_try_push(frame, TELEMETRY_EVENT_QUEUED_SIZE)
                 : telemetry_queue_push(frame, TELEMETRY_EVENT_QUEUED_SIZE, EVENT_QOS);
}

ER
telemetry_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
                      uint32_t odometer_mm)
{
    return emit_event(source, type, p1, p2, odometer_mm, false);
}

ER
telemetry_try_emit_event (uint8_t source, uint8_t type, int16_t p1,
                          uint16_t p2, uint32_t odometer_mm)
{
    return emit_event(source, type, p1, p2, odometer_mm, true);
}

uint16_t
telemetry_event_count (void)
{
    return (uint16_t) atomic_add(&g_next_event_id, 0u);
}

ER telemetry_try_emit_barcode_sample (telemetry_barcode_sample_t const * p_sample)
{
    uint8_t frame[1U + TELEMETRY_BARCODE_SAMPLE_WIRE_SIZE];
    frame[0] = (uint8_t)TELEMETRY_TOPIC_BARCODE_SAMPLE;
    if (telemetry_pack_barcode_sample(&frame[1], p_sample) == 0U)
    {
        return E_PAR;
    }
    return telemetry_queue_try_push(frame, sizeof(frame));
}
