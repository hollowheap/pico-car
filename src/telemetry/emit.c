/*
 *	emit.c
 *	Queue one telemetry event. See emit.h.
 */

#include <tk/smp.h>

#include "telemetry/emit.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"

#define EVENT_QOS 1u

/* The next event id. The kernel atomic keeps ids unique when several tasks
   emit at once. */
static UW g_next_event_id;

ER
telemetry_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
                      uint32_t odometer_mm)
{
    uint8_t           packet[TELEMETRY_EVENT_WIRE_SIZE];
    telemetry_event_t event;

    event.odometer_mm = odometer_mm;
    event.event_id    = (uint16_t) atomic_inc(&g_next_event_id);
    event.p1          = p1;
    event.p2          = p2;
    event.event_type  = type;
    event.source      = source;
    (void) telemetry_pack_event(packet, &event);
    return telemetry_queue_push_packet((uint8_t) TELEMETRY_TOPIC_EVENT, packet,
                                       TELEMETRY_EVENT_WIRE_SIZE, EVENT_QOS);
}

uint16_t
telemetry_event_count (void)
{
    return (uint16_t) atomic_add(&g_next_event_id, 0u);
}
