/*
 *	emit.c
 *	Queue one telemetry event. See emit.h.
 */

#include <stdbool.h>
#include <tk/smp.h>

#include "platform/static_assert.h"
#include "telemetry/emit.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"

#define EVENT_QOS                   (1U)
#define TELEMETRY_ACK_RING_MASK     (TELEMETRY_ACK_RING_CAPACITY - 1U)
#define TELEMETRY_ACK_MASK_BITS     (16U)
#define TELEMETRY_ACK_ENTRY_SIZE    (8U)
#define TELEMETRY_ACK_RING_META_SZ  (12U)
#define TELEMETRY_ACK_FLAG_DRAINING (0x01U)
#define TELEMETRY_ACK_FLAG_ID_HELD  (0x02U)

STATIC_ASSERT((TELEMETRY_ACK_RING_CAPACITY & TELEMETRY_ACK_RING_MASK) == 0U,
              "ack ring capacity must be a power of two");
STATIC_ASSERT(TELEMETRY_ACK_RING_CAPACITY <= TELEMETRY_ACK_MASK_BITS,
              "accept_mask must cover ack ring capacity");

typedef struct
{
    uint32_t odometer_mm;
    int16_t  p1;
    uint16_t p2;
} telemetry_ack_entry_t;

STATIC_ASSERT(sizeof(telemetry_ack_entry_t) == TELEMETRY_ACK_ENTRY_SIZE,
              "telemetry_ack_entry_t size");

typedef struct
{
    telemetry_ack_entry_t item[TELEMETRY_ACK_RING_CAPACITY];
    uint32_t              overflow;
    uint16_t              accept_mask;
    uint16_t              held_event_id;
    uint8_t               head;
    uint8_t               tail;
    uint8_t               count;
    uint8_t               flags;
} telemetry_ack_ring_t;

STATIC_ASSERT(sizeof(telemetry_ack_ring_t) ==
                  ((TELEMETRY_ACK_RING_CAPACITY *
                    sizeof(telemetry_ack_entry_t)) +
                   TELEMETRY_ACK_RING_META_SZ),
              "telemetry_ack_ring_t size");

/* The next event id. The kernel atomic keeps ids unique when several tasks
   emit at once. T_SPLOCK is unlocked at zero, so static zero-init is valid
   before any task runs. */
static UW                   g_next_event_id;
static T_SPLOCK             g_ack_lock;
static telemetry_ack_ring_t g_ack_ring;

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

ER
telemetry_post_cmd_ack (uint8_t type, int16_t p1, uint16_t p2,
                        uint32_t odometer_mm)
{
    UINT saved    = 0U;
    bool b_stored = false;

    if (((uint8_t) TELEMETRY_EVT_CMD_ACCEPT != type) &&
        ((uint8_t) TELEMETRY_EVT_CMD_REJECT != type))
    {
        return E_PAR;
    }

    (void) ISpinLock(&g_ack_lock, &saved);
    if (g_ack_ring.count < TELEMETRY_ACK_RING_CAPACITY)
    {
        uint8_t const slot = g_ack_ring.tail;
        uint16_t const bit = (uint16_t) (1U << slot);

        g_ack_ring.item[slot].odometer_mm = odometer_mm;
        g_ack_ring.item[slot].p1          = p1;
        g_ack_ring.item[slot].p2          = p2;
        if ((uint8_t) TELEMETRY_EVT_CMD_ACCEPT == type)
        {
            g_ack_ring.accept_mask |= bit;
        }
        else
        {
            g_ack_ring.accept_mask &= (uint16_t) (~(uint32_t) bit);
        }
        g_ack_ring.tail = (uint8_t) (((uint32_t) slot + 1U) &
                                     TELEMETRY_ACK_RING_MASK);
        g_ack_ring.count++;
        b_stored = true;
    }
    else
    {
        g_ack_ring.overflow++;
    }
    (void) ISpinUnlock(&g_ack_lock, saved);

    if (!b_stored)
    {
        telemetry_queue_note_drop();
        return E_TMOUT;
    }
    return E_OK;
}

uint32_t
telemetry_drain_cmd_acks (void)
{
    uint32_t drained = 0U;

    for (uint32_t step = 0U; step < TELEMETRY_ACK_RING_CAPACITY; step++)
    {
        telemetry_ack_entry_t entry     = {0U, 0, 0U};
        uint8_t               frame[TELEMETRY_EVENT_QUEUED_SIZE];
        telemetry_event_t     event     = {0U, 0U, 0, 0U, 0U, 0U};
        UINT                  saved     = 0U;
        uint16_t              event_id  = 0U;
        uint8_t               type      = (uint8_t) TELEMETRY_EVT_CMD_REJECT;
        bool                  b_have    = false;
        bool                  b_need_id = false;

        (void) ISpinLock(&g_ack_lock, &saved);
        if ((0U == (g_ack_ring.flags & TELEMETRY_ACK_FLAG_DRAINING)) &&
            (0U != g_ack_ring.count))
        {
            uint8_t const  head = g_ack_ring.head;
            uint16_t const bit  = (uint16_t) (1U << head);

            g_ack_ring.flags |= TELEMETRY_ACK_FLAG_DRAINING;
            entry = g_ack_ring.item[head];
            if (0U != (g_ack_ring.accept_mask & bit))
            {
                type = (uint8_t) TELEMETRY_EVT_CMD_ACCEPT;
            }
            if (0U != (g_ack_ring.flags & TELEMETRY_ACK_FLAG_ID_HELD))
            {
                event_id = g_ack_ring.held_event_id;
            }
            else
            {
                b_need_id = true;
            }
            b_have = true;
        }
        (void) ISpinUnlock(&g_ack_lock, saved);

        if (!b_have)
        {
            break;
        }
        if (b_need_id)
        {
            event_id = (uint16_t) atomic_inc(&g_next_event_id);
        }

        event.odometer_mm = entry.odometer_mm;
        event.event_id    = event_id;
        event.p1          = entry.p1;
        event.p2          = entry.p2;
        event.event_type  = type;
        event.source      = (uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR;
        frame[0]          = (uint8_t) TELEMETRY_TOPIC_EVENT;
        (void) telemetry_pack_event(&frame[1], &event);

        ER const result = telemetry_queue_try_push_no_drop(
            frame, TELEMETRY_EVENT_QUEUED_SIZE);

        (void) ISpinLock(&g_ack_lock, &saved);
        g_ack_ring.flags &= (uint8_t) (~(uint32_t) TELEMETRY_ACK_FLAG_DRAINING);
        if (E_OK == result)
        {
            g_ack_ring.head = (uint8_t) (((uint32_t) g_ack_ring.head + 1U) &
                                         TELEMETRY_ACK_RING_MASK);
            g_ack_ring.count--;
            g_ack_ring.flags &= (uint8_t) (~(uint32_t) TELEMETRY_ACK_FLAG_ID_HELD);
            drained++;
        }
        else
        {
            g_ack_ring.held_event_id = event_id;
            g_ack_ring.flags |= TELEMETRY_ACK_FLAG_ID_HELD;
        }
        (void) ISpinUnlock(&g_ack_lock, saved);

        if (E_OK != result)
        {
            break;
        }
    }

    return drained;
}

UW
telemetry_cmd_ack_overflow_count (void)
{
    UINT     saved    = 0U;
    uint32_t overflow = 0U;

    (void) ISpinLock(&g_ack_lock, &saved);
    overflow = g_ack_ring.overflow;
    (void) ISpinUnlock(&g_ack_lock, saved);
    return (UW) overflow;
}

uint16_t
telemetry_event_count (void)
{
    return (uint16_t) atomic_add(&g_next_event_id, 0u);
}

void
telemetry_emit_test_reset (void)
{
    UINT saved = 0U;

    (void) InitSpinLock(&g_ack_lock);
    (void) ISpinLock(&g_ack_lock, &saved);
    g_ack_ring = (telemetry_ack_ring_t) {0};
    (void) ISpinUnlock(&g_ack_lock, saved);
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
