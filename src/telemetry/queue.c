#include <tk/smp.h>

#include "platform/static_assert.h"
#include "telemetry/packet.h"
#include "telemetry/queue.h"

#define QUEUE_BUFFER_ALIGN 4

/* Smallest message cost in the buffer: 4 B header + the 4 B heartbeat. It
   bounds the eviction loop by the number of messages the buffer can hold. */
#define QUEUE_MIN_COST      8U
#define QUEUE_EVICT_LIMIT   (TELEMETRY_QUEUE_BYTES / QUEUE_MIN_COST)

/* D87: the topic is inferred from the stored length, so the four lengths must
   be distinct and fit the pop buffer (topic byte + frame) and the kernel
   maxmsz. */
STATIC_ASSERT (TELEMETRY_HEARTBEAT_QUEUED_SIZE != TELEMETRY_EVENT_QUEUED_SIZE,
               "queued lengths distinct");
STATIC_ASSERT (TELEMETRY_HEARTBEAT_QUEUED_SIZE != TELEMETRY_STATUS_QUEUED_SIZE,
               "queued lengths distinct");
STATIC_ASSERT (TELEMETRY_HEARTBEAT_QUEUED_SIZE != TELEMETRY_SYSTEM_QUEUED_SIZE,
               "queued lengths distinct");
STATIC_ASSERT (TELEMETRY_EVENT_QUEUED_SIZE != TELEMETRY_STATUS_QUEUED_SIZE,
               "queued lengths distinct");
STATIC_ASSERT (TELEMETRY_EVENT_QUEUED_SIZE != TELEMETRY_SYSTEM_QUEUED_SIZE,
               "queued lengths distinct");
STATIC_ASSERT (TELEMETRY_STATUS_QUEUED_SIZE != TELEMETRY_SYSTEM_QUEUED_SIZE,
               "queued lengths distinct");
STATIC_ASSERT (TELEMETRY_HEARTBEAT_QUEUED_SIZE <= TELEMETRY_QUEUE_MAX_MSG,
               "heartbeat fits queue");
STATIC_ASSERT (TELEMETRY_EVENT_QUEUED_SIZE <= TELEMETRY_QUEUE_MAX_MSG,
               "event fits queue");
STATIC_ASSERT (TELEMETRY_STATUS_QUEUED_SIZE <= TELEMETRY_QUEUE_MAX_MSG,
               "status fits queue");
STATIC_ASSERT (TELEMETRY_SYSTEM_QUEUED_SIZE + 1U <= TELEMETRY_QUEUE_MAX_MSG,
               "system plus topic byte fits pop buffer");
STATIC_ASSERT (TELEMETRY_EVENT_QUEUED_SIZE + 1U == TELEMETRY_EVENT_WIRE_SIZE,
               "event drops exactly the reserved byte");
STATIC_ASSERT (TELEMETRY_HEARTBEAT_QUEUED_SIZE == TELEMETRY_HEARTBEAT_WIRE_SIZE,
               "heartbeat stored whole");
STATIC_ASSERT (TELEMETRY_STATUS_QUEUED_SIZE == TELEMETRY_STATUS_WIRE_SIZE,
               "status stored whole");
STATIC_ASSERT (TELEMETRY_SYSTEM_QUEUED_SIZE == TELEMETRY_SYSTEM_WIRE_SIZE,
               "system stored whole");

static uint8_t g_queue_buffer[TELEMETRY_QUEUE_BYTES]
    __attribute__((aligned(QUEUE_BUFFER_ALIGN)));
static UW g_drop_count;

/* Serialises producers only. See telemetry_queue_push. */
static ID g_push_mtx;

ID g_mbf_telem_id;

ER
telemetry_queue_create (void)
{
    T_CMTX cmtx = {0};
    T_CMBF cmbf = {0};
    ER     result = E_OK;

    if (g_mbf_telem_id > 0)
    {
        return E_OK;
    }

    /* TA_INHERIT alone: TA_INHERIT | TA_TPRI has the value of TA_CEILING. */
    cmtx.mtxatr = TA_INHERIT;
    result = tk_cre_mtx(&cmtx);
    if (result <= 0)
    {
        return result;
    }
    g_push_mtx = result;

    cmbf.mbfatr = TA_USERBUF | TA_TPRI;
    cmbf.bufsz  = TELEMETRY_QUEUE_BYTES;
    cmbf.maxmsz = TELEMETRY_QUEUE_MAX_MSG;
    cmbf.bufptr = g_queue_buffer;

    result = tk_cre_mbf(&cmbf);
    if (result > 0)
    {
        g_mbf_telem_id = result;
        return E_OK;
    }

    (void) tk_del_mtx(g_push_mtx);
    g_push_mtx = 0;
    return result;
}

/* Topic of a message stored with `queued_len` bytes, 0 if no topic has it. */
static uint8_t
topic_of_queued (INT queued_len)
{
    switch (queued_len)
    {
        case (INT) TELEMETRY_HEARTBEAT_QUEUED_SIZE:
            return (uint8_t) TELEMETRY_TOPIC_HEARTBEAT;
        case (INT) TELEMETRY_EVENT_QUEUED_SIZE:
            return (uint8_t) TELEMETRY_TOPIC_EVENT;
        case (INT) TELEMETRY_STATUS_QUEUED_SIZE:
            return (uint8_t) TELEMETRY_TOPIC_STATUS;
        case (INT) TELEMETRY_SYSTEM_QUEUED_SIZE:
            return (uint8_t) TELEMETRY_TOPIC_SYSTEM;
        default:
            return 0U;
    }
}

/* Send into the full queue after making room, called with g_push_mtx held.
   The send is retried after each eviction:
   - the oldest message is peeked with tk_ref_mbf; an event at the head stops
     the loop and the new frame is rejected;
   - otherwise the oldest is received and counted as dropped. If a consumer
     popped the peeked message first, the receive returns the next one. When
     that is an event it is sent back at once: the receive just freed its
     space, only this lock holder can fill space, and the consumer only frees
     more, so the send cannot fail. The event is then behind the newer frames
     (never lost) and the loop goes on: the new frame may already fit, or the
     next peek decides.
   Non-blocking (TMO_POL). At most QUEUE_EVICT_LIMIT rounds. Returns
   E_OK when the frame is stored, E_TMOUT when it was rejected (counted). */
static ER
send_evicting (const uint8_t * stored, INT stored_len)
{
    uint8_t scratch[TELEMETRY_QUEUE_MAX_MSG];

    for (uint32_t round = 0U; round < QUEUE_EVICT_LIMIT; round++)
    {
        T_RMBF oldest;
        INT    got = 0;

        if (tk_ref_mbf(g_mbf_telem_id, &oldest) != E_OK)
        {
            break;
        }
        if (oldest.msgsz == (INT) TELEMETRY_EVENT_QUEUED_SIZE)
        {
            break;
        }

        got = tk_rcv_mbf(g_mbf_telem_id, scratch, TMO_POL);
        if (got == (INT) TELEMETRY_EVENT_QUEUED_SIZE)
        {
            /* Cannot fail (see above); counted if it ever did. */
            if (tk_snd_mbf(g_mbf_telem_id, scratch, got, TMO_POL) != E_OK)
            {
                (void) atomic_inc(&g_drop_count);
            }
        }
        else if (got > 0)
        {
            (void) atomic_inc(&g_drop_count);
        }
        else if (got != E_TMOUT)
        {
            break;
        }

        if (tk_snd_mbf(g_mbf_telem_id, stored, stored_len, TMO_POL) == E_OK)
        {
            return E_OK;
        }
    }

    (void) atomic_inc(&g_drop_count);
    return E_TMOUT;
}

/* Several producer tasks push, on Core 0 and on the unpinned barcode task,
   and the unpinned worker pops, possibly on the other core.

   Producers hold g_push_mtx around every send, so that two producers never
   evict at once and nobody takes the space an evicting producer freed for a
   sent-back event. It is a mutex and not a spinlock because eviction needs
   tk_rcv_mbf, which returns E_CTX while interrupts or dispatch are disabled
   (CHECK_DISPATCH), and a plain spinlock held across a preemption would
   deadlock a higher priority task on the same core. TA_INHERIT bounds the
   wait of a high priority producer. Inside the lock only non-blocking
   (TMO_POL) message buffer calls run, each copies at most 16 B, and eviction
   is bounded by QUEUE_EVICT_LIMIT, so the lock is held for a few
   microseconds.

   The consumer does not take the mutex, because its pop may block. A pop
   between the peek and the receive is harmless: the producer only learns of
   it from the length of the message it receives. A non-event is dropped as
   intended (it is the oldest at that moment), an event is sent back, and the
   consumer only ever frees space. The one effect is that a sent-back event
   lands behind the frames queued after it. */
ER
telemetry_queue_push (const uint8_t * payload, uint16_t len, uint8_t qos)
{
    uint8_t stored[TELEMETRY_QUEUE_MAX_MSG];
    INT     stored_len = 0;
    ER      result     = E_OK;

    if (payload == 0)
    {
        return E_PAR;
    }

    if ((len == TELEMETRY_HEARTBEAT_QUEUED_SIZE) ||
        (len == TELEMETRY_EVENT_QUEUED_SIZE) ||
        (len == TELEMETRY_STATUS_QUEUED_SIZE) ||
        (len == TELEMETRY_SYSTEM_QUEUED_SIZE))
    {
        stored_len = (INT) len;
    }
    else
    {
        return E_PAR;
    }

    if (g_mbf_telem_id <= 0)
    {
        return E_NOEXS;
    }

    /* The topic byte is not stored; an event has no reserved byte 7. */
    for (INT i = 0; i < stored_len; i++)
    {
        stored[i] = payload[i];
    }

    result = tk_loc_mtx(g_push_mtx, TMO_FEVR);
    if (result != E_OK)
    {
        return result;
    }
    result = tk_snd_mbf(g_mbf_telem_id, stored, stored_len, TMO_POL);
    if ((result == E_TMOUT) && (qos == 0U))
    {
        result = send_evicting(stored, stored_len);
    }
    (void) tk_unl_mtx(g_push_mtx);
    return result;
}

INT
telemetry_queue_pop (uint8_t * payload, TMO tmout)
{
    uint8_t stored[TELEMETRY_QUEUE_MAX_MSG];
    uint8_t topic = 0U;
    INT     got = 0;

    if (payload == 0)
    {
        return E_PAR;
    }
    if (g_mbf_telem_id <= 0)
    {
        return E_NOEXS;
    }

    got = tk_rcv_mbf(g_mbf_telem_id, stored, tmout);
    if (got <= 0)
    {
        return got;
    }

    topic = topic_of_queued(got);
    if (topic == 0U)
    {
        /* Not a telemetry frame (cannot come from telemetry_queue_push). */
        (void) atomic_inc(&g_drop_count);
        return E_OBJ;
    }

    payload[0] = topic;
    for (INT i = 0; i < got; i++)
    {
        payload[i + 1] = stored[i];
    }
    if (topic == (uint8_t) TELEMETRY_TOPIC_EVENT)
    {
        payload[got + 1] = 0U;
        got++;
    }
    return got + 1;
}

UW
telemetry_queue_drop_count (void)
{
    return atomic_add(&g_drop_count, 0u);
}
