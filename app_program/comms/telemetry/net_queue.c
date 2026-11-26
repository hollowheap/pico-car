#define PROHIBIT_DEF_SIZE_T
#include <stdbool.h>
#include <string.h>
#include <tk/smp.h>
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/queue.h"

#define QUEUE_BUFFER_ALIGN 4
#define TELEMETRY_EVENT_HEADROOM_BYTES (128U)

static uint8_t g_queue_buffer[TELEMETRY_NET_QUEUE_BYTES]
    __attribute__((aligned(QUEUE_BUFFER_ALIGN)));
static ID g_mbf_net_tx_id;
static UW g_drop_count;
/* Only the latest periodic bundle waits for the radio. Its slot has no
 * effect on the shared FIFO capacity. */
static T_SPLOCK g_status_lock;
static uint8_t  g_status[TELEMETRY_STATUS_BUNDLE_FRAME_SIZE];
static uint32_t g_status_version;
static UW       g_status_replacements;
static bool     g_b_status_pending;

ER telemetry_net_queue_create (void)
{
    T_CMBF cmbf = {0};

    if (g_mbf_net_tx_id > 0)
    {
        return E_OK;
    }

    cmbf.mbfatr = TA_USERBUF | TA_TPRI;
    cmbf.bufsz  = TELEMETRY_NET_QUEUE_BYTES;
    cmbf.maxmsz = TELEMETRY_NET_QUEUE_MAX_MSG;
    cmbf.bufptr = g_queue_buffer;

    const ER result = tk_cre_mbf(&cmbf);
    if (result > 0)
    {
        (void)InitSpinLock(&g_status_lock);
        g_mbf_net_tx_id = result;
        return E_OK;
    }
    return result;
}

static ER push (const uint8_t * payload, uint16_t len, bool b_count_drop)
{
    if ((payload == 0) || (len == 0U) || (len > TELEMETRY_NET_QUEUE_MAX_MSG))
    {
        return E_PAR;
    }
    if (g_mbf_net_tx_id <= 0)
    {
        return E_NOEXS;
    }
    bool const b_barcode = (len == (1U + TELEMETRY_BARCODE_SAMPLE_WIRE_SIZE)) &&
                           (payload[0] == (uint8_t)TELEMETRY_TOPIC_BARCODE_SAMPLE);
    bool const b_bundle = (len == TELEMETRY_STATUS_BUNDLE_FRAME_SIZE) &&
                          (payload[0] == (uint8_t)TELEMETRY_TOPIC_STATUS);
    if (b_bundle)
    {
        UINT saved = 0U;

        (void)ISpinLock(&g_status_lock, &saved);
        if (g_b_status_pending)
        {
            g_status_replacements++;
        }
        (void)memcpy(g_status, payload, sizeof(g_status));
        g_status_version++;
        g_b_status_pending = true;
        (void)ISpinUnlock(&g_status_lock, saved);
        return E_OK;
    }
    bool const b_periodic =
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_STATUS) &&
         ((len == (1U + TELEMETRY_STATUS_WIRE_SIZE)) ||
          (len == (1U + TELEMETRY_STATUS_PIVOT_WIRE_SIZE)))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_MOTION) &&
         (len == (1U + TELEMETRY_MOTION_WIRE_SIZE))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_PHYSICS) &&
         (len == (1U + TELEMETRY_PHYSICS_WIRE_SIZE))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_OBSTACLE) &&
         (len == (1U + TELEMETRY_OBSTACLE_WIRE_SIZE))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_DIAG) &&
         (len == (1U + TELEMETRY_DIAG_COUNTS_WIRE_SIZE)));
    if (b_barcode || b_periodic)
    {
        /* The telemetry task is the only FIFO producer and the radio only
         * frees space. */
        T_RMBF   info;
        ER const status  = tk_ref_mbf(g_mbf_net_tx_id, &info);
        SZ const reserve = b_barcode
                               ? (SZ)(TELEMETRY_NET_QUEUE_BYTES /
                                      TELEMETRY_RAW_HEADROOM_DIVISOR)
                               : (SZ)TELEMETRY_EVENT_HEADROOM_BYTES;
        if ((E_OK != status) || (info.frbufsz < reserve))
        {
            if (b_count_drop)
            {
                (void)atomic_inc(&g_drop_count);
            }
            return (E_OK == status) ? E_TMOUT : status;
        }
    }
    const ER result = tk_snd_mbf(g_mbf_net_tx_id, payload, (INT)len, TMO_POL);

    if ((result == E_TMOUT) && b_count_drop)
    {
        (void) atomic_inc(&g_drop_count);
    }
    return result;
}

ER telemetry_net_queue_push (const uint8_t * payload, uint16_t len)
{
    return push(payload, len, true);
}

ER telemetry_net_queue_push_retained_event (const uint8_t * frame)
{
    if ((frame == 0) || (frame[0] != (uint8_t)TELEMETRY_TOPIC_EVENT))
    {
        return E_PAR;
    }
    return push(frame, TELEMETRY_EVENT_QUEUED_SIZE, false);
}

UW telemetry_net_queue_drop_count (void)
{
    return atomic_add(&g_drop_count, 0u);
}

void telemetry_net_queue_test_reset (void)
{
    g_mbf_net_tx_id = 0;
    g_b_status_pending = false;
}

INT telemetry_net_queue_pop_fifo (uint8_t * payload, TMO tmout)
{
    if (payload == 0)
    {
        return E_PAR;
    }
    if (g_mbf_net_tx_id <= 0)
    {
        return E_NOEXS;
    }
    return tk_rcv_mbf(g_mbf_net_tx_id, payload, tmout);
}

bool telemetry_net_queue_peek_status (uint8_t * payload, uint32_t * p_version)
{
    UINT saved = 0U;

    (void)ISpinLock(&g_status_lock, &saved);
    bool const b_pending = g_b_status_pending;
    if (b_pending)
    {
        (void)memcpy(payload, g_status, sizeof(g_status));
        *p_version = g_status_version;
    }
    (void)ISpinUnlock(&g_status_lock, saved);
    return b_pending;
}

void telemetry_net_queue_accept_status (uint32_t version)
{
    UINT saved = 0U;

    (void)ISpinLock(&g_status_lock, &saved);
    if (version == g_status_version)
    {
        g_b_status_pending = false;
    }
    (void)ISpinUnlock(&g_status_lock, saved);
}

UW telemetry_net_queue_status_replacements (void)
{
    UINT saved = 0U;

    (void)ISpinLock(&g_status_lock, &saved);
    UW const count = g_status_replacements;
    (void)ISpinUnlock(&g_status_lock, saved);
    return count;
}

INT telemetry_net_queue_pop (uint8_t * payload, TMO tmout)
{
    uint32_t version = 0U;
    INT const len = telemetry_net_queue_pop_fifo(payload, tmout);

    if ((len == E_TMOUT) && telemetry_net_queue_peek_status(payload, &version))
    {
        telemetry_net_queue_accept_status(version);
        return TELEMETRY_STATUS_BUNDLE_FRAME_SIZE;
    }
    return len;
}
