#include <tk/smp.h>

#include <stdbool.h>
#include "telemetry/net_queue.h"
#include "telemetry/queue.h"

#define QUEUE_BUFFER_ALIGN 4
#define TELEMETRY_EVENT_HEADROOM_BYTES (128U)

static uint8_t g_queue_buffer[TELEMETRY_NET_QUEUE_BYTES]
    __attribute__((aligned(QUEUE_BUFFER_ALIGN)));
static ID g_mbf_net_tx_id;
static UW g_drop_count;

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
        g_mbf_net_tx_id = result;
        return E_OK;
    }
    return result;
}

ER telemetry_net_queue_push (const uint8_t * payload, uint16_t len)
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
    bool const b_periodic =
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_STATUS) &&
         ((len == (1U + TELEMETRY_STATUS_WIRE_SIZE)) || (len == (1U + TELEMETRY_STATUS_PIVOT_WIRE_SIZE)))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_MOTION) && (len == (1U + TELEMETRY_MOTION_WIRE_SIZE))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_PHYSICS) && (len == (1U + TELEMETRY_PHYSICS_WIRE_SIZE))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_OBSTACLE) && (len == (1U + TELEMETRY_OBSTACLE_WIRE_SIZE))) ||
        ((payload[0] == (uint8_t)TELEMETRY_TOPIC_DIAG) && (len == (1U + TELEMETRY_DIAG_COUNTS_WIRE_SIZE)));
    if (b_barcode || b_periodic)
    {
        /* Reserve uplink headroom so events are preserved ahead of periodic frames. */
        T_RMBF info;
        ER const status = tk_ref_mbf(g_mbf_net_tx_id, &info);
        SZ const reserve = b_barcode
            ? (SZ)(TELEMETRY_NET_QUEUE_BYTES / TELEMETRY_RAW_HEADROOM_DIVISOR)
            : (SZ)TELEMETRY_EVENT_HEADROOM_BYTES;
        if ((E_OK != status) || (info.frbufsz < reserve))
        {
            (void)atomic_inc(&g_drop_count);
            return (E_OK == status) ? E_TMOUT : status;
        }
    }
    const ER result = tk_snd_mbf(g_mbf_net_tx_id, payload, (INT)len, TMO_POL);

    if (result == E_TMOUT)
    {
        (void) atomic_inc(&g_drop_count);
    }
    return result;
}

UW telemetry_net_queue_drop_count (void)
{
    return atomic_add(&g_drop_count, 0u);
}

void telemetry_net_queue_test_reset (void)
{
    g_mbf_net_tx_id = 0;
}

INT telemetry_net_queue_pop (uint8_t * payload, TMO tmout)
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
