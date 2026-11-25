#include <tk/smp.h>

#include "telemetry/net_queue.h"

static uint8_t s_queue_buffer[TELEMETRY_NET_QUEUE_BYTES]
    __attribute__((aligned(4)));
ID mbf_net_tx_id;
static UW s_drop_count;

ER telemetry_net_queue_create (void)
{
    T_CMBF cmbf = {0};

    if (mbf_net_tx_id > 0)
    {
        return E_OK;
    }

    cmbf.mbfatr = TA_USERBUF | TA_TPRI;
    cmbf.bufsz  = TELEMETRY_NET_QUEUE_BYTES;
    cmbf.maxmsz = TELEMETRY_NET_QUEUE_MAX_MSG;
    cmbf.bufptr = s_queue_buffer;

    const ER result = tk_cre_mbf(&cmbf);
    if (result > 0)
    {
        mbf_net_tx_id = result;
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
    if (mbf_net_tx_id <= 0)
    {
        return E_NOEXS;
    }
    const ER result = tk_snd_mbf(mbf_net_tx_id, payload, (INT)len, TMO_POL);

    if (result == E_TMOUT)
    {
        (void) atomic_inc(&s_drop_count);
    }
    return result;
}

UW telemetry_net_queue_drop_count (void)
{
    return s_drop_count;
}

INT telemetry_net_queue_pop (uint8_t * payload, TMO tmout)
{
    if (payload == 0)
    {
        return E_PAR;
    }
    if (mbf_net_tx_id <= 0)
    {
        return E_NOEXS;
    }
    return tk_rcv_mbf(mbf_net_tx_id, payload, tmout);
}
