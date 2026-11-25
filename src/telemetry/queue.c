#include <tk/smp.h>

#include "telemetry/queue.h"

#define QUEUE_BUFFER_ALIGN 4

static uint8_t g_queue_buffer[TELEMETRY_QUEUE_BYTES]
    __attribute__((aligned(QUEUE_BUFFER_ALIGN)));
static UW g_drop_count;

ID g_mbf_telem_id;

ER
telemetry_queue_create (void)
{
    T_CMBF cmbf = {0};
    ER     result = E_OK;

    if (g_mbf_telem_id > 0)
    {
        return E_OK;
    }

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

    return result;
}

ER
telemetry_queue_push (const uint8_t * payload, uint16_t len, uint8_t topic,
                      uint8_t qos)
{
    uint8_t tagged[TELEMETRY_QUEUE_MAX_MSG];
    uint8_t discarded[TELEMETRY_QUEUE_MAX_MSG];
    ER       result = E_OK;
    INT      discarded_len = 0;
    uint16_t tagged_len = 0u;

    if ((payload == 0) || (len == 0u) || (len > (TELEMETRY_QUEUE_MAX_MSG - 1u)))
    {
        return E_PAR;
    }
    if ((topic != (uint8_t) 's') && (topic != (uint8_t) 'e') &&
        (topic != (uint8_t) 'h') && (topic != (uint8_t) 't'))
    {
        return E_PAR;
    }
    if (g_mbf_telem_id <= 0)
    {
        return E_NOEXS;
    }

    tagged[0] = topic;
    for (uint16_t i = 0u; i < len; i++)
    {
        tagged[i + 1u] = payload[i];
    }
    tagged_len = (uint16_t) (len + 1u);

    result = tk_snd_mbf(g_mbf_telem_id, tagged, (INT) tagged_len, TMO_POL);
    if ((result != E_TMOUT) || (qos != 0u))
    {
        return result;
    }

    discarded_len = tk_rcv_mbf(g_mbf_telem_id, discarded, TMO_POL);
    if (discarded_len <= 0)
    {
        return result;
    }

    result = tk_snd_mbf(g_mbf_telem_id, tagged, (INT) tagged_len, TMO_POL);
    if (result == E_OK)
    {
        (void) atomic_inc(&g_drop_count);
    }
    return result;
}

INT
telemetry_queue_pop (uint8_t * payload, TMO tmout)
{
    if (payload == 0)
    {
        return E_PAR;
    }
    if (g_mbf_telem_id <= 0)
    {
        return E_NOEXS;
    }

    return tk_rcv_mbf(g_mbf_telem_id, payload, tmout);
}

UW
telemetry_queue_drop_count (void)
{
    return atomic_add(&g_drop_count, 0u);
}
