#ifndef TELEMETRY_STREAM_LOSS_H
#define TELEMETRY_STREAM_LOSS_H

#include <stdbool.h>
#include <stdint.h>

/* The count of lost streams after a stream ended. It moves only when the
 * client had bytes on that stream: it wrote some into the connection, or its
 * CONNECT is still queued for it. An idle stream, or a failed attempt with
 * nothing queued, lost nothing, and a count that moved while the client backs
 * off would make its first read on the next, healthy stream fail. */
static inline uint8_t
stream_loss_note (uint8_t lost, bool stream_tx, uint16_t tx_pending)
{
    if (stream_tx || (tx_pending > 0u))
    {
        return (uint8_t) (lost + 1u);
    }
    return lost;
}

#endif /* TELEMETRY_STREAM_LOSS_H */
