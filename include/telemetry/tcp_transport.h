#ifndef TELEMETRY_TCP_TRANSPORT_H
#define TELEMETRY_TCP_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

#define TCP_TRANSPORT_HOST_MAX      64u
#define TCP_TRANSPORT_RX_CAPACITY  512u
#define TCP_TRANSPORT_TX_CAPACITY  512u

typedef enum
{
    TCP_TRANSPORT_IDLE = 0,
    TCP_TRANSPORT_WAITING,
    TCP_TRANSPORT_RESOLVING,
    TCP_TRANSPORT_CONNECTING,
    TCP_TRANSPORT_CONNECTED,
    TCP_TRANSPORT_FAILED,
    TCP_TRANSPORT_CLOSED
} tcp_transport_state_t;

STATIC_ASSERT (sizeof(tcp_transport_state_t) <= 4U,
               "tcp_transport_state_t size");

/* Queue a connection request.  The lwIP owner task opens it on the next poll. */
bool tcp_transport_open (const char * host, uint16_t port);

/* Ask the lwIP owner task to close the connection. */
void tcp_transport_close (void);

tcp_transport_state_t tcp_transport_state (void);

/* lwIP millisecond clock without dragging lwIP headers into T-Kernel TUs. */
uint32_t tcp_transport_now_ms (void);

/* mqtt_transport_t send/recv adapters.  Both run on the calling task and only
 * touch the lock-free rings. */
int tcp_transport_send (void * context, const uint8_t * data, uint16_t len);
int tcp_transport_recv (void * context, uint8_t * data, uint16_t len);

/* Called by the CYW43/lwIP owner task only. */
void tcp_transport_poll (void);

#endif /* TELEMETRY_TCP_TRANSPORT_H */
