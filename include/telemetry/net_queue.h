#ifndef TELEMETRY_NET_QUEUE_H
#define TELEMETRY_NET_QUEUE_H

#include <stdint.h>
#include <tk/tkernel.h>

#define TELEMETRY_NET_QUEUE_BYTES   384u
#define TELEMETRY_NET_QUEUE_MAX_MSG 17u

extern ID mbf_net_tx_id;

ER telemetry_net_queue_create (void);
ER telemetry_net_queue_push (const uint8_t * payload, uint16_t len);
INT telemetry_net_queue_pop (uint8_t * payload, TMO tmout);
UW telemetry_net_queue_drop_count (void);

#endif /* TELEMETRY_NET_QUEUE_H */
