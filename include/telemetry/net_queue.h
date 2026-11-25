#ifndef TELEMETRY_NET_QUEUE_H
#define TELEMETRY_NET_QUEUE_H

#include <stdint.h>
#include <tk/tkernel.h>

#include "telemetry/packet.h"

/* Holds one 100 ms connection poll burst (~328 B) plus a quarter. */
#define TELEMETRY_NET_QUEUE_BYTES   512u
/* Topic byte + the largest wire packet (D110). */
#define TELEMETRY_NET_QUEUE_MAX_MSG TELEMETRY_FRAME_MAX

ER telemetry_net_queue_create (void);
ER telemetry_net_queue_push (const uint8_t * payload, uint16_t len);
INT telemetry_net_queue_pop (uint8_t * payload, TMO tmout);
UW telemetry_net_queue_drop_count (void);

/* Host tests only: forget the queue so that the next telemetry_net_queue_create
   makes a new one after the test has reset its kernel fake. The kernel object
   is not deleted, so firmware never calls it. */
void telemetry_net_queue_test_reset (void);

#endif /* TELEMETRY_NET_QUEUE_H */
