#ifndef TELEMETRY_NET_QUEUE_H
#define TELEMETRY_NET_QUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "comms/telemetry/packet.h"

/* Holds one 100 ms connection poll burst (~328 B) plus a quarter. */
#define TELEMETRY_NET_QUEUE_BYTES   512u
/* Topic byte + the largest wire packet. */
#define TELEMETRY_NET_QUEUE_MAX_MSG TELEMETRY_FRAME_MAX

ER telemetry_net_queue_create (void);
ER telemetry_net_queue_push (const uint8_t * payload, uint16_t len);
/* Retains the caller's ownership on failure and leaves the drop count alone. */
ER telemetry_net_queue_push_retained_event (const uint8_t * frame);
INT telemetry_net_queue_pop (uint8_t * payload, TMO tmout);
INT telemetry_net_queue_pop_fifo (uint8_t * payload, TMO tmout);
/* The radio can keep a bundle pending across a failed MQTT publish. Accept
 * only the version that was handed to the client. */
bool telemetry_net_queue_peek_status (uint8_t * payload, uint32_t * p_version);
void telemetry_net_queue_accept_status (uint32_t version);
UW telemetry_net_queue_status_replacements (void);
UW telemetry_net_queue_drop_count (void);

/* Host tests only: forget the queue so that the next telemetry_net_queue_create
   makes a new one after the test has reset its kernel fake. The kernel object
   is not deleted, so firmware never calls it. */
void telemetry_net_queue_test_reset (void);

#endif /* TELEMETRY_NET_QUEUE_H */
