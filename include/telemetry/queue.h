#ifndef TELEMETRY_QUEUE_H
#define TELEMETRY_QUEUE_H

#include <stdint.h>
#include <tk/tkernel.h>

#include "telemetry/packet.h"

#define TELEMETRY_QUEUE_BYTES   1024u

/* Size of the buffer a caller passes to telemetry_queue_pop: the topic byte
   plus the largest wire packet (profile, 82 B). It is also the kernel maxmsz. */
#define TELEMETRY_QUEUE_MAX_MSG TELEMETRY_FRAME_MAX

/* A queued frame is the topic byte followed by the wire packet (D110, which
   supersedes D87: the topic is carried, not inferred from the length).
   Eviction has to recognise an event from its stored length alone, because
   tk_ref_mbf reports only a size. The event frame length is therefore unique
   among the frame lengths. */
#define TELEMETRY_EVENT_QUEUED_SIZE (TELEMETRY_EVENT_WIRE_SIZE + 1u)

/* Created by the composition root before producers or the consumer run. */
extern ID g_mbf_telem_id;

ER telemetry_queue_create(void);

/* Pushes a frame: frame[0] is the topic (TELEMETRY_TOPIC_*), the rest is the
   wire packet, and len is 1 + the packet's TELEMETRY_<NAME>_WIRE_SIZE. A topic
   that is unknown or whose length does not match returns E_PAR and nothing is
   stored. The command topic is not a queue topic.
   Never blocks on the message buffer (TMO_POL). Producers serialise on a
   priority-inheritance mutex that is held for a few non-blocking calls, so
   call it from a task, not from a handler. A nonzero qos frame is rejected
   with E_TMOUT on a full queue and nothing is evicted. A qos 0 frame on a
   full queue evicts the oldest frames while the oldest is a qos 0 frame,
   until it fits; if the oldest is an event, the new frame is rejected with
   E_TMOUT. Every frame lost this way (evicted or rejected qos 0) adds one
   to the drop counter. Safe to call from several tasks on both cores. */
ER telemetry_queue_push(const uint8_t *frame, uint16_t len, uint8_t qos);

/* Push a wire packet under `topic`: builds topic || packet and calls
   telemetry_queue_push. packet_len is the packet's TELEMETRY_<NAME>_WIRE_SIZE.
   Returns E_PAR for a NULL packet or a length above TELEMETRY_WIRE_MAX. */
ER telemetry_queue_push_packet(uint8_t topic, const uint8_t *packet,
                               uint16_t packet_len, uint8_t qos);

/* Writes topic || wire packet to frame, which must hold
   TELEMETRY_QUEUE_MAX_MSG bytes. Returns the frame length, or an error code. */
INT telemetry_queue_pop(uint8_t *frame, TMO tmout);

UW telemetry_queue_drop_count(void);

#endif /* TELEMETRY_QUEUE_H */
