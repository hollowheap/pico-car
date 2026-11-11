#ifndef TELEMETRY_QUEUE_H
#define TELEMETRY_QUEUE_H

#include <stdint.h>
#include <tk/tkernel.h>

#define TELEMETRY_QUEUE_BYTES   384u

/* Size of the buffer a caller passes to telemetry_queue_pop: the topic byte
   plus the largest wire frame (system, 16 B).  It is also the kernel maxmsz,
   which only has to be at least the largest stored length. */
#define TELEMETRY_QUEUE_MAX_MSG 17u

/* Length of each frame as stored in the message buffer (D87).  The topic is
   inferred from the length, so the four values must stay distinct.  The event
   is the 8 B wire frame without its reserved byte 7. */
#define TELEMETRY_HEARTBEAT_QUEUED_SIZE 4u
#define TELEMETRY_EVENT_QUEUED_SIZE     7u
#define TELEMETRY_STATUS_QUEUED_SIZE    8u
#define TELEMETRY_SYSTEM_QUEUED_SIZE    16u

/* Created by the composition root before producers or the consumer run. */
extern ID g_mbf_telem_id;

ER telemetry_queue_create(void);

/* Pushes a telemetry frame. The topic is inferred from the queued length
   (heartbeat 4, event 7, status 8, system 16 B; D87).
   Never blocks on the message buffer (TMO_POL). Producers serialise on a
   priority-inheritance mutex that is held for a few non-blocking calls, so
   call it from a task, not from a handler. A nonzero qos frame is rejected
   with E_TMOUT on a full queue and nothing is evicted. A qos 0 frame on a
   full queue evicts the oldest frames while the oldest is a qos 0 frame,
   until it fits; if the oldest is an event, the new frame is rejected with
   E_TMOUT. Every frame lost this way (evicted or rejected qos 0) adds one
   to the drop counter. Safe to call from several tasks on both cores. */
ER telemetry_queue_push(const uint8_t *payload, uint16_t len, uint8_t qos);

/* Writes topic || wire frame (events with byte 7 restored as 0) to payload,
   which must hold TELEMETRY_QUEUE_MAX_MSG bytes.  Returns the message size
   (1 + the wire frame length), or an error code. */
INT telemetry_queue_pop(uint8_t *payload, TMO tmout);

UW telemetry_queue_drop_count(void);

#endif /* TELEMETRY_QUEUE_H */
