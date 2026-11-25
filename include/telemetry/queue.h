#ifndef TELEMETRY_QUEUE_H
#define TELEMETRY_QUEUE_H

#include <stdint.h>
#include <tk/tkernel.h>

#include "telemetry/packet.h"

/* Holds one 50 ms worst-case telemetry burst (~308 B) plus a quarter, with
 * headroom for optional raw IR frames. */
#define TELEMETRY_QUEUE_BYTES   512u
/* Admit optional raw IR samples only while at least half the queue is free. */
#define TELEMETRY_RAW_HEADROOM_DIVISOR (2U)

/* Size of the buffer a caller passes to telemetry_queue_pop: the topic byte
   plus the largest wire packet (profile, 82 B). It is also the kernel maxmsz. */
#define TELEMETRY_QUEUE_MAX_MSG TELEMETRY_FRAME_MAX

/* A queued frame is the topic byte followed by the wire packet (D110, which
   supersedes D87: the topic is carried, not inferred from the length).
   Eviction has to recognise an event from its stored length alone, because
   tk_ref_mbf reports only a size. The event frame length is therefore unique
   among the frame lengths. */
#define TELEMETRY_EVENT_QUEUED_SIZE (TELEMETRY_EVENT_WIRE_SIZE + 1u)

/* Called by the composition root before producers or the consumer run. */
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
   E_TMOUT. Every frame lost this way (evicted, or rejected at any qos) adds
   one to the drop counter. Safe to call from several tasks on both cores. */
ER telemetry_queue_push(const uint8_t *frame, uint16_t len, uint8_t qos);

/* Same frame contract, with one poll of the producer mutex and one send.
   Never evicts. Contention or a full buffer returns E_TMOUT and counts a drop.
   Call from a task outside interrupt handlers and other locks. */
ER telemetry_queue_try_push(const uint8_t *frame, uint16_t len);

/* Non-blocking, non-evicting push used when the caller retains the frame in a
   bounded handoff ring on E_TMOUT and retries later. Does not increment the
   queue drop counter on E_TMOUT. */
ER telemetry_queue_try_push_no_drop(const uint8_t *frame, uint16_t len);

/* Record one dropped frame from an upstream bounded handoff ring so diag and
   heartbeat drop counters reflect handoff overflow without changing the wire
   schema. */
void telemetry_queue_note_drop(void);

/* Push a wire packet under `topic`: builds topic || packet and calls
   telemetry_queue_push. packet_len is the packet's TELEMETRY_<NAME>_WIRE_SIZE.
   Returns E_PAR for a NULL packet or a length above TELEMETRY_WIRE_MAX. */
ER telemetry_queue_push_packet(uint8_t topic, const uint8_t *packet,
                               uint16_t packet_len, uint8_t qos);

/* Writes topic || wire packet to frame, which must hold
   TELEMETRY_QUEUE_MAX_MSG bytes. Returns the frame length, or an error code. */
INT telemetry_queue_pop(uint8_t *frame, TMO tmout);

/* Frames lost since boot: evicted, rejected on a full queue at any qos,
   dropped on try-lock contention, or foreign on pop. The diag frame reports
   it; the heartbeat adds it to the net queue's count. */
UW telemetry_queue_drop_count(void);

/* Host tests only: forget the queue so that the next telemetry_queue_create
   makes a new one after the test has reset its kernel fake. The kernel
   objects are not deleted, so firmware never calls it. */
void telemetry_queue_test_reset(void);

#endif /* TELEMETRY_QUEUE_H */
