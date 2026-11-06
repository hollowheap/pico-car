#ifndef TELEMETRY_QUEUE_H
#define TELEMETRY_QUEUE_H

#include <stdint.h>
#include <tk/tkernel.h>

#define TELEMETRY_QUEUE_BYTES   384u
#define TELEMETRY_QUEUE_MAX_MSG 17u

/* Created by the composition root before producers or the consumer run. */
extern ID mbf_telem_id;

ER telemetry_queue_create(void);

/* Messages are a one-byte topic tag followed by the wire frame; maxmsz is
   the tag plus the largest frame.  qos 0 is droppable status; nonzero frames
   are not dropped on a full queue. */
ER telemetry_queue_push(const uint8_t *payload, uint16_t len, uint8_t topic,
                        uint8_t qos);

/* Returns the message size, or an error code. */
INT telemetry_queue_pop(uint8_t *payload, TMO tmout);

UW telemetry_queue_drop_count(void);

#endif /* TELEMETRY_QUEUE_H */
