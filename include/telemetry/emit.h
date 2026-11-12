/*
 *	emit.h
 *	Queue one telemetry event from any task (D110, S46).
 *
 *	An event is an 12-byte packet with a running id, a type from the events
 *	list in protocol/telemetry.yaml, the task that raised it, two parameters
 *	and the odometer at that moment. It goes into the telemetry queue as QoS 1:
 *	a status frame never evicts it. Safe from any task on either core.
 */

#ifndef TELEMETRY_EMIT_H
#define TELEMETRY_EMIT_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Returns E_OK, E_TMOUT when the queue is full, or the queue's error. A full
   queue drops the event; the drop is counted by the queue. */
ER telemetry_emit_event(uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
                        uint32_t odometer_mm);

/* Events queued since boot (the next id), for tests and diagnostics. */
uint16_t telemetry_event_count(void);

#endif /* TELEMETRY_EMIT_H */
