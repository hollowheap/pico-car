/*
 *	emit.h
 *	Queue one telemetry event from any task.
 *
 *	An event is an 12-byte packet with a running id, a type from the events
 *	list in protocol/telemetry.yaml, the task that raised it, two parameters
 *	and the odometer at that moment. It goes into the telemetry queue as QoS 1:
 *	a status frame never evicts it. Safe from any task on either core.
 */

#ifndef TELEMETRY_EMIT_H
#define TELEMETRY_EMIT_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include "comms/telemetry/packet.h"

/* Bounded spinlocked handoff ring capacity for command acknowledgements. */
#define TELEMETRY_ACK_RING_CAPACITY (16U)
#define TELEMETRY_BARCODE_RING_CAPACITY (256U)

/* Returns E_OK, E_TMOUT when the queue is full, or the queue's error. A full
   queue drops the event; the drop is counted by the queue. */
ER telemetry_emit_event(uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
                        uint32_t odometer_mm);

/* Emits the same event format without waiting for the queue mutex or evicting.
   Task context only, outside locks. Contention and a full queue count drops. */
ER telemetry_try_emit_event(uint8_t source, uint8_t type, int16_t p1,
                            uint16_t p2, uint32_t odometer_mm);

/* Enqueues one supervisor command ACK (CMD_ACCEPT or CMD_REJECT) into the
   bounded spinlocked handoff ring without touching the telemetry queue mutex.
   Returns E_OK when stored, E_PAR for a non-ACK type, or E_TMOUT on overflow
   (counted in both the ACK overflow counter and the queue drop counter). */
ER telemetry_post_cmd_ack(uint8_t type, int16_t p1, uint16_t p2,
                          uint32_t odometer_mm);

/* Drains queued command ACKs into the telemetry queue in FIFO order without
   blocking or evicting. Returns the number of ACKs moved to the queue. */
uint32_t telemetry_drain_cmd_acks(void);

/* Command ACKs lost since boot because the handoff ring was full. */
UW telemetry_cmd_ack_overflow_count(void);

ER telemetry_try_emit_barcode_sample(telemetry_barcode_sample_t const * p_sample);
/* The radio copies a bounded FIFO prefix and retires it on a matching bridge receipt. */
void telemetry_peek_barcode_batch(telemetry_barcode_batch_t * p_batch);
bool telemetry_accept_barcode_batch(uint8_t count, uint32_t last_record_id);

/* Event IDs issued since boot (including dropped events). */
uint16_t telemetry_event_count(void);

/* Host tests only: clear the command ACK handoff ring and its overflow count. */
void telemetry_emit_test_reset(void);

#endif /* TELEMETRY_EMIT_H */
