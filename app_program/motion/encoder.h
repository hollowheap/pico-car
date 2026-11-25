/** @file encoder.h
 * @brief RP2040 quadrature capture: one PIO state machine per wheel.
 */
#ifndef ENCODER_CAPTURE_H
#define ENCODER_CAPTURE_H

#include <tk/tkernel.h>
#include "quadrature.h"

typedef struct
{
    uint32_t            timestamp_us;
    quadrature_counts_t wheel[ENCODER_WHEELS];
} encoder_sample_t;

/* Call once from task context, before any reader starts. Sets the encoder pads,
 * creates the capture mutex and starts the state machines; they count with no
 * interrupt and no CPU time. Returns E_OBJ if already initialised, E_LIMIT when
 * no mutex or PIO block is available, E_CTX outside task context, or E_IO when
 * a state machine pushes no word (releasing claimed resources on failure).
 */
ER encoder_init (void);

/* Coherent copy from task context on either core. Serialises PIO FIFO waits
 * (~1.5 us) with a TA_INHERIT mutex while leaving interrupts unmasked, and
 * updates the paired wheel counts and timestamp under a spinlock. Call before
 * one wheel moves 2^23 steps (30 min at full speed) or skips 256 states.
 * Counter and time rollover is modulo 2^32. Returns E_OBJ before init, E_CTX
 * outside task context, or E_IO, with the counters unchanged, when a state
 * machine pushes no word within a few microseconds.
 */
ER encoder_read (encoder_sample_t * p_sample);

/* Diagnostic only (task context): zero the counters from the present count
 * under the TA_INHERIT mutex and spinlock. Returns E_OBJ before init, E_CTX
 * outside task context, or E_IO with counters unchanged. No concurrent
 * consumer may need the old counter baseline. */
ER encoder_reset (void);

#endif /* ENCODER_CAPTURE_H */

/*** end of file ***/
