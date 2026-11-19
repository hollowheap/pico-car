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

/* Call once, before any reader starts. Sets the encoder pads and starts the
 * state machines; they count with no interrupt and no CPU time. Returns
 * E_LIMIT when no PIO block has room for them, E_IO when a state machine
 * pushes no word.
 */
ER encoder_init (void);

/* Coherent copy from either core. Reads one fresh word per wheel under a
 * spinlock, about 1.5 us. Call before one wheel moves 2^23 steps (30 min
 * at full speed) or skips 256 states. Counter and time rollover is modulo
 * 2^32. Returns E_IO, with the counters unchanged, when a state machine
 * pushes no word within a few microseconds.
 */
ER encoder_read (encoder_sample_t * p_sample);

/* Diagnostic only: zero the counters from the present count. No concurrent
 * consumer may need the old counter baseline. */
ER encoder_reset (void);

#endif /* ENCODER_CAPTURE_H */

/*** end of file ***/
