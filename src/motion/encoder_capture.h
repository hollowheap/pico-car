/** @file encoder_capture.h
 * @brief RP2040 quadrature edge capture using the SMP kernel.
 */
#ifndef ENCODER_CAPTURE_H
#define ENCODER_CAPTURE_H

#include <tk/tkernel.h>
#include "quadrature.h"

typedef struct
{
    uint32_t              timestamp_us;
    quadrature_counts_t wheel[ENCODER_WHEELS];
} encoder_capture_sample_t;

/* Call once from a task pinned to TP_PRC1, after isr_dispatcher_init().
 * Registers the four encoder pins with the shared IO_IRQ_BANK0 dispatcher.
 * Returns E_OBJ if any pin is already owned by another dispatcher client.
 * A/B inputs must be valid 3.3 V quadrature signals; no internal pulls used.
 */
ER encoder_capture_init (void);

/* Coherent copy from either core, protected against capture IRQ updates.
 * No blocking kernel call, printing or conversion while holding this lock.
 * Counter/time rollover is modulo 2^32, not unlimited history.
 */
ER encoder_capture_read (encoder_capture_sample_t * p_sample);

#endif /* ENCODER_CAPTURE_H */

/*** end of file ***/
