/** @file encoder_capture.h
 * @brief RP2040 quadrature diagnostic capture using the SMP kernel.
 */
#ifndef ENCODER_CAPTURE_H
#define ENCODER_CAPTURE_H

#include <tk/tkernel.h>
#include "encoder_quad.h"

typedef struct
{
    uint32_t timestamp_us;
    encoder_quad_t quad;
} encoder_capture_sample_t;

/* Call once from a task pinned to TP_PRC1, before enabling other GPIO IRQs.
 * This diagnostic driver owns IO_IRQ_BANK0 exclusively. Integration with the
 * sonar needs a common dispatcher; do not install two independent handlers.
 * Returns E_BUSY if any GPIO IRQ is already enabled on either core.
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
