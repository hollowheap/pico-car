/** @file drv_encoder.h
 * @brief The quadrature state machines: one per wheel on one PIO block.
 *
 * This file and encoder_pio.c use the Pico SDK PIO driver and no kernel call.
 * The SDK claims keep the encoder clear of the CYW43 state machine.
 */
#ifndef ENCODER_PIO_H
#define ENCODER_PIO_H

#include <stdbool.h>
#include <stdint.h>

#include "quadrature.h"

/* Claim a PIO block with room for the program and two free state machines,
 * and start one per wheel. pins_a[wheel] is the A pin; B is the next pin.
 * Returns false when no block has room; nothing stays claimed then. */
bool encoder_pio_start (const unsigned int pins_a[ENCODER_WHEELS]);

/* Stop both state machines, unclaim them and remove the program. Safe to call
 * when nothing is claimed. */
void encoder_pio_stop (void);

/* Polls of the RX FIFO before a wait gives up. The state machine pushes a
 * word about every 1.3 us (16 cycles per sample, 125 MHz). One poll is a
 * register read, a compare and a branch: at least 6 cycles, 48 ns, so 128
 * polls last at least 6.1 us, about 4.7 pushes. */
#define ENCODER_PIO_WAIT_SPINS (128U)

/* Poll is_empty up to spins times. True when it reported a word in time. */
static inline bool encoder_pio_wait (bool (*is_empty)(void * p_ctx),
                                     void * p_ctx, uint32_t spins)
{
    uint32_t spin = 0U;

    for (spin = 0U; spin < spins; spin++)
    {
        if (!is_empty(p_ctx))
        {
            return true;
        }
    }
    return false;
}

/* Drain each wheel's FIFO and wait for one fresh word, about 1.5 us. A state
 * machine that pushes nothing within ENCODER_PIO_WAIT_SPINS makes this return
 * false and leaves words unchanged. The caller serialises the calls. */
bool encoder_pio_read (uint32_t words[ENCODER_WHEELS]);

#endif /* ENCODER_PIO_H */

/*** end of file ***/
