/** @file isr.h
 *
 * @brief The rp2040 only allows a single interrupt handler for GPIO events.
 * This module installs a handler than multiplexes other handlers, allowing
 * downstream handlers to fire for only certain GPIO events.
 *
 */
#ifndef PLATFORM_ISR_H
#define PLATFORM_ISR_H

#include <stdint.h>
#include <tk/tkernel.h>

#include "platform/static_assert.h"

/** Maximum number of handlers for GPIO interrupts. */
#define MAX_GPIO_IRQ_HANDLERS 4U

/**
 * The GPIO edges seen in one dispatcher entry, restricted to the pins a
 * handler registered.
 *
 * `timestamp_us` is the low word of the 1 MHz timer, read once when the
 * dispatcher is entered, so every handler in one entry sees the same value.
 * In both masks, bit n corresponds to GPIOn. `rise_mask` holds the pins with a
 * low to high edge, `fall_mask` the pins with a high to low edge. A pin can be
 * in both masks.
 */
typedef struct
{
    uint32_t timestamp_us;
    uint32_t rise_mask;
    uint32_t fall_mask;
} gpio_edge_snapshot_t;

STATIC_ASSERT(sizeof(gpio_edge_snapshot_t) == 12U,
              gpio_edge_snapshot_must_be_12_bytes);

/**
 * Called from the GPIO interrupt with the edges for the handler's pins. The
 * edges in the snapshot are already acknowledged. The pointer is only valid
 * during the call.
 */
typedef void (*gpio_irq_handler_t)(const gpio_edge_snapshot_t * p_snapshot);

ER gpio_irq_init (void);
/**
 * `handler` will be called on processor `proc` when any GPIO pin in `gpio_mask`
 * has a falling or rising edge. It is not called for other pins.
 *
 * In `gpio_mask`, the least significant bit corresponds to GPIO0, the 2nd least
 * significant bit corresponds to GPIO1, etc. The 2 most significant bits are
 * ignored as GPIO30 and GPIO31 do not exist.
 */
ER gpio_irq_add_handler (UW proc, uint32_t gpio_mask,
                         gpio_irq_handler_t handler);

#endif /* PLATFORM_ISR_H */

/*** end of file ***/
