/** @file gpio_irq.c
 *
 * @brief The rp2040 only allows a single interrupt handler for GPIO events.
 * This module installs a handler than multiplexes other handlers, allowing
 * downstream handlers to fire for only certain GPIO events.
 *
 */
#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h>
#include <tk/tkernel.h>

/** Maximum number of handlers for GPIO interrupts. */
#define MAX_GPIO_IRQ_HANDLERS 4U

/** Matches ut-kernel's ISR definition. */
typedef void (*gpio_irq_handler_t)(UINT intno);

ER gpio_irq_init (void);
/**
 * `handler` will be called on processor `proc` when any GPIO pin in `gpio_mask`
 * has a falling or rising edge.
 *
 * In `gpio_mask`, the least significant bit corresponds to GPIO0, the 2nd least
 * significant bit corresponds to GPIO1, etc. The 2 most significant bits are
 * ignored as GPIO30 and GPIO31 do not exist.
 */
ER gpio_irq_add_handler (UW proc, uint32_t gpio_mask,
                         gpio_irq_handler_t handler);

#endif /* GPIO_IRQ_H */

/*** end of file ***/
