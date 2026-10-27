/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Shared GPIO edge dispatcher on IO_IRQ_BANK0. */

#ifndef PLATFORM_ISR_H
#define PLATFORM_ISR_H

#include <stdint.h>
#include <tk/tkernel.h>

#define ISR_GPIO_EDGE_FALL (0x4U)
#define ISR_GPIO_EDGE_RISE (0x8U)

/* GP24 is the CYW43439 gSPI data/IRQ line and is never dispatched here. */
#define ISR_GPIO_CYW43_PIN (24U)

/* Called in interrupt context. Must finish within 2.5 us and not block. */
typedef void (*gpio_irq_callback_t)(uint32_t pin, uint32_t events, void *user_data);

/* Define the IO_IRQ_BANK0 handler and enable it. Call once, from Core 0. */
void isr_dispatcher_init(void);

/*
 * Register cb for the edges in event_mask on pin.
 * Returns E_PAR for GP24, pins above GP29, level events or a NULL callback,
 * and E_OBJ if the pin already has a handler.
 */
ER isr_register_gpio_handler(uint32_t pin, uint32_t event_mask, gpio_irq_callback_t cb, void *user_data);

#endif /* PLATFORM_ISR_H */
