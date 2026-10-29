/** @file isr.h
 * @brief Shared Core 0 GPIO edge dispatcher on IO_IRQ_BANK0.
 */
#ifndef PLATFORM_ISR_H
#define PLATFORM_ISR_H

#include <stdint.h>
#include <tk/tkernel.h>

#define ISR_GPIO_EDGE_FALL (0x4U)
#define ISR_GPIO_EDGE_RISE (0x8U)
#define ISR_GPIO_EDGE_ANY  (ISR_GPIO_EDGE_FALL | ISR_GPIO_EDGE_RISE)

/* GP24 is the CYW43439 gSPI line. */
#define ISR_GPIO_CYW43_PIN (24U)
#define ISR_GPIO_MAX_PIN   (29U)
#define ISR_GPIO_MAX_HANDLERS (4U)

typedef struct
{
    uint32_t changed_pins;
    uint32_t rise_pins;
    uint32_t fall_pins;
} isr_gpio_event_t;

/* Interrupt context: finish within 2.5 us and do not block. */
typedef void (*gpio_irq_callback_t)(const isr_gpio_event_t * event,
                                    void *                   user_data);

ER isr_dispatcher_init (void);

/* Register a non-overlapping pin mask and edge selection. */
ER isr_register_gpio_handler (uint32_t            pin_mask,
                              uint32_t            event_mask,
                              gpio_irq_callback_t cb,
                              void *              user_data);

#endif /* PLATFORM_ISR_H */

/*** end of file ***/
