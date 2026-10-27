/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "platform/isr.h"

#include <tk/syslib.h>

#define GPIO_PIN_COUNT      (30U)
#define PINS_PER_REG        (8U)
#define BITS_PER_PIN        (4U)
#define PIN_EVENT_MASK      (0xFU)
#define EDGE_EVENTS         (ISR_GPIO_EDGE_FALL | ISR_GPIO_EDGE_RISE)
#define REG_COUNT           (4U)
#define REG_STRIDE          (4U)

#define IO_INTR0_OFFSET     (0x0F0UL)
#define IO_PROC0_INTE0_OFS  (0x100UL)
#define IO_PROC0_INTS0_OFS  (0x120UL)
#define ATOMIC_SET_OFFSET   (0x2000UL)

#define IO_IRQ_BANK0_INTNO  (13U)
#define IO_IRQ_BANK0_LEVEL  (1)

typedef struct
{
    gpio_irq_callback_t callback;
    void *user_data;
} gpio_slot_t;

static gpio_slot_t g_slots[GPIO_PIN_COUNT];

static volatile uint32_t *io_reg(uint32_t offset, uint32_t index)
{
    return (volatile uint32_t *)((uintptr_t)IO_BANK0_BASE + offset + (index * REG_STRIDE));
}

static void dispatch_reg(uint32_t reg)
{
    uint32_t status = *io_reg(IO_PROC0_INTS0_OFS, reg);
    uint32_t slot;

    for (slot = 0U; slot < PINS_PER_REG; slot++)
    {
        uint32_t shift = slot * BITS_PER_PIN;
        uint32_t events = (status >> shift) & PIN_EVENT_MASK;
        uint32_t pin = (reg * PINS_PER_REG) + slot;

        if ((events != 0U) && (pin < GPIO_PIN_COUNT))
        {
            /* Edge bits are write-1-to-clear. */
            *io_reg(IO_INTR0_OFFSET, reg) = (events & EDGE_EVENTS) << shift;
            if (g_slots[pin].callback != NULL)
            {
                g_slots[pin].callback(pin, events, g_slots[pin].user_data);
            }
        }
    }
}

static void gpio_bank0_handler(UINT intno)
{
    uint32_t reg;

    (void)intno;
    for (reg = 0U; reg < REG_COUNT; reg++)
    {
        dispatch_reg(reg);
    }
}

void isr_dispatcher_init(void)
{
    T_DINT dint;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_bank0_handler;
    (void)tk_def_int(IO_IRQ_BANK0_INTNO, &dint);
    ClearInt(IO_IRQ_BANK0_INTNO);
    EnableInt(IO_IRQ_BANK0_INTNO, IO_IRQ_BANK0_LEVEL);
}

ER isr_register_gpio_handler(uint32_t pin, uint32_t event_mask, gpio_irq_callback_t cb, void *user_data)
{
    uint32_t shift;
    uint32_t intsts;
    ER result = E_OK;

    if ((pin >= GPIO_PIN_COUNT) || (pin == ISR_GPIO_CYW43_PIN) || (cb == NULL) ||
        (event_mask == 0U) || ((event_mask & ~EDGE_EVENTS) != 0U))
    {
        result = E_PAR;
    }
    else
    {
        DI(intsts);
        if (g_slots[pin].callback != NULL)
        {
            result = E_OBJ;
        }
        else
        {
            g_slots[pin].user_data = user_data;
            g_slots[pin].callback = cb;
            shift = (pin % PINS_PER_REG) * BITS_PER_PIN;
            /* Atomic set alias: safe against the other core touching INTE. */
            *io_reg(IO_PROC0_INTE0_OFS + ATOMIC_SET_OFFSET, pin / PINS_PER_REG) = event_mask << shift;
        }
        EI(intsts);
    }
    return result;
}
