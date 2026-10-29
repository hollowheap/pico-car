/** @file isr_dispatcher.c
 * @brief Shared Core 0 GPIO edge dispatcher on IO_IRQ_BANK0.
 */
#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tk/syslib.h>
#include <bsp/libbsp.h>

#include "platform/isr.h"

#define ISR_GPIO_INTNO      13U
#define ISR_GPIO_INT_PRIORITY 2U

#define IO_INTR0_OFFSET     0x0F0UL
#define IO_PROC0_INTE0_OFS  0x100UL
#define IO_PROC0_INTS0_OFS  0x120UL
#define IO_PROC1_INTE0_OFS  0x130UL

#define REG_COUNT           4U
#define REG_STRIDE          4U
#define PINS_PER_REG        8U
#define BITS_PER_PIN        4U
#define PIN_EDGE_BITS       0xCU
#define ALL_PIN_MASK        UINT32_C(0x3FFFFFFF)
#define REG_EDGE_MASK       UINT32_C(0xCCCCCCCC)

typedef struct
{
    uint32_t            pin_mask;
    uint32_t            event_mask;
    gpio_irq_callback_t callback;
    void *              user_data;
} isr_slot_t;

static isr_slot_t g_slots[ISR_GPIO_MAX_HANDLERS];
static T_SPLOCK   g_lock;
static bool       g_initialised;

static uintptr_t io_addr (uint32_t offset, uint32_t index)
{
    return (uintptr_t)IO_BANK0_BASE + offset + (index * REG_STRIDE);
}

static void enable_bits (uint32_t pin_mask, uint32_t event_mask,
                         uint32_t bits[REG_COUNT])
{
    uint32_t index;
    uint32_t pin;

    for (index = 0U; index < REG_COUNT; index++)
    {
        bits[index] = 0U;
    }
    for (pin = 0U; pin <= ISR_GPIO_MAX_PIN; pin++)
    {
        if ((pin_mask & (UINT32_C(1) << pin)) != 0U)
        {
            bits[pin / PINS_PER_REG] |=
                event_mask << ((pin % PINS_PER_REG) * BITS_PER_PIN);
        }
    }
}

static bool other_core_owns (const uint32_t bits[REG_COUNT])
{
    uint32_t index;

    for (index = 0U; index < REG_COUNT; index++)
    {
        if ((in_w(io_addr(IO_PROC1_INTE0_OFS, index)) & bits[index]) != 0U)
        {
            return true;
        }
    }
    return false;
}

static void gpio_bank0_handler (UINT intno)
{
    uint32_t rise = 0U;
    uint32_t fall = 0U;
    uint32_t reg;
    uint32_t index;

    (void)intno;

    for (reg = 0U; reg < REG_COUNT; reg++)
    {
        const uint32_t status = in_w(io_addr(IO_PROC0_INTS0_OFS, reg));
        const uint32_t edge   = status & REG_EDGE_MASK;
        uint32_t       slot;

        if (edge == 0U)
        {
            continue;
        }

        /* Ack only observed bits: a new edge re-fires the interrupt. */
        out_w(io_addr(IO_INTR0_OFFSET, reg), edge);

        for (slot = 0U; slot < PINS_PER_REG; slot++)
        {
            const uint32_t events =
                (edge >> (slot * BITS_PER_PIN)) & PIN_EDGE_BITS;
            const uint32_t pin = (reg * PINS_PER_REG) + slot;

            if (events != 0U)
            {
                if ((events & ISR_GPIO_EDGE_RISE) != 0U)
                {
                    rise |= UINT32_C(1) << pin;
                }
                if ((events & ISR_GPIO_EDGE_FALL) != 0U)
                {
                    fall |= UINT32_C(1) << pin;
                }
            }
        }
    }

    if ((rise | fall) == 0U)
    {
        return;
    }

    for (index = 0U; index < ISR_GPIO_MAX_HANDLERS; index++)
    {
        isr_gpio_event_t event;
        const isr_slot_t *slot = &g_slots[index];

        if (slot->callback == NULL)
        {
            continue;
        }

        event.changed_pins = (rise | fall) & slot->pin_mask;
        event.rise_pins    = rise & slot->pin_mask & slot->event_mask;
        event.fall_pins    = fall & slot->pin_mask & slot->event_mask;
        if ((event.rise_pins | event.fall_pins) != 0U)
        {
            slot->callback(&event, slot->user_data);
        }
    }
}

ER isr_dispatcher_init (void)
{
    T_DINT dint = {0};
    ER     result;

    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }
    if (g_initialised)
    {
        return E_OBJ;
    }

    (void)InitSpinLock(&g_lock);
    DisableInt(ISR_GPIO_INTNO);
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_bank0_handler;
    result      = tk_def_int(ISR_GPIO_INTNO, &dint);
    if (result != E_OK)
    {
        return result;
    }
    g_initialised = true;
    ClearInt(ISR_GPIO_INTNO);
    EnableInt(ISR_GPIO_INTNO, ISR_GPIO_INT_PRIORITY);
    return E_OK;
}

ER isr_register_gpio_handler (uint32_t            pin_mask,
                              uint32_t            event_mask,
                              gpio_irq_callback_t cb,
                              void *              user_data)
{
    uint32_t bits[REG_COUNT];
    uint32_t index;
    uint32_t free_index = ISR_GPIO_MAX_HANDLERS;
    UINT     saved;
    ER       result = E_OK;

    if ((pin_mask == 0U) || ((pin_mask & ~ALL_PIN_MASK) != 0U) ||
        ((pin_mask & (UINT32_C(1) << ISR_GPIO_CYW43_PIN)) != 0U) ||
        (event_mask == 0U) ||
        ((event_mask & ~ISR_GPIO_EDGE_ANY) != 0U) || (cb == NULL))
    {
        return E_PAR;
    }
    if (!g_initialised)
    {
        return E_OBJ;
    }
    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }

    enable_bits(pin_mask, event_mask, bits);
    (void)ISpinLock(&g_lock, &saved);
    if (other_core_owns(bits))
    {
        result = E_OBJ;
    }
    else
    {
        for (index = 0U; index < ISR_GPIO_MAX_HANDLERS; index++)
        {
            if ((g_slots[index].callback != NULL) &&
                ((g_slots[index].pin_mask & pin_mask) != 0U))
            {
                result = E_OBJ;
                break;
            }
        }
        if (result == E_OK)
        {
            for (index = 0U; index < ISR_GPIO_MAX_HANDLERS; index++)
            {
                if (g_slots[index].callback == NULL)
                {
                    free_index = index;
                    break;
                }
            }
            if (free_index == ISR_GPIO_MAX_HANDLERS)
            {
                result = E_LIMIT;
            }
        }
    }

    if (result == E_OK)
    {
        DisableInt(ISR_GPIO_INTNO);
        for (index = 0U; index < REG_COUNT; index++)
        {
            set_w(io_addr(IO_PROC0_INTE0_OFS, index), bits[index]);
        }
        g_slots[free_index].pin_mask   = pin_mask;
        g_slots[free_index].event_mask = event_mask;
        g_slots[free_index].user_data  = user_data;
        mp_memory_barrir();
        g_slots[free_index].callback = cb;
        EnableInt(ISR_GPIO_INTNO, ISR_GPIO_INT_PRIORITY);
    }
    (void)ISpinUnlock(&g_lock, saved);
    return result;
}

/*** end of file ***/
