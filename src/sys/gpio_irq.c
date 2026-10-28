#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "gpio_irq.h"

#define GPIO_INTNO     13
#define GPIO_INT_PRI   2
#define GPIO_IRQ_COUNT 30U
#define ALL_GPIO_MASK  ((1U << GPIO_IRQ_COUNT) - 1U)

#define INT_REG_COUNT 4U
#define INTR          (IO_BANK0_BASE + 0xF0U)
#define PROC1_INTE    (IO_BANK0_BASE + 0x100U)
#define PROC1_INTS    (IO_BANK0_BASE + 0x120U)
#define PROC2_INTE    (IO_BANK0_BASE + 0x130U)
#define PROC2_INTS    (IO_BANK0_BASE + 0x150U)

#if TK_SUPPORT_SMP
#   include <tk/smp.h>
#   define THIS_PRC() tk_get_prc()
#else
#   define THIS_PRC() TP_PRC1
#endif

typedef struct
{
    gpio_irq_handler_t handler;
    uint32_t           mask;
} handler_t;

LOCAL T_SPLOCK g_handlers_lock;
/**
 * Handlers must be added in sequence, and can never be removed.
 * A handler is considered to be added when the `handler` member is non-zero.
 * This constraint means that the ISR can read the handlers safely
 * without a lock.
 */
LOCAL volatile handler_t g_handlers[MAX_GPIO_IRQ_HANDLERS] = {0};

/**
 * Expands `gpio_mask` into the EDGE_LOW and EDGE_HIGH bits for each GPIO which
 * has its corresponding bit set. The bits are written to `enable_masks`.
 */
LOCAL void get_edge_masks (const uint32_t gpio_mask,
                           uint32_t       enable_masks[INT_REG_COUNT])
{
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        const uint32_t mask_byte = (gpio_mask >> (idx * 8U)) & 0xFFU;
        const uint32_t m_lo      = mask_byte & 0xFU;
        const uint32_t m_hi      = mask_byte >> 4U;

        // Spread each bit to every 3rd and 4th bit. These correspond to the
        // EDGE_LOW and EDGE_HIGH bits in the registers.
        // As each bit is in a 4 bit slot, we can only spread 4 bits at a time.
        // Example: 0b0110 => 0b0000 1100 1100 0000
        // Based on: https://stackoverflow.com/a/79404703
        const uint32_t enable_lo =
            (((m_lo & 0x5U) * 0x924U) | ((m_lo & 0xAU) * 0x924U)) & 0x4444U;
        const uint32_t enable_hi =
            (((m_hi & 0x5U) * 0x924U) | ((m_hi & 0xAU) * 0x924U)) & 0x4444U;
        const uint32_t enable = (enable_hi << 16U) | enable_lo;
        enable_masks[idx]     = enable | (enable << 1U);
    }
}

/**
 * Returns whether any bits of the status registers are set, after masking with
 * `enable_masks`. The masked status registers are written to `status_masks`.
 */
LOCAL bool has_status (const UW       proc,
                       const uint32_t enable_masks[INT_REG_COUNT],
                       uint32_t       status_masks[INT_REG_COUNT])
{
    const UW status_base = (proc == TP_PRC1) ? PROC1_INTS : PROC2_INTS;
    uint32_t status      = 0;
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        uint32_t status_w = in_w(status_base + (idx * sizeof(UW)));
        status_masks[idx] = status_w & enable_masks[idx];
        status |= status_masks[idx];
    }
    return status != 0;
}

/** Counts the number of commited handlers in `g_handlers`. */
LOCAL uint8_t handler_count (void)
{
    uint8_t i = 0;
    for (; i < MAX_GPIO_IRQ_HANDLERS; i++)
    {
        if (g_handlers[i].handler == 0)
        {
            break;
        }
    }
    return i;
}

LOCAL ER enable_pin_interrupts (const UW       proc,
                                const uint32_t enable_masks[INT_REG_COUNT])
{
    const UW enable_base       = (proc == TP_PRC1) ? PROC1_INTE : PROC2_INTE;
    const UW other_enable_base = (proc == TP_PRC1) ? PROC2_INTE : PROC1_INTE;
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        const uint32_t enabled = in_w(other_enable_base + (idx * sizeof(UW)));
        if ((enabled & enable_masks[idx]) != 0)
        {
            // The other processor already owns this pin; we cannot claim it
            return E_OBJ;
        }
    }
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        set_w(enable_base + (idx * sizeof(UW)), enable_masks[idx]);
    }
    return E_OK;
}

LOCAL void ack_gpio_irqs (const uint32_t ack_masks[INT_REG_COUNT])
{
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        out_w(INTR + (idx * sizeof(UW)), ack_masks[idx]);
    }
}

LOCAL void gpio_isr (UINT intno)
{
    for (uint8_t idx = 0; idx < MAX_GPIO_IRQ_HANDLERS; idx++)
    {
        if (g_handlers[idx].handler == 0)
        {
            break;
        }

        uint32_t enable_masks[INT_REG_COUNT];
        get_edge_masks(g_handlers[idx].mask, enable_masks);
        uint32_t status_masks[INT_REG_COUNT];
        if (has_status(THIS_PRC(), enable_masks, status_masks))
        {
            g_handlers[idx].handler(intno);
            ack_gpio_irqs(status_masks);
        }
    }
}

// TODO: set interrupt on both cores
EXPORT ER gpio_irq_init (void)
{
    DisableInt(GPIO_INTNO);
    ER result = tk_def_int(GPIO_INTNO, &(T_DINT){
                                           .intatr = TA_HLNG,
                                           .inthdr = (FP)gpio_isr,
                                       });
    if (result != E_OK)
    {
        return result;
    }
    (void)InitSpinLock(&g_handlers_lock);
    ClearInt(GPIO_INTNO);
    EnableInt(GPIO_INTNO, GPIO_INT_PRI);
    return E_OK;
}

EXPORT ER gpio_irq_add_handler (UW proc, uint32_t gpio_mask,
                                gpio_irq_handler_t handler)
{
    if (handler_count() >= MAX_GPIO_IRQ_HANDLERS)
    {
        return E_LIMIT;
    }

    gpio_mask &= ALL_GPIO_MASK;
    uint32_t edge_masks[INT_REG_COUNT];
    get_edge_masks(gpio_mask, edge_masks);

    (void)SpinLock(&g_handlers_lock);
    // We must check again as someone else could have added a handler before we
    // aqquired the lock.
    const uint8_t handler_idx = handler_count();
    if (handler_idx >= MAX_GPIO_IRQ_HANDLERS)
    {
        (void)SpinUnlock(&g_handlers_lock);
        return E_LIMIT;
    }

    // Disable interrupts before enabling interrupts for the pin.
    // Otherwise, the interrupt may fire immediately and `gpio_isr` will not
    // clear it as the mask has not been commited to `g_handlers`.
    DisableInt(GPIO_INTNO);
    const ER result = enable_pin_interrupts(proc, edge_masks);
    if (result != E_OK)
    {
        (void)SpinUnlock(&g_handlers_lock);
        EnableInt(GPIO_INTNO, GPIO_INT_PRI);
        return result;
    }

    g_handlers[handler_idx].mask = gpio_mask;
    mp_memory_barrir();
    // Set the handler last to commit it
    g_handlers[handler_idx].handler = handler;

    (void)SpinUnlock(&g_handlers_lock);
    EnableInt(GPIO_INTNO, GPIO_INT_PRI);

    return E_OK;
}

/*** end of file ***/
