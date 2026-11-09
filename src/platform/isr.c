#include <stdint.h>
#include <tk/tkernel.h>

#include "platform/isr.h"

#define GPIO_INTNO     13
#define GPIO_INT_PRI   2
#define GPIO_IRQ_COUNT 30U
#define ALL_GPIO_MASK  ((1U << GPIO_IRQ_COUNT) - 1U)

#define INT_REG_COUNT 4U
#define BITS_PER_BYTE 8U
#define NIBBLE_BITS   4U
#define TIMER_RAWL    (TIMER_BASE + 0x28U)
#define INTR          (IO_BANK0_BASE + 0xF0U)
#define PROC1_INTE    (IO_BANK0_BASE + 0x100U)
#define PROC1_INTS    (IO_BANK0_BASE + 0x120U)
#define PROC2_INTE    (IO_BANK0_BASE + 0x130U)
#define PROC2_INTS    (IO_BANK0_BASE + 0x150U)

/* Per GPIO, INTR and INTS hold LEVEL_LOW, LEVEL_HIGH, EDGE_LOW, EDGE_HIGH in
 * bits 0..3 of a 4-bit slot. EDGE_LOW is a falling edge, EDGE_HIGH a rising
 * edge. Only the edge bits are write-1-to-clear in INTR. */
#define EDGE_LOW_BITS  0x44444444U
#define EDGE_HIGH_BITS 0x88888888U
#define EDGE_BITS      (EDGE_LOW_BITS | EDGE_HIGH_BITS)

/* Bit selection masks and shifts used by `unspread4`. */
#define UNSPREAD_HI_MASK   0x8800U
#define UNSPREAD_LO_MASK   0x0088U
#define UNSPREAD_HI_SHIFT  8U
#define UNSPREAD_LO_SHIFT  2U
#define UNSPREAD2_HI_MASK  0xA0U
#define UNSPREAD2_LO_MASK  0x0AU
#define UNSPREAD2_HI_SHIFT 4U
#define UNSPREAD2_LO_SHIFT 1U

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

LOCAL T_SPLOCK           g_handlers_lock;
LOCAL volatile handler_t g_handlers[MAX_GPIO_IRQ_HANDLERS] = {0};
/**
 * This count shall only increase, and only increase when the `g_handler` has
 * be written. This constraint means that the ISR can read the handlers safely
 * without a lock.
 */
LOCAL volatile uint8_t g_handler_count = 0;

/**
 * Selects every 4th bit in a number and returns the result as a 4-bit int.
 * Example: 0b0000 1000 1000 0111 => 0b0110
 */
LOCAL inline uint8_t unspread4(const uint16_t x)
{
    /* x = 0b 1xxx 2xxx 3xxx 4xxx */

    /* 0b 1000 2000
     * 0b   30 0040
     * 0b 1030 2040 */
    uint32_t hi       = (uint32_t)x & UNSPREAD_HI_MASK;
    uint32_t lo       = (uint32_t)x & UNSPREAD_LO_MASK;
    uint32_t unspread = (hi >> UNSPREAD_HI_SHIFT) | (lo >> UNSPREAD_LO_SHIFT);
    /* 0b 1030
     * 0b  204
     * 0b 1234 */
    hi       = unspread & UNSPREAD2_HI_MASK;
    lo       = unspread & UNSPREAD2_LO_MASK;
    unspread = (hi >> UNSPREAD2_HI_SHIFT) | (lo >> UNSPREAD2_LO_SHIFT);
    return (uint8_t)unspread;
}

/**
 * Expands `gpio_mask` into the EDGE_LOW and EDGE_HIGH bits for each GPIO which
 * has its corresponding bit set. The bits are written to `enable_masks`.
 */
LOCAL void get_edge_masks (const uint32_t gpio_mask,
                           uint32_t       enable_masks[INT_REG_COUNT])
{
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        const uint32_t mask_byte = (gpio_mask >> (idx * BITS_PER_BYTE)) & 0xFFU;
        const uint32_t m_lo      = mask_byte & 0xFU;
        const uint32_t m_hi      = mask_byte >> NIBBLE_BITS;

        /* Spread each bit to every 3rd and 4th bit. These correspond to the
         * EDGE_LOW and EDGE_HIGH bits in the registers.
         * As each bit is in a 4 bit slot, we can only spread 4 bits at a time.
         * Example: 0b0110 => 0b0000 1100 1100 0000
         * Based on: https://stackoverflow.com/a/79404703 */
        const uint32_t enable_lo =
            (((m_lo & 0x5U) * 0x924U) | ((m_lo & 0xAU) * 0x924U)) & 0x4444U;
        const uint32_t enable_hi =
            (((m_hi & 0x5U) * 0x924U) | ((m_hi & 0xAU) * 0x924U)) & 0x4444U;
        const uint32_t enable = (enable_hi << 16U) | enable_lo;
        enable_masks[idx]     = enable | (enable << 1U);
    }
}

/**
 * Packs the bit 3 of every 4-bit slot of `w` into one GPIO bit per slot, so
 * bit n of the result is slot n of the register word.
 */
LOCAL uint32_t gather_slots (const uint32_t w)
{
    const uint32_t lo = unspread4((uint16_t)w);
    const uint32_t hi = unspread4((uint16_t)(w >> 16U));
    return (hi << NIBBLE_BITS) | lo;
}

/**
 * Reads the interrupt status of `proc` once. `edge_words` receives the EDGE
 * bits that are set, in INTR layout. `rise_mask` and `fall_mask` receive one
 * bit per GPIO for the rising and falling edges.
 */
LOCAL void read_edges (const UW proc, uint32_t edge_words[INT_REG_COUNT],
                       uint32_t * const p_rise_mask,
                       uint32_t * const p_fall_mask)
{
    const UW status_base = (proc == TP_PRC1) ? PROC1_INTS : PROC2_INTS;
    uint32_t rise        = 0;
    uint32_t fall        = 0;
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        const uint32_t status_w = in_w(status_base + (idx * sizeof(UW)));
        edge_words[idx]         = status_w & EDGE_BITS;
        /* EDGE_LOW is bit 2 of a slot; shift it up to bit 3 to share the
         * gather with EDGE_HIGH. */
        rise |= gather_slots(status_w & EDGE_HIGH_BITS)
                << (idx * BITS_PER_BYTE);
        fall |= gather_slots((status_w & EDGE_LOW_BITS) << 1U)
                << (idx * BITS_PER_BYTE);
    }
    *p_rise_mask = rise;
    *p_fall_mask = fall;
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
            /* The other processor already owns this pin; we cannot claim it */
            return E_OBJ;
        }
    }
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        set_w(enable_base + (idx * sizeof(UW)), enable_masks[idx]);
    }
    return E_OK;
}

LOCAL void ack_gpio_irqs (const uint32_t ack_words[INT_REG_COUNT])
{
    for (uint8_t idx = 0; idx < INT_REG_COUNT; idx++)
    {
        out_w(INTR + (idx * sizeof(UW)), ack_words[idx]);
    }
}

/**
 * Takes one snapshot of the timer and the edge status, acknowledges exactly
 * the edges in it, and only then calls the handlers. An edge that arrives
 * during a handler is not in the snapshot and was not acknowledged, so it stays
 * pending and enters this ISR again.
 */
LOCAL void gpio_isr (UINT intno)
{
    uint32_t edge_words[INT_REG_COUNT];
    uint32_t rise_mask = 0U;
    uint32_t fall_mask = 0U;

    (void)intno;
    read_edges(THIS_PRC(), edge_words, &rise_mask, &fall_mask);
    if ((rise_mask | fall_mask) == 0U)
    {
        return;
    }

    /* Acknowledge right after the status read to keep the window short in
     * which a repeat of an observed edge would be cleared unseen. */
    ack_gpio_irqs(edge_words);

    /* Read after the status, so the timestamp is not earlier than any edge in
     * the snapshot. */
    const uint32_t timestamp_us = in_w(TIMER_RAWL);

    for (uint8_t idx = 0; idx < g_handler_count; idx++)
    {
        const uint32_t mask = g_handlers[idx].mask;
        const gpio_edge_snapshot_t snapshot = {
            .timestamp_us = timestamp_us,
            .rise_mask    = rise_mask & mask,
            .fall_mask    = fall_mask & mask,
        };
        if ((snapshot.rise_mask | snapshot.fall_mask) != 0U)
        {
            g_handlers[idx].handler(&snapshot);
        }
    }
}

/* TODO: set interrupt on both cores */
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
    ClearInt(GPIO_INTNO);
    EnableInt(GPIO_INTNO, GPIO_INT_PRI);
    (void)InitSpinLock(&g_handlers_lock);
    return E_OK;
}

EXPORT ER gpio_irq_add_handler (UW proc, uint32_t gpio_mask,
                                gpio_irq_handler_t handler)
{
    if (g_handler_count >= MAX_GPIO_IRQ_HANDLERS)
    {
        return E_LIMIT;
    }

    gpio_mask &= ALL_GPIO_MASK;
    uint32_t edge_masks[INT_REG_COUNT];
    get_edge_masks(gpio_mask, edge_masks);

    (void)SpinLock(&g_handlers_lock);
    /* We must check again as someone else could have added a handler before we
     * aqquired the lock. */
    if (g_handler_count >= MAX_GPIO_IRQ_HANDLERS)
    {
        (void)SpinUnlock(&g_handlers_lock);
        return E_LIMIT;
    }

    g_handlers[g_handler_count] = (handler_t){
        .handler = handler,
        .mask    = gpio_mask,
    };
    /* Disable interrupts before enabling interrupts for the pin.
     * Otherwise, the interrupt may fire immediately and `gpio_isr` will not
     * clear it as the mask has not been commited to `g_handlers`. */
    DisableInt(GPIO_INTNO);
    const ER result = enable_pin_interrupts(proc, edge_masks);
    if (result != E_OK)
    {
        (void)SpinUnlock(&g_handlers_lock);
        return result;
    }

    /* Count can only be incremented after setting the handler as the ISR does
     * not use the spinlock. */
    mp_memory_barrir();
    g_handler_count++;

    (void)SpinUnlock(&g_handlers_lock);
    EnableInt(GPIO_INTNO, GPIO_INT_PRI);

    return E_OK;
}

/*** end of file ***/
