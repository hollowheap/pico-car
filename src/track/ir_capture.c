#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "platform/isr.h"
#include "track/track.h"
#include "ir_pins.h"

#define IO_IRQ_BANK0 13U
#define BARCODE_EDGE_RING 64U
#define BARCODE_LEVEL_MASK 1U

/**
 * Barcode edge ring, written by `ir_isr` and read by `ir_barcode_pop`, which
 * may run on the other core. Each entry holds the edge timestamp with the new
 * level in bit 0, so the timestamp resolution is 2 us.
 */
static volatile uint32_t g_barcode_edges[BARCODE_EDGE_RING];
static volatile uint32_t g_barcode_head;
static volatile uint32_t g_barcode_tail;
static volatile uint32_t g_barcode_drops;

/**
 * After the isr has been initialised, this can only be written to by `ir_isr`.
 */
LOCAL volatile ir_sensors_state_t g_state = {0};
LOCAL ID                          g_line_sem;
LOCAL ID                          g_barcode_sem;

/** Atomically loads `p_state`. */
LOCAL ir_sensors_state_t load_state (volatile ir_sensors_state_t * p_state)
{
    /* The state is only 1 byte, so a normal load is already atomic.
     * There are no memory barriers as we are not concerned about ordering. */
    return *p_state;
}

/** Atomically stores `p_state`. */
LOCAL void store_state (volatile ir_sensors_state_t * p_state,
                        ir_sensors_state_t            new_state)
{
    /* The state is only 1 byte, so a normal store is already atomic.
     * There are no memory barriers as we are not concerned about ordering. */
    *p_state = new_state;
}

/** Reads the current sensor state from GPIO. */
LOCAL ir_sensors_state_t current_state (void)
{
    uint32_t           gpio  = in_w(GPIO_IN);
    ir_sensors_state_t state = {0};
    state.barcode            = (gpio >> IR_BARCODE_PIN) & 1U;
    state.left               = (gpio >> IR_LEFT_PIN) & 1U;
    state.right              = (gpio >> IR_RIGHT_PIN) & 1U;
    return state;
}

LOCAL void ir_isr (const gpio_edge_snapshot_t * p_snapshot)
{
    ir_sensors_state_t old_state = load_state(&g_state);
    mp_memory_barrir();
    ir_sensors_state_t new_state = current_state();
    store_state(&g_state, new_state);

    if (old_state.barcode != new_state.barcode)
    {
        uint32_t const now  = p_snapshot->timestamp_us;
        uint32_t const next = (g_barcode_head + 1U) % BARCODE_EDGE_RING;

        if (next != g_barcode_tail)
        {
            g_barcode_edges[g_barcode_head] =
                (now & ~BARCODE_LEVEL_MASK) |
                (new_state.barcode ? BARCODE_LEVEL_MASK : 0U);
            mp_memory_barrir();
            g_barcode_head = next;
        }
        else
        {
            g_barcode_drops++;
        }

        (void)tk_sig_sem(g_barcode_sem, 1);
    }
    if ((old_state.left != new_state.left) ||
        (old_state.right != new_state.right))
    {
        (void)tk_sig_sem(g_line_sem, 1);
    }
}

LOCAL ER ir_sensor_init (UINT pin)
{
    return gpio_set_pin(pin, GPIO_MODE_IN);
}

EXPORT ER ir_capture_init (void)
{
    const T_CSEM sem_def = {
        .exinf   = NULL,
        .sematr  = TA_TPRI | TA_FIRST,
        .isemcnt = 0,
        .maxsem  = 1,
    };

    ER result = tk_cre_sem(&sem_def);
    if (result < 0)
    {
        return result;
    }
    g_line_sem = (ID)result;

    result = tk_cre_sem(&sem_def);
    if (result < 0)
    {
        return result;
    }
    g_barcode_sem = (ID)result;

    (void)ir_sensor_init(IR_BARCODE_PIN);
    (void)ir_sensor_init(IR_LEFT_PIN);
    (void)ir_sensor_init(IR_RIGHT_PIN);
    store_state(&g_state, current_state());

    return gpio_irq_add_handler(tk_get_prc(),
                                (1U << IR_BARCODE_PIN) | (1U << IR_LEFT_PIN) |
                                    (1U << IR_RIGHT_PIN),
                                ir_isr);
}

EXPORT ir_sensors_state_t ir_get_state (void)
{
    return load_state(&g_state);
}

EXPORT bool ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level)
{
    bool b_popped = false;

    if ((NULL != p_timestamp_us) && (NULL != p_level))
    {
        uint32_t const tail = g_barcode_tail;
        uint32_t const head = g_barcode_head;

        /* Read the head before the entry it publishes. */
        mp_memory_barrir();
        if (tail != head)
        {
            uint32_t const entry = g_barcode_edges[tail];

            *p_timestamp_us = entry & ~BARCODE_LEVEL_MASK;
            *p_level        = (0U != (entry & BARCODE_LEVEL_MASK));
            /* Finish reading the entry before the ISR may reuse its slot. */
            mp_memory_barrir();
            g_barcode_tail = (tail + 1U) % BARCODE_EDGE_RING;
            b_popped       = true;
        }
    }

    return (b_popped);
}

EXPORT uint32_t ir_barcode_drops (void)
{
    return (g_barcode_drops);
}

EXPORT ER ir_wait_change (uint8_t type, TMO tmout)
{
    switch (type)
    {
        case IR_TYPE_BARCODE:
            return tk_wai_sem(g_barcode_sem, 1, tmout);
        case IR_TYPE_LINE:
            return tk_wai_sem(g_line_sem, 1, tmout);
        default:
#ifndef NDEBUG
            tm_printf((UB *)"Bad type param in ir_wait_change: %d\n", type);
#endif
            return E_PAR;
    }
}

/*** end of file ***/
