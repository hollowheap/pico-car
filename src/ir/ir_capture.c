#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../sys/gpio_irq.h"
#include "ir_capture.h"
#include "ir_pins.h"

#define IO_IRQ_BANK0 13U

/**
 * After the isr has been initialised, this can only be written to by `ir_isr`.
 */
LOCAL volatile ir_sensors_state_t g_state = {0};
LOCAL ID                          g_line_sem;
LOCAL ID                          g_barcode_sem;

/** Atomically loads `p_state`. */
LOCAL ir_sensors_state_t load_state (volatile ir_sensors_state_t * p_state)
{
    // The state is only 1 byte, so a normal load is already atomic.
    // There are no memory barriers as we are not concerned about ordering.
    return *p_state;
}

/** Atomically stores `p_state`. */
LOCAL void store_state (volatile ir_sensors_state_t * p_state,
                        ir_sensors_state_t            new_state)
{
    // The state is only 1 byte, so a normal store is already atomic.
    // There are no memory barriers as we are not concerned about ordering.
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

LOCAL void ir_isr (UINT intno)
{
    (void)intno;

    ir_sensors_state_t old_state = load_state(&g_state);
    mp_memory_barrir();
    ir_sensors_state_t new_state = current_state();
    store_state(&g_state, new_state);

    if (old_state.barcode != new_state.barcode)
    {
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
            tm_printf((UB *)"Bad type param in ir_wait_change: %d\n", sensor);
#endif
            return E_PAR;
    }
}

/*** end of file ***/
