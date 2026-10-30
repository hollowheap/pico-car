#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "platform/isr.h"
#include "track/ir_capture.h"
#include "ir_pins.h"

/**
 * After the isr has been initialised, this can only be written to by `ir_isr`.
 */
LOCAL volatile ir_sensors_state_t g_state = {0};

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

LOCAL void ir_isr (const isr_gpio_event_t * p_event, void * p_user)
{
    (void)p_event;
    (void)p_user;
    g_state = current_state();
}

LOCAL ER ir_sensor_init (UINT pin)
{
    return gpio_set_pin(pin, GPIO_MODE_IN);
}

EXPORT ER ir_capture_init (void)
{
    (void)ir_sensor_init(IR_BARCODE_PIN);
    (void)ir_sensor_init(IR_LEFT_PIN);
    (void)ir_sensor_init(IR_RIGHT_PIN);
    g_state = current_state();

    return isr_register_gpio_handler(
        (1U << IR_BARCODE_PIN) | (1U << IR_LEFT_PIN) | (1U << IR_RIGHT_PIN),
        ISR_GPIO_EDGE_ANY, ir_isr, NULL);
}

EXPORT ir_sensors_state_t ir_get_state (void)
{
    return g_state;
}

/*** end of file ***/
