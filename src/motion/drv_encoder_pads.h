/** @file drv_encoder_pads.h
 * @brief Pad setup for the two PIO quadrature encoders.
 */
#ifndef MOTION_DRV_ENCODER_PADS_H
#define MOTION_DRV_ENCODER_PADS_H

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "motion_pins.h"
#include "quadrature.h"
#include "platform/static_assert.h"

#define ENCODER_CHANNELS (2U)

STATIC_ASSERT((MOTOR_ENCODER_LEFT_A + 1U == MOTOR_ENCODER_LEFT_B) &&
                  (MOTOR_ENCODER_RIGHT_A + 1U == MOTOR_ENCODER_RIGHT_B),
              "the state machine reads B from the pin after A");

static inline void drv_encoder_pads_init (const unsigned int pins_a[ENCODER_WHEELS])
{
    for (uint8_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        for (uint8_t channel = 0U; channel < ENCODER_CHANNELS; channel++)
        {
            unsigned int const pin = pins_a[wheel] + channel;
            (void)gpio_set_pin(pin, GPIO_MODE_IN);
            /* A floating open-collector or disconnected pin can count PWM
             * noise as edges. */
            clr_w(GPIO(pin), MOTION_PAD_PULL_DOWN);
            set_w(GPIO(pin), MOTION_PAD_PULL_UP | MOTION_PAD_INPUT_ENABLE | MOTION_PAD_SCHMITT);
        }
    }
}

#endif /* MOTION_DRV_ENCODER_PADS_H */

/*** end of file ***/
