/** @file drv_stop.h
 * @brief GP21 emergency STOP input, active low.
 */
#ifndef MOTION_DRV_STOP_H
#define MOTION_DRV_STOP_H

#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "motion_pins.h"

#define DRV_STOP_PIN      (21U)
#define DRV_STOP_PIN_MASK (1U << DRV_STOP_PIN)

static inline void drv_stop_init (void)
{
    (void)gpio_set_pin(DRV_STOP_PIN, GPIO_MODE_IN);
    clr_w(GPIO(DRV_STOP_PIN), MOTION_PAD_PULL_DOWN);
    set_w(GPIO(DRV_STOP_PIN), MOTION_PAD_PULL_UP | MOTION_PAD_SCHMITT);
}

static inline bool drv_stop_pressed (void)
{
    return (0U == gpio_get_val(DRV_STOP_PIN));
}

#endif /* MOTION_DRV_STOP_H */

/*** end of file ***/
