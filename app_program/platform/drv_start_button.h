/** @file drv_start_button.h
 * @brief GP20 operator START input, active low.
 */
#ifndef PLATFORM_DRV_START_BUTTON_H
#define PLATFORM_DRV_START_BUTTON_H

#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#define START_BUTTON_PIN       (20U)
#define START_BUTTON_PULL_UP   (1UL << 3U)
#define START_BUTTON_PULL_DOWN (1UL << 2U)
#define START_BUTTON_SCHMITT   (1UL << 1U)

static inline ER drv_start_button_init (void)
{
    ER const result = gpio_set_pin(START_BUTTON_PIN, GPIO_MODE_IN);

    if (E_OK == result)
    {
        clr_w(GPIO(START_BUTTON_PIN), START_BUTTON_PULL_DOWN);
        set_w(GPIO(START_BUTTON_PIN), START_BUTTON_PULL_UP | START_BUTTON_SCHMITT);
    }
    return result;
}

static inline bool drv_start_button_pressed (void)
{
    return (0U == gpio_get_val(START_BUTTON_PIN));
}

#endif /* PLATFORM_DRV_START_BUTTON_H */

/*** end of file ***/
