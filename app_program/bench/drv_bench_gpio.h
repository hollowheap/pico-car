/** @file drv_bench_gpio.h
 * @brief GPIO operations for standalone bring-up and measurement images.
 */
#ifndef BENCH_DRV_GPIO_H
#define BENCH_DRV_GPIO_H

#include <stdbool.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#define BENCH_PAD_PULL_UP   (1UL << 3U)
#define BENCH_PAD_PULL_DOWN (1UL << 2U)

static inline void drv_bench_gpio_input (UINT pin)
{
    (void)gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), BENCH_PAD_PULL_DOWN);
    set_w(GPIO(pin), BENCH_PAD_PULL_UP);
}

static inline UINT drv_bench_gpio_level (UINT pin)
{
    return gpio_get_val(pin);
}

static inline bool drv_bench_gpio_pressed (UINT pin)
{
    return (0U == drv_bench_gpio_level(pin));
}

static inline void drv_bench_gpio_drive (UINT pin, UINT level)
{
    (void)gpio_set_val(pin, level);
    (void)gpio_set_pin(pin, GPIO_MODE_OUT);
}

#endif /* BENCH_DRV_GPIO_H */

/*** end of file ***/
