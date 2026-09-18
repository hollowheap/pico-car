#include <pico/time.h>
#include <hardware/gpio.h>

#include "lcg32.h"

#ifndef LED_DELAY_MS
#   define LED_DELAY_MS 10
#endif

#define PIN_COUNT 16

/*!
 * @brief Initialize and turn off the LEDs on the left side of the pico maker
 * board.
 */
static void pico_leds_init (void)
{
    for (uint8_t led = 0; led < PIN_COUNT; led++)
    {
        gpio_init(led);
        gpio_set_dir(led, GPIO_OUT);
        gpio_put(led, false);
    }
}

int main (void)
{
    pico_leds_init();

    const uint32_t rng_seed = 69420u;
    lcg32_t        rng      = {.state = rng_seed};

    for (;;)
    {
        uint led         = lcg32_next(&rng) % PIN_COUNT;
        bool b_is_led_on = lcg32_bool(&rng);
        gpio_put(led, b_is_led_on);

        sleep_ms(LED_DELAY_MS);
    }
}

/*** end of file ***/
