#include <stdio.h>

#include "platform/isr.h"

static unsigned int failures;

static void check (int condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

int main (void)
{
    isr_gpio_event_t event;

    isr_select_events(1U << 16, 1U << 4, (1U << 16) | (1U << 4),
                      ISR_GPIO_EDGE_ANY, &event);
    check(event.rise_pins == (1U << 16), "high pin rise");
    check(event.fall_pins == (1U << 4), "low pin fall");
    check(event.changed_pins == ((1U << 16) | (1U << 4)), "changed pins");

    isr_select_events(1U << 2, 1U << 3, 0x0FU, ISR_GPIO_EDGE_RISE, &event);
    check(event.rise_pins == (1U << 2), "rise only");
    check(event.fall_pins == 0U, "fall filtered");
    check(event.changed_pins == (1U << 2), "changed rise only");

    isr_select_events(1U << 5, 1U << 5, 1U << 5, ISR_GPIO_EDGE_FALL, &event);
    check(event.rise_pins == 0U, "rise filtered");
    check(event.fall_pins == (1U << 5), "fall selected");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("isr select OK\n");
    return 0;
}
