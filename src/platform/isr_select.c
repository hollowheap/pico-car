#include "platform/isr.h"

void isr_select_events (uint32_t rise, uint32_t fall, uint32_t pin_mask,
                        uint32_t            event_mask,
                        isr_gpio_event_t * p_event)
{
    if (p_event == 0)
    {
        return;
    }

    p_event->rise_pins = ((event_mask & ISR_GPIO_EDGE_RISE) != 0U)
                             ? (rise & pin_mask)
                             : 0U;
    p_event->fall_pins = ((event_mask & ISR_GPIO_EDGE_FALL) != 0U)
                             ? (fall & pin_mask)
                             : 0U;
    p_event->changed_pins = p_event->rise_pins | p_event->fall_pins;
}
