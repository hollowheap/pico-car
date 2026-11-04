/** @file encoder_capture.c
 * @brief Encoder edge ISR and protected count snapshot.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tk/smp.h>

#include "motion_pins.h"
#include "encoder_capture.h"
#include "platform/isr.h"

/* RP2040 IO_BANK0 timer offset, matching the pinned SDK's io_bank0.h. Use
 * kernel MMIO accessors, not SDK IRQ registration: the kernel owns VTOR.
 */
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)

static const unsigned int g_encoder_pins[ENCODER_WHEELS][2] = {
    {MOTOR_ENCODER_LEFT_A, MOTOR_ENCODER_LEFT_B},
    {MOTOR_ENCODER_RIGHT_A, MOTOR_ENCODER_RIGHT_B},
};

static T_SPLOCK       g_capture_lock;
static quadrature_t g_captured;
/* Lifecycle belongs to the initialisation task, before any readers start. */
static bool g_initialised = false;

static uint8_t pin_state (uint32_t gpio, unsigned int wheel)
{
    return (uint8_t)((((gpio >> g_encoder_pins[wheel][0]) & 1U) << 1U) |
                     ((gpio >> g_encoder_pins[wheel][1]) & 1U));
}

static void encoder_isr (const isr_gpio_event_t * p_event, void * p_user)
{
    uint32_t     gpio;
    unsigned int wheel;
    UINT         saved;

    (void)p_user;
    if (p_event == NULL)
    {
        return;
    }

    gpio = in_w(GPIO_IN);
    (void)ISpinLock(&g_capture_lock, &saved);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        const uint32_t pin_a =
            UINT32_C(1) << g_encoder_pins[wheel][0];
        const uint32_t pin_b =
            UINT32_C(1) << g_encoder_pins[wheel][1];
        const uint32_t wheel_mask = pin_a | pin_b;
        const uint32_t changed    = p_event->changed_pins & wheel_mask;
        const uint32_t rise       = p_event->rise_pins & wheel_mask;
        const uint32_t fall       = p_event->fall_pins & wheel_mask;

        if (changed != 0U)
        {
            /* Both channels changed, or both polarities on one pin: the
             * sequence is ambiguous. Record loss instead of guessing. */
            const bool ambiguous =
                ((changed & pin_a) != 0U && (changed & pin_b) != 0U) ||
                ((rise & fall) != 0U);
            (void)quadrature_update(&g_captured, wheel,
                                      pin_state(gpio, wheel), ambiguous);
        }
    }
    (void)ISpinUnlock(&g_capture_lock, saved);
}

ER encoder_capture_init (void)
{
    ER result;

    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }
    if (g_initialised)
    {
        return E_OBJ;
    }

    for (uint8_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        for (uint8_t channel = 0U; channel < 2U; channel++)
        {
            unsigned int pin = g_encoder_pins[wheel][channel];
            (void)gpio_set_pin(pin, GPIO_MODE_IN);
            clr_w(GPIO(pin), GPIO_PUE | GPIO_PDE);
            set_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT);
        }
    }

    uint32_t gpio_mask = 0;
    for (uint8_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        for (uint8_t channel = 0U; channel < 2U; channel++)
        {
            unsigned int pin = g_encoder_pins[wheel][channel];
            gpio_mask |= 1U << pin;
        }
    }

    (void)InitSpinLock(&g_capture_lock);
    uint32_t gpio = in_w(GPIO_IN);
    quadrature_init(&g_captured, (uint8_t)(pin_state(gpio, 0U) |
                                             (pin_state(gpio, 1U) << 2U)));

    result = isr_register_gpio_handler(gpio_mask, ISR_GPIO_EDGE_ANY,
                                       encoder_isr, NULL);
    if (result != E_OK)
    {
        return result;
    }

    g_initialised = true;
    return E_OK;
}

ER encoder_capture_read (encoder_capture_sample_t * p_sample)
{
    UINT         saved;
    unsigned int wheel;

    if (p_sample == NULL)
    {
        return E_PAR;
    }
    /* Initialisation must finish before consumers are started. */
    if (!g_initialised)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&g_capture_lock, &saved);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        p_sample->wheel[wheel] = g_captured.wheel[wheel];
    }
    p_sample->timestamp_us = in_w(TIMER_RAW_LOW);
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

/*** end of file ***/
