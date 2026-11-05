/** @file encoder_capture.c
 * @brief Short GPIO edge ISR and a protected diagnostic snapshot.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tk/smp.h>

#include "motion_pins.h"
#include "encoder_capture.h"
#include "../sys/gpio_irq.h"

/* RP2040 IO_BANK0 offsets, matching the pinned SDK's hardware/regs/io_bank0.h.
 * Use kernel MMIO accessors, not SDK IRQ registration: the kernel owns VTOR.
 */
#define BANK_INTR     (IO_BANK0_BASE + 0xF0U)
#define CORE0_INTS    (IO_BANK0_BASE + 0x120U)
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)
#define EDGE_FLAGS    12U

static const unsigned int g_encoder_pins[ENCODER_WHEELS][2] = {
    {MOTOR_ENCODER_LEFT_A, MOTOR_ENCODER_LEFT_B},
    {MOTOR_ENCODER_RIGHT_A, MOTOR_ENCODER_RIGHT_B},
};

// TODO: add 'volatile' for global variables written to by the interrupt
static T_SPLOCK       g_capture_lock;
static encoder_quad_t g_captured;
/* Lifecycle belongs to the initialisation task, before any readers start. */
static bool g_initialised = false;

static uint8_t pin_state (uint32_t gpio, unsigned int wheel)
{
    return (uint8_t)((((gpio >> g_encoder_pins[wheel][0]) & 1U) << 1U) |
                     ((gpio >> g_encoder_pins[wheel][1]) & 1U));
}

static uint32_t edge_mask (unsigned int pin)
{
    return EDGE_FLAGS << ((pin % 8U) * 4U);
}

static void ack_encoder_gpio ()
{
    for (uint8_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        UW bank = g_encoder_pins[wheel][0] / 8U;
        out_w(BANK_INTR + (bank * 4U), edge_mask(g_encoder_pins[wheel][0]) |
                                           edge_mask(g_encoder_pins[wheel][1]));
    }
}

static void encoder_isr (UINT intno)
{
    uint32_t     pending[ENCODER_WHEELS];
    unsigned int wheel;
    UINT         saved;

    (void)intno;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        unsigned int bank = g_encoder_pins[wheel][0] / 8U;
        uint32_t     mask = edge_mask(g_encoder_pins[wheel][0]) |
                            edge_mask(g_encoder_pins[wheel][1]);
        pending[wheel]    = in_w(CORE0_INTS + (bank * 4U)) & mask;
    }

    /* Edges after the acknowledgement remain pending for the next entry. */
    uint32_t gpio = in_w(GPIO_IN);
    (void)ISpinLock(&g_capture_lock, &saved);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (pending[wheel] != 0U)
        {
            uint32_t a =
                (pending[wheel] >> ((g_encoder_pins[wheel][0] % 8U) * 4U)) &
                EDGE_FLAGS;
            uint32_t b =
                (pending[wheel] >> ((g_encoder_pins[wheel][1] % 8U) * 4U)) &
                EDGE_FLAGS;
            /* Both channels changed, or both polarities on one pin: the
             * sequence is ambiguous. Record loss instead of guessing. */
            (void)encoder_quad_update(
                &g_captured, wheel, pin_state(gpio, wheel),
                ((a != 0U) && (b != 0U)) || (a == EDGE_FLAGS) ||
                    (b == EDGE_FLAGS));
        }
    }
    (void)ISpinUnlock(&g_capture_lock, saved);
}

ER encoder_capture_init (void)
{
    UINT saved;
    ER   result;

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
    (void)ISpinLock(&g_capture_lock, &saved);
    ack_encoder_gpio();
    uint32_t gpio = in_w(GPIO_IN);
    encoder_quad_init(&g_captured, (uint8_t)(pin_state(gpio, 0U) |
                                             (pin_state(gpio, 1U) << 2U)));

    result = gpio_irq_add_handler(TP_PRC1, gpio_mask, encoder_isr);
    if (result != E_OK)
    {
        (void)ISpinUnlock(&g_capture_lock, saved);
        return result;
    }

    g_initialised = true;
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

ER encoder_capture_read (encoder_capture_sample_t * p_sample)
{
    UINT         saved;
    unsigned int wheel;
    uint32_t     gpio;
    uint8_t      metadata;

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
    gpio     = in_w(GPIO_IN);
    metadata = (uint8_t)(pin_state(gpio, 0U) | (pin_state(gpio, 1U) << 2U));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        unsigned int bank      = g_encoder_pins[wheel][0] / 8U;
        uint32_t     mask      = edge_mask(g_encoder_pins[wheel][0]) |
                                 edge_mask(g_encoder_pins[wheel][1]);
        p_sample->wheel[wheel] = g_captured.wheel[wheel];
        if ((in_w(CORE0_INTS + (bank * 4U)) & mask) != 0U)
        {
            metadata |= (uint8_t)(1U << (4U + wheel));
        }
    }
    p_sample->captured_states = g_captured.states;
    p_sample->pins_and_irq    = metadata;
    p_sample->timestamp_us    = in_w(TIMER_RAW_LOW);
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

ER encoder_capture_reset (void)
{
    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }
    if (!g_initialised)
    {
        return E_OBJ;
    }

    UINT saved = 0;
    (void)ISpinLock(&g_capture_lock, &saved);
    ack_encoder_gpio();
    uint32_t gpio = in_w(GPIO_IN);
    encoder_quad_init(&g_captured, (uint8_t)(pin_state(gpio, 0U) |
                                             (pin_state(gpio, 1U) << 2U)));
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

/*** end of file ***/
