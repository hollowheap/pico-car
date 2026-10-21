/** @file encoder_capture.c
 * @brief Short GPIO edge ISR and a protected diagnostic snapshot.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tk/smp.h>
#include "buddy2_pins.h"
#include "encoder_capture.h"

/* RP2040 IO_BANK0 offsets, matching the pinned SDK's hardware/regs/io_bank0.h.
 * Use kernel MMIO accessors, not SDK IRQ registration: the kernel owns VTOR.
 */
#define ENCODER_IRQ 13U
#define BANK_INTR  (IO_BANK0_BASE + 0xF0U)
#define CORE0_INTE (IO_BANK0_BASE + 0x100U)
#define CORE0_INTS (IO_BANK0_BASE + 0x120U)
#define CORE1_INTE (IO_BANK0_BASE + 0x130U)
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)
#define EDGE_FLAGS 12U

static const unsigned int encoder_pins[ENCODER_WHEELS][2] = {
    {BUDDY2_ENCODER_LEFT_A, BUDDY2_ENCODER_LEFT_B},
    {BUDDY2_ENCODER_RIGHT_A, BUDDY2_ENCODER_RIGHT_B}
};

static T_SPLOCK capture_lock;
static encoder_quad_t captured;
/* Lifecycle belongs to the initialisation task, before any readers start. */
static bool initialised;

static uint8_t pin_state (uint32_t gpio, unsigned int wheel)
{
    return (uint8_t)((((gpio >> encoder_pins[wheel][0]) & 1U) << 1U) |
                     ((gpio >> encoder_pins[wheel][1]) & 1U));
}

static uint32_t edge_mask (unsigned int pin)
{
    return EDGE_FLAGS << ((pin % 8U) * 4U);
}

static void encoder_interrupt (UINT intno)
{
    uint32_t pending[ENCODER_WHEELS];
    uint32_t a;
    uint32_t b;
    uint32_t gpio;
    unsigned int wheel;
    UINT saved;

    (void)intno;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        unsigned int bank = encoder_pins[wheel][0] / 8U;
        uint32_t mask = edge_mask(encoder_pins[wheel][0]) |
                        edge_mask(encoder_pins[wheel][1]);
        pending[wheel] = in_w(CORE0_INTS + (bank * 4U)) & mask;
        /* W1C only our observed edge flags; never clear unrelated GPIOs. */
        out_w(BANK_INTR + (bank * 4U), pending[wheel]);
    }
    /* Edges after the acknowledgement remain pending for the next entry. */
    gpio = in_w(GPIO_IN);
    (void)ISpinLock(&capture_lock, &saved);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (pending[wheel] != 0U)
        {
            a = (pending[wheel] >>
                 ((encoder_pins[wheel][0] % 8U) * 4U)) & EDGE_FLAGS;
            b = (pending[wheel] >>
                 ((encoder_pins[wheel][1] % 8U) * 4U)) & EDGE_FLAGS;
            /* Both channels changed, or both polarities on one pin: the
             * sequence is ambiguous. Record loss instead of guessing. */
            (void)encoder_quad_update(&captured, wheel,
                pin_state(gpio, wheel),
                ((a != 0U) && (b != 0U)) ||
                (a == EDGE_FLAGS) || (b == EDGE_FLAGS));
        }
    }
    (void)ISpinUnlock(&capture_lock, saved);
}

ER encoder_capture_init (void)
{
    T_DINT definition = {.intatr = TA_HLNG, .inthdr = (FP)encoder_interrupt};
    unsigned int bank;
    unsigned int wheel;
    unsigned int channel;
    uint32_t gpio;
    UINT saved;
    ER result;

    if (tk_get_prc() != TP_PRC1)
    {
        return E_CTX;
    }
    if (initialised)
    {
        return E_OBJ;
    }
    for (bank = 0U; bank < 4U; bank++)
    {
        if ((in_w(CORE0_INTE + (bank * 4U)) != 0U) ||
            (in_w(CORE1_INTE + (bank * 4U)) != 0U))
        {
            return E_BUSY;
        }
    }
    DisableInt(ENCODER_IRQ);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        for (channel = 0U; channel < 2U; channel++)
        {
            unsigned int pin = encoder_pins[wheel][channel];
            (void)gpio_set_pin(pin, GPIO_MODE_IN);
            clr_w(GPIO(pin), GPIO_PUE | GPIO_PDE);
            set_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT);
        }
    }
    result = tk_def_int(ENCODER_IRQ, &definition);
    if (result != E_OK)
    {
        return result;
    }
    (void)InitSpinLock(&capture_lock);
    (void)ISpinLock(&capture_lock, &saved);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        bank = encoder_pins[wheel][0] / 8U;
        out_w(BANK_INTR + (bank * 4U),
              edge_mask(encoder_pins[wheel][0]) |
              edge_mask(encoder_pins[wheel][1]));
    }
    gpio = in_w(GPIO_IN);
    encoder_quad_init(&captured,
        (uint8_t)(pin_state(gpio, 0U) | (pin_state(gpio, 1U) << 2U)));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        bank = encoder_pins[wheel][0] / 8U;
        set_w(CORE0_INTE + (bank * 4U),
              edge_mask(encoder_pins[wheel][0]) |
              edge_mask(encoder_pins[wheel][1]));
    }
    initialised = true;
    (void)ISpinUnlock(&capture_lock, saved);
    ClearInt(ENCODER_IRQ);
    EnableInt(ENCODER_IRQ, 2);
    return E_OK;
}

ER encoder_capture_read (encoder_capture_sample_t * p_sample)
{
    UINT saved;
    unsigned int wheel;
    uint32_t gpio;
    uint8_t metadata;

    if (p_sample == NULL)
    {
        return E_PAR;
    }
    /* Initialisation must finish before consumers are started. */
    if (!initialised)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&capture_lock, &saved);
    gpio = in_w(GPIO_IN);
    metadata = (uint8_t)(pin_state(gpio, 0U) |
                         (pin_state(gpio, 1U) << 2U));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        unsigned int bank = encoder_pins[wheel][0] / 8U;
        uint32_t mask = edge_mask(encoder_pins[wheel][0]) |
                        edge_mask(encoder_pins[wheel][1]);
        p_sample->wheel[wheel] = captured.wheel[wheel];
        if ((in_w(CORE0_INTS + (bank * 4U)) & mask) != 0U)
        {
            metadata |= (uint8_t)(1U << (4U + wheel));
        }
        if (((in_w(CORE0_INTE + (bank * 4U)) & mask) == mask) &&
            ((in_w(NVIC_ISER(ENCODER_IRQ)) & (1U << ENCODER_IRQ)) != 0U))
        {
            metadata |= (uint8_t)(1U << (6U + wheel));
        }
    }
    p_sample->captured_states = captured.states;
    p_sample->pins_and_irq = metadata;
    p_sample->timestamp_us = in_w(TIMER_RAW_LOW);
    (void)ISpinUnlock(&capture_lock, saved);
    return E_OK;
}

/*** end of file ***/
