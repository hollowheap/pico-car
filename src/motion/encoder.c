/** @file encoder.c
 * @brief Short GPIO edge ISR and a protected diagnostic snapshot.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tk/smp.h>

#include "motion_pins.h"
#include "encoder.h"
#include "platform/isr.h"

/* RP2040 IO_BANK0 offsets, matching the pinned SDK's hardware/regs/io_bank0.h.
 * Use kernel MMIO accessors, not SDK IRQ registration: the kernel owns VTOR.
 */
#define BANK_INTR      (IO_BANK0_BASE + 0xF0U)
#define CORE0_INTS     (IO_BANK0_BASE + 0x120U)
#define TIMER_RAW_LOW  (TIMER_BASE + 0x28U)
#define EDGE_LOW_FLAG  0x4U
#define EDGE_HIGH_FLAG 0x8U
#define EDGE_FLAGS     (EDGE_LOW_FLAG | EDGE_HIGH_FLAG)

/* IO_BANK0 packs eight pins per register, four edge/level bits each; the
 * registers are one word apart. */
#define PINS_PER_BANK   8U
#define EDGE_FIELD_BITS 4U
#define BANK_STRIDE     4U

/* Quadrature channels (A and B) per wheel. */
#define ENCODER_CHANNELS 2U

/* Bit offset of the second wheel's A/B state in the combined state byte, and
 * of the first wheel's pending-IRQ flag in the metadata byte. */
#define WHEEL_STATE_SHIFT 2U
#define IRQ_FLAG_SHIFT    4U

static const unsigned int g_encoder_pins[ENCODER_WHEELS][ENCODER_CHANNELS] = {
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

static uint32_t edge_mask (unsigned int pin)
{
    return EDGE_FLAGS << ((pin % PINS_PER_BANK) * EDGE_FIELD_BITS);
}

static void ack_encoder_gpio ()
{
    for (uint8_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        UW bank = g_encoder_pins[wheel][0] / PINS_PER_BANK;
        out_w(BANK_INTR + (bank * BANK_STRIDE),
              edge_mask(g_encoder_pins[wheel][0]) |
                  edge_mask(g_encoder_pins[wheel][1]));
    }
}

/* Edge flags of one pin, in the layout of an IO_BANK0 INTS slot. */
static uint32_t pin_edge_flags (const gpio_edge_snapshot_t * p_snapshot,
                                unsigned int                 pin)
{
    uint32_t flags = 0U;
    if (((p_snapshot->fall_mask >> pin) & 1U) != 0U)
    {
        flags |= EDGE_LOW_FLAG;
    }
    if (((p_snapshot->rise_mask >> pin) & 1U) != 0U)
    {
        flags |= EDGE_HIGH_FLAG;
    }
    return flags;
}

static void encoder_isr (const gpio_edge_snapshot_t * p_snapshot)
{
    unsigned int wheel = 0U;
    UINT         saved = 0U;

    /* The dispatcher has already acknowledged the edges in the snapshot.
     * Edges after that remain pending for the next entry. */
    uint32_t gpio = in_w(GPIO_IN);
    (void)ISpinLock(&g_capture_lock, &saved);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        uint32_t a = pin_edge_flags(p_snapshot, g_encoder_pins[wheel][0]);
        uint32_t b = pin_edge_flags(p_snapshot, g_encoder_pins[wheel][1]);
        if ((a | b) != 0U)
        {
            /* Both channels changed, or both polarities on one pin: the
             * sequence is ambiguous. Record loss instead of guessing. */
            (void)quadrature_update(
                &g_captured, wheel, pin_state(gpio, wheel),
                ((a != 0U) && (b != 0U)) || (a == EDGE_FLAGS) ||
                    (b == EDGE_FLAGS));
        }
    }
    (void)ISpinUnlock(&g_capture_lock, saved);
}

ER encoder_init (void)
{
    UINT saved  = 0U;
    ER   result = E_OK;

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
        for (uint8_t channel = 0U; channel < ENCODER_CHANNELS; channel++)
        {
            unsigned int pin = g_encoder_pins[wheel][channel];
            (void)gpio_set_pin(pin, GPIO_MODE_IN);
            clr_w(GPIO(pin), MOTION_PAD_PULL_UP | MOTION_PAD_PULL_DOWN);
            set_w(GPIO(pin), MOTION_PAD_INPUT_ENABLE | MOTION_PAD_SCHMITT);
        }
    }

    uint32_t gpio_mask = 0;
    for (uint8_t wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        for (uint8_t channel = 0U; channel < ENCODER_CHANNELS; channel++)
        {
            unsigned int pin = g_encoder_pins[wheel][channel];
            gpio_mask |= 1U << pin;
        }
    }

    (void)InitSpinLock(&g_capture_lock);
    (void)ISpinLock(&g_capture_lock, &saved);
    ack_encoder_gpio();
    uint32_t gpio = in_w(GPIO_IN);
    quadrature_init(&g_captured,
                    (uint8_t)(pin_state(gpio, 0U) |
                              ((uint32_t)pin_state(gpio, 1U)
                               << WHEEL_STATE_SHIFT)));

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

ER encoder_read (encoder_sample_t * p_sample)
{
    UINT         saved    = 0U;
    unsigned int wheel    = 0U;
    uint32_t     gpio     = 0U;
    uint8_t      metadata = 0U;

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
    metadata = (uint8_t)(pin_state(gpio, 0U) |
                         ((uint32_t)pin_state(gpio, 1U) << WHEEL_STATE_SHIFT));
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        unsigned int bank      = g_encoder_pins[wheel][0] / PINS_PER_BANK;
        uint32_t     mask      = edge_mask(g_encoder_pins[wheel][0]) |
                                 edge_mask(g_encoder_pins[wheel][1]);
        p_sample->wheel[wheel] = g_captured.wheel[wheel];
        if ((in_w(CORE0_INTS + (bank * BANK_STRIDE)) & mask) != 0U)
        {
            metadata |= (uint8_t)(1U << (IRQ_FLAG_SHIFT + wheel));
        }
    }
    p_sample->captured_states = g_captured.states;
    p_sample->pins_and_irq    = metadata;
    p_sample->timestamp_us    = in_w(TIMER_RAW_LOW);
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

ER encoder_reset (void)
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
    quadrature_init(&g_captured,
                    (uint8_t)(pin_state(gpio, 0U) |
                              ((uint32_t)pin_state(gpio, 1U)
                               << WHEEL_STATE_SHIFT)));
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

/*** end of file ***/
