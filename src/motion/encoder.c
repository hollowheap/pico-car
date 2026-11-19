/** @file encoder.c
 * @brief Encoder pads, the PIO quadrature counters and a protected snapshot.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tk/smp.h>

#include "motion_pins.h"
#include "encoder.h"
#include "encoder_pio.h"
#include "platform/static_assert.h"


/* Quadrature channels (A and B) per wheel. */
#define ENCODER_CHANNELS 2U

STATIC_ASSERT((MOTOR_ENCODER_LEFT_A + 1U == MOTOR_ENCODER_LEFT_B) &&
                  (MOTOR_ENCODER_RIGHT_A + 1U == MOTOR_ENCODER_RIGHT_B),
              "the state machine reads B from the pin after A");

static const unsigned int g_encoder_pins[ENCODER_WHEELS][ENCODER_CHANNELS] = {
    {MOTOR_ENCODER_LEFT_A, MOTOR_ENCODER_LEFT_B},
    {MOTOR_ENCODER_RIGHT_A, MOTOR_ENCODER_RIGHT_B},
};
static const unsigned int g_pin_a[ENCODER_WHEELS] = {
    MOTOR_ENCODER_LEFT_A, MOTOR_ENCODER_RIGHT_A,
};

static T_SPLOCK     g_capture_lock;
static quadrature_t g_captured;
/* Lifecycle belongs to the initialisation task, before any readers start. */
static bool g_initialised = false;

/* Take fresh words as the new baseline. Call with the lock held. */
static ER rebase (void)
{
    uint32_t words[ENCODER_WHEELS];

    if (!encoder_pio_read(words))
    {
        return E_IO;
    }
    quadrature_init(&g_captured, words);
    return E_OK;
}

ER encoder_init (void)
{
    UINT saved  = 0U;
    ER   result = E_OK;

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
            /* Pull-up: a Hall encoder with an open-collector output, or a
             * loose encoder wire, otherwise leaves the pin floating, and a
             * floating pin counts motor PWM noise as edges. */
            clr_w(GPIO(pin), MOTION_PAD_PULL_DOWN);
            set_w(GPIO(pin), MOTION_PAD_PULL_UP | MOTION_PAD_INPUT_ENABLE | MOTION_PAD_SCHMITT);
        }
    }

    if (!encoder_pio_start(g_pin_a))
    {
        return E_LIMIT;
    }
    (void)InitSpinLock(&g_capture_lock);
    (void)ISpinLock(&g_capture_lock, &saved);
    result = rebase();
    g_initialised = (E_OK == result);
    (void)ISpinUnlock(&g_capture_lock, saved);
    return result;
}

ER encoder_read (encoder_sample_t * p_sample)
{
    UINT     saved = 0U;
    uint32_t words[ENCODER_WHEELS];

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
    if (!encoder_pio_read(words))
    {
        (void)ISpinUnlock(&g_capture_lock, saved);
        return E_IO;
    }
    for (unsigned int wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        (void)quadrature_apply(&g_captured, wheel, words[wheel]);
        p_sample->wheel[wheel] = g_captured.wheel[wheel];
    }
    p_sample->timestamp_us = in_w(TIMER_TIMERAWL);
    (void)ISpinUnlock(&g_capture_lock, saved);
    return E_OK;
}

ER encoder_reset (void)
{
    UINT saved  = 0U;
    ER   result = E_OK;

    if (!g_initialised)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&g_capture_lock, &saved);
    result = rebase();
    (void)ISpinUnlock(&g_capture_lock, saved);
    return result;
}

/*** end of file ***/
