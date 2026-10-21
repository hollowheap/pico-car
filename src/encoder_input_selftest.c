/** @file encoder_input_selftest.c
 * @brief Exercise GPIO input reads, NVIC delivery and the decoder on-board.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "buddy2_pins.h"
#include "encoder_capture.h"
#include "encoder_input_selftest.h"

#ifndef BUDDY2_INPUT_SELFTEST
#error "Build only in the isolated input self-test target"
#endif

#define TEST_INTE (IO_BANK0_BASE + 0x100U)
#define TEST_INTF (IO_BANK0_BASE + 0x110U)
#define INOVER_MASK 0x00030000U
#define FORCE_LOW   0x00020000U
#define FORCE_HIGH  0x00030000U
#define CYCLES 5U

static const unsigned int test_pins[2][2] = {
    {BUDDY2_ENCODER_LEFT_A, BUDDY2_ENCODER_LEFT_B},
    {BUDDY2_ENCODER_RIGHT_A, BUDDY2_ENCODER_RIGHT_B}
};

static uint32_t test_mask (unsigned int wheel)
{
    return (12U << ((test_pins[wheel][0] % 8U) * 4U)) |
           (12U << ((test_pins[wheel][1] % 8U) * 4U));
}

/* INOVER changes what the peripheral reads, not the pad output. INTF
 * supplies a synthetic rising/falling event independently of the real pin.
 * Both INOVER and INTF updates precede interrupt delivery on this core. */
static void inject_state (unsigned int wheel, uint8_t previous, uint8_t state)
{
    unsigned int channel;
    unsigned int bank = test_pins[wheel][0] / 8U;
    uint32_t events = 0U;
    UINT saved;

    DI(saved);
    for (channel = 0U; channel < 2U; channel++)
    {
        unsigned int pin = test_pins[wheel][channel];
        unsigned int bit = 1U - channel;
        uint32_t level = ((state >> bit) & 1U) != 0U ?
                         FORCE_HIGH : FORCE_LOW;
        out_w(GPIO_CTRL(pin), (in_w(GPIO_CTRL(pin)) & ~INOVER_MASK) | level);
        if (((previous ^ state) & (1U << bit)) != 0U)
        {
            uint32_t event = ((state >> bit) & 1U) != 0U ? 8U : 4U;
            events |= event << ((pin % 8U) * 4U);
        }
    }
    set_w(TEST_INTF + (bank * 4U), events);
    EI(saved);
    (void)tk_dly_tsk(10);
}

bool encoder_input_selftest (void)
{
    static const uint8_t positive[4] = {1U, 3U, 2U, 0U};
    static const uint8_t negative[4] = {2U, 3U, 1U, 0U};
    uint32_t saved_ctrl[2][2];
    uint32_t saved_enable[2];
    uint32_t input_mask = 0U;
    encoder_capture_sample_t sample;
    unsigned int wheel;
    unsigned int channel;
    unsigned int cycle;
    unsigned int step;
    UINT saved;
    bool passed = true;

    if (tk_get_prc() != TP_PRC1)
    {
        return false;
    }
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        for (channel = 0U; channel < 2U; channel++)
        {
            unsigned int pin = test_pins[wheel][channel];
            input_mask |= 1U << pin;
            tm_printf((UB *)"[selftest] GP%u ctrl=%08x pad=%08x\n",
                      pin, in_w(GPIO_CTRL(pin)), in_w(GPIO(pin)));
            /* Pad OD and SIO OE are both checked; do not test by electrically
             * driving an attached encoder output. */
            if ((in_w(GPIO(pin)) & (GPIO_OD | GPIO_IE)) !=
                (GPIO_OD | GPIO_IE))
            {
                return false;
            }
        }
    }
    if ((in_w(GPIO_OE) & input_mask) != 0U)
    {
        return false;
    }

    DI(saved);
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        unsigned int bank = test_pins[wheel][0] / 8U;
        saved_enable[wheel] = in_w(TEST_INTE + (bank * 4U)) & test_mask(wheel);
        /* Mask physical events; INTF still reaches INTS and the NVIC. */
        clr_w(TEST_INTE + (bank * 4U), test_mask(wheel));
        for (channel = 0U; channel < 2U; channel++)
        {
            unsigned int pin = test_pins[wheel][channel];
            saved_ctrl[wheel][channel] = in_w(GPIO_CTRL(pin));
            out_w(GPIO_CTRL(pin),
                  (saved_ctrl[wheel][channel] & ~INOVER_MASK) | FORCE_LOW);
        }
    }
    (void)encoder_capture_reset();
    EI(saved);
    (void)tk_dly_tsk(10);

    for (wheel = 0U; wheel < 2U; wheel++)
    {
        uint8_t previous = 0U;
        for (cycle = 0U; cycle < CYCLES; cycle++)
        {
            for (step = 0U; step < 4U; step++)
            {
                inject_state(wheel, previous, positive[step]);
                previous = positive[step];
            }
        }
        for (cycle = 0U; cycle < CYCLES; cycle++)
        {
            for (step = 0U; step < 4U; step++)
            {
                inject_state(wheel, previous, negative[step]);
                previous = negative[step];
            }
        }
    }
    /* Simultaneous two-bit jump must be rejected, not counted as motion. */
    inject_state(0U, 0U, 3U);
    if (encoder_capture_read(&sample) != E_OK)
    {
        passed = false;
    }
    else
    {
        for (wheel = 0U; wheel < 2U; wheel++)
        {
            uint32_t expected_bad = wheel == 0U ? 1U : 0U;
            bool wheel_ok = sample.wheel[wheel].positive == (4U * CYCLES) &&
                            sample.wheel[wheel].negative == (4U * CYCLES) &&
                            sample.wheel[wheel].invalid == expected_bad;
            passed = passed && wheel_ok;
            tm_printf((UB *)"[selftest] wheel=%u +=%u -=%u bad=%u %s\n",
                wheel, sample.wheel[wheel].positive,
                sample.wheel[wheel].negative, sample.wheel[wheel].invalid,
                wheel_ok ? "PASS" : "FAIL");
        }
        passed = passed && ((sample.pins_and_irq & 15U) == 3U) &&
                 (sample.captured_states == 3U);
    }

    DI(saved);
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        unsigned int bank = test_pins[wheel][0] / 8U;
        clr_w(TEST_INTF + (bank * 4U), test_mask(wheel));
        for (channel = 0U; channel < 2U; channel++)
        {
            out_w(GPIO_CTRL(test_pins[wheel][channel]),
                  saved_ctrl[wheel][channel]);
        }
    }
    /* Discard ALL synthetic counts and reseed from the external inputs. */
    (void)encoder_capture_reset();
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        unsigned int bank = test_pins[wheel][0] / 8U;
        set_w(TEST_INTE + (bank * 4U), saved_enable[wheel]);
    }
    EI(saved);
    tm_printf((UB *)"[selftest] %s; overrides restored; counts reset.\n",
              passed ? "PASS" : "FAIL");
    tm_printf((UB *)"[selftest] External encoders/physical edges NOT verified.\n");
    return passed;
}

/*** end of file ***/
