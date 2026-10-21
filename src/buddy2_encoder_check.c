/** @file buddy2_encoder_check.c
 * @brief Encoder-only bring-up: hold both motor bridges low, log hand turns.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "buddy2_pins.h"
#include "encoder_capture.h"
#include "usb_console_compat.h"
#if defined(BUDDY2_COAST_CALIBRATION) && defined(BUDDY2_INPUT_SELFTEST)
#error "Keep physical calibration and synthetic-input tests separate"
#endif
#ifdef BUDDY2_INPUT_SELFTEST
#include "encoder_input_selftest.h"
#endif

static void hold_motors (void)
{
    static const UINT pins[4] = {
        BUDDY2_MOTOR_LEFT_A, BUDDY2_MOTOR_LEFT_B,
        BUDDY2_MOTOR_RIGHT_A, BUDDY2_MOTOR_RIGHT_B
    };
    unsigned int i;

    /* Cytron ROBO-PICO truth table: A=0, B=0 -> brake. Preload the output
     * latch before switching each pin to SIO output. No PWM is enabled. */
    for (i = 0U; i < 4U; i++)
    {
        (void)gpio_set_val(pins[i], 0U);
        (void)gpio_set_pin(pins[i], GPIO_MODE_OUT);
    }
}

#ifdef BUDDY2_COAST_CALIBRATION
#define CALIBRATION_START_PIN 20U
#define CALIBRATION_FINISH_PIN 21U
#define CALIBRATION_TIMER_LOW (TIMER_BASE + 0x28U)

typedef struct
{
    uint32_t changed_us;
    bool raw_pressed;
    bool armed;
} calibration_button_t;

/* Require a released button before accepting one debounced press. */
static bool calibration_press (calibration_button_t * p_button,
                               bool pressed, uint32_t now)
{
    if (pressed != p_button->raw_pressed)
    {
        p_button->raw_pressed = pressed;
        p_button->changed_us = now;
    }
    if (!pressed && ((uint32_t)(now - p_button->changed_us) >= 50000U))
    {
        p_button->armed = true;
    }
    if (pressed && p_button->armed &&
        ((uint32_t)(now - p_button->changed_us) >= 20000U))
    {
        p_button->armed = false;
        return true;
    }
    return false;
}

static void calibration_sample (const char * p_label,
                                const encoder_capture_sample_t * p_sample)
{
    tm_printf((UB *)"%s t=%u L+=%u L-=%u Lbad=%u "
                    "R+=%u R-=%u Rbad=%u\n",
        p_label, p_sample->timestamp_us,
        p_sample->wheel[0].positive, p_sample->wheel[0].negative,
        p_sample->wheel[0].invalid,
        p_sample->wheel[1].positive, p_sample->wheel[1].negative,
        p_sample->wheel[1].invalid);
}

static void calibration_loop (void)
{
    calibration_button_t start_button = {0U, true, false};
    calibration_button_t finish_button = {0U, true, false};
    encoder_capture_sample_t baseline;
    encoder_capture_sample_t ending;
    uint32_t last_report = in_w(CALIBRATION_TIMER_LOW);
    uint32_t trial = 0U;
    bool recording = false;
    bool have_result = false;
    unsigned int pin;

    for (pin = CALIBRATION_START_PIN; pin <= CALIBRATION_FINISH_PIN; pin++)
    {
        (void)gpio_set_pin(pin, GPIO_MODE_IN);
        clr_w(GPIO(pin), GPIO_PDE);
        set_w(GPIO(pin), GPIO_PUE | GPIO_IE | GPIO_SHEMITT);
    }
    for (;;)
    {
        uint32_t now = in_w(CALIBRATION_TIMER_LOW);
        bool finish_pressed = gpio_get_val(CALIBRATION_FINISH_PIN) == 0U;
        bool start = calibration_press(&start_button,
            gpio_get_val(CALIBRATION_START_PIN) == 0U, now);
        bool finish = calibration_press(&finish_button, finish_pressed, now);

        if (recording)
        {
            if (finish && (encoder_capture_read(&ending) == E_OK))
            {
                recording = false;
                have_result = true;
                /* Publish the frozen result immediately, then repeat while
                 * idle so a host can open the port after the hand turn. */
                last_report = now - 1000000U;
            }
        }
        else if (start && !finish_pressed)
        {
            if (encoder_capture_read(&baseline) == E_OK)
            {
                recording = true;
                have_result = false;
                trial++;
            }
        }
        /* tm_printf masks local interrupts in the pinned kernel. Never call
         * it during the measured interval. GP20 takes the baseline without
         * an acknowledgement print; GP21 freezes the end before reporting. */
        if (!recording && ((uint32_t)(now - last_report) >= 1000000U))
        {
            if (have_result)
            {
                unsigned int wheel;

                tm_printf((UB *)"TRIAL %u COMPLETE; frozen counters\n", trial);
                calibration_sample("BEGIN", &baseline);
                calibration_sample("END", &ending);
                for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
                {
                    tm_printf((UB *)"%s delta +=%u -=%u bad=%u\n",
                        wheel == ENCODER_LEFT ? "LEFT" : "RIGHT",
                        (uint32_t)(ending.wheel[wheel].positive -
                                   baseline.wheel[wheel].positive),
                        (uint32_t)(ending.wheel[wheel].negative -
                                   baseline.wheel[wheel].negative),
                        (uint32_t)(ending.wheel[wheel].invalid -
                                   baseline.wheel[wheel].invalid));
                }
            }
            tm_printf((UB *)"READY: GP20 start; GP21 finish; "
                            "silent while recording; motors COAST\n");
            last_report = in_w(CALIBRATION_TIMER_LOW);
        }
        (void)tk_dly_tsk(1);
    }
}
#endif

static void check_task (INT start_code, void * p_extra)
{
    ER result;
    unsigned int i;
#ifndef BUDDY2_COAST_CALIBRATION
    encoder_capture_sample_t sample;
    uint32_t last_report;
    uint32_t pin_changes[ENCODER_WHEELS] = {0U, 0U};
    uint8_t previous_pins;
    uint8_t pending_seen = 0U;
#endif

    (void)start_code;
    (void)p_extra;
    result = encoder_capture_init();
#ifdef BUDDY2_COAST_CALIBRATION
    if (result == E_OK)
    {
        /* All motor pins are already low SIO outputs. Set both bridge
         * inputs HIGH together: ROBO-PICO 11 = coast, not powered motion.
         * One SIO write avoids sequential software drive states. */
        out_w(GPIO_OUT_SET,
            (1U << BUDDY2_MOTOR_LEFT_A) | (1U << BUDDY2_MOTOR_LEFT_B) |
            (1U << BUDDY2_MOTOR_RIGHT_A) | (1U << BUDDY2_MOTOR_RIGHT_B));
    }
#endif
    for (i = 0U; (i < 200U) && (tm_usb_state() < 2U); i++)
    {
        (void)tk_dly_tsk(100);
    }
    (void)tk_dly_tsk(500);
#ifdef BUDDY2_COAST_CALIBRATION
    tm_printf((UB *)"\nBuddy 2 encoder CALIBRATION; motors COAST; no PWM\n");
    tm_printf((UB *)"Align mark; GP20 start; wait 1s, then turn.\n");
    tm_printf((UB *)"Turn ONE wheel exactly FIVE complete revolutions.\n");
    tm_printf((UB *)"Do not press motor-test buttons; keep wheels lifted.\n");
    tm_printf((UB *)"Stop wheel before GP21 finish. No live measurement prints.\n");
#else
    tm_printf((UB *)"\nBuddy 2 encoder check; motor drive DISABLED\n");
#endif
    tm_printf((UB *)"L=A%u/B%u R=A%u/B%u; x4 quadrature, no pulls\n",
              BUDDY2_ENCODER_LEFT_A, BUDDY2_ENCODER_LEFT_B,
              BUDDY2_ENCODER_RIGHT_A, BUDDY2_ENCODER_RIGHT_B);
    tm_printf((UB *)"Turn ONE wheel by hand; +/- are signal directions.\n");
    tm_printf((UB *)"Counter values are not calibrated mm or speed.\n");
#ifndef BUDDY2_COAST_CALIBRATION
    tm_printf((UB *)"live/poll changes diagnose pins, not encoder distance.\n");
#endif
    if (result != E_OK)
    {
        hold_motors();
        tm_printf((UB *)"Capture init failed: %d; motors remain braked\n",
                  result);
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
#ifdef BUDDY2_INPUT_SELFTEST
    if (!encoder_input_selftest())
    {
        tm_printf((UB *)"Internal input-path test failed; inspect logs.\n");
    }
#endif
#ifdef BUDDY2_COAST_CALIBRATION
    calibration_loop();
#else
    (void)encoder_capture_read(&sample);
    previous_pins = sample.pins_and_irq & 15U;
    last_report = sample.timestamp_us;
    for (;;)
    {
        result = encoder_capture_read(&sample);
        if (result == E_OK)
        {
            uint8_t changed = (previous_pins ^ sample.pins_and_irq) & 15U;
            if ((changed & 3U) != 0U)
            {
                pin_changes[0]++;
            }
            if ((changed & 12U) != 0U)
            {
                pin_changes[1]++;
            }
            previous_pins = sample.pins_and_irq & 15U;
            pending_seen |= sample.pins_and_irq & 48U;
            if ((uint32_t)(sample.timestamp_us - last_report) >= 500000U)
            {
                tm_printf((UB *)"t=%u L_ab=%u live=%u poll=%u +=%u -=%u "
                                "bad=%u R_ab=%u live=%u poll=%u +=%u -=%u "
                                "bad=%u pend=%u en=%u\n",
                    sample.timestamp_us, sample.captured_states & 3U,
                    sample.pins_and_irq & 3U, pin_changes[0],
                    sample.wheel[0].positive, sample.wheel[0].negative,
                    sample.wheel[0].invalid,
                    (sample.captured_states >> 2U) & 3U,
                    (sample.pins_and_irq >> 2U) & 3U, pin_changes[1],
                    sample.wheel[1].positive, sample.wheel[1].negative,
                    sample.wheel[1].invalid, pending_seen >> 4U,
                    sample.pins_and_irq >> 6U);
                last_report = sample.timestamp_us;
                pending_seen = 0U;
            }
        }
        /* Diagnostic polling only. Requested delay is not a proven sampling
         * period; kernel tick resolution and scheduling still apply. */
        (void)tk_dly_tsk(1);
    }
#endif
}

EXPORT INT usermain (void)
{
    T_CTSK definition = {
        .task = check_task,
        .itskpri = 5,
        .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
        .assprc = TP_PRC1
    };
    ID task;
    ER result;

    hold_motors();
    task = tk_cre_tsk(&definition);
    if (task <= E_OK)
    {
        return 1;
    }
    result = tk_sta_tsk(task, 0);
    if (result != E_OK)
    {
        return 1;
    }
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
