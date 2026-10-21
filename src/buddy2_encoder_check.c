/** @file buddy2_encoder_check.c
 * @brief Encoder-only bring-up: hold both motor bridges low, log hand turns.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "buddy2_pins.h"
#include "encoder_capture.h"
#include "usb_console_compat.h"

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

static void check_task (INT start_code, void * p_extra)
{
    encoder_capture_sample_t sample;
    ER result;
    unsigned int i;
    uint32_t last_report;
    uint32_t pin_changes[ENCODER_WHEELS] = {0U, 0U};
    uint8_t previous_pins;
    uint8_t pending_seen = 0U;

    (void)start_code;
    (void)p_extra;
    result = encoder_capture_init();
    for (i = 0U; (i < 200U) && (tm_usb_state() < 2U); i++)
    {
        (void)tk_dly_tsk(100);
    }
    (void)tk_dly_tsk(500);
    tm_printf((UB *)"\nBuddy 2 encoder check; motor drive DISABLED\n");
    tm_printf((UB *)"L=A4/B5 R=A16/B17; x4 quadrature, no pulls\n");
    tm_printf((UB *)"Turn ONE wheel by hand; +/- are signal directions.\n");
    tm_printf((UB *)"Counter values are not calibrated mm or speed.\n");
    tm_printf((UB *)"live/poll changes diagnose pins, not encoder distance.\n");
    if (result != E_OK)
    {
        tm_printf((UB *)"Capture init failed: %d; motors remain braked\n",
                  result);
        (void)tk_slp_tsk(TMO_FEVR);
    }
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
