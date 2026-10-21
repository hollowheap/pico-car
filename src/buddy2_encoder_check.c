/** @file buddy2_encoder_check.c
 * @brief Encoder-only bring-up: hold both motor bridges low, log hand turns.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "buddy2_pins.h"
#include "encoder_capture.h"
#include "usb_console_compat.h"
#ifdef BUDDY2_MOTION_MEASUREMENT
#include "motion_sampler.h"
#ifndef BUDDY2_COAST_CALIBRATION
#error "Motion measurement requires the quiet coast calibration harness"
#endif

typedef struct
{
    motion_sampler_t sampler;
    motion_sample_t sample;
    uint32_t peak_speed[ENCODER_WHEELS];
    uint32_t max_interval_us;
    uint32_t failures;
    bool have_sample;
} motion_probe_t;

static motion_input_t motion_input (const encoder_capture_sample_t * p_capture)
{
    motion_input_t input;
    unsigned int wheel;

    input.timestamp_us = p_capture->timestamp_us;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        input.wheel[wheel] = p_capture->wheel[wheel];
    }
    return input;
}

static void motion_probe_start (motion_probe_t * p_probe,
                                const encoder_capture_sample_t * p_capture)
{
    /* Provisional hand-turn means and nominal pi * 67 mm, rounded to um.
     * 200 ms is this probe's gap threshold, not a verified control deadline.
     * The library accepts configuration; no such constants live in its math. */
    const motion_config_t config = {
        {{210487U, 2541U, -1, false}, {210487U, 2530U, 1, false}}, 200000U
    };
    motion_probe_t initial = {0};
    motion_input_t input = motion_input(p_capture);

    if (!motion_sampler_init(&initial.sampler, &config, &input))
    {
        initial.failures++;
    }
    *p_probe = initial;
}

static void motion_probe_update (motion_probe_t * p_probe,
                                 const encoder_capture_sample_t * p_capture)
{
    motion_input_t input = motion_input(p_capture);
    unsigned int wheel;

    if (input.timestamp_us == p_probe->sampler.previous.timestamp_us)
    {
        return;
    }
    if (!motion_sampler_update(&p_probe->sampler, &input, &p_probe->sample))
    {
        p_probe->failures++;
        return;
    }
    p_probe->have_sample = true;
    if (p_probe->sample.elapsed_us > p_probe->max_interval_us)
    {
        p_probe->max_interval_us = p_probe->sample.elapsed_us;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int32_t velocity = p_probe->sample.wheel[wheel].velocity_um_s;
        uint32_t speed = (uint32_t)(velocity < 0 ? -velocity : velocity);

        if (p_probe->sample.wheel[wheel].speed_valid &&
            (speed > p_probe->peak_speed[wheel]))
        {
            p_probe->peak_speed[wheel] = speed;
        }
    }
}

static void motion_probe_report (const motion_probe_t * p_probe)
{
    unsigned int wheel;

    tm_printf((UB *)"MOTION provisional scale; max_dt_us=%u failures=%u\n",
              p_probe->max_interval_us, p_probe->failures);
    if (!p_probe->have_sample)
    {
        tm_printf((UB *)"MOTION no usable sample\n");
        return;
    }
    tm_printf((UB *)"MOTION frozen_t=%u last_dt_us=%u\n",
              p_probe->sample.timestamp_us, p_probe->sample.elapsed_us);
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        const motion_wheel_sample_t * p_result = &p_probe->sample.wheel[wheel];

        /* T-Monitor has no 64-bit format. Refuse out-of-range display instead
         * of truncating the library's 64-bit displacement. */
        if ((p_result->displacement_um < INT32_MIN) ||
            (p_result->displacement_um > INT32_MAX))
        {
            tm_printf((UB *)"MOTION displacement outside display range\n");
            continue;
        }
        tm_printf((UB *)"MOTION %s displacement_um=%d velocity_um_s=%d "
                        "peak_abs_um_s=%u speed_valid=%u odometry_valid=%u "
                        "scale_verified=%u\n",
            wheel == ENCODER_LEFT ? "LEFT" : "RIGHT",
            (int32_t)p_result->displacement_um, p_result->velocity_um_s,
            p_probe->peak_speed[wheel], p_result->speed_valid ? 1U : 0U,
            p_result->odometry_valid ? 1U : 0U,
            p_result->scale_verified ? 1U : 0U);
    }
}
#endif
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
    uint32_t last_loop = last_report;
    uint32_t max_idle_gap = 0U;
    uint32_t trial = 0U;
    bool recording = false;
    bool have_result = false;
    unsigned int pin;
#ifdef BUDDY2_MOTION_MEASUREMENT
    motion_probe_t motion = {0};
#endif

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

        if (!recording && ((uint32_t)(now - last_loop) > max_idle_gap))
        {
            max_idle_gap = now - last_loop;
        }
        last_loop = now;

        if (recording)
        {
#ifdef BUDDY2_MOTION_MEASUREMENT
            encoder_capture_sample_t current;

            if (encoder_capture_read(&current) == E_OK)
            {
                motion_probe_update(&motion, &current);
            }
            else
            {
                motion.failures++;
            }
#endif
            if (finish && (encoder_capture_read(&ending) == E_OK))
            {
#ifdef BUDDY2_MOTION_MEASUREMENT
                motion_probe_update(&motion, &ending);
#endif
                recording = false;
                have_result = true;
                /* Publish the frozen result immediately, then repeat while
                 * idle so a host can open the port after the hand turn. */
                last_report = now - 1000000U;
            }
        }
        else if (start && !finish_pressed)
        {
            /* Acknowledge before taking the baseline: this print cannot
             * mask encoder IRQs inside the measured interval. */
            tm_printf((UB *)"START accepted: trial=%u; taking baseline; "
                            "then quiet until GP21\n", trial + 1U);
            if (encoder_capture_read(&baseline) == E_OK)
            {
#ifdef BUDDY2_MOTION_MEASUREMENT
                motion_probe_start(&motion, &baseline);
#endif
                recording = true;
                have_result = false;
                trial++;
            }
            else
            {
                tm_printf((UB *)"START failed: no baseline; keep wheels still\n");
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
#ifdef BUDDY2_MOTION_MEASUREMENT
                motion_probe_report(&motion);
#endif
            }
            tm_printf((UB *)"READY: GP20 start; GP21 finish; "
                            "silent while recording; motors COAST\n");
            tm_printf((UB *)"IDLE t=%u max_loop_gap_us=%u gp20=%u gp21=%u "
                            "usb_dropped=%u\n", in_w(CALIBRATION_TIMER_LOW),
                max_idle_gap, gpio_get_val(CALIBRATION_START_PIN),
                gpio_get_val(CALIBRATION_FINISH_PIN), tm_usb_dropped_bytes());
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
#ifdef BUDDY2_MOTION_MEASUREMENT
    tm_printf((UB *)"MOTION CHECK: provisional scales; no closed-loop control\n");
#endif
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
