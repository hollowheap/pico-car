/** @file buddy2_motor_check.c
 * @brief GP20 triggers one short PWM pulse; GP21 stops until reboot.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "encoder_capture.h"
#include "motor_pulse.h"
#include "motor_pwm.h"
#include "usb_console_compat.h"

#define START_PIN 20U
#define STOP_PIN 21U
#define TEST_DUTY_PERMILLE 200U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)

static const char * const phase_names[5] = {"OFF", "M1A", "M1B", "M2A", "M2B"};

static void report_delta (const encoder_capture_sample_t * p_start,
                         const encoder_capture_sample_t * p_end)
{
    unsigned int wheel;

    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        tm_printf((UB *)"%s delta +=%u -=%u bad=%u\n",
            wheel == ENCODER_LEFT ? "LEFT" : "RIGHT",
            (uint32_t)(p_end->wheel[wheel].positive - p_start->wheel[wheel].positive),
            (uint32_t)(p_end->wheel[wheel].negative - p_start->wheel[wheel].negative),
            (uint32_t)(p_end->wheel[wheel].invalid - p_start->wheel[wheel].invalid));
    }
}

static void motor_task (INT start_code, void * p_extra)
{
    motor_pulse_t pulse;
    motor_pulse_phase_t previous = MOTOR_PULSE_OFF;
    encoder_capture_sample_t baseline;
    encoder_capture_sample_t sample;
    ER result;
    unsigned int i;
    bool stop_reported = false;

    (void)start_code;
    (void)p_extra;
    motor_pulse_init(&pulse);
    result = motor_pwm_init();
    if (result == E_OK)
    {
        result = encoder_capture_init();
    }
    (void)gpio_set_pin(START_PIN, GPIO_MODE_IN);
    (void)gpio_set_pin(STOP_PIN, GPIO_MODE_IN);
    clr_w(GPIO(START_PIN), GPIO_PDE);
    clr_w(GPIO(STOP_PIN), GPIO_PDE);
    set_w(GPIO(START_PIN), GPIO_PUE | GPIO_SHEMITT);
    set_w(GPIO(STOP_PIN), GPIO_PUE | GPIO_SHEMITT);
    for (i = 0U; (i < 200U) && (tm_usb_state() < 2U); i++)
    {
        (void)tk_dly_tsk(100);
    }
    (void)tk_dly_tsk(500);
    tm_printf((UB *)"\nBuddy 2 MOTOR PWM CHECK; start BRAKED\n");
    tm_printf((UB *)"10kHz; 20 percent duty; nominal 750ms per pulse\n");
    tm_printf((UB *)"WHEELS LIFTED. Do not use M1/M2 motor-test buttons.\n");
    tm_printf((UB *)"GP20: next pulse M1A/M1B/M2A/M2B; release between presses.\n");
    tm_printf((UB *)"GP21: STOP latched until reboot. No automatic restart.\n");
    if (result != E_OK)
    {
        motor_pwm_boot_brake();
        tm_printf((UB *)"Initialisation failed: %d; output remains low\n", result);
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
    (void)encoder_capture_read(&baseline);
    tm_printf((UB *)"READY for M1A: press and release GP20\n");
    for (;;)
    {
        motor_pulse_phase_t phase;
        uint32_t now = in_w(TIMER_RAW_LOW);
        bool stop_pressed = gpio_get_val(STOP_PIN) == 0U;

        phase = motor_pulse_update(&pulse, now,
            gpio_get_val(START_PIN) == 0U, stop_pressed);
        if (stop_pressed)
        {
            (void)motor_pwm_latch_stop();
        }
        if (phase != previous)
        {
            if (phase == MOTOR_PULSE_OFF)
            {
                (void)motor_pwm_brake();
                (void)encoder_capture_read(&sample);
                tm_printf((UB *)"%s output BRAKED; decision_elapsed_us=%u\n",
                    phase_names[previous], (uint32_t)(now - pulse.started_us));
                report_delta(&baseline, &sample);
                if (!pulse.stopped)
                {
                    if (pulse.next < 4U)
                    {
                        tm_printf((UB *)"READY for %s: release, then press GP20\n",
                                  phase_names[pulse.next + 1U]);
                    }
                    else
                    {
                        tm_printf((UB *)"Sequence complete; remains BRAKED\n");
                    }
                }
            }
            else
            {
                unsigned int step = (unsigned int)phase - 1U;

                (void)encoder_capture_read(&baseline);
                result = motor_pwm_drive(step / 2U, (step & 1U) == 0U,
                                         TEST_DUTY_PERMILLE);
                if (result != E_OK)
                {
                    (void)motor_pwm_latch_stop();
                    (void)motor_pulse_update(&pulse, now, false, true);
                    tm_printf((UB *)"Drive failed: %d; STOP latched\n", result);
                    phase = MOTOR_PULSE_OFF;
                }
            }
            previous = phase;
        }
        if (pulse.stopped && !stop_reported)
        {
            tm_printf((UB *)"STOP LATCHED: reboot required; outputs low\n");
            stop_reported = true;
        }
        /* Deadline and button response include scheduling delay. No serial
         * printing during a running pulse. This is not a hardware cutoff. */
        (void)tk_dly_tsk(1);
    }
}

EXPORT INT usermain (void)
{
    T_CTSK definition = {
        .task = motor_task,
        .itskpri = 5,
        .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
        .assprc = TP_PRC1
    };
    ID task;

    motor_pwm_boot_brake();
    task = tk_cre_tsk(&definition);
    if (task <= E_OK)
    {
        return 1;
    }
    if (tk_sta_tsk(task, 0) != E_OK)
    {
        return 1;
    }
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
