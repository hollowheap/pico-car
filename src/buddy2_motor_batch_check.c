/** @file buddy2_motor_batch_check.c
 * @brief One GP20 start, six short wheel-up pulses, GP21 latched stop.
 */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>
#include "encoder_capture.h"
#include "motor_batch.h"
#include "motor_pwm.h"
#include "usb_console_compat.h"

#define START_PIN 20U
#define STOP_PIN 21U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)

static const char * const names[MOTOR_BATCH_STEPS] = {
    "LEFT_FORWARD", "RIGHT_FORWARD", "BOTH_FORWARD",
    "LEFT_REVERSE", "RIGHT_REVERSE", "BOTH_REVERSE"
};

typedef struct
{
    encoder_capture_sample_t begin;
    encoder_capture_sample_t end;
    uint32_t elapsed_us;
    motor_batch_feedback_t feedback;
    bool recorded;
} batch_result_t;

static batch_result_t results[MOTOR_BATCH_STEPS];

static void print_results (const motor_batch_t * p_batch, ER driver_error,
                           bool capture_error, bool button_stop)
{
    unsigned int step;
    unsigned int wheel;

    tm_printf((UB *)"BATCH END state=%u gap_fault=%u max_active_gap_us=%u driver=%d capture_error=%u button_stop=%u\n",
        (unsigned int)p_batch->phase, p_batch->gap_fault ? 1U : 0U,
        p_batch->max_gap_us, driver_error, capture_error ? 1U : 0U,
        button_stop ? 1U : 0U);
    for (step = 0U; step < MOTOR_BATCH_STEPS; step++)
    {
        if (results[step].recorded)
        {
            tm_printf((UB *)"STEP %u %s elapsed_us=%u feedback=%u\n",
                step + 1U, names[step], results[step].elapsed_us,
                (unsigned int)results[step].feedback);
            for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
            {
                const encoder_quad_counts_t * p_begin = &results[step].begin.wheel[wheel];
                const encoder_quad_counts_t * p_end = &results[step].end.wheel[wheel];

                tm_printf((UB *)"%s +=%u -=%u bad=%u\n",
                    wheel == ENCODER_LEFT ? "LEFT" : "RIGHT",
                    (uint32_t)(p_end->positive - p_begin->positive),
                    (uint32_t)(p_end->negative - p_begin->negative),
                    (uint32_t)(p_end->invalid - p_begin->invalid));
            }
        }
    }
    tm_printf((UB *)"BATCH REPORT COMPLETE; braked; reboot required for another batch\n");
}

static void batch_task (INT start_code, void * p_extra)
{
    motor_batch_t batch;
    encoder_capture_sample_t baseline = {0};
    encoder_capture_sample_t sample = {0};
    ER driver_error;
    bool capture_error = false;
    bool button_stop = false;
    bool feedback_fault = false;
    int16_t left = 0;
    int16_t right = 0;
    uint32_t last_ready_us = 0U;
    unsigned int i;

    (void)start_code;
    (void)p_extra;
    motor_batch_init(&batch);
    driver_error = motor_pwm_init();
    if (driver_error == E_OK)
    {
        capture_error = encoder_capture_init() != E_OK;
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
    tm_printf((UB *)"\nBuddy 2 MOTOR BATCH CHECK; boot BRAKED\n");
    tm_printf((UB *)"WHEELS LIFTED; no M1/M2 carrier buttons. USB bench supply.\n");
    tm_printf((UB *)"One GP20 start: six automatic 20-percent 300ms pulses, 400ms braked pauses.\n");
    tm_printf((UB *)"GP21 STOP latches until reboot. No printing while batch runs.\n");
    if ((driver_error != E_OK) || capture_error)
    {
        motor_pwm_boot_brake();
        tm_printf((UB *)"INIT FAILED driver=%d capture_error=%u; no motion\n",
                  driver_error, capture_error ? 1U : 0U);
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
    tm_printf((UB *)"READY: release, then press GP20 once; await BATCH END\n");
    for (;;)
    {
        uint32_t now = in_w(TIMER_RAW_LOW);
        motor_batch_phase_t previous = batch.phase;
        unsigned int previous_step = batch.step;
        bool stop = gpio_get_val(STOP_PIN) == 0U;

        button_stop = button_stop || stop;
        if (previous == MOTOR_BATCH_RUN)
        {
            capture_error = encoder_capture_read(&sample) != E_OK;
            if (!capture_error)
            {
                feedback_fault =
                    (sample.wheel[0].invalid != baseline.wheel[0].invalid) ||
                    (sample.wheel[1].invalid != baseline.wheel[1].invalid);
            }
        }
        (void)motor_batch_update(&batch, now, gpio_get_val(START_PIN) == 0U,
            stop || capture_error || feedback_fault || (driver_error != E_OK));
        if ((previous == MOTOR_BATCH_RUN) && (batch.phase != MOTOR_BATCH_RUN))
        {
            batch_result_t * p_result = &results[previous_step];

            driver_error = motor_pwm_brake();
            capture_error = encoder_capture_read(&sample) != E_OK || capture_error;
            p_result->begin = baseline;
            p_result->end = sample;
            /* Includes baseline-to-brake observation, not physical rest. */
            p_result->elapsed_us = sample.timestamp_us - baseline.timestamp_us;
            p_result->feedback = capture_error ? MOTOR_BATCH_FEEDBACK_INVALID :
                motor_batch_feedback(baseline.wheel, sample.wheel, left, right);
            p_result->recorded = true;
            feedback_fault = p_result->feedback != MOTOR_BATCH_FEEDBACK_OK;
            if (feedback_fault || capture_error || driver_error != E_OK)
            {
                (void)motor_batch_update(&batch, now, false, true);
            }
        }
        if ((batch.phase == MOTOR_BATCH_RUN) && (previous != MOTOR_BATCH_RUN))
        {
            if (previous == MOTOR_BATCH_WAIT)
            {
                /* Console lock masks capture IRQs; acknowledge before PWM
                 * and the first baseline, never during a driven interval. */
                tm_printf((UB *)"START accepted; automatic wheel-up batch\n");
                now = in_w(TIMER_RAW_LOW);
                batch.changed_us = now;
                batch.previous_us = now;
            }
            capture_error = encoder_capture_read(&baseline) != E_OK;
            /* Check stop again immediately before energising a bridge. */
            stop = gpio_get_val(STOP_PIN) == 0U;
            button_stop = button_stop || stop;
            if (capture_error || stop)
            {
                (void)motor_batch_update(&batch, now, false, true);
            }
            else
            {
                motor_batch_duties(&batch, &left, &right);
                driver_error = motor_pwm_set_pair(left, right);
                if (driver_error != E_OK)
                {
                    (void)motor_batch_update(&batch, now, false, true);
                }
            }
        }
        if ((batch.phase == MOTOR_BATCH_DONE) || (batch.phase == MOTOR_BATCH_STOPPED))
        {
            (void)motor_pwm_latch_stop();
            print_results(&batch, driver_error, capture_error, button_stop);
            for (;;)
            {
                (void)tk_dly_tsk(2000);
                print_results(&batch, driver_error, capture_error, button_stop);
            }
        }
        if ((batch.phase == MOTOR_BATCH_WAIT) &&
            ((uint32_t)(now - last_ready_us) >= UINT32_C(1000000)))
        {
            tm_printf((UB *)"READY motor batch; GP20=%u GP21=%u; wheels lifted before start\n",
                gpio_get_val(START_PIN), gpio_get_val(STOP_PIN));
            last_ready_us = now;
        }
        /* Actual loop cadence is scheduled by the kernel; no 1ms guarantee. */
        (void)tk_dly_tsk(1);
    }
}

EXPORT INT usermain (void)
{
    T_CTSK definition = {
        .task = batch_task, .itskpri = 5, .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1
    };
    ID task;

    motor_pwm_boot_brake();
    task = tk_cre_tsk(&definition);
    if ((task <= E_OK) || (tk_sta_tsk(task, 0) != E_OK))
    {
        return 1;
    }
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
