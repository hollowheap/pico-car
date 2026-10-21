/** @file motion_service.c
 * @brief One core-0 task owns capture conversion, two PIDs and motor output.
 */
#include <tk/smp.h>
#include <bsp/libbsp.h>
#include "encoder_capture.h"
#include "motion_service.h"
#include "motor_pwm.h"

#define STOP_PIN 21U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)

static T_SPLOCK service_lock;
static motion_service_config_t configuration;
static motion_service_status_t published;
static motion_command_t pending;
static uint32_t next_id;
static bool has_pending;
static bool stop_latched;
/* Startup lifecycle only: published before any producer/reader starts. */
static bool start_called;
static motion_control_t controller;

static motion_apply_result_t apply_output (void * context, int16_t left, int16_t right)
{
    ER result;

    (void)context;
    result = motor_pwm_set_pair(left, right);
    if (result == E_OK)
    {
        return MOTION_APPLIED;
    }
    if (result == E_BUSY)
    {
        /* Also covers unexpected legacy-driver activity: explicitly brake
         * before promising RETRY to the controller. */
        return motor_pwm_brake() == E_OK ? MOTION_BRAKED_RETRY : MOTION_OUTPUT_FAILED;
    }
    return MOTION_OUTPUT_FAILED;
}

static bool brake_output (void * context, bool latch)
{
    (void)context;
    return (latch ? motor_pwm_latch_stop() : motor_pwm_brake()) == E_OK;
}

static bool capture_input (motion_input_t * p_input)
{
    encoder_capture_sample_t captured;
    unsigned int wheel;

    if (encoder_capture_read(&captured) != E_OK)
    {
        return false;
    }
    p_input->timestamp_us = captured.timestamp_us;
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        p_input->wheel[wheel] = captured.wheel[wheel];
    }
    return true;
}

static void owner_task (INT start_code, void * p_extra)
{
    motion_output_t output = {NULL, apply_output, brake_output};
    motion_input_t input;
    ER result;
    UINT saved;
    bool initial_stop;

    (void)start_code;
    (void)p_extra;
    result = motor_pwm_init();
    if (result == E_OK)
    {
        result = encoder_capture_init();
    }
    (void)gpio_set_pin(STOP_PIN, GPIO_MODE_IN);
    clr_w(GPIO(STOP_PIN), GPIO_PDE);
    set_w(GPIO(STOP_PIN), GPIO_PUE | GPIO_SHEMITT);
    if ((result == E_OK) && !capture_input(&input))
    {
        result = E_IO;
    }
    if ((result == E_OK) &&
        !motion_control_init(&controller, &configuration.control, &input, &output))
    {
        result = E_SYS;
    }
    if (result != E_OK)
    {
        /* Owner-only latch if initialisation reached it; boot brake is also
         * asserted because a failed PWM init may not have registered owner. */
        motor_pwm_boot_brake();
        (void)motor_pwm_latch_stop();
        (void)ISpinLock(&service_lock, &saved);
        published.init_error = result;
        published.control.state = MOTION_FAULT;
        published.control.fault = MOTION_FAULT_OUTPUT;
        published.ready = false;
        (void)ISpinUnlock(&service_lock, saved);
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
    (void)ISpinLock(&service_lock, &saved);
    initial_stop = stop_latched;
    (void)ISpinUnlock(&service_lock, saved);
    if (initial_stop || gpio_get_val(STOP_PIN) == 0U)
    {
        motion_control_tick(&controller, NULL, in_w(TIMER_RAW_LOW), NULL, true);
    }
    (void)ISpinLock(&service_lock, &saved);
    if (controller.status.state == MOTION_STOPPED)
    {
        stop_latched = true;
    }
    published.stop_requested = stop_latched;
    published.ready = !stop_latched && (controller.status.state != MOTION_STOPPED) &&
                      (controller.status.state != MOTION_FAULT);
    published.control = controller.status;
    (void)ISpinUnlock(&service_lock, saved);
    for (;;)
    {
        motion_command_t command;
        bool received;
        bool stop;
        bool captured;

        (void)tk_dly_tsk((RELTIM)configuration.poll_delay_ms);
        (void)ISpinLock(&service_lock, &saved);
        received = has_pending;
        command = pending;
        has_pending = false;
        stop = stop_latched;
        (void)ISpinUnlock(&service_lock, saved);
        stop = stop || gpio_get_val(STOP_PIN) == 0U;
        if (stop)
        {
            /* Do not wait for capture or perform conversion before STOP. */
            motion_control_tick(&controller, NULL, in_w(TIMER_RAW_LOW), NULL, true);
        }
        else
        {
            captured = capture_input(&input);
            /* Re-read the priority stop slot after capture, before output. */
            (void)ISpinLock(&service_lock, &saved);
            stop = stop_latched;
            (void)ISpinUnlock(&service_lock, saved);
            stop = stop || gpio_get_val(STOP_PIN) == 0U;
            motion_control_tick(&controller, captured ? &input : NULL,
                in_w(TIMER_RAW_LOW), received ? &command : NULL, stop);
        }
        (void)ISpinLock(&service_lock, &saved);
        if (controller.status.state == MOTION_STOPPED)
        {
            stop_latched = true;
        }
        published.control = controller.status;
        published.stop_requested = stop_latched;
        published.ready = !stop_latched && (controller.status.state != MOTION_STOPPED) &&
                          (controller.status.state != MOTION_FAULT);
        (void)ISpinUnlock(&service_lock, saved);
    }
}

ER motion_service_start (const motion_service_config_t * p_config)
{
    T_CTSK task_definition = {
        .task = owner_task, .itskpri = 5, .stksz = 4096,
        .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1
    };
    ID task;
    ER result;

    if ((tk_get_prc() != TP_PRC1) || (tk_get_tid() <= 0))
    {
        return E_CTX;
    }
    if (start_called)
    {
        return E_OBJ;
    }
    if ((p_config == NULL) || !motion_control_config_valid(&p_config->control) ||
        (p_config->poll_delay_ms == 0U) ||
        (p_config->poll_delay_ms > p_config->control.max_control_gap_us / 1000U))
    {
        return E_PAR;
    }
    configuration = *p_config;
    (void)InitSpinLock(&service_lock);
    start_called = true;
    motor_pwm_boot_brake();
    task = tk_cre_tsk(&task_definition);
    result = task <= E_OK ? (ER)task : tk_sta_tsk(task, 0);
    if (result != E_OK)
    {
        published.init_error = result;
        published.control.state = MOTION_FAULT;
    }
    return result;
}

ER motion_service_set_speed (int32_t left_um_s, int32_t right_um_s,
                             uint32_t ttl_us)
{
    UINT saved;
    ER result = E_OK;
    const int32_t targets[2] = {left_um_s, right_um_s};
    unsigned int wheel;

    if (!start_called)
    {
        return E_OBJ;
    }
    if ((ttl_us == 0U) || (ttl_us > configuration.control.max_command_ttl_us))
    {
        return E_PAR;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t limit = configuration.control.speed[wheel].max_target_um_s;

        if (((int64_t)targets[wheel] > limit) || ((int64_t)targets[wheel] < -limit))
        {
            return E_PAR;
        }
    }
    (void)ISpinLock(&service_lock, &saved);
    if (!published.ready || stop_latched)
    {
        result = E_OBJ;
    }
    else
    {
        next_id++;
        if (next_id == 0U)
        {
            next_id++;
        }
        pending = (motion_command_t){next_id, in_w(TIMER_RAW_LOW), ttl_us,
                                     {left_um_s, right_um_s}};
        has_pending = true;
    }
    (void)ISpinUnlock(&service_lock, saved);
    return result;
}

ER motion_service_request_stop (void)
{
    UINT saved;

    if (!start_called)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&service_lock, &saved);
    stop_latched = true;
    has_pending = false;
    published.ready = false;
    published.stop_requested = true;
    (void)ISpinUnlock(&service_lock, saved);
    return E_OK;
}

ER motion_service_read_status (motion_service_status_t * p_status)
{
    UINT saved;

    if (p_status == NULL)
    {
        return E_PAR;
    }
    if (!start_called)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&service_lock, &saved);
    *p_status = published;
    (void)ISpinUnlock(&service_lock, saved);
    return E_OK;
}
