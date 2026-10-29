/** @file motor_task.c
 * @brief One core-0 task owns capture conversion, two PIDs and motor output.
 */
#include <tk/smp.h>
#include <bsp/libbsp.h>
#include "command_slot.h"
#include "motion_config.h"
#include "encoder_capture.h"
#include "guidance.h"
#include "motion/motion.h"
#include "pwm.h"
#include "platform/isr.h"

#define STOP_PIN      21U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)

static T_SPLOCK                service_lock;
static motion_config_t configuration;
static motion_status_t published;
static command_slot_t          inbox;
static uint32_t                next_id;
static ID                      commander;
static ID                      hold_owner[3];
/* Startup lifecycle only: published before any producer/reader starts. */
static bool             start_called;
static controller_t controller;

static int hold_slot (uint32_t reason)
{
    switch (reason)
    {
        case MOTION_HOLD_OBSTACLE:
            return 0;
        case MOTION_HOLD_LINE_LOST:
            return 1;
        case MOTION_HOLD_OPERATOR:
            return 2;
        default:
            return -1;
    }
}

static controller_apply_result_t apply_output (void * context, int16_t left,
                                           int16_t right)
{    ER result;

    (void)context;
    result = pwm_set_pair(left, right);
    if (result == E_OK)
    {
        return CONTROLLER_APPLIED;
    }
    if (result == E_BUSY)
    {
        return pwm_brake() == E_OK ? CONTROLLER_BRAKED_RETRY
                                         : CONTROLLER_OUTPUT_FAILED;
    }
    return CONTROLLER_OUTPUT_FAILED;
}

static bool brake_output (void * context, bool latch)
{
    (void)context;
    return (latch ? pwm_latch_stop() : pwm_brake()) == E_OK;
}

static bool capture_input (odometry_input_t * p_input)
{
    encoder_capture_sample_t captured;
    unsigned int             wheel;

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

static void stop_isr (const isr_gpio_event_t * p_event, void * p_user)
{
    UINT saved;

    (void)p_user;
    if ((p_event->fall_pins & (1U << STOP_PIN)) == 0U)
    {
        return;
    }
    pwm_emergency_stop();
    (void)ISpinLock(&service_lock, &saved);
    command_slot_stop(&inbox);
    (void)ISpinUnlock(&service_lock, saved);
}

static void owner_task (INT start_code, void * p_extra)
{
    controller_output_t output = {NULL, apply_output, brake_output};
    odometry_input_t  input;
    ER              result;
    UINT            saved;
    bool            initial_stop;

    (void)start_code;
    (void)p_extra;
    result = pwm_init();
    if (result == E_OK)
    {
        result = encoder_capture_init();
    }
    (void)gpio_set_pin(STOP_PIN, GPIO_MODE_IN);
    clr_w(GPIO(STOP_PIN), GPIO_PDE);
    set_w(GPIO(STOP_PIN), GPIO_PUE | GPIO_SHEMITT);
    if (result == E_OK)
    {
        result = isr_register_gpio_handler(1U << STOP_PIN,
                                           ISR_GPIO_EDGE_FALL, stop_isr, NULL);
    }
    if ((result == E_OK) && !capture_input(&input))
    {
        result = E_IO;
    }
    if ((result == E_OK) &&
        !controller_init(&controller, &configuration.control, &input,
                             &output))
    {
        result = E_SYS;
    }
    if (result != E_OK)
    {
        /* Owner-only latch if initialisation reached it; boot brake is also
         * asserted because a failed PWM init may not have registered owner. */
        pwm_boot_brake();
        (void)pwm_latch_stop();
        (void)ISpinLock(&service_lock, &saved);
        published.state         = MOTION_FAULT;
        published.fault         = MOTION_FAULT_OUTPUT;
        published.ready         = false;
        (void)ISpinUnlock(&service_lock, saved);
        for (;;)
        {
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }
    (void)ISpinLock(&service_lock, &saved);
    initial_stop = inbox.stopped;
    (void)ISpinUnlock(&service_lock, saved);
    if (initial_stop || gpio_get_val(STOP_PIN) == 0U)
    {
        controller_tick(&controller, NULL, in_w(TIMER_RAW_LOW), NULL, true);
    }
    (void)ISpinLock(&service_lock, &saved);
    if (controller.status.state == MOTION_STOPPED)
    {
        command_slot_stop(&inbox);
    }
    published.stop_requested = inbox.stopped;
    published.hold_mask      = inbox.hold_mask;
    published.ready          = !inbox.stopped &&
                               (controller.status.state != MOTION_STOPPED) &&
                               (controller.status.state != MOTION_FAULT);
    published.state          = controller.status.state;
    published.fault          = controller.status.fault;
    (void)ISpinUnlock(&service_lock, saved);
    for (;;)
    {
        motion_command_t command;
        bool             received;
        bool             stop;
        bool             captured;

        (void)tk_dly_tsk((RELTIM)configuration.poll_delay_ms);
        (void)ISpinLock(&service_lock, &saved);
        stop = inbox.stopped;
        (void)ISpinUnlock(&service_lock, saved);
        stop = stop || gpio_get_val(STOP_PIN) == 0U;
        if (stop)
        {
            /* Do not wait for capture or perform conversion before STOP. */
            controller_tick(&controller, NULL, in_w(TIMER_RAW_LOW), NULL,
                                true);
        }
        else
        {
            captured = capture_input(&input);
            /* Re-read the priority stop slot after capture, before output. */
            (void)ISpinLock(&service_lock, &saved);
            received =
                command_slot_take(&inbox, in_w(TIMER_RAW_LOW), &command, &stop);
            (void)ISpinUnlock(&service_lock, saved);
            stop = stop || gpio_get_val(STOP_PIN) == 0U;
            controller_tick(&controller, captured ? &input : NULL,
                                in_w(TIMER_RAW_LOW), received ? &command : NULL,
                                stop);
        }
        (void)ISpinLock(&service_lock, &saved);
        if (controller.status.state == MOTION_STOPPED)
        {
            command_slot_stop(&inbox);
        }
        published.stop_requested = inbox.stopped;
        published.hold_mask      = inbox.hold_mask;
        published.ready = !inbox.stopped &&
                          (controller.status.state != MOTION_STOPPED) &&
                          (controller.status.state != MOTION_FAULT);
        published.state          = controller.status.state;
        published.fault          = controller.status.fault;
        (void)ISpinUnlock(&service_lock, saved);
    }
}

ER motion_init (void)
{
    if (start_called)
    {
        return E_OBJ;
    }
    motion_config_default(&configuration);
    if (!controller_config_valid(&configuration.control) ||
        (configuration.poll_delay_ms == 0U) ||
        (configuration.poll_delay_ms >
         configuration.control.max_control_gap_us / 1000U))
    {
        return E_PAR;
    }
    (void)command_slot_init(&inbox, configuration.control.max_command_ttl_us);
    (void)InitSpinLock(&service_lock);
    start_called = true;
    pwm_boot_brake();
    return E_OK;
}

void motion_run (void)
{
    owner_task(0, NULL);
}

ER motion_bind (ID tid)
{
    UINT saved;
    ER   result;

    if (!start_called || (tid <= 0))
    {
        return E_OBJ;
    }
    (void)ISpinLock(&service_lock, &saved);
    if ((commander == 0) || (commander == tid))
    {
        commander = tid;
        result    = E_OK;
    }
    else
    {
        result = E_OBJ;
    }
    (void)ISpinUnlock(&service_lock, saved);
    return result;
}

static ER post_command (motion_command_t command)
{
    UINT         saved;
    ER           result = E_OK;
    unsigned int wheel;

    if (!start_called)
    {
        return E_OBJ;
    }
    if (commander == 0)
    {
        return E_OBJ;
    }
    if (tk_get_tid() != commander)
    {
        return E_CTX;
    }
    if ((command.ttl_us == 0U) ||
        (command.ttl_us > configuration.control.max_command_ttl_us))
    {
        return E_PAR;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t limit = configuration.control.speed[wheel].max_target_um_s;

        if (((int64_t)command.target_um_s[wheel] > limit) ||
            ((int64_t)command.target_um_s[wheel] < -limit))
        {
            return E_PAR;
        }
    }
    (void)ISpinLock(&service_lock, &saved);
    if (!published.ready || inbox.stopped)
    {
        result = E_OBJ;
    }
    else
    {
        command_slot_result_t admitted;
        uint32_t              now = in_w(TIMER_RAW_LOW);

        command.id = next_id + 1U;
        if (command.id == 0U)
        {
            command.id++;
        }
        admitted = command_slot_post(&inbox, &command, now);
        if (admitted == COMMAND_SLOT_ACCEPTED)
        {
            next_id = command.id;
        }
        else
        {
            result = admitted == COMMAND_SLOT_HELD
                         ? E_BUSY
                         : (admitted == COMMAND_SLOT_STALE ? E_TMOUT : E_OBJ);
        }
    }
    (void)ISpinUnlock(&service_lock, saved);
    return result;
}

ER motion_submit (const guidance_request_t * p_request)
{
    motion_command_t command;
    uint32_t         limits[2];
    uint32_t         now;
    UINT             saved;

    if (!start_called)
    {
        return E_OBJ;
    }
    limits[0] = configuration.control.speed[0].max_target_um_s;
    limits[1] = configuration.control.speed[1].max_target_um_s;
    now       = in_w(TIMER_RAW_LOW);
    if (!guidance_plan(p_request, limits,
                             configuration.control.max_command_ttl_us, now,
                             &command))
    {
        /* Invalid guidance must not leave the old movement queued. This
         * requests a normal brake and cannot clear emergency/hold state. */
        (void)ISpinLock(&service_lock, &saved);
        command_slot_cancel(&inbox);
        (void)ISpinUnlock(&service_lock, saved);
        return (p_request != NULL && p_request->valid_for_us > 0U &&
                (uint32_t)(now - p_request->sensed_us) >=
                    p_request->valid_for_us)
                   ? E_TMOUT
                   : E_PAR;
    }
    return post_command(command);
}

ER motion_hold (uint32_t reason, bool held)
{
    const int slot = hold_slot(reason);
    const ID  tid  = tk_get_tid();
    UINT      saved;
    ER        result = E_OK;

    if (!start_called)
    {
        return E_OBJ;
    }
    if (slot < 0)
    {
        return E_PAR;
    }
    (void)ISpinLock(&service_lock, &saved);
    if (inbox.stopped || published.state == MOTION_FAULT)
    {
        result = E_OBJ;
    }
    else if (held)
    {
        if ((hold_owner[slot] != 0) && (hold_owner[slot] != tid))
        {
            result = E_CTX;
        }
        else if (command_slot_hold(&inbox, reason, true, in_w(TIMER_RAW_LOW)))
        {
            hold_owner[slot] = tid;
        }
        else
        {
            result = E_PAR;
        }
    }
    else if ((hold_owner[slot] != 0) && (hold_owner[slot] != tid))
    {
        result = E_CTX;
    }
    else if (command_slot_hold(&inbox, reason, false, in_w(TIMER_RAW_LOW)))
    {
        hold_owner[slot] = 0;
    }
    else
    {
        result = E_PAR;
    }
    published.hold_mask = inbox.hold_mask;
    (void)ISpinUnlock(&service_lock, saved);
    return result;
}

ER motion_stop (void)
{
    UINT saved;

    if (!start_called)
    {
        return E_OBJ;
    }
    pwm_emergency_stop();
    (void)ISpinLock(&service_lock, &saved);
    command_slot_stop(&inbox);
    published.ready          = false;
    published.stop_requested = true;
    (void)ISpinUnlock(&service_lock, saved);
    return E_OK;
}

ER motion_status (motion_status_t * p_status)
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
