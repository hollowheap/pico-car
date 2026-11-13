/** @file motion_service.c
 * @brief One core-0 task owns capture conversion, two PIDs and motor output.
 */
#include <tk/smp.h>
#include <bsp/libbsp.h>
#include "encoder.h"
#include "motion/motion.h"
#include "motion_pins.h"
#include "motion_service.h"
#include "pwm.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "supervisor/health_ids.h"

#define STOP_PIN      21U
#define TIMER_RAW_LOW (TIMER_BASE + 0x28U)

/* Control-gap limits are in microseconds; the poll delay is in milliseconds. */
#define US_PER_MS 1000U

/* Ages beyond half the 32-bit microsecond range are clock-read ordering
 * artefacts, not real ages. */
#define CLOCK_HALF_RANGE (UINT32_MAX / 2U)

#define MOTION_SLEEP_TIMEOUT_MS (50U)

static T_SPLOCK                g_service_lock;
static motion_service_config_t g_configuration;
static motion_service_status_t g_published;
static motion_inbox_t          g_inbox;
static uint32_t                g_next_id;
/* Startup lifecycle only: published before any producer/reader starts. */
static bool         g_start_called;
static controller_t g_controller;
static ID           g_motion_task_id;
static ID           g_motion_cyc_id;
static uint32_t     g_motion_overruns;

static void motion_cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_motion_task_id);
}

static motion_apply_result_t apply_output (void * context, int16_t left,
                                           int16_t right)
{
    ER result = E_OK;

    (void)context;
    result = pwm_set_pair(left, right);
    if (result == E_OK)
    {
        return MOTION_APPLIED;
    }
    if (result == E_BUSY)
    {
        /* Also covers unexpected legacy-driver activity: explicitly brake
         * before promising RETRY to the controller. */
        return pwm_brake() == E_OK ? MOTION_BRAKED_RETRY
                                         : MOTION_OUTPUT_FAILED;
    }
    return MOTION_OUTPUT_FAILED;
}

static bool brake_output (void * context, bool latch)
{
    (void)context;
    return (latch ? pwm_latch_stop() : pwm_brake()) == E_OK;
}

static bool capture_input (motion_input_t * p_input)
{
    encoder_sample_t captured;
    unsigned int     wheel = 0U;

    if (encoder_read(&captured) != E_OK)
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

/* Bring up PWM, the encoder capture, the STOP pin and the controller.
 * Returns the first failure; the STOP pin is configured regardless. */
static ER service_start (motion_output_t * p_output, motion_input_t * p_input)
{
    ER result = pwm_init();

    if (result == E_OK)
    {
        result = encoder_init();
    }
    (void)gpio_set_pin(STOP_PIN, GPIO_MODE_IN);
    clr_w(GPIO(STOP_PIN), MOTION_PAD_PULL_DOWN);
    set_w(GPIO(STOP_PIN), MOTION_PAD_PULL_UP | MOTION_PAD_SCHMITT);
    if ((result == E_OK) && !capture_input(p_input))
    {
        result = E_IO;
    }
    if ((result == E_OK) &&
        !controller_init(&g_controller, &g_configuration.control, p_input,
                             p_output))
    {
        result = E_SYS;
    }
    return result;
}

/* Latch the output and publish the initialisation error. Never returns. */
static void service_halt (ER result)
{
    UINT saved = 0U;

    /* Owner-only latch if initialisation reached it; boot brake is also
     * asserted because a failed PWM init may not have registered owner. */
    pwm_boot_brake();
    (void)pwm_latch_stop();
    (void)ISpinLock(&g_service_lock, &saved);
    g_published.init_error    = result;
    g_published.control.state = MOTION_FAULT;
    g_published.control.fault = MOTION_FAULT_OUTPUT;
    g_published.ready         = false;
    (void)ISpinUnlock(&g_service_lock, saved);
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/* Set by the supervisor drive check. */
static volatile bool g_encoder_crossed = false;

/* Report the motion fault bits to the shared register: a motor with no edges
 * (stall), a dead encoder, and the latched stop. Other motion faults reach
 * telemetry through the motion packet. */
static void report_faults (motion_fault_t fault, bool stopped)
{
    uint16_t active = 0U;

    if (g_encoder_crossed)
    {
        active |= FAULT_ENCODER_DEAD_BIT;
    }
    if (MOTION_FAULT_NO_EDGES == fault)
    {
        active |= FAULT_MOTOR_STALL_BIT;
    }
    if (MOTION_FAULT_FEEDBACK == fault)
    {
        active |= FAULT_ENCODER_DEAD_BIT;
    }
    if (stopped)
    {
        active |= FAULT_ESTOP_LATCHED_BIT;
    }
    system_fault_assign((uint16_t)(FAULT_MOTOR_STALL_BIT |
                                   FAULT_ENCODER_DEAD_BIT |
                                   FAULT_ESTOP_LATCHED_BIT),
                        active);
}

/* Apply a stop requested before the first cycle, then publish the first
 * status. */
static void service_first_status (void)
{
    UINT           saved        = 0U;
    bool           initial_stop = false;
    motion_fault_t fault        = MOTION_FAULT_NONE;
    bool           stopped      = false;

    (void)ISpinLock(&g_service_lock, &saved);
    initial_stop = g_inbox.stopped;
    (void)ISpinUnlock(&g_service_lock, saved);
    if (initial_stop || gpio_get_val(STOP_PIN) == 0U)
    {
        controller_tick(&g_controller, NULL, in_w(TIMER_RAW_LOW), NULL, true);
    }
    (void)ISpinLock(&g_service_lock, &saved);
    if (g_controller.status.state == MOTION_STOPPED)
    {
        motion_inbox_stop(&g_inbox);
    }
    g_published.stop_requested = g_inbox.stopped;
    g_published.hold_mask      = g_inbox.hold_mask;
    g_published.ready =
        !g_inbox.stopped && (g_controller.status.state != MOTION_STOPPED) &&
        (g_controller.status.state != MOTION_FAULT);
    g_published.control        = g_controller.status;
    fault                      = g_controller.status.fault;
    stopped                    = g_inbox.stopped;
    (void)ISpinUnlock(&g_service_lock, saved);
    report_faults(fault, stopped);
}

/* Run permit (D160). until_us is written before armed, so the check never
 * sees an armed permit with an old time. */
static volatile uint32_t g_permit_until_us = 0U;
static volatile bool     g_permit_armed    = false;

ER motion_service_renew_permit (uint32_t lease_ms)
{
    g_permit_until_us = in_w(TIMER_RAW_LOW) + (lease_ms * US_PER_MS);
    g_permit_armed    = true;

    return E_OK;
}

/* Hold the car while the permit is lapsed, release the hold when it is renewed. */
static void check_permit (void)
{
    UINT          saved = 0U;
    uint32_t const now  = in_w(TIMER_RAW_LOW);
    bool const     b_lapsed =
        g_permit_armed && ((int32_t)(now - g_permit_until_us) > 0);

    (void)ISpinLock(&g_service_lock, &saved);
    if (b_lapsed != (0U != (g_inbox.hold_mask & MOTION_HOLD_SUPERVISOR)))
    {
        (void)motion_inbox_hold(&g_inbox, MOTION_HOLD_SUPERVISOR, b_lapsed, now);
    }
    (void)ISpinUnlock(&g_service_lock, saved);
}

static void wait_motion_cycle (void)
{
    INT queued = 0;
    ER const er = tk_slp_tsk((TMO)MOTION_SLEEP_TIMEOUT_MS);

    queued = tk_can_wup(TSK_SELF);
    if ((E_OK == er) && (queued > 0))
    {
        g_motion_overruns += (uint32_t)queued;
    }
    else if (E_TMOUT == er)
    {
        g_motion_overruns++;
        service_halt(E_TMOUT);
    }
}

/* One control cycle: STOP handling, capture, command intake, the controller
 * tick, and publication of the result. */
static void service_cycle (motion_input_t * p_input)
{
    UINT             saved    = 0U;
    motion_command_t command;
    bool             received = false;
    bool             stop     = false;
    bool             captured = false;
    motion_fault_t   fault    = MOTION_FAULT_NONE;
    motion_fault_t const previous_fault = g_controller.status.fault;

    wait_motion_cycle();
    liveness_kick(LIVE_MOTION);
    check_permit();
    (void)ISpinLock(&g_service_lock, &saved);
    stop = g_inbox.stopped;
    (void)ISpinUnlock(&g_service_lock, saved);
    stop = stop || gpio_get_val(STOP_PIN) == 0U;
    if (stop)
    {
        /* Do not wait for capture or perform conversion before STOP. */
        controller_tick(&g_controller, NULL, in_w(TIMER_RAW_LOW), NULL,
                            true);
    }
    else
    {
        captured = capture_input(p_input);
        /* Re-read the priority stop slot after capture, before output. */
        (void)ISpinLock(&g_service_lock, &saved);
        received =
            motion_inbox_take(&g_inbox, in_w(TIMER_RAW_LOW), &command, &stop);
        (void)ISpinUnlock(&g_service_lock, saved);
        stop = stop || gpio_get_val(STOP_PIN) == 0U;
        controller_tick(&g_controller, captured ? p_input : NULL,
                            in_w(TIMER_RAW_LOW), received ? &command : NULL,
                            stop);
    }
    (void)ISpinLock(&g_service_lock, &saved);
    if (g_controller.status.state == MOTION_STOPPED)
    {
        motion_inbox_stop(&g_inbox);
    }
    g_published.control        = g_controller.status;
    g_published.stop_requested = g_inbox.stopped;
    g_published.hold_mask      = g_inbox.hold_mask;
    g_published.ready = !g_inbox.stopped &&
                        (g_controller.status.state != MOTION_STOPPED) &&
                        (g_controller.status.state != MOTION_FAULT);
    fault = g_controller.status.fault;
    stop  = g_inbox.stopped;
    (void)ISpinUnlock(&g_service_lock, saved);
    if ((fault != previous_fault) && (MOTION_FAULT_FEEDBACK == fault))
    {
        motion_sample_t const * const sample = &g_controller.status.sample;
        LOG_ERROR(LOG_TASK_MOTION, LOG_MOD_NONE, "feedback captured=%u sample=%u dt_us=%u invalid=%u/%u\n",
                (unsigned int)captured, (unsigned int)g_controller.status.sample_available,
                (unsigned int)sample->elapsed_us, (unsigned int)sample->wheel[0].invalid_delta,
                (unsigned int)sample->wheel[1].invalid_delta);
    }
    report_faults(fault, stop);
}

void motion_service_run (void)
{
    motion_output_t output = {NULL, apply_output, brake_output};
    motion_input_t  input;
    ER              result = service_start(&output, &input);

    if (result != E_OK)
    {
        service_halt(result);
    }
    service_first_status();
    for (;;)
    {
        service_cycle(&input);
    }
}

ER motion_service_init (const motion_service_config_t * p_config, ID task_id)
{
    T_CCYC ccyc = {0};

    if ((tk_get_prc() != TP_PRC1) || (tk_get_tid() <= 0))
    {
        return E_CTX;
    }
    if (g_start_called)
    {
        return E_OBJ;
    }
    if ((p_config == NULL) || (task_id <= 0) ||
        !controller_config_valid(&p_config->control) ||
        (p_config->poll_delay_ms == 0U) ||
        (p_config->poll_delay_ms >
         p_config->control.max_control_gap_us / US_PER_MS))
    {
        return E_PAR;
    }
    g_configuration  = *p_config;
    g_motion_task_id = task_id;

    ccyc.cycatr = TA_HLNG | TA_STA | TA_ASSPRC;
    ccyc.cychdr = (FP)motion_cyclic_wakeup;
    ccyc.cyctim = (RELTIM)g_configuration.poll_delay_ms;
    ccyc.cycphs = (RELTIM)g_configuration.poll_delay_ms;
    ccyc.assprc = TP_PRC1;
    g_motion_cyc_id = tk_cre_cyc(&ccyc);
    if (g_motion_cyc_id < E_OK)
    {
        return (ER)g_motion_cyc_id;
    }

    (void)motion_inbox_init(&g_inbox,
                            g_configuration.control.max_command_ttl_us);
    (void)InitSpinLock(&g_service_lock);
    g_start_called = true;
    pwm_boot_brake();

    return E_OK;
}

static ER post_command (motion_command_t command, bool stamp_now)
{
    UINT         saved  = 0U;
    ER           result = E_OK;
    unsigned int wheel  = 0U;

    if (!g_start_called)
    {
        return E_OBJ;
    }
    if ((command.ttl_us == 0U) ||
        (command.ttl_us > g_configuration.control.max_command_ttl_us))
    {
        return E_PAR;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t limit = g_configuration.control.speed[wheel].max_target_um_s;

        if (((int64_t)command.target_um_s[wheel] > limit) ||
            ((int64_t)command.target_um_s[wheel] < -limit))
        {
            return E_PAR;
        }
    }
    (void)ISpinLock(&g_service_lock, &saved);
    if (!g_published.ready || g_inbox.stopped)
    {
        result = E_OBJ;
    }
    else
    {
        motion_inbox_result_t admitted = MOTION_INBOX_STALE;
        uint32_t              now      = in_w(TIMER_RAW_LOW);

        command.id = g_next_id + 1U;
        if (command.id == 0U)
        {
            command.id++;
        }
        if (stamp_now)
        {
            command.issued_us = now;
        }
        admitted = motion_inbox_post(&g_inbox, &command, now);
        if (admitted == MOTION_INBOX_ACCEPTED)
        {
            g_next_id = command.id;
        }
        else
        {
            result = admitted == MOTION_INBOX_HELD
                         ? E_BUSY
                         : (admitted == MOTION_INBOX_STALE ? E_TMOUT : E_OBJ);
        }
    }
    (void)ISpinUnlock(&g_service_lock, saved);
    return result;
}

ER motion_service_set_speed (int32_t left_um_s, int32_t right_um_s,
                             uint32_t ttl_us)
{
    motion_command_t command = {0U, 0U, ttl_us, {left_um_s, right_um_s}};

    return post_command(command, true);
}

ER motion_service_submit_request (const motion_drive_request_t * p_request)
{
    motion_command_t command;
    uint32_t         limits[ENCODER_WHEELS];
    uint32_t         now   = 0U;
    UINT             saved = 0U;

    if (!g_start_called)
    {
        return E_OBJ;
    }
    limits[0] = g_configuration.control.speed[0].max_target_um_s;
    limits[1] = g_configuration.control.speed[1].max_target_um_s;
    now       = in_w(TIMER_RAW_LOW);
    if (!motion_request_plan(p_request, limits,
                             g_configuration.control.max_command_ttl_us, now,
                             &command))
    {
        /* Invalid guidance must not leave the old movement queued. This
         * requests a normal brake and cannot clear emergency/hold state. */
        (void)ISpinLock(&g_service_lock, &saved);
        motion_inbox_cancel(&g_inbox);
        (void)ISpinUnlock(&g_service_lock, saved);
        return (p_request != NULL && p_request->valid_for_us > 0U &&
                (uint32_t)(now - p_request->sensed_us) >=
                    p_request->valid_for_us)
                   ? E_TMOUT
                   : E_PAR;
    }
    return post_command(command, false);
}

ER motion_service_set_hold (uint32_t reason, bool held)
{
    UINT saved  = 0U;
    ER   result = E_OK;

    if (!g_start_called)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&g_service_lock, &saved);
    if (g_inbox.stopped || g_published.control.state == MOTION_FAULT)
    {
        result = E_OBJ;
    }
    else
    {
        result = motion_inbox_hold(&g_inbox, reason, held, in_w(TIMER_RAW_LOW))
                     ? E_OK
                     : E_PAR;
        g_published.hold_mask = g_inbox.hold_mask;
    }
    (void)ISpinUnlock(&g_service_lock, saved);
    return result;
}

ER motion_service_request_stop (void)
{
    UINT saved = 0U;

    if (!g_start_called)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&g_service_lock, &saved);
    motion_inbox_stop(&g_inbox);
    g_published.ready          = false;
    g_published.stop_requested = true;
    (void)ISpinUnlock(&g_service_lock, saved);
    return E_OK;
}

ER motion_service_read_status (motion_service_status_t * p_status)
{
    UINT saved = 0U;

    if (p_status == NULL)
    {
        return E_PAR;
    }
    if (!g_start_called)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&g_service_lock, &saved);
    *p_status = g_published;
    (void)ISpinUnlock(&g_service_lock, saved);
    return E_OK;
}

ER motion_feedback_snapshot (motion_feedback_t * p_feedback)
{
    motion_service_status_t status;
    ER                      result = E_OK;
    uint32_t                age_us = 0U;

    if (p_feedback == NULL)
    {
        return E_PAR;
    }

    result = motion_service_read_status(&status);
    if (E_OK != result)
    {
        return result;
    }

    age_us = in_w(TIMER_RAW_LOW) - status.control.sample.timestamp_us;
    if (age_us > CLOCK_HALF_RANGE)
    {
        /* The encoder can be sampled just after this clock read. */
        age_us = 0U;
    }
    p_feedback->left_speed_um_s  = status.control.sample.wheel[ENCODER_LEFT].velocity_um_s;
    p_feedback->right_speed_um_s = status.control.sample.wheel[ENCODER_RIGHT].velocity_um_s;
    p_feedback->left_displacement_um =
        status.control.sample.wheel[ENCODER_LEFT].displacement_um;
    p_feedback->right_displacement_um =
        status.control.sample.wheel[ENCODER_RIGHT].displacement_um;
    p_feedback->timestamp_us = status.control.sample.timestamp_us;
    p_feedback->age_us       = age_us;
    p_feedback->speed_valid[ENCODER_LEFT] =
        status.control.sample.wheel[ENCODER_LEFT].speed_valid;
    p_feedback->speed_valid[ENCODER_RIGHT] =
        status.control.sample.wheel[ENCODER_RIGHT].speed_valid;
    p_feedback->odometry_valid[ENCODER_LEFT] =
        status.control.sample.wheel[ENCODER_LEFT].odometry_valid;
    p_feedback->odometry_valid[ENCODER_RIGHT] =
        status.control.sample.wheel[ENCODER_RIGHT].odometry_valid;
    p_feedback->fresh = status.ready && status.control.sample_available &&
                        (age_us <= MOTION_FEEDBACK_MAX_AGE_US);

    return E_OK;
}

ER motion_status (motion_status_t * p_status)
{
    motion_service_status_t status;
    ER                      result = E_OK;

    if (p_status == NULL)
    {
        return E_PAR;
    }

    result = motion_service_read_status(&status);
    if (E_OK != result)
    {
        return result;
    }

    p_status->state          = (uint8_t)status.control.state;
    p_status->fault          = (uint8_t)status.control.fault;
    p_status->ready          = status.ready;
    p_status->stop_requested = status.stop_requested;
    p_status->hold_mask      = status.hold_mask;
    p_status->init_error     = status.init_error;

    return E_OK;
}

ER motion_diag_snapshot (motion_diag_t * p_diag)
{
    motion_service_status_t status;
    ER                      result = E_OK;

    if (p_diag == NULL)
    {
        return E_PAR;
    }

    result = motion_service_read_status(&status);
    if (E_OK != result)
    {
        return result;
    }

    p_diag->left_target_um_s     = status.control.target_um_s[ENCODER_LEFT];
    p_diag->right_target_um_s    = status.control.target_um_s[ENCODER_RIGHT];
    p_diag->left_duty_permille   = status.control.duty[ENCODER_LEFT];
    p_diag->right_duty_permille  = status.control.duty[ENCODER_RIGHT];
    p_diag->overruns             = g_motion_overruns;

    return E_OK;
}

ER motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    return motion_service_set_speed(left_um_s, right_um_s, ttl_us);
}

ER motion_set_hold (uint32_t reason, bool held)
{
    return motion_service_set_hold(reason, held);
}

ER motion_request_stop (void)
{
    return motion_service_request_stop();
}

ER motion_renew_permit (uint32_t lease_ms)
{
    return motion_service_renew_permit(lease_ms);
}

void motion_note_encoder_crossed (bool b_crossed)
{
    g_encoder_crossed = b_crossed;
}
