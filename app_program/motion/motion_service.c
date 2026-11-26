/** @file motion_service.c
 * @brief One core-0 task owns capture conversion, two PIDs and motor output.
 */
#include <tk/smp.h>
#include "drv_stop.h"
#include "encoder.h"
#include "motion/motion_state.h"
#include "motion_service.h"
#include "drv_pwm.h"
#include "platform/drv_timer.h"
#include "platform/faults.h"
#include "platform/drv_gpio_irq.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/units.h"
#include "supervisor/health_ids.h"
#include "comms/comms.h"

/* The supervisor and the dashboard read motion_status_t.fault as the wire
 * code; the supervisor reboots on EXPIRED and GAP. */
STATIC_ASSERT ((uint32_t)MOTION_FAULT_EXPIRED == TELEMETRY_MOTION_FAULT_EXPIRED,
               "EXPIRED wire code");
STATIC_ASSERT ((uint32_t)MOTION_FAULT_GAP == TELEMETRY_MOTION_FAULT_GAP, "GAP wire code");
STATIC_ASSERT ((uint32_t)MOTION_FAULT_OUTPUT == TELEMETRY_MOTION_FAULT_OUTPUT,
               "OUTPUT wire code");
STATIC_ASSERT (MOTION_HOLD_ALL <= UINT8_MAX, "motion event hold mask");

/* Control-gap limits are in microseconds; the poll delay is in milliseconds. */
#define US_PER_MS 1000U

#define MOTION_SLEEP_TIMEOUT_MS (50U)
#define MOTION_EVENT_BYTE_SHIFT (8U)

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
/* Guarded by g_service_lock: the task writes it, other cores read it. */
static uint32_t     g_motion_overruns;

/* Only the motion task writes the event baseline. */
static uint8_t g_event_state = (uint8_t)MOTION_IDLE;
static uint8_t g_event_fault = (uint8_t)MOTION_FAULT_NONE;
static uint8_t g_event_holds;

static void emit_motion_change (motion_state_t state, motion_fault_t fault,
                                uint8_t holds)
{
    uint8_t const new_state = (uint8_t)state;
    uint8_t const new_fault = (uint8_t)fault;

    if ((g_event_state != new_state) || (g_event_fault != new_fault) ||
        (g_event_holds != holds))
    {
        motion_sample_t const * const p_sample = &g_controller.status.sample;
        int16_t const states =
            (int16_t)(((uint32_t)g_event_state << MOTION_EVENT_BYTE_SHIFT) | new_state);
        uint16_t const fault_holds =
            (uint16_t)(((uint32_t)new_fault << MOTION_EVENT_BYTE_SHIFT) | holds);
        uint32_t const odometer_mm = wheel_odometer_mm(
            p_sample->wheel[ENCODER_LEFT].displacement_um,
            p_sample->wheel[ENCODER_RIGHT].displacement_um);

        /* An unavailable queue must not cause retries from the control loop. */
        g_event_state = new_state;
        g_event_fault = new_fault;
        g_event_holds = holds;
        (void)telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_MOTION,
                                      TELEMETRY_EVT_MOTION_STATE, states,
                                      fault_holds, odometer_mm);
    }
}

static void motion_cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_motion_task_id);
}

/* The inbox and the published drive inhibit change under the same lock. */
static void latch_stop_locked (uint32_t stamp_us)
{
    if (!g_inbox.stopped)
    {
        g_published.stop_stamp_us = stamp_us;
    }
    motion_inbox_stop(&g_inbox);
    g_published.ready          = false;
    g_published.stop_requested = true;
}

static ER request_stop (uint32_t stamp_us);

static void stop_pin_edge (const gpio_edge_snapshot_t * p_edges)
{
    if (0U != (p_edges->fall_mask & DRV_STOP_PIN_MASK))
    {
        (void)request_stop(p_edges->timestamp_us);
    }
}

static bool stop_requested (void)
{
    UINT saved = 0U;
    bool stop  = false;

    (void)ISpinLock(&g_service_lock, &saved);
    stop = g_inbox.stopped;
    (void)ISpinUnlock(&g_service_lock, saved);
    return stop || drv_stop_pressed();
}

static motion_apply_result_t apply_output (void * context, int16_t left,
                                           int16_t right)
{
    UINT saved = 0U;
    ER   result = E_OK;

    (void)context;
    /* Serialize the final output write with a cross-core stop request. The
     * pin level also covers a falling edge whose interrupt is still pending. */
    (void)ISpinLock(&g_service_lock, &saved);
    if (g_inbox.stopped || drv_stop_pressed())
    {
        latch_stop_locked(now_us());
        (void)ISpinUnlock(&g_service_lock, saved);
        return MOTION_STOP_PENDING;
    }
    result = drv_pwm_set_pair(left, right);
    (void)ISpinUnlock(&g_service_lock, saved);
    if (result == E_OK)
    {
        return MOTION_APPLIED;
    }
    if (result == E_BUSY)
    {
        /* Also covers unexpected legacy-driver activity: explicitly brake
         * before promising RETRY to the controller. */
        return drv_pwm_brake() == E_OK ? MOTION_BRAKED_RETRY
                                         : MOTION_OUTPUT_FAILED;
    }
    return MOTION_OUTPUT_FAILED;
}

static bool brake_output (void * context, bool latch)
{
    (void)context;
    return (latch ? drv_pwm_latch_stop() : drv_pwm_brake()) == E_OK;
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
    ER result = drv_pwm_init();

    if (result == E_OK)
    {
        result = encoder_init();
    }
    drv_stop_init();
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
    if (result == E_OK)
    {
        result = drv_gpio_irq_add_handler(TP_PRC1, DRV_STOP_PIN_MASK, stop_pin_edge);
    }
    return result;
}

/* Latch the output and publish the initialisation error. Never returns. */
static void service_halt (ER result)
{
    UINT saved = 0U;

    /* Owner-only latch if initialisation reached it; boot brake is also
     * asserted because a failed PWM init may not have registered owner. */
    drv_pwm_boot_brake();
    (void)drv_pwm_latch_stop();
    if (g_motion_cyc_id > 0)
    {
        (void)tk_del_cyc(g_motion_cyc_id);
    }
    (void)ISpinLock(&g_service_lock, &saved);
    g_published.init_error    = result;
    g_published.control.state = MOTION_FAULT;
    g_published.control.fault = MOTION_FAULT_OUTPUT;
    g_published.ready         = false;
    (void)ISpinUnlock(&g_service_lock, saved);
    emit_motion_change(MOTION_FAULT, MOTION_FAULT_OUTPUT, g_event_holds);
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/* Set by the supervisor drive check. */
static volatile bool g_encoder_crossed = false;

/* Report the motion fault bits to the shared register: a dead encoder and the
 * latched stop. Other motion faults reach telemetry through the motion packet. */
static void report_faults (motion_fault_t fault, bool stopped)
{
    uint16_t active = 0U;

    if (g_encoder_crossed)
    {
        active |= FAULT_ENCODER_DEAD_BIT;
    }
    if (MOTION_FAULT_FEEDBACK == fault)
    {
        active |= FAULT_ENCODER_DEAD_BIT;
    }
    if (stopped)
    {
        active |= FAULT_ESTOP_LATCHED_BIT;
    }
    system_fault_assign(FAULT_OWNER_MOTION,
                        FAULT_MOTOR_STALL_BIT | FAULT_ENCODER_DEAD_BIT |
                            FAULT_ESTOP_LATCHED_BIT,
                        active);
}

/* Apply a stop requested before the first cycle, then publish the first
 * status. */
static void service_first_status (void)
{
    UINT           saved        = 0U;
    bool const     initial_stop = stop_requested();
    motion_fault_t fault        = MOTION_FAULT_NONE;
    bool           stopped      = false;
    uint8_t        holds        = 0U;

    if (initial_stop)
    {
        controller_tick(&g_controller, NULL, now_us(), NULL, true);
    }
    (void)ISpinLock(&g_service_lock, &saved);
    if (initial_stop)
    {
        latch_stop_locked(now_us());
    }
    g_published.stop_requested = g_inbox.stopped;
    g_published.hold_mask      = (uint8_t)g_inbox.hold_mask;
    g_published.ready =
        !g_inbox.stopped && (g_controller.status.state != MOTION_STOPPED) &&
        (g_controller.status.state != MOTION_FAULT);
    g_published.control        = g_controller.status;
    fault                      = g_controller.status.fault;
    stopped                    = g_inbox.stopped;
    holds                      = (uint8_t)g_inbox.hold_mask;
    (void)ISpinUnlock(&g_service_lock, saved);
    report_faults(fault, stopped);
    emit_motion_change(g_controller.status.state, fault, holds);
}

/* Run permit, guarded by g_service_lock: the supervisor renews it, and the
 * motion task and the supervisor read its age on the same clock. */
static uint64_t g_permit_renewed_us = 0U;
static uint64_t g_permit_lease_us   = 0U;
static bool     g_permit_armed      = false;

#define MOTION_PERMIT_MAX_MS (5000U)

ER motion_service_renew_permit (uint32_t lease_ms)
{
    UINT          saved = 0U;

    if ((0U == lease_ms) || (lease_ms > MOTION_PERMIT_MAX_MS))
    {
        return E_PAR;
    }

    (void)ISpinLock(&g_service_lock, &saved);
    g_permit_renewed_us = drv_timer_now_us64();
    g_permit_lease_us   = (uint64_t)lease_ms * US_PER_MS;
    g_permit_armed      = true;
    (void)ISpinUnlock(&g_service_lock, saved);

    return E_OK;
}

uint32_t motion_service_permit_age_ms (void)
{
    UINT          saved = 0U;
    uint32_t       age  = 0U;

    (void)ISpinLock(&g_service_lock, &saved);
    age = motion_permit_elapsed_ms(g_permit_armed, g_permit_renewed_us, drv_timer_now_us64());
    (void)ISpinUnlock(&g_service_lock, saved);

    return age;
}

/* Hold the car while the permit is lapsed, release the hold when it is renewed. */
static void check_permit (void)
{
    UINT          saved    = 0U;
    uint64_t       now     = 0U;
    bool           b_lapsed = false;

    /* Fail closed: until the supervisor first renews the permit the wheels
     * are held, so a supervisor that never starts never lets the car move. */
    (void)ISpinLock(&g_service_lock, &saved);
    now = drv_timer_now_us64();
    b_lapsed = motion_permit_lapsed(g_permit_armed, g_permit_renewed_us,
                                    g_permit_lease_us, now);
    if (b_lapsed != (0U != (g_inbox.hold_mask & MOTION_HOLD_SUPERVISOR)))
    {
        (void)motion_inbox_hold(&g_inbox, MOTION_HOLD_SUPERVISOR, b_lapsed, (uint32_t)now);
    }
    (void)ISpinUnlock(&g_service_lock, saved);
}

static ER wait_motion_cycle (void)
{
    UINT saved  = 0U;
    INT  queued = 0;
    ER const er = tk_slp_tsk((TMO)MOTION_SLEEP_TIMEOUT_MS);

    queued = tk_can_wup(TSK_SELF);
    if ((E_OK == er) && (queued > 0))
    {
        (void)ISpinLock(&g_service_lock, &saved);
        /* The one urgent wake can coincide with a cyclic wake. Once the
         * stop has latched there is no motion sample to count as missed. */
        if (!g_inbox.stopped)
        {
            g_motion_overruns += (uint32_t)queued;
        }
        (void)ISpinUnlock(&g_service_lock, saved);
    }
    else if (E_TMOUT == er)
    {
        (void)ISpinLock(&g_service_lock, &saved);
        g_motion_overruns++;
        (void)ISpinUnlock(&g_service_lock, saved);
    }
    return er;
}

static bool measured_movement (const controller_status_t * p_control,
                              uint32_t released_us, uint32_t now_us)
{
    motion_sample_t const * const p_sample = &p_control->sample;
    uint32_t const age_us = now_us - p_sample->timestamp_us;
    uint32_t const release_age_us = now_us - released_us;

    return p_control->sample_available && age_us <= MOTION_FEEDBACK_MAX_AGE_US &&
           age_us < release_age_us && p_sample->elapsed_us <= (release_age_us - age_us) &&
           p_sample->wheel[ENCODER_LEFT].speed_valid &&
           p_sample->wheel[ENCODER_RIGHT].speed_valid &&
           (0 != p_sample->wheel[ENCODER_LEFT].velocity_um_s ||
            0 != p_sample->wheel[ENCODER_RIGHT].velocity_um_s);
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
    uint8_t          applied_holds = g_event_holds;
    motion_fault_t const previous_fault = g_controller.status.fault;
    ER const         wake = wait_motion_cycle();

    stop = stop_requested();
    if (stop)
    {
        /* Brake on the urgent wake. Later releases still measure coasting. */
        captured = (MOTION_STOPPED == g_controller.status.state) &&
                   (E_OK == wake) && capture_input(p_input);
        controller_tick(&g_controller, captured ? p_input : NULL,
                            now_us(), NULL,
                            true);
    }
    else if (E_TMOUT == wake)
    {
        service_halt(E_TMOUT);
    }
    else
    {
        uint8_t intake_holds = 0U;

        check_permit();
        captured = capture_input(p_input);
        /* Re-read the priority stop slot after capture, before output. */
        (void)ISpinLock(&g_service_lock, &saved);
        received =
            motion_inbox_take(&g_inbox, now_us(), &command, &stop);
        intake_holds = (uint8_t)g_inbox.hold_mask;
        (void)ISpinUnlock(&g_service_lock, saved);
        stop = stop || drv_stop_pressed();
        controller_tick(&g_controller, captured ? p_input : NULL,
                            now_us(), received ? &command : NULL,
                            stop);
        /* A concurrent hold posted after intake belongs to the next pass. */
        if (!stop && (MOTION_STOPPED != g_controller.status.state))
        {
            applied_holds = intake_holds;
        }
        if (!stop && (MOTION_STOPPED != g_controller.status.state) && stop_requested())
        {
            stop = true;
            controller_tick(&g_controller, NULL, now_us(), NULL, true);
        }
    }
    liveness_kick(LIVE_MOTION);
    (void)ISpinLock(&g_service_lock, &saved);
    /* An output fault can prevent braking, but the observed stop still latches. */
    if (stop)
    {
        latch_stop_locked(now_us());
    }
    g_published.control        = g_controller.status;
    g_published.stop_requested = g_inbox.stopped;
    g_published.hold_mask      = (uint8_t)g_inbox.hold_mask;
    g_published.ready = !g_inbox.stopped &&
                        (g_controller.status.state != MOTION_STOPPED) &&
                        (g_controller.status.state != MOTION_FAULT);
    if (g_published.ready && (0U == g_published.hold_mask) &&
        measured_movement(&g_controller.status, g_inbox.released_us, now_us()))
    {
        g_published.movement_sequence++;
    }
    fault = g_controller.status.fault;
    stop  = g_inbox.stopped;
    (void)ISpinUnlock(&g_service_lock, saved);
    report_faults(fault, stop);
    emit_motion_change(g_controller.status.state, fault, applied_holds);
    if (E_TMOUT == wake)
    {
        service_halt(E_TMOUT);
    }
    if ((fault != previous_fault) && (MOTION_FAULT_FEEDBACK == fault))
    {
        /* Say which check failed: no capture, no sample, a late sample, or
         * invalid quadrature steps on a wheel. */
        motion_sample_t const * const p_sample = &g_controller.status.sample;

        LOG_ERROR(LOG_TASK_MOTION, LOG_MOD_NONE,
                  "feedback captured=%u sample=%u dt_us=%u invalid=%u/%u",
                  (unsigned int)captured, (unsigned int)g_controller.status.sample_available,
                  (unsigned int)p_sample->elapsed_us,
                  (unsigned int)p_sample->wheel[0].invalid_delta,
                  (unsigned int)p_sample->wheel[1].invalid_delta);
    }
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
    drv_pwm_boot_brake();

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
        command.target_um_s[wheel] /= MOTION_SPEED_DIVISOR;
    }
    (void)ISpinLock(&g_service_lock, &saved);
    if (!g_published.ready || g_inbox.stopped)
    {
        result = E_OBJ;
    }
    else
    {
        motion_inbox_result_t admitted = MOTION_INBOX_STALE;
        uint32_t              now      = now_us();

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
                             uint32_t ttl_us, bool b_obstacle_drive)
{
    motion_command_t command = {0U, 0U, ttl_us, {left_um_s, right_um_s}, b_obstacle_drive};

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
    now       = now_us();
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
    /* A pickup or STOP hold during a clearing fault must survive its automatic clear. */
    if (g_inbox.stopped ||
        ((MOTION_FAULT == g_published.control.state) &&
         (!held || !controller_fault_clears_in_place(g_published.control.fault))))
    {
        result = E_OBJ;
    }
    else
    {
        result = motion_inbox_hold(&g_inbox, reason, held, now_us())
                     ? E_OK
                     : E_PAR;
        g_published.hold_mask = (uint8_t)g_inbox.hold_mask;
    }
    (void)ISpinUnlock(&g_service_lock, saved);
    return result;
}

static ER request_stop (uint32_t stamp_us)
{
    UINT saved = 0U;
    bool first = false;

    (void)ISpinLock(&g_service_lock, &saved);
    first = !g_inbox.stopped;
    if (first)
    {
        latch_stop_locked(stamp_us);
    }
    (void)ISpinUnlock(&g_service_lock, saved);
    if (first)
    {
        (void)tk_wup_tsk(g_motion_task_id);
    }
    return E_OK;
}

ER motion_service_request_stop (void)
{
    return g_start_called ? request_stop(now_us()) : E_OBJ;
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

static void fill_feedback (const motion_service_status_t * p_status,
                           motion_feedback_t * p_feedback)
{
    /* The sample was copied before reading the clock. Unsigned subtraction
     * keeps its age across a timer rollover. */
    uint32_t const age_us = now_us() - p_status->control.sample.timestamp_us;
    uint32_t const left_span_us = p_status->control.sample.wheel[ENCODER_LEFT].odometry_span_us;
    uint32_t const right_span_us = p_status->control.sample.wheel[ENCODER_RIGHT].odometry_span_us;

    p_feedback->left_speed_um_s  = p_status->control.sample.wheel[ENCODER_LEFT].velocity_um_s;
    p_feedback->right_speed_um_s = p_status->control.sample.wheel[ENCODER_RIGHT].velocity_um_s;
    p_feedback->left_displacement_um =
        p_status->control.sample.wheel[ENCODER_LEFT].displacement_um;
    p_feedback->right_displacement_um =
        p_status->control.sample.wheel[ENCODER_RIGHT].displacement_um;
    p_feedback->timestamp_us     = p_status->control.sample.timestamp_us;
    p_feedback->age_us           = age_us;
    p_feedback->odometry_span_us = (left_span_us < right_span_us) ? left_span_us : right_span_us;
    p_feedback->stationary_span_us = p_status->control.sample.stationary_span_us;
    p_feedback->speed_valid[ENCODER_LEFT] =
        p_status->control.sample.wheel[ENCODER_LEFT].speed_valid;
    p_feedback->speed_valid[ENCODER_RIGHT] =
        p_status->control.sample.wheel[ENCODER_RIGHT].speed_valid;
    p_feedback->odometry_valid[ENCODER_LEFT] =
        p_status->control.sample.wheel[ENCODER_LEFT].odometry_valid;
    p_feedback->odometry_valid[ENCODER_RIGHT] =
        p_status->control.sample.wheel[ENCODER_RIGHT].odometry_valid;
    p_feedback->invalid_samples[ENCODER_LEFT] =
        p_status->control.sample.wheel[ENCODER_LEFT].invalid_samples;
    p_feedback->invalid_samples[ENCODER_RIGHT] =
        p_status->control.sample.wheel[ENCODER_RIGHT].invalid_samples;
    p_feedback->fresh = (p_status->ready || (MOTION_STOPPED == p_status->control.state)) &&
                        p_status->control.sample_available &&
                        (age_us <= MOTION_FEEDBACK_MAX_AGE_US);
}

static void fill_status (const motion_service_status_t * p_service,
                         motion_status_t * p_status)
{
    p_status->state          = (uint8_t)p_service->control.state;
    p_status->fault          = (uint8_t)p_service->control.fault;
    p_status->ready          = p_service->ready;
    p_status->stop_requested = p_service->stop_requested;
    p_status->hold_mask      = p_service->hold_mask;
    p_status->init_error     = p_service->init_error;
    p_status->stop_age_ms    = p_service->stop_requested
                                  ? (now_us() - p_service->stop_stamp_us) / US_PER_MS
                                  : 0U;
    p_status->movement_sequence = p_service->movement_sequence;
}

static void fill_diag (const motion_service_status_t * p_status, uint32_t overruns,
                       motion_diag_t * p_diag)
{
    p_diag->left_target_um_s    = p_status->control.target_um_s[ENCODER_LEFT];
    p_diag->right_target_um_s   = p_status->control.target_um_s[ENCODER_RIGHT];
    p_diag->left_duty_permille  = p_status->control.duty[ENCODER_LEFT];
    p_diag->right_duty_permille = p_status->control.duty[ENCODER_RIGHT];
    p_diag->overruns            = overruns;
}

ER motion_feedback_snapshot (motion_feedback_t * p_feedback)
{
    motion_service_status_t status;
    ER                      result = E_OK;

    if (p_feedback == NULL)
    {
        return E_PAR;
    }

    result = motion_service_read_status(&status);
    if (E_OK != result)
    {
        return result;
    }
    fill_feedback(&status, p_feedback);

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
    fill_status(&status, p_status);

    return E_OK;
}

ER motion_diag_snapshot (motion_diag_t * p_diag)
{
    motion_service_status_t status;
    ER                      result = E_OK;
    UINT                    saved  = 0U;
    uint32_t                overruns = 0U;

    if (p_diag == NULL)
    {
        return E_PAR;
    }

    result = motion_service_read_status(&status);
    if (E_OK != result)
    {
        return result;
    }
    (void)ISpinLock(&g_service_lock, &saved);
    overruns = g_motion_overruns;
    (void)ISpinUnlock(&g_service_lock, saved);
    fill_diag(&status, overruns, p_diag);

    return E_OK;
}

ER motion_snapshot (motion_status_t * p_status, motion_feedback_t * p_feedback,
                    motion_diag_t * p_diag)
{
    motion_service_status_t status;
    UINT                    saved    = 0U;
    uint32_t                overruns = 0U;

    if ((p_status == NULL) || (p_feedback == NULL))
    {
        return E_PAR;
    }
    if (!g_start_called)
    {
        return E_OBJ;
    }
    (void)ISpinLock(&g_service_lock, &saved);
    status   = g_published;
    overruns = g_motion_overruns;
    (void)ISpinUnlock(&g_service_lock, saved);

    fill_status(&status, p_status);
    fill_feedback(&status, p_feedback);
    if (p_diag != NULL)
    {
        fill_diag(&status, overruns, p_diag);
    }

    return E_OK;
}

ER motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    return motion_service_set_speed(left_um_s, right_um_s, ttl_us, false);
}

ER motion_set_obstacle_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    return motion_service_set_speed(left_um_s, right_um_s, ttl_us, true);
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

uint32_t motion_permit_age_ms (void)
{
    return motion_service_permit_age_ms();
}

void motion_note_encoder_crossed (bool b_crossed)
{
    g_encoder_crossed = b_crossed;
}
