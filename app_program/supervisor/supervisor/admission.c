/** @file admission.c
 * @brief Boot self-test, parked readiness and START admission.
 */

#include "internal.h"

#include "health_core.h"
#include "motion/motion.h"
#include "obstacle/obstacle.h"
#include "imu/imu.h"
#include "platform/log.h"
#include "platform/drv_watchdog.h"
#include "comms/comms.h"
#include "ir/ir.h"

#define OFFLINE_START_MS (10000U)
#define START_BUTTON_DEBOUNCE_MS (2U * SUPERVISOR_PASS_MS)

/* Failed checks in the p2 of the boot event. */
#define SELFTEST_IMU   (1U)
#define SELFTEST_SONAR (2U)
#define SELFTEST_IMU_CAL (8U) /* no rest calibration in the first 2 s */
#define PHYSICS_FAULT_I2C_BIT (1U)

#define RECOVERY_LIMIT_SHIFT (8U)

static bool     g_b_offline_start_pending;
static bool     g_b_watchdog_resume_pending;
static uint8_t  g_run_resumes;
static uint8_t  g_token_flags;
static uint8_t  g_token_resumes;

/* Offline launch and in-run pose recovery cannot both own the stopped car. */
static bool     g_parked_echo_seen;
static uint8_t  g_parked_echo_seq;
static bool     g_b_start_button_down;
static bool     g_b_start_button_pending;
static uint32_t g_start_button_since_ms;
/* Keep the original operator command ID until calibration succeeds or the
 * bounded wait is cancelled. The operator hold remains on throughout. */
static vehicle_cmd_t g_calibration_start;
static uint16_t g_calibration_start_passes;
static uint8_t g_start_rest_interruptions;

static void restore_run_token (void)
{
    uint8_t flags   = 0U;
    uint8_t resumes = 0U;

    g_b_watchdog_resume_pending = false;
    g_run_resumes               = 0U;
    g_token_flags               = 0U;
    g_token_resumes             = 0U;
    if (platform_run_token_read (&flags, &resumes))
    {
        g_token_flags   = flags;
        g_token_resumes = resumes;
        if ((PLATFORM_RESET_WATCHDOG == platform_reset_reason ()) &&
            (0U != (flags & PLATFORM_RUN_ARMED)) &&
            (0U != (flags & PLATFORM_RUN_CONTINUOUS)) &&
            (resumes < SUPERVISOR_WATCHDOG_RESUMES))
        {
            g_b_watchdog_resume_pending = true;
            g_run_resumes               = resumes;
        }
    }
}

void supervisor_admission_init (const supervisor_config_t * p_config)
{
    /* Require a stable release before the first press can start the car. */
    g_b_start_button_down = true;
    g_b_start_button_pending = false;
    g_b_offline_start_pending = (p_config->p_telemetry_seen != NULL) && platform_reset_was_power_up ();
    restore_run_token ();
}

/* A pending resume keeps the token the last reset left, until it runs or is cancelled. */
void supervisor_sync_run_token (bool b_armed, bool b_continuous)
{
    uint8_t const flags = (uint8_t) ((b_armed ? PLATFORM_RUN_ARMED : 0U) |
                                     (b_continuous ? PLATFORM_RUN_CONTINUOUS : 0U));

    if (!g_b_watchdog_resume_pending &&
        ((flags != g_token_flags) || (g_run_resumes != g_token_resumes)))
    {
        platform_run_token_write (flags, g_run_resumes);
        g_token_flags   = flags;
        g_token_resumes = g_run_resumes;
    }
}

/* The boot self-test: what can be checked without moving the car. */
void supervisor_selftest (void)
{
    physics_status_t physics;
    uint32_t         echo[OBSTACLE_ECHO_CLASSES] = { 0U };
    uint16_t         failed = 0U;

    if ((E_OK != physics_snapshot (&physics)) || !physics.ready ||
        (0U != (physics.faults & PHYSICS_FAULT_I2C_BIT)))
    {
        failed |= SELFTEST_IMU;
    }
    else if (!physics.calibrated)
    {
        failed |= SELFTEST_IMU_CAL;
    }
    else
    {
        /* IMU present and calibrated. */
    }
    obstacle_echo_counts (echo);
    if ((echo[OBSTACLE_ECHO_IDX_VALID] + echo[OBSTACLE_ECHO_IDX_OPEN] +
         echo[OBSTACLE_ECHO_IDX_TOO_CLOSE]) == 0U)
    {
        failed |= SELFTEST_SONAR;
    }

    supervisor_emit_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                           (uint8_t) TELEMETRY_EVT_BOOT,
                           (int16_t) platform_reset_reason (), failed);
    supervisor_report_build ();
    if (0U != failed)
    {
        LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "self-test failed, mask=%d", (int) failed);
    }
    else
    {
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "self-test passed");
    }
}

bool supervisor_parked_observation (const run_inputs_t * p_in, uint32_t hold,
                                   uint16_t clear_mm, bool * p_clear)
{
    motion_status_t motion;
    motion_feedback_t feedback;
    obstacle_status_t obstacle;

    if (!p_in->b_upright || p_in->b_attitude_unknown || p_in->b_on_hump ||
        p_in->b_connection_fault || p_in->b_permit_lapsed ||
        (0 != motion_snapshot (&motion, &feedback, NULL)) || !motion.ready ||
        motion.stop_requested || (motion.hold_mask != hold) || (0U != motion.fault) || !feedback.fresh ||
        !feedback.speed_valid[0] || !feedback.speed_valid[1] ||
        (0 != feedback.left_speed_um_s) || (0 != feedback.right_speed_um_s) ||
        (E_OK != obstacle_snapshot (&obstacle)) || !obstacle.ready)
    {
        g_parked_echo_seen = false;
        return false;
    }
    /* The status sequence also advances for side surveys. The ping word
       pairs the range with its own bearing and identifies requested replies. */
    uint32_t const ping = obstacle_ping_read ();
    if (obstacle_pack_ping_is_reply (ping) || (0 != obstacle_pack_ping_servo (ping)))
    {
        g_parked_echo_seen = false;
        return false;
    }
    uint8_t const seq = obstacle_pack_ping_seq (ping);
    bool const b_fresh = g_parked_echo_seen && (seq != g_parked_echo_seq);

    g_parked_echo_seen = true;
    g_parked_echo_seq = seq;
    uint8_t const echo = obstacle_pack_ping_echo (ping);
    uint16_t const mm = obstacle_pack_ping_mm (ping);
    bool const b_range = (LOOK_ECHO_VALID == echo) && (0U != mm);
    *p_clear = (LOOK_ECHO_OPEN == echo) || (b_range && (mm >= clear_mm));
    return b_fresh && (*p_clear || b_range || (LOOK_ECHO_TOO_CLOSE == echo));
}

/* A START the operator did not send: parked, with a fresh clear ping. */
static bool parked_start (run_ctx_t * p_ctx, const run_inputs_t * p_in)
{
    vehicle_cmd_t const cmd = {0U, 0U, VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY};
    run_result_t        result;
    bool                clear = false;

    if (!supervisor_parked_observation (p_in, MOTION_HOLD_OPERATOR, LINE_BUMPER_FAR_MM, &clear) || !clear)
    {
        return false;
    }
    run_state_command (p_ctx, cmd, p_in, &result);
    if (result.b_accepted)
    {
        /* A fresh stationary boot reference is already required above. */
        result.b_physics_recal = false;
        supervisor_apply_result (&result, cmd, true);
    }
    return result.b_accepted;
}

static void consider_offline_start (run_ctx_t * p_ctx, const run_inputs_t * p_in,
                                    bool b_permit_renewed, bool b_critical,
                                    const supervisor_config_t * p_config)
{
    SYSTIM time = {0, 0U};

    if (!g_b_offline_start_pending)
    {
        return;
    }
    if (p_config->p_telemetry_seen () || p_in->b_estop || p_ctx->b_pickup || p_in->b_lifted ||
        p_in->b_overturned || (b_permit_renewed && p_in->b_permit_lapsed))
    {
        g_b_offline_start_pending = false;
    }
    else if (!b_critical && (E_OK == tk_get_otm (&time)) && (time.lo >= OFFLINE_START_MS) &&
             parked_start (p_ctx, p_in))
    {
        g_b_offline_start_pending = false;
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "offline START after telemetry timeout");
    }
    else
    {
        /* Pending until the boot deadline and the readiness checks pass. */
    }
}

static void consider_watchdog_resume (run_ctx_t * p_ctx, const run_inputs_t * p_in,
                                      bool b_permit_renewed, bool b_critical)
{
    if (!g_b_watchdog_resume_pending)
    {
        return;
    }
    /* Before the first renewal there is no permit to lapse. */
    if (p_in->b_estop || p_ctx->b_pickup || p_in->b_lifted || p_in->b_overturned ||
        (b_permit_renewed && p_in->b_permit_lapsed))
    {
        g_b_watchdog_resume_pending = false;
    }
    else if (!b_critical && parked_start (p_ctx, p_in))
    {
        g_b_watchdog_resume_pending = false;
        g_run_resumes++;
        supervisor_emit_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                               (uint8_t) TELEMETRY_EVT_RECOVERY,
                               (int16_t) TELEMETRY_RESET_REASON_WATCHDOG,
                               (uint16_t) ((SUPERVISOR_WATCHDOG_RESUMES << RECOVERY_LIMIT_SHIFT) | g_run_resumes));
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "watchdog reset: the armed run resumes, %d of %d",
                  (int) g_run_resumes, (int) SUPERVISOR_WATCHDOG_RESUMES);
    }
    else
    {
        /* Pending until the car is parked with a clear ping. */
    }
}

void supervisor_poll_start_button (const supervisor_config_t * p_config)
{
    SYSTIM time = {0, 0U};

    if (p_config->p_start_pressed == NULL)
    {
        return;
    }
    bool const b_down = p_config->p_start_pressed ();
    if ((b_down == g_b_start_button_down) || (E_OK != tk_get_otm (&time)))
    {
        g_b_start_button_pending = false;
    }
    else if (!g_b_start_button_pending)
    {
        g_b_start_button_pending = true;
        g_start_button_since_ms = time.lo;
    }
    else if ((time.lo - g_start_button_since_ms) >= START_BUTTON_DEBOUNCE_MS)
    {
        g_b_start_button_down = b_down;
        g_b_start_button_pending = false;
        if (b_down)
        {
            (void) vehicle_command_post (VEHICLE_CMD_START, VEHICLE_SRC_BUTTON);
        }
    }
    else
    {
        /* Wait for the changed level to settle. */
    }
}

static bool wheels_parked_for_start (void)
{
    motion_status_t motion;
    motion_feedback_t feedback;
    return (0 == motion_snapshot (&motion, &feedback, NULL)) && motion.ready &&
           !motion.stop_requested && (0U == motion.fault) && feedback.fresh &&
           feedback.speed_valid[0] && feedback.speed_valid[1] &&
           (0 == feedback.left_speed_um_s) && (0 == feedback.right_speed_um_s) &&
           (0U != (motion.hold_mask & MOTION_HOLD_OPERATOR));
}

static void refuse_calibration_start (uint8_t reason)
{
    run_result_t const refusal = { .b_rejected = true, .reason = reason };
    g_calibration_start_passes = 0U;
    supervisor_apply_result (&refusal, g_calibration_start, true);
}

void supervisor_finish_calibration_start (const run_ctx_t * p_ctx, const run_inputs_t * p_in,
                                          bool b_critical)
{
    if (0U == g_calibration_start_passes)
    {
        return;
    }
    g_calibration_start_passes--;
    if (p_in->b_connection_fault)
    {
        refuse_calibration_start (RUN_REJECT_HARDWARE_FAULT);
    }
    else if ((RUN_STATE_STOPPED != run_state_of (p_ctx)) || p_in->b_estop || p_ctx->b_pickup ||
             p_in->b_lifted || p_in->b_overturned)
    {
        refuse_calibration_start (RUN_REJECT_LATCHED);
    }
    else if (b_critical || p_in->b_permit_lapsed || !wheels_parked_for_start ())
    {
        refuse_calibration_start (RUN_REJECT_STALLED);
    }
    else if (!p_in->b_collecting_rest || p_in->b_on_hump ||
             (p_in->rest_interruptions != g_start_rest_interruptions))
    {
        refuse_calibration_start (RUN_REJECT_NO_ATTITUDE);
    }
    else if (p_in->b_upright && !p_in->b_attitude_unknown)
    {
        run_result_t result;
        g_calibration_start_passes = 0U;
        supervisor_admit_command (g_calibration_start, *p_in, &result);
        /* This START has just acquired its level reference. */
        result.b_physics_recal = false;
        supervisor_apply_result (&result, g_calibration_start, true);
    }
    else if (0U == g_calibration_start_passes)
    {
        refuse_calibration_start (RUN_REJECT_NO_ATTITUDE);
    }
    else
    {
        /* Wait for the complete calibration and level/still windows. */
    }
}

void supervisor_handle_command (const run_ctx_t * p_ctx, vehicle_cmd_t cmd,
                                const run_inputs_t * p_in, bool b_critical)
{
    run_result_t result;
    bool const b_pending = (0U != g_calibration_start_passes);

    if (b_pending && vehicle_is_start (cmd.opcode) && (VEHICLE_SRC_BARCODE != cmd.source))
    {
        /* A repeat cannot extend or duplicate the first request. */
        run_result_t const busy = { .b_rejected = true, .reason = RUN_REJECT_BUSY };
        supervisor_apply_result (&busy, cmd, true);
        return;
    }
    supervisor_admit_command (cmd, *p_in, &result);
    bool const b_wait_for_pose = (RUN_REJECT_NO_ATTITUDE == result.reason) ||
        ((RUN_REJECT_LIFTED == result.reason) && !p_in->b_lifted && !p_in->b_overturned);
    if (vehicle_is_start (cmd.opcode) && result.b_rejected && b_wait_for_pose && !p_ctx->b_pickup &&
        p_in->b_collecting_rest && !p_in->b_permit_lapsed && !b_critical &&
        (RUN_STATE_STOPPED == run_state_of (p_ctx)) && wheels_parked_for_start ())
    {
        g_calibration_start = cmd;
        g_calibration_start_passes = CALHOLD_MAX_PASSES;
        g_start_rest_interruptions = p_in->rest_interruptions;
        g_b_offline_start_pending = false;
        g_b_watchdog_resume_pending = false;
        return;
    }
    if (result.b_accepted)
    {
        g_b_offline_start_pending = false;
        g_b_watchdog_resume_pending = false;
    }
    supervisor_apply_result (&result, cmd, true);
    if (b_pending && result.b_accepted &&
        ((VEHICLE_CMD_STOP == cmd.opcode) || (VEHICLE_CMD_ESTOP == cmd.opcode) ||
         (VEHICLE_CMD_RESET == cmd.opcode)))
    {
        refuse_calibration_start (RUN_REJECT_NOT_RUNNING);
    }
}

void supervisor_consider_parked_starts (run_ctx_t * p_ctx, const run_inputs_t * p_in,
                                       bool b_permit_renewed, bool b_critical,
                                       const supervisor_config_t * p_config)
{
    consider_offline_start (p_ctx, p_in, b_permit_renewed, b_critical, p_config);
    consider_watchdog_resume (p_ctx, p_in, b_permit_renewed, b_critical);
}

void supervisor_reset_parked_observation (void)
{
    g_parked_echo_seen = false;
}

/* A fresh run resets the retry budget. A watchdog resume increments it
 * after the resumed START is admitted. */
void supervisor_note_start_accepted (void)
{
    if (!g_b_watchdog_resume_pending)
    {
        g_run_resumes = 0U;
    }
}

/*** end of file ***/
