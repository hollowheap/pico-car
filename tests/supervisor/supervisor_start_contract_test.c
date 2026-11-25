/* Exercise supervisor command handling against sensor snapshots.
 * The hardware boundaries are synthetic. Dispatch, run control, and the
 * shared fault register use the application implementations. */

#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "health_core.h"
#include "run_state.h"
#include "motion/motion_state.h"
#include "obstacle/obstacle.h"
#include "obstacle/obstacle_flags.h"
#include "obstacle/obstacle_ping.h"
#include "imu/physics.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/drv_watchdog.h"
#include "supervisor/supervisor.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/wire_schema.h"

#define STOP_INBOX_CAPACITY (8U)

static jmp_buf g_pass_complete;
static uint32_t g_holds;
static unsigned g_fails;
static unsigned g_obstacle_resets;
static unsigned g_obstacle_starts;
static unsigned g_obstacle_stops;
static unsigned g_calibration_requests;
static unsigned g_permit_renewals;
static unsigned g_watchdog_feeds;
static unsigned g_supervisor_errors;
static uint32_t g_holds_at_reply;
static uint32_t g_holds_at_run_start;
static int16_t g_stop_state;
static uint16_t g_stop_reason;
static uint16_t g_reply_id;
static uint16_t g_reply_detail;
static uint8_t g_reply_type;
static uint8_t g_probe_at_u_reply = SUPERVISOR_NO_TURN;
static bool g_pickup_at_busy_reply;
static uint16_t g_accepted_ids[STOP_INBOX_CAPACITY + 1U];
static unsigned g_accept_count;
static bool g_motion_stop_requested;
static uint32_t g_motion_stop_age_ms;
static uint32_t g_motion_movement_sequence;
static bool g_motion_stop_at_reply;
static uint8_t g_motion_fault;
static bool g_wheel_moved;
static motion_feedback_t g_feedback = {.fresh = true, .speed_valid = {true, true}};
static motion_diag_t g_diag;
static bool g_encoder_crossed;
static obstacle_status_t g_obstacle;
static bool g_survey_reply;
static physics_status_t g_physics;
static bool g_delay_calibration;
static uint32_t g_now_ms;
static bool g_mqtt_seen;
static bool g_start_button_down;
static unsigned g_start_button_replies;
static uint8_t g_reset_reason;
static bool g_reset_was_power_up = true;
static bool g_token_valid;
static uint8_t g_token_flags;
static uint8_t g_token_resumes;
static unsigned g_recovery_events;
static unsigned g_boot_events;
static unsigned g_build_events;
static bool g_build_after_boot;
static int16_t g_build_p1;
static uint16_t g_build_p2;
static unsigned g_stop_timing_events;
static int16_t g_stop_timing_p1;
static uint16_t g_stop_timing_p2;
static bool g_telemetry_consumer_blocked;
static bool g_ack_handoff_full;
static unsigned g_blocking_emit_calls;
static unsigned g_ack_handoff_calls;
static unsigned g_ack_handoff_overflows;
typedef struct
{
    uint32_t odometer_mm;
    int16_t  p1;
    uint16_t p2;
    uint8_t  type;
} pending_ack_t;
static pending_ack_t g_pending_acks[TELEMETRY_ACK_RING_CAPACITY];
static unsigned g_pending_ack_count;
static uint8_t g_recovery_source;
static int16_t g_recovery_p1;
static uint16_t g_recovery_p2;

#define CHECK(cond)                                                       \
    do                                                                    \
    {                                                                     \
        if (!(cond))                                                      \
        {                                                                 \
            (void)fprintf (stderr, "FAIL line %d: %s\n", __LINE__, #cond); \
            g_fails++;                                                    \
        }                                                                 \
    } while (0)

ER tk_slp_tsk (TMO timeout)
{
    (void)timeout;
    longjmp (g_pass_complete, 1);
}

ER tk_get_otm (SYSTIM * p_time)
{
    *p_time = (SYSTIM){0, g_now_ms};
    return E_OK;
}

static void drain_pending_acks (void);

static void one_pass (void)
{
    if (0 == setjmp (g_pass_complete))
    {
        supervisor_run ();
    }
    if (!g_telemetry_consumer_blocked)
    {
        drain_pending_acks ();
    }
}

static bool start_button_pressed (void)
{
    return g_start_button_down;
}

static void button_passes (bool b_pressed, unsigned passes)
{
    g_start_button_down = b_pressed;
    while (passes > 0U)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        one_pass ();
        passes--;
    }
}

ER physics_snapshot (physics_status_t * p_status)
{
    *p_status = g_physics;
    return E_OK;
}

uint32_t physics_request_new_run (void)
{
    g_calibration_requests++;
    if (!g_delay_calibration)
    {
        g_physics.reference_request = g_calibration_requests;
        g_physics.calibrated = false;
        g_physics.attitude_unknown = true;
    }
    return g_calibration_requests;
}

void physics_set_wheels_free (bool free)
{
    (void)free;
}

ER obstacle_snapshot (obstacle_status_t * p_status)
{
    *p_status = g_obstacle;
    return E_OK;
}

uint32_t obstacle_ping_read (void)
{
    uint32_t const servo = (uint32_t)(g_obstacle.servo_deci / OBSTACLE_PING_DECI_PER_DEG + OBSTACLE_PING_SERVO_OFFSET);
    return (g_obstacle.seq & OBSTACLE_PING_SEQ_MK) | ((uint32_t)g_obstacle.echo_class << OBSTACLE_PING_CLASS_SH) |
        ((uint32_t)g_obstacle.distance_mm << OBSTACLE_PING_DIST_SH) | (servo << OBSTACLE_PING_SERVO_SH) |
        ((uint32_t)g_survey_reply << OBSTACLE_PING_REPLY_SH);
}

ER obstacle_copy_plan (obstacle_plan_summary_t * p_plan)
{
    (void)p_plan;
    return E_OBJ;
}

void obstacle_echo_counts (uint32_t * p_counts)
{
    /* Retain past replies even during the current no-echo fault. */
    p_counts[OBSTACLE_ECHO_IDX_VALID] = 10U;
}

ER obstacle_post_cmd (uint8_t cmd)
{
    if (OBSTACLE_CMD_RESET == cmd)
    {
        g_obstacle_resets++;
    }
    if (OBSTACLE_CMD_START == cmd)
    {
        g_obstacle_starts++;
    }
    if (OBSTACLE_CMD_STOP == cmd)
    {
        g_obstacle_stops++;
    }
    return E_OK;
}

int motion_set_hold (uint32_t reason, bool held)
{
    g_holds = held ? (g_holds | reason) : (g_holds & ~reason);
    return 0;
}

int motion_status (motion_status_t * p_status)
{
    uint8_t const state = (0U == g_motion_fault) ? MOTION_STATE_IDLE : MOTION_STATE_FAULT;

    *p_status = (motion_status_t){
        .ready = (MOTION_STATE_IDLE == state) && !g_motion_stop_requested,
        .stop_requested = g_motion_stop_requested,
        .stop_age_ms = g_motion_stop_age_ms,
        .movement_sequence = g_motion_movement_sequence,
        .hold_mask = g_holds,
        .state = g_motion_stop_requested ? MOTION_STATE_STOPPED : state,
        .fault = g_motion_fault,
    };
    return 0;
}

int motion_feedback_snapshot (motion_feedback_t * p_feedback)
{
    *p_feedback = g_feedback;
    if (g_wheel_moved)
    {
        p_feedback->left_speed_um_s = 10000;
    }
    return 0;
}

int motion_diag_snapshot (motion_diag_t * p_diag)
{
    *p_diag = g_diag;
    return 0;
}

int motion_snapshot (motion_status_t * p_status, motion_feedback_t * p_feedback,
                     motion_diag_t * p_diag)
{
    (void)motion_status(p_status);
    (void)motion_feedback_snapshot(p_feedback);
    if (p_diag != NULL)
    {
        (void)motion_diag_snapshot(p_diag);
    }
    return 0;
}

int motion_renew_permit (uint32_t lease_ms)
{
    (void)lease_ms;
    g_permit_renewals++;
    return 0;
}

uint32_t motion_permit_age_ms (void)
{
    return (0U == g_permit_renewals) ? UINT32_MAX : 0U;
}

int motion_request_stop (void)
{
    g_motion_stop_requested = true;
    return 0;
}

void motion_note_encoder_crossed (bool crossed)
{
    g_encoder_crossed = crossed;
}

uint8_t platform_reset_reason (void)
{
    return g_reset_reason;
}

bool platform_reset_was_power_up (void)
{
    return g_reset_was_power_up;
}

void platform_wdt_feed (void)
{
    g_watchdog_feeds++;
}

void platform_run_token_write (uint8_t flags, uint8_t resumes)
{
    g_token_valid = true;
    g_token_flags = flags;
    g_token_resumes = resumes;
}

bool platform_run_token_read (uint8_t * p_flags, uint8_t * p_resumes)
{
    *p_flags = g_token_flags;
    *p_resumes = g_token_resumes;
    return g_token_valid;
}

void log_write (log_sev_t severity, log_task_t task, log_mod_t module, const char * p_format, ...)
{
    (void)module;
    (void)p_format;
    if ((LOG_SEV_ERROR == severity) && (LOG_TASK_SUPERVISOR == task))
    {
        g_supervisor_errors++;
    }
}

static ER record_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2, uint32_t odometer_mm)
{
    (void)odometer_mm;
    if ((TELEMETRY_EVT_CMD_ACCEPT == type) || (TELEMETRY_EVT_CMD_REJECT == type))
    {
        g_reply_id = p2;
        g_reply_detail = (uint16_t)p1;
        g_reply_type = type;
        if (g_pickup_at_busy_reply && (TELEMETRY_EVT_CMD_REJECT == type))
        {
            g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED;
            g_pickup_at_busy_reply = false;
        }
        g_motion_stop_at_reply = g_motion_stop_requested;
        g_holds_at_reply = g_holds;
        if (0x0301U == (g_reply_detail & 0x0fffU))
        {
            g_start_button_replies++;
        }
    }
    if (TELEMETRY_EVT_RUN_START == type)
    {
        g_holds_at_run_start = g_holds;
    }
    if (TELEMETRY_EVT_BOOT == type)
    {
        g_boot_events++;
    }
    if (TELEMETRY_EVT_BUILD == type)
    {
        g_build_events++;
        g_build_after_boot = (0U != g_boot_events);
        g_build_p1 = p1;
        g_build_p2 = p2;
    }
    if (TELEMETRY_EVT_STOP_TIMING == type)
    {
        g_stop_timing_events++;
        g_stop_timing_p1 = p1;
        g_stop_timing_p2 = p2;
    }
    if (TELEMETRY_EVT_RECOVERY == type)
    {
        g_recovery_events++;
        g_recovery_source = source;
        g_recovery_p1 = p1;
        g_recovery_p2 = p2;
    }
    if (TELEMETRY_EVT_RUN_STOP == type)
    {
        g_stop_state = p1;
        g_stop_reason = p2;
    }
    if (TELEMETRY_EVT_CMD_ACCEPT == type)
    {
        if ((SUPERVISOR_NO_TURN != g_probe_at_u_reply) && (VEHICLE_CMD_UTURN == ((unsigned)p1 & 0xffU)))
        {
            CHECK (!supervisor_route_pending (g_probe_at_u_reply, true));
        }
        if (g_accept_count < (sizeof (g_accepted_ids) / sizeof (g_accepted_ids[0])))
        {
            g_accepted_ids[g_accept_count] = p2;
        }
        g_accept_count++;
    }
    return E_OK;
}

ER telemetry_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2, uint32_t odometer_mm)
{
    g_blocking_emit_calls++;
    if (g_telemetry_consumer_blocked)
    {
        g_now_ms += 2U * SUPERVISOR_PASS_MS;
    }
    return record_event (source, type, p1, p2, odometer_mm);
}

ER telemetry_try_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2, uint32_t odometer_mm)
{
    CHECK ((TELEMETRY_EVT_CMD_ACCEPT != type) && (TELEMETRY_EVT_CMD_REJECT != type));
    return record_event (source, type, p1, p2, odometer_mm);
}

static void drain_pending_acks (void)
{
    unsigned const count = g_pending_ack_count;
    g_pending_ack_count = 0U;
    for (unsigned idx = 0U; idx < count; idx++)
    {
        (void)record_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                            g_pending_acks[idx].type,
                            g_pending_acks[idx].p1,
                            g_pending_acks[idx].p2,
                            g_pending_acks[idx].odometer_mm);
    }
}

ER telemetry_post_cmd_ack (uint8_t type, int16_t p1, uint16_t p2, uint32_t odometer_mm)
{
    g_ack_handoff_calls++;
    if (g_ack_handoff_full || (g_pending_ack_count >= TELEMETRY_ACK_RING_CAPACITY))
    {
        g_ack_handoff_overflows++;
        return E_TMOUT;
    }
    if (g_telemetry_consumer_blocked)
    {
        g_pending_acks[g_pending_ack_count].odometer_mm = odometer_mm;
        g_pending_acks[g_pending_ack_count].p1 = p1;
        g_pending_acks[g_pending_ack_count].p2 = p2;
        g_pending_acks[g_pending_ack_count].type = type;
        g_pending_ack_count++;
        return E_OK;
    }
    return record_event ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, type, p1, p2, odometer_mm);
}

static void set_sonar_connection_fault (uint16_t fault)
{
    uint32_t active = 0U;

    g_obstacle.faults = fault;
    if (OBSTACLE_FAULT_NO_ECHO == fault)
    {
        active = FAULT_SONAR_NO_ECHO_BIT;
    }
    else if (OBSTACLE_FAULT_ECHO_STUCK == fault)
    {
        active = FAULT_ECHO_LINE_BIT;
    }
    system_fault_assign (FAULT_OWNER_OBSTACLE,
                        FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT, active);
}

static void remote_start (uint16_t cmd_id)
{
    g_reply_id = 0U;
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY,
                                             cmd_id, 1U));
    one_pass ();
    CHECK (cmd_id == g_reply_id);
}

static void test_unverified_start_stays_stopped (void)
{
    bool armed = false;
    remote_start (1U);
    uint8_t const generation = supervisor_armed_start (&armed);
    CHECK (armed);
    supervisor_start_check_failed ((uint8_t)(generation - 1U));
    one_pass ();
    CHECK (generation == supervisor_armed_start (&armed) && armed);

    supervisor_start_check_failed (generation);
    one_pass ();
    CHECK (generation == supervisor_armed_start (&armed) && !armed);
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (RUN_STATE_STOPPED == g_stop_state);
    CHECK (TELEMETRY_RUN_STOP_REASON_STARTUP_UNVERIFIED == g_stop_reason);
    CHECK (0U == (system_fault_get () & FAULT_LIFTED_BIT));
    CHECK (0U == (g_token_flags & PLATFORM_RUN_ARMED));
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (generation == supervisor_armed_start (&armed) && !armed);

    remote_start (2U);
    CHECK ((uint8_t)(generation + 1U) == supervisor_armed_start (&armed) && armed);
    supervisor_start_check_failed (generation);
    one_pass ();
    CHECK ((uint8_t)(generation + 1U) == supervisor_armed_start (&armed) && armed);
}

static void check_calibrating_start (const char * scenario)
{
    bool const b_settling = (strcmp (scenario, "calibrating-settling") == 0);
    bool const b_floor = (strcmp (scenario, "calibrating-floor") == 0);
    bool const b_stationary = (strcmp (scenario, "calibrating-stationary") == 0);
    bool const b_start = b_floor || b_settling || (strcmp (scenario, "calibrating-start") == 0);
    g_mqtt_seen = true;
    g_physics.attitude_unknown = !b_settling;
    g_physics.calibrated = b_settling;
    g_physics.level_still = false;
    g_physics.collecting_rest = true;
    CHECK (E_OK == vehicle_command_post_ids (b_stationary ? 10U : (b_floor ? VEHICLE_CMD_START_CONTINUOUS_FLOOR : VEHICLE_CMD_START),
                                             VEHICLE_SRC_TELEMETRY, 27534U, 1U));
    one_pass ();
    if (b_stationary)
    {
        CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type && 27534U == g_reply_id);
        CHECK (RUN_REJECT_OPCODE == (g_reply_detail >> 12));
    }
    else
    {
        CHECK (0U == g_reply_id);
    }
    CHECK (0U == supervisor_start_count () && 0U == g_obstacle_starts);
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    if (strcmp (scenario, "calibrating-stop") == 0)
    {
        CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_STOP, VEHICLE_SRC_BUTTON));
        one_pass ();
    }
    else if (strcmp (scenario, "calibrating-timeout") == 0)
    {
        button_passes (false, 300U);
        CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type && 27534U == g_reply_id);
    }
    else if (strcmp (scenario, "calibrating-loss") == 0)
    {
        g_physics.ready = false;
        one_pass ();
        g_physics.ready = true;
    }
    else if (strcmp (scenario, "calibrating-wheel") == 0)
    {
        g_wheel_moved = true;
        one_pass ();
        g_wheel_moved = false;
    }
    else if (strcmp (scenario, "calibrating-fault") == 0)
    {
        set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
        one_pass ();
        set_sonar_connection_fault (0U);
    }
    else if (strcmp (scenario, "calibrating-start") == 0)
    {
        remote_start (27535U);
        CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type && 0x2101U == g_reply_detail);
    }
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    g_physics.collecting_rest = true;
    one_pass ();
    CHECK (0U == supervisor_start_count () && 0U != (g_holds & MOTION_HOLD_OPERATOR));
    if (strcmp (scenario, "calibrating-tilted") == 0 || strcmp (scenario, "calibrating-stale") == 0)
    {
        g_physics.collecting_rest = false;
        g_physics.ready = strcmp (scenario, "calibrating-stale") != 0;
        one_pass ();
        CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type && 27534U == g_reply_id);
        g_physics.ready = true;
    }
    if (strcmp (scenario, "calibrating-interrupted") == 0)
    {
        /* Physics saw a tilted sample and restarted its settling window
           between supervisor passes. */
        g_physics.rest_interruptions++;
    }
    g_physics.level_still = true;
    g_physics.collecting_rest = true;
    one_pass ();
    if (b_start)
    {
        CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type && 27534U == g_reply_id);
        bool continuous_floor = false;
        CHECK (1U == supervisor_start_snapshot (&continuous_floor) && 1U == g_obstacle_starts && 0U == g_holds);
        CHECK (continuous_floor);
        CHECK (0U == g_calibration_requests);
    }
    else
    {
        CHECK (0U == supervisor_start_count () && 0U == g_obstacle_starts);
        CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    }
}

static void check_unqualified_settling (bool b_previous_pickup)
{
    if (b_previous_pickup)
    {
        g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED;
        g_physics.level_still = false;
        one_pass ();
    }
    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_OK;
    g_physics.collecting_rest = b_previous_pickup;
    g_physics.level_still = false;
    remote_start (100U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type && 0x4101U == g_reply_detail);
    g_physics.level_still = true;
    g_physics.collecting_rest = true;
    one_pass ();
    CHECK (0U == supervisor_start_count () && 0U != g_holds);
}

static void check_fault_refuses_start (void)
{
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (0U == supervisor_start_count ());
    CHECK (0U == g_obstacle_starts);
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
}

static void check_connection_refuses_start (void)
{
    check_fault_refuses_start ();
    CHECK (0xA101U == g_reply_detail); /* Reason 10, telemetry source, START. */
    CHECK (0U == g_calibration_requests);
}

static void test_start_after_sonar_recovers (void)
{
    CHECK (0U == g_obstacle_resets && 0U == g_obstacle_starts);
    g_mqtt_seen = true;
    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    check_connection_refuses_start ();
    CHECK (0U == g_obstacle_resets);
    set_sonar_connection_fault (0U);
    g_now_ms = 10000U;
    one_pass ();
    g_now_ms += SUPERVISOR_PASS_MS;
    g_obstacle.seq++;
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (0U == supervisor_start_count ());
    CHECK (0U == g_obstacle_resets && 0U == g_obstacle_starts);
    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
    CHECK (1U == g_obstacle_resets && 1U == g_obstacle_starts);
    CHECK (0U == (g_holds & MOTION_HOLD_OPERATOR));
}

static void test_sonar_fault_does_not_add_supervisor_hold_midrun (void)
{
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);

    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    one_pass ();
    CHECK (0U == g_holds);
    CHECK (1U == supervisor_start_count ());
}

static void test_faulty_imu_refuses_start (void)
{
    /* A current connection fault can precede loss of the rest reference. */
    g_physics.faults = (uint8_t) (1U << TELEMETRY_PHYSICS_FAULTS_BIT_I2C);
    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, FAULT_IMU_NACK_BIT);
    check_connection_refuses_start ();

    g_physics.attitude_unknown = true;
    check_connection_refuses_start ();

    CHECK (0U != (system_fault_sticky_get () & FAULT_IMU_NACK_BIT));
    /* The detector retains its own I2C history after recovery. */
    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, 0U);
    g_physics.attitude_unknown = false;
    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
    CHECK (0U == (system_fault_sticky_get () & FAULT_IMU_NACK_BIT));
}

static void expect_turn_reply (uint8_t cmd, uint8_t source, bool accepted)
{
    CHECK (E_OK == vehicle_command_post (cmd, source));
    one_pass ();
    CHECK (g_reply_type == (accepted ? TELEMETRY_EVT_CMD_ACCEPT : TELEMETRY_EVT_CMD_REJECT));
    CHECK (((uint32_t) cmd | ((uint32_t) source << 8U)) == (g_reply_detail & 0x0fffU));
}

static void test_pending_barcode_route (void)
{
    uint8_t source = 0U;
    remote_start (1U);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();

    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE));
    one_pass ();
    CHECK (VEHICLE_CMD_LEFT == supervisor_poll_turn (&source));
    CHECK (VEHICLE_SRC_BARCODE == source);
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_RIGHT, VEHICLE_SRC_TELEMETRY));
    one_pass ();
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (VEHICLE_CMD_LEFT == supervisor_poll_turn (&source));
    CHECK (supervisor_route_pending (VEHICLE_CMD_LEFT, true));
    CHECK (!supervisor_route_pending (VEHICLE_CMD_LEFT, true));
    CHECK (SUPERVISOR_NO_TURN == supervisor_poll_turn (&source));

    static uint8_t const choices[] = {VEHICLE_CMD_LEFT, VEHICLE_CMD_RIGHT, VEHICLE_CMD_STRAIGHT};
    for (size_t index = 0; index < sizeof (choices); index++)
    {
        uint8_t const choice = choices[index];
        expect_turn_reply (choice, VEHICLE_SRC_BARCODE, true);
        CHECK (choice == supervisor_poll_turn (&source));
        CHECK (choice == supervisor_poll_turn (&source));
        CHECK (VEHICLE_SRC_BARCODE == source);
        CHECK (!supervisor_route_pending (choices[(index + 1U) % sizeof (choices)], true));
        CHECK (supervisor_route_pending (choice, false));
        supervisor_set_turning (true); /* Junction probe started, branch still unselected. */
        expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_TELEMETRY, false);
        g_obstacle.state = TELEMETRY_OBSTACLE_STATE_BYPASS;
        expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, false);
        g_obstacle.state = TELEMETRY_OBSTACLE_STATE_MONITOR;
        g_holds |= MOTION_HOLD_OBSTACLE;
        expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, false);
        g_holds &= ~MOTION_HOLD_OBSTACLE;
        g_probe_at_u_reply = choice;
        expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, true);
        g_probe_at_u_reply = SUPERVISOR_NO_TURN;
        CHECK (!supervisor_route_pending (VEHICLE_CMD_UTURN, true));
        supervisor_set_turning (false); /* The displaced probe has unwound. */
        CHECK (VEHICLE_CMD_UTURN == supervisor_poll_turn (&source));
        CHECK (SUPERVISOR_NO_TURN == supervisor_poll_turn (&source));
        expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, false);
        supervisor_set_turning (false);
    }

    expect_turn_reply (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE, true);
    supervisor_set_turning (true);
    CHECK (supervisor_route_pending (VEHICLE_CMD_LEFT, true)); /* Branch committed. */
    expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, false);
    supervisor_set_turning (false);

    expect_turn_reply (VEHICLE_CMD_RIGHT, VEHICLE_SRC_TELEMETRY, true);
    expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, false);
    CHECK (VEHICLE_CMD_RIGHT == supervisor_poll_turn (&source));
    CHECK (VEHICLE_SRC_TELEMETRY == source);
    expect_turn_reply (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE, false);
    supervisor_turn_refused (VEHICLE_CMD_RIGHT);

    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE));
    one_pass ();
    g_pickup_at_busy_reply = true;
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY));
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE));
    one_pass ();
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (supervisor_route_pending (VEHICLE_CMD_LEFT, false));
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_LIFT));
    CHECK (SUPERVISOR_NO_TURN == supervisor_poll_turn (&source));
    CHECK (!supervisor_route_pending (VEHICLE_CMD_LEFT, true));
}

static void test_pickup_during_run (void)
{
    uint32_t const pickup_holds = MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT;
    remote_start (1U);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);

    g_obstacle.state = TELEMETRY_OBSTACLE_STATE_BYPASS;
    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED;
    one_pass ();
    CHECK (pickup_holds == g_holds);
    CHECK (RUN_STATE_STOPPED == g_stop_state);
    CHECK (0U != (system_fault_get () & FAULT_LIFTED_BIT));
    CHECK (1U == g_obstacle_stops);
    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (pickup_holds == g_holds);

    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_OK;
    for (unsigned pass = 0U; pass < 10000U / SUPERVISOR_PASS_MS; pass++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        one_pass ();
        CHECK (pickup_holds == g_holds);
        CHECK (0U != (system_fault_get () & FAULT_LIFTED_BIT));
    }
    CHECK (1U == supervisor_start_count ());
    CHECK (1U == g_obstacle_stops);
    CHECK (1U == g_obstacle_starts);

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY, 3U, 1U));
    one_pass ();
    CHECK (3U == g_reply_id && TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (pickup_holds == g_holds);
    CHECK (0U != (system_fault_get () & FAULT_LIFTED_BIT));
    CHECK (0U != (system_fault_sticky_get () & FAULT_LIFTED_BIT));

    remote_start (4U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (MOTION_HOLD_CALIBRATE == g_holds);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);
    CHECK (0U == (system_fault_get () & FAULT_LIFTED_BIT));
    CHECK (0U == (system_fault_sticky_get () & FAULT_LIFTED_BIT));
    CHECK (2U == supervisor_start_count ());
}

static void test_historical_sonar_fault_does_not_refuse_start (void)
{
    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    set_sonar_connection_fault (0U);
    CHECK (0U != (system_fault_sticky_get () & FAULT_SONAR_NO_ECHO_BIT));
    CHECK (0U == (system_fault_get () & FAULT_SONAR_NO_ECHO_BIT));
    system_fault_assign (FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, FAULT_LINE_LOST_BIT);
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
    CHECK (0U == (system_fault_sticky_get () & FAULT_SONAR_NO_ECHO_BIT));
    CHECK (0U != (system_fault_sticky_get () & FAULT_LINE_LOST_BIT));
}

static void wait_for_calibration_timeout (void)
{
    unsigned pass = 0U;

    for (pass = 0U; pass < (5100U / SUPERVISOR_PASS_MS); pass++)
    {
        one_pass ();
    }
    CHECK (0U != (g_holds & MOTION_HOLD_CALIBRATE));
}

static void test_start_retries_calibration_after_timeout (void)
{
    unsigned requests;

    wait_for_calibration_timeout ();
    requests = g_calibration_requests;

    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, FAULT_IMU_NACK_BIT);
    check_connection_refuses_start ();
    CHECK (requests == g_calibration_requests);
    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, 0U);

    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (0x9101U == g_reply_detail);
    CHECK ((requests + 1U) == g_calibration_requests);
    CHECK (0U != (g_holds & MOTION_HOLD_CALIBRATE));
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (0U == supervisor_start_count ());
    CHECK (0U == g_obstacle_starts);

    remote_start (3U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK ((requests + 1U) == g_calibration_requests);

    g_physics.attitude_unknown = false;
    g_physics.calibrated = true;
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (0U == (g_holds & MOTION_HOLD_CALIBRATE));
    CHECK (0U == supervisor_start_count ());

    remote_start (4U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
}

static void test_retry_revokes_timed_out_start (void)
{
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (0U == (g_holds & MOTION_HOLD_OPERATOR));
    wait_for_calibration_timeout ();

    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (2U == g_calibration_requests);
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (1U == g_obstacle_stops);
    CHECK (1U == g_obstacle_starts);

    g_physics.attitude_unknown = false;
    g_physics.calibrated = true;
    one_pass ();
    CHECK (0U == (g_holds & MOTION_HOLD_CALIBRATE));
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (1U == supervisor_start_count ());

    remote_start (3U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (2U == supervisor_start_count ());
}

static void check_offline_start (void)
{
    g_now_ms = 9999U;
    one_pass ();
    CHECK (0U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));

    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    g_now_ms = 10000U;
    one_pass ();
    CHECK (0U == supervisor_start_count ());

    set_sonar_connection_fault (0U);
    one_pass ();
    CHECK (0U == supervisor_start_count ());
    g_now_ms += 50U;
    g_obstacle.seq++;
    one_pass ();
    bool floor = false;
    CHECK (1U == supervisor_start_snapshot (&floor) && floor);
    CHECK (0U == (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (1U == g_obstacle_starts);

    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_STOP, VEHICLE_SRC_BUTTON));
    one_pass ();
    g_now_ms += 10000U;
    one_pass ();
    CHECK (1U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
}

static void check_online_boot_then_drop (void)
{
    g_mqtt_seen = true;
    g_now_ms = 300U;
    one_pass ();
    g_now_ms = 10000U;
    one_pass ();
    CHECK (0U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
}

static void check_stop_cancels_offline_start (void)
{
    g_now_ms = 4000U;
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_STOP, VEHICLE_SRC_BUTTON));
    one_pass ();
    g_now_ms = 20000U;
    one_pass ();
    CHECK (0U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
}

static bool telemetry_seen (void)
{
    return g_mqtt_seen;
}

/* Parked, level passes with a fresh clear centred ping each, as the offline
 * start needs. A watchdog resume waits for no telemetry timeout. */
#define RESUME_PASSES   (5U)
#define RESUME_PASS_MS  (50U)

static void parked_passes (unsigned passes)
{
    while (passes > 0U)
    {
        g_now_ms += RESUME_PASS_MS;
        g_obstacle.seq++;
        one_pass ();
        passes--;
    }
}

static uint16_t resume_attempt (uint8_t attempt)
{
    return (uint16_t)(((uint32_t)SUPERVISOR_WATCHDOG_RESUMES << 8U) | attempt);
}

/* The run token says the car was running when the watchdog reset it. */
static void check_watchdog_resume (bool b_continuous, uint8_t resumes)
{
    bool floor = !b_continuous;

    parked_passes (RESUME_PASSES);
    CHECK (1U == supervisor_start_snapshot (&floor));
    CHECK (b_continuous == floor);
    CHECK (0U == (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (1U == g_recovery_events);
    CHECK ((uint8_t)TELEMETRY_EVENT_SOURCE_SUPERVISOR == g_recovery_source);
    CHECK ((int16_t)TELEMETRY_RESET_REASON_WATCHDOG == g_recovery_p1);
    CHECK (resume_attempt ((uint8_t)(resumes + 1U)) == g_recovery_p2);
    CHECK (g_token_valid && (0U != (g_token_flags & PLATFORM_RUN_ARMED)));
    CHECK ((uint8_t)(resumes + 1U) == g_token_resumes);

    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_STOP, VEHICLE_SRC_BUTTON));
    one_pass ();
    CHECK (0U == (g_token_flags & PLATFORM_RUN_ARMED));
}

/* No resume: the car keeps its operator hold through the same passes, and a
 * telemetry timeout cannot start it either. */
static void check_no_watchdog_resume (void)
{
    g_now_ms = 20000U;
    parked_passes (RESUME_PASSES);
    CHECK (0U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (0U == g_recovery_events);
}

/* The test target is built with this hash, a dirty tree and the pilot. */
#define BUILD_HASH        (0x1234ABCU)
#define BUILD_HASH_HIGH   (12U)
#define BUILD_FLAGS_SHIFT (12U)
#define BUILD_SELFTEST_PASSES (120U)

static void check_build_identity (void)
{
    uint32_t const flags = TELEMETRY_BUILD_FLAG_DIRTY | TELEMETRY_BUILD_FLAG_PILOT;
    uint32_t const high = (BUILD_HASH >> 16U) & ((1U << BUILD_HASH_HIGH) - 1U);

    for (unsigned i = 0U; i < BUILD_SELFTEST_PASSES; i++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        one_pass ();
    }
    CHECK (1U == g_boot_events);
    CHECK (1U == g_build_events);
    CHECK (g_build_after_boot);
    CHECK ((int16_t)((flags << BUILD_FLAGS_SHIFT) | high) == g_build_p1);
    CHECK ((uint16_t)(BUILD_HASH & 0xFFFFU) == g_build_p2);
}

/* A STOP is accepted while both wheels drive. The duty reaches 0 after
 * STOP_DUTY_PASSES passes and the wheels rest after STOP_REST_PASSES. */
#define STOP_DUTY_PASSES (2U)
#define STOP_REST_PASSES (5U)
#define STOP_SPEED_UM_S  (150000)
#define STOP_DUTY_PM     (300)

static void timed_passes (unsigned passes)
{
    while (passes > 0U)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        one_pass ();
        passes--;
    }
}

static void check_stop_timing (uint8_t opcode)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_feedback.left_speed_um_s = STOP_SPEED_UM_S;
    g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = STOP_DUTY_PM;
    g_diag.right_duty_permille = STOP_DUTY_PM;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);

    CHECK (E_OK == vehicle_command_post (opcode, VEHICLE_SRC_TELEMETRY));
    timed_passes (1U);
    timed_passes (STOP_DUTY_PASSES - 1U);
    g_diag.left_duty_permille = 0;
    g_diag.right_duty_permille = 0;
    timed_passes (STOP_REST_PASSES - STOP_DUTY_PASSES - 1U);
    CHECK (0U == g_stop_timing_events);
    g_feedback.left_speed_um_s = 0;
    g_feedback.right_speed_um_s = 0;
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    timed_passes (STOP_REST_PASSES);
    CHECK (1U == g_stop_timing_events);
    CHECK ((int16_t)(STOP_DUTY_PASSES * SUPERVISOR_PASS_MS) == g_stop_timing_p1);
    CHECK ((uint16_t)(STOP_REST_PASSES * SUPERVISOR_PASS_MS) == g_stop_timing_p2);
}

static void check_overlapping_stop_timing (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    unsigned const accepted_before = g_accept_count;

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY, 10U, 1U));
    timed_passes (1U);
    CHECK (g_accept_count == accepted_before + 2U);
    CHECK (9U == g_accepted_ids[accepted_before]);
    CHECK (10U == g_accepted_ids[accepted_before + 1U]);

    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    timed_passes (1U);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = 0;
    for (unsigned pass = 0U; pass < 3U; ++pass)
    {
        g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
        timed_passes (1U);
    }
    CHECK (g_stop_timing_events == 1U);
    CHECK (g_stop_timing_p1 == (int16_t)(2U * SUPERVISOR_PASS_MS));
}

static void check_stale_rest_is_ignored (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY));
    timed_passes (1U);

    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = 0;
    g_feedback.fresh = false;
    timed_passes (3U);
    CHECK (g_stop_timing_events == 0U);
    g_feedback.fresh = true;
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (2U);
    CHECK (g_stop_timing_events == 0U);
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (g_stop_timing_events == 1U);
}

static void check_gp21_stop_timing (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_motion_stop_requested = true;
    g_motion_stop_age_ms = 3U * SUPERVISOR_PASS_MS;
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    CHECK (9U == g_reply_id && TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (0U == g_stop_timing_events);

    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = 0;
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (1U == g_stop_timing_events);
    CHECK ((int16_t)(3U * SUPERVISOR_PASS_MS) == g_stop_timing_p1);
    CHECK ((uint16_t)(5U * SUPERVISOR_PASS_MS) == g_stop_timing_p2);
}

static void check_stop_timing_without_brake_evidence (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    g_motion_fault = TELEMETRY_MOTION_FAULT_OUTPUT;
    g_feedback.fresh = false;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = 0;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);

    g_now_ms += UINT16_MAX;
    one_pass ();
    CHECK (1U == g_stop_timing_events);
    CHECK (INT16_MAX == g_stop_timing_p1 && UINT16_MAX == g_stop_timing_p2);
}

static void complete_stop_with_fresh_samples (void)
{
    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = 0;
    for (unsigned i = 0; i < 3U; i++)
    {
        g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
        timed_passes (1U);
    }
}

static void check_stop_episode_rearms_after_motion (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_motion_movement_sequence++;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    timed_passes (1U);

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    complete_stop_with_fresh_samples ();
    CHECK (g_stop_timing_events == 1U);

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 10U, 1U));
    timed_passes (1U);
    complete_stop_with_fresh_samples ();
    CHECK (g_reply_id == 10U && g_reply_type == TELEMETRY_EVT_CMD_ACCEPT);
    CHECK (g_stop_timing_events == 1U);

    remote_start (11U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_feedback.left_displacement_um += 10000;
    g_feedback.right_displacement_um += 10000;
    g_motion_movement_sequence++;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    g_feedback.timestamp_us = (g_now_ms + SUPERVISOR_PASS_MS) * 1000U;
    timed_passes (1U);
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 12U, 1U));
    timed_passes (1U);
    complete_stop_with_fresh_samples ();
    CHECK (g_stop_timing_events == 2U);
}

static void check_stop_timing_after_duty_returns (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    timed_passes (1U);

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = 0;
    g_feedback.timestamp_us = (g_now_ms + SUPERVISOR_PASS_MS) * 1000U;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);

    remote_start (10U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    g_feedback.timestamp_us += 2U * SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);

    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (0U == g_stop_timing_events);
    g_feedback.timestamp_us += SUPERVISOR_PASS_MS * 1000U;
    timed_passes (1U);
    CHECK (1U == g_stop_timing_events);
    CHECK ((int16_t) SUPERVISOR_PASS_MS == g_stop_timing_p1);
}

static void check_no_rearm_from_pre_release_motion (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_motion_movement_sequence++;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    timed_passes (1U);
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    CHECK (0U != g_holds);
    g_diag.left_duty_permille = g_diag.right_duty_permille = 0;
    g_feedback.fresh = false;
    g_now_ms += UINT16_MAX;
    one_pass ();
    CHECK (1U == g_stop_timing_events && UINT16_MAX == g_stop_timing_p2);

    remote_start (10U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (0U != g_holds);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    g_feedback.fresh = true;
    g_feedback.timestamp_us = (g_now_ms + SUPERVISOR_PASS_MS) * 1000U - 1000U;
    timed_passes (1U);
    CHECK (0U == g_holds);
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 11U, 1U));
    timed_passes (1U);
    CHECK (21U == g_now_ms - g_feedback.timestamp_us / 1000U);
    CHECK (11U == g_reply_id && TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    complete_stop_with_fresh_samples ();
    CHECK (1U == g_stop_timing_events);
}

static void check_estop_between_passes (bool b_moved)
{
    g_mqtt_seen = true;
    remote_start (8U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_motion_movement_sequence++;
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    timed_passes (1U);
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    complete_stop_with_fresh_samples ();
    CHECK (1U == g_stop_timing_events);

    remote_start (10U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_now_ms += 15U;
    g_feedback.timestamp_us = g_now_ms * 1000U;
    if (b_moved)
    {
        g_motion_movement_sequence++;
        g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
        g_feedback.left_displacement_um += STOP_SPEED_UM_S / 100;
        g_feedback.right_displacement_um += STOP_SPEED_UM_S / 100;
    }

    /* GP21 latches after the encoder interval and before the supervisor wakes. */
    g_motion_stop_requested = true;
    g_now_ms += 5U;
    g_motion_stop_age_ms = 5U;
    one_pass ();
    complete_stop_with_fresh_samples ();
    CHECK ((b_moved ? 2U : 1U) == g_stop_timing_events);
    if (b_moved)
    {
        CHECK (5 == g_stop_timing_p1);
        CHECK ((uint16_t)(5U + 2U * SUPERVISOR_PASS_MS) == g_stop_timing_p2);
    }
}

static void check_stop_with_slow_telemetry_consumer (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;
    uint32_t const start_ms = g_now_ms;
    unsigned const accepted_before = g_accept_count;

    g_telemetry_consumer_blocked = true;
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    timed_passes (1U);
    CHECK (g_motion_stop_requested);
    CHECK (g_now_ms - start_ms == SUPERVISOR_PASS_MS);
    CHECK (g_accept_count == accepted_before);
    CHECK (1U == g_pending_ack_count);
    g_telemetry_consumer_blocked = false;
    timed_passes (2U);
    CHECK (0U == g_pending_ack_count);
    CHECK (g_accept_count == accepted_before + 1U);
    CHECK (g_accepted_ids[accepted_before] == 9U);
}

static void check_saturated_ack_handoff_stop (void)
{
    g_mqtt_seen = true;
    remote_start (8U);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    timed_passes (1U);
    CHECK (0U == g_holds);
    g_feedback.left_speed_um_s = g_feedback.right_speed_um_s = STOP_SPEED_UM_S;
    g_diag.left_duty_permille = g_diag.right_duty_permille = STOP_DUTY_PM;

    uint32_t const start_ms = g_now_ms;
    unsigned const handoffs_before = g_ack_handoff_calls;

    g_telemetry_consumer_blocked = true;
    g_ack_handoff_full = true;
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 9U, 1U));
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY, 10U, 1U));
    timed_passes (1U);
    CHECK (g_motion_stop_requested);
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (g_now_ms - start_ms == SUPERVISOR_PASS_MS);
    CHECK (g_ack_handoff_calls == handoffs_before + 2U);
    CHECK (2U == g_ack_handoff_overflows);
    CHECK (0U == g_blocking_emit_calls);

    g_ack_handoff_full = false;
    g_telemetry_consumer_blocked = false;
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, 11U, 1U));
    timed_passes (1U);
    CHECK (11U == g_reply_id && TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
}

static void check_stationary_start_is_refused (void)
{
    g_mqtt_seen = true;
    CHECK (E_OK == vehicle_command_post_ids (10U, VEHICLE_SRC_TELEMETRY, 42U, 1U));
    one_pass ();
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type && 42U == g_reply_id);
    CHECK (RUN_REJECT_OPCODE == (g_reply_detail >> 12));
    CHECK (0U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
}

/* An operator START arms the token with no resumes; a STOP disarms it. */
static void check_run_token (void)
{
    g_mqtt_seen = true;
    remote_start (7U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (g_token_valid);
    CHECK ((PLATFORM_RUN_ARMED | PLATFORM_RUN_CONTINUOUS) == g_token_flags);
    CHECK (0U == g_token_resumes);
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_STOP, VEHICLE_SRC_BUTTON));
    one_pass ();
    CHECK (0U == (g_token_flags & PLATFORM_RUN_ARMED));
}

static void check_estop_precedes_reply (void)
{
    uint16_t const cmd_id = 101U;

    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_ESTOP, VEHICLE_SRC_BARCODE));
    one_pass ();
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (!g_motion_stop_requested);

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY,
                                             cmd_id, 1U));
    one_pass ();
    CHECK (g_motion_stop_requested);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (cmd_id == g_reply_id);
    CHECK (g_motion_stop_at_reply);
}

/* The healthy and feedback-fault passes before the timing fault, and the
 * passes a persistent timing fault runs past its window. */
#define TIMING_EARLY_PASSES  (2U)
#define TIMING_LATE_PASSES   (60U)

static void check_motion_timing_fault (uint8_t fault, bool b_stopped)
{
    unsigned const feeds = g_watchdog_feeds;
    unsigned const renewals = g_permit_renewals;

    g_motion_stop_requested = b_stopped;
    liveness_kick (LIVE_MOTION);
    one_pass ();
    CHECK ((feeds + 1U) == g_watchdog_feeds);
    CHECK ((renewals + 1U) == g_permit_renewals);

    /* A feedback fault keeps the motion task alive without requiring a reboot. */
    g_motion_fault = TELEMETRY_MOTION_FAULT_FEEDBACK;
    liveness_kick (LIVE_MOTION);
    one_pass ();
    CHECK ((feeds + TIMING_EARLY_PASSES) == g_watchdog_feeds);
    CHECK ((renewals + TIMING_EARLY_PASSES) == g_permit_renewals);

    /* A timing fault that clears inside its window keeps the run and the feed. */
    g_motion_fault = fault;
    unsigned const window = MOTION_TIMING_RECOVERY_MS / SUPERVISOR_PASS_MS;
    for (unsigned pass = 0U; pass < window; pass++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        liveness_kick (LIVE_MOTION);
        one_pass ();
    }
    CHECK ((feeds + TIMING_EARLY_PASSES + window) == g_watchdog_feeds);
    CHECK ((renewals + TIMING_EARLY_PASSES + window) == g_permit_renewals);
    CHECK (0U == g_supervisor_errors);

    /* One that stays past the window stops the feed, so the car reboots. */
    for (unsigned pass = 0U; pass < TIMING_LATE_PASSES; pass++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        liveness_kick (LIVE_MOTION);
        one_pass ();
    }
    CHECK ((feeds + TIMING_EARLY_PASSES + 1U + window) >= g_watchdog_feeds);
    unsigned const stopped_feeds = g_watchdog_feeds;
    unsigned const stopped_renewals = g_permit_renewals;
    one_pass ();
    CHECK (stopped_feeds == g_watchdog_feeds);
    CHECK (stopped_renewals == g_permit_renewals);
    CHECK (1U == g_supervisor_errors);
}

static void check_physics_watchdog (void)
{
    /* A responsive task can retry the bus locally for longer than the board timeout. */
    g_physics.attitude_unknown = true;
    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, FAULT_IMU_NACK_BIT);
    unsigned feeds = g_watchdog_feeds;
    for (unsigned pass = 0U; pass < 100U; pass++)
    {
        liveness_kick (LIVE_PHYSICS);
        one_pass ();
    }
    CHECK (feeds + 100U == g_watchdog_feeds);
    for (unsigned pass = 0U; pass < 35U; pass++)
    {
        one_pass ();
    }
    feeds = g_watchdog_feeds;
    for (unsigned pass = 0U; pass < 10U; pass++)
    {
        one_pass ();
    }
    CHECK (feeds == g_watchdog_feeds);
    for (unsigned pass = 0U; pass < 6U; pass++)
    {
        liveness_kick (LIVE_PHYSICS);
        one_pass ();
    }
    CHECK (g_watchdog_feeds > feeds);
}

static void check_running_reference_loss (const char * scenario)
{
    bool const b_stop_during_loss = 0 == strcmp (scenario, "lost-reference-stop");
    bool const near = 0 == strcmp (scenario, "lost-reference-near");
    remote_start (1U);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);
    /* A single bus failure leaves the calibrated run intact. */
    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, FAULT_IMU_NACK_BIT);
    one_pass ();
    CHECK (0U == g_holds);
    system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_IMU_NACK_BIT, 0U);
    expect_turn_reply (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE, true);
    g_physics.calibrated = false;
    g_physics.level_still = false;
    g_physics.attitude_unknown = true;
    one_pass ();
    /* Mid-run IMU reference loss drives on without holding (S139). */
    CHECK (0U == g_holds);
    if (b_stop_during_loss)
    {
        expect_turn_reply (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, true);
    }
    for (unsigned pass = 0U; pass < 20U; pass++)
    {
        one_pass ();
    }
    CHECK (b_stop_during_loss ? (0U != g_holds) : (0U == g_holds));
    CHECK (1U == supervisor_start_count ());
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    g_physics.level_still = true;
    bool const pickup = 0 == strcmp (scenario, "lost-reference-pickup");
    bool const overturn = 0 == strcmp (scenario, "lost-reference-overturn");
    bool const estop = 0 == strcmp (scenario, "lost-reference-estop");
    if (pickup || overturn)
    {
        g_physics.attitude_state = overturn ? TELEMETRY_ATTITUDE_STATE_OVERTURNED : TELEMETRY_ATTITUDE_STATE_LIFTED;
        g_physics.level_still = false;
        one_pass ();
        g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_OK;
        g_physics.level_still = true;
    }
    if (estop)
    {
        expect_turn_reply (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY, true);
    }
    g_obstacle.echo_class = (uint8_t)(near ? TELEMETRY_ECHO_CLASS_VALID : TELEMETRY_ECHO_CLASS_OPEN);
    g_obstacle.distance_mm = near ? 100U : 0U;
    g_obstacle.seq++;
    one_pass ();
    if (estop || pickup || overturn || b_stop_during_loss)
    {
        if (estop) CHECK (g_motion_stop_requested);
        else CHECK (0U != (g_holds & (b_stop_during_loss ? MOTION_HOLD_OPERATOR : MOTION_HOLD_LIFT)));
    }
    else
    {
        CHECK (0U == g_holds);
    }
    CHECK (1U == supervisor_start_count ());
}

static void check_drive_feedback_validity (void)
{
    remote_start (1U);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);
    g_diag.left_target_um_s = 180000;
    g_diag.right_target_um_s = 100000;
    g_feedback.left_speed_um_s = 100000;
    g_feedback.right_speed_um_s = 180000;
    for (unsigned valid = 0U; valid < 3U; valid++)
    {
        g_feedback.speed_valid[0] = valid != 0U;
        g_feedback.speed_valid[1] = valid != 1U;
        for (unsigned pass = 0U; pass < 125U; pass++)
        {
            one_pass ();
        }
        CHECK (g_encoder_crossed == (valid == 2U));
    }
}

static void check_delayed_upright_start (void)
{
    g_delay_calibration = true;
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    for (unsigned pass = 0U; pass < 18U; pass++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        g_physics.cycles++;
        one_pass ();
        CHECK (MOTION_HOLD_CALIBRATE == g_holds);
    }
    g_physics.reference_request = g_calibration_requests;
    one_pass ();
    CHECK (0U == g_holds);
}

static void check_start_calibration_hold_handoff (void)
{
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == g_calibration_requests);
    CHECK (0U != (g_holds_at_reply & MOTION_HOLD_CALIBRATE));
    CHECK (0U != (g_holds_at_run_start & MOTION_HOLD_CALIBRATE));
    CHECK (MOTION_HOLD_CALIBRATE == g_holds);

    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);

    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_OVERTURNED;
    g_physics.level_still = false;
    one_pass ();
    CHECK (MOTION_HOLD_LIFT == g_holds);
    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    g_physics.level_still = true;
    g_delay_calibration = true;
    remote_start (3U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK ((MOTION_HOLD_LIFT | MOTION_HOLD_CALIBRATE) == g_holds_at_reply);
    CHECK (MOTION_HOLD_CALIBRATE == g_holds);
    CHECK (2U == supervisor_start_count ());
    CHECK (0U == supervisor_reset_count ());
    for (unsigned pass = 0U; pass < 60U; pass++)
    {
        one_pass ();
        CHECK (MOTION_HOLD_CALIBRATE == g_holds);
    }
    g_physics.reference_request = g_calibration_requests;
    g_physics.calibrated = false;
    g_physics.attitude_unknown = true;
    one_pass ();
    CHECK (MOTION_HOLD_CALIBRATE == g_holds);
    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_OK;
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);
}

static void check_stop_precedes_reply (void)
{
    uint32_t const bits[] = { MOTION_HOLD_OPERATOR, MOTION_HOLD_BARCODE };
    uint8_t const sources[] = { VEHICLE_SRC_TELEMETRY, VEHICLE_SRC_BARCODE };

    /* Preserve the existing running reference on this hump. */
    g_physics.hump_active = true;
    for (uint16_t idx = 0U; idx < 2U; idx++)
    {
        uint16_t const cmd_id = (uint16_t)(idx + 101U);

        remote_start ((uint16_t)(idx + 1U));
        CHECK (0U == g_holds);
        CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, sources[idx], cmd_id, 1U));
        one_pass ();
        CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
        CHECK (cmd_id == g_reply_id);
        CHECK (0U != (g_holds_at_reply & bits[idx]));
    }
}

static void check_estop_with_full_stop_inbox (void)
{
    uint16_t const estop_id = 101U;
    uint16_t const next_id  = 102U;
    uint16_t       stop_id  = 0U;

    for (stop_id = 1U; stop_id <= STOP_INBOX_CAPACITY; stop_id++)
    {
        CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY,
                                                 stop_id, 1U));
    }
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY,
                                             estop_id, 1U));
    CHECK (0U == supervisor_cmd_drops ());
    CHECK (E_TMOUT == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY,
                                                next_id, 1U));
    CHECK (1U == supervisor_cmd_drops ());

    one_pass ();
    CHECK (g_motion_stop_requested);
    CHECK ((STOP_INBOX_CAPACITY + 1U) == g_accept_count);
    CHECK (estop_id == g_accepted_ids[0]);
    for (stop_id = 1U; stop_id <= STOP_INBOX_CAPACITY; stop_id++)
    {
        CHECK (stop_id == g_accepted_ids[stop_id]);
    }

    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY,
                                             next_id, 1U));
    one_pass ();
    CHECK ((STOP_INBOX_CAPACITY + 2U) == g_accept_count);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (next_id == g_reply_id);
}

static void check_start_button_debounce (void)
{
    button_passes (true, 6U);
    button_passes (false, 1U);
    button_passes (true, 6U);
    CHECK (0U == g_start_button_replies);
    CHECK (0U == supervisor_start_count ());

    button_passes (false, 3U);
    button_passes (true, 2U);
    button_passes (false, 1U);
    button_passes (true, 1U);
    one_pass ();
    one_pass ();
    CHECK (0U == g_start_button_replies);
    button_passes (true, 1U);
    CHECK (0U == g_start_button_replies);
    button_passes (true, 1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (0x0301U == g_reply_detail);
    CHECK (0U == g_reply_id);
    bool floor = false;
    CHECK (1U == supervisor_start_snapshot (&floor) && floor);
    CHECK (1U == g_obstacle_starts);

    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    button_passes (true, 6U);
    button_passes (false, 1U);
    button_passes (true, 6U);
    CHECK (1U == g_start_button_replies);
    CHECK (1U == supervisor_start_count ());

    button_passes (false, 3U);
    g_now_ms = UINT32_MAX - SUPERVISOR_PASS_MS;
    button_passes (true, 3U);
    CHECK (2U == g_start_button_replies);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
}

static void check_start_button_gates (void)
{
    button_passes (false, 3U);
    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    button_passes (true, 3U);
    CHECK (1U == g_start_button_replies);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (0xa301U == g_reply_detail);
    CHECK (0U == supervisor_start_count ());
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));

    set_sonar_connection_fault (0U);
    button_passes (true, 6U);
    CHECK (1U == g_start_button_replies);
    CHECK (0U == supervisor_start_count ());

    button_passes (false, 3U);
    button_passes (true, 2U);
    CHECK (E_OK == vehicle_command_post (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY));
    button_passes (true, 1U);
    CHECK (2U == g_start_button_replies);
    CHECK (g_motion_stop_at_reply);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (0x3301U == g_reply_detail);
    CHECK (0U == supervisor_start_count ());
    CHECK (0U == g_obstacle_starts);
}

int main (int argc, char ** argv)
{
    supervisor_config_t config = { .p_telemetry_seen = telemetry_seen };

    if (2 != argc)
    {
        (void)fprintf (stderr, "Usage: %s faulty-sonar|echo-line|faulty-imu|restored-sonar|"
                       "running-sonar|running-pickup|unverified-start|historical-sonar|retry-calibration|retry-active-calibration|"
                       "offline-start|online-boot|watchdog-offline|other-reset-offline|offline-stop-before|"
                       "estop-before-report|estop-full-stop-inbox|stop-before-report|"
                       "start-calibration-handoff|watchdog-gap|watchdog-expired|"
                       "watchdog-gap-after-estop|watchdog-expired-after-estop|start-button|start-button-gates|"
                       "watchdog-resume[-bounded|-used|-stopped|-corrupt|-power]|run-token|build-identity|stop-timing|estop-timing\n", argv[0]);
        return 2;
    }
    if ((0 == strcmp (argv[1], "start-button")) || (0 == strcmp (argv[1], "start-button-gates")))
    {
        config.p_start_pressed = start_button_pressed;
        g_start_button_down = (0 == strcmp (argv[1], "start-button"));
        g_mqtt_seen = true;
    }
    g_obstacle.ready = true;
    if (0 == strcmp (argv[1], "watchdog-physics"))
    {
        config.required_mask = (uint16_t)(1U << LIVE_PHYSICS);
    }
    g_obstacle.state = TELEMETRY_OBSTACLE_STATE_MONITOR;
    g_obstacle.echo_class = TELEMETRY_ECHO_CLASS_OPEN;
    if (0 == strcmp (argv[1], "watchdog-offline"))
    {
        g_reset_reason = PLATFORM_RESET_WATCHDOG;
        g_reset_was_power_up = false;
    }
    else if (0 == strcmp (argv[1], "other-reset-offline"))
    {
        /* The legacy two-value wire report says POWER for other hardware resets. */
        g_reset_was_power_up = false;
    }
    else if (0 == strncmp (argv[1], "watchdog-resume", 15U))
    {
        g_reset_reason = PLATFORM_RESET_WATCHDOG;
        g_reset_was_power_up = false;
        g_mqtt_seen = true;
        g_token_valid = (0 != strcmp (argv[1], "watchdog-resume-corrupt"));
        g_token_flags = (uint8_t)(PLATFORM_RUN_ARMED | PLATFORM_RUN_CONTINUOUS);
        if (0 == strcmp (argv[1], "watchdog-resume-bounded"))
        {
            g_token_resumes = (uint8_t)(SUPERVISOR_WATCHDOG_RESUMES - 1U);
        }
        else if (0 == strcmp (argv[1], "watchdog-resume-reject-stationary"))
        {
            g_token_flags = PLATFORM_RUN_ARMED;
            g_token_resumes = (uint8_t)(SUPERVISOR_WATCHDOG_RESUMES - 1U);
        }
        else if (0 == strcmp (argv[1], "watchdog-resume-used"))
        {
            g_token_resumes = SUPERVISOR_WATCHDOG_RESUMES;
        }
        else if (0 == strcmp (argv[1], "watchdog-resume-stopped"))
        {
            g_token_flags = PLATFORM_RUN_CONTINUOUS;
        }
        else if (0 == strcmp (argv[1], "watchdog-resume-power"))
        {
            g_reset_reason = PLATFORM_RESET_POWER;
            g_reset_was_power_up = true;
        }
        else
        {
            /* A first resume on the continuous floor. */
        }
    }
    g_physics.ready = true;
    g_physics.level_still = true;
    g_physics.attitude_unknown = (0 == strcmp (argv[1], "retry-calibration"));
    g_physics.calibrated = !g_physics.attitude_unknown;
    CHECK (E_OK == supervisor_setup (&config));
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));

    if (0 == strncmp (argv[1], "calibrating-", 12U))
    {
        check_calibrating_start (argv[1]);
    }
    else if (0 == strcmp (argv[1], "continuous-floor"))
    {
        bool floor = true;
        CHECK (0U == supervisor_start_snapshot (&floor) && !floor);
        /* Only an operator may choose the test floor for the next run. */
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_BARCODE, false);
        CHECK (RUN_REJECT_SOURCE == (g_reply_detail >> 12));
        set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_TELEMETRY, false);
        set_sonar_connection_fault (0U);
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_TELEMETRY, true);
        CHECK (1U == supervisor_start_snapshot (&floor) && floor);
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_TELEMETRY, false);
        CHECK (RUN_REJECT_BUSY == (g_reply_detail >> 12));
        expect_turn_reply (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, true);
        CHECK (1U == supervisor_start_snapshot (&floor) && !floor);
        g_physics.calibrated = true;
        g_physics.attitude_unknown = false;
        remote_start (12U);
        CHECK (2U == supervisor_start_snapshot (&floor) && floor);
        expect_turn_reply (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY, false);
        CHECK (RUN_REJECT_BUSY == (g_reply_detail >> 12));
        expect_turn_reply (10U, VEHICLE_SRC_TELEMETRY, false);
        CHECK (RUN_REJECT_OPCODE == (g_reply_detail >> 12));
        CHECK (2U == supervisor_start_snapshot (&floor) && floor);
        expect_turn_reply (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, true);
        g_physics.calibrated = true;
        g_physics.attitude_unknown = false;
        expect_turn_reply (10U, VEHICLE_SRC_BARCODE, false);
        CHECK (RUN_REJECT_OPCODE == (g_reply_detail >> 12));
        expect_turn_reply (10U, VEHICLE_SRC_TELEMETRY, false);
        CHECK (RUN_REJECT_OPCODE == (g_reply_detail >> 12));
        CHECK (2U == supervisor_start_snapshot (&floor) && !floor);
    }
    else if (0 == strcmp (argv[1], "settling-pickup") || 0 == strcmp (argv[1], "settling-tilted"))
    {
        check_unqualified_settling (0 == strcmp (argv[1], "settling-pickup"));
    }
    else if (0 == strcmp (argv[1], "route-slot"))
    {
        test_pending_barcode_route ();
    }
    else if (0 == strcmp (argv[1], "faulty-sonar"))
    {
        set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
        check_connection_refuses_start ();
    }
    else if (0 == strcmp (argv[1], "echo-line"))
    {
        set_sonar_connection_fault (OBSTACLE_FAULT_ECHO_STUCK);
        check_connection_refuses_start ();
    }
    else if (0 == strcmp (argv[1], "faulty-imu"))
    {
        test_faulty_imu_refuses_start ();
    }
    else if (0 == strcmp (argv[1], "restored-sonar"))
    {
        test_start_after_sonar_recovers ();
    }
    else if (0 == strcmp (argv[1], "running-sonar"))
    {
        test_sonar_fault_does_not_add_supervisor_hold_midrun ();
    }
    else if (0 == strcmp (argv[1], "running-pickup"))
    {
        test_pickup_during_run ();
    }
    else if (0 == strcmp (argv[1], "unverified-start"))
    {
        test_unverified_start_stays_stopped ();
    }
    else if (0 == strcmp (argv[1], "historical-sonar"))
    {
        test_historical_sonar_fault_does_not_refuse_start ();
    }
    else if (0 == strcmp (argv[1], "retry-calibration"))
    {
        test_start_retries_calibration_after_timeout ();
    }
    else if (0 == strcmp (argv[1], "retry-active-calibration"))
    {
        test_retry_revokes_timed_out_start ();
    }
    else if (0 == strcmp (argv[1], "offline-start"))
    {
        check_offline_start ();
    }
    else if (0 == strcmp (argv[1], "online-boot"))
    {
        check_online_boot_then_drop ();
    }
    else if ((0 == strcmp (argv[1], "watchdog-offline")) ||
             (0 == strcmp (argv[1], "other-reset-offline")))
    {
        g_now_ms = 20000U;
        one_pass ();
        g_now_ms += 50U;
        g_obstacle.seq++;
        one_pass ();
        CHECK (0U == supervisor_start_count ());
        CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    }
    else if (0 == strcmp (argv[1], "offline-stop-before"))
    {
        check_stop_cancels_offline_start ();
    }
    else if (0 == strcmp (argv[1], "watchdog-resume"))
    {
        check_watchdog_resume (true, 0U);
    }
    else if (0 == strcmp (argv[1], "watchdog-resume-bounded"))
    {
        check_watchdog_resume (true, (uint8_t)(SUPERVISOR_WATCHDOG_RESUMES - 1U));
    }
    else if ((0 == strcmp (argv[1], "watchdog-resume-used")) ||
             (0 == strcmp (argv[1], "watchdog-resume-stopped")) ||
             (0 == strcmp (argv[1], "watchdog-resume-corrupt")) ||
             (0 == strcmp (argv[1], "watchdog-resume-power")) ||
             (0 == strcmp (argv[1], "watchdog-resume-reject-stationary")))
    {
        check_no_watchdog_resume ();
    }
    else if (0 == strcmp (argv[1], "run-token"))
    {
        check_run_token ();
    }
    else if (0 == strcmp (argv[1], "build-identity"))
    {
        check_build_identity ();
    }
    else if (0 == strcmp (argv[1], "stop-timing"))
    {
        check_stop_timing (VEHICLE_CMD_STOP);
    }
    else if (0 == strcmp (argv[1], "estop-timing"))
    {
        check_stop_timing (VEHICLE_CMD_ESTOP);
    }
    else if (0 == strcmp (argv[1], "overlapping-stop-timing"))
    {
        check_overlapping_stop_timing ();
    }
    else if (0 == strcmp (argv[1], "stale-rest-is-ignored"))
    {
        check_stale_rest_is_ignored ();
    }
    else if (0 == strcmp (argv[1], "gp21-stop-timing"))
    {
        check_gp21_stop_timing ();
    }
    else if (0 == strcmp (argv[1], "stop-timing-without-brake-evidence"))
    {
        check_stop_timing_without_brake_evidence ();
    }
    else if (0 == strcmp (argv[1], "stop-episode-rearms-after-motion"))
    {
        check_stop_episode_rearms_after_motion ();
    }
    else if (0 == strcmp (argv[1], "stop-timing-after-duty-returns"))
    {
        check_stop_timing_after_duty_returns ();
    }
    else if (0 == strcmp (argv[1], "no-rearm-from-pre-release-motion"))
    {
        check_no_rearm_from_pre_release_motion ();
    }
    else if (0 == strcmp (argv[1], "between-pass-estop-after-motion"))
    {
        check_estop_between_passes (true);
    }
    else if (0 == strcmp (argv[1], "between-pass-estop-without-motion"))
    {
        check_estop_between_passes (false);
    }
    else if (0 == strcmp (argv[1], "stop-with-slow-telemetry-consumer"))
    {
        check_stop_with_slow_telemetry_consumer ();
    }
    else if (0 == strcmp (argv[1], "saturated-ack-handoff-stop"))
    {
        check_saturated_ack_handoff_stop ();
    }
    else if (0 == strcmp (argv[1], "stationary-start-is-refused"))
    {
        check_stationary_start_is_refused ();
    }
    else if (0 == strcmp (argv[1], "estop-before-report"))
    {
        check_estop_precedes_reply ();
    }
    else if (0 == strcmp (argv[1], "estop-full-stop-inbox"))
    {
        check_estop_with_full_stop_inbox ();
    }
    else if (0 == strcmp (argv[1], "stop-before-report"))
    {
        check_stop_precedes_reply ();
    }
    else if (0 == strcmp (argv[1], "delayed-upright-start"))
    {
        check_delayed_upright_start ();
    }
    else if (0 == strcmp (argv[1], "start-calibration-handoff"))
    {
        check_start_calibration_hold_handoff ();
    }
    else if (0 == strcmp (argv[1], "drive-feedback-validity"))
    {
        check_drive_feedback_validity ();
    }
    else if (0 == strncmp (argv[1], "lost-reference", 14U))
    {
        check_running_reference_loss (argv[1]);
    }
    else if (0 == strcmp (argv[1], "watchdog-physics"))
    {
        check_physics_watchdog ();
    }
    else if (0 == strcmp (argv[1], "watchdog-gap"))
    {
        check_motion_timing_fault (TELEMETRY_MOTION_FAULT_GAP, false);
    }
    else if (0 == strcmp (argv[1], "watchdog-expired"))
    {
        check_motion_timing_fault (TELEMETRY_MOTION_FAULT_EXPIRED, false);
    }
    else if (0 == strcmp (argv[1], "watchdog-gap-after-estop"))
    {
        check_motion_timing_fault (TELEMETRY_MOTION_FAULT_GAP, true);
    }
    else if (0 == strcmp (argv[1], "watchdog-expired-after-estop"))
    {
        check_motion_timing_fault (TELEMETRY_MOTION_FAULT_EXPIRED, true);
    }
    else if (0 == strcmp (argv[1], "start-button"))
    {
        check_start_button_debounce ();
    }
    else if (0 == strcmp (argv[1], "start-button-gates"))
    {
        check_start_button_gates ();
    }
    else
    {
        (void)fprintf (stderr, "Unknown case: %s\n", argv[1]);
        return 2;
    }
    CHECK (0U == g_blocking_emit_calls);
    return (0U == g_fails) ? 0 : 1;
}
