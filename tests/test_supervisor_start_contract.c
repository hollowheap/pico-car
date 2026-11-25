/* Exercise supervisor command handling against sensor snapshots.
 * The hardware boundaries are synthetic. Dispatch, run control, and the
 * shared fault register use the application implementations. */

#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "navigation/obstacle_flags.h"
#include "navigation/obstacle_ping.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/watchdog.h"
#include "supervisor/supervisor.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"

#define STOP_INBOX_CAPACITY (8U)

static jmp_buf g_pass_complete;
static uint32_t g_holds;
static unsigned g_fails;
static unsigned g_obstacle_starts;
static unsigned g_obstacle_stops;
static unsigned g_calibration_requests;
static unsigned g_permit_renewals;
static unsigned g_watchdog_feeds;
static unsigned g_supervisor_errors;
static uint32_t g_holds_at_reply;
static uint32_t g_holds_at_run_start;
static uint16_t g_reply_id;
static uint16_t g_reply_detail;
static uint8_t g_reply_type;
static uint8_t g_probe_at_u_reply = SUPERVISOR_NO_TURN;
static bool g_pickup_at_busy_reply;
static uint16_t g_accepted_ids[STOP_INBOX_CAPACITY + 1U];
static unsigned g_accept_count;
static bool g_motion_stop_requested;
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

static void one_pass (void)
{
    if (0 == setjmp (g_pass_complete))
    {
        supervisor_run ();
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
    return 0U;
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

void log_write (log_sev_t severity, log_task_t task, log_mod_t module, const char * p_format, ...)
{
    (void)module;
    (void)p_format;
    if ((LOG_SEV_ERROR == severity) && (LOG_TASK_SUPERVISOR == task))
    {
        g_supervisor_errors++;
    }
}

ER telemetry_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2, uint32_t odometer_mm)
{
    (void)source;
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

static void check_calibrating_start (const char * scenario)
{
    bool const b_settling = (strcmp (scenario, "calibrating-settling") == 0);
    bool const b_floor = (strcmp (scenario, "calibrating-floor") == 0);
    bool const b_stationary = (strcmp (scenario, "calibrating-stationary") == 0);
    bool const b_start = b_stationary || b_floor || b_settling || (strcmp (scenario, "calibrating-start") == 0);
    g_mqtt_seen = true;
    g_physics.attitude_unknown = !b_settling;
    g_physics.calibrated = b_settling;
    g_physics.level_still = false;
    g_physics.collecting_rest = true;
    CHECK (E_OK == vehicle_command_post_ids (b_stationary ? VEHICLE_CMD_START_STATIONARY : (b_floor ? VEHICLE_CMD_START_CONTINUOUS_FLOOR : VEHICLE_CMD_START),
                                             VEHICLE_SRC_TELEMETRY, 27534U, 1U));
    one_pass ();
    CHECK (0U == g_reply_id);
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
        bool continuous_floor = b_stationary;
        CHECK (1U == supervisor_start_snapshot (&continuous_floor) && 1U == g_obstacle_starts && 0U == g_holds);
        CHECK (continuous_floor == !b_stationary);
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
    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    check_connection_refuses_start ();
    set_sonar_connection_fault (0U);
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
    CHECK (0U == supervisor_start_count ());
    CHECK (0U == g_obstacle_starts);
    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
    CHECK (1U == g_obstacle_starts);
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
    remote_start (1U);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);

    g_obstacle.state = TELEMETRY_OBSTACLE_STATE_BYPASS;
    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED;
    one_pass ();
    CHECK (MOTION_HOLD_LIFT == g_holds);
    CHECK (1U == g_obstacle_stops);
    remote_start (2U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (MOTION_HOLD_LIFT == g_holds);

    g_physics.attitude_state = TELEMETRY_ATTITUDE_STATE_OK;
    for (unsigned pass = 0U; pass < 10000U / SUPERVISOR_PASS_MS; pass++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        one_pass ();
        CHECK (MOTION_HOLD_LIFT == g_holds);
    }
    CHECK (1U == supervisor_start_count ());
    CHECK (1U == g_obstacle_stops);
    CHECK (1U == g_obstacle_starts);

    remote_start (3U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (MOTION_HOLD_CALIBRATE == g_holds);
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    one_pass ();
    CHECK (0U == g_holds);
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
    CHECK ((feeds + 2U) == g_watchdog_feeds);
    CHECK ((renewals + 2U) == g_permit_renewals);

    g_motion_fault = fault;
    for (unsigned pass = 0U; pass < 60U; pass++)
    {
        g_now_ms += SUPERVISOR_PASS_MS;
        liveness_kick (LIVE_MOTION);
        one_pass ();
    }
    CHECK ((feeds + 2U) == g_watchdog_feeds);
    CHECK ((renewals + 2U) == g_permit_renewals);
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
    CHECK (0U != g_holds);
    CHECK (SUPERVISOR_NO_TURN == supervisor_poll_turn (NULL));
    if (b_stop_during_loss)
    {
        expect_turn_reply (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, true);
    }
    for (unsigned pass = 0U; pass < 20U; pass++)
    {
        one_pass ();
    }
    CHECK (0U != g_holds);
    CHECK (1U == supervisor_start_count ());
    g_physics.calibrated = true;
    g_physics.attitude_unknown = false;
    g_obstacle.seq++;
    one_pass ();
    CHECK (0U != g_holds);
    g_physics.level_still = true;
    g_wheel_moved = true;
    g_obstacle.seq++;
    one_pass ();
    CHECK (0U != g_holds);
    g_wheel_moved = false;
    g_feedback.speed_valid[1] = false;
    g_obstacle.seq++;
    one_pass ();
    CHECK (0U != g_holds);
    g_feedback.speed_valid[1] = true;
    for (unsigned pass = 0U; pass < 12U; pass++)
    {
        one_pass ();
    }
    CHECK (0U != g_holds);
    g_survey_reply = true;
    g_obstacle.seq++;
    one_pass ();
    CHECK (0U != g_holds);
    g_survey_reply = false;
    g_obstacle.echo_class = TELEMETRY_ECHO_CLASS_NO_ECHO;
    g_obstacle.seq++;
    one_pass ();
    CHECK (0U != g_holds);
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
    bool clear = false;
    if (estop || pickup || overturn || b_stop_during_loss)
    {
        if (estop) CHECK (g_motion_stop_requested);
        else CHECK (0U != (g_holds & (b_stop_during_loss ? MOTION_HOLD_OPERATOR : MOTION_HOLD_LIFT)));
        CHECK (0U == supervisor_recovery_snapshot (&clear));
    }
    else
    {
        CHECK (0U == g_holds);
        CHECK (1U == supervisor_recovery_snapshot (&clear) && clear != near);
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
    CHECK (2U == supervisor_start_count ());
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
                       "running-sonar|running-pickup|historical-sonar|retry-calibration|retry-active-calibration|"
                       "offline-start|online-boot|watchdog-offline|other-reset-offline|offline-stop-before|"
                       "estop-before-report|estop-full-stop-inbox|stop-before-report|"
                       "start-calibration-handoff|watchdog-gap|watchdog-expired|"
                       "watchdog-gap-after-estop|watchdog-expired-after-estop|start-button|start-button-gates\n", argv[0]);
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
    g_physics.ready = true;
    g_physics.level_still = true;
    g_physics.attitude_unknown = (0 == strcmp (argv[1], "retry-calibration"));
    g_physics.calibrated = !g_physics.attitude_unknown;
    CHECK (E_OK == supervisor_init (&config));
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
        set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_TELEMETRY, false);
        set_sonar_connection_fault (0U);
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_TELEMETRY, true);
        CHECK (1U == supervisor_start_snapshot (&floor) && floor);
        expect_turn_reply (VEHICLE_CMD_START_CONTINUOUS_FLOOR, VEHICLE_SRC_TELEMETRY, false);
        expect_turn_reply (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, true);
        CHECK (1U == supervisor_start_snapshot (&floor) && !floor);
        g_physics.calibrated = true;
        g_physics.attitude_unknown = false;
        remote_start (12U);
        CHECK (2U == supervisor_start_snapshot (&floor) && floor);
        expect_turn_reply (VEHICLE_CMD_START_STATIONARY, VEHICLE_SRC_TELEMETRY, false);
        expect_turn_reply (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY, true);
        g_physics.calibrated = true;
        g_physics.attitude_unknown = false;
        expect_turn_reply (VEHICLE_CMD_START_STATIONARY, VEHICLE_SRC_BARCODE, false);
        expect_turn_reply (VEHICLE_CMD_START_STATIONARY, VEHICLE_SRC_TELEMETRY, true);
        CHECK (3U == supervisor_start_snapshot (&floor) && !floor);
        expect_turn_reply (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY, false);
        CHECK (3U == supervisor_start_snapshot (&floor) && !floor);
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
    return (0U == g_fails) ? 0 : 1;
}
