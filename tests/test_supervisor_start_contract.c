/* Exercise remote START through the supervisor task against sensor snapshots.
 * The hardware boundaries are synthetic. Dispatch, run control, and the
 * shared fault register use the application implementations. */

#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "navigation/obstacle_flags.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/log.h"
#include "platform/watchdog.h"
#include "supervisor/supervisor.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"

static jmp_buf g_pass_complete;
static uint32_t g_holds;
static unsigned g_fails;
static unsigned g_obstacle_starts;
static unsigned g_calibration_requests;
static uint16_t g_reply_id;
static uint8_t g_reply_type;
static obstacle_status_t g_obstacle;
static physics_status_t g_physics;

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

static void one_pass (void)
{
    if (0 == setjmp (g_pass_complete))
    {
        supervisor_run ();
    }
}

ER physics_snapshot (physics_status_t * p_status)
{
    *p_status = g_physics;
    return E_OK;
}

void physics_request_new_run (void)
{
    g_calibration_requests++;
    g_physics.calibrated = false;
    g_physics.attitude_unknown = true;
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
    return E_OK;
}

int motion_set_hold (uint32_t reason, bool held)
{
    g_holds = held ? (g_holds | reason) : (g_holds & ~reason);
    return 0;
}

int motion_status (motion_status_t * p_status)
{
    *p_status = (motion_status_t){ .hold_mask = g_holds };
    return 0;
}

int motion_feedback_snapshot (motion_feedback_t * p_feedback)
{
    *p_feedback = (motion_feedback_t){0};
    return 0;
}

int motion_diag_snapshot (motion_diag_t * p_diag)
{
    *p_diag = (motion_diag_t){0};
    return 0;
}

int motion_renew_permit (uint32_t lease_ms)
{
    (void)lease_ms;
    return 0;
}

uint32_t motion_permit_age_ms (void)
{
    return 0U;
}

int motion_request_stop (void)
{
    return 0;
}

void motion_note_encoder_crossed (bool crossed)
{
    (void)crossed;
}

uint8_t platform_reset_reason (void)
{
    return PLATFORM_RESET_POWER;
}

void platform_wdt_feed (void)
{
}

void log_write (log_sev_t severity, log_task_t task, log_mod_t module, const char * p_format, ...)
{
    (void)severity;
    (void)task;
    (void)module;
    (void)p_format;
}

ER telemetry_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2, uint32_t odometer_mm)
{
    (void)source;
    (void)p1;
    (void)odometer_mm;
    if ((TELEMETRY_EVT_CMD_ACCEPT == type) || (TELEMETRY_EVT_CMD_REJECT == type))
    {
        g_reply_id = p2;
        g_reply_type = type;
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
    CHECK (E_OK == vehicle_command_post_ids (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY,
                                             cmd_id, 1U));
    one_pass ();
    CHECK (cmd_id == g_reply_id);
}

static void check_fault_refuses_start (void)
{
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_REJECT == g_reply_type);
    CHECK (0U == supervisor_start_count ());
    CHECK (0U == g_obstacle_starts);
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
}

static void test_start_after_sonar_recovers (void)
{
    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    check_fault_refuses_start ();
    set_sonar_connection_fault (0U);
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));
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
    check_fault_refuses_start ();
}

static void test_historical_sonar_fault_does_not_refuse_start (void)
{
    set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
    set_sonar_connection_fault (0U);
    CHECK (0U != (system_fault_sticky_get () & FAULT_SONAR_NO_ECHO_BIT));
    CHECK (0U == (system_fault_get () & FAULT_SONAR_NO_ECHO_BIT));
    remote_start (1U);
    CHECK (TELEMETRY_EVT_CMD_ACCEPT == g_reply_type);
    CHECK (1U == supervisor_start_count ());
}

static void test_start_retries_calibration_after_timeout (void)
{
    unsigned pass = 0U;
    unsigned requests;

    for (pass = 0U; pass < (5100U / SUPERVISOR_PASS_MS); pass++)
    {
        one_pass ();
    }
    CHECK (0U != (g_holds & MOTION_HOLD_CALIBRATE));
    requests = g_calibration_requests;

    check_fault_refuses_start ();
    CHECK ((requests + 1U) == g_calibration_requests);
    CHECK (0U != (g_holds & MOTION_HOLD_CALIBRATE));
}

int main (int argc, char ** argv)
{
    supervisor_config_t const config = {0};

    if (2 != argc)
    {
        (void)fprintf (stderr, "Usage: %s faulty-sonar|echo-line|faulty-imu|restored-sonar|"
                       "running-sonar|historical-sonar|retry-calibration\n", argv[0]);
        return 2;
    }
    g_obstacle.ready = true;
    g_obstacle.state = TELEMETRY_OBSTACLE_STATE_MONITOR;
    g_physics.ready = true;
    g_physics.attitude_unknown = (0 == strcmp (argv[1], "retry-calibration"));
    g_physics.calibrated = !g_physics.attitude_unknown;
    CHECK (E_OK == supervisor_init (&config));
    one_pass ();
    CHECK (0U != (g_holds & MOTION_HOLD_OPERATOR));

    if (0 == strcmp (argv[1], "faulty-sonar"))
    {
        set_sonar_connection_fault (OBSTACLE_FAULT_NO_ECHO);
        check_fault_refuses_start ();
    }
    else if (0 == strcmp (argv[1], "echo-line"))
    {
        set_sonar_connection_fault (OBSTACLE_FAULT_ECHO_STUCK);
        check_fault_refuses_start ();
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
    else if (0 == strcmp (argv[1], "historical-sonar"))
    {
        test_historical_sonar_fault_does_not_refuse_start ();
    }
    else if (0 == strcmp (argv[1], "retry-calibration"))
    {
        test_start_retries_calibration_after_timeout ();
    }
    else
    {
        (void)fprintf (stderr, "Unknown case: %s\n", argv[1]);
        return 2;
    }
    return (0U == g_fails) ? 0 : 1;
}
