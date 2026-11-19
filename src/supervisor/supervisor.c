/*
 *	supervisor.c
 *	The supervisor task: mission control and health.
 *	See supervisor/supervisor.h.
 *
 *	The decisions are in run_state.c and health_core.c, which a host test
 *	drives. This file reads the facts from the other subsystems, calls those
 *	functions and applies the result.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/smp.h>
#include <tk/tkernel.h>

#include "command_ring.h"
#include "edge_events.h"
#include "health_core.h"
#include "run_state.h"
#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "navigation/obstacle_flags.h"
#include "physics/physics.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/watchdog.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"

/* One liveness check every 5 passes (100 ms). */
#define HEALTH_PASSES   (5U)
#define HEALTH_CHECK_MS (SUPERVISOR_PASS_MS * HEALTH_PASSES)

/* The background hook runs every 50 passes (1 s). */
#define BACKGROUND_PASSES (50U)

/* The boot self-test reads its facts after 100 passes (2 s). */
#define SELFTEST_PASSES (100U)

/* Failed checks in the p2 of the boot event. */
#define SELFTEST_IMU   (1U)
#define SELFTEST_SONAR (2U)
#define SELFTEST_BEAM  (4U)
#define SELFTEST_IMU_CAL (8U) /* no rest calibration in the first 2 s */
#define PHYSICS_FAULT_I2C_BIT (1U)

/* The run permit lease. The motion task holds the car when it lapses. */
#define PERMIT_LEASE_MS (200U)

/* The motion task faults GAP when its 10 ms control loop misses 100 ms
   (MOTOR_CONTROL_GAP_US in motor_setup.c). */
#define MOTION_GAP_MS (100U)

/* Watchdog timeout, when the build turns the watchdog on. It must outlast the
   software stop that follows the last feed, so a reset never comes while the
   wheels are still driven. A critical task that stops kicking is stalled after
   LIVENESS_MISS_LIMIT checks, one per HEALTH_CHECK_MS (5 x 100 ms = 500 ms);
   the supervisor then sends task_stall and stops the feed and the permit. The
   permit lapses PERMIT_LEASE_MS (200 ms) after the last renew; the motion loop
   (10 ms) holds the wheels on its next turn, and a turn more than
   MOTION_GAP_MS (100 ms) late faults GAP. One more check (100 ms) lets a task
   that kicks again restore the feed. 200 + 100 + 100 = 400 ms < 1000 ms; the
   board resets about 1.5 s after the last kick. */
#define WATCHDOG_TIMEOUT_MS (1000U)
STATIC_ASSERT (WATCHDOG_TIMEOUT_MS > (PERMIT_LEASE_MS + MOTION_GAP_MS + HEALTH_CHECK_MS),
               "the watchdog outlasts the software stop");

/* run_state.c counts its passes in supervisor passes. */
STATIC_ASSERT (RUN_PASS_MS == SUPERVISOR_PASS_MS, "run_state counts supervisor passes");

/* Event p1 of cmd_accept and cmd_reject: reason << 12 | source << 8 | opcode.
   p2 is the cmd_id. */
#define CMD_EVENT_SOURCE_SHIFT (8U)
#define CMD_EVENT_REASON_SHIFT (12U)

/* The task sleeps at most this long when the cyclic handler does not wake it. */
#define SLEEP_TIMEOUT_MS (SUPERVISOR_PASS_MS + SUPERVISOR_PASS_MS)

/* The wheels must move at least this fast for the progress check to expect travel. */
#define PROGRESS_MIN_TARGET_UM_S (10000)

#define UM_PER_MM (1000)
#define MS_PER_S  (1000U)

static T_SPLOCK            g_lock;
static command_ring_t      g_ring;
static run_ctx_t           g_ctx;
static liveness_t          g_live;
static progress_t          g_progress;
static stuck_t             g_stuck;
static drive_t             g_drive;
static edge_watch_t        g_edge;
static calhold_t           g_cal;
static bool                g_b_cal_held = false;
static supervisor_config_t g_config;
static uint8_t             g_obstacle_owed = 0U; /* RUN_OWE_* */

/* Armed and not stalled, published as one word after each liveness check, so
   the telemetry task on core 1 never reads a torn armed/stalled pair. */
static volatile uint16_t g_alive = 0U;

static volatile uint8_t  g_turn_pending = SUPERVISOR_NO_TURN;
static volatile uint8_t  g_turn_source  = 0U;
static volatile uint8_t  g_reset_count  = 0U; /* accepted RESETs; wraps */
static volatile uint8_t  g_start_count  = 0U; /* accepted STARTs; wraps */
static volatile bool     g_turning      = false;
static volatile uint16_t g_passes       = 0U;

/* The holds the supervisor has applied to the motion service. */
static uint32_t g_applied_holds   = 0U;
static uint8_t  g_attitude        = TELEMETRY_ATTITUDE_STATE_OK;
static bool     g_b_stalled_fault = false;
static bool     g_b_selftest_done = false;
static bool     g_b_renew_skipped = false;

ER supervisor_init (const supervisor_config_t * p_config)
{
    ER result = E_PAR;

    if (p_config != NULL)
    {
        g_config = *p_config;
        (void) InitSpinLock (&g_lock);
        command_ring_init (&g_ring);
        run_state_init (&g_ctx);
        liveness_init (&g_live, p_config->required_mask);
        progress_init (&g_progress);
        stuck_init (&g_stuck);
        health_drive_init (&g_drive);
        edge_init (&g_edge);
        /* No motion before the first rest calibration. */
        calhold_start (&g_cal, false);

        if (PLATFORM_RESET_WATCHDOG == platform_reset_reason ())
        {
            system_fault_assign (FAULT_OWNER_SUPERVISOR, FAULT_ABNORMAL_RESET_BIT, FAULT_ABNORMAL_RESET_BIT);
            LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "the last reset came from the watchdog");
        }
#ifdef PICO_CAR_WATCHDOG
        platform_wdt_start (WATCHDOG_TIMEOUT_MS);
#endif
        result = E_OK;
    }

    return result;
}

ER vehicle_command_post (uint8_t opcode, uint8_t source)
{
    return vehicle_command_post_ids (opcode, source, 0U, 0U);
}

ER vehicle_command_post_ids (uint8_t opcode, uint8_t source, uint16_t cmd_id,
                             uint16_t run_id)
{
    vehicle_cmd_t const cmd      = { cmd_id, run_id, opcode, source };
    UINT                saved    = 0U;
    bool                b_stored = false;

    (void) ISpinLock (&g_lock, &saved);
    b_stored = command_ring_push (&g_ring, cmd);
    (void) ISpinUnlock (&g_lock, saved);

    return b_stored ? E_OK : E_TMOUT;
}

uint16_t supervisor_cmd_drops (void)
{
    UINT     saved   = 0U;
    uint16_t dropped = 0U;

    (void) ISpinLock (&g_lock, &saved);
    dropped = g_ring.dropped;
    (void) ISpinUnlock (&g_lock, saved);

    return dropped;
}

uint8_t supervisor_take_turn (uint8_t * p_source)
{
    UINT    saved = 0U;
    uint8_t turn  = SUPERVISOR_NO_TURN;

    (void) ISpinLock (&g_lock, &saved);
    turn           = g_turn_pending;
    if ((SUPERVISOR_NO_TURN != turn) && (p_source != NULL))
    {
        *p_source = g_turn_source;
    }
    g_turn_pending = SUPERVISOR_NO_TURN;
    (void) ISpinUnlock (&g_lock, saved);

    return turn;
}

uint8_t supervisor_reset_count (void)
{
    UINT    saved = 0U;
    uint8_t count = 0U;

    (void) ISpinLock (&g_lock, &saved);
    count = g_reset_count;
    (void) ISpinUnlock (&g_lock, saved);

    return count;
}

uint8_t supervisor_start_count (void)
{
    UINT    saved = 0U;
    uint8_t count = 0U;

    (void) ISpinLock (&g_lock, &saved);
    count = g_start_count;
    (void) ISpinUnlock (&g_lock, saved);

    return count;
}

uint16_t supervisor_alive_mask (void)
{
    return g_alive;
}

void supervisor_set_turning (bool b_turning)
{
    g_turning = b_turning;
}

uint16_t supervisor_pass_count (void)
{
    return g_passes;
}

static uint32_t odometer_mm (void)
{
    motion_feedback_t feedback;
    uint32_t          mm = 0U;

    if (0 == motion_feedback_snapshot (&feedback))
    {
        mm = telemetry_odometer_mm (&feedback);
    }

    return mm;
}

static void emit (uint8_t source, uint8_t type, int16_t p1, uint16_t p2)
{
    (void) telemetry_emit_event (source, type, p1, p2, odometer_mm ());
}

void supervisor_turn_refused (uint8_t opcode)
{
    emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_CMD_REJECT,
          (int16_t) (((uint32_t) RUN_REJECT_BUSY << CMD_EVENT_REASON_SHIFT) | (uint32_t) opcode), 0U);
}

static void publish_counts (void)
{
    UINT saved = 0U;

    (void) ISpinLock (&g_lock, &saved);
    g_start_count = g_ctx.start_count;
    g_reset_count = g_ctx.reset_count;
    (void) ISpinUnlock (&g_lock, saved);
}

static run_inputs_t read_inputs (bool b_lapsed)
{
    run_inputs_t        in = { false, false, false, true, false, false, false,
                               false };
    physics_status_t    physics;
    run_physics_t       phys = { false, false, false, false, 0U, 0U, 0 };
    motion_status_t     motion;
    obstacle_status_t   obstacle;

    in.b_permit_lapsed = b_lapsed;
    if (E_OK == physics_snapshot (&physics))
    {
        phys.b_ready            = physics.ready;
        phys.b_calibrated       = physics.calibrated;
        phys.b_attitude_unknown = physics.attitude_unknown;
        phys.b_hump_active      = physics.hump_active;
        phys.hump_state         = physics.state;
        phys.attitude_state     = physics.attitude_state;
        phys.roll_cdeg          = physics.roll_cdeg;
    }
    run_state_read_physics (&in, &phys);
    if (0 == motion_status (&motion))
    {
        in.b_estop = motion.stop_requested;
    }
    if (E_OK == obstacle_snapshot (&obstacle))
    {
        in.b_follow = (TELEMETRY_OBSTACLE_STATE_MONITOR == obstacle.state);
    }
    if (g_turning || (SUPERVISOR_NO_TURN != g_turn_pending))
    {
        in.b_follow = false;
    }

    return in;
}

/* Post one obstacle command. True when the full queue refused it: the post is
   owed and sent again on the next pass. */
static bool obstacle_queue_full (uint8_t cmd)
{
    return (E_LIMIT == obstacle_post_cmd (cmd));
}

static void apply_holds (uint32_t wanted);

/* Carry out what a step decided. */
static void apply (const run_result_t * p_res, vehicle_cmd_t cmd, bool b_command)
{
    if (b_command)
    {
        uint32_t const what = ((uint32_t) cmd.source << CMD_EVENT_SOURCE_SHIFT) | (uint32_t) cmd.opcode;

        if (p_res->b_accepted)
        {
            emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_CMD_ACCEPT,
                  (int16_t) what, cmd.cmd_id);
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "command %d from %d accepted, id=%d run=%d",
                      (int) cmd.opcode, (int) cmd.source, (int) cmd.cmd_id, (int) cmd.run_id);
        }
        else if (p_res->b_rejected)
        {
            emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_CMD_REJECT,
                  (int16_t) (what | ((uint32_t) p_res->reason << CMD_EVENT_REASON_SHIFT)), cmd.cmd_id);
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "command %d from %d refused (%d), id=%d run=%d",
                      (int) cmd.opcode, (int) cmd.source, (int) p_res->reason, (int) cmd.cmd_id,
                      (int) cmd.run_id);
        }
        else
        {
            /* Nothing to report. */
        }
    }

    if (p_res->b_motion_estop)
    {
        (void) motion_request_stop ();
    }
    if (p_res->b_physics_recal)
    {
        physics_request_new_run ();
        calhold_start (&g_cal, true);
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "IMU calibrates, the car waits at rest");
    }
    if (p_res->b_obstacle_stop)
    {
        (void) obstacle_post_cmd (OBSTACLE_CMD_STOP);
    }
    if (p_res->b_sticky_clear)
    {
        system_fault_sticky_clear (FAULT_ALL_MASK);
        /* The wheels hold before the line task sees the RESET, so that it does not
           drive between clearing its latches and the hold. */
        apply_holds (run_state_hold_mask (&g_ctx));
    }
    g_obstacle_owed = run_obstacle_post (g_obstacle_owed, p_res, obstacle_queue_full);
    if (p_res->b_cancel_turn || (RUN_TURN_NONE != p_res->turn))
    {
        UINT saved = 0U;

        (void) ISpinLock (&g_lock, &saved);
        g_turn_pending = (RUN_TURN_NONE != p_res->turn) ? p_res->turn : SUPERVISOR_NO_TURN;
        g_turn_source  = cmd.source;
        (void) ISpinUnlock (&g_lock, saved);
    }
}

/* Make the motion service holds equal to the wanted holds. A refused change
   is tried again on the next pass. */
static void apply_holds (uint32_t wanted)
{
    uint32_t const bits[] = { MOTION_HOLD_OPERATOR, MOTION_HOLD_BARCODE, MOTION_HOLD_LIFT };
    uint8_t        idx    = 0U;

    for (idx = 0U; idx < (sizeof (bits) / sizeof (bits[0])); idx++)
    {
        uint32_t const bit      = bits[idx];
        bool const     b_wanted = (0U != (wanted & bit));

        if (b_wanted != (0U != (g_applied_holds & bit)))
        {
            if (0 == motion_set_hold (bit, b_wanted))
            {
                g_applied_holds = b_wanted ? (g_applied_holds | bit) : (g_applied_holds & ~bit);
            }
        }
    }
}

/* Send the events a state change produced. */
static void emit_edges (uint8_t source, const edge_event_t * p_events, uint8_t count)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        emit (source, p_events[idx].type, p_events[idx].p1, p_events[idx].p2);
    }
}

/* Run start and stop, the obstacle cycle and the humps. */
static void watch_edges (void)
{
    edge_event_t      ev[EDGE_EVENTS_MAX];
    obstacle_status_t obstacle;
    physics_status_t  physics;

    emit_edges ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                ev, edge_run (&g_edge, (uint8_t) run_state_of (&g_ctx), ev));
    if (E_OK == obstacle_snapshot (&obstacle))
    {
        obstacle_plan_summary_t plan = { 0U, 0U, 0U, 0U, 0U, 0U, false, 0U };
        edge_obstacle_in_t      in;

        (void) obstacle_copy_plan (&plan);
        in.state          = obstacle.state;
        in.cause          = obstacle.cause;
        /* The stop event's p2 is the give-up reason for BYPASS_GAVE_UP,
           else the echo class. */
        in.echo_class     = (TELEMETRY_OBSTACLE_CAUSE_BYPASS_GAVE_UP == obstacle.cause)
                                ? obstacle.stop_reason
                                : obstacle.echo_class;
        in.distance_mm    = obstacle.distance_mm;
        in.plan_status    = plan.status;
        in.plan_side      = plan.side;
        in.plan_start_deg = plan.start_deg;
        in.plan_width_mm  = plan.width_mm;
        emit_edges ((uint8_t) TELEMETRY_EVENT_SOURCE_OBSTACLE, ev, edge_obstacle (&g_edge, &in, ev));
    }
    if ((E_OK == physics_snapshot (&physics)) && physics.ready)
    {
        emit_edges ((uint8_t) TELEMETRY_EVENT_SOURCE_PHYSICS, ev,
                    edge_hump (&g_edge, physics.hump_active, physics.hump_peak_mm, physics.height_valid, ev));
    }
}

/* Keep MOTION_HOLD_CALIBRATE on while the IMU takes its rest reference. Only a
   calibration the supervisor asked for holds: a reference physics drops in a
   run (an IMU read failure) does not. */
static void hold_for_calibration (bool b_calibrated)
{
    bool const b_held = calhold_update (&g_cal, b_calibrated);

    if (calhold_timeout_edge (&g_cal))
    {
        LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
                  "IMU has not calibrated in 5 s: "
                  "the car stays held until it does");
    }
    if (b_held != g_b_cal_held)
    {
        if (0 == motion_set_hold (MOTION_HOLD_CALIBRATE, b_held))
        {
            g_b_cal_held = b_held;
            if (!b_held)
            {
                LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "IMU calibrated");
            }
            else
            {
                /* Hold set. */
            }
        }
    }
}

/* Lift and overturn events follow the attitude state. */
static void note_attitude (uint8_t state)
{
    if (state != g_attitude)
    {
        uint8_t const source = (uint8_t) TELEMETRY_EVENT_SOURCE_PHYSICS;

        if (TELEMETRY_ATTITUDE_STATE_OVERTURNED == state)
        {
            emit (source, (uint8_t) TELEMETRY_EVT_OVERTURN, 0, 0U);
            LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car overturned, needs RESET");
        }
        else if (TELEMETRY_ATTITUDE_STATE_LIFTED == state)
        {
            emit (source, (uint8_t) TELEMETRY_EVT_LIFT_START, 0, 0U);
            LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car lifted, holding the wheels");
        }
        else if ((TELEMETRY_ATTITUDE_STATE_OK == state) &&
                 (TELEMETRY_ATTITUDE_STATE_LIFTED == g_attitude))
        {
            emit (source, (uint8_t) TELEMETRY_EVT_LIFT_END, 0, 0U);
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car set down, START releases the hold");
        }
        else
        {
            /* CANDIDATE, or back from overturned: no event. */
        }
        g_attitude = state;
    }
}

/* Absolute wheel travel, both wheels added, in mm. */
static uint32_t travel_abs_mm (const motion_feedback_t * p_feedback)
{
    int64_t const left  = p_feedback->left_displacement_um;
    int64_t const right = p_feedback->right_displacement_um;
    int64_t const sum   = ((left < 0) ? -left : left) + ((right < 0) ? -right : right);

    return (uint32_t) (sum / UM_PER_MM);
}

static void report_stall (uint8_t id, uint8_t misses)
{
    emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_TASK_STALL,
          (int16_t) id, (uint16_t) misses);
    LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "stall id=%d, misses=%d", (int) id, (int) misses);
}

/* The drive check: crossed encoders, and the wheels-run-free cue for physics.
   The cue needs a car on the floor, so a lifted or overturned car is not judged. */
static void check_drive (const motion_diag_t * p_diag, const motion_feedback_t * p_feedback,
                         bool b_free)
{
    drive_in_t in;

    in.duty_pm[0]     = p_diag->left_duty_permille;
    in.duty_pm[1]     = p_diag->right_duty_permille;
    in.speed_um_s[0]  = p_feedback->left_speed_um_s;
    in.speed_um_s[1]  = p_feedback->right_speed_um_s;
    in.target_um_s[0] = p_diag->left_target_um_s;
    in.target_um_s[1] = p_diag->right_target_um_s;
    in.b_judge        = b_free && p_feedback->fresh &&
                 ((TELEMETRY_ATTITUDE_STATE_OK == g_attitude) ||
                  (TELEMETRY_ATTITUDE_STATE_CANDIDATE == g_attitude));

    if (health_drive_update (&g_drive, &in, HEALTH_CHECK_MS))
    {
        report_stall (SUPERVISOR_STALL_ENCODERS, 0U);
        LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
                   "the wheels answer crosswise: check the encoder plugs");
    }
    motion_note_encoder_crossed (g_drive.b_crossed);
    physics_set_wheels_free (g_drive.b_free);
}

/* The liveness, progress and stuck-sensor checks. They run every 100 ms.
   Returns true when a critical task is stalled. */
static bool health_check (void)
{
    uint16_t          fresh = 0U;
    uint16_t          back  = 0U;
    uint32_t          idx   = 0U;
    motion_feedback_t feedback;
    motion_status_t   motion;
    motion_diag_t     diag;
    bool              b_moving_expected = false;
    bool              b_progress_stall  = false;
    bool              b_motion_timing   = false;

    liveness_check (&g_live, liveness_take (), &fresh, &back);
    g_alive = (uint16_t) ((uint32_t) g_live.armed & ~(uint32_t) g_live.stalled);
    for (idx = 0U; idx < LIVE_TASKS; idx++)
    {
        if (0U != ((uint32_t) fresh & (1UL << idx)))
        {
            report_stall ((uint8_t) idx, g_live.miss[idx]);
        }
        if (0U != ((uint32_t) back & (1UL << idx)))
        {
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "task %d runs again", (int) idx);
        }
    }

    if ((0 == motion_feedback_snapshot (&feedback)) && (0 == motion_status (&motion)) &&
        (0 == motion_diag_snapshot (&diag)))
    {
        int32_t const l = diag.left_target_um_s;
        int32_t const r = diag.right_target_um_s;
        bool const    b_driven = ((((l < 0) ? -l : l) + ((r < 0) ? -r : r)) >= PROGRESS_MIN_TARGET_UM_S);
        bool const    b_free   = (RUN_STATE_RUNNING == run_state_of (&g_ctx)) &&
                              (0U == motion.hold_mask) && !motion.stop_requested;

        b_motion_timing = health_motion_timing_fault (motion.state, motion.fault);
        b_moving_expected = b_driven && b_free && feedback.fresh;
        if (progress_update (&g_progress, b_moving_expected, travel_abs_mm (&feedback),
                             HEALTH_CHECK_MS))
        {
            report_stall (SUPERVISOR_STALL_PROGRESS, 0U);
        }
        b_progress_stall = g_progress.b_tripped;
        check_drive (&diag, &feedback, b_free);

        /* Reported, never held: a car centred on a long straight line trips it. */
        if ((g_config.p_line_edges != NULL) &&
            stuck_update (&g_stuck, b_free && !g_turning,
                          g_config.p_line_edges (), telemetry_odometer_mm (&feedback)))
        {
            report_stall (SUPERVISOR_STALL_SENSOR, 0U);
        }
    }

    g_b_stalled_fault = (0U != g_live.stalled) || b_progress_stall;
    system_fault_assign (FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, g_b_stalled_fault ? FAULT_TASK_STALLED_BIT : 0U);
    if (b_motion_timing)
    {
        LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "motion timing fault %d: no watchdog feed",
                   (int) motion.fault);
    }

    return liveness_critical_stalled (&g_live) || b_motion_timing;
}

/* The boot self-test: what can be checked without moving the car. */
static void selftest (void)
{
    physics_status_t  physics;
    obstacle_status_t obstacle;
    uint32_t          echo[OBSTACLE_ECHO_CLASSES] = { 0U };
    uint16_t          failed = 0U;

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
    if ((E_OK == obstacle_snapshot (&obstacle)) && (0U != (obstacle.flags & OBSTACLE_FLAG_BEAM_BLOCKED)))
    {
        failed |= SELFTEST_BEAM;
    }

    emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_BOOT,
          (int16_t) platform_reset_reason (), failed);
    if (0U != failed)
    {
        LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "self-test failed, mask=%d", (int) failed);
    }
    else
    {
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "self-test passed");
    }
}

static void pass (void)
{
    bool const    b_lapsed   = run_permit_lapsed (g_b_renew_skipped,
                                                  motion_permit_age_ms (),
                                                  PERMIT_LEASE_MS);
    run_inputs_t  in         = read_inputs (b_lapsed);
    run_result_t  res;
    vehicle_cmd_t cmd        = { 0U, 0U, 0U, 0U };
    UINT          saved      = 0U;
    bool          b_more     = true;
    bool          b_critical = liveness_critical_stalled (&g_live);
    bool          b_renew_failed = false;

    run_state_tick (&g_ctx, &in, &res);
    apply (&res, cmd, false);

    while (b_more)
    {
        (void) ISpinLock (&g_lock, &saved);
        b_more = command_ring_pop (&g_ring, &cmd);
        (void) ISpinUnlock (&g_lock, saved);
        if (b_more)
        {
            run_state_command (&g_ctx, cmd, &in, &res);
            apply (&res, cmd, true);
            /* The next command sees the effect of this one. */
            in = read_inputs (b_lapsed);
        }
    }
    publish_counts ();
    apply_holds (run_state_hold_mask (&g_ctx));
    watch_edges ();

    {
        physics_status_t physics;

        bool             b_cal = false;

        if ((E_OK == physics_snapshot (&physics)) && physics.ready)
        {
            note_attitude (physics.attitude_state);
            b_cal = physics.calibrated;
        }
        hold_for_calibration (b_cal);
    }

    if (0U == (g_passes % HEALTH_PASSES))
    {
        b_critical = health_check ();
    }
    if (!b_critical)
    {
        b_renew_failed = (0 != motion_renew_permit (PERMIT_LEASE_MS));
        platform_wdt_feed ();
    }
    g_b_renew_skipped = run_permit_skipped (b_critical, b_renew_failed);
    if ((SELFTEST_PASSES == g_passes) && !g_b_selftest_done)
    {
        g_b_selftest_done = true;
        selftest ();
    }
    if ((g_config.p_background != NULL) && (0U == (g_passes % BACKGROUND_PASSES)))
    {
        g_config.p_background ();
    }
    g_passes++;
}

void supervisor_run (void)
{
    for (;;)
    {
        pass ();
        (void) tk_slp_tsk ((TMO) SLEEP_TIMEOUT_MS);
    }
}
