/** @file loop.c
 * @brief Core 0 supervisor task, command ring, run state and health checks.
 */

#include <tk/smp.h>

#include "internal.h"

#include "command_ring.h"
#include "health_core.h"
#include "motion/motion.h"
#include "obstacle/obstacle.h"
#include "imu/imu.h"
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/drv_watchdog.h"
#include "comms/comms.h"

/* One liveness check every 5 passes (100 ms). */
#define HEALTH_PASSES   (5U)
#define HEALTH_CHECK_MS (SUPERVISOR_PASS_MS * HEALTH_PASSES)

/* The boot self-test reads its facts after 100 passes (2 s). */
#define SELFTEST_PASSES (100U)

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

#define START_CONNECTION_FAULTS \
    (FAULT_IMU_NACK_BIT | FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT)

/* The task sleeps at most this long when the cyclic handler does not wake it. */
#define SLEEP_TIMEOUT_MS (SUPERVISOR_PASS_MS + SUPERVISOR_PASS_MS)

/* The wheels must move at least this fast for the progress check to expect travel. */
#define PROGRESS_MIN_TARGET_UM_S (10000)

#define UM_PER_MM (1000)

static T_SPLOCK            g_lock;
static command_ring_t      g_ring;
/* Only VEHICLE_CMD_ESTOP marks this slot occupied. */
static vehicle_cmd_t       g_estop_pending;
static run_ctx_t           g_ctx;
static liveness_t          g_live;
static progress_t          g_progress;
static stuck_t             g_stuck;
static drive_t             g_drive;
static calhold_t           g_cal;
static bool                g_b_cal_held = false;
static bool                g_pose_recovery;
static uint8_t             g_recovered_count;
static bool                g_recovered_clear;
static supervisor_config_t g_config;
static uint8_t             g_obstacle_owed = 0U; /* RUN_OWE_* */

/* Armed and not stalled, published as one word after each liveness check, so
   the telemetry task on core 1 never reads a torn armed/stalled pair. */
static volatile uint16_t g_alive = 0U;

static volatile uint8_t  g_turn_pending = SUPERVISOR_NO_TURN;
static volatile uint8_t  g_turn_source  = 0U;
static volatile uint8_t  g_reset_count  = 0U; /* accepted RESETs; wraps */
static bool              g_continuous_floor; /* published under g_lock with the start count */
static volatile uint8_t  g_start_count  = 0U; /* accepted STARTs; wraps */
static bool              g_b_armed;
static bool              g_b_start_failed;
static uint8_t           g_failed_start;
static volatile bool     g_wheels_loaded = false;
static volatile bool     g_turning      = false;
static volatile uint16_t g_passes       = 0U;

/* The holds the supervisor has applied to the motion service. */
static uint32_t g_applied_holds   = 0U;
static uint8_t  g_attitude        = TELEMETRY_ATTITUDE_STATE_OK;
static bool     g_b_stalled_fault = false;
static bool     g_b_selftest_done = false;
static bool     g_b_renew_skipped = false;
static bool     g_b_permit_renewed = false;
static bool     g_b_motion_timing_fault;
static timing_watch_t g_timing;

ER supervisor_setup (const supervisor_config_t * p_config)
{
    ER result = E_PAR;

    if (p_config != NULL)
    {
        g_config = *p_config;
        (void) InitSpinLock (&g_lock);
        command_ring_init (&g_ring);
        g_estop_pending.opcode = VEHICLE_CMD_STOP;
        run_state_init (&g_ctx);
        g_continuous_floor = false;
        g_start_count = 0U;
        g_b_armed = false;
        g_b_start_failed = false;
        g_b_permit_renewed = false;
        g_reset_count = 0U;
        liveness_init (&g_live, p_config->required_mask);
        progress_init (&g_progress);
        stuck_init (&g_stuck);
        health_drive_init (&g_drive);
        timing_watch_init (&g_timing);
        supervisor_telem_init ();
        /* No motion before the first rest calibration. */
        calhold_start (&g_cal, 0U);
        supervisor_admission_init (p_config);

        if (PLATFORM_RESET_WATCHDOG == platform_reset_reason ())
        {
            system_fault_assign (FAULT_OWNER_SUPERVISOR, FAULT_ABNORMAL_RESET_BIT, FAULT_ABNORMAL_RESET_BIT);
            LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "the last reset came from the watchdog");
        }
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
    if ((VEHICLE_CMD_ESTOP == opcode) && (VEHICLE_SRC_BARCODE != source))
    {
        if (VEHICLE_CMD_ESTOP != g_estop_pending.opcode)
        {
            g_estop_pending = cmd;
            b_stored = true;
        }
        else
        {
            g_ring.dropped++;
        }
    }
    else
    {
        b_stored = command_ring_push (&g_ring, cmd);
    }
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

uint8_t supervisor_poll_turn (uint8_t * p_source)
{
    UINT    saved = 0U;
    uint8_t turn  = SUPERVISOR_NO_TURN;

    (void) ISpinLock (&g_lock, &saved);
    turn           = g_turn_pending;
    if ((SUPERVISOR_NO_TURN != turn) && (p_source != NULL))
    {
        *p_source = g_turn_source;
    }
    if ((SUPERVISOR_NO_TURN != turn) &&
        (!vehicle_is_junction_choice (turn) || (g_turn_source != VEHICLE_SRC_BARCODE)))
    {
        g_turn_pending = SUPERVISOR_NO_TURN;
        g_turning = true;
    }
    (void) ISpinUnlock (&g_lock, saved);

    return turn;
}

bool supervisor_route_pending (uint8_t expected_opcode, bool b_consume)
{
    UINT saved = 0U;
    bool b_pending = false;
    (void) ISpinLock (&g_lock, &saved);
    b_pending = vehicle_is_junction_choice (expected_opcode) &&
                (expected_opcode == g_turn_pending) && (VEHICLE_SRC_BARCODE == g_turn_source);
    if (b_pending && b_consume)
    {
        g_turn_pending = SUPERVISOR_NO_TURN;
    }
    (void) ISpinUnlock (&g_lock, saved);
    return b_pending;
}

bool supervisor_wheels_loaded (void)
{
    UINT saved    = 0U;
    bool b_loaded = false;

    (void) ISpinLock (&g_lock, &saved);
    b_loaded = g_wheels_loaded;
    (void) ISpinUnlock (&g_lock, saved);

    return b_loaded;
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

uint8_t supervisor_start_snapshot (bool * p_continuous_floor)
{
    UINT    saved = 0U;
    uint8_t count = 0U;

    (void) ISpinLock (&g_lock, &saved);
    count = g_start_count;
    if (p_continuous_floor != NULL)
    {
        *p_continuous_floor = g_continuous_floor;
    }
    (void) ISpinUnlock (&g_lock, saved);

    return count;
}

uint8_t supervisor_start_count (void)
{
    return supervisor_start_snapshot (NULL);
}

uint8_t supervisor_armed_start (bool * p_armed)
{
    UINT saved = 0U;
    (void) ISpinLock (&g_lock, &saved);
    uint8_t const generation = g_start_count;
    *p_armed = g_b_armed;
    (void) ISpinUnlock (&g_lock, saved);
    return generation;
}

void supervisor_start_check_failed (uint8_t generation)
{
    UINT saved = 0U;
    (void) ISpinLock (&g_lock, &saved);
    if (g_b_armed && generation == g_start_count)
    {
        g_failed_start = generation;
        g_b_start_failed = true;
    }
    (void) ISpinUnlock (&g_lock, saved);
}

uint8_t supervisor_recovery_snapshot (bool * p_clear)
{
    UINT saved = 0U;
    (void) ISpinLock (&g_lock, &saved);
    uint8_t const count = g_recovered_count;
    *p_clear = g_recovered_clear;
    (void) ISpinUnlock (&g_lock, saved);
    return count;
}

uint16_t supervisor_alive_mask (void)
{
    return g_alive;
}

void supervisor_set_turning (bool b_turning)
{
    UINT saved = 0U;
    (void) ISpinLock (&g_lock, &saved);
    g_turning = b_turning;
    (void) ISpinUnlock (&g_lock, saved);
}

uint16_t supervisor_pass_count (void)
{
    return g_passes;
}

static void publish_counts (void)
{
    UINT saved = 0U;
    bool const b_running = (RUN_STATE_RUNNING == run_state_of (&g_ctx));
    bool const b_floor   = g_ctx.b_continuous_floor;

    (void) ISpinLock (&g_lock, &saved);
    g_continuous_floor = b_floor && b_running;
    g_b_armed = b_running;
    g_start_count = g_ctx.start_count;
    g_reset_count = g_ctx.reset_count;
    (void) ISpinUnlock (&g_lock, saved);
    supervisor_sync_run_token (b_running, b_floor);
}

static run_inputs_t read_inputs (bool b_lapsed)
{
    run_inputs_t        in = {0};
    physics_status_t    physics;
    run_physics_t       phys = {0};
    motion_status_t     motion;
    obstacle_status_t   obstacle;

    in.b_connection_fault = (0U != (system_fault_get () & START_CONNECTION_FAULTS));
    in.b_calibration_timed_out = g_cal.b_timed_out && g_cal.b_waiting;
    if (E_OK == physics_snapshot (&physics))
    {
        phys.b_ready            = physics.ready;
        phys.b_calibrated       = physics.calibrated;
        phys.b_attitude_unknown = physics.attitude_unknown;
        phys.b_hump_active      = physics.hump_active;
        phys.hump_state         = physics.state;
        phys.attitude_state     = physics.attitude_state;
        phys.b_level_still      = physics.level_still;
        in.b_collecting_rest    = physics.ready && physics.collecting_rest;
        in.rest_interruptions   = physics.rest_interruptions;
    }
    run_state_read_physics (&in, &phys);
    if (0 == motion_status (&motion))
    {
        in.b_estop = motion.stop_requested;
        in.b_follow = motion.ready && (0U == motion.hold_mask) && !in.b_estop;
    }
    in.b_permit_lapsed = b_lapsed || g_b_motion_timing_fault;
    in.b_follow = in.b_follow && !in.b_lifted && !in.b_overturned && !in.b_permit_lapsed &&
                  (E_OK == obstacle_snapshot (&obstacle)) &&
                  (TELEMETRY_OBSTACLE_STATE_MONITOR == obstacle.state);

    return in;
}

/* A probe owns the wheels before it commits to a branch. Its pending choice
   and the replacement U must be compared and published under the same lock. */
void supervisor_admit_command (vehicle_cmd_t cmd, run_inputs_t in, run_result_t * p_res)
{
    UINT saved = 0U;
    (void) ISpinLock (&g_lock, &saved);
    bool const b_replace = (VEHICLE_CMD_UTURN == cmd.opcode) && (VEHICLE_SRC_BARCODE == cmd.source) &&
                           (VEHICLE_SRC_BARCODE == g_turn_source) && vehicle_is_junction_choice (g_turn_pending);
    in.b_follow = in.b_follow && (b_replace || (!g_turning && (SUPERVISOR_NO_TURN == g_turn_pending)));
    run_state_command (&g_ctx, cmd, &in, p_res);
    if (RUN_TURN_NONE != p_res->turn)
    {
        g_turn_pending = p_res->turn;
        g_turn_source = cmd.source;
    }
    (void) ISpinUnlock (&g_lock, saved);
}

/* Post one obstacle command. True when the full queue refused it: the post is
   owed and sent again on the next pass. */
static bool obstacle_queue_full (uint8_t cmd)
{
    return (E_LIMIT == obstacle_post_cmd (cmd));
}

static void apply_holds (uint32_t wanted);

static void set_calibration_hold (bool b_held)
{
    if ((b_held != g_b_cal_held) && (0 == motion_set_hold (MOTION_HOLD_CALIBRATE, b_held)))
    {
        g_b_cal_held = b_held;
        if (!b_held)
        {
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "IMU calibrated");
        }
    }
}

/* Carry out what a step decided. */
void supervisor_apply_result (const run_result_t * p_res, vehicle_cmd_t cmd, bool b_command)
{
    /* Stop and hold updates always precede non-blocking telemetry handoff. */
    if (p_res->b_motion_estop)
    {
        (void) motion_request_stop ();
    }
    if (b_command && p_res->b_accepted &&
        ((VEHICLE_CMD_STOP == cmd.opcode) || (VEHICLE_CMD_ESTOP == cmd.opcode)))
    {
        supervisor_arm_stop_timing ();
    }
    if (p_res->b_cancel_turn)
    {
        UINT saved = 0U;
        (void) ISpinLock (&g_lock, &saved);
        g_turn_pending = SUPERVISOR_NO_TURN;
        (void) ISpinUnlock (&g_lock, saved);
    }
    if (p_res->b_physics_recal)
    {
        set_calibration_hold (true);
        calhold_start (&g_cal, physics_request_new_run ());
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "IMU calibrates, the car waits at rest");
    }
    /* Apply new holds before reporting. Release holds once pending commands
       have been handled, so a later STOP can keep the wheels held. */
    apply_holds (p_res->hold_mask | g_applied_holds);

    if (b_command)
    {
        supervisor_report_command (cmd, p_res);
        if (p_res->b_accepted && vehicle_is_start (cmd.opcode))
        {
            supervisor_note_start_accepted ();
        }
    }

    if (p_res->b_obstacle_stop)
    {
        (void) obstacle_post_cmd (OBSTACLE_CMD_STOP);
    }
    /* The physical lift can end while the pickup stop still awaits START. */
    system_fault_assign (FAULT_OWNER_SUPERVISOR, FAULT_LIFTED_BIT,
                         g_ctx.b_pickup ? FAULT_LIFTED_BIT : 0U);
    if (p_res->b_sticky_clear)
    {
        system_fault_sticky_clear (FAULT_ALL_MASK);
    }
    g_obstacle_owed = run_obstacle_post (g_obstacle_owed, p_res, obstacle_queue_full);
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

/* Give the failed generation an operator hold before draining other commands. */
static bool stop_unverified_start (const run_inputs_t * p_in)
{
    UINT saved = 0U;
    (void) ISpinLock (&g_lock, &saved);
    bool const b_failed = g_b_start_failed && (g_failed_start == g_ctx.start_count);
    g_b_start_failed = false;
    (void) ISpinUnlock (&g_lock, saved);

    if (b_failed && (RUN_STATE_RUNNING == run_state_of (&g_ctx)))
    {
        vehicle_cmd_t const stop = {0U, 0U, VEHICLE_CMD_STOP, 0U};
        run_result_t result;
        run_state_command (&g_ctx, stop, p_in, &result);
        supervisor_apply_result (&result, stop, false);
        supervisor_sync_run_token (false, g_ctx.b_continuous_floor);
        LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "START motion check failed, car stopped");
        return true;
    }
    return false;
}

/* One owner releases the calibration hold once every reason for it clears. */
static void hold_for_calibration (uint32_t applied_request, bool b_calibrated)
{
    bool const b_held = calhold_update (&g_cal, applied_request, b_calibrated);

    if (calhold_timeout_edge (&g_cal))
    {
        LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
                  "IMU has not calibrated in 5 s: "
                  "the car stays held until it does");
    }
    set_calibration_hold (b_held || g_pose_recovery);
}

/* The supervisor logs the attitude behind its motion hold. */
static void note_attitude (uint8_t state)
{
    if (state != g_attitude)
    {
        if (TELEMETRY_ATTITUDE_STATE_OVERTURNED == state)
        {
            LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car overturned, set it level and press START");
        }
        else if (TELEMETRY_ATTITUDE_STATE_LIFTED == state)
        {
            LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car stopped after pickup, set it level and press START");
        }
        else if ((TELEMETRY_ATTITUDE_STATE_OK == state) &&
                 (TELEMETRY_ATTITUDE_STATE_LIFTED == g_attitude))
        {
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car set down, START releases the hold");
        }
        else
        {
            /* Candidate or the return from overturn needs no message. */
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

/* The drive check: crossed encoders, and the wheels-run-free cue for physics.
   The cue needs a car on the floor, so a lifted or overturned car is not judged. */
static void check_drive (const motion_diag_t * p_diag, const motion_feedback_t * p_feedback,
                         bool b_free)
{
    drive_in_t in;
    UINT       saved = 0U;

    in.duty_pm[0]     = p_diag->left_duty_permille;
    in.duty_pm[1]     = p_diag->right_duty_permille;
    in.speed_um_s[0]  = p_feedback->left_speed_um_s;
    in.speed_um_s[1]  = p_feedback->right_speed_um_s;
    in.target_um_s[0] = p_diag->left_target_um_s;
    in.target_um_s[1] = p_diag->right_target_um_s;
    in.b_judge        = b_free && p_feedback->fresh &&
                 p_feedback->speed_valid[0] && p_feedback->speed_valid[1] &&
                 ((TELEMETRY_ATTITUDE_STATE_OK == g_attitude) ||
                  (TELEMETRY_ATTITUDE_STATE_CANDIDATE == g_attitude));

    if (health_drive_update (&g_drive, &in, HEALTH_CHECK_MS))
    {
        supervisor_report_stall (SUPERVISOR_STALL_ENCODERS, 0U);
        LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
                   "the wheels answer crosswise: check the encoder plugs");
    }
    motion_note_encoder_crossed (g_drive.b_crossed);
    physics_set_wheels_free (g_drive.b_free);
    (void) ISpinLock (&g_lock, &saved);
    g_wheels_loaded = g_drive.b_loaded;
    (void) ISpinUnlock (&g_lock, saved);
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

    liveness_check (&g_live, liveness_take (), &fresh, &back);
    g_alive = (uint16_t) ((uint32_t) g_live.armed & ~(uint32_t) g_live.stalled);
    for (idx = 0U; idx < LIVE_TASKS; idx++)
    {
        if (0U != ((uint32_t) fresh & (1UL << idx)))
        {
            supervisor_report_stall ((uint8_t) idx, g_live.miss[idx]);
        }
        if (0U != ((uint32_t) back & (1UL << idx)))
        {
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "task %d runs again", (int) idx);
        }
    }

    if (0 == motion_snapshot (&motion, &feedback, &diag))
    {
        int32_t const l = diag.left_target_um_s;
        int32_t const r = diag.right_target_um_s;
        bool const    b_driven = ((((l < 0) ? -l : l) + ((r < 0) ? -r : r)) >= PROGRESS_MIN_TARGET_UM_S);
        bool const    b_free   = (RUN_STATE_RUNNING == run_state_of (&g_ctx)) &&
                              (0U == motion.hold_mask) && !motion.stop_requested;

        b_moving_expected = b_driven && b_free && feedback.fresh;
        if (progress_update (&g_progress, b_moving_expected, travel_abs_mm (&feedback),
                             HEALTH_CHECK_MS))
        {
            supervisor_report_stall (SUPERVISOR_STALL_PROGRESS, 0U);
        }
        b_progress_stall = g_progress.b_tripped;
        check_drive (&diag, &feedback, b_free);

        /* Reported, never held: a car centred on a long straight line trips it. */
        if ((g_config.p_line_edges != NULL) &&
            stuck_update (&g_stuck, b_free && !g_turning,
                          g_config.p_line_edges (), telemetry_odometer_mm (&feedback)))
        {
            supervisor_report_stall (SUPERVISOR_STALL_SENSOR, 0U);
        }
    }

    g_b_stalled_fault = (0U != g_live.stalled) || b_progress_stall || g_stuck.b_tripped;
    system_fault_assign (FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, g_b_stalled_fault ? FAULT_TASK_STALLED_BIT : 0U);

    return liveness_critical_stalled (&g_live);
}

static void recover_attitude (const run_inputs_t * p_in, bool b_critical, bool b_level)
{
    if (!g_pose_recovery)
    {
        return;
    }
    bool clear = false;
    uint16_t const clear_mm = HUG_STANDOFF_MM + 1U;
    if (RUN_STATE_RUNNING != run_state_of (&g_ctx))
    {
        /* STOP, a pickup or a lapsed permit owns the wheels from here. */
        g_pose_recovery = (0U == g_applied_holds) && !g_ctx.b_estop;
    }
    else if (!b_level || b_critical || !g_b_cal_held || g_cal.b_waiting)
    {
        supervisor_reset_parked_observation ();
    }
    else if (supervisor_parked_observation (p_in, MOTION_HOLD_CALIBRATE, clear_mm, &clear))
    {
        UINT saved = 0U;
        (void) ISpinLock (&g_lock, &saved);
        g_recovered_clear = clear;
        g_recovered_count++;
        (void) ISpinUnlock (&g_lock, saved);
        g_pose_recovery = false;
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "IMU reference restored, forward clear=%d", (int)clear);
    }
}

static void update_attitude_hold (const run_inputs_t * p_in, bool b_critical)
{
    physics_status_t physics = {0};
    bool calibrated = false;
    if ((E_OK == physics_snapshot (&physics)) && physics.ready)
    {
        note_attitude (physics.attitude_state);
        calibrated = physics.calibrated && !physics.attitude_unknown &&
                     (TELEMETRY_ATTITUDE_STATE_OK == physics.attitude_state);
    }
    recover_attitude (p_in, b_critical, calibrated && physics.level_still);
    hold_for_calibration (physics.reference_request, calibrated);
}

/* Once per pass: a timing fault clears in place; a persistent or repeated
   one stops the permit and the watchdog feed. */

static void watch_motion_timing (void)
{
    motion_status_t motion;
    bool            b_fault = false;

    if (0 == motion_status (&motion))
    {
        b_fault = health_motion_timing_fault (motion.state, motion.fault);
        if (b_fault && !g_timing.b_fault)
        {
            LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "motion timing fault %d, clearing in place",
                      (int) motion.fault);
        }
    }
    if (timing_watch_update (&g_timing, b_fault, SUPERVISOR_PASS_MS) && !g_b_motion_timing_fault)
    {
        g_b_motion_timing_fault = true;
        LOG_ERROR (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "motion timing fault %u times or %u ms: no watchdog feed",
                   (unsigned) g_timing.faults, (unsigned) g_timing.fault_ms);
    }
}

static void note_imu_loss (bool b_unknown)
{
    static bool s_b_imu_loss_logged = false;

    if (!s_b_imu_loss_logged && !g_cal.b_waiting &&
        (RUN_STATE_RUNNING == run_state_of (&g_ctx)) && b_unknown)
    {
        s_b_imu_loss_logged = true;
        LOG_WARN (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "IMU reference lost, driving on");
    }
    else if (s_b_imu_loss_logged && !b_unknown)
    {
        s_b_imu_loss_logged = false;
    }
    else
    {
        /* No change. */
    }
}

static void feed_watchdog (void)
{
#ifdef PICO_CAR_WATCHDOG
    static bool s_b_wdt_armed = false;

    if (!s_b_wdt_armed)
    {
        platform_wdt_start (WATCHDOG_TIMEOUT_MS);
        s_b_wdt_armed = true;
    }
#endif
    platform_wdt_feed ();
}

static void pass (void)
{
    watch_motion_timing ();
    bool const    b_lapsed   = run_permit_lapsed (g_b_renew_skipped,
                                                  motion_permit_age_ms (),
                                                  PERMIT_LEASE_MS);
    run_inputs_t  in         = read_inputs (b_lapsed);
    run_result_t  res;
    vehicle_cmd_t cmd        = { 0U, 0U, 0U, 0U };
    UINT          saved      = 0U;
    bool          b_more     = true;
    bool          b_estop_taken = false;
    bool          b_critical = liveness_critical_stalled (&g_live);
    bool          b_renew_failed = false;

    supervisor_watch_stop_timing (in.b_estop, true);
    run_state_tick (&g_ctx, &in, &res);
    note_imu_loss (in.b_attitude_unknown);
    supervisor_apply_result (&res, cmd, false);
    bool const b_unverified_start = stop_unverified_start (&in);
    supervisor_poll_start_button (&g_config);

    while (b_more)
    {
        (void) ISpinLock (&g_lock, &saved);
        /* Handle one E-STOP per pass so repeats cannot delay admitted STOPs. */
        if (!b_estop_taken && (VEHICLE_CMD_ESTOP == g_estop_pending.opcode))
        {
            cmd = g_estop_pending;
            g_estop_pending.opcode = VEHICLE_CMD_STOP;
            b_estop_taken = true;
        }
        else
        {
            b_more = command_ring_pop (&g_ring, &cmd);
        }
        (void) ISpinUnlock (&g_lock, saved);
        if (b_more)
        {
            supervisor_handle_command (&g_ctx, cmd, &in, b_critical);
            /* The next command sees the effect of this one. */
            in = read_inputs (b_lapsed);
        }
    }
    supervisor_finish_calibration_start (&g_ctx, &in, b_critical);
    supervisor_consider_parked_starts (&g_ctx, &in, g_b_permit_renewed, b_critical, &g_config);
    publish_counts ();
    if (g_b_cal_held || !g_cal.b_waiting)
    {
        /* Keep the prior holds if motion could not install the calibration hold. */
        apply_holds (run_state_hold_mask (&g_ctx));
    }
    supervisor_watch_edges (run_state_of (&g_ctx), b_unverified_start);
    supervisor_watch_stop_timing (in.b_estop, false);

    update_attitude_hold (&in, b_critical);

    if (0U == (g_passes % HEALTH_PASSES))
    {
        b_critical = health_check ();
    }
    b_critical = b_critical || g_b_motion_timing_fault;
    if (!b_critical)
    {
        b_renew_failed = (0 != motion_renew_permit (PERMIT_LEASE_MS));
        g_b_permit_renewed = g_b_permit_renewed || !b_renew_failed;
        /* A live physics task retries a missing IMU itself. A task that
           cannot run those retries needs the board watchdog. */
        if ((g_live.stalled & (1U << LIVE_PHYSICS)) == 0U)
        {
            feed_watchdog ();
        }
    }
    g_b_renew_skipped = run_permit_skipped (b_critical, b_renew_failed);
    if ((SELFTEST_PASSES == g_passes) && !g_b_selftest_done)
    {
        g_b_selftest_done = true;
        supervisor_selftest ();
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

/*** end of file ***/
