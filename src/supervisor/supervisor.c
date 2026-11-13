/*
 *	supervisor.c
 *	The supervisor task: mission control and health (D120, D160).
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
#include <bsp/libbsp.h>

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
#define PHYSICS_FAULT_I2C_BIT (1U)

/* The run permit lease. The motion task holds the car when it lapses. */
#define PERMIT_LEASE_MS (200U)

/* Watchdog timeout, when the build turns the watchdog on. */
#define WATCHDOG_TIMEOUT_MS (1000U)

/* Event p2 of cmd_reject: source in the high byte, reason in the low byte. */
#define REJECT_SOURCE_SHIFT (8U)

/* The task sleeps at most this long when the cyclic handler does not wake it. */
#define SLEEP_TIMEOUT_MS (SUPERVISOR_PASS_MS + SUPERVISOR_PASS_MS)

/* The wheels must move at least this fast for the progress check to expect travel. */
#define PROGRESS_MIN_TARGET_UM_S (10000)

#define UM_PER_MM (1000)

static T_SPLOCK            g_lock;
static command_ring_t      g_ring;
static run_ctx_t           g_ctx;
static liveness_t          g_live;
static progress_t          g_progress;
static stuck_t             g_stuck;
static drive_t             g_drive;
static edge_watch_t        g_edge;
static supervisor_config_t g_config;

static volatile uint8_t  g_turn_pending = SUPERVISOR_NO_TURN;
static volatile uint8_t  g_turn_source  = 0U;
static volatile bool     g_turning      = false;
static volatile uint16_t g_passes       = 0U;

/* The holds the supervisor has applied to the motion service. */
static uint32_t g_applied_holds   = 0U;
static uint8_t  g_attitude        = TELEMETRY_ATTITUDE_STATE_OK;
static bool     g_b_stalled_fault = false;
static bool     g_b_selftest_done = false;

#if PICO_CAR_BUTTON_START
#define START_BUTTON_PIN (20U)
#define START_BUTTON_DEBOUNCE_PASSES (3U)
#define START_BUTTON_PULL_DOWN (1UL << 2U)
#define START_BUTTON_PULL_UP   (1UL << 3U)
#define START_BUTTON_SCHMITT   (1UL << 1U)

static uint8_t g_button_passes = 0U;
/* A button held through boot must first be released and pressed again. */
static bool g_b_button_sample_down = true;
static bool g_b_button_stable_down = true;

static bool start_button_pressed (void)
{
    bool const b_down = (0U == gpio_get_val (START_BUTTON_PIN));

    if (b_down != g_b_button_sample_down)
    {
        g_b_button_sample_down = b_down;
        g_button_passes = 0U;
    }
    if (g_button_passes < START_BUTTON_DEBOUNCE_PASSES)
    {
        g_button_passes++;
    }
    if ((START_BUTTON_DEBOUNCE_PASSES == g_button_passes) && (b_down != g_b_button_stable_down))
    {
        g_b_button_stable_down = b_down;
        return b_down;
    }
    return false;
}
#endif

ER supervisor_init (const supervisor_config_t * p_config)
{
    ER result = E_PAR;

    if (p_config != NULL)
    {
        g_config = *p_config;
        (void) InitSpinLock (&g_lock);
        command_ring_init (&g_ring);
        run_state_init (&g_ctx, 0 != PICO_CAR_BUTTON_START);
#if PICO_CAR_BUTTON_START
        result = gpio_set_pin (START_BUTTON_PIN, GPIO_MODE_IN);
        if (E_OK != result)
        {
            return result;
        }
        clr_w (GPIO (START_BUTTON_PIN), START_BUTTON_PULL_DOWN);
        set_w (GPIO (START_BUTTON_PIN), START_BUTTON_PULL_UP | START_BUTTON_SCHMITT);
        LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "dev-stack d17f2c0 motion fault latch; waiting for GP20 START");
#endif
        liveness_init (&g_live, p_config->required_mask);
        progress_init (&g_progress);
        stuck_init (&g_stuck);
        drive_init (&g_drive);
        edge_init (&g_edge);

        if (PLATFORM_RESET_WATCHDOG == platform_reset_reason ())
        {
            system_fault_set (FAULT_ABNORMAL_RESET_BIT);
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
    UINT saved    = 0U;
    bool b_stored = false;

    (void) ISpinLock (&g_lock, &saved);
    b_stored = command_ring_push (&g_ring, opcode, source);
    (void) ISpinUnlock (&g_lock, saved);

    return b_stored ? E_OK : E_TMOUT;
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

uint16_t supervisor_alive_mask (void)
{
    return (uint16_t) ((uint32_t) g_live.armed & ~(uint32_t) g_live.stalled);
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
          (int16_t) opcode, (uint16_t) RUN_REJECT_BUSY);
}

static run_inputs_t read_inputs (void)
{
    run_inputs_t        in = { .b_follow = true };
    physics_status_t    physics;
    motion_status_t     motion;
    obstacle_status_t   obstacle;

    if ((E_OK == physics_snapshot (&physics)) && physics.ready)
    {
        in.b_lifted     = (TELEMETRY_ATTITUDE_STATE_LIFTED == physics.attitude_state);
        in.b_overturned = (TELEMETRY_ATTITUDE_STATE_OVERTURNED == physics.attitude_state);
    }
    if (0 == motion_status (&motion))
    {
        in.b_estop = motion.stop_requested;
        in.b_motion_fault = (MOTION_STATE_FAULT == motion.state);
    }
    if (E_OK == obstacle_snapshot (&obstacle))
    {
        in.b_follow = (TELEMETRY_OBSTACLE_STATE_MONITOR == obstacle.state);
        in.b_obstacle_stopped = (TELEMETRY_OBSTACLE_STATE_STOPPED == obstacle.state);
    }
    if (g_turning || (SUPERVISOR_NO_TURN != g_turn_pending))
    {
        in.b_follow = false;
    }

    return in;
}

/* Carry out what a step decided. */
static void apply (const run_result_t * p_res, vehicle_cmd_t cmd, bool b_command)
{
    if (b_command)
    {
        if (p_res->b_accepted)
        {
            emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_CMD_ACCEPT,
                  (int16_t) cmd.opcode, (uint16_t) cmd.source);
        }
        else if (p_res->b_rejected)
        {
            emit ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR, (uint8_t) TELEMETRY_EVT_CMD_REJECT,
                  (int16_t) cmd.opcode,
                  (uint16_t) (((uint32_t) cmd.source << REJECT_SOURCE_SHIFT) | (uint32_t) p_res->reason));
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
    }
    if (p_res->b_obstacle_stop)
    {
        (void) obstacle_post_cmd (OBSTACLE_CMD_STOP);
    }
    if (p_res->b_obstacle_reset)
    {
        (void) obstacle_post_cmd (OBSTACLE_CMD_RESET);
    }
    if (p_res->b_obstacle_start)
    {
        (void) obstacle_post_cmd (OBSTACLE_CMD_START);
    }
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

/* Run start and stop, the obstacle cycle and the humps (D163). */
static void watch_edges (void)
{
    edge_event_t      ev[EDGE_EVENTS_MAX];
    obstacle_status_t obstacle;
    physics_status_t  physics;

    emit_edges ((uint8_t) TELEMETRY_EVENT_SOURCE_SUPERVISOR,
                ev, edge_run (&g_edge, (uint8_t) run_state_of (&g_ctx), ev));
    if (E_OK == obstacle_snapshot (&obstacle))
    {
        obstacle_plan_summary_t plan = { 0U, 0U, 0U, 0U, 0U, 0U, false };
        edge_obstacle_in_t      in;

        (void) obstacle_copy_plan (&plan);
        in.state          = obstacle.state;
        in.cause          = obstacle.cause;
        in.echo_class     = obstacle.echo_class;
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
        else if ((TELEMETRY_ATTITUDE_STATE_OK == state) &&
                 (TELEMETRY_ATTITUDE_STATE_OVERTURNED == g_attitude))
        {
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE, "car upright, needs RESET then START");
        }
        else
        {
            /* No event. */
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

    if (drive_update (&g_drive, &in, HEALTH_CHECK_MS))
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

    liveness_check (&g_live, liveness_take (), &fresh, &back);
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

        b_moving_expected = b_driven && b_free && feedback.fresh;
        if (progress_update (&g_progress, b_moving_expected, travel_abs_mm (&feedback),
                             HEALTH_CHECK_MS))
        {
            report_stall (SUPERVISOR_STALL_PROGRESS, 0U);
        }
        b_progress_stall = g_progress.b_tripped;
        check_drive (&diag, &feedback, b_free);

        if ((g_config.p_line_edges != NULL) &&
            stuck_update (&g_stuck, b_free && !g_turning,
                          g_config.p_line_edges (), telemetry_odometer_mm (&feedback)))
        {
            report_stall (SUPERVISOR_STALL_SENSOR, 0U);
        }
    }

    g_b_stalled_fault = (0U != g_live.stalled) || b_progress_stall;
    system_fault_assign (FAULT_TASK_STALLED_BIT, g_b_stalled_fault ? FAULT_TASK_STALLED_BIT : 0U);

    return liveness_critical_stalled (&g_live);
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
    run_inputs_t  in = read_inputs ();
    run_result_t  res;
    vehicle_cmd_t cmd        = { 0U, 0U };
    UINT          saved      = 0U;
    bool          b_more     = true;
    bool          b_critical = liveness_critical_stalled (&g_live);

    run_state_tick (&g_ctx, &in, &res);
    apply (&res, cmd, false);

#if PICO_CAR_BUTTON_START
    if (start_button_pressed () &&
        ((RUN_STATE_RUNNING != run_state_of (&g_ctx)) || in.b_obstacle_stopped))
    {
        cmd = (vehicle_cmd_t){VEHICLE_CMD_START, VEHICLE_SRC_BUTTON};
        run_state_command (&g_ctx, cmd, &in, &res);
        apply (&res, cmd, true);
        in = read_inputs ();
    }
#endif

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
            in = read_inputs ();
        }
    }
    apply_holds (run_state_hold_mask (&g_ctx));
    watch_edges ();

    {
        physics_status_t physics;

        if ((E_OK == physics_snapshot (&physics)) && physics.ready)
        {
            note_attitude (physics.attitude_state);
        }
    }

    if (0U == (g_passes % HEALTH_PASSES))
    {
        b_critical = health_check ();
    }
    if (!b_critical)
    {
        if (!g_ctx.b_wait_button)
        {
            (void) motion_renew_permit (PERMIT_LEASE_MS);
        }
        platform_wdt_feed ();
    }
    if ((SELFTEST_PASSES == g_passes) && !g_b_selftest_done)
    {
        g_b_selftest_done = true;
        selftest ();
    }
    if (0U == (g_passes % BACKGROUND_PASSES))
    {
#if PICO_CAR_BUTTON_START
        physics_status_t physics;

        if (E_OK == physics_snapshot (&physics))
        {
            LOG_INFO (LOG_TASK_SUPERVISOR, LOG_MOD_NONE,
                      "IMU ready=%d cal=%d attitude=%d pitch=%d roll=%d magnitude=%d faults=%d",
                      (int) physics.ready, (int) physics.calibrated, (int) physics.attitude_state,
                      (int) physics.pitch_cdeg, (int) physics.roll_cdeg,
                      (int) physics.amag_mg, (int) physics.faults);
        }
#endif
        if (g_config.p_background != NULL)
        {
            g_config.p_background ();
        }
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
