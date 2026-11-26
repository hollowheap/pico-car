/** @file navigation.c
 *
 * @brief Buddy 5 Phase 2 glue between the obstacle statechart, the line
 *        task in main.c and the motion service.
 *
 * What the line task does with Buddy 5 (design review 4.1):
 *   - OBSTACLE      hold the car, wait EXTRAS_NAV_BRAKE_MS, send SCAN_REQ(id).
 *   - SCAN_RESULT   OK or CLEAR: send ACCEPT(id), and keep the plan: the
 *                   side, B (the closest reading) and that side's raw
 *                   cleared angle.
 *   - BYPASS and    drive the revised bypass [Chat] from the wheel odometry
 *     REACQUIRE     (positions are for the middle of the wheel axle). Every
 *                   change of heading is a rotation: both wheels opposite,
 *                   so the car turns on the middle of its axle.
 *       AHEAD       straight on B / 2 towards the obstacle, but keeping the
 *                   axle EXTRAS_OUT_SWING_MM from it; backs up when the car
 *                   stopped closer than that (extras_plan_ahead_mm()).
 *       SPIN_AWAY   rotate 90 degrees away from the line, the side the
 *                   planner chose (remembered for the 360 degree map).
 *       OUT         straight on E = A + EXTRAS_OUT_SIDE_MM (room for the
 *                   rotation beside the obstacle, IR protrusion included),
 *                   A = B x tan(cleared angle) from the first scan, capped
 *                   (extras_plan_room_mm()).
 *       SPIN_PARALLEL rotate back, parallel to the line.
 *       LEVEL       straight on B + EXTRAS_LEVEL_BUFFER_MM (plus what AHEAD
 *                   fell short of B / 2), level with the obstacle's near
 *                   face.
 *       SPIN_FACE   rotate to face the line (and the obstacle).
 *       SWEEP       stand still (collision guard off, obstacle_extras_set_sweep())
 *                   and sweep the servo +-EXTRAS_SIDE_PAN_DECI, then
 *                   judge the readings (extras_plan_side_scan()):
 *                     lane to the line clear -> TRACK_OUT (or, if an echo
 *                     lies inside the swing circle, SPIN_BACK and MOVE_ON
 *                     EXTRAS_TRACK_MIN_MOVE_MM without rotating there);
 *                     blocked -> SPIN_BACK, MOVE_ON, SPIN_FACE, SWEEP again
 *                     (no limit on the number of scans).
 *       TRACK_OUT   chicken-head rotation scan, pass 1: rotate
 *                   EXTRAS_TRACK_PAN_DECI towards the obstacle's side at a
 *                   constant speed while the servo turns the other way, so
 *                   the beam keeps pointing straight at the line; every
 *                   echo becomes a floor point checked against the lane and
 *                   the swing circle (extras_plan_track_*()).
 *       TRACK_BACK  pass 2: rotate back with the beam tilted
 *                   EXTRAS_TRACK_TILT_DECI towards the obstacle's side. Clear
 *                   -> TO_LINE; blocked -> SPIN_BACK and MOVE_ON
 *                   (lane half width - closest side, at least
 *                   EXTRAS_TRACK_MIN_MOVE_MM); too many unknown pings -> one
 *                   retry, then blocked.
 *       SPIN_BACK   rotate back, parallel to the line.
 *       MOVE_ON     straight on (D + H + EXTRAS_MOVE_BUFFER_MM) / 2, H = half
 *                   width on the obstacle side (IR protrusion on the left),
 *                   D from the forward cleared angle (or
 *                   EXTRAS_SIDE_ASSUMED_DECI when there is none).
 *       TO_LINE     straight at the line, full speed until the IR sensors
 *                   are EXTRAS_LINE_SLOW_MM short of it by odometry, then
 *                   slowly, until an IR sensor has seen
 *                   black for EXTRAS_LINE_MIN_MM, no more than
 *                   EXTRAS_LINE_WINDOW_MM short of the line; stop
 *                   EXTRAS_LINE_GIVE_UP_MM past it.
 *       CENTRE      on EXTRAS_NAV_CENTRE_MM at full speed, so the axle sits on
 *                   the line.
 *       MAP         rotate in the remembered direction (the same way as
 *                   SPIN_AWAY), servo straight ahead, until an IR sensor
 *                   finds the line (at most a full turn), then send
 *                   LINE_FOUND(id) until Buddy 5 is back in MONITOR and
 *                   line following resumes from there.
 *     A virtual bumper (a wheel much slower than it was told) stops the car.
 *   - FAULT         hold the car; retry RESET + START every 3 s, except after
 *                   a collision, a bump or a bypass that Navigation gave up
 *                   on: the car then stays stopped until it is reset.
 *   - MONITOR       release the hold and let the line follower drive,
 *                   except while Buddy 5's latest reading is a VALID echo
 *                   within the detection distance (no creeping forward
 *                   into an obstacle while a retry is in progress).
 *
 * Position frame (dead reckoning from the wheel encoders): x is sideways,
 * positive to the left of the original line; y is along the line; both
 * are for the middle of the axle and start at 0 when BYPASS begins. The
 * heading is worked out from the total wheel difference, so it does not
 * drift between passes. "Side" and "away" count towards the side being
 * passed.
 *
 * Everything that happens is also written to the flight recorder
 * (extras_log.c), which prints it when a USB serial monitor is opened.
 *
 * Links given to Buddy 5:
 *   - Emergency stop: sets MOTION_HOLD_OBSTACLE. NOTE: It does not call
 *     motion_service_request_stop(), which latches until reboot, because
 *     every Buddy 5 fault can be cleared with RESET + START.
 *   - Distance: average forward displacement of the two wheels from
 *     motion_service_read_status(), in mm.
 *
 * NOTE: The kernel API types ER, ID, UB and SYSTIM are used where the
 * kernel requires them (deviation from BARR-C 5.2.a, see
 * testing/README.md). Casts to (INT) feed LOG_CONT()
 * only; every value printed is below 2^16.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "platform/log.h"

#include "motion/motion_service.h"
#include "navigation.h"
#include "obstacle/extras/extras.h"
#include "obstacle/obstacle.h"
#include "obstacle/obstacle_pack.h"

/* Micrometres per millimetre. */
#define UM_PER_MM (1000)

/* Deci-degrees per degree, and a right angle. */
#define DECI_PER_DEG     (10)
#define DECI_RIGHT_ANGLE (900)

/* Q14 fixed point: 16384 = 1.0. */
#define Q14_ONE (16384)

/* Whole, for percentages. */
#define PERCENT_FULL (100)

/* The two wheels of the motion service. */
#define WHEEL_LEFT  (0U)
#define WHEEL_RIGHT (1U)
#define WHEELS      (2U)

/* Bypass phases inside BYPASS and REACQUIRE (see the file comment). The
 * order matches the names in extras_log.c. */
#define PHASE_AHEAD         (0U)
#define PHASE_SPIN_AWAY     (1U)
#define PHASE_OUT           (2U)
#define PHASE_SPIN_PARALLEL (3U)
#define PHASE_LEVEL         (4U)
#define PHASE_SPIN_FACE     (5U)
#define PHASE_SWEEP         (6U)
#define PHASE_TRACK_OUT     (7U)
#define PHASE_TRACK_BACK    (8U)
#define PHASE_SPIN_BACK     (9U)
#define PHASE_MOVE_ON       (10U)
#define PHASE_TO_LINE       (11U)
#define PHASE_CENTRE        (12U)
#define PHASE_MAP           (13U)
#define PHASE_DONE          (14U)
#define PHASE_HALT          (15U)

/* First and last sweep slots (+-EXTRAS_SIDE_PAN_DECI). */
#define SWEEP_FIRST_SLOT                                                       \
    ((uint8_t)(EXTRAS_SLOT_CENTRE -                                                \
               (uint32_t)(EXTRAS_SIDE_PAN_DECI / EXTRAS_SLOT_STEP_DECI)))
#define SWEEP_LAST_SLOT                                                        \
    ((uint8_t)(EXTRAS_SLOT_CENTRE +                                                \
               (uint32_t)(EXTRAS_SIDE_PAN_DECI / EXTRAS_SLOT_STEP_DECI)))

/* TRACK record results (extras_log.c). */
#define TRACK_CLEAR        (0U)
#define TRACK_BLOCKED      (1U)
#define TRACK_RETRY        (2U)
#define TRACK_UNKNOWN_STOP (3U)

/* TRACK record pass numbers. */
#define TRACK_PASS_OUT  (1U)
#define TRACK_PASS_BACK (2U)

/* Steering intent, turned into LEFT / RIGHT by the side being passed. */
#define INTENT_STRAIGHT (0U)
#define INTENT_AWAY     (1U)
#define INTENT_TOWARD   (2U)

/* SWEEP record results (extras_log.c). */
#define SWEEP_CLEAR   (0U)
#define SWEEP_CLEARED (1U)
#define SWEEP_ASSUMED (2U)

/* LINE record events (extras_log.c). */
#define LINE_ACCEPTED (0U)
#define LINE_OUTSIDE  (1U)
#define LINE_SHORT    (2U)
#define LINE_EVENTS   (3U)

/* Why Navigation gave up on a bypass (HALT record, extras_log.c). */
#define HALT_BUMP      (0U)
#define HALT_MISSED    (1U)
#define HALT_COLLISION (2U)
#define HALT_NO_PATH   (3U)

/* Reading used for OPEN, UNKNOWN and STUCK pings (farther than any echo). */
#define NAV_FAR_MM (4095U)

/* Value that never matches a real state, so the first one is always
 * printed. */
#define NAV_UNSEEN (0xFFU)

/* Drive key that never matches a real one, so the first is logged. */
#define NAV_DRIVE_UNSEEN (UINT32_MAX)

/* Bit positions of the drive key (steer, left mm/s, right mm/s); the
 * speeds are offset so that a backward speed stays positive. */
#define DRIVE_OFFSET_MM_S (128)
#define DRIVE_LEFT_SH     (2U)
#define DRIVE_RIGHT_SH    (12U)

/* IR and steer bits of a TRACE record (see extras_log.c). */
#define TRACE_IR_LEFT  (0x1U)
#define TRACE_IR_RIGHT (0x2U)
#define TRACE_STEER_SH (2U)

/* Largest value logged in a 16-bit field. */
#define LOG_I16_MAX (32767)

/* Navigation state; only the line task touches it. NOTE: Booleans are b_
 * members of a g_ struct, which satisfies both BARR-C 7.1.m/7.1.o and the
 * team .clang-tidy. */
typedef struct
{
    bool     b_enabled;
    bool     b_armed;
    bool     b_scan_pending;
    bool     b_servo_wait;
    bool     b_pausing;
    bool     b_ir_cleared;
    bool     b_held;
    bool     b_no_retry;
    bool     b_black;
    bool     b_line_noted;
    bool     b_track_blocked;
    bool     b_low[WHEELS];
    bool     b_vel_ok[WHEELS];
    uint8_t  req_id;
    uint8_t  next_id;
    uint8_t  no_path_count;
    uint8_t  start_count;
    uint8_t  last_state;
    uint8_t  side;
    uint8_t  phase;
    uint8_t  wait_seq;
    uint8_t  ping_seq;
    uint8_t  sweep_slot;
    uint8_t  sweep_pings;
    uint8_t  scans;
    uint8_t  track_unknown;
    uint8_t  track_retry;
    int16_t  look_deci;
    int16_t  track_sent;
    uint16_t sweep_mm[EXTRAS_SLOT_COUNT];
    uint32_t status;
    uint32_t wait_ms;
    uint32_t pause_ms;
    uint32_t last_drive;
    uint32_t hold_ms;
    uint32_t retry_ms;
    uint32_t line_ms;
    uint32_t trace_ms;
    uint32_t motion_word;
    uint32_t cmd_ms;
    uint32_t closest_mm;
    uint32_t room_mm;
    uint32_t out_mm;
    uint32_t level_mm;
    uint32_t move_mm;
    uint32_t low_ms[WHEELS];
    int32_t  ahead_end_mm;
    int32_t  map_start;
    int32_t  track_h0;
    int32_t  track_away0;
    int32_t  track_gaze;
    int32_t  track_gap_mm;
    uint32_t track_lookahead_mm;
    int32_t  phase_mm;
    int32_t  black_mm;
    int32_t  wheel_mm[WHEELS];
    int32_t  last_cmd[WHEELS];
    int32_t  vel_um_s[WHEELS];
    int64_t  wheel_um[WHEELS];
    int64_t  base_um[WHEELS];
    int64_t  prev_um[WHEELS];
    int64_t  x_um;
    int64_t  y_um;
} nav_state_t;

/* Last good odometer reading; only the Buddy 5 task touches it. */
typedef struct
{
    uint32_t last_mm;
} nav_odo_t;

static nav_state_t g_nav = {
    .b_enabled          = false,
    .b_scan_pending     = false,
    .b_servo_wait       = false,
    .b_pausing          = false,
    .b_ir_cleared       = false,
    .b_held             = false,
    .b_no_retry         = false,
    .b_black            = false,
    .b_line_noted       = false,
    .b_track_blocked    = false,
    .b_low              = {false, false},
    .b_vel_ok           = {false, false},
    .req_id             = 0U,
    .next_id            = 0U,
    .no_path_count      = 0U,
    .last_state         = NAV_UNSEEN,
    .side               = EXTRAS_SIDE_LEFT,
    .phase              = PHASE_DONE,
    .wait_seq           = 0U,
    .ping_seq           = 0U,
    .sweep_slot         = 0U,
    .sweep_pings        = 0U,
    .scans              = 0U,
    .track_unknown      = 0U,
    .track_retry        = 0U,
    .look_deci          = 0,
    .track_sent         = 0,
    .sweep_mm           = {0U},
    .status             = 0U,
    .wait_ms            = 0U,
    .pause_ms           = 0U,
    .last_drive         = NAV_DRIVE_UNSEEN,
    .hold_ms            = 0U,
    .retry_ms           = 0U,
    .line_ms            = 0U,
    .trace_ms           = 0U,
    .motion_word        = UINT32_MAX,
    .cmd_ms             = 0U,
    .closest_mm         = 0U,
    .room_mm            = 0U,
    .out_mm             = 0U,
    .level_mm           = 0U,
    .move_mm            = 0U,
    .low_ms             = {0U, 0U},
    .ahead_end_mm       = 0,
    .map_start          = 0,
    .track_h0           = 0,
    .track_away0        = 0,
    .track_gaze         = 0,
    .track_gap_mm       = 0,
    .track_lookahead_mm = 0U,
    .phase_mm           = 0,
    .black_mm           = 0,
    .wheel_mm           = {0, 0},
    .last_cmd           = {0, 0},
    .vel_um_s           = {0, 0},
    .wheel_um           = {0, 0},
    .base_um            = {0, 0},
    .prev_um            = {0, 0},
    .x_um               = 0,
    .y_um               = 0,
};
static nav_odo_t g_odo = {0U};

static uint32_t now_ms (void);
static int16_t  clamp_i16 (int32_t value);
static void log_rec (uint8_t kind, uint8_t val_a, uint8_t val_b, uint8_t val_c,
                     int32_t val_d, int32_t val_e, int32_t val_f);
static void post (uint32_t type, uint8_t req_id);
static void hold_car (bool b_held);
static int32_t  side_sign (void);
static int32_t  heading_deci (void);
static int32_t  away_deci (void);
static int32_t  lateral_mm (void);
static int32_t  ahead_mm (void);
static int32_t  travel_mm (void);
static void     reset_pose (void);
static void     update_pose (void);
static uint32_t intent_steer (uint32_t intent);
static void     drive_steer (uint32_t steer, int32_t inside_um_s,
                             int32_t outside_um_s);
static void     drive_speeds (uint32_t steer, int32_t left_um_s,
                              int32_t right_um_s);
static void     drive_stop (void);
static void     drive_hold (int32_t target_deci, int32_t speed_um_s);
static bool     spin_to (int32_t target_deci);
static void     spin_drive (uint32_t intent, int32_t speed_um_s);
static bool     is_reverse_pause (int32_t const p_speed[WHEELS]);
static void     set_phase (uint8_t phase);
static void     begin_bypass (void);
static void     halt_bypass (uint8_t reason);
static void     step_bypass (uint32_t state, uint32_t status, bool b_line_seen,
                             uint32_t now);
static void     run_phase (uint32_t state, uint32_t status, bool b_line_seen,
                           uint32_t now);
static void     run_leg (void);
static void     run_sweep (uint32_t status);
static void     next_sweep_slot (void);
static void     judge_sweep (void);
static void     log_sweep (extras_plan_side_t const * p_side);
static void     run_map (bool b_line_seen);
static bool     is_swing_clear (void);
static void     run_track (uint32_t status);
static void     track_ping (uint32_t status, uint32_t dist_mm, int32_t psi);
static void     track_echo (uint32_t dist_mm, int32_t psi);
static bool     track_spin (int32_t target_deci);
static void     finish_track (void);
static void     log_track (uint8_t result, uint32_t move_mm);
static int16_t  clamp_look (int32_t deci);
static void     sync_ping (void);
static void     start_track_pass (uint8_t phase);
static void     run_to_line (bool b_line_seen);
static void     note_line (uint8_t event, int32_t ir_side_mm, int32_t black_mm);
static bool     take_ping (uint32_t status, uint32_t * p_dist_mm);
static bool     is_ping_at_look (uint32_t status);
static void     set_look (int16_t deci);
static void     start_servo_wait (void);
static bool     check_bump (uint32_t now);
static bool     is_obstacle_close (void);
static bool     is_servo_moving (uint32_t status, uint32_t now);
static void     report_state (uint32_t status);
static bool     step_stopped (uint32_t status, uint32_t now);
static void     keep_plan (extras_plan_result_t const * p_result, uint32_t side);
static void     explain_scan (uint16_t const *   p_profile,
                              extras_plan_result_t * p_result);
static void     handle_event (uint32_t evt, uint32_t now);
static void     look_for_line (bool b_line_seen, uint32_t now);
static void     update_motion (void);
static int64_t true_wheel_um (motion_sample_t const * p_sample, uint32_t wheel);
static int32_t true_wheel_speed (motion_sample_t const * p_sample,
                                 uint32_t                wheel);
static void    report_fault (motion_service_status_t const * p_status);
static void    trace (uint32_t status, bool b_left, bool b_right, uint32_t now);
static ER      nav_stop (void);
static ER      nav_distance_mm (uint32_t * p_distance_mm);

/*!
 * @brief Set up Buddy 5 with the motion links. The supervisor starts it.
 *
 * Call once during startup on core 0, after drv_gpio_irq_init(),
 * motion_setup() and ir_capture_init(). If it fails, extras_nav_step()
 * only services the flight recorder, so line following works without
 * Buddy 5 and the failure can still be read back.
 *
 * @return The result of obstacle_extras_setup().
 */
ER extras_nav_init (void)
{
    obstacle_extras_links_t const links  = {0, nav_stop, nav_distance_mm};
    ER const                     result = obstacle_extras_setup(&links);

    log_rec(EXTRAS_LOG_BOOT, 0U, 0U, 0U, (int32_t)result, 0, 0);

    if (E_OK == result)
    {
        g_nav.b_enabled = true;
    }

    return (result);
}

static void reset_route (uint32_t now)
{
    g_nav.b_scan_pending = false;
    g_nav.b_servo_wait = false;
    g_nav.b_no_retry = false;
    g_nav.no_path_count = 0U;
    g_nav.req_id = NAV_UNSEEN;
    g_nav.last_state = NAV_UNSEEN;
    g_nav.phase = PHASE_DONE;
    g_nav.retry_ms = now;
    obstacle_extras_set_track(false);
    obstacle_extras_set_sweep(false);
}

/*!
 * @brief One pass of the obstacle handshake. Call every line-task loop.
 *
 * @param[in] b_line_left  Left IR sensor sees the line.
 * @param[in] b_line_right Right IR sensor sees the line.
 * @param[in] b_armed Supervisor's current run permission.
 * @param[in] start_count Supervisor's START generation.
 * @param[in] b_drive_allowed The line task's motion and tape inputs are usable.
 *
 * @return True if Buddy 5 is in charge of the motors this pass, so the
 *         line follower must not send a speed.
 */
bool extras_nav_step (bool b_line_left, bool b_line_right, bool b_armed,
                      uint8_t start_count, bool b_drive_allowed)
{
    bool     b_in_charge = false;
    uint32_t evt         = 0U;
    uint32_t state       = EXTRAS_STATE_MONITOR;
    uint32_t status      = 0U;
    uint32_t now         = 0U;

    extras_log_service(b_armed);
    update_motion();

    if (g_nav.b_enabled)
    {
        now = now_ms();
        if ((g_nav.b_armed != b_armed) || (g_nav.start_count != start_count))
        {
            reset_route(now);
            g_nav.b_armed = b_armed;
            g_nav.start_count = start_count;
        }
        if (!b_armed)
        {
            /* STOP is owned by the supervisor. Retire old navigation events
             * and the navigator's hold before the next START probe. */
            while (E_OK == obstacle_extras_get_event(&evt, TMO_POL))
            {
                /* Discard queued detour state from the stopped run. */
            }
            hold_car(false);
            return true;
        }

        while (E_OK == obstacle_extras_get_event(&evt, TMO_POL))
        {
            uint32_t const type = extras_pack_get(evt, EXTRAS_EV_TYPE_SH, EXTRAS_EV_TYPE_MK);

            status = obstacle_extras_get_status();
            state  = extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK);
            if ((EXTRAS_EVT_OBSTACLE != type) || (EXTRAS_STATE_WAIT_STOP == state))
            {
                handle_event(evt, now);
            }
        }

        if (g_nav.b_scan_pending && ((now - g_nav.hold_ms) >= EXTRAS_NAV_BRAKE_MS))
        {
            g_nav.b_scan_pending = false;
            post(EXTRAS_CMD_SCAN_REQ, g_nav.req_id);
        }

        status = obstacle_extras_get_status();
        state  = extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK);
        report_state(status);
        trace(status, b_line_left, b_line_right, now);

        switch (state)
        {
            case EXTRAS_STATE_MONITOR:
                b_in_charge = (is_servo_moving(status, now)) ||
                              (is_obstacle_close());
                hold_car(b_in_charge);
                if (!b_in_charge)
                {
                    g_nav.req_id = NAV_UNSEEN;
                }
            break;

            case EXTRAS_STATE_BYPASS:
            case EXTRAS_STATE_REACQUIRE:
                /* NOTE: the side comes from the plan, never from the STEER
                 * word, which is read at a different moment. */
                if (b_drive_allowed &&
                    (g_nav.req_id == extras_pack_get(status, EXTRAS_ST_ID_SH, EXTRAS_ST_ID_MK)))
                {
                    step_bypass(state, status, b_line_left || b_line_right, now);
                }
                else
                {
                    hold_car(true);
                }
                b_in_charge = true;
            break;

            case EXTRAS_STATE_STOPPED:
                b_in_charge = step_stopped(status, now);
            break;

            default:
                /* WAIT_STOP, SCANNING and PLAN: the car stands still. */
                hold_car(true);
                b_in_charge = true;
            break;
        }
    }

    return (b_in_charge);
}

/*!
 * @brief One line-task pass in BYPASS or REACQUIRE: stand still while the
 *        servo swings, otherwise check the bumper and drive the phase.
 *
 * @param[in] state       Buddy 5 state (BYPASS or REACQUIRE).
 * @param[in] status      Packed Buddy 5 status word.
 * @param[in] b_line_seen An IR sensor sees the line.
 * @param[in] now         Current time, ms.
 */
static void step_bypass (uint32_t state, uint32_t status, bool b_line_seen,
                         uint32_t now)
{
    g_nav.status = status;
    update_pose();

    if (PHASE_HALT == g_nav.phase)
    {
        hold_car(true);
    }
    else if (is_servo_moving(status, now))
    {
        /* A servo swing while a motor drives disturbs the encoders. */
        hold_car(true);
    }
    else
    {
        hold_car(false);

        if (check_bump(now))
        {
            halt_bypass(HALT_BUMP);
        }
        else
        {
            run_phase(state, status, b_line_seen, now);
        }
    }
}

/*!
 * @brief Drive the current bypass phase (see the file comment).
 *
 * NOTE: From SPIN_FACE on the bypass needs the servo, which follows
 * Navigation only in REACQUIRE; if Buddy 5 is still in BYPASS (the beam
 * ahead is not clear yet), the car waits.
 *
 * @param[in] state       Buddy 5 state (BYPASS or REACQUIRE).
 * @param[in] status      Packed Buddy 5 status word.
 * @param[in] b_line_seen An IR sensor sees the line.
 * @param[in] now         Current time, ms.
 */
static void run_phase (uint32_t state, uint32_t status, bool b_line_seen,
                       uint32_t now)
{
    if ((g_nav.phase >= PHASE_SPIN_FACE) && (EXTRAS_STATE_REACQUIRE != state))
    {
        drive_stop();
    }
    else
    {
        switch (g_nav.phase)
        {
            case PHASE_AHEAD:
            case PHASE_OUT:
            case PHASE_LEVEL:
            case PHASE_MOVE_ON:
            case PHASE_CENTRE:
                run_leg();
            break;

            case PHASE_SPIN_AWAY:
                if (spin_to(DECI_RIGHT_ANGLE))
                {
                    set_phase(PHASE_OUT);
                }
            break;

            case PHASE_SPIN_PARALLEL:
                if (spin_to(0))
                {
                    set_phase(PHASE_LEVEL);
                }
            break;

            case PHASE_SPIN_FACE:
                if (spin_to(-DECI_RIGHT_ANGLE))
                {
                    set_phase(PHASE_SWEEP);
                }
            break;

            case PHASE_SWEEP:
                run_sweep(status);
            break;

            case PHASE_TRACK_OUT:
            case PHASE_TRACK_BACK:
                run_track(status);
            break;

            case PHASE_SPIN_BACK:
                if (spin_to(0))
                {
                    set_phase(PHASE_MOVE_ON);
                }
            break;

            case PHASE_TO_LINE:
                run_to_line(b_line_seen);
            break;

            case PHASE_MAP:
                run_map(b_line_seen);
            break;

            default:
                /* DONE: stand still and hand back to line following. */
                drive_stop();
                look_for_line(PHASE_DONE == g_nav.phase, now);
            break;
        }
    }
}

/*!
 * @brief The straight legs that end on a distance: AHEAD (B / 2 towards
 *        the obstacle), OUT (A + half width + buffer, away from the
 *        line), LEVEL (B + buffer, along the line), MOVE_ON (along the line)
 *        and CENTRE (onto the line before the map).
 */
static void run_leg (void)
{
    int32_t const moved_mm = travel_mm() - g_nav.phase_mm;

    /* Casts are safe: every leg is far shorter than 2^31 mm and the speeds
     * are 70 000 or 140 000 um/s. */
    switch (g_nav.phase)
    {
        case PHASE_AHEAD:
            if (g_nav.ahead_end_mm < 0)
            {
                /* Too close to rotate: back straight up (no heading hold,
                 * the leg is short). */
                if (moved_mm <= g_nav.ahead_end_mm)
                {
                    set_phase(PHASE_SPIN_AWAY);
                }
                else
                {
                    drive_speeds(EXTRAS_STEER_STRAIGHT,
                                 -(int32_t)EXTRAS_NAV_BACKUP_UM_S,
                                 -(int32_t)EXTRAS_NAV_BACKUP_UM_S);
                }
            }
            else if (moved_mm >= g_nav.ahead_end_mm)
            {
                set_phase(PHASE_SPIN_AWAY);
            }
            else
            {
                drive_hold(0, (int32_t)EXTRAS_NAV_SPEED_UM_S);
            }
        break;

        case PHASE_OUT:
            if (moved_mm >= (int32_t)g_nav.out_mm)
            {
                set_phase(PHASE_SPIN_PARALLEL);
            }
            else
            {
                drive_hold(DECI_RIGHT_ANGLE, (int32_t)EXTRAS_NAV_SPEED_UM_S);
            }
        break;

        case PHASE_LEVEL:
            if (moved_mm >= (int32_t)g_nav.level_mm)
            {
                set_phase(PHASE_SPIN_FACE);
            }
            else
            {
                drive_hold(0, (int32_t)EXTRAS_NAV_SPEED_UM_S);
            }
        break;

        case PHASE_MOVE_ON:
            if (moved_mm >= (int32_t)g_nav.move_mm)
            {
                set_phase(PHASE_SPIN_FACE);
            }
            else
            {
                drive_hold(0, (int32_t)EXTRAS_NAV_SPEED_UM_S);
            }
        break;

        default:
            /* CENTRE */
            if (moved_mm >= (int32_t)EXTRAS_NAV_CENTRE_MM)
            {
                set_phase(PHASE_MAP);
            }
            else
            {
                drive_hold(-DECI_RIGHT_ANGLE, (int32_t)EXTRAS_NAV_SPEED_UM_S);
            }
        break;
    }
}

/*!
 * @brief SWEEP: the car stands still while the servo steps through the
 *        slot angles from -EXTRAS_SIDE_PAN_DECI to +EXTRAS_SIDE_PAN_DECI; at each,
 *        EXTRAS_SWEEP_PINGS pings taken at that angle (is_ping_at_look()) are
 *        used and the nearest VALID one is kept.
 *
 * @param[in] status Packed Buddy 5 status word.
 */
static void run_sweep (uint32_t status)
{
    uint32_t dist_mm = NAV_FAR_MM;

    drive_stop();

    /* A ping taken before the servo reached this slot does not count. */
    if ((take_ping(status, &dist_mm)) && (is_ping_at_look(status)))
    {
        uint16_t * const p_slot = &g_nav.sweep_mm[g_nav.sweep_slot];

        /* Cast is safe: a VALID distance is 12 bits. */
        if ((dist_mm < NAV_FAR_MM) && (dist_mm < (uint32_t)*p_slot))
        {
            *p_slot = (uint16_t)dist_mm;
        }

        /* Cast is safe: sweep_pings stops at EXTRAS_SWEEP_PINGS. */
        g_nav.sweep_pings = (uint8_t)(g_nav.sweep_pings + 1U);

        if (g_nav.sweep_pings >= EXTRAS_SWEEP_PINGS)
        {
            next_sweep_slot();
        }
    }
}

/*!
 * @brief SWEEP: point the servo at the next angle, or judge the sweep
 *        after the last one.
 */
static void next_sweep_slot (void)
{
    /* Cast is safe: sweep_slot stops at EXTRAS_SLOT_COUNT. */
    g_nav.sweep_pings = 0U;
    g_nav.sweep_slot  = (uint8_t)(g_nav.sweep_slot + 1U);

    if (g_nav.sweep_slot > SWEEP_LAST_SLOT)
    {
        judge_sweep();
    }
    else
    {
        /* Cast is safe: a slot angle is within +-600. */
        set_look((int16_t)extras_scan_slot_deci(g_nav.sweep_slot));
    }
}

/*!
 * @brief SWEEP done: judge it (extras_plan_side_scan()) and go on. The lane
 *        runs from the sensor to EXTRAS_LANE_EXTRA_MM past the line. A left
 *        servo angle looks forward along the line when passing on the
 *        left (the car faces the line from its left), and a right one when
 *        passing on the right. A clear lane is checked once more with the
 *        chicken-head rotation scan (TRACK_OUT) before TO_LINE, unless an
 *        echo lies inside the swing circle (then the car moves on first).
 */
static void judge_sweep (void)
{
    int32_t const sonar_mm =
        extras_plan_point_side_mm(lateral_mm(), away_deci(), EXTRAS_AXLE_TO_SONAR_MM);
    extras_plan_side_t side;

    /* Cast is safe: sonar_mm is non-negative and a few hundred mm. */
    uint32_t const line_mm = (sonar_mm > 0) ? (uint32_t)sonar_mm : 0U;

    extras_plan_side_scan(g_nav.sweep_mm, side_sign(), line_mm, EXTRAS_SIDE_PAN_DECI,
                      &side);

    /* Cast is safe: the count stops at 255 (only printed and logged). */
    if (g_nav.scans < UINT8_MAX)
    {
        g_nav.scans = (uint8_t)(g_nav.scans + 1U);
    }

    log_sweep(&side);
    set_look(0);

    if (side.b_lane_clear && (is_swing_clear()))
    {
        set_phase(PHASE_TRACK_OUT);
    }
    else if (side.b_lane_clear)
    {
        /* Something is within the swing circle: do not rotate here. The
         * rotation back retraces the arc SPIN_FACE has already swept. */
        LOG_CONT("[b5-nav] track: echo within the swing circle, "
                        "move on %d mm first\n",
                  (INT)EXTRAS_TRACK_MIN_MOVE_MM);
        g_nav.move_mm = EXTRAS_TRACK_MIN_MOVE_MM;
        set_phase(PHASE_SPIN_BACK);
    }
    else
    {
        g_nav.move_mm = side.move_mm;
        set_phase(PHASE_SPIN_BACK);
    }
}

/*!
 * @brief Print and log one side scan, with its readings as a profile.
 *
 * @param[in] p_side The judgement of the sweep.
 */
static void log_sweep (extras_plan_side_t const * p_side)
{
    uint16_t profile[EXTRAS_SLOT_COUNT];
    uint32_t slot   = 0U;
    uint32_t index  = 0U;
    uint8_t  result = SWEEP_CLEAR;

    for (slot = 0U; slot < EXTRAS_SLOT_COUNT; slot++)
    {
        uint16_t const range_mm = g_nav.sweep_mm[slot];

        profile[slot] = (EXTRAS_PLAN_NO_ECHO == range_mm)
                            ? extras_pack_slot(0U, EXTRAS_CLASS_OPEN)
                            : extras_pack_slot(range_mm, EXTRAS_CLASS_VALID);
    }

    if (p_side->b_lane_clear)
    {
        result = SWEEP_CLEAR;
    }
    else if (p_side->b_cleared)
    {
        result = SWEEP_CLEARED;
    }
    else
    {
        result = SWEEP_ASSUMED;
    }

    index = extras_log_profile(g_nav.req_id, profile);
    LOG_CONT("[b5-nav] side scan %d: %s, cleared %d deg, d %d mm, "
                    "move on %d mm\n",
              (INT)g_nav.scans,
              p_side->b_lane_clear ? "lane to the line clear" : "blocked",
              (INT)(p_side->clear_deci / DECI_PER_DEG), (INT)p_side->closest_mm,
              (INT)p_side->move_mm);

    /* Casts are safe: the angle is 0..60 degrees, the distances are below
     * 2^15 and index is below EXTRAS_LOG_PROFILES. */
    log_rec(EXTRAS_LOG_SWEEP, g_nav.scans, result,
            (uint8_t)(p_side->clear_deci / DECI_PER_DEG),
            (int32_t)p_side->closest_mm, (int32_t)p_side->move_mm,
            (int32_t)index);
}

/*!
 * @brief MAP: rotate in the remembered direction (the same way as
 *        SPIN_AWAY), servo straight ahead, until an IR sensor finds the
 *        line; then hand back to line following (DONE). Black that is
 *        already under the sensors when the rotation starts does not count
 *        (the sensors must have been off black first). No line within
 *        EXTRAS_MAP_TURN_DECI: stop.
 *
 * @param[in] b_line_seen An IR sensor sees the line.
 */
static void run_map (bool b_line_seen)
{
    int32_t const turned = away_deci() - g_nav.map_start;

    if (!b_line_seen)
    {
        g_nav.b_ir_cleared = true;
    }

    if (g_nav.b_ir_cleared && b_line_seen)
    {
        drive_stop();
        LOG_CONT("[b5-nav] map: line found after %d deg\n",
                  (INT)(turned / DECI_PER_DEG));
        log_rec(EXTRAS_LOG_MAP, 1U, 0U, 0U, 0, 0, turned);
        set_phase(PHASE_DONE);
    }
    else if (turned >= EXTRAS_MAP_TURN_DECI)
    {
        drive_stop();
        LOG_CONT("[b5-nav] map: no line in a full turn\n");
        log_rec(EXTRAS_LOG_MAP, 0U, 0U, 0U, 0, 0, turned);
        halt_bypass(HALT_MISSED);
    }
    else
    {
        /* Cast is safe: the speed is 46 666 um/s. */
        spin_drive(INTENT_AWAY, (int32_t)EXTRAS_NAV_SPIN_SLOW_UM_S);
    }
}

/*!
 * @brief SWEEP: tell whether the car may rotate where it stands, i.e. no
 *        sweep echo lies inside the circle its furthest point sweeps
 *        (extras_plan_track_near(), the sweep taken with psi = 0 and the beam
 *        at each servo angle).
 *
 * @return True if no echo is within EXTRAS_SWING_MM of the axle.
 */
static bool is_swing_clear (void)
{
    bool     b_clear = true;
    uint32_t slot    = 0U;

    for (slot = 0U; slot < EXTRAS_SLOT_COUNT; slot++)
    {
        uint16_t const range_mm = g_nav.sweep_mm[slot];
        int32_t        fwd_mm   = 0;
        int32_t        side_mm  = 0;

        if (EXTRAS_PLAN_NO_ECHO != range_mm)
        {
            /* Cast is safe: slot is 0..24. */
            extras_plan_track_point(0, extras_scan_slot_deci((uint8_t)slot), range_mm,
                                &fwd_mm, &side_mm);
            b_clear = b_clear && (!extras_plan_track_near(fwd_mm, side_mm));
        }
    }

    return (b_clear);
}

/*!
 * @brief TRACK_OUT and TRACK_BACK, every line-task pass (about 20 ms):
 *        aim the servo so the beam keeps the gaze direction, take any new
 *        ping, and rotate on.
 *
 * NOTE: The servo is set with obstacle_extras_set_look() directly, not
 * set_look(): there is no servo wait and the wheels keep turning (the
 * Buddy 5 task is in tracking mode, see scan_task.c).
 *
 * @param[in] status Packed Buddy 5 status word.
 */
static void run_track (uint32_t status)
{
    int32_t const psi     = heading_deci() - g_nav.track_h0;
    int16_t const servo   = clamp_look(g_nav.track_gaze - psi);
    int32_t const step    = (int32_t)servo - (int32_t)g_nav.track_sent;
    uint32_t      dist_mm = NAV_FAR_MM;

    if ((step >= EXTRAS_TRACK_MIN_STEP_DECI) || (step <= -EXTRAS_TRACK_MIN_STEP_DECI))
    {
        obstacle_extras_set_look(servo);
        g_nav.track_sent = servo;
        g_nav.look_deci  = servo;
    }

    if (take_ping(status, &dist_mm))
    {
        track_ping(status, dist_mm, psi);
    }

    if ((PHASE_TRACK_OUT == g_nav.phase) &&
        (track_spin(g_nav.track_away0 - EXTRAS_TRACK_PAN_DECI)))
    {
        set_phase(PHASE_TRACK_BACK);
    }
    else if ((PHASE_TRACK_BACK == g_nav.phase) &&
             (track_spin(g_nav.track_away0)))
    {
        finish_track();
    }
    else
    {
        /* Still rotating, or the scan has just ended on an echo. */
    }
}

/*!
 * @brief TRACK: use one ping. A VALID echo becomes a floor point (beam in
 *        the commanded gaze direction); one inside the swing circle or the
 *        lane ends the scan as blocked at once. UNKNOWN and STUCK pings are
 *        counted; OPEN needs nothing.
 *
 * @param[in] status  Packed Buddy 5 status word (for the echo class).
 * @param[in] dist_mm Reading, mm (NAV_FAR_MM unless VALID).
 * @param[in] psi     Heading since the scan started (deci-degrees).
 */
static void track_ping (uint32_t status, uint32_t dist_mm, int32_t psi)
{
    uint32_t const cls = extras_pack_get(status, EXTRAS_ST_CLASS_SH, EXTRAS_ST_CLASS_MK);

    if (EXTRAS_CLASS_VALID == cls)
    {
        track_echo(dist_mm, psi);
    }
    else if ((EXTRAS_CLASS_UNKNOWN == cls) || (EXTRAS_CLASS_STUCK == cls))
    {
        /* Cast is safe: the count stops at 255. */
        if (g_nav.track_unknown < UINT8_MAX)
        {
            g_nav.track_unknown = (uint8_t)(g_nav.track_unknown + 1U);
        }
    }
    else
    {
        /* OPEN: nothing in range. */
    }
}

/*!
 * @brief TRACK: turn one VALID echo into a floor point (beam in the
 *        commanded gaze direction). A point inside the swing circle or the
 *        lane ends the scan as blocked at once.
 *
 * @param[in] dist_mm Reading, mm.
 * @param[in] psi     Heading since the scan started (deci-degrees).
 */
static void track_echo (uint32_t dist_mm, int32_t psi)
{
    int32_t fwd_mm  = 0;
    int32_t side_mm = 0;

    extras_plan_track_point(psi, g_nav.track_gaze, dist_mm, &fwd_mm, &side_mm);

    if ((extras_plan_track_near(fwd_mm, side_mm)) ||
        (extras_plan_track_blocked(fwd_mm, side_mm, g_nav.track_lookahead_mm)))
    {
        int32_t const gap_mm = extras_plan_track_gap_mm(side_mm);

        g_nav.b_track_blocked = true;

        if (gap_mm > g_nav.track_gap_mm)
        {
            g_nav.track_gap_mm = gap_mm;
        }

        finish_track();
    }
}

/*!
 * @brief TRACK: rotate towards a heading at the constant
 *        EXTRAS_TRACK_SPIN_UM_S (no slow-down step, so the beam drifts evenly
 *        between servo updates).
 *
 * @param[in] target_deci Heading to reach (deci-degrees, positive = away).
 *
 * @return True once the heading is within EXTRAS_NAV_SPIN_TOL_DECI.
 */
static bool track_spin (int32_t target_deci)
{
    int32_t const error  = target_deci - away_deci();
    int32_t const left   = (error < 0) ? -error : error;
    bool const    b_done = (left <= EXTRAS_NAV_SPIN_TOL_DECI);

    if (b_done)
    {
        drive_stop();
    }
    else
    {
        /* Cast is safe: the speed is 70 000 um/s. */
        spin_drive((error > 0) ? INTENT_AWAY : INTENT_TOWARD,
                   (int32_t)EXTRAS_TRACK_SPIN_UM_S);
    }

    return (b_done);
}

/*!
 * @brief TRACK: end the rotation scan and go on.
 *   - Blocked: SPIN_BACK, then MOVE_ON (lane half width - closest blocked
 *     side, at least EXTRAS_TRACK_MIN_MOVE_MM); moving on along the line shifts
 *     the obstacle sideways in the scan frame by the same amount.
 *   - More than EXTRAS_TRACK_MAX_UNKNOWN unknown pings: repeat the scan (the
 *     car is back at its start heading), at most EXTRAS_TRACK_RETRIES times,
 *     then count it as blocked (EXTRAS_TRACK_MIN_MOVE_MM).
 *   - Clear: TO_LINE.
 * Tracking is always switched off first.
 */
static void finish_track (void)
{
    uint32_t move_mm = EXTRAS_TRACK_MIN_MOVE_MM;

    drive_stop();
    obstacle_extras_set_track(false);

    if (g_nav.b_track_blocked)
    {
        int32_t const gap_mm = g_nav.track_gap_mm;

        /* Cast is safe: gap_mm is checked to be above the minimum. */
        if (gap_mm > (int32_t)EXTRAS_TRACK_MIN_MOVE_MM)
        {
            move_mm = (uint32_t)gap_mm;
        }

        log_track(TRACK_BLOCKED, move_mm);
        g_nav.move_mm = move_mm;
        set_look(0);
        set_phase(PHASE_SPIN_BACK);
    }
    else if ((g_nav.track_unknown > EXTRAS_TRACK_MAX_UNKNOWN) &&
             (g_nav.track_retry < EXTRAS_TRACK_RETRIES))
    {
        /* Cast is safe: track_retry stops at EXTRAS_TRACK_RETRIES. */
        g_nav.track_retry = (uint8_t)(g_nav.track_retry + 1U);
        log_track(TRACK_RETRY, 0U);
        set_look(0);
        set_phase(PHASE_TRACK_OUT);
    }
    else if (g_nav.track_unknown > EXTRAS_TRACK_MAX_UNKNOWN)
    {
        log_track(TRACK_UNKNOWN_STOP, move_mm);
        g_nav.move_mm = move_mm;
        set_look(0);
        set_phase(PHASE_SPIN_BACK);
    }
    else
    {
        log_track(TRACK_CLEAR, 0U);
        set_look(0);
        set_phase(PHASE_TO_LINE);
    }
}

/*!
 * @brief Print and log how a rotation scan ended.
 *
 * @param[in] result  TRACK_CLEAR, TRACK_BLOCKED, TRACK_RETRY or
 *                    TRACK_UNKNOWN_STOP.
 * @param[in] move_mm Move-on distance chosen (0 if none).
 */
static void log_track (uint8_t result, uint32_t move_mm)
{
    static char const * const g_what[] = {"clear", "blocked",
                                          "too many unknown pings, retry",
                                          "too many unknown pings, blocked"};
    int32_t const             psi      = heading_deci() - g_nav.track_h0;
    uint8_t const             pass =
        (PHASE_TRACK_OUT == g_nav.phase) ? TRACK_PASS_OUT : TRACK_PASS_BACK;

    LOG_CONT("[b5-nav] track pass %d: %s, into the lane %d mm, %d "
                    "unknown, turned %d deg, move on %d mm\n",
              (INT)pass, g_what[result % (TRACK_UNKNOWN_STOP + 1U)],
              (INT)g_nav.track_gap_mm, (INT)g_nav.track_unknown,
              (INT)(psi / DECI_PER_DEG), (INT)move_mm);

    /* Cast is safe: move_mm is a few hundred mm. */
    log_rec(EXTRAS_LOG_TRACK, pass, result, g_nav.track_unknown, g_nav.track_gap_mm,
            psi, (int32_t)move_mm);
}

/*!
 * @brief Limit a servo angle to +-EXTRAS_SERVO_LIMIT_DECI.
 *
 * @param[in] deci Angle (deci-degrees, positive = left).
 *
 * @return The clamped angle.
 */
static int16_t clamp_look (int32_t deci)
{
    int32_t clamped = deci;

    if (clamped > EXTRAS_SERVO_LIMIT_DECI)
    {
        clamped = EXTRAS_SERVO_LIMIT_DECI;
    }
    else if (clamped < -EXTRAS_SERVO_LIMIT_DECI)
    {
        clamped = -EXTRAS_SERVO_LIMIT_DECI;
    }
    else
    {
        /* In range. */
    }

    /* Cast is safe: clamped is within +-600. */
    return ((int16_t)clamped);
}

/*!
 * @brief Set up one pass of the rotation scan.
 *   - TRACK_OUT (also on a retry): note the start heading, the lookahead
 *     (to EXTRAS_LANE_EXTRA_MM past the line, the same lane as judge_sweep()),
 *     gaze straight ahead, clear the counts, and turn tracking on.
 *   - TRACK_BACK: tilt the gaze EXTRAS_TRACK_TILT_DECI towards the obstacle's
 *     side and move the servo there with the normal settle wait (the
 *     wheels are stopped at the turnaround).
 * Pings from before the pass are not used.
 *
 * @param[in] phase PHASE_TRACK_OUT or PHASE_TRACK_BACK.
 */
static void start_track_pass (uint8_t phase)
{
    if (PHASE_TRACK_OUT == phase)
    {
        int32_t const side_mm = lateral_mm();

        g_nav.track_h0    = heading_deci();
        g_nav.track_away0 = away_deci();

        /* Cast is safe: side_mm is non-negative here and a few hundred mm. */
        g_nav.track_lookahead_mm =
            ((side_mm > 0) ? (uint32_t)side_mm : 0U) + EXTRAS_LANE_EXTRA_MM;
        g_nav.track_gaze      = 0;
        g_nav.track_sent      = g_nav.look_deci;
        g_nav.track_unknown   = 0U;
        g_nav.b_track_blocked = false;
        g_nav.track_gap_mm    = 0;
        obstacle_extras_set_track(true);
    }
    else
    {
        int32_t const psi = heading_deci() - g_nav.track_h0;

        /* Towards the obstacle's side is -side_sign() in heading terms. */
        g_nav.track_gaze = -side_sign() * EXTRAS_TRACK_TILT_DECI;
        g_nav.track_sent = clamp_look(g_nav.track_gaze - psi);
        set_look(g_nav.track_sent);
    }

    sync_ping();
}

/*!
 * @brief Mark the latest ping as already seen, so take_ping() only
 *        reports pings that arrive from now on.
 */
static void sync_ping (void)
{
    /* Cast is safe: the sequence field is 6 bits. */
    g_nav.ping_seq =
        (uint8_t)extras_pack_get(g_nav.status, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK);
}

/*!
 * @brief TO_LINE: drive straight at the line (fast until EXTRAS_LINE_SLOW_MM
 *        short of it, then slowly). Once both IR sensors
 *        have been off the line (IR gate), black counts as the line only
 *        after EXTRAS_LINE_MIN_MM of travel (floor-mat gaps are thinner) and
 *        with the sensor no more than EXTRAS_LINE_WINDOW_MM short of the
 *        original line by odometry. A car whose sensor gets
 *        EXTRAS_LINE_GIVE_UP_MM past the line without finding it stops.
 *
 * @param[in] b_line_seen An IR sensor sees the line.
 */
static void run_to_line (bool b_line_seen)
{
    int32_t const ir_side_mm =
        extras_plan_point_side_mm(lateral_mm(), away_deci(), EXTRAS_AXLE_TO_IR_MM);
    bool const    b_inside = (ir_side_mm <= (int32_t)EXTRAS_LINE_WINDOW_MM);
    int32_t const black_mm = travel_mm() - g_nav.black_mm;

    if (!g_nav.b_ir_cleared)
    {
        if (!b_line_seen)
        {
            g_nav.b_ir_cleared = true;
            LOG_CONT("[b5-nav] IR clear of the line\n");
        }
    }
    else if (!b_line_seen)
    {
        if (g_nav.b_black && (!g_nav.b_line_noted))
        {
            note_line(LINE_SHORT, ir_side_mm, black_mm);
        }

        g_nav.b_black      = false;
        g_nav.b_line_noted = false;
    }
    else if (!g_nav.b_black)
    {
        g_nav.b_black  = true;
        g_nav.black_mm = travel_mm();
    }
    else if ((black_mm >= (int32_t)EXTRAS_LINE_MIN_MM) && b_inside)
    {
        note_line(LINE_ACCEPTED, ir_side_mm, black_mm);
        set_phase(PHASE_CENTRE);
    }
    else if ((black_mm >= (int32_t)EXTRAS_LINE_MIN_MM) && (!g_nav.b_line_noted))
    {
        note_line(LINE_OUTSIDE, ir_side_mm, black_mm);
        g_nav.b_line_noted = true;
    }
    else
    {
        /* Black, but not for long enough yet. */
    }

    if (PHASE_TO_LINE == g_nav.phase)
    {
        if (ir_side_mm < -(int32_t)EXTRAS_LINE_GIVE_UP_MM)
        {
            halt_bypass(HALT_MISSED);
        }
        else if (ir_side_mm > (int32_t)EXTRAS_LINE_SLOW_MM)
        {
            /* Still well short of the line: full speed. */
            drive_hold(-DECI_RIGHT_ANGLE, (int32_t)EXTRAS_NAV_SPEED_UM_S);
        }
        else
        {
            /* Cast is safe: the speed is 70 000 um/s. */
            drive_hold(-DECI_RIGHT_ANGLE, (int32_t)EXTRAS_NAV_LINE_UM_S);
        }
    }
}

/*!
 * @brief Print and log what an IR line contact was taken for.
 *
 * @param[in] event      LINE_ACCEPTED, LINE_OUTSIDE or LINE_SHORT.
 * @param[in] ir_side_mm Side position of the IR sensors, mm.
 * @param[in] black_mm   Travel while the sensor saw black, mm.
 */
static void note_line (uint8_t event, int32_t ir_side_mm, int32_t black_mm)
{
    static char const * const g_what[LINE_EVENTS] = {
        "found", "ignored (outside the window)", "ignored (too short)"};

    LOG_CONT("[b5-nav] line %s: IR at %d mm from the line, black for "
                    "%d mm\n",
              g_what[event % LINE_EVENTS], (INT)ir_side_mm, (INT)black_mm);
    log_rec(EXTRAS_LOG_LINE, event, 0U, 0U, ir_side_mm, black_mm, ahead_mm());
}

/*!
 * @brief Kernel time in milliseconds.
 *
 * @return Low 32 bits of the system time.
 */
static uint32_t now_ms (void)
{
    SYSTIM tim = {0, 0U};

    (void)tk_get_otm(&tim);

    return (tim.lo);
}

/*!
 * @brief Limit a value to the int16_t range for a log field.
 *
 * @param[in] value Value to log.
 *
 * @return value, clamped to -32767..32767.
 */
static int16_t clamp_i16 (int32_t value)
{
    int32_t clamped = value;

    if (clamped > LOG_I16_MAX)
    {
        clamped = LOG_I16_MAX;
    }
    else if (clamped < -LOG_I16_MAX)
    {
        clamped = -LOG_I16_MAX;
    }
    else
    {
        /* In range. */
    }

    /* Cast is safe: clamped is inside the int16_t range. */
    return ((int16_t)clamped);
}

/*!
 * @brief Add one record to the flight recorder.
 *
 * @param[in] kind  Record kind (EXTRAS_LOG_*).
 * @param[in] val_a Field a.
 * @param[in] val_b Field b.
 * @param[in] val_c Field c.
 * @param[in] val_d Field d (clamped to 16 bits).
 * @param[in] val_e Field e (clamped to 16 bits).
 * @param[in] val_f Field f.
 */
static void log_rec (uint8_t kind, uint8_t val_a, uint8_t val_b, uint8_t val_c,
                     int32_t val_d, int32_t val_e, int32_t val_f)
{
    extras_log_rec_t rec;

    rec.t_ms  = 0U;
    rec.kind  = kind;
    rec.val_a = val_a;
    rec.val_b = val_b;
    rec.val_c = val_c;
    rec.val_d = clamp_i16(val_d);
    rec.val_e = clamp_i16(val_e);
    rec.val_f = val_f;
    extras_log_add(&rec);
}

/*!
 * @brief Post one command to Buddy 5, print it and log it.
 *
 * @param[in] type   Command type (EXTRAS_CMD_*).
 * @param[in] req_id Request id.
 */
static void post (uint32_t type, uint8_t req_id)
{
    ER const result = obstacle_extras_post_cmd(extras_pack_cmd(type, req_id));

    LOG_CONT("[b5-nav] cmd %d id=%d -> %d\n", (INT)type, (INT)req_id,
              (INT)result);

    /* Cast is safe: the command type is 3 bits wide. */
    log_rec(EXTRAS_LOG_CMD, (uint8_t)type, req_id, 0U, (int32_t)result, 0, 0);
}

/*!
 * @brief Set or release the obstacle hold. The motion service ignores a
 *        call that does not change the hold, so this runs every pass. A
 *        change restarts the bumper's settling time.
 *
 * @param[in] b_held True to hold the car.
 */
static void hold_car (bool b_held)
{
    if (b_held != g_nav.b_held)
    {
        g_nav.b_held = b_held;
        g_nav.cmd_ms = now_ms();
    }

    (void)motion_service_set_hold(MOTION_HOLD_OBSTACLE, b_held);
}

/*!
 * @brief Heading measured towards the side being passed.
 *
 * @return Deci-degrees, positive = away from the obstacle.
 */
static int32_t away_deci (void)
{
    return (side_sign() * heading_deci());
}

/*!
 * @brief Average travel of the two wheels since boot (forwards positive),
 *        used to measure the straight legs.
 *
 * @return Distance in mm.
 */
static int32_t travel_mm (void)
{
    return ((g_nav.wheel_mm[WHEEL_LEFT] + g_nav.wheel_mm[WHEEL_RIGHT]) /
            (int32_t)WHEELS);
}

/*!
 * @brief Start the dead reckoning when BYPASS begins.
 */
static void reset_pose (void)
{
    uint32_t wheel = 0U;

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        g_nav.base_um[wheel]  = g_nav.wheel_um[wheel];
        g_nav.prev_um[wheel]  = g_nav.wheel_um[wheel];
        g_nav.last_cmd[wheel] = 0;
        g_nav.b_low[wheel]    = false;
    }

    g_nav.x_um      = 0;
    g_nav.y_um      = 0;
    g_nav.b_pausing = false;
}

/*!
 * @brief Turn a steering decision into wheel speeds.
 *
 * STRAIGHT drives both wheels at EXTRAS_NAV_SPEED_UM_S; LEFT and RIGHT run the
 * inside wheel at inside_um_s and the outside wheel at outside_um_s; HOLD
 * stands still.
 *
 * @param[in] steer        Steering (EXTRAS_STEER_*).
 * @param[in] inside_um_s  Inside wheel speed (0 = pivot on the inside
 *                          wheel).
 * @param[in] outside_um_s Outside wheel speed (up to 200 000 um/s).
 */
static void drive_steer (uint32_t steer, int32_t inside_um_s,
                         int32_t outside_um_s)
{
    switch (steer)
    {
        case EXTRAS_STEER_STRAIGHT:
            /* Cast is safe: the speed is 140 000 um/s. */
            drive_speeds(steer, (int32_t)EXTRAS_NAV_SPEED_UM_S,
                         (int32_t)EXTRAS_NAV_SPEED_UM_S);
        break;

        case EXTRAS_STEER_LEFT:
            drive_speeds(steer, inside_um_s, outside_um_s);
        break;

        case EXTRAS_STEER_RIGHT:
            drive_speeds(steer, outside_um_s, inside_um_s);
        break;

        default:
            drive_speeds(EXTRAS_STEER_HOLD, 0, 0);
        break;
    }
}

/*!
 * @brief Send a speed command, and print and log it whenever the speeds
 *        change. A wheel that speeds up restarts the bumper's settling
 *        time.
 *
 * @param[in] steer      Steering it stands for (EXTRAS_STEER_*), for the log.
 * @param[in] left_um_s  Left wheel speed (-100 000..200 000 um/s).
 * @param[in] right_um_s Right wheel speed (-100 000..200 000 um/s).
 */
static void drive_speeds (uint32_t steer, int32_t left_um_s, int32_t right_um_s)
{
    int32_t  speed[WHEELS] = {left_um_s, right_um_s};
    int32_t  before[WHEELS];
    uint32_t wheel = 0U;

    before[WHEEL_LEFT]  = g_nav.last_cmd[WHEEL_LEFT];
    before[WHEEL_RIGHT] = g_nav.last_cmd[WHEEL_RIGHT];

    if (is_reverse_pause(speed))
    {
        speed[WHEEL_LEFT]  = 0;
        speed[WHEEL_RIGHT] = 0;
    }

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        int32_t const now_abs =
            (speed[wheel] < 0) ? -speed[wheel] : speed[wheel];
        int32_t const was_abs =
            (before[wheel] < 0) ? -before[wheel] : before[wheel];

        if (now_abs > (was_abs + (int32_t)EXTRAS_NAV_TRIM_UM_S))
        {
            g_nav.cmd_ms = now_ms();
        }
    }

    {
        int32_t const left_mm_s  = speed[WHEEL_LEFT] / UM_PER_MM;
        int32_t const right_mm_s = speed[WHEEL_RIGHT] / UM_PER_MM;

        /* Casts are safe: both speeds are -100..200 mm/s, so the offset
         * values are 28..328, inside the 10-bit fields. */
        uint32_t const key =
            steer |
            ((uint32_t)(left_mm_s + DRIVE_OFFSET_MM_S) << DRIVE_LEFT_SH) |
            ((uint32_t)(right_mm_s + DRIVE_OFFSET_MM_S) << DRIVE_RIGHT_SH);

        if (g_nav.last_drive != key)
        {
            g_nav.last_drive = key;
            LOG_CONT("[b5-nav] steer %d: left %d right %d mm/s\n",
                      (INT)steer, (INT)left_mm_s, (INT)right_mm_s);

            /* Cast is safe: the steer field is 2 bits wide. */
            log_rec(EXTRAS_LOG_STEER, (uint8_t)steer, 0U, 0U, left_mm_s, right_mm_s,
                    heading_deci());
        }
    }

    (void)motion_set_obstacle_speed(speed[WHEEL_LEFT], speed[WHEEL_RIGHT], EXTRAS_NAV_TTL_US);
}

/*!
 * @brief Stand still (both wheels at 0, not the hold).
 */
static void drive_stop (void)
{
    drive_speeds(EXTRAS_STEER_HOLD, 0, 0);
}

/*!
 * @brief Drive straight on, holding a heading: more than
 *        EXTRAS_NAV_HOLD_DEAD_DECI off, the wheel on the side the car drifts
 *        towards runs EXTRAS_NAV_TRIM_UM_S slower.
 *
 * @param[in] target_deci Heading to hold (deci-degrees, positive = away).
 * @param[in] speed_um_s  Speed of the faster wheel (um/s).
 */
static void drive_hold (int32_t target_deci, int32_t speed_um_s)
{
    int32_t const error  = away_deci() - target_deci;
    uint32_t      intent = INTENT_STRAIGHT;

    if (error > EXTRAS_NAV_HOLD_DEAD_DECI)
    {
        intent = INTENT_TOWARD;
    }
    else if (error < -EXTRAS_NAV_HOLD_DEAD_DECI)
    {
        intent = INTENT_AWAY;
    }
    else
    {
        intent = INTENT_STRAIGHT;
    }

    if (INTENT_STRAIGHT == intent)
    {
        drive_speeds(EXTRAS_STEER_STRAIGHT, speed_um_s, speed_um_s);
    }
    else
    {
        /* Cast is safe: the trim is 14 000 um/s. */
        drive_steer(intent_steer(intent),
                    speed_um_s - (int32_t)EXTRAS_NAV_TRIM_UM_S, speed_um_s);
    }
}

/*!
 * @brief Rotate on the middle of the axle: both wheels at speed_um_s in
 *        opposite directions.
 *
 * @param[in] intent     INTENT_AWAY or INTENT_TOWARD (direction).
 * @param[in] speed_um_s Wheel speed (um/s).
 */
static void spin_drive (uint32_t intent, int32_t speed_um_s)
{
    uint32_t const steer = intent_steer(intent);

    if (EXTRAS_STEER_LEFT == steer)
    {
        drive_speeds(steer, -speed_um_s, speed_um_s);
    }
    else
    {
        drive_speeds(steer, speed_um_s, -speed_um_s);
    }
}

/*!
 * @brief Spin on the middle of the axle towards a heading: both wheels at
 *        the same speed in opposite directions, slowing near it.
 *
 * NOTE: If the spin overshoots, both wheels change direction to correct
 * it; drive_speeds() then stops them for EXTRAS_NAV_REVERSE_PAUSE_MS first.
 *
 * @param[in] target_deci Heading to reach (deci-degrees, positive = away).
 *
 * @return True once the heading is within EXTRAS_NAV_SPIN_TOL_DECI.
 */
static bool spin_to (int32_t target_deci)
{
    int32_t const error  = target_deci - away_deci();
    int32_t const left   = (error < 0) ? -error : error;
    bool const    b_done = (left <= EXTRAS_NAV_SPIN_TOL_DECI);

    if (b_done)
    {
        drive_stop();
        LOG_CONT("[b5-nav] spun to %d deg (target %d)\n",
                  (INT)(away_deci() / DECI_PER_DEG),
                  (INT)(target_deci / DECI_PER_DEG));
        log_rec(EXTRAS_LOG_TURNED, 0U, 0U, 0U, away_deci(), target_deci, 0);
    }
    else
    {
        /* Casts are safe: the speeds are 46 666 and 140 000 um/s. */
        int32_t const speed_um_s = (left <= EXTRAS_NAV_TURN_SLOW_DECI)
                                       ? (int32_t)EXTRAS_NAV_SPIN_SLOW_UM_S
                                       : (int32_t)EXTRAS_NAV_SPIN_UM_S;
        spin_drive((error > 0) ? INTENT_AWAY : INTENT_TOWARD, speed_um_s);
    }

    return (b_done);
}

/*!
 * @brief Move to a new bypass phase, set it up, and print and log where
 *        the car is.
 *
 * @param[in] phase PHASE_*.
 */
static void set_phase (uint8_t phase)
{
    uint32_t slot = 0U;

    g_nav.phase    = phase;
    g_nav.phase_mm = travel_mm();

    /* The collision guard is off only while the car stands still for a
     * side scan; every other phase (HALT included) turns it back on. */
    obstacle_extras_set_sweep(PHASE_SWEEP == phase);

    if (PHASE_SWEEP == phase)
    {
        for (slot = 0U; slot < EXTRAS_SLOT_COUNT; slot++)
        {
            g_nav.sweep_mm[slot] = EXTRAS_PLAN_NO_ECHO;
        }

        g_nav.sweep_slot  = SWEEP_FIRST_SLOT;
        g_nav.sweep_pings = 0U;
        g_nav.track_retry = 0U;

        /* Cast is safe: a slot angle is within +-600. */
        set_look((int16_t)extras_scan_slot_deci(SWEEP_FIRST_SLOT));
    }
    else if ((PHASE_TRACK_OUT == phase) || (PHASE_TRACK_BACK == phase))
    {
        start_track_pass(phase);
    }
    else if (PHASE_MAP == phase)
    {
        g_nav.map_start    = away_deci();
        g_nav.b_ir_cleared = false;
    }
    else if (PHASE_TO_LINE == phase)
    {
        g_nav.b_ir_cleared = false;
        g_nav.b_black      = false;
        g_nav.b_line_noted = false;
    }
    else
    {
        /* Nothing to set up. */
    }

    LOG_CONT("[b5-nav] phase %d: side %d mm, ahead %d mm, heading %d "
                    "deg\n",
              (INT)phase, (INT)lateral_mm(), (INT)ahead_mm(),
              (INT)(away_deci() / DECI_PER_DEG));
    log_rec(EXTRAS_LOG_PHASE, phase, 0U, 0U, lateral_mm(), ahead_mm(), away_deci());
}

/*!
 * @brief BYPASS has begun: start the dead reckoning, point the servo
 *        ahead, and drive the first leg (AHEAD).
 */
static void begin_bypass (void)
{
    reset_pose();
    g_nav.scans = 0U;
    obstacle_extras_set_track(false);
    set_look(0);
    set_phase(PHASE_AHEAD);
}

/*!
 * @brief Give up on the bypass: stop the car, stop Buddy 5, and do not
 *        retry until the car is reset.
 *
 * @param[in] reason HALT_*.
 */
static void halt_bypass (uint8_t reason)
{
    static char const * const g_why[] = {"bumped into something",
                                         "the line was not found", "collision",
                                         "no way past the obstacle"};

    drive_stop();
    hold_car(true);
    obstacle_extras_set_track(false);
    set_phase(PHASE_HALT);
    g_nav.b_no_retry = true;
    LOG_CONT("[b5-nav] HALT: %s. Reset the car to go on.\n",
              g_why[reason % (HALT_NO_PATH + 1U)]);
    log_rec(EXTRAS_LOG_HALT, reason, 0U, 0U, lateral_mm(), ahead_mm(), away_deci());
    post(EXTRAS_CMD_STOP, g_nav.req_id);
}

/*!
 * @brief Take the newest ping from the status word, if it is new.
 *
 * @param[in]  status    Packed Buddy 5 status word.
 * @param[out] p_dist_mm Distance in mm; NAV_FAR_MM for OPEN, UNKNOWN or
 *                       STUCK.
 *
 * @return True if a ping arrived since the last call.
 */
static bool take_ping (uint32_t status, uint32_t * p_dist_mm)
{
    uint32_t const cls = extras_pack_get(status, EXTRAS_ST_CLASS_SH, EXTRAS_ST_CLASS_MK);
    uint32_t const dist_mm = extras_pack_get(status, EXTRAS_ST_DIST_SH, EXTRAS_ST_DIST_MK);
    uint32_t const seq     = extras_pack_get(status, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK);
    bool const     b_new   = (seq != g_nav.ping_seq);

    /* Cast is safe: the sequence field is 6 bits. */
    g_nav.ping_seq = (uint8_t)seq;
    *p_dist_mm     = (EXTRAS_CLASS_VALID == cls) ? dist_mm : NAV_FAR_MM;

    return (b_new);
}

/*!
 * @brief Tell whether the ping in this status word was taken at the
 *        current look angle (ping tag, obstacle_extras_get_ping_tag()). A tag
 *        that belongs to a newer ping does not count.
 *
 * @param[in] status Packed Buddy 5 status word.
 *
 * @return True if the ping was taken at g_nav.look_deci.
 */
static bool is_ping_at_look (uint32_t status)
{
    uint32_t const tag = obstacle_extras_get_ping_tag();
    uint32_t const seq = extras_pack_get(status, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK);

    /* Cast is safe: the angle field is 11 bits. */
    int32_t const look =
        (int32_t)extras_pack_get(tag, EXTRAS_TAG_LOOK_SH, EXTRAS_TAG_LOOK_MK) -
        EXTRAS_TAG_LOOK_BIAS;

    return ((extras_pack_get(tag, EXTRAS_TAG_SEQ_SH, EXTRAS_TAG_SEQ_MK) == seq) &&
            (look == (int32_t)g_nav.look_deci));
}

/*!
 * @brief Ask Buddy 5 to point the servo somewhere else (REACQUIRE); the
 *        car then stands still until it has settled.
 *
 * @param[in] deci Servo angle (deci-degrees, positive = left).
 */
static void set_look (int16_t deci)
{
    if (deci != g_nav.look_deci)
    {
        g_nav.look_deci = deci;
        obstacle_extras_set_look(deci);
        start_servo_wait();
    }
}

/*!
 * @brief The servo is about to move: the car stands still until
 *        EXTRAS_NAV_SERVO_PINGS new pings have arrived (Buddy 5 pings only
 *        after the servo has settled) or EXTRAS_NAV_SERVO_WAIT_MS has passed.
 */
static void start_servo_wait (void)
{
    /* Cast is safe: the sequence field is 6 bits. */
    g_nav.b_servo_wait = true;
    g_nav.wait_seq =
        (uint8_t)extras_pack_get(g_nav.status, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK);
    g_nav.wait_ms = now_ms();
}

/*!
 * @brief Virtual bumper: when the nominal command reaches EXTRAS_BUMP_MIN_UM_S,
 *        compare the measured wheel speed to its scaled target. Report a bump
 *        after EXTRAS_BUMP_MS below EXTRAS_BUMP_PERCENT of that target. Wait for
 *        EXTRAS_BUMP_SETTLE_MS after the last release or speed increase.
 *
 * @param[in] now Current time, ms.
 *
 * @return True on a bump.
 */
static bool check_bump (uint32_t now)
{
    bool const b_settled =
        (!g_nav.b_held) && ((now - g_nav.cmd_ms) >= EXTRAS_BUMP_SETTLE_MS);
    bool     b_bump = false;
    uint32_t wheel  = 0U;

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        int32_t const cmd     = g_nav.last_cmd[wheel] / MOTION_SPEED_DIVISOR;
        int32_t const vel     = g_nav.vel_um_s[wheel];
        int32_t const cmd_abs = (cmd < 0) ? -cmd : cmd;
        int32_t const vel_abs = (vel < 0) ? -vel : vel;
        bool const    b_slow =
            b_settled && g_nav.b_vel_ok[wheel] &&
            (cmd_abs >= (EXTRAS_BUMP_MIN_UM_S / MOTION_SPEED_DIVISOR)) &&
            ((vel_abs * PERCENT_FULL) < (cmd_abs * EXTRAS_BUMP_PERCENT));

        if (!b_slow)
        {
            g_nav.b_low[wheel] = false;
        }
        else if (!g_nav.b_low[wheel])
        {
            g_nav.b_low[wheel]  = true;
            g_nav.low_ms[wheel] = now;
        }
        else if ((!b_bump) && ((now - g_nav.low_ms[wheel]) >= EXTRAS_BUMP_MS))
        {
            b_bump = true;

            /* Cast is safe: wheel is 0 or 1. */
            log_rec(EXTRAS_LOG_BUMP, (uint8_t)wheel, g_nav.phase, 0U,
                    cmd / UM_PER_MM, vel / UM_PER_MM, ahead_mm());
        }
        else
        {
            /* Slow, but not for long enough yet. */
        }
    }

    return (b_bump);
}

/*!
 * @brief Sign that turns "away from the obstacle" into left / right.
 *
 * @return +1 when passing on the left, -1 when passing on the right.
 */
static int32_t side_sign (void)
{
    return ((EXTRAS_SIDE_LEFT == g_nav.side) ? 1 : -1);
}

/*!
 * @brief Heading since BYPASS began, from the total wheel difference:
 *        (right - left) / calibrated track, positive = left.
 *
 * @return Heading in deci-degrees.
 */
static int32_t heading_deci (void)
{
    int64_t const left_um =
        g_nav.wheel_um[WHEEL_LEFT] - g_nav.base_um[WHEEL_LEFT];
    int64_t const right_um =
        g_nav.wheel_um[WHEEL_RIGHT] - g_nav.base_um[WHEEL_RIGHT];
    int64_t const track_um = (int64_t)EXTRAS_NAV_TRACK_UM;

    /* Cast is safe: a bypass turns the car far less than 2^31 deci-deg. */
    return ((int32_t)(((right_um - left_um) * EXTRAS_NAV_DECI_PER_RAD) / track_um));
}

/*!
 * @brief Sideways distance from the original line, towards the side being
 *        passed (so it grows as the car moves away from the obstacle).
 *
 * @return Distance in mm.
 */
static int32_t lateral_mm (void)
{
    /* Cast is safe: a bypass moves the car far less than 2 km. */
    return (side_sign() * (int32_t)(g_nav.x_um / UM_PER_MM));
}

/*!
 * @brief Distance travelled along the original line.
 *
 * @return Distance in mm.
 */
static int32_t ahead_mm (void)
{
    /* Cast is safe: a bypass moves the car far less than 2 km. */
    return ((int32_t)(g_nav.y_um / UM_PER_MM));
}

/*!
 * @brief Move the position on by the wheel travel since the last pass,
 *        along the current heading.
 */
static void update_pose (void)
{
    int64_t const left_um =
        g_nav.wheel_um[WHEEL_LEFT] - g_nav.prev_um[WHEEL_LEFT];
    int64_t const right_um =
        g_nav.wheel_um[WHEEL_RIGHT] - g_nav.prev_um[WHEEL_RIGHT];
    int64_t const step_um = (left_um + right_um) / (int64_t)WHEELS;
    int32_t const heading = heading_deci();

    /* x grows to the left: sin of a left (positive) heading. */
    g_nav.x_um += (step_um * extras_plan_sin_q14(heading)) / Q14_ONE;
    g_nav.y_um += (step_um * extras_plan_cos_q14(heading)) / Q14_ONE;
    g_nav.prev_um[WHEEL_LEFT]  = g_nav.wheel_um[WHEEL_LEFT];
    g_nav.prev_um[WHEEL_RIGHT] = g_nav.wheel_um[WHEEL_RIGHT];
}

/*!
 * @brief Turn a steering intent into LEFT / RIGHT for the side passed.
 *
 * @param[in] intent INTENT_STRAIGHT, INTENT_AWAY or INTENT_TOWARD.
 *
 * @return EXTRAS_STEER_*.
 */
static uint32_t intent_steer (uint32_t intent)
{
    bool const b_left = (EXTRAS_SIDE_LEFT == g_nav.side);
    uint32_t   steer  = EXTRAS_STEER_STRAIGHT;

    if (INTENT_AWAY == intent)
    {
        steer = b_left ? EXTRAS_STEER_LEFT : EXTRAS_STEER_RIGHT;
    }
    else if (INTENT_TOWARD == intent)
    {
        steer = b_left ? EXTRAS_STEER_RIGHT : EXTRAS_STEER_LEFT;
    }
    else
    {
        steer = EXTRAS_STEER_STRAIGHT;
    }

    return (steer);
}

/*!
 * @brief Stop both wheels for EXTRAS_NAV_REVERSE_PAUSE_MS before any wheel
 *        changes direction.
 *
 * NOTE: On the car, braking or reversing a wheel while the motion service
 * is driving made the geared motor rattle; the encoder reported invalid
 * steps and the motion service latched its FEEDBACK fault (reboot only).
 * With both wheels at 0 the motion service is idle and does not check the
 * encoders, so the wheel can settle first.
 *
 * @param[in] p_speed Wheel speeds about to be commanded (um/s).
 *
 * @return True while the car must stand still instead.
 */
static bool is_reverse_pause (int32_t const p_speed[WHEELS])
{
    uint32_t const now     = now_ms();
    bool           b_pause = false;
    uint32_t       wheel   = 0U;

    if (g_nav.b_pausing && ((now - g_nav.pause_ms) < EXTRAS_NAV_REVERSE_PAUSE_MS))
    {
        b_pause = true;
    }
    else
    {
        g_nav.b_pausing = false;

        for (wheel = 0U; wheel < WHEELS; wheel++)
        {
            b_pause = b_pause ||
                      ((p_speed[wheel] > 0) && (g_nav.last_cmd[wheel] < 0)) ||
                      ((p_speed[wheel] < 0) && (g_nav.last_cmd[wheel] > 0));
        }
    }

    if (b_pause && (!g_nav.b_pausing))
    {
        g_nav.b_pausing = true;
        g_nav.pause_ms  = now;
        LOG_CONT("[b5-nav] wheel reverses: stop for %d ms first\n",
                  (INT)EXTRAS_NAV_REVERSE_PAUSE_MS);
    }

    if (!b_pause)
    {
        g_nav.last_cmd[WHEEL_LEFT]  = p_speed[WHEEL_LEFT];
        g_nav.last_cmd[WHEEL_RIGHT] = p_speed[WHEEL_RIGHT];
    }
    else
    {
        g_nav.last_cmd[WHEEL_LEFT]  = 0;
        g_nav.last_cmd[WHEEL_RIGHT] = 0;
    }

    return (b_pause);
}

/*!
 * @brief Tell whether Buddy 5's latest reading is an obstacle within the
 *        detection distance, so the line follower must not drive on.
 *
 * @return True for a recent centered hit at or below EXTRAS_DETECT_MM.
 */
static bool is_obstacle_close (void)
{
    uint32_t const ping = obstacle_ping_recent();
    uint16_t const mm = obstacle_pack_ping_mm(ping);

    return 0 != ping && 0 == obstacle_pack_ping_servo(ping) &&
        OBSTACLE_CLASS_VALID == obstacle_pack_ping_echo(ping) &&
        0 < mm && mm <= EXTRAS_DETECT_MM;
}

/*!
 * @brief Print and log each Buddy 5 state change. Entering BYPASS also
 *        starts the bypass.
 *
 * NOTE: States: 0 STOPPED, 1 MONITOR, 2 WAIT_STOP, 3 SCANNING, 4 PLAN,
 * 5 BYPASS, 6 REACQUIRE.
 *
 * @param[in] status Packed Buddy 5 status word.
 */
static void report_state (uint32_t status)
{
    uint32_t const state = extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK);

    if (g_nav.last_state != state)
    {
        uint32_t const cause =
            extras_pack_get(status, EXTRAS_ST_CAUSE_SH, EXTRAS_ST_CAUSE_MK);

        /* Casts are safe: the state field is 3 bits, the cause 4 bits. */
        g_nav.last_state = (uint8_t)state;
        g_nav.last_drive = NAV_DRIVE_UNSEEN;
        g_nav.status     = status;
        LOG_CONT("[b5-nav] state %d cause %d\n", (INT)state, (INT)cause);
        log_rec(EXTRAS_LOG_STATE, (uint8_t)state, (uint8_t)cause,
                obstacle_extras_get_faults(), 0, 0, 0);

        if (EXTRAS_STATE_REACQUIRE != state)
        {
            /* Servo tracking and the sweep belong to REACQUIRE only. */
            obstacle_extras_set_track(false);
            obstacle_extras_set_sweep(false);
        }

        if (EXTRAS_STATE_BYPASS == state)
        {
            begin_bypass();
        }

        if ((EXTRAS_STATE_BYPASS == state) ||
            ((EXTRAS_STATE_MONITOR == state) && (NAV_UNSEEN != g_nav.req_id)) ||
            ((EXTRAS_STATE_REACQUIRE == state) && (0 != g_nav.look_deci)))
        {
            /* The servo turns on entering these states (back to 0 after
             * the scan or after the bypass, or to the angle Navigation
             * asked for if that is not straight ahead): wait until it has
             * settled. */
            start_servo_wait();
        }
    }
}

/*!
 * @brief STOPPED: hold the car and retry RESET + START every
 *        EXTRAS_NAV_RETRY_MS, except after a collision or a bypass Navigation
 *        gave up on (bump, blocked, line missed): retrying would send the
 *        car back to line following, straight into the obstacle. Those
 *        stay stopped until the car is reset, and so does a car that got
 *        NO_SAFE_PATH EXTRAS_NO_PATH_TRIES times in a row (the same scan would
 *        only repeat).
 *
 * @param[in] status Packed Buddy 5 status word.
 * @param[in] now    Current time, ms.
 */
static bool step_stopped (uint32_t status, uint32_t now)
{
    uint32_t const cause = extras_pack_get(status, EXTRAS_ST_CAUSE_SH, EXTRAS_ST_CAUSE_MK);
    bool const b_sensor = (EXTRAS_CAUSE_NO_ECHO == cause) || (EXTRAS_CAUSE_ECHO_STUCK == cause);
    bool const b_hold = !b_sensor || (NAV_UNSEEN != g_nav.req_id);

    hold_car(b_hold);

    if ((EXTRAS_CAUSE_COLLISION == cause) && (!g_nav.b_no_retry))
    {
        g_nav.b_no_retry = true;
        g_nav.phase      = PHASE_HALT;
        LOG_CONT("[b5-nav] HALT: collision. Reset the car to go on.\n");
        log_rec(EXTRAS_LOG_HALT, HALT_COLLISION, 0U, 0U, lateral_mm(), ahead_mm(),
                away_deci());
    }

    if ((EXTRAS_CAUSE_NO_SAFE_PATH == cause) &&
        (g_nav.no_path_count >= EXTRAS_NO_PATH_TRIES) && (!g_nav.b_no_retry))
    {
        g_nav.b_no_retry = true;
        g_nav.phase      = PHASE_HALT;
        LOG_CONT("[b5-nav] HALT: no way past the obstacle (NO_SAFE_PATH "
                        "%d times in a row). Reset the car to go on.\n",
                  (INT)g_nav.no_path_count);
        log_rec(EXTRAS_LOG_HALT, HALT_NO_PATH, 0U, 0U, lateral_mm(), ahead_mm(),
                away_deci());
    }

    if ((!g_nav.b_no_retry) && (EXTRAS_CAUSE_EXTERNAL != cause) &&
        (EXTRAS_CAUSE_BOOT != cause) && ((now - g_nav.retry_ms) >= EXTRAS_NAV_RETRY_MS))
    {
        g_nav.retry_ms = now;
        post(EXTRAS_CMD_RESET, 0U);
        post(EXTRAS_CMD_START, 0U);
    }
    return b_hold;
}

/*!
 * @brief Tell whether the servo may still be turning, so the wheels must
 *        stand still.
 *
 * NOTE: A servo swing while a motor drives disturbs the wheel encoders
 * (seen on the car: false backward pulses and an invalid step on a wheel
 * told to stand still, which latches the motion service's FEEDBACK fault).
 * With both wheels at 0 the motion service is idle and does not check the
 * encoders. The wait ends at the first new ping taken at the new angle
 * (is_ping_at_look(); the scan task pings only after the servo has
 * settled), once EXTRAS_NAV_SERVO_PINGS new pings have arrived (the first may
 * have been taken just before it moved; covers states where the
 * statechart picks the angle), or after EXTRAS_NAV_SERVO_WAIT_MS.
 *
 * @param[in] status Packed Buddy 5 status word.
 * @param[in] now    Current time, ms.
 *
 * @return True while the car must stand still for the servo.
 */
static bool is_servo_moving (uint32_t status, uint32_t now)
{
    uint32_t const seq = extras_pack_get(status, EXTRAS_ST_SEQ_SH, EXTRAS_ST_SEQ_MK);
    uint32_t const pings =
        (seq - (uint32_t)g_nav.wait_seq) & (uint32_t)EXTRAS_ST_SEQ_MK;

    bool const b_at_look = (pings > 0U) && (is_ping_at_look(status));

    if (g_nav.b_servo_wait && (b_at_look || (pings >= EXTRAS_NAV_SERVO_PINGS) ||
                               ((now - g_nav.wait_ms) >= EXTRAS_NAV_SERVO_WAIT_MS)))
    {
        g_nav.b_servo_wait = false;
    }

    return (g_nav.b_servo_wait);
}

/*!
 * @brief Keep the accepted plan: the side, B (the closest reading) and the
 *        side's raw cleared angle, and work out the first leg, A, E and
 *        the LEVEL leg (extras_plan.c).
 *
 * @param[in] p_result Planner result, replayed from the scan profile.
 * @param[in] side     Side the car passes on (EXTRAS_SIDE_*).
 */
static void keep_plan (extras_plan_result_t const * p_result, uint32_t side)
{
    int32_t const clear_deci = p_result->clear_deci[side % EXTRAS_SCAN_SIDES];

    g_nav.closest_mm   = p_result->closest_mm;
    g_nav.ahead_end_mm = extras_plan_ahead_mm(g_nav.closest_mm);
    g_nav.room_mm      = extras_plan_room_mm(g_nav.closest_mm, clear_deci);
    g_nav.out_mm       = extras_plan_out_mm(g_nav.room_mm);
    g_nav.level_mm     = extras_plan_level_mm(g_nav.closest_mm, g_nav.ahead_end_mm);

    LOG_CONT("[b5-nav] plan: B %d mm, cleared %d deg, A %d mm, first "
                    "leg %d mm, out %d mm, level %d mm\n",
              (INT)g_nav.closest_mm, (INT)(clear_deci / DECI_PER_DEG),
              (INT)g_nav.room_mm, (INT)g_nav.ahead_end_mm, (INT)g_nav.out_mm,
              (INT)g_nav.level_mm);

    /* Cast is safe: the side field is 1 bit wide. */
    log_rec(EXTRAS_LOG_PLAN, (uint8_t)side, 0U, 0U, (int32_t)g_nav.closest_mm,
            (int32_t)g_nav.room_mm, clear_deci);
}

/*!
 * @brief Print the 25 scan readings and why each side was or was not
 *        usable, and hand back the planner's result (the side offsets the
 *        bypass uses). The scan is replayed through
 *        scan_algo and extras_plan from the published profile (the same slots
 *        in the same order, so the result is the one Buddy 5 found).
 *
 * @param[in]  p_profile EXTRAS_SLOT_COUNT packed slots.
 * @param[out] p_result  The planner's result for this scan.
 */
static void explain_scan (uint16_t const *   p_profile,
                          extras_plan_result_t * p_result)
{
    static extras_scan_ctx_t g_replay;
    extras_plan_cfg_t          cfg;
    uint32_t               slot   = 0U;
    uint32_t               guard  = 0U;
    bool                   b_done = false;

    for (slot = 0U; slot < EXTRAS_SLOT_COUNT; slot++)
    {
        uint16_t const value = p_profile[slot];

        /* Cast is safe: slot is 0..24. */
        LOG_CONT(" %+3d:",
                  (INT)(extras_scan_slot_deci((uint8_t)slot) / DECI_PER_DEG));

        if (!extras_pack_slot_is_measured(value))
        {
            LOG_CONT("--  ");
        }
        else if (EXTRAS_CLASS_VALID == extras_pack_slot_class(value))
        {
            LOG_CONT("%-4d", (INT)extras_pack_slot_dist(value));
        }
        else
        {
            LOG_CONT("%s", (EXTRAS_CLASS_OPEN == extras_pack_slot_class(value))
                                      ? "OPN "
                                      : "UNK ");
        }

        if ((EXTRAS_SLOT_COUNT - 1U) == slot)
        {
            LOG_CONT("\n");
        }
    }

    /* Replay: feed each requested slot's stored value twice, so the
     * agreement rule settles it exactly as before. */
    extras_scan_start(&g_replay);
    while ((!b_done) && (guard < EXTRAS_SLOT_COUNT))
    {
        uint8_t const target = extras_scan_target(&g_replay);

        if (EXTRAS_SCAN_NO_SLOT == target)
        {
            b_done = true;
        }
        else
        {
            uint16_t const value = p_profile[target];
            uint8_t const  cls   = extras_pack_slot_class(value);
            uint16_t const dist  = extras_pack_slot_dist(value);

            (void)extras_scan_feed(&g_replay, cls, dist);
            b_done = extras_scan_feed(&g_replay, cls, dist);
        }

        guard++;
    }

    extras_plan_cfg_default(&cfg);
    extras_plan_build(&g_replay, &cfg, p_result);

    /* Edge states: 0 none, 1 bounded, 2 unbounded (reaches +-60 deg).
     * need 65535 = side blocked (unbounded, too far out, or something in
     * the way beyond the edge). flat face 1 = side echoes were left out
     * (extras_scan_is_side_echo()). */
    LOG_CONT("[b5-nav] scan: closest %d mm, region %d..%d deg | LEFT "
                    "edge %d last hit %d deg need %d mm | RIGHT edge %d last "
                    "hit %d deg need %d mm | flat face %d\n",
              (INT)g_replay.closest_mm,
              (INT)(extras_scan_slot_deci(g_replay.region_lo) / DECI_PER_DEG),
              (INT)(extras_scan_slot_deci(g_replay.region_hi) / DECI_PER_DEG),
              (INT)g_replay.edge_state[EXTRAS_SCAN_LEFT],
              (INT)(extras_scan_slot_deci(g_replay.edge_hit[EXTRAS_SCAN_LEFT]) /
                    DECI_PER_DEG),
              (INT)p_result->need_mm[EXTRAS_SIDE_LEFT],
              (INT)g_replay.edge_state[EXTRAS_SCAN_RIGHT],
              (INT)(extras_scan_slot_deci(g_replay.edge_hit[EXTRAS_SCAN_RIGHT]) /
                    DECI_PER_DEG),
              (INT)p_result->need_mm[EXTRAS_SIDE_RIGHT], (INT)g_replay.b_flat);
}

/*!
 * @brief React to one Buddy 5 event.
 *
 * @param[in] evt Packed event word.
 * @param[in] now Current time, ms.
 */
static void handle_event (uint32_t evt, uint32_t now)
{
    uint32_t const type   = extras_pack_get(evt, EXTRAS_EV_TYPE_SH, EXTRAS_EV_TYPE_MK);
    uint32_t const req_id = extras_pack_get(evt, EXTRAS_EV_ID_SH, EXTRAS_EV_ID_MK);
    uint32_t const status = extras_pack_get(evt, EXTRAS_EV_STATUS_SH, EXTRAS_EV_STATUS_MK);
    bool const b_go = (EXTRAS_RESULT_OK == status) || (EXTRAS_RESULT_CLEAR == status);

    if (EXTRAS_EVT_OBSTACLE == type)
    {
        uint32_t const dist_mm = extras_pack_get(evt, EXTRAS_EV_DIST_SH, EXTRAS_EV_DIST_MK);

        LOG_CONT("[b5-nav] OBSTACLE at %d mm: holding\n", (INT)dist_mm);
        log_rec(EXTRAS_LOG_OBST, 0U, 0U, 0U, (int32_t)dist_mm, 0, 0);
        hold_car(true);
        g_nav.hold_ms        = now;
        g_nav.req_id         = g_nav.next_id;
        g_nav.b_scan_pending = true;
        g_nav.closest_mm     = dist_mm;

        /* Cast is safe: the id is masked to 5 bits. */
        g_nav.next_id = (uint8_t)((g_nav.next_id + 1U) & EXTRAS_ID_MAX);
    }
    else if (EXTRAS_EVT_FAULT == type)
    {
        uint32_t const cause = extras_pack_get(evt, EXTRAS_EV_CAUSE_SH, EXTRAS_EV_CAUSE_MK);
        uint32_t const faults =
            extras_pack_get(evt, EXTRAS_EV_FAULTS_SH, EXTRAS_EV_FAULTS_MK);

        LOG_CONT("[b5-nav] FAULT cause=%d: holding\n", (INT)cause);

        /* Casts are safe: the cause is 4 bits, the faults 8 bits. */
        log_rec(EXTRAS_LOG_FAULT, (uint8_t)cause, 0U, (uint8_t)faults, 0, 0, 0);
        hold_car(true);
        g_nav.b_scan_pending = false;
        g_nav.retry_ms       = now;

        /* Cast is safe: the count stops at EXTRAS_NO_PATH_TRIES. */
        if ((EXTRAS_CAUSE_NO_SAFE_PATH == cause) &&
            (g_nav.no_path_count < EXTRAS_NO_PATH_TRIES))
        {
            g_nav.no_path_count = (uint8_t)(g_nav.no_path_count + 1U);
        }
    }
    else if (EXTRAS_EVT_SCAN_RESULT == type)
    {
        uint16_t         profile[EXTRAS_SLOT_COUNT];
        extras_plan_result_t result;
        uint32_t const   side = extras_pack_get(evt, EXTRAS_EV_SIDE_SH, EXTRAS_EV_SIDE_MK);
        uint32_t const width = extras_pack_get(evt, EXTRAS_EV_WIDTH_SH, EXTRAS_EV_WIDTH_MK);
        uint32_t const angle = extras_pack_get(evt, EXTRAS_EV_ANGLE_SH, EXTRAS_EV_ANGLE_MK);
        uint32_t       index = 0U;

        LOG_CONT("[b5-nav] SCAN_RESULT status=%d side=%d width=%d mm "
                        "start=%d deg\n",
                  (INT)status, (INT)side, (INT)width, (INT)angle);

        /* Without the profile: no cleared angle (A is then the cap) and
         * the distance the obstacle was detected at. */
        result.closest_mm                = (uint16_t)g_nav.closest_mm;
        result.need_mm[EXTRAS_SIDE_LEFT]     = EXTRAS_PLAN_SIDE_BLOCKED;
        result.need_mm[EXTRAS_SIDE_RIGHT]    = EXTRAS_PLAN_SIDE_BLOCKED;
        result.clear_deci[EXTRAS_SIDE_LEFT]  = EXTRAS_PLAN_CLEAR_UNBOUNDED;
        result.clear_deci[EXTRAS_SIDE_RIGHT] = EXTRAS_PLAN_CLEAR_UNBOUNDED;

        /* Casts are safe: req_id is 5 bits and angle 6 bits wide. */
        if (E_OK == obstacle_extras_copy_profile(profile))
        {
            index = extras_log_profile((uint8_t)req_id, profile);
            explain_scan(profile, &result);
        }
        log_rec(EXTRAS_LOG_RESULT, (uint8_t)status, (uint8_t)side, (uint8_t)angle,
                (int32_t)width, (int32_t)index, 0);

        if ((g_nav.req_id == req_id) && b_go)
        {
            /* Cast is safe: the side field is 1 bit wide. */
            g_nav.no_path_count = 0U;
            g_nav.side          = (uint8_t)side;
            keep_plan(&result, side);
            post(EXTRAS_CMD_ACCEPT, g_nav.req_id);
        }
    }
    else
    {
        /* STEER words are read with obstacle_extras_get_steer(). */
    }
}

/*!
 * @brief Tell Buddy 5 the bypass is over (LINE_FOUND), repeated every
 *        EXTRAS_NAV_LINE_REPEAT_MS until it is back in MONITOR.
 *
 * @param[in] b_line_seen The bypass has ended on the line (DONE).
 * @param[in] now         Current time, ms.
 */
static void look_for_line (bool b_line_seen, uint32_t now)
{
    if (b_line_seen && ((now - g_nav.line_ms) >= EXTRAS_NAV_LINE_REPEAT_MS))
    {
        g_nav.line_ms = now;
        post(EXTRAS_CMD_LINE_FOUND, g_nav.req_id);
    }
}

/*!
 * @brief The motion service has latched a fault: print and log, for each
 *        wheel, the last sample it judged, so the failed check can be seen
 *        (FEEDBACK: speed_valid 0 means an invalid encoder step or a sample
 *        over 100 ms old; NO_EDGES: duty set but no new edges for 2 s).
 *
 * @param[in] p_status Motion service status at the fault.
 */
static void report_fault (motion_service_status_t const * p_status)
{
    uint32_t wheel = 0U;

    LOG_CONT("[b5-nav] MOTION FAULT %d: sample %d, elapsed %d us\n",
              (INT)p_status->control.fault,
              (INT)p_status->control.sample_available,
              (INT)p_status->control.sample.elapsed_us);

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        motion_wheel_sample_t const * const p_wheel =
            &p_status->control.sample.wheel[wheel];

        LOG_CONT("[b5-nav]   wheel %d: target %d um/s, duty %d, "
                        "speed %d um/s, speed_valid %d, odo_valid %d, "
                        "bad encoder steps %d\n",
                  (INT)wheel, (INT)p_status->control.target_um_s[wheel],
                  (INT)p_status->control.duty[wheel],
                  (INT)p_wheel->velocity_um_s, (INT)p_wheel->speed_valid,
                  (INT)p_wheel->odometry_valid, (INT)p_wheel->invalid_delta);

        /* Casts are safe: wheel is 0..1 and the flags are 0/1. */
        log_rec(EXTRAS_LOG_MFAULT, (uint8_t)wheel, (uint8_t)p_wheel->speed_valid,
                (uint8_t)p_wheel->odometry_valid,
                (int32_t)p_wheel->invalid_delta,
                p_wheel->velocity_um_s / UM_PER_MM,
                (int32_t)p_status->control.sample.elapsed_us);
    }
}

/*!
 * @brief Read the motion service once per pass: keep the wheel distances
 *        for the dead reckoning and the trace, and print and log the
 *        state, fault and hold mask whenever one of them changes, so a
 *        motor fault shows up.
 *
 * NOTE: States: 0 IDLE, 1 RUNNING, 2 REVERSING, 3 STOPPED, 4 FAULT.
 * Faults: 0 NONE, 1 COMMAND, 2 EXPIRED, 3 FEEDBACK, 4 GAP, 5 NO_EDGES,
 * 6 OUTPUT. STOPPED and FAULT both need a reboot.
 */
static void update_motion (void)
{
    motion_service_status_t status;

    if (E_OK == motion_service_read_status(&status))
    {
        /* Casts are safe: the enums are 0..6 and the hold mask is 0..7. */
        uint32_t const word = ((uint32_t)status.control.state << 16U) |
                              ((uint32_t)status.control.fault << 8U) |
                              status.hold_mask;
        bool const b_changed = (word != g_nav.motion_word);
        uint32_t   wheel     = 0U;

        if (b_changed)
        {
            g_nav.motion_word = word;
            LOG_CONT("[b5-nav] motion state=%d fault=%d hold=%d\n",
                      (INT)status.control.state, (INT)status.control.fault,
                      (INT)status.hold_mask);
            log_rec(EXTRAS_LOG_MOTION, (uint8_t)status.control.state,
                    (uint8_t)status.control.fault, (uint8_t)status.hold_mask, 0,
                    0, 0);
        }

        if (b_changed && (MOTION_FAULT == status.control.state))
        {
            report_fault(&status);
        }

        for (wheel = 0U; wheel < WHEELS; wheel++)
        {
            g_nav.b_vel_ok[wheel] =
                status.control.sample_available &&
                status.control.sample.wheel[wheel].speed_valid;

            /* NOTE: odometry_valid latches false after one invalid encoder
             * step; the displacement is still counted, so keep using it. */
            if (status.control.sample_available)
            {
                g_nav.wheel_um[wheel] =
                    true_wheel_um(&status.control.sample, wheel);
                g_nav.vel_um_s[wheel] =
                    true_wheel_speed(&status.control.sample, wheel);

                /* Cast is safe: a run is far shorter than 2147 km. */
                g_nav.wheel_mm[wheel] =
                    (int32_t)(g_nav.wheel_um[wheel] / UM_PER_MM);
            }
        }
    }
}

/*!
 * @brief Log a TRACE record every EXTRAS_LOG_TRACE_MS while Buddy 5 drives
 *        the car (BYPASS, REACQUIRE), and every EXTRAS_LOG_IDLE_TRACE_MS
 *        otherwise.
 *
 * @param[in] status  Packed Buddy 5 status word.
 * @param[in] b_left  Left IR sensor sees the line.
 * @param[in] b_right Right IR sensor sees the line.
 * @param[in] now     Current time, ms.
 */
static void trace (uint32_t status, bool b_left, bool b_right, uint32_t now)
{
    uint32_t const state = extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK);
    bool const     b_driving =
        (EXTRAS_STATE_BYPASS == state) || (EXTRAS_STATE_REACQUIRE == state);
    uint32_t const period_ms =
        b_driving ? EXTRAS_LOG_TRACE_MS : EXTRAS_LOG_IDLE_TRACE_MS;

    if ((now - g_nav.trace_ms) >= period_ms)
    {
        uint32_t const steer =
            extras_pack_get(obstacle_extras_get_steer(), EXTRAS_EV_STEER_SH, EXTRAS_EV_STEER_MK);
        uint32_t bits = steer << TRACE_STEER_SH;

        g_nav.trace_ms = now;

        if (b_left)
        {
            bits |= TRACE_IR_LEFT;
        }

        if (b_right)
        {
            bits |= TRACE_IR_RIGHT;
        }

        /* Casts are safe: state 3 bits, class 2 bits, bits 4 bits. */
        log_rec(EXTRAS_LOG_TRACE, (uint8_t)state,
                (uint8_t)extras_pack_get(status, EXTRAS_ST_CLASS_SH, EXTRAS_ST_CLASS_MK),
                (uint8_t)bits,
                (int32_t)extras_pack_get(status, EXTRAS_ST_DIST_SH, EXTRAS_ST_DIST_MK),
                g_nav.wheel_mm[WHEEL_LEFT], g_nav.wheel_mm[WHEEL_RIGHT]);
    }
}

/*!
 * @brief Emergency-stop link for Buddy 5: hold the car at once.
 *
 * @return The result of motion_service_set_hold().
 */
static ER nav_stop (void)
{
    return (motion_service_set_hold(MOTION_HOLD_OBSTACLE, true));
}

/*!
 * @brief Distance link for Buddy 5: average forward travel of the wheels.
 *
 * NOTE: Before the first good sample, and whenever a sample is not valid,
 * the last good reading is returned, so Buddy 5 never mixes this with its
 * own time-based estimate.
 *
 * @param[out] p_distance_mm Distance travelled since boot, mm.
 *
 * @return E_OK.
 */
static ER nav_distance_mm (uint32_t * p_distance_mm)
{
    motion_service_status_t status;
    bool                    b_valid = false;

    if (E_OK == motion_service_read_status(&status))
    {
        /* NOTE: odometry_valid latches false after one invalid encoder
         * step, which would freeze the odometer for the rest of the run;
         * the displacement is still counted, so only the sample is
         * checked. */
        b_valid = status.control.sample_available;
    }

    if (b_valid)
    {
        int64_t const sum_um =
            true_wheel_um(&status.control.sample, WHEEL_LEFT) +
            true_wheel_um(&status.control.sample, WHEEL_RIGHT);
        int64_t const mean_mm = (sum_um / 2) / UM_PER_MM;

        if (mean_mm <= 0)
        {
            g_odo.last_mm = 0U;
        }
        else if (mean_mm >= (int64_t)UINT32_MAX)
        {
            g_odo.last_mm = UINT32_MAX;
        }
        else
        {
            /* Cast is safe: mean_mm is 1 .. UINT32_MAX - 1 here. */
            g_odo.last_mm = (uint32_t)mean_mm;
        }
    }

    *p_distance_mm = g_odo.last_mm;

    return (E_OK);
}

/*!
 * @brief Travel of one real wheel, undoing the crossed encoder plugs
 *        (EXTRAS_ENCODERS_CROSSED).
 *
 * NOTE: With the plugs crossed, the motion service's wheel 0 reads the
 * right encoder and wheel 1 the left one, each with the other wheel's
 * forward sign, so forward travel counts negative (seen in the flight log:
 * both totals fell while the car drove forwards).
 *
 * @param[in] p_sample Motion service sample.
 * @param[in] wheel    WHEEL_LEFT or WHEEL_RIGHT (the real wheel).
 *
 * @return Signed travel since boot in um, positive = forwards.
 */
static int64_t true_wheel_um (motion_sample_t const * p_sample, uint32_t wheel)
{
    int64_t travel_um = 0;

#if (EXTRAS_ENCODERS_CROSSED)
    travel_um = -(p_sample->wheel[(WHEELS - 1U) - wheel].displacement_um);
#else
    travel_um = p_sample->wheel[wheel].displacement_um;
#endif

    return (travel_um);
}

/*!
 * @brief Speed of one real wheel, undoing the crossed encoder plugs
 *        (EXTRAS_ENCODERS_CROSSED), for the virtual bumper.
 *
 * @param[in] p_sample Motion service sample.
 * @param[in] wheel    WHEEL_LEFT or WHEEL_RIGHT (the real wheel).
 *
 * @return Signed speed in um/s, positive = forwards.
 */
static int32_t true_wheel_speed (motion_sample_t const * p_sample,
                                 uint32_t                wheel)
{
    int32_t speed_um_s = 0;

#if (EXTRAS_ENCODERS_CROSSED)
    speed_um_s = -(p_sample->wheel[(WHEELS - 1U) - wheel].velocity_um_s);
#else
    speed_um_s = p_sample->wheel[wheel].velocity_um_s;
#endif

    return (speed_um_s);
}

/*** end of file ***/
