/** @file obstacle_nav.c
 *
 * @brief Buddy 5 Phase 2 glue between the obstacle statechart, the line
 *        task in main.c and the motion service.
 *
 * What the line task does with Buddy 5 (D96):
 *   - OBSTACLE      hold the car, wait OBSTACLE_NAV_BRAKE_MS, send SCAN_REQ(id).
 *   - SCAN_RESULT   OK or CLEAR: send ACCEPT(id), and keep the plan: the
 *                   side, the start angle and the sideways offset the car
 *                   needs.
 *   - BYPASS        pivot away from the obstacle (inside wheel stopped)
 *                   until Buddy 5 sees the way ahead is clear AND the
 *                   planned start angle is reached; this task reports the
 *                   angle with TURNED (same opcode as LINE_FOUND). A box
 *                   plan turns to its start angle, a wall plan to +90.
 *   - REACQUIRE     drive the rest of the bypass via stop-and-look side checks:
 *                     TURN          keep pivoting until the planned angle;
 *                     OUT           box plan: straight on until planned lateral offset;
 *                     ALIGN         box plan: turn back to parallel with original line;
 *                     ALONG         drive forward with forward-only sonar;
 *                     LOOK_STOP     stop before rotating to side;
 *                     LOOK_IN       pivot body to face obstacle/wall;
 *                     LOOK_PING     take stationary side checks;
 *                     LOOK_OUT      pivot back to along heading;
 *                     EXTRA         extra forward travel once passed;
 *                     RETURN_PIVOT  pivot back towards line;
 *                     RETURN_DRIVE  drive forward to line until IR triggers.
 *   - FAULT         hold the car; retry RESET + START every 3 s.
 *   - MONITOR       release the hold and let the line follower drive,
 *                   except while Buddy 5's latest reading is a VALID echo
 *                   within the detection distance.
 *
 * Position frame (dead reckoning from the wheel encoders): x is sideways,
 * positive to the left of the original line; y is along the line; both
 * are for the point midway between the wheels and start at 0 when BYPASS
 * begins. The heading is worked out from the total wheel difference, so
 * it does not drift between passes.
 *
 * Everything that happens is also written to the flight recorder
 * (flight_log.c), which prints it when a USB serial monitor is opened.
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
 * docs/obstacle.md). Casts to (INT) feed the log macros only; every
 * value printed is below 2^16.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "platform/log.h"
#include "motion/motion.h"
#include "obstacle_config.h"
#include "flight_log.h"
#include "obstacle_nav.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "obstacle_internal.h"
#include "scan_algo.h"

/* Micrometres per millimetre. */
#define UM_PER_MM (1000)

/* Deci-degrees per degree. */
#define DECI_PER_DEG (10)

/* Q14 fixed point: 16384 = 1.0. */
#define Q14_ONE (16384)

/* The two wheels of the motion service. */
#define WHEEL_LEFT  (0U)
#define WHEEL_RIGHT (1U)
#define WHEELS      (2U)

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

/* IR and steer bits of a TRACE record (see flight_log.c). */
#define TRACE_IR_LEFT  (0x1U)
#define TRACE_IR_RIGHT (0x2U)
#define TRACE_STEER_SH (2U)

/* Largest value logged in a 16-bit field. */
#define LOG_I16_MAX (32767)

/* Maximum clamp for heading drift trim in deci-degrees. */
#define WALL_TRIM_MAX_DECI (800)

/* Navigation state; only the line task touches it. NOTE: Booleans are b_
 * members of a g_ struct, which satisfies both BARR-C 7.1.m/7.1.o and the
 * team .clang-tidy. */
typedef struct
{
    bool     b_enabled;
    bool     b_scan_pending;
    bool     b_turned;
    bool     b_servo_wait;
    bool     b_pausing;
    bool     b_ir_armed;
    bool     b_ir_cleared;
    bool     b_wall;
    bool     b_have_prev_gap;
    bool     b_passed;
    uint8_t  req_id;
    uint8_t  next_id;
    uint8_t  last_state;
    uint8_t  start_deg;
    uint8_t  side;
    uint8_t  phase;
    uint8_t  wait_seq;
    uint8_t  clear_checks;
    uint8_t  side_ping_count;
    uint8_t  side_agree_count;
    uint16_t last_side_dist_mm;
    int32_t  prev_gap_mm;
    int32_t  prev_lean_deci;
    int32_t  wall_trim_deci;
    int32_t  along_head_deci;
    uint32_t look_stop_ms;
    int64_t  leg_base_um;
    uint32_t wait_ms;
    uint32_t pause_ms;
    uint32_t last_drive;
    uint32_t hold_ms;
    uint32_t retry_ms;
    uint32_t line_ms;
    uint32_t trace_ms;
    uint32_t motion_word;
    int32_t  need_mm;
    int32_t  obst_mm;
    int32_t  wheel_mm[WHEELS];
    int32_t  last_cmd[WHEELS];
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
    .b_turned           = false,
    .b_servo_wait       = false,
    .b_pausing          = false,
    .b_ir_armed         = false,
    .b_ir_cleared       = false,
    .b_wall             = false,
    .b_have_prev_gap    = false,
    .b_passed           = false,
    .req_id             = 0U,
    .next_id            = 0U,
    .last_state         = NAV_UNSEEN,
    .start_deg          = 0U,
    .side               = OBSTACLE_SIDE_LEFT,
    .phase              = OBSTACLE_PHASE_TURN,
    .wait_seq           = 0U,
    .clear_checks       = 0U,
    .side_ping_count    = 0U,
    .side_agree_count   = 0U,
    .last_side_dist_mm  = 0U,
    .prev_gap_mm        = 0,
    .prev_lean_deci     = 0,
    .wall_trim_deci     = 0,
    .along_head_deci    = 0,
    .look_stop_ms       = 0U,
    .leg_base_um        = 0,
    .wait_ms            = 0U,
    .pause_ms           = 0U,
    .last_drive         = NAV_DRIVE_UNSEEN,
    .hold_ms            = 0U,
    .retry_ms           = 0U,
    .line_ms            = 0U,
    .trace_ms           = 0U,
    .motion_word        = UINT32_MAX,
    .need_mm            = 0,
    .obst_mm            = 0,
    .wheel_mm           = {0, 0},
    .last_cmd           = {0, 0},
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
static int32_t  lateral_mm (void);
static int32_t  ahead_mm (void);
static void     reset_pose (void);
static void     update_pose (void);
static void     note_turned (bool b_turned, uint32_t now);
static void     drive_wheels (uint8_t steer, int32_t left_um_s,
                              int32_t right_um_s);
static bool     is_reverse_pause (int32_t const p_speed[WHEELS]);
static void     set_phase (uint8_t phase);
static void     step_pivot (uint32_t now);
static void     drive_reacquire (uint32_t status, uint32_t now);
static void step_reacquire (uint32_t status, bool b_line_seen, uint32_t now);
static void     update_ir_gate (bool b_line_seen);
static bool     is_obstacle_close (uint32_t status);
static bool     is_servo_moving (uint32_t status, uint32_t now);
static void     report_state (uint32_t status);
static void     keep_plan (uint32_t angle, bool b_wall);
static void     explain_scan (uint16_t const * p_profile);
static void     handle_event (uint32_t evt, uint32_t now);
static void     look_for_line (bool b_line_seen, uint32_t now);
static void     update_motion (void);
static int64_t true_wheel_um (motion_feedback_t const * p_feedback,
                              uint32_t wheel);
static void    report_fault (motion_status_t const * p_status,
                             motion_feedback_t const * p_feedback);
static void    trace (uint32_t status, bool b_left, bool b_right, uint32_t now);
static ER      nav_stop (void);
static ER      nav_distance_mm (uint32_t * p_distance_mm);

/*!
 * @brief Start Buddy 5 with the motion links, then RESET and START it.
 *
 * Call once from the line task on processor 1, after gpio_irq_init(),
 * motion_service_init() and ir_capture_init(). If it fails, obstacle_nav_step()
 * only services the flight recorder, so line following works without
 * Buddy 5 and the failure can still be read back.
 *
 * @return The result of obstacle_init().
 */
ER obstacle_nav_init (void)
{
    obstacle_links_t const links  = {0, nav_stop, nav_distance_mm};
    ER const               result = obstacle_init(&links);

    log_rec(OBSTACLE_LOG_BOOT, 0U, 0U, 0U, (int32_t)result, 0, 0);

    if (E_OK == result)
    {
        g_nav.b_enabled = true;
        g_nav.retry_ms  = now_ms();
        post(OBSTACLE_CMD_RESET, 0U);
        post(OBSTACLE_CMD_START, 0U);
    }

    return (result);
}

/*!
 * @brief One pass of the obstacle handshake. Call every line-task loop.
 *
 * @param[in] b_line_left  Left IR sensor sees the line.
 * @param[in] b_line_right Right IR sensor sees the line.
 *
 * @return True if Buddy 5 is in charge of the motors this pass, so the
 *         line follower must not send a speed.
 */
bool obstacle_nav_step (bool b_line_left, bool b_line_right)
{
    bool     b_in_charge = false;
    uint32_t evt         = 0U;
    uint32_t state       = OBSTACLE_STATE_MONITOR;
    uint32_t status      = 0U;
    uint32_t now         = 0U;

    flight_log_service();
    update_motion();

    if (g_nav.b_enabled)
    {
        now = now_ms();
        while (E_OK == obstacle_get_event(&evt, TMO_POL))
        {
            handle_event(evt, now);
        }

        if (g_nav.b_scan_pending && ((now - g_nav.hold_ms) >= OBSTACLE_NAV_BRAKE_MS))
        {
            g_nav.b_scan_pending = false;
            post(OBSTACLE_CMD_SCAN_REQ, g_nav.req_id);
        }

        status = obstacle_get_status();
        state  = obstacle_pack_get(status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);
        report_state(status);
        trace(status, b_line_left, b_line_right, now);

        switch (state)
        {
            case OBSTACLE_STATE_MONITOR:
                b_in_charge = (is_servo_moving(status, now)) ||
                              (is_obstacle_close(status));
                hold_car(b_in_charge);
            break;

            case OBSTACLE_STATE_BYPASS:
                /* Pivot until Buddy 5 sees the way ahead is clear and the
                 * planned angle is reached (TURNED). NOTE: the
                 * side comes from the plan, not from the STEER word: the
                 * status and the STEER word are read at different moments,
                 * and on the pass where Buddy 5 moves on to REACQUIRE the
                 * word already says STRAIGHT, which used to jerk the inside
                 * wheel 0 -> 40 -> 0 mm/s in 20 ms. */
                update_pose();
                if (is_servo_moving(status, now))
                {
                    /* The servo is returning to 0 after the scan. */
                    hold_car(true);
                }
                else
                {
                    hold_car(false);
                    step_pivot(now);
                }
                b_in_charge = true;
            break;

            case OBSTACLE_STATE_REACQUIRE:
                step_reacquire(status, b_line_left || b_line_right, now);
                b_in_charge = true;
            break;

            case OBSTACLE_STATE_STOPPED:
                hold_car(true);
                /* Retry policy: only a start after boot or a transient cause
                 * restarts by itself. A collision, a scan with no safe path, a
                 * sensor fault and a STOP wait for an explicit RESET and START. */
                if (obstacle_pack_cause_may_auto_restart(
                        obstacle_pack_get(status, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK)) &&
                    ((now - g_nav.retry_ms) >= OBSTACLE_NAV_RETRY_MS))
                {
                    g_nav.retry_ms = now;
                    post(OBSTACLE_CMD_RESET, 0U);
                    post(OBSTACLE_CMD_START, 0U);
                }
                b_in_charge = true;
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
 * @brief One line-task pass in REACQUIRE: stand still while the servo
 *        settles, otherwise drive the stop-and-look bypass and look for
 *        the line.
 *
 * @param[in] status      Packed Buddy 5 status word.
 * @param[in] b_line_seen An IR sensor sees the line.
 * @param[in] now         Current time, ms.
 */
static void step_reacquire (uint32_t status, bool b_line_seen, uint32_t now)
{
    update_pose();

    if (is_servo_moving(status, now))
    {
        hold_car(true);
    }
    else
    {
        bool const b_line_active =
            (OBSTACLE_PHASE_ALONG == g_nav.phase) ||
            (OBSTACLE_PHASE_RETURN_DRIVE == g_nav.phase);

        hold_car(false);
        update_ir_gate(b_line_seen);
        drive_reacquire(status, now);
        look_for_line(b_line_active && g_nav.b_ir_cleared && b_line_seen, now);
    }
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
 * @param[in] kind  Record kind (OBSTACLE_LOG_*).
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
    flight_log_rec_t rec;

    rec.t_ms  = 0U;
    rec.kind  = kind;
    rec.val_a = val_a;
    rec.val_b = val_b;
    rec.val_c = val_c;
    rec.val_d = clamp_i16(val_d);
    rec.val_e = clamp_i16(val_e);
    rec.val_f = val_f;
    flight_log_add(&rec);
}

/*!
 * @brief Post one command to Buddy 5, print it and log it.
 *
 * @param[in] type   Command type (OBSTACLE_CMD_*).
 * @param[in] req_id Request id.
 */
static void post (uint32_t type, uint8_t req_id)
{
    ER const result = obstacle_post_cmd(obstacle_pack_cmd(type, req_id));

    LOG_NOW("INFO", "line:nav", "cmd %d id=%d -> %d\n", (INT)type, (INT)req_id,
              (INT)result);

    /* Cast is safe: the command type is 3 bits wide. */
    log_rec(OBSTACLE_LOG_CMD, (uint8_t)type, req_id, 0U, (int32_t)result, 0, 0);
}

/*!
 * @brief Set or release the obstacle hold. The motion service ignores a
 *        call that does not change the hold, so this runs every pass.
 *
 * @param[in] b_held True to hold the car.
 */
static void hold_car (bool b_held)
{
    (void)motion_set_hold(MOTION_HOLD_OBSTACLE, b_held);
}

/*!
 * @brief Sign that turns "away from the obstacle" into left / right.
 *
 * @return +1 when passing on the left, -1 when passing on the right.
 */
static int32_t side_sign (void)
{
    return ((OBSTACLE_SIDE_LEFT == g_nav.side) ? 1 : -1);
}

/*!
 * @brief Heading since BYPASS began, from the total wheel difference:
 *        (right - left) / track, positive = left.
 *
 * @return Heading in deci-degrees.
 */
static int32_t heading_deci (void)
{
    int64_t const left_um =
        g_nav.wheel_um[WHEEL_LEFT] - g_nav.base_um[WHEEL_LEFT];
    int64_t const right_um =
        g_nav.wheel_um[WHEEL_RIGHT] - g_nav.base_um[WHEEL_RIGHT];
    int64_t const track_um = (int64_t)OBSTACLE_WHEEL_TRACK_MM * UM_PER_MM;

    /* Cast is safe: a bypass turns the car far less than 2^31 deci-deg. */
    return ((int32_t)(((right_um - left_um) * OBSTACLE_NAV_DECI_PER_RAD) / track_um));
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
 * @brief Start the dead reckoning when BYPASS begins.
 */
static void reset_pose (void)
{
    uint32_t wheel = 0U;

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        g_nav.base_um[wheel] = g_nav.wheel_um[wheel];
        g_nav.prev_um[wheel] = g_nav.wheel_um[wheel];
    }

    g_nav.x_um                  = 0;
    g_nav.y_um                  = 0;
    g_nav.last_cmd[WHEEL_LEFT]  = 0;
    g_nav.last_cmd[WHEEL_RIGHT] = 0;
    g_nav.b_pausing             = false;
    g_nav.b_turned              = false;
    g_nav.b_ir_armed            = false;
    g_nav.b_ir_cleared          = false;
    g_nav.b_have_prev_gap       = false;
    g_nav.b_passed              = false;
    g_nav.clear_checks          = 0U;
    g_nav.side_ping_count       = 0U;
    g_nav.side_agree_count      = 0U;
    g_nav.last_side_dist_mm     = 0U;
    g_nav.prev_gap_mm           = 0;
    g_nav.prev_lean_deci        = 0;
    g_nav.wall_trim_deci        = 0;
    g_nav.along_head_deci       = 0;
    g_nav.look_stop_ms          = 0U;
    g_nav.leg_base_um           = 0;
    g_nav.phase                 = OBSTACLE_PHASE_TURN;
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
    g_nav.x_um += (step_um * obstacle_plan_sin_q14(heading)) / Q14_ONE;
    g_nav.y_um += (step_um * obstacle_plan_cos_q14(heading)) / Q14_ONE;
    g_nav.prev_um[WHEEL_LEFT]  = g_nav.wheel_um[WHEEL_LEFT];
    g_nav.prev_um[WHEEL_RIGHT] = g_nav.wheel_um[WHEEL_RIGHT];
}

/*!
 * @brief The pivot has reached the planned start angle: log it the first
 *        time, and tell Buddy 5 (TURNED) every OBSTACLE_NAV_LINE_REPEAT_MS
 *        until it leaves BYPASS, so a full command queue cannot lose it.
 *
 * @param[in] b_turned The pivot has turned at least the planned angle.
 * @param[in] now      Current time, ms.
 */
static void note_turned (bool b_turned, uint32_t now)
{
    if (b_turned)
    {
        int32_t const turned = side_sign() * heading_deci();

        if (!g_nav.b_turned)
        {
            g_nav.b_turned = true;
            LOG_NOW("INFO", "line:nav", "escape turned %d deg (plan %d)\n",
                      (INT)(turned / DECI_PER_DEG), (INT)g_nav.start_deg);
            log_rec(OBSTACLE_LOG_TURNED, 0U, 0U, 0U, turned, (int32_t)g_nav.start_deg, 0);
        }

        if ((now - g_nav.line_ms) >= OBSTACLE_NAV_LINE_REPEAT_MS)
        {
            g_nav.line_ms = now;
            post(OBSTACLE_CMD_TURNED, g_nav.req_id);
        }
    }
}

/*!
 * @brief Turn a steering decision into a speed command, and print and log
 *        it whenever the speeds change.
 *
 * STRAIGHT drives both wheels at OBSTACLE_NAV_SPEED_UM_S; LEFT and RIGHT run the
 * inside wheel at inside_um_s and the outside wheel at outside_um_s; HOLD
 * stands still.
 *
 * @param[in] steer      Steering (OBSTACLE_STEER_*).
 * @param[in] left_um_s  Left wheel speed (um/s).
 * @param[in] right_um_s Right wheel speed (um/s).
 */
static void drive_wheels (uint8_t steer, int32_t left_um_s, int32_t right_um_s)
{
    int32_t speed[WHEELS] = {0, 0};

    speed[WHEEL_LEFT]  = left_um_s;
    speed[WHEEL_RIGHT] = right_um_s;

    if (is_reverse_pause(speed))
    {
        speed[WHEEL_LEFT]  = 0;
        speed[WHEEL_RIGHT] = 0;
    }

    {
        int32_t const left_mm_s  = speed[WHEEL_LEFT] / UM_PER_MM;
        int32_t const right_mm_s = speed[WHEEL_RIGHT] / UM_PER_MM;
        uint32_t const key =
            (uint32_t)steer |
            ((uint32_t)(left_mm_s + DRIVE_OFFSET_MM_S) << DRIVE_LEFT_SH) |
            ((uint32_t)(right_mm_s + DRIVE_OFFSET_MM_S) << DRIVE_RIGHT_SH);

        if (g_nav.last_drive != key)
        {
            g_nav.last_drive = key;
            LOG_NOW("INFO", "line:nav", "steer %d: left %d right %d mm/s\n",
                      (INT)steer, (INT)left_mm_s, (INT)right_mm_s);

            log_rec(OBSTACLE_LOG_STEER, steer, 0U, 0U, left_mm_s, right_mm_s,
                    heading_deci());
        }
    }

    (void)motion_set_speed(speed[WHEEL_LEFT], speed[WHEEL_RIGHT],
                           OBSTACLE_NAV_TTL_US);
}

/*!
 * @brief Stop both wheels for OBSTACLE_NAV_REVERSE_PAUSE_MS before any wheel
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

    if (g_nav.b_pausing && ((now - g_nav.pause_ms) < OBSTACLE_NAV_REVERSE_PAUSE_MS))
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
        LOG_NOW("INFO", "line:nav", "wheel reverses: stop for %d ms first\n",
                  (INT)OBSTACLE_NAV_REVERSE_PAUSE_MS);
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
 * @brief Move to a new bypass phase, and print and log where the car is.
 *
 * @param[in] phase PHASE_*.
 */
static void set_phase (uint8_t phase)
{
    int32_t const heading = side_sign() * heading_deci();

    g_nav.phase = phase;
    LOG_NOW("INFO", "line:nav", "phase %d: side %d mm, ahead %d mm, heading %d "
                    "deg\n",
              (INT)phase, (INT)lateral_mm(), (INT)ahead_mm(),
              (INT)(heading / DECI_PER_DEG));
    log_rec(OBSTACLE_LOG_PHASE, phase, 0U, 0U, lateral_mm(), ahead_mm(), heading);
}

/*/*!
 * @brief BYPASS: pivot away, and report TURNED once the planned angle is
 *        reached. The statechart ends BYPASS when the beam is also clear.
 *
 * @param[in] now Current time, ms.
 */
static void step_pivot (uint32_t now)
{
    int32_t const target_head = side_sign() * (int32_t)g_nav.start_deg * DECI_PER_DEG;
    int32_t       left_um_s   = 0;
    int32_t       right_um_s  = 0;
    uint8_t       steer       = OBSTACLE_STEER_HOLD;
    bool          b_done      = false;

    if (g_nav.b_turned)
    {
        obstacle_plan_wheel_straight(heading_deci(), target_head, 0,
                                     &left_um_s, &right_um_s, &steer);
        note_turned(true, now);
    }
    else
    {
        obstacle_plan_wheel_pivot(heading_deci(), target_head,
                                  &left_um_s, &right_um_s, &steer, &b_done);
        if (b_done)
        {
            note_turned(true, now);
        }
    }

    drive_wheels(steer, left_um_s, right_um_s);
}

/*!
 * @brief Process one side ping in LOOK_PING phase.
 *
 * @param[in] status Packed status word from Buddy 5.
 *
 * @return True when side check cycle is complete.
 */
static bool step_look_ping (uint32_t status)
{
    uint32_t const cls  = obstacle_pack_get(status, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK);
    uint32_t const dist = obstacle_pack_get(status, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK);
    uint32_t const seq  = obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);
    bool           b_side_done = false;

    if (seq != (uint32_t)g_nav.wait_seq)
    {
        g_nav.wait_seq = (uint8_t)seq;
        g_nav.side_ping_count++;

        if ((OBSTACLE_CLASS_OPEN == cls) ||
            ((OBSTACLE_CLASS_VALID == cls) && (dist >= (uint32_t)OBSTACLE_CORNER_CLEAR_MM)))
        {
            g_nav.clear_checks++;
            if (g_nav.clear_checks >= OBSTACLE_PASS_CLEARS)
            {
                g_nav.b_passed = true;
            }
            g_nav.b_have_prev_gap = false;
            b_side_done           = true;
        }
        else if (obstacle_pack_class_is_ranged(cls))
        {
            if ((g_nav.side_agree_count > 0U) &&
                (geom_abs((int32_t)dist - (int32_t)g_nav.last_side_dist_mm) <= (int32_t)OBSTACLE_AGREE_MM))
            {
                int32_t const gap_mm     = (int32_t)dist + (int32_t)OBSTACLE_SONAR_FWD_MM;
                int32_t const target_gap = g_nav.b_wall ? (int32_t)OBSTACLE_WALL_HOLD_MM : (int32_t)OBSTACLE_BOX_GAP_MM;
                int32_t const lean       = obstacle_plan_side_lean_deci(gap_mm, g_nav.prev_gap_mm,
                                                                        g_nav.b_have_prev_gap, target_gap);
                int32_t       base_head  = 0;

                if (g_nav.b_wall && g_nav.b_have_prev_gap)
                {
                    int32_t const trim = obstacle_plan_side_trim_deci(gap_mm, g_nav.prev_gap_mm,
                                                                      g_nav.prev_lean_deci);

                    g_nav.wall_trim_deci += trim;
                    if (g_nav.wall_trim_deci > WALL_TRIM_MAX_DECI)
                    {
                        g_nav.wall_trim_deci = WALL_TRIM_MAX_DECI;
                    }
                    else if (g_nav.wall_trim_deci < -WALL_TRIM_MAX_DECI)
                    {
                        g_nav.wall_trim_deci = -WALL_TRIM_MAX_DECI;
                    }
                    else
                    {
                        /* In range. */
                    }
                }

                base_head = g_nav.b_wall ? ((int32_t)OBSTACLE_WALL_PIVOT_DECI - g_nav.wall_trim_deci) : 0;
                g_nav.along_head_deci = base_head + (side_sign() * lean);
                g_nav.prev_lean_deci  = lean;
                g_nav.prev_gap_mm     = gap_mm;
                g_nav.b_have_prev_gap = true;
                g_nav.clear_checks    = 0U;
                b_side_done           = true;
            }
            else
            {
                g_nav.last_side_dist_mm = (uint16_t)dist;
                g_nav.side_agree_count++;
            }
        }
        else
        {
            /* NO_ECHO, GLITCH or STUCK: wait for next ping. */
        }

        if ((!b_side_done) && (g_nav.side_ping_count >= OBSTACLE_SIDE_PINGS_MAX))
        {
            b_side_done = true;
        }
    }

    return (b_side_done);
}

/*!
 * @brief Handle stop-and-look phases: LOOK_STOP, LOOK_IN, LOOK_PING, LOOK_OUT.
 */
static void reacquire_look_cycle (int32_t cur_head, uint32_t status, uint32_t now,
                                  int64_t mean_um, int32_t * p_left, int32_t * p_right,
                                  uint8_t * p_steer)
{
    bool b_done = false;

    if (OBSTACLE_PHASE_LOOK_STOP == g_nav.phase)
    {
        *p_left  = 0;
        *p_right = 0;
        *p_steer = OBSTACLE_STEER_HOLD;
        if ((now - g_nav.look_stop_ms) >= OBSTACLE_STOP_SETTLE_MS)
        {
            set_phase(OBSTACLE_PHASE_LOOK_IN);
        }
    }
    else if (OBSTACLE_PHASE_LOOK_IN == g_nav.phase)
    {
        int32_t const look_head = g_nav.along_head_deci - (side_sign() * (int32_t)OBSTACLE_WALL_PIVOT_DECI);

        obstacle_plan_wheel_pivot(cur_head, look_head, p_left, p_right, p_steer, &b_done);
        if (b_done)
        {
            set_phase(OBSTACLE_PHASE_LOOK_PING);
            g_nav.side_ping_count   = 0U;
            g_nav.side_agree_count  = 0U;
            g_nav.last_side_dist_mm = 0U;
            *p_left                 = 0;
            *p_right                = 0;
            *p_steer                = OBSTACLE_STEER_HOLD;
        }
    }
    else if (OBSTACLE_PHASE_LOOK_PING == g_nav.phase)
    {
        *p_left  = 0;
        *p_right = 0;
        *p_steer = OBSTACLE_STEER_HOLD;
        if (step_look_ping(status))
        {
            set_phase(OBSTACLE_PHASE_LOOK_OUT);
        }
    }
    else if (OBSTACLE_PHASE_LOOK_OUT == g_nav.phase)
    {
        obstacle_plan_wheel_pivot(cur_head, g_nav.along_head_deci, p_left, p_right, p_steer, &b_done);
        if (b_done)
        {
            g_nav.leg_base_um = mean_um;
            if (g_nav.b_passed && (!g_nav.b_wall))
            {
                set_phase(OBSTACLE_PHASE_EXTRA);
            }
            else
            {
                set_phase(OBSTACLE_PHASE_ALONG);
            }
        }
    }
    else
    {
        /* Not in look cycle. */
    }
}

/*!
 * @brief Handle return phases: EXTRA, RETURN_PIVOT, RETURN_DRIVE.
 */
static void reacquire_drive_and_return (int32_t cur_head, int64_t mean_um,
                                        int32_t * p_left, int32_t * p_right,
                                        uint8_t * p_steer)
{
    int32_t const leg_dist = (int32_t)((mean_um - g_nav.leg_base_um) / UM_PER_MM);
    bool          b_done   = false;

    if (OBSTACLE_PHASE_EXTRA == g_nav.phase)
    {
        obstacle_plan_wheel_straight(cur_head, g_nav.along_head_deci,
                                     (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                     p_left, p_right, p_steer);
        if (leg_dist >= (int32_t)OBSTACLE_PASS_EXTRA_MM)
        {
            set_phase(OBSTACLE_PHASE_RETURN_PIVOT);
        }
    }
    else if (OBSTACLE_PHASE_RETURN_PIVOT == g_nav.phase)
    {
        int32_t const return_head = -side_sign() * (int32_t)OBSTACLE_RETURN_DECI;

        obstacle_plan_wheel_pivot(cur_head, return_head, p_left, p_right, p_steer, &b_done);
        if (b_done)
        {
            set_phase(OBSTACLE_PHASE_RETURN_DRIVE);
        }
    }
    else if (OBSTACLE_PHASE_RETURN_DRIVE == g_nav.phase)
    {
        int32_t const return_head = -side_sign() * (int32_t)OBSTACLE_RETURN_DECI;

        obstacle_plan_wheel_straight(cur_head, return_head,
                                     (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                     p_left, p_right, p_steer);
    }
    else
    {
        /* Not in return phases. */
    }
}

/*!
 * @brief REACQUIRE: execute stop-and-look bypass phases and drive wheels.
 *
 * @param[in] status Packed Buddy 5 status word (latest ping).
 * @param[in] now    Current time, ms.
 */
static void drive_reacquire (uint32_t status, uint32_t now)
{
    int32_t const cur_head   = heading_deci();
    int64_t const mean_um    = (g_nav.wheel_um[WHEEL_LEFT] + g_nav.wheel_um[WHEEL_RIGHT]) / 2;
    int32_t       left_um_s  = 0;
    int32_t       right_um_s = 0;
    uint8_t       steer      = OBSTACLE_STEER_HOLD;
    bool          b_done     = false;

    if (OBSTACLE_PHASE_TURN == g_nav.phase)
    {
        int32_t const target_head = side_sign() * (int32_t)g_nav.start_deg * DECI_PER_DEG;

        obstacle_plan_wheel_pivot(cur_head, target_head, &left_um_s, &right_um_s, &steer, &b_done);
        if (b_done)
        {
            if (g_nav.b_wall)
            {
                set_phase(OBSTACLE_PHASE_ALONG);
                g_nav.along_head_deci = (int32_t)OBSTACLE_WALL_PIVOT_DECI;
                g_nav.leg_base_um     = mean_um;
                g_nav.b_ir_armed      = true;
                g_nav.b_ir_cleared    = false;
            }
            else
            {
                set_phase(OBSTACLE_PHASE_OUT);
            }
        }
    }
    else if (OBSTACLE_PHASE_OUT == g_nav.phase)
    {
        int32_t const target_head = side_sign() * (int32_t)g_nav.start_deg * DECI_PER_DEG;

        obstacle_plan_wheel_straight(cur_head, target_head, (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                     &left_um_s, &right_um_s, &steer);
        if (lateral_mm() >= g_nav.need_mm)
        {
            set_phase(OBSTACLE_PHASE_ALIGN);
        }
    }
    else if (OBSTACLE_PHASE_ALIGN == g_nav.phase)
    {
        obstacle_plan_wheel_pivot(cur_head, 0, &left_um_s, &right_um_s, &steer, &b_done);
        if (b_done)
        {
            set_phase(OBSTACLE_PHASE_ALONG);
            g_nav.along_head_deci = 0;
            g_nav.leg_base_um     = mean_um;
            g_nav.b_ir_armed      = true;
            g_nav.b_ir_cleared    = false;
        }
    }
    else if (OBSTACLE_PHASE_ALONG == g_nav.phase)
    {
        int32_t const leg_dist = (int32_t)((mean_um - g_nav.leg_base_um) / UM_PER_MM);

        obstacle_plan_wheel_straight(cur_head, g_nav.along_head_deci,
                                     (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                     &left_um_s, &right_um_s, &steer);
        if (leg_dist >= (int32_t)OBSTACLE_SIDE_LOOK_MM)
        {
            set_phase(OBSTACLE_PHASE_LOOK_STOP);
            g_nav.look_stop_ms = now;
            left_um_s          = 0;
            right_um_s         = 0;
            steer              = OBSTACLE_STEER_HOLD;
        }
    }
    else if ((g_nav.phase >= OBSTACLE_PHASE_LOOK_STOP) && (g_nav.phase <= OBSTACLE_PHASE_LOOK_OUT))
    {
        reacquire_look_cycle(cur_head, status, now, mean_um, &left_um_s, &right_um_s, &steer);
    }
    else if ((g_nav.phase >= OBSTACLE_PHASE_EXTRA) && (g_nav.phase <= OBSTACLE_PHASE_RETURN_DRIVE))
    {
        reacquire_drive_and_return(cur_head, mean_um, &left_um_s, &right_um_s, &steer);
    }
    else
    {
        /* Unknown phase. */
    }

    drive_wheels(steer, left_um_s, right_um_s);
}

/*!
 * @brief Note when both IR sensors have been off the line since the car
 *        reached its offset (end of OUT); only after that may an IR line
 *        contact end the bypass.
 *
 * @param[in] b_line_seen An IR sensor sees the line.
 */
static void update_ir_gate (bool b_line_seen)
{
    if (g_nav.b_ir_armed && (!g_nav.b_ir_cleared) && (!b_line_seen))
    {
        g_nav.b_ir_cleared = true;
        LOG_NOW("INFO", "line:nav", "IR clear of the line: the next line "
                        "contact ends the bypass\n");
    }
}

/*!
 * @brief Tell whether Buddy 5's latest reading is an obstacle within the
 *        detection distance, so the line follower must not drive on.
 *
 * @param[in] status Packed Buddy 5 status word.
 *
 * @return True when the latest ping is a hit: VALID at or below
 *         OBSTACLE_DETECT_MM, or TOO_CLOSE. The statechart confirms an
 *         obstacle with the same hit rule, 2 of the last 3 pings.
 */
static bool is_obstacle_close (uint32_t status)
{
    uint32_t const cls = obstacle_pack_get(status, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK);
    uint32_t const dist_mm = obstacle_pack_get(status, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK);

    return (obstacle_pack_class_is_ranged(cls) && (dist_mm <= OBSTACLE_DETECT_MM));
}

/*!
 * @brief Print and log each Buddy 5 state change. Entering BYPASS also
 *        starts the dead reckoning.
 *
 * NOTE: States: 0 STOPPED, 1 MONITOR, 2 WAIT_STOP, 3 SCANNING, 4 PLAN,
 * 5 BYPASS, 6 REACQUIRE.
 *
 * @param[in] status Packed Buddy 5 status word.
 */
static void report_state (uint32_t status)
{
    uint32_t const state = obstacle_pack_get(status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);

    if (g_nav.last_state != state)
    {
        uint32_t const cause =
            obstacle_pack_get(status, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK);

        /* Casts are safe: the state field is 3 bits, the cause 4 bits. */
        g_nav.last_state = (uint8_t)state;
        g_nav.last_drive = NAV_DRIVE_UNSEEN;
        LOG_NOW("INFO", "line:nav", "state %d cause %d\n", (INT)state, (INT)cause);
        log_rec(OBSTACLE_LOG_STATE, (uint8_t)state, (uint8_t)cause,
                (uint8_t)obstacle_get_faults(), 0, 0, 0);

        if (OBSTACLE_STATE_BYPASS == state)
        {
            reset_pose();
        }

        if (OBSTACLE_STATE_REACQUIRE == state)
        {
            int64_t const mean_um =
                (g_nav.wheel_um[WHEEL_LEFT] + g_nav.wheel_um[WHEEL_RIGHT]) / 2;

            g_nav.clear_checks    = 0U;
            g_nav.b_have_prev_gap = false;
            g_nav.b_passed        = false;

            if (g_nav.b_wall)
            {
                set_phase(OBSTACLE_PHASE_ALONG);
                g_nav.along_head_deci = (int32_t)OBSTACLE_WALL_PIVOT_DECI;
                g_nav.leg_base_um     = mean_um;
                g_nav.b_ir_armed      = true;
                g_nav.b_ir_cleared    = false;
                g_nav.prev_gap_mm     = (int32_t)OBSTACLE_WALL_HOLD_MM;
                g_nav.b_have_prev_gap = true;
                g_nav.prev_lean_deci  = 0;
                g_nav.wall_trim_deci  = 0;
            }
            else
            {
                set_phase(OBSTACLE_PHASE_OUT);
            }
        }

        if ((OBSTACLE_STATE_BYPASS == state) || (OBSTACLE_STATE_REACQUIRE == state) ||
            (OBSTACLE_STATE_MONITOR == state))
        {
            /* The servo turns on entering these states (back to 0 after
             * the scan, to 45 degrees, or back to 0 after the bypass): wait
             * for the next ping, which comes only after it has settled.
             * Cast is safe: the sequence field is 6 bits. */
            g_nav.b_servo_wait = true;
            g_nav.wait_seq =
                (uint8_t)obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);
            g_nav.wait_ms = now_ms();
        }
    }
}

/*!
 * @brief Tell whether the servo may still be turning after a state change,
 *        so the wheels must stand still.
 *
 * NOTE: A servo swing while a motor drives disturbs the wheel encoders
 * (seen on the car: false backward pulses and an invalid step on a wheel
 * told to stand still, which latches the motion service's FEEDBACK fault).
 * With both wheels at 0 the motion service is idle and does not check the
 * encoders. The wait ends with the first new ping (the scan task pings
 * only after the servo has settled) or after OBSTACLE_NAV_SERVO_WAIT_MS.
 *
 * @param[in] status Packed Buddy 5 status word.
 * @param[in] now    Current time, ms.
 *
 * @return True while the car must stand still for the servo.
 */
static bool is_servo_moving (uint32_t status, uint32_t now)
{
    uint32_t const seq = obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);

    if (g_nav.b_servo_wait && ((seq != g_nav.wait_seq) ||
                               ((now - g_nav.wait_ms) >= OBSTACLE_NAV_SERVO_WAIT_MS)))
    {
        g_nav.b_servo_wait = false;
    }

    return (g_nav.b_servo_wait);
}

/*!
 * @brief Keep the accepted plan: the start angle, the sideways offset the
 *        car needs (closest x tan(start angle), the planner's own offset).
 *        The closest reading comes from the scan profile, as in
 *        scan_algo.c.
 *
 * @param[in] angle  Start angle from SCAN_RESULT, degrees.
 * @param[in] b_wall Wall plan flag from SCAN_RESULT.
 */
static void keep_plan (uint32_t angle, bool b_wall)
{
    uint16_t profile[OBSTACLE_SLOT_COUNT];
    uint32_t slot    = 0U;
    int32_t  closest = g_nav.obst_mm;

    if (E_OK == obstacle_copy_profile(profile))
    {
        for (slot = 0U; slot < OBSTACLE_SLOT_COUNT; slot++)
        {
            /* Cast is safe: slot is 0..24. */
            int32_t const  deci  = scan_algo_slot_deci((uint8_t)slot);
            uint16_t const value = profile[slot];
            int32_t const  dist  = (int32_t)obstacle_pack_slot_dist(value);

            if ((deci >= -OBSTACLE_PATH_HALF_DECI) && (deci <= OBSTACLE_PATH_HALF_DECI) &&
                (obstacle_pack_slot_is_measured(value)) &&
                (obstacle_pack_class_is_ranged(obstacle_pack_slot_class(value))) &&
                (dist <= (int32_t)OBSTACLE_REGION_MAX_MM) && (dist < closest))
            {
                closest = dist;
            }
        }
    }

    /* Casts are safe: angle is 0..60 and closest is 0..4095. */
    g_nav.start_deg = (uint8_t)angle;
    g_nav.b_wall    = b_wall;
    g_nav.need_mm =
        (closest * obstacle_plan_tan_q14((int32_t)angle * DECI_PER_DEG)) / Q14_ONE;
    LOG_NOW("INFO", "line:nav", "plan: obstacle %d mm, start %d deg, side "
                    "offset %d mm, %s\n",
              (INT)closest, (INT)angle, (INT)g_nav.need_mm,
              b_wall ? "wall (follow in parallel)" : "box (out and back)");
}

/*!
 * @brief Print why each side was or was not usable, for tethered debugging.
 *        The scan is replayed through
 *        scan_algo and obstacle_plan from the published profile (the same slots
 *        in the same order, so the result is the one Buddy 5 found).
 *
 * @param[in] p_profile OBSTACLE_SLOT_COUNT packed slots.
 */
static void explain_scan (uint16_t const * p_profile)
{
    static scan_algo_ctx_t g_replay;
    obstacle_plan_cfg_t          cfg;
    obstacle_plan_result_t       result;
    uint32_t               guard  = 0U;
    bool                   b_done = false;

    /* The 37 readings are not printed here: the flight log prints each
     * stored profile (flight_log.c) and the dashboard plots it. */

    /* Replay: feed each requested slot's stored value twice, so the
     * agreement rule settles it exactly as before. */
    scan_algo_start(&g_replay);
    while ((!b_done) && (guard < OBSTACLE_SLOT_COUNT))
    {
        uint8_t const target = scan_algo_target(&g_replay);

        if (SCAN_ALGO_NO_SLOT == target)
        {
            b_done = true;
        }
        else
        {
            uint16_t const value = p_profile[target];
            uint8_t const  cls   = obstacle_pack_slot_class(value);
            uint16_t const dist  = obstacle_pack_slot_dist(value);

            (void)scan_algo_feed(&g_replay, cls, dist);
            b_done = scan_algo_feed(&g_replay, cls, dist);
        }

        guard++;
    }

    obstacle_plan_cfg_default(&cfg);
    obstacle_plan_build(&g_replay, &cfg, &result);

    /* Edge states: 0 none, 1 bounded, 2 unbounded (reaches +-60 deg).
     * need 65535 = side blocked (unbounded, too far out, or something in
     * the way beyond the edge). */
    LOG_NOW("INFO", "line:nav", "scan: closest %d mm, region %d..%d deg | LEFT "
                    "edge %d last hit %d deg need %d mm | RIGHT edge %d last "
                    "hit %d deg need %d mm\n",
              (INT)g_replay.closest_mm,
              (INT)(scan_algo_slot_deci(g_replay.region_lo) / DECI_PER_DEG),
              (INT)(scan_algo_slot_deci(g_replay.region_hi) / DECI_PER_DEG),
              (INT)g_replay.edge_state[SCAN_ALGO_LEFT],
              (INT)(scan_algo_slot_deci(g_replay.edge_hit[SCAN_ALGO_LEFT]) /
                    DECI_PER_DEG),
              (INT)result.need_mm[OBSTACLE_SIDE_LEFT],
              (INT)g_replay.edge_state[SCAN_ALGO_RIGHT],
              (INT)(scan_algo_slot_deci(g_replay.edge_hit[SCAN_ALGO_RIGHT]) /
                    DECI_PER_DEG),
              (INT)result.need_mm[OBSTACLE_SIDE_RIGHT]);
}

/*!
 * @brief React to one Buddy 5 event.
 *
 * @param[in] evt Packed event word.
 * @param[in] now Current time, ms.
 */
static void handle_event (uint32_t evt, uint32_t now)
{
    uint32_t const type   = obstacle_pack_get(evt, OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK);
    uint32_t const req_id = obstacle_pack_get(evt, OBSTACLE_EV_ID_SH, OBSTACLE_EV_ID_MK);
    uint32_t const status = obstacle_pack_get(evt, OBSTACLE_EV_STATUS_SH, OBSTACLE_EV_STATUS_MK);
    bool const b_go = (OBSTACLE_RESULT_OK == status) || (OBSTACLE_RESULT_CLEAR == status);

    if (OBSTACLE_EVT_OBSTACLE == type)
    {
        uint32_t const dist_mm = obstacle_pack_get(evt, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK);

        LOG_NOW("INFO", "line:nav", "OBSTACLE at %d mm: holding\n", (INT)dist_mm);
        log_rec(OBSTACLE_LOG_OBST, 0U, 0U, 0U, (int32_t)dist_mm, 0, 0);
        hold_car(true);
        g_nav.hold_ms        = now;
        g_nav.req_id         = g_nav.next_id;
        g_nav.b_scan_pending = true;
        g_nav.obst_mm        = (int32_t)dist_mm;

        /* Cast is safe: the id is masked to 5 bits. */
        g_nav.next_id = (uint8_t)((g_nav.next_id + 1U) & OBSTACLE_ID_MAX);
    }
    else if (OBSTACLE_EVT_FAULT == type)
    {
        uint32_t const cause = obstacle_pack_get(evt, OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK);
        uint32_t const faults =
            obstacle_pack_get(evt, OBSTACLE_EV_FAULTS_SH, OBSTACLE_EV_FAULTS_MK);

        LOG_NOW("ERROR", "line:nav", "FAULT cause=%d: holding, %s\n", (INT)cause,
                  obstacle_pack_cause_may_auto_restart(cause) ? "retry in 3 s"
                                                              : "needs RESET and START");

        /* Casts are safe: the cause is 4 bits. The flight log keeps the low 8
         * fault bits. */
        log_rec(OBSTACLE_LOG_FAULT, (uint8_t)cause, 0U, (uint8_t)faults, 0, 0, 0);
        hold_car(true);
        g_nav.b_scan_pending = false;
        g_nav.retry_ms       = now;
    }
    else if (OBSTACLE_EVT_SCAN_RESULT == type)
    {
        uint16_t       profile[OBSTACLE_SLOT_COUNT];
        uint32_t const side  = obstacle_pack_get(evt, OBSTACLE_EV_SIDE_SH, OBSTACLE_EV_SIDE_MK);
        uint32_t const width = obstacle_pack_get(evt, OBSTACLE_EV_WIDTH_SH, OBSTACLE_EV_WIDTH_MK);
        uint32_t const angle = obstacle_pack_get(evt, OBSTACLE_EV_ANGLE_SH, OBSTACLE_EV_ANGLE_MK);
        bool const     b_wall =
            (0U != obstacle_pack_get(evt, OBSTACLE_EV_WALL_SH, OBSTACLE_EV_WALL_MK));
        uint32_t       index = 0U;

        LOG_NOW("INFO", "line:nav", "SCAN_RESULT status=%d side=%d width=%d mm "
                        "start=%d deg\n",
                  (INT)status, (INT)side, (INT)width, (INT)angle);

        /* Casts are safe: req_id is 5 bits and angle 6 bits wide. */
        if (E_OK == obstacle_copy_profile(profile))
        {
            index = flight_log_profile((uint8_t)req_id, profile);
            explain_scan(profile);
        }
        log_rec(OBSTACLE_LOG_RESULT, (uint8_t)status, (uint8_t)side, (uint8_t)angle,
                (int32_t)width, (int32_t)index, 0);

        if ((g_nav.req_id == req_id) && b_go)
        {
            /* Cast is safe: the side field is 1 bit wide. */
            g_nav.side = (uint8_t)side;
            keep_plan(angle, b_wall);
            post(OBSTACLE_CMD_ACCEPT, g_nav.req_id);
        }
    }
    else
    {
        /* STEER words are read with obstacle_get_steer(). */
    }
}

/*!
 * @brief In REACQUIRE, tell Buddy 5 when an IR sensor sees the line, once
 *        the car is curving round the obstacle (HUG) and both IR sensors
 *        have been off the line since the end of OUT.
 *
 * @param[in] b_line_seen The car is in HUG, the IR gate is open and an IR
 *                        sensor sees the line.
 * @param[in] now         Current time, ms.
 */
static void look_for_line (bool b_line_seen, uint32_t now)
{
    if (b_line_seen && ((now - g_nav.line_ms) >= OBSTACLE_NAV_LINE_REPEAT_MS))
    {
        g_nav.line_ms = now;
        post(OBSTACLE_CMD_LINE_FOUND, g_nav.req_id);
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
static void report_fault (motion_status_t const * p_status,
                          motion_feedback_t const * p_feedback)
{
    uint32_t wheel = 0U;

    LOG_NOW("ERROR", "line:nav", "MOTION FAULT %d: state %d, stop %d, "
                    "hold %d\n",
              (INT)p_status->fault, (INT)p_status->state,
              (INT)p_status->stop_requested, (INT)p_status->hold_mask);

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        int32_t const speed = (wheel == WHEEL_LEFT)
                                  ? p_feedback->left_speed_um_s
                                  : p_feedback->right_speed_um_s;

        LOG_NOW("ERROR", "line:nav", "  wheel %d: speed %d um/s, "
                        "speed_valid %d, odo_valid %d\n",
                  (INT)wheel, (INT)speed,
                  (INT)p_feedback->speed_valid[wheel],
                  (INT)p_feedback->odometry_valid[wheel]);

        log_rec(OBSTACLE_LOG_MFAULT, (uint8_t)wheel,
                (uint8_t)p_feedback->speed_valid[wheel],
                (uint8_t)p_feedback->odometry_valid[wheel], 0,
                speed / UM_PER_MM, 0);
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
    motion_status_t   status;
    motion_feedback_t feedback;

    if (E_OK == motion_status(&status))
    {
        uint32_t const word = ((uint32_t)status.state << 16U) |
                              ((uint32_t)status.fault << 8U) |
                              status.hold_mask;
        bool const b_changed = (word != g_nav.motion_word);

        if (b_changed)
        {
            g_nav.motion_word = word;
            LOG_NOW("INFO", "line:nav", "motion state=%d fault=%d hold=%d\n",
                      (INT)status.state, (INT)status.fault,
                      (INT)status.hold_mask);
            log_rec(OBSTACLE_LOG_MOTION, status.state, status.fault,
                    (uint8_t)status.hold_mask, 0, 0, 0);
        }

        if (b_changed && (MOTION_STATE_FAULT == status.state))
        {
            if (E_OK == motion_feedback_snapshot(&feedback))
            {
                report_fault(&status, &feedback);
            }
        }
    }

    if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
    {
        uint32_t wheel = 0U;

        for (wheel = 0U; wheel < WHEELS; wheel++)
        {
            g_nav.wheel_um[wheel] = true_wheel_um(&feedback, wheel);

            /* Cast is safe: a run is far shorter than 2147 km. */
            g_nav.wheel_mm[wheel] =
                (int32_t)(g_nav.wheel_um[wheel] / UM_PER_MM);
        }
    }
}

/*!
 * @brief Log a TRACE record every OBSTACLE_LOG_TRACE_MS while Buddy 5 drives
 *        the car (BYPASS, REACQUIRE), and every OBSTACLE_LOG_IDLE_TRACE_MS
 *        otherwise.
 *
 * @param[in] status  Packed Buddy 5 status word.
 * @param[in] b_left  Left IR sensor sees the line.
 * @param[in] b_right Right IR sensor sees the line.
 * @param[in] now     Current time, ms.
 */
static void trace (uint32_t status, bool b_left, bool b_right, uint32_t now)
{
    uint32_t const state = obstacle_pack_get(status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);
    bool const     b_driving =
        (OBSTACLE_STATE_BYPASS == state) || (OBSTACLE_STATE_REACQUIRE == state);
    uint32_t const period_ms =
        b_driving ? OBSTACLE_LOG_TRACE_MS : OBSTACLE_LOG_IDLE_TRACE_MS;

    if ((now - g_nav.trace_ms) >= period_ms)
    {
        uint32_t const steer =
            obstacle_pack_get(obstacle_get_steer(), OBSTACLE_EV_STEER_SH, OBSTACLE_EV_STEER_MK);
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
        log_rec(OBSTACLE_LOG_TRACE, (uint8_t)state,
                (uint8_t)obstacle_pack_get(status, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK),
                (uint8_t)bits,
                (int32_t)obstacle_pack_get(status, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK),
                g_nav.wheel_mm[WHEEL_LEFT], g_nav.wheel_mm[WHEEL_RIGHT]);
    }
}

/*!
 * @brief Emergency-stop link for Buddy 5: hold the car at once.
 *
 * @return The result of motion_set_hold().
 */
static ER nav_stop (void)
{
    return (motion_set_hold(MOTION_HOLD_OBSTACLE, true));
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
    motion_feedback_t feedback;

    if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
    {
        int64_t const sum_um =
            true_wheel_um(&feedback, WHEEL_LEFT) +
            true_wheel_um(&feedback, WHEEL_RIGHT);
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
 *        (OBSTACLE_ENCODERS_CROSSED).
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
static int64_t true_wheel_um (motion_feedback_t const * p_feedback,
                              uint32_t wheel)
{
    int64_t travel_um = 0;

#if (OBSTACLE_ENCODERS_CROSSED)
    travel_um = (wheel == WHEEL_LEFT) ? p_feedback->right_displacement_um
                                      : p_feedback->left_displacement_um;
    travel_um = -travel_um;
#else
    travel_um = (wheel == WHEEL_LEFT) ? p_feedback->left_displacement_um
                                      : p_feedback->right_displacement_um;
#endif

    return (travel_um);
}

/*!
 * @brief Public facade: start Buddy 5 with the motion links.
 *
 * @return The result of obstacle_nav_init().
 */
ER obstacle_line_init (void)
{
    return (obstacle_nav_init());
}

/*!
 * @brief Public facade: one handshake pass for the line task.
 *
 * @param[in] b_line_left  Left IR sensor sees the line.
 * @param[in] b_line_right Right IR sensor sees the line.
 *
 * @return True when Buddy 5 owns the motors this pass.
 */
bool obstacle_line_step (bool b_line_left, bool b_line_right)
{
    return (obstacle_nav_step(b_line_left, b_line_right));
}

/*** end of file ***/

