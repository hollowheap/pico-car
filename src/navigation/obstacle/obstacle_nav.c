/** @file obstacle_nav.c
 *
 * @brief Buddy 5 Phase 2 glue between the obstacle statechart, the line
 *        task in main.c and the motion service.
 *
 * What the line task does with Buddy 5:
 *   - OBSTACLE      hold the car, wait OBSTACLE_NAV_BRAKE_MS, send SCAN_REQ(id).
 *   - SCAN_RESULT   OK or CLEAR: send ACCEPT(id), and keep the plan: the
 *                   side, the start angle and the closest reading.
 *   - BYPASS        pivot away from the obstacle until Buddy 5 sees the way
 *                   ahead is clear AND the planned start angle is reached;
 *                   this task reports the angle with TURNED (same opcode as
 *                   LINE_FOUND).
 *   - REACQUIRE     the Bug2 face follow (follow.c): follow the obstacle face
 *                   on the plan side while moving, ask for the servo angle,
 *                   and send LINE_FOUND once the car is back on the tape past
 *                   the hit point and faces the original heading.
 *   - FAULT         hold the car until the supervisor accepts operator START.
 *   - MONITOR       release the hold and let the line follower drive,
 *                   except while Buddy 5's latest reading is a VALID echo
 *                   within the detection distance.
 *
 * Position frame (dead reckoning from the wheel encoders): x is sideways,
 * positive to the left of the original line; y is along the line; both
 * are for the point midway between the wheels and start at 0 when the
 * stationary scan begins. The heading is worked out from the total wheel difference, so
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
#include "motion_link.h"
#include "obstacle_config.h"
#include "flight_log.h"
#include "follow.h"
#include "nav_core.h"
#include "obstacle_nav.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "obstacle_internal.h"
#include "comms.h"

/* Body target in the aim word published by comms.c. */
#define AIM_BODY_SHIFT (16U)
#include "scan_algo.h"

/* Micrometres per millimetre. */
#define UM_PER_MM (1000)
#define MS_PER_S  (1000)

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

/* Navigation state; only the line task touches it. NOTE: Booleans are b_
 * members of a g_ struct, which satisfies both BARR-C 7.1.m/7.1.o and the
 * team .clang-tidy. */
typedef struct
{
    nav_core_t core; /* pose, servo wait, IR gate, ping freshness (nav_core.c) */
    int64_t  wheel_um[WHEELS]; /* 8-byte fields first: no padding */
    bool     b_enabled;
    bool     b_scan_pending;
    bool     b_turned;
    bool     b_pausing;
    bool     b_wall;
    bool     b_scan_turning; /* the body turns to a scan window */
    bool     b_follow_turning; /* the face follow turns the body on the spot */
    uint8_t  req_id;
    uint8_t  next_id;
    uint8_t  last_state;
    uint8_t  start_deg;
    uint8_t  side;
    uint32_t pause_ms;
    uint32_t last_drive;
    uint32_t hold_ms;
    uint32_t trace_ms;
    uint32_t motion_word;
    int16_t  need_mm;  /* side offset of the plan, for the log      */
    int16_t  hit_mm;   /* closest reading of the accepted plan      */
    int32_t  obst_mm;
    int32_t  wheel_mm[WHEELS];
    int32_t  last_cmd[WHEELS];
    uint32_t bump_ms;  /* time of the last bumper check               */
    follow_t follow;
    follow_bump_t bump;
} nav_state_t;


static nav_state_t g_nav = {
    .b_enabled          = false,
    .b_scan_pending     = false,
    .b_turned           = false,
    .b_pausing          = false,
    .b_wall             = false,
    .b_scan_turning     = false,
    .b_follow_turning   = false,
    .req_id             = 0U,
    .next_id            = 0U,
    .last_state         = NAV_UNSEEN,
    .start_deg          = 0U,
    .side               = OBSTACLE_SIDE_LEFT,
    .pause_ms           = 0U,
    .last_drive         = NAV_DRIVE_UNSEEN,
    .hold_ms            = 0U,
    .trace_ms           = 0U,
    .motion_word        = UINT32_MAX,
    .need_mm            = 0,
    .obst_mm            = 0,
    .hit_mm             = 0,
    .wheel_mm           = {0, 0},
    .last_cmd           = {0, 0},
    .wheel_um           = {0, 0},
    .bump_ms            = 0U,
};

static uint32_t now_ms (void);
static int16_t  clamp_i16 (int32_t value);
static void log_rec (uint8_t kind, uint8_t val_a, uint8_t val_b, uint8_t val_c,
                     int32_t val_d, int32_t val_e, int32_t val_f);
static void post (uint32_t type, uint8_t req_id);
static void hold_car (bool b_held);
static int32_t  side_sign (void);
static int32_t  heading_deci (void);
static void     reset_pose (void);
static void     step_scan_body (uint32_t status, uint32_t now);
static void     update_pivoting (uint32_t state);
static void     update_pose (void);
static void     note_turned (bool b_turned, uint32_t now);
static void     drive_wheels (uint8_t steer, int32_t left_um_s,
                              int32_t right_um_s, bool b_scan_pivot);
static bool     is_reverse_pause (int32_t const p_speed[WHEELS]);
static void     step_pivot (uint32_t now);
static void step_reacquire (uint32_t status, bool b_line_seen, uint32_t now);
static void     give_up (uint8_t reason, uint32_t now);
static bool     is_obstacle_close (uint32_t status);
static bool     is_servo_moving (uint32_t status, uint32_t now);
static void     report_state (uint32_t status);
static void     keep_plan (uint32_t angle, bool b_wall);
static void     explain_scan (uint16_t const * p_profile);
static void     handle_event (uint32_t evt, uint32_t now);
static void     look_for_line (bool b_line_seen, uint32_t now);
static void     update_motion (void);
static void    report_fault (motion_status_t const * p_status,
                             motion_feedback_t const * p_feedback);
static void    trace (uint32_t status, bool b_left, bool b_right, uint32_t now);
static ER      nav_stop (void);

/*!
 * @brief Initialize Buddy 5 with the motion links and leave it parked.
 *
 * Call once from the line task on processor 1, after gpio_irq_init(),
 * motion_service_init() and ir_capture_init(). If it fails, obstacle_line_step()
 * only services the flight recorder, so line following works without
 * Buddy 5 and the failure can still be read back.
 *
 * @return The result of obstacle_init().
 */
ER obstacle_nav_init (obstacle_distance_fn_t p_get_distance)
{
    obstacle_links_t const links = {0, nav_stop, p_get_distance};
    ER const               result = obstacle_init(&links);

    log_rec(OBSTACLE_LOG_BOOT, 0U, 0U, 0U, (int32_t)result, 0, 0);

    if (E_OK == result)
    {
        g_nav.b_enabled = true;
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
bool obstacle_line_step (bool b_line_left, bool b_line_right)
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
        while (E_OK == obstacle_comms_get_event(&evt, TMO_POL))
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
                b_in_charge = true;
            break;

            case OBSTACLE_STATE_SCANNING:
            case OBSTACLE_STATE_PLAN:
                /* Turn the body to the scan window the statechart asks for
                 *. It is 0 after the scan, so PLAN turns back. */
                update_pose();
                step_scan_body(status, now);
                b_in_charge = true;
            break;

            default:
                /* WAIT_STOP: the car stands still. */
                hold_car(true);
                b_in_charge = true;
            break;
        }
    }

    update_pivoting(state);

    return (b_in_charge);
}

/*!
 * @brief Tell the scan task whether the body is turning: a scan pivot, or a
 *        spin of the face follow in REACQUIRE.
 *
 * @param[in] state Current Buddy 5 state.
 */
static void update_pivoting (uint32_t state)
{
    bool const b_scan = g_nav.b_scan_turning &&
                        ((OBSTACLE_STATE_SCANNING == state) || (OBSTACLE_STATE_PLAN == state));
    bool const b_look = (OBSTACLE_STATE_REACQUIRE == state) && g_nav.b_follow_turning;

    obstacle_comms_set_pivoting(b_scan || b_look);
}

/*!
 * @brief SCANNING and PLAN: pivot the body to the target of the scan. The car
 *        stands still once it is there, and while the servo swings.
 *
 * @param[in] status Packed Buddy 5 status word.
 * @param[in] now    Current time, ms.
 */
static void step_scan_body (uint32_t status, uint32_t now)
{
    int32_t const target = (int32_t)(int16_t)(uint16_t)(obstacle_comms_aim() >> AIM_BODY_SHIFT);
    int32_t       left   = 0;
    int32_t       right  = 0;
    uint8_t       steer  = OBSTACLE_STEER_HOLD;
    bool          b_done = false;

    obstacle_plan_wheel_pivot(heading_deci(), target, &left, &right, &steer, &b_done);
    g_nav.b_scan_turning = !b_done;
    if (b_done || is_servo_moving(status, now))
    {
        /* Also stand still while the servo swings: a swing while a motor
         * drives disturbs the wheel encoders and can latch a motion
         * FEEDBACK fault. */
        hold_car(true);
    }
    else
    {
        hold_car(false);
        drive_wheels(steer, left, right, true);
    }
}

/*!
 * @brief One line-task pass in REACQUIRE: stand still while the servo
 *        settles, otherwise run one step of the Bug2 face follow and
 *        hand the line back once it is on the tape past the hit point.
 *
 * @param[in] status      Packed Buddy 5 status word.
 * @param[in] b_line_seen An IR sensor sees the line.
 * @param[in] now         Current time, ms.
 */
static void step_reacquire (uint32_t status, bool b_line_seen, uint32_t now)
{
    uint32_t const dt_ms = now - g_nav.bump_ms;
    /* Measured speeds from this pass's travel; obstacle_wheel_um already undid
     * any crossed encoder plugs. Casts are safe: one pass moves the car far
     * less than 2 km. */
    int32_t const meas_um_s[WHEELS] = {
        (dt_ms > 0U) ? (int32_t)(((g_nav.wheel_um[WHEEL_LEFT] - g_nav.core.prev_um[WHEEL_LEFT]) * MS_PER_S) / (int64_t)dt_ms) : 0,
        (dt_ms > 0U) ? (int32_t)(((g_nav.wheel_um[WHEEL_RIGHT] - g_nav.core.prev_um[WHEEL_RIGHT]) * MS_PER_S) / (int64_t)dt_ms) : 0,
    };

    g_nav.bump_ms = now;
    update_pose();

    if (is_servo_moving(status, now))
    {
        hold_car(true);
    }
    else
    {
        follow_in_t  in;
        follow_out_t out;

        hold_car(false);
        if (nav_core_follow_input(&g_nav.core, obstacle_comms_ping(), obstacle_comms_look(),
                                  g_nav.wheel_um, b_line_seen, &in))
        {
            LOG_NOW("INFO", "line:nav", "IR clear of the line: the next line "
                            "contact ends the bypass\n");
        }

        {
            uint8_t const before = (uint8_t)g_nav.follow.phase;

            follow_step(&g_nav.follow, &in, &out);
            if (before != (uint8_t)g_nav.follow.phase)
            {
                /* Casts are safe: the phase field is 4 bits. */
                log_rec(OBSTACLE_LOG_PHASE, (uint8_t)g_nav.follow.phase, 0U, 0U, in.x_mm, in.y_mm,
                        in.head_deci);
            }
        }
        g_nav.b_follow_turning = out.b_turning;
        obstacle_comms_set_look(out.servo_deci);
        if ((FOLLOW_RUN == out.result) &&
            follow_bump_update(&g_nav.bump, g_nav.last_cmd, meas_um_s,
                               (dt_ms > (uint32_t)MS_PER_S) ? (uint32_t)MS_PER_S : dt_ms))
        {
            /* A driven wheel does not turn: the car touches something. */
            out.result = FOLLOW_GAVE_UP;
            out.reason = FOLLOW_REASON_WHEEL_STALL;
        }
        if (FOLLOW_GAVE_UP == out.result)
        {
            hold_car(true);
            give_up(out.reason, now);
        }
        else
        {
            drive_wheels(out.steer, out.left_um_s, out.right_um_s, false);
            look_for_line(FOLLOW_LINE == out.result, now);
        }
    }
}

/*!
 * @brief Tell Buddy 5 the bypass gave up (REJECT, the id field carries the
 *        reason), every OBSTACLE_NAV_LINE_REPEAT_MS until it stops.
 *
 * @param[in] reason FOLLOW_REASON_*.
 * @param[in] now    Current time, ms.
 */
static void give_up (uint8_t reason, uint32_t now)
{
    if (nav_core_repeat_due(&g_nav.core, now))
    {
        LOG_NOW("WARN", "line:nav", "bypass gave up, reason %d\n", (INT)reason);
        post(OBSTACLE_CMD_REJECT, reason);
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
    return (nav_core_heading_deci(&g_nav.core, g_nav.wheel_um));
}

/*!
 * @brief Start the dead reckoning when BYPASS begins.
 */
static void reset_pose (void)
{
    nav_core_reset_pose(&g_nav.core, g_nav.wheel_um);
    g_nav.last_cmd[WHEEL_LEFT]  = 0;
    g_nav.last_cmd[WHEEL_RIGHT] = 0;
    g_nav.b_pausing             = false;
    g_nav.b_turned              = false;
}

/*!
 * @brief Move the position on by the wheel travel since the last pass,
 *        along the current heading.
 */
static void update_pose (void)
{
    nav_core_update_pose(&g_nav.core, g_nav.wheel_um);
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

        if (nav_core_repeat_due(&g_nav.core, now))
        {
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
 * @param[in] b_scan_pivot Use the scan-aiming duty limit.
 */
static void drive_wheels (uint8_t steer, int32_t left_um_s, int32_t right_um_s,
                         bool b_scan_pivot)
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

    if (b_scan_pivot)
    {
        (void)motion_set_scan_pivot(speed[WHEEL_LEFT], speed[WHEEL_RIGHT],
                                    OBSTACLE_NAV_TTL_US);
    }
    else
    {
        (void)motion_set_speed(speed[WHEEL_LEFT], speed[WHEEL_RIGHT],
                               OBSTACLE_NAV_TTL_US);
    }
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
 * @brief BYPASS: pivot away to the statechart's escape target, and report
 *        TURNED once it is reached. While the beam still sees the obstacle
 *        the statechart moves the target on, so the car turns further; it
 *        ends BYPASS when the beam is clear.
 *
 * @param[in] now Current time, ms.
 */
static void step_pivot (uint32_t now)
{
    int32_t const target_head = (int32_t)(int16_t)(uint16_t)(obstacle_comms_aim() >> AIM_BODY_SHIFT);
    int32_t       left_um_s   = 0;
    int32_t       right_um_s  = 0;
    uint8_t       steer       = OBSTACLE_STEER_HOLD;
    bool          b_done      = false;

    obstacle_plan_wheel_pivot(heading_deci(), target_head,
                              &left_um_s, &right_um_s, &steer, &b_done);
    note_turned(b_done, now);
    drive_wheels(steer, left_um_s, right_um_s, false);
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

        if (OBSTACLE_STATE_SCANNING == state)
        {
            /* The stop point is the reference for the scan pivots and the
             * whole bypass. */
            reset_pose();
        }

        if (OBSTACLE_STATE_REACQUIRE == state)
        {
            follow_in_t in;

            nav_core_follow_begin(&g_nav.core, obstacle_comms_ping(), g_nav.wheel_um, &in);
            /* The reading that stopped the car is the first face point. */
            follow_start(&g_nav.follow, OBSTACLE_SIDE_LEFT == g_nav.side, &in,
                         (int32_t)g_nav.hit_mm + (int32_t)OBSTACLE_SONAR_FWD_MM);
            g_nav.b_follow_turning = false;
            g_nav.bump.cmd_mm_s[WHEEL_LEFT]  = 0;
            g_nav.bump.cmd_mm_s[WHEEL_RIGHT] = 0;
            g_nav.bump.settle_ms             = 0U;
            g_nav.bump.low_ms                = 0U;
            g_nav.bump_ms                    = now_ms();
            obstacle_comms_set_look((int16_t)(-side_sign() * (int32_t)OBSTACLE_FOLLOW_LOOK_DECI));
        }

        if ((OBSTACLE_STATE_BYPASS == state) || (OBSTACLE_STATE_REACQUIRE == state) ||
            (OBSTACLE_STATE_MONITOR == state))
        {
            /* The servo turns on entering these states (back to 0 after
             * the scan, to 45 degrees, or back to 0 after the bypass): wait
             * for the next ping, which comes only after it has settled. */
            nav_core_servo_wait(&g_nav.core, status, now_ms());
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
    return (nav_core_servo_moving(&g_nav.core, status, now));
}

/*!
 * @brief Keep the accepted plan: the start angle and the sideways offset the
 *        car needs. The offset is the planner's side room (the box edge plus
 *        OBSTACLE_BOX_GAP_MM). It used to be closest x tan(start
 *        angle), which ignored the box width: with a 45 degree start every
 *        box got the same offset, and boxes wider than 240 mm were brushed.
 *        That formula stays as the lower bound.
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
    g_nav.hit_mm    = (int16_t)closest;
    /* Aim the servo at the face side now: the scan task enters REACQUIRE on
     * its own and must not take the first ping at an old angle. */
    obstacle_comms_set_look((int16_t)(-side_sign() * (int32_t)OBSTACLE_FOLLOW_LOOK_DECI));
    g_nav.need_mm =
        clamp_i16((closest * obstacle_plan_tan_q14((int32_t)angle * DECI_PER_DEG)) / Q14_ONE);
    {
        obstacle_plan_summary_t plan;

        if ((E_OK == obstacle_copy_plan(&plan)) && (plan.need_mm < OBSTACLE_PLAN_SIDE_BLOCKED) &&
            ((int32_t)plan.need_mm > (int32_t)g_nav.need_mm))
        {
            g_nav.need_mm = clamp_i16((int32_t)plan.need_mm);
        }
    }
    LOG_NOW("INFO", "line:nav", "plan: obstacle %d mm, start %d deg, side "
                    "offset %d mm, %s\n",
              (INT)closest, (INT)angle, (INT)g_nav.need_mm,
              b_wall ? "wall" : "box");
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

        LOG_NOW("ERROR", "line:nav", "FAULT cause=%d: holding for operator START\n", (INT)cause);

        /* Casts are safe: the cause is 4 bits. The flight log keeps the low 8
         * fault bits. */
        log_rec(OBSTACLE_LOG_FAULT, (uint8_t)cause, 0U, (uint8_t)faults, 0, 0, 0);
        hold_car(true);
        g_nav.b_scan_pending = false;
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
        else if (g_nav.req_id == req_id)
        {
            /* No way round: answer at once, so Buddy 5 stops with the
             * plan's cause instead of a PLAN timeout and a rescan. */
            post(OBSTACLE_CMD_REJECT, g_nav.req_id);
        }
        else
        {
            /* A result for an older request. */
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
    if (b_line_seen && nav_core_repeat_due(&g_nav.core, now))
    {
        post(OBSTACLE_CMD_LINE_FOUND, g_nav.req_id);
    }
}

/*!
 * @brief The motion service has latched a fault: print and log, for each
 *        wheel, the last sample it judged, so the failed check can be seen
 *        (FEEDBACK: speed_valid 0 means an invalid encoder step or a sample
 *        over 100 ms old; code 5 NO_EDGES is no longer raised by the motion
 *        service, the pilot judges a wheel without travel).
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
 * 6 OUTPUT (5 is kept for the wire; it is no longer raised). STOPPED and
 * FAULT both need a reboot.
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
            g_nav.wheel_um[wheel] = obstacle_wheel_um(&feedback, wheel);

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

/*** end of file ***/
