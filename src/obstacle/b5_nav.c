/** @file b5_nav.c
 *
 * @brief Buddy 5 Phase 2 glue between the obstacle statechart, the line
 *        task in main.c and the motion service.
 *
 * What the line task does with Buddy 5 (design review 4.1):
 *   - OBSTACLE      hold the car, wait B5_NAV_BRAKE_MS, send SCAN_REQ(id).
 *   - SCAN_RESULT   OK or CLEAR: send ACCEPT(id), and keep the plan: the
 *                   side, the start angle, the sideways offset the car
 *                   needs and how far ahead the obstacle ends.
 *   - BYPASS        pivot away from the obstacle (inside wheel stopped)
 *                   until Buddy 5 sees the way ahead is clear.
 *   - REACQUIRE     drive the rest of the bypass from the wheel odometry,
 *                   in four phases [Chat]:
 *                     TURN   keep pivoting until the planned start angle
 *                            is reached (if the beam cleared first);
 *                     OUT    straight on until the car is the planned
 *                            offset to the side of the line;
 *                     ALONG  zig-zag along the obstacle, holding that
 *                            offset, until the tail has passed the far end
 *                            of the obstacle (front + assumed depth);
 *                     BACK   curve towards the line, then straight on
 *                            until the first IR line contact;
 *                     ALIGN  pivot back to a shallow angle to the line
 *                            (B5_NAV_ALIGN_DECI), then the next IR line
 *                            contact hands back to line following.
 *                   IR line contact counts only from BACK on, and only
 *                   once both IR sensors have been off the line since the
 *                   car reached its offset (the end of OUT).
 *                   Guard, in OUT, ALONG and BACK: when the median of the
 *                   last three 45 degree readings is closer than
 *                   B5_HUG_NEAR_MM, the car turns away until it is
 *                   farther than B5_HUG_FAR_MM.
 *   - FAULT         hold the car; retry RESET + START every 3 s.
 *   - MONITOR       release the hold and let the line follower drive,
 *                   except while Buddy 5's latest reading is a VALID echo
 *                   within the detection distance (no creeping forward
 *                   into an obstacle while a retry is in progress).
 *
 * Position frame (dead reckoning from the wheel encoders): x is sideways,
 * positive to the left of the original line; y is along the line; both
 * are for the point midway between the wheels and start at 0 when BYPASS
 * begins. The heading is worked out from the total wheel difference, so
 * it does not drift between passes.
 *
 * Everything that happens is also written to the flight recorder
 * (b5_log.c), which prints it when a USB serial monitor is opened.
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
 * docs/buddy5_obstacle.md). Casts to (UB *) and (INT) feed tm_printf()
 * only; every value printed is below 2^16.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../motion/motion_service.h"
#include "b5_cfg.h"
#include "b5_log.h"
#include "b5_nav.h"
#include "b5_pack.h"
#include "b5_plan.h"
#include "obstacle.h"
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

/* Inside wheel speed during the escape pivot (0 = stopped). */
#define NAV_ESCAPE_INSIDE_UM_S (B5_NAV_ESCAPE_INSIDE_UM_S)

/* Bypass phases inside REACQUIRE (see the file comment). */
#define PHASE_TURN  (0U)
#define PHASE_OUT   (1U)
#define PHASE_ALONG (2U)
#define PHASE_BACK  (3U)
#define PHASE_ALIGN (4U)

/* Steering intent, turned into LEFT / RIGHT by the side being passed. */
#define INTENT_STRAIGHT (0U)
#define INTENT_AWAY     (1U)
#define INTENT_TOWARD   (2U)

/* Guard: readings kept for the median, and the value used for OPEN,
 * UNKNOWN or STUCK readings (farther than any echo). */
#define GUARD_PINGS  (3U)
#define GUARD_FAR_MM (4095U)

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

/* IR and steer bits of a TRACE record (see b5_log.c). */
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
    bool     b_scan_pending;
    bool     b_turned;
    bool     b_servo_wait;
    bool     b_pausing;
    bool     b_ir_armed;
    bool     b_ir_cleared;
    bool     b_guard;
    bool     b_level;
    uint8_t  req_id;
    uint8_t  next_id;
    uint8_t  last_state;
    uint8_t  start_deg;
    uint8_t  side;
    uint8_t  phase;
    uint8_t  wait_seq;
    uint8_t  ping_seq;
    uint8_t  ping_count;
    uint8_t  ping_next;
    uint16_t ping_mm[GUARD_PINGS];
    uint32_t wait_ms;
    uint32_t pause_ms;
    uint32_t last_drive;
    uint32_t hold_ms;
    uint32_t retry_ms;
    uint32_t line_ms;
    uint32_t trace_ms;
    uint32_t motion_word;
    int32_t  need_mm;
    int32_t  pass_mm;
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
    .b_enabled      = false,
    .b_scan_pending = false,
    .b_turned       = false,
    .b_servo_wait   = false,
    .b_pausing      = false,
    .b_ir_armed     = false,
    .b_ir_cleared   = false,
    .b_guard        = false,
    .b_level        = false,
    .req_id         = 0U,
    .next_id        = 0U,
    .last_state     = NAV_UNSEEN,
    .start_deg      = 0U,
    .side           = B5_SIDE_LEFT,
    .phase          = PHASE_TURN,
    .wait_seq       = 0U,
    .ping_seq       = 0U,
    .ping_count     = 0U,
    .ping_next      = 0U,
    .ping_mm        = {GUARD_FAR_MM, GUARD_FAR_MM, GUARD_FAR_MM},
    .wait_ms        = 0U,
    .pause_ms       = 0U,
    .last_drive     = NAV_DRIVE_UNSEEN,
    .hold_ms        = 0U,
    .retry_ms       = 0U,
    .line_ms        = 0U,
    .trace_ms       = 0U,
    .motion_word    = UINT32_MAX,
    .need_mm        = 0,
    .pass_mm        = 0,
    .obst_mm        = 0,
    .wheel_mm       = {0, 0},
    .last_cmd       = {0, 0},
    .wheel_um       = {0, 0},
    .base_um        = {0, 0},
    .prev_um        = {0, 0},
    .x_um           = 0,
    .y_um           = 0,
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
static bool     is_turn_done (void);
static uint32_t intent_steer (uint32_t intent);
static void     drive_steer (uint32_t steer, int32_t inside_um_s,
                             int32_t outside_um_s);
static bool     is_reverse_pause (int32_t const p_speed[WHEELS]);
static void     set_phase (uint8_t phase);
static uint32_t along_intent (void);
static uint32_t back_intent (void);
static uint32_t align_intent (void);
static bool     is_aligned (void);
static void     drive_reacquire (uint32_t status, bool b_line_seen);
static void     reset_guard (uint32_t status);
static uint32_t median_of_3 (uint32_t first, uint32_t second, uint32_t third);
static uint32_t guard_reading (uint32_t status);
static uint32_t guard_intent (uint32_t reading, uint32_t intent);
static void     update_ir_gate (bool b_line_seen);
static bool     is_obstacle_close (uint32_t status);
static bool     is_servo_moving (uint32_t status, uint32_t now);
static void     report_state (uint32_t status);
static void     keep_plan (uint32_t angle);
static void     explain_scan (uint16_t const * p_profile);
static void     handle_event (uint32_t evt, uint32_t now);
static void     look_for_line (bool b_line_seen, uint32_t now);
static void     update_motion (void);
static int64_t true_wheel_um (motion_sample_t const * p_sample, uint32_t wheel);
static void    report_fault (motion_service_status_t const * p_status);
static void    trace (uint32_t status, bool b_left, bool b_right, uint32_t now);
static ER      nav_stop (void);
static ER      nav_distance_mm (uint32_t * p_distance_mm);

/*!
 * @brief Start Buddy 5 with the motion links, then RESET and START it.
 *
 * Call once from the line task on processor 1, after gpio_irq_init(),
 * motion_service_start() and ir_capture_init(). If it fails, b5_nav_step()
 * only services the flight recorder, so line following works without
 * Buddy 5 and the failure can still be read back.
 *
 * @return The result of obstacle_init().
 */
ER b5_nav_init (void)
{
    obstacle_links_t const links  = {0, nav_stop, nav_distance_mm};
    ER const               result = obstacle_init(&links);

    log_rec(B5_LOG_BOOT, 0U, 0U, 0U, (int32_t)result, 0, 0);

    if (E_OK == result)
    {
        g_nav.b_enabled = true;
        g_nav.retry_ms  = now_ms();
        post(B5_CMD_RESET, 0U);
        post(B5_CMD_START, 0U);
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
bool b5_nav_step (bool b_line_left, bool b_line_right)
{
    bool     b_in_charge = false;
    uint32_t evt         = 0U;
    uint32_t state       = B5_STATE_MONITOR;
    uint32_t status      = 0U;
    uint32_t now         = 0U;

    b5_log_service();
    update_motion();

    if (g_nav.b_enabled)
    {
        now = now_ms();
        while (E_OK == obstacle_get_event(&evt, TMO_POL))
        {
            handle_event(evt, now);
        }

        if (g_nav.b_scan_pending && ((now - g_nav.hold_ms) >= B5_NAV_BRAKE_MS))
        {
            g_nav.b_scan_pending = false;
            post(B5_CMD_SCAN_REQ, g_nav.req_id);
        }

        status = obstacle_get_status();
        state  = b5_pack_get(status, B5_ST_STATE_SH, B5_ST_STATE_MK);
        report_state(status);
        trace(status, b_line_left, b_line_right, now);

        switch (state)
        {
            case B5_STATE_MONITOR:
                b_in_charge = (is_servo_moving(status, now)) ||
                              (is_obstacle_close(status));
                hold_car(b_in_charge);
            break;

            case B5_STATE_BYPASS:
                /* Pivot until Buddy 5 sees the way ahead is clear. NOTE: the
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
                    (void)is_turn_done();
                    drive_steer(intent_steer(INTENT_AWAY),
                                (int32_t)NAV_ESCAPE_INSIDE_UM_S,
                                (int32_t)B5_NAV_SPEED_UM_S);
                }
                b_in_charge = true;
            break;

            case B5_STATE_REACQUIRE:
                update_pose();
                if (is_servo_moving(status, now))
                {
                    /* The servo is swinging to 45 degrees: stand still. */
                    hold_car(true);
                }
                else
                {
                    hold_car(false);
                    update_ir_gate(b_line_left || b_line_right);
                    drive_reacquire(status, b_line_left || b_line_right);
                    look_for_line((PHASE_ALIGN == g_nav.phase) &&
                                      (is_aligned()) &&
                                      (b_line_left || b_line_right),
                                  now);
                }
                b_in_charge = true;
            break;

            case B5_STATE_STOPPED:
                hold_car(true);
                if ((now - g_nav.retry_ms) >= B5_NAV_RETRY_MS)
                {
                    g_nav.retry_ms = now;
                    post(B5_CMD_RESET, 0U);
                    post(B5_CMD_START, 0U);
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
 * @param[in] kind  Record kind (B5_LOG_*).
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
    b5_log_rec_t rec;

    rec.t_ms  = 0U;
    rec.kind  = kind;
    rec.val_a = val_a;
    rec.val_b = val_b;
    rec.val_c = val_c;
    rec.val_d = clamp_i16(val_d);
    rec.val_e = clamp_i16(val_e);
    rec.val_f = val_f;
    b5_log_add(&rec);
}

/*!
 * @brief Post one command to Buddy 5, print it and log it.
 *
 * @param[in] type   Command type (B5_CMD_*).
 * @param[in] req_id Request id.
 */
static void post (uint32_t type, uint8_t req_id)
{
    ER const result = obstacle_post_cmd(b5_pack_cmd(type, req_id));

    tm_printf((UB *)"[b5-nav] cmd %d id=%d -> %d\n", (INT)type, (INT)req_id,
              (INT)result);

    /* Cast is safe: the command type is 3 bits wide. */
    log_rec(B5_LOG_CMD, (uint8_t)type, req_id, 0U, (int32_t)result, 0, 0);
}

/*!
 * @brief Set or release the obstacle hold. The motion service ignores a
 *        call that does not change the hold, so this runs every pass.
 *
 * @param[in] b_held True to hold the car.
 */
static void hold_car (bool b_held)
{
    (void)motion_service_set_hold(MOTION_HOLD_OBSTACLE, b_held);
}

/*!
 * @brief Sign that turns "away from the obstacle" into left / right.
 *
 * @return +1 when passing on the left, -1 when passing on the right.
 */
static int32_t side_sign (void)
{
    return ((B5_SIDE_LEFT == g_nav.side) ? 1 : -1);
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
    int64_t const track_um = (int64_t)B5_WHEEL_TRACK_MM * UM_PER_MM;

    /* Cast is safe: a bypass turns the car far less than 2^31 deci-deg. */
    return ((int32_t)(((right_um - left_um) * B5_NAV_DECI_PER_RAD) / track_um));
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
    g_nav.b_level               = false;
    g_nav.phase                 = PHASE_TURN;
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
    g_nav.x_um += (step_um * b5_plan_sin_q14(heading)) / Q14_ONE;
    g_nav.y_um += (step_um * b5_plan_cos_q14(heading)) / Q14_ONE;
    g_nav.prev_um[WHEEL_LEFT]  = g_nav.wheel_um[WHEEL_LEFT];
    g_nav.prev_um[WHEEL_RIGHT] = g_nav.wheel_um[WHEEL_RIGHT];
}

/*!
 * @brief Tell whether the pivot has reached the planned start angle
 *        (always true when the plan has no angle). Logs a TURNED record
 *        the first time it does.
 *
 * @return True once the car has turned at least the planned angle.
 */
static bool is_turn_done (void)
{
    int32_t const turned = side_sign() * heading_deci();

    if ((!g_nav.b_turned) &&
        (turned >= ((int32_t)g_nav.start_deg * DECI_PER_DEG)))
    {
        g_nav.b_turned = true;
        tm_printf((UB *)"[b5-nav] escape turned %d deg (plan %d)\n",
                  (INT)(turned / DECI_PER_DEG), (INT)g_nav.start_deg);
        log_rec(B5_LOG_TURNED, 0U, 0U, 0U, turned, (int32_t)g_nav.start_deg, 0);
    }

    return (g_nav.b_turned);
}

/*!
 * @brief Turn a steering intent into LEFT / RIGHT for the side passed.
 *
 * @param[in] intent INTENT_STRAIGHT, INTENT_AWAY or INTENT_TOWARD.
 *
 * @return B5_STEER_*.
 */
static uint32_t intent_steer (uint32_t intent)
{
    bool const b_left = (B5_SIDE_LEFT == g_nav.side);
    uint32_t   steer  = B5_STEER_STRAIGHT;

    if (INTENT_AWAY == intent)
    {
        steer = b_left ? B5_STEER_LEFT : B5_STEER_RIGHT;
    }
    else if (INTENT_TOWARD == intent)
    {
        steer = b_left ? B5_STEER_RIGHT : B5_STEER_LEFT;
    }
    else
    {
        steer = B5_STEER_STRAIGHT;
    }

    return (steer);
}

/*!
 * @brief Turn a steering decision into a speed command, and print and log
 *        it whenever the speeds change.
 *
 * STRAIGHT drives both wheels at B5_NAV_SPEED_UM_S; LEFT and RIGHT run the
 * inside wheel at inside_um_s and the outside wheel at outside_um_s; HOLD
 * stands still.
 *
 * @param[in] steer        Steering (B5_STEER_*).
 * @param[in] inside_um_s  Inside wheel speed (-40 000..40 000 um/s;
 *                          0 = pivot on the inside wheel).
 * @param[in] outside_um_s Outside wheel speed (40 000..100 000 um/s).
 */
static void drive_steer (uint32_t steer, int32_t inside_um_s,
                         int32_t outside_um_s)
{
    int32_t speed[WHEELS] = {0, 0};

    switch (steer)
    {
        case B5_STEER_STRAIGHT:
            /* Cast is safe: the speed is 40 000 um/s. */
            speed[WHEEL_LEFT]  = (int32_t)B5_NAV_SPEED_UM_S;
            speed[WHEEL_RIGHT] = (int32_t)B5_NAV_SPEED_UM_S;
        break;

        case B5_STEER_LEFT:
            speed[WHEEL_LEFT]  = inside_um_s;
            speed[WHEEL_RIGHT] = outside_um_s;
        break;

        case B5_STEER_RIGHT:
            speed[WHEEL_LEFT]  = outside_um_s;
            speed[WHEEL_RIGHT] = inside_um_s;
        break;

        default:
            /* HOLD: stand still. */
            speed[WHEEL_LEFT]  = 0;
            speed[WHEEL_RIGHT] = 0;
        break;
    }

    if (is_reverse_pause(speed))
    {
        speed[WHEEL_LEFT]  = 0;
        speed[WHEEL_RIGHT] = 0;
    }

    {
        int32_t const left_mm_s  = speed[WHEEL_LEFT] / UM_PER_MM;
        int32_t const right_mm_s = speed[WHEEL_RIGHT] / UM_PER_MM;

        /* Casts are safe: both speeds are -100..100 mm/s, so the offset
         * values are 28..228. */
        uint32_t const key =
            steer |
            ((uint32_t)(left_mm_s + DRIVE_OFFSET_MM_S) << DRIVE_LEFT_SH) |
            ((uint32_t)(right_mm_s + DRIVE_OFFSET_MM_S) << DRIVE_RIGHT_SH);

        if (g_nav.last_drive != key)
        {
            g_nav.last_drive = key;
            tm_printf((UB *)"[b5-nav] steer %d: left %d right %d mm/s\n",
                      (INT)steer, (INT)left_mm_s, (INT)right_mm_s);

            /* Cast is safe: the steer field is 2 bits wide. */
            log_rec(B5_LOG_STEER, (uint8_t)steer, 0U, 0U, left_mm_s, right_mm_s,
                    heading_deci());
        }
    }

    (void)motion_service_set_speed(speed[WHEEL_LEFT], speed[WHEEL_RIGHT],
                                   B5_NAV_TTL_US);
}

/*!
 * @brief Stop both wheels for B5_NAV_REVERSE_PAUSE_MS before any wheel
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

    if (g_nav.b_pausing && ((now - g_nav.pause_ms) < B5_NAV_REVERSE_PAUSE_MS))
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
        tm_printf((UB *)"[b5-nav] wheel reverses: stop for %d ms first\n",
                  (INT)B5_NAV_REVERSE_PAUSE_MS);
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
    tm_printf((UB *)"[b5-nav] phase %d: side %d mm, ahead %d mm, heading %d "
                    "deg\n",
              (INT)phase, (INT)lateral_mm(), (INT)ahead_mm(),
              (INT)(heading / DECI_PER_DEG));
    log_rec(B5_LOG_PHASE, phase, 0U, 0U, lateral_mm(), ahead_mm(), heading);
}

/*!
 * @brief ALONG: zig-zag that holds the planned sideways offset. Too close
 *        bends away, too far bends towards, inside the band the car is
 *        steered back to parallel. The heading stays within
 *        B5_NAV_HEAD_MAX_DECI away from and B5_NAV_ALONG_TOWARD_DECI
 *        towards the line.
 *
 * NOTE: Inside the band the car turns until its heading is within
 * B5_NAV_LEVEL_IN_DECI, then drives straight until it is more than
 * B5_NAV_LEVEL_OUT_DECI off (g_nav.b_level). Without this dead zone a
 * pivot (about 100 degrees per second while the encoders are crossed)
 * overshot past 0 every pass and the car flipped between left and right
 * pivots every 30-60 ms (seen on the car).
 *
 * @return INTENT_*.
 */
static uint32_t along_intent (void)
{
    int32_t const lateral = lateral_mm();
    int32_t const heading = side_sign() * heading_deci();
    uint32_t      intent  = INTENT_STRAIGHT;

    int32_t const size = (heading < 0) ? -heading : heading;

    if (lateral < (g_nav.need_mm - (int32_t)B5_NAV_BAND_IN_MM))
    {
        g_nav.b_level = false;
        intent        = INTENT_AWAY;
    }
    else if (lateral > (g_nav.need_mm + (int32_t)B5_NAV_BAND_OUT_MM))
    {
        g_nav.b_level = false;
        intent        = INTENT_TOWARD;
    }
    else
    {
        if (g_nav.b_level)
        {
            g_nav.b_level = (size <= B5_NAV_LEVEL_OUT_DECI);
        }
        else
        {
            g_nav.b_level = (size <= B5_NAV_LEVEL_IN_DECI);
        }

        if (g_nav.b_level)
        {
            intent = INTENT_STRAIGHT;
        }
        else
        {
            intent = (heading > 0) ? INTENT_TOWARD : INTENT_AWAY;
        }
    }

    if ((INTENT_AWAY == intent) && (heading >= B5_NAV_HEAD_MAX_DECI))
    {
        intent = INTENT_STRAIGHT;
    }
    else if ((INTENT_TOWARD == intent) &&
             (heading <= -B5_NAV_ALONG_TOWARD_DECI))
    {
        intent = INTENT_STRAIGHT;
    }
    else
    {
        /* Inside the heading limit. */
    }

    return (intent);
}

/*!
 * @brief BACK: curve towards the line until the rejoin angle, then
 *        straight on until an IR sensor finds the line.
 *
 * @return INTENT_*.
 */
static uint32_t back_intent (void)
{
    int32_t const heading = side_sign() * heading_deci();

    return ((heading > -B5_NAV_REJOIN_DECI) ? INTENT_TOWARD : INTENT_STRAIGHT);
}

/*!
 * @brief ALIGN: pivot back until the car heads no more than
 *        B5_NAV_ALIGN_DECI towards the line, then straight on.
 *
 * NOTE: BACK crosses the line at B5_NAV_REJOIN_DECI. At today's speed
 * (about 150 mm/s with the encoders crossed) the line follower saw the
 * line once and drove on across it (seen on the car). Turning to a
 * shallow angle at the first contact gives it a crossing it can follow.
 * The car is not turned fully parallel: the IR sensors are ahead of the
 * pivot, so that would leave the line beside the car, out of their
 * reach.
 *
 * @return INTENT_*.
 */
static uint32_t align_intent (void)
{
    int32_t const toward = -(side_sign() * heading_deci());
    uint32_t      intent = INTENT_STRAIGHT;

    if (toward > B5_NAV_ALIGN_DECI)
    {
        intent = INTENT_AWAY;
    }
    else if (toward < 0)
    {
        intent = INTENT_TOWARD;
    }
    else
    {
        /* Shallow angle towards the line: straight on. */
    }

    return (intent);
}

/*!
 * @brief Tell whether ALIGN has finished its pivot.
 *
 * @return True once the car heads 0 .. B5_NAV_ALIGN_DECI (plus one pass
 *         of overshoot) towards the line.
 */
static bool is_aligned (void)
{
    int32_t const toward = -(side_sign() * heading_deci());

    return ((toward >= 0) &&
            (toward <= (B5_NAV_ALIGN_DECI + B5_NAV_ALIGN_SLACK_DECI)));
}

/*!
 * @brief REACQUIRE: run the TURN, OUT, ALONG, BACK and ALIGN phases.
 *
 * @param[in] status      Packed Buddy 5 status word (latest 45 degree
 *                        reading).
 * @param[in] b_line_seen An IR sensor sees the line.
 */
static void drive_reacquire (uint32_t status, bool b_line_seen)
{
    uint32_t const reading = guard_reading(status);
    uint32_t       intent  = INTENT_STRAIGHT;

    if ((PHASE_TURN == g_nav.phase) && (is_turn_done()))
    {
        set_phase(PHASE_OUT);
    }

    if ((PHASE_OUT == g_nav.phase) && (lateral_mm() >= g_nav.need_mm))
    {
        set_phase(PHASE_ALONG);
        g_nav.b_ir_armed = true;
    }

    if ((PHASE_ALONG == g_nav.phase) && (ahead_mm() >= g_nav.pass_mm))
    {
        set_phase(PHASE_BACK);
    }

    if ((PHASE_BACK == g_nav.phase) && g_nav.b_ir_cleared && b_line_seen)
    {
        set_phase(PHASE_ALIGN);
    }

    if (PHASE_TURN == g_nav.phase)
    {
        /* The beam cleared before the planned angle: keep pivoting. */
        drive_steer(intent_steer(INTENT_AWAY), (int32_t)NAV_ESCAPE_INSIDE_UM_S,
                    (int32_t)B5_NAV_SPEED_UM_S);
    }
    else
    {
        if (PHASE_ALONG == g_nav.phase)
        {
            intent = along_intent();
        }
        else if (PHASE_BACK == g_nav.phase)
        {
            intent = back_intent();
        }
        else if (PHASE_ALIGN == g_nav.phase)
        {
            intent = align_intent();
        }
        else
        {
            /* OUT: straight on at the planned angle. */
            intent = INTENT_STRAIGHT;
        }

        intent = guard_intent(reading, intent);

        drive_steer(intent_steer(intent), (int32_t)B5_NAV_HUG_INSIDE_UM_S,
                    (int32_t)B5_NAV_HUG_OUTSIDE_UM_S);
    }
}

/*!
 * @brief Start the guard afresh: forget earlier readings and count only
 *        pings newer than the one in this status word.
 *
 * @param[in] status Packed Buddy 5 status word.
 */
static void reset_guard (uint32_t status)
{
    uint32_t slot = 0U;

    for (slot = 0U; slot < GUARD_PINGS; slot++)
    {
        g_nav.ping_mm[slot] = (uint16_t)GUARD_FAR_MM;
    }

    /* Cast is safe: the sequence field is 6 bits. */
    g_nav.ping_seq   = (uint8_t)b5_pack_get(status, B5_ST_SEQ_SH, B5_ST_SEQ_MK);
    g_nav.ping_count = 0U;
    g_nav.ping_next  = 0U;
    g_nav.b_guard    = false;
}

/*!
 * @brief Middle value of three.
 *
 * @param[in] first  A value.
 * @param[in] second A value.
 * @param[in] third  A value.
 *
 * @return The median.
 */
static uint32_t median_of_3 (uint32_t first, uint32_t second, uint32_t third)
{
    uint32_t median = second;

    if (((first >= second) && (first <= third)) ||
        ((first <= second) && (first >= third)))
    {
        median = first;
    }
    else if (((third >= first) && (third <= second)) ||
             ((third <= first) && (third >= second)))
    {
        median = third;
    }
    else
    {
        /* The second value is the middle one. */
    }

    return (median);
}

/*!
 * @brief Keep the newest 45 degree readings and return their median, so
 *        one stray echo cannot steer the car.
 *
 * @param[in] status Packed Buddy 5 status word (latest reading).
 *
 * @return Median of the last GUARD_PINGS readings in mm (GUARD_FAR_MM
 *         until that many have arrived; OPEN, UNKNOWN and STUCK count as
 *         GUARD_FAR_MM).
 */
static uint32_t guard_reading (uint32_t status)
{
    uint32_t const cls = b5_pack_get(status, B5_ST_CLASS_SH, B5_ST_CLASS_MK);
    uint32_t const dist_mm = b5_pack_get(status, B5_ST_DIST_SH, B5_ST_DIST_MK);
    uint32_t const seq     = b5_pack_get(status, B5_ST_SEQ_SH, B5_ST_SEQ_MK);
    uint32_t       reading = GUARD_FAR_MM;

    if (seq != g_nav.ping_seq)
    {
        /* Casts are safe: the sequence field is 6 bits, the distance 12
         * bits, ping_next is 0..2 and ping_count stops at 3. */
        g_nav.ping_seq = (uint8_t)seq;
        g_nav.ping_mm[g_nav.ping_next] =
            (uint16_t)((B5_CLASS_VALID == cls) ? dist_mm : GUARD_FAR_MM);
        g_nav.ping_next = (uint8_t)((g_nav.ping_next + 1U) % GUARD_PINGS);

        if (g_nav.ping_count < GUARD_PINGS)
        {
            g_nav.ping_count = (uint8_t)(g_nav.ping_count + 1U);
        }
    }

    if (GUARD_PINGS == g_nav.ping_count)
    {
        reading =
            median_of_3(g_nav.ping_mm[0], g_nav.ping_mm[1], g_nav.ping_mm[2]);
    }

    return (reading);
}

/*!
 * @brief Guard for OUT, ALONG and BACK: closer than B5_HUG_NEAR_MM turns
 *        the car away from the obstacle, and it keeps turning away until
 *        the reading is farther than B5_HUG_FAR_MM. The turn stops at
 *        B5_NAV_GUARD_HEAD_MAX_DECI away from the line.
 *
 * @param[in] reading Filtered 45 degree reading, mm.
 * @param[in] intent  INTENT_* chosen by the phase.
 *
 * @return INTENT_* to drive.
 */
static uint32_t guard_intent (uint32_t reading, uint32_t intent)
{
    int32_t const heading = side_sign() * heading_deci();
    uint32_t      result  = intent;
    bool const    b_was   = g_nav.b_guard;

    if ((!g_nav.b_guard) && (reading < B5_HUG_NEAR_MM))
    {
        g_nav.b_guard = true;
    }
    else if (g_nav.b_guard && (reading > B5_HUG_FAR_MM))
    {
        g_nav.b_guard = false;
    }
    else
    {
        /* Inside the band: keep what the guard was doing. */
    }

    if (b_was != g_nav.b_guard)
    {
        tm_printf((UB *)"[b5-nav] guard %s: 45 deg reading %d mm, heading %d "
                        "deg\n",
                  g_nav.b_guard ? "ON (turn away)" : "off", (INT)reading,
                  (INT)(heading / DECI_PER_DEG));

        /* Casts are safe: the phase is 0..3 and reading at most 4095. */
        log_rec(B5_LOG_GUARD, (uint8_t)(g_nav.b_guard ? 1U : 0U), g_nav.phase,
                0U, (int32_t)reading, 0, heading);
    }

    if (g_nav.b_guard)
    {
        result = (heading < B5_NAV_GUARD_HEAD_MAX_DECI) ? INTENT_AWAY
                                                        : INTENT_STRAIGHT;
    }

    return (result);
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
        tm_printf((UB *)"[b5-nav] IR clear of the line: line contacts count "
                        "from BACK on\n");
    }
}

/*!
 * @brief Tell whether Buddy 5's latest reading is an obstacle within the
 *        detection distance, so the line follower must not drive on.
 *
 * @param[in] status Packed Buddy 5 status word.
 *
 * @return True for a VALID reading at or below B5_DETECT_MM.
 */
static bool is_obstacle_close (uint32_t status)
{
    uint32_t const cls = b5_pack_get(status, B5_ST_CLASS_SH, B5_ST_CLASS_MK);
    uint32_t const dist_mm = b5_pack_get(status, B5_ST_DIST_SH, B5_ST_DIST_MK);

    return ((B5_CLASS_VALID == cls) && (dist_mm <= B5_DETECT_MM));
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
    uint32_t const state = b5_pack_get(status, B5_ST_STATE_SH, B5_ST_STATE_MK);

    if (g_nav.last_state != state)
    {
        uint32_t const cause =
            b5_pack_get(status, B5_ST_CAUSE_SH, B5_ST_CAUSE_MK);

        /* Casts are safe: the state field is 3 bits, the cause 4 bits. */
        g_nav.last_state = (uint8_t)state;
        g_nav.last_drive = NAV_DRIVE_UNSEEN;
        tm_printf((UB *)"[b5-nav] state %d cause %d\n", (INT)state, (INT)cause);
        log_rec(B5_LOG_STATE, (uint8_t)state, (uint8_t)cause,
                obstacle_get_faults(), 0, 0, 0);

        if (B5_STATE_BYPASS == state)
        {
            reset_pose();
        }

        if (B5_STATE_REACQUIRE == state)
        {
            /* The servo turns to 45 degrees: older readings were straight
             * ahead. */
            reset_guard(status);
        }

        if ((B5_STATE_BYPASS == state) || (B5_STATE_REACQUIRE == state) ||
            (B5_STATE_MONITOR == state))
        {
            /* The servo turns on entering these states (back to 0 after
             * the scan, to 45 degrees, or back to 0 after the bypass): wait
             * for the next ping, which comes only after it has settled.
             * Cast is safe: the sequence field is 6 bits. */
            g_nav.b_servo_wait = true;
            g_nav.wait_seq =
                (uint8_t)b5_pack_get(status, B5_ST_SEQ_SH, B5_ST_SEQ_MK);
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
 * only after the servo has settled) or after B5_NAV_SERVO_WAIT_MS.
 *
 * @param[in] status Packed Buddy 5 status word.
 * @param[in] now    Current time, ms.
 *
 * @return True while the car must stand still for the servo.
 */
static bool is_servo_moving (uint32_t status, uint32_t now)
{
    uint32_t const seq = b5_pack_get(status, B5_ST_SEQ_SH, B5_ST_SEQ_MK);

    if (g_nav.b_servo_wait && ((seq != g_nav.wait_seq) ||
                               ((now - g_nav.wait_ms) >= B5_NAV_SERVO_WAIT_MS)))
    {
        g_nav.b_servo_wait = false;
    }

    return (g_nav.b_servo_wait);
}

/*!
 * @brief Keep the accepted plan: the start angle, the sideways offset the
 *        car needs (closest x tan(start angle), the planner's own offset)
 *        and how far ahead the tail must get before turning back (closest
 *        + assumed depth + servo-to-tail). The closest reading comes from
 *        the scan profile, as in scan_algo.c.
 *
 * @param[in] angle Start angle from SCAN_RESULT, degrees.
 */
static void keep_plan (uint32_t angle)
{
    uint16_t profile[B5_SLOT_COUNT];
    uint32_t slot    = 0U;
    int32_t  closest = g_nav.obst_mm;

    if (E_OK == obstacle_copy_profile(profile))
    {
        for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
        {
            /* Cast is safe: slot is 0..24. */
            int32_t const  deci  = scan_algo_slot_deci((uint8_t)slot);
            uint16_t const value = profile[slot];
            int32_t const  dist  = (int32_t)b5_pack_slot_dist(value);

            if ((deci >= -B5_PATH_HALF_DECI) && (deci <= B5_PATH_HALF_DECI) &&
                (b5_pack_slot_is_measured(value)) &&
                (B5_CLASS_VALID == b5_pack_slot_class(value)) &&
                (dist <= (int32_t)B5_REGION_MAX_MM) && (dist < closest))
            {
                closest = dist;
            }
        }
    }

    /* Casts are safe: angle is 0..60 and closest is 0..4095. */
    g_nav.start_deg = (uint8_t)angle;
    g_nav.need_mm =
        (closest * b5_plan_tan_q14((int32_t)angle * DECI_PER_DEG)) / Q14_ONE;
    g_nav.pass_mm =
        closest + (int32_t)B5_DEPTH_ASSUMED_MM + (int32_t)B5_SERVO_TO_REAR_MM;
    tm_printf((UB *)"[b5-nav] plan: obstacle %d mm, start %d deg, side "
                    "offset %d mm, turn back after %d mm\n",
              (INT)closest, (INT)angle, (INT)g_nav.need_mm, (INT)g_nav.pass_mm);
}

/*!
 * @brief Print the 25 scan readings and why each side was or was not
 *        usable, for tethered debugging. The scan is replayed through
 *        scan_algo and b5_plan from the published profile (the same slots
 *        in the same order, so the result is the one Buddy 5 found).
 *
 * @param[in] p_profile B5_SLOT_COUNT packed slots.
 */
static void explain_scan (uint16_t const * p_profile)
{
    static scan_algo_ctx_t g_replay;
    b5_plan_cfg_t          cfg;
    b5_plan_result_t       result;
    uint32_t               slot   = 0U;
    uint32_t               guard  = 0U;
    bool                   b_done = false;

    for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
    {
        uint16_t const value = p_profile[slot];

        /* Cast is safe: slot is 0..24. */
        tm_printf((UB *)" %+3d:",
                  (INT)(scan_algo_slot_deci((uint8_t)slot) / DECI_PER_DEG));

        if (!b5_pack_slot_is_measured(value))
        {
            tm_printf((UB *)"--  ");
        }
        else if (B5_CLASS_VALID == b5_pack_slot_class(value))
        {
            tm_printf((UB *)"%-4d", (INT)b5_pack_slot_dist(value));
        }
        else
        {
            tm_printf((UB *)"%s", (B5_CLASS_OPEN == b5_pack_slot_class(value))
                                      ? "OPN "
                                      : "UNK ");
        }

        if ((B5_SLOT_COUNT - 1U) == slot)
        {
            tm_printf((UB *)"\n");
        }
    }

    /* Replay: feed each requested slot's stored value twice, so the
     * agreement rule settles it exactly as before. */
    scan_algo_start(&g_replay);
    while ((!b_done) && (guard < B5_SLOT_COUNT))
    {
        uint8_t const target = scan_algo_target(&g_replay);

        if (SCAN_ALGO_NO_SLOT == target)
        {
            b_done = true;
        }
        else
        {
            uint16_t const value = p_profile[target];
            uint8_t const  cls   = b5_pack_slot_class(value);
            uint16_t const dist  = b5_pack_slot_dist(value);

            (void)scan_algo_feed(&g_replay, cls, dist);
            b_done = scan_algo_feed(&g_replay, cls, dist);
        }

        guard++;
    }

    b5_plan_cfg_default(&cfg);
    b5_plan_build(&g_replay, &cfg, &result);

    /* Edge states: 0 none, 1 bounded, 2 unbounded (reaches +-60 deg).
     * need 65535 = side blocked (unbounded, too far out, or something in
     * the way beyond the edge). */
    tm_printf((UB *)"[b5-nav] scan: closest %d mm, region %d..%d deg | LEFT "
                    "edge %d last hit %d deg need %d mm | RIGHT edge %d last "
                    "hit %d deg need %d mm\n",
              (INT)g_replay.closest_mm,
              (INT)(scan_algo_slot_deci(g_replay.region_lo) / DECI_PER_DEG),
              (INT)(scan_algo_slot_deci(g_replay.region_hi) / DECI_PER_DEG),
              (INT)g_replay.edge_state[SCAN_ALGO_LEFT],
              (INT)(scan_algo_slot_deci(g_replay.edge_hit[SCAN_ALGO_LEFT]) /
                    DECI_PER_DEG),
              (INT)result.need_mm[B5_SIDE_LEFT],
              (INT)g_replay.edge_state[SCAN_ALGO_RIGHT],
              (INT)(scan_algo_slot_deci(g_replay.edge_hit[SCAN_ALGO_RIGHT]) /
                    DECI_PER_DEG),
              (INT)result.need_mm[B5_SIDE_RIGHT]);
}

/*!
 * @brief React to one Buddy 5 event.
 *
 * @param[in] evt Packed event word.
 * @param[in] now Current time, ms.
 */
static void handle_event (uint32_t evt, uint32_t now)
{
    uint32_t const type   = b5_pack_get(evt, B5_EV_TYPE_SH, B5_EV_TYPE_MK);
    uint32_t const req_id = b5_pack_get(evt, B5_EV_ID_SH, B5_EV_ID_MK);
    uint32_t const status = b5_pack_get(evt, B5_EV_STATUS_SH, B5_EV_STATUS_MK);
    bool const b_go = (B5_RESULT_OK == status) || (B5_RESULT_CLEAR == status);

    if (B5_EVT_OBSTACLE == type)
    {
        uint32_t const dist_mm = b5_pack_get(evt, B5_EV_DIST_SH, B5_EV_DIST_MK);

        tm_printf((UB *)"[b5-nav] OBSTACLE at %d mm: holding\n", (INT)dist_mm);
        log_rec(B5_LOG_OBST, 0U, 0U, 0U, (int32_t)dist_mm, 0, 0);
        hold_car(true);
        g_nav.hold_ms        = now;
        g_nav.req_id         = g_nav.next_id;
        g_nav.b_scan_pending = true;
        g_nav.obst_mm        = (int32_t)dist_mm;

        /* Cast is safe: the id is masked to 5 bits. */
        g_nav.next_id = (uint8_t)((g_nav.next_id + 1U) & B5_ID_MAX);
    }
    else if (B5_EVT_FAULT == type)
    {
        uint32_t const cause = b5_pack_get(evt, B5_EV_CAUSE_SH, B5_EV_CAUSE_MK);
        uint32_t const faults =
            b5_pack_get(evt, B5_EV_FAULTS_SH, B5_EV_FAULTS_MK);

        tm_printf((UB *)"[b5-nav] FAULT cause=%d: holding, retry in 3 s\n",
                  (INT)cause);

        /* Casts are safe: the cause is 4 bits, the faults 8 bits. */
        log_rec(B5_LOG_FAULT, (uint8_t)cause, 0U, (uint8_t)faults, 0, 0, 0);
        hold_car(true);
        g_nav.b_scan_pending = false;
        g_nav.retry_ms       = now;
    }
    else if (B5_EVT_SCAN_RESULT == type)
    {
        uint16_t       profile[B5_SLOT_COUNT];
        uint32_t const side  = b5_pack_get(evt, B5_EV_SIDE_SH, B5_EV_SIDE_MK);
        uint32_t const width = b5_pack_get(evt, B5_EV_WIDTH_SH, B5_EV_WIDTH_MK);
        uint32_t const angle = b5_pack_get(evt, B5_EV_ANGLE_SH, B5_EV_ANGLE_MK);
        uint32_t       index = 0U;

        tm_printf((UB *)"[b5-nav] SCAN_RESULT status=%d side=%d width=%d mm "
                        "start=%d deg\n",
                  (INT)status, (INT)side, (INT)width, (INT)angle);

        /* Casts are safe: req_id is 5 bits and angle 6 bits wide. */
        if (E_OK == obstacle_copy_profile(profile))
        {
            index = b5_log_profile((uint8_t)req_id, profile);
            explain_scan(profile);
        }
        log_rec(B5_LOG_RESULT, (uint8_t)status, (uint8_t)side, (uint8_t)angle,
                (int32_t)width, (int32_t)index, 0);

        if ((g_nav.req_id == req_id) && b_go)
        {
            /* Cast is safe: the side field is 1 bit wide. */
            g_nav.side = (uint8_t)side;
            keep_plan(angle);
            post(B5_CMD_ACCEPT, g_nav.req_id);
        }
    }
    else
    {
        /* STEER words are read with obstacle_get_steer(). */
    }
}

/*!
 * @brief In REACQUIRE, tell Buddy 5 when an IR sensor sees the line, once
 *        the car has turned to its shallow hand-back angle (ALIGN).
 *
 * @param[in] b_line_seen The car is aligned in ALIGN and an IR sensor sees
 *                        the line.
 * @param[in] now         Current time, ms.
 */
static void look_for_line (bool b_line_seen, uint32_t now)
{
    if (b_line_seen && ((now - g_nav.line_ms) >= B5_NAV_LINE_REPEAT_MS))
    {
        g_nav.line_ms = now;
        post(B5_CMD_LINE_FOUND, g_nav.req_id);
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

    tm_printf((UB *)"[b5-nav] MOTION FAULT %d: sample %d, elapsed %d us\n",
              (INT)p_status->control.fault,
              (INT)p_status->control.sample_available,
              (INT)p_status->control.sample.elapsed_us);

    for (wheel = 0U; wheel < WHEELS; wheel++)
    {
        motion_wheel_sample_t const * const p_wheel =
            &p_status->control.sample.wheel[wheel];

        tm_printf((UB *)"[b5-nav]   wheel %d: target %d um/s, duty %d, "
                        "speed %d um/s, speed_valid %d, odo_valid %d, "
                        "bad encoder steps %d\n",
                  (INT)wheel, (INT)p_status->control.target_um_s[wheel],
                  (INT)p_status->control.duty[wheel],
                  (INT)p_wheel->velocity_um_s, (INT)p_wheel->speed_valid,
                  (INT)p_wheel->odometry_valid, (INT)p_wheel->invalid_delta);

        /* Casts are safe: wheel is 0..1 and the flags are 0/1. */
        log_rec(B5_LOG_MFAULT, (uint8_t)wheel, (uint8_t)p_wheel->speed_valid,
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
            tm_printf((UB *)"[b5-nav] motion state=%d fault=%d hold=%d\n",
                      (INT)status.control.state, (INT)status.control.fault,
                      (INT)status.hold_mask);
            log_rec(B5_LOG_MOTION, (uint8_t)status.control.state,
                    (uint8_t)status.control.fault, (uint8_t)status.hold_mask, 0,
                    0, 0);
        }

        if (b_changed && (MOTION_FAULT == status.control.state))
        {
            report_fault(&status);
        }

        for (wheel = 0U; wheel < WHEELS; wheel++)
        {
            /* NOTE: odometry_valid latches false after one invalid encoder
             * step; the displacement is still counted, so keep using it. */
            if (status.control.sample_available)
            {
                g_nav.wheel_um[wheel] =
                    true_wheel_um(&status.control.sample, wheel);

                /* Cast is safe: a run is far shorter than 2147 km. */
                g_nav.wheel_mm[wheel] =
                    (int32_t)(g_nav.wheel_um[wheel] / UM_PER_MM);
            }
        }
    }
}

/*!
 * @brief Log a TRACE record every B5_LOG_TRACE_MS while Buddy 5 drives
 *        the car (BYPASS, REACQUIRE), and every B5_LOG_IDLE_TRACE_MS
 *        otherwise.
 *
 * @param[in] status  Packed Buddy 5 status word.
 * @param[in] b_left  Left IR sensor sees the line.
 * @param[in] b_right Right IR sensor sees the line.
 * @param[in] now     Current time, ms.
 */
static void trace (uint32_t status, bool b_left, bool b_right, uint32_t now)
{
    uint32_t const state = b5_pack_get(status, B5_ST_STATE_SH, B5_ST_STATE_MK);
    bool const     b_driving =
        (B5_STATE_BYPASS == state) || (B5_STATE_REACQUIRE == state);
    uint32_t const period_ms =
        b_driving ? B5_LOG_TRACE_MS : B5_LOG_IDLE_TRACE_MS;

    if ((now - g_nav.trace_ms) >= period_ms)
    {
        uint32_t const steer =
            b5_pack_get(obstacle_get_steer(), B5_EV_STEER_SH, B5_EV_STEER_MK);
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
        log_rec(B5_LOG_TRACE, (uint8_t)state,
                (uint8_t)b5_pack_get(status, B5_ST_CLASS_SH, B5_ST_CLASS_MK),
                (uint8_t)bits,
                (int32_t)b5_pack_get(status, B5_ST_DIST_SH, B5_ST_DIST_MK),
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
 *        (B5_ENCODERS_CROSSED).
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

#if (B5_ENCODERS_CROSSED)
    travel_um = -(p_sample->wheel[(WHEELS - 1U) - wheel].displacement_um);
#else
    travel_um = p_sample->wheel[wheel].displacement_um;
#endif

    return (travel_um);
}

/*** end of file ***/

