/** @file b5_nav.c
 *
 * @brief Buddy 5 Phase 2 glue between the obstacle statechart, the line
 *        task in main.c and the motion service.
 *
 * What the line task does with Buddy 5 (design review 4.1):
 *   - OBSTACLE      hold the car, wait B5_NAV_BRAKE_MS, send SCAN_REQ(id).
 *   - SCAN_RESULT   OK or CLEAR: send ACCEPT(id).
 *   - BYPASS and    turn each STEER word into a motion speed command;
 *     REACQUIRE     in REACQUIRE, send LINE_FOUND(id) when an IR sensor
 *                   sees the line (Buddy 5 ignores the first 200 mm).
 *   - FAULT         hold the car; retry RESET + START every 3 s.
 *   - MONITOR       release the hold and let the line follower drive,
 *                   except while Buddy 5's latest reading is a VALID echo
 *                   within the detection distance (no creeping forward
 *                   into an obstacle while a retry is in progress).
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
#include "b5_nav.h"
#include "b5_pack.h"
#include "obstacle.h"

/* Micrometres per millimetre. */
#define UM_PER_MM (1000)

/* Percent scale. */
#define PCT_FULL (100U)

/* The two wheels of the motion service. */
#define WHEEL_LEFT  (0U)
#define WHEEL_RIGHT (1U)
#define WHEELS      (2U)

/* Inside wheel speed during a steer: sharp while escaping, gentle while
 * hugging. */
#define NAV_ESCAPE_INSIDE_UM_S                                                 \
    ((B5_NAV_SPEED_UM_S * B5_NAV_ESCAPE_INSIDE_PCT) / PCT_FULL)
#define NAV_HUG_INSIDE_UM_S                                                    \
    ((B5_NAV_SPEED_UM_S * B5_NAV_HUG_INSIDE_PCT) / PCT_FULL)

/* Value that never matches a real state or steer, so the first one is
 * always printed. */
#define NAV_UNSEEN (0xFFU)

/* Navigation state; only the line task touches it. NOTE: Booleans are b_
 * members of a g_ struct, which satisfies both BARR-C 7.1.m/7.1.o and the
 * team .clang-tidy. */
typedef struct
{
    bool     b_enabled;
    bool     b_scan_pending;
    uint8_t  req_id;
    uint8_t  next_id;
    uint32_t hold_ms;
    uint32_t retry_ms;
    uint32_t line_ms;
    uint32_t motion_word;
    uint8_t  last_state;
    uint8_t  last_steer;
} nav_state_t;

/* Last good odometer reading; only the Buddy 5 task touches it. */
typedef struct
{
    uint32_t last_mm;
} nav_odo_t;

static nav_state_t g_nav = {false, false, 0U,         0U,         0U,
                            0U,    0U,    UINT32_MAX, NAV_UNSEEN, NAV_UNSEEN};
static nav_odo_t   g_odo = {0U};

static uint32_t now_ms (void);
static void     post (uint32_t type, uint8_t req_id);
static void     hold_car (bool b_held);
static void     drive_steer (int32_t inside_um_s);
static bool     is_obstacle_close (uint32_t status);
static void     report_state (uint32_t state);
static void     handle_event (uint32_t evt, uint32_t now);
static void     look_for_line (bool b_line_seen, uint32_t now);
static void     report_motion (void);
static ER       nav_stop (void);
static ER       nav_distance_mm (uint32_t * p_distance_mm);

/*!
 * @brief Start Buddy 5 with the motion links, then RESET and START it.
 *
 * Call once from the line task on processor 1, after gpio_irq_init(),
 * motion_service_start() and ir_capture_init(). If it fails, b5_nav_step()
 * always returns false, so line following works without Buddy 5.
 *
 * @return The result of obstacle_init().
 */
ER b5_nav_init (void)
{
    obstacle_links_t const links  = {0, nav_stop, nav_distance_mm};
    ER const               result = obstacle_init(&links);

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

    report_motion();
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
        report_state(state);
    }

    switch (state)
    {
        case B5_STATE_MONITOR:
            b_in_charge = is_obstacle_close(status);
            hold_car(b_in_charge);
        break;

        case B5_STATE_BYPASS:
            hold_car(false);
            drive_steer((int32_t)NAV_ESCAPE_INSIDE_UM_S);
            b_in_charge = true;
        break;

        case B5_STATE_REACQUIRE:
            hold_car(false);
            drive_steer((int32_t)NAV_HUG_INSIDE_UM_S);
            look_for_line(b_line_left || b_line_right, now);
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
 * @brief Post one command to Buddy 5 and print it.
 *
 * @param[in] type   Command type (B5_CMD_*).
 * @param[in] req_id Request id.
 */
static void post (uint32_t type, uint8_t req_id)
{
    ER const result = obstacle_post_cmd(b5_pack_cmd(type, req_id));

    tm_printf((UB *)"[b5-nav] cmd %d id=%d -> %d\n", (INT)type, (INT)req_id,
              (INT)result);
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
 * @brief Turn the latest STEER word into a speed command. The car curves
 *        by slowing the inside wheel [Chat].
 *
 * @param[in] inside_um_s Inside wheel speed during a steer (casts at the
 *                        callers are safe: both values are 0..40 000).
 */
static void drive_steer (int32_t inside_um_s)
{
    uint32_t const steer =
        b5_pack_get(obstacle_get_steer(), B5_EV_STEER_SH, B5_EV_STEER_MK);
    int32_t speed[WHEELS] = {0, 0};

    /* Casts are safe: both speeds are at most 40 000 um/s. */
    speed[WHEEL_LEFT]  = (int32_t)B5_NAV_SPEED_UM_S;
    speed[WHEEL_RIGHT] = (int32_t)B5_NAV_SPEED_UM_S;

    switch (steer)
    {
        case B5_STEER_STRAIGHT:
            /* Both wheels at full speed. */
        break;

        case B5_STEER_LEFT:
            speed[WHEEL_LEFT] = inside_um_s;
        break;

        case B5_STEER_RIGHT:
            speed[WHEEL_RIGHT] = inside_um_s;
        break;

        default:
            /* HOLD: stand still. */
            speed[WHEEL_LEFT]  = 0;
            speed[WHEEL_RIGHT] = 0;
        break;
    }

    if (g_nav.last_steer != steer)
    {
        /* Cast is safe: the steer field is 2 bits wide. */
        g_nav.last_steer = (uint8_t)steer;
        tm_printf((UB *)"[b5-nav] steer %d: left %d right %d um/s\n",
                  (INT)steer, (INT)speed[WHEEL_LEFT], (INT)speed[WHEEL_RIGHT]);
    }

    (void)motion_service_set_speed(speed[WHEEL_LEFT], speed[WHEEL_RIGHT],
                                   B5_NAV_TTL_US);
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
 * @brief Print each Buddy 5 state change.
 *
 * NOTE: States: 0 STOPPED, 1 MONITOR, 2 WAIT_STOP, 3 SCANNING, 4 PLAN,
 * 5 BYPASS, 6 REACQUIRE.
 *
 * @param[in] state Current Buddy 5 state.
 */
static void report_state (uint32_t state)
{
    if (g_nav.last_state != state)
    {
        /* Cast is safe: the state field is 3 bits wide. */
        g_nav.last_state = (uint8_t)state;
        g_nav.last_steer = NAV_UNSEEN;
        tm_printf((UB *)"[b5-nav] state %d\n", (INT)state);
    }
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
        tm_printf((UB *)"[b5-nav] OBSTACLE at %d mm: holding\n",
                  (INT)b5_pack_get(evt, B5_EV_DIST_SH, B5_EV_DIST_MK));
        hold_car(true);
        g_nav.hold_ms        = now;
        g_nav.req_id         = g_nav.next_id;
        g_nav.b_scan_pending = true;

        /* Cast is safe: the id is masked to 5 bits. */
        g_nav.next_id = (uint8_t)((g_nav.next_id + 1U) & B5_ID_MAX);
    }
    else if (B5_EVT_FAULT == type)
    {
        tm_printf((UB *)"[b5-nav] FAULT cause=%d: holding, retry in 3 s\n",
                  (INT)b5_pack_get(evt, B5_EV_CAUSE_SH, B5_EV_CAUSE_MK));
        hold_car(true);
        g_nav.b_scan_pending = false;
        g_nav.retry_ms       = now;
    }
    else if (B5_EVT_SCAN_RESULT == type)
    {
        tm_printf((UB *)"[b5-nav] SCAN_RESULT status=%d side=%d width=%d mm "
                        "start=%d deg\n",
                  (INT)status,
                  (INT)b5_pack_get(evt, B5_EV_SIDE_SH, B5_EV_SIDE_MK),
                  (INT)b5_pack_get(evt, B5_EV_WIDTH_SH, B5_EV_WIDTH_MK),
                  (INT)b5_pack_get(evt, B5_EV_ANGLE_SH, B5_EV_ANGLE_MK));
        if ((g_nav.req_id == req_id) && b_go)
        {
            post(B5_CMD_ACCEPT, g_nav.req_id);
        }
    }
    else
    {
        /* STEER words are read with obstacle_get_steer(). */
    }
}

/*!
 * @brief In REACQUIRE, tell Buddy 5 when an IR sensor sees the line.
 *
 * @param[in] b_line_seen An IR sensor sees the line.
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
 * @brief Print the motion service state, fault and hold mask whenever one
 *        of them changes, so a silent motor fault shows up in the log.
 *
 * NOTE: States: 0 IDLE, 1 RUNNING, 2 REVERSING, 3 STOPPED, 4 FAULT.
 * Faults: 0 NONE, 1 COMMAND, 2 EXPIRED, 3 FEEDBACK, 4 GAP, 5 NO_EDGES,
 * 6 OUTPUT. STOPPED and FAULT both need a reboot.
 */
static void report_motion (void)
{
    motion_service_status_t status;

    if (E_OK == motion_service_read_status(&status))
    {
        /* Casts are safe: the enums are 0..6 and the hold mask is 0..7. */
        uint32_t const word = ((uint32_t)status.control.state << 16U) |
                              ((uint32_t)status.control.fault << 8U) |
                              status.hold_mask;

        if (word != g_nav.motion_word)
        {
            g_nav.motion_word = word;
            tm_printf((UB *)"[b5-nav] motion state=%d fault=%d hold=%d\n",
                      (INT)status.control.state, (INT)status.control.fault,
                      (INT)status.hold_mask);
        }
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
        b_valid = status.control.sample_available &&
                  status.control.sample.wheel[WHEEL_LEFT].odometry_valid &&
                  status.control.sample.wheel[WHEEL_RIGHT].odometry_valid;
    }

    if (b_valid)
    {
        int64_t const sum_um =
            status.control.sample.wheel[WHEEL_LEFT].displacement_um +
            status.control.sample.wheel[WHEEL_RIGHT].displacement_um;
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

/*** end of file ***/

