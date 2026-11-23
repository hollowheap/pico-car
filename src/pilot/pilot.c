/** @file pilot.c
 * @brief Pilot core: safety stop and line follow, in priority order.
 *
 * Each tick produces one wheel command. Safety takes priority over obstacle
 * navigation, tape reacquisition and ordinary line following.
 */

#include "pilot/pilot.h"

#include <stddef.h>

#include "navigation/obstacle_ping.h"
#include "platform/chassis.h"
#include "platform/units.h"
#include "geometry.h"

#define HALF_DIVISOR         (2)
#define REAPPROACH_CAP_UM_S  (70 * UM_PER_MM)

#define COURSE_SCAN_IDLE      (0U)
#define COURSE_SCAN_SETTLE    (1U)
#define COURSE_SCAN_FIRST     (2U)
#define COURSE_SCAN_RETRY_MS  (320U)
#define COURSE_SCAN_REPLY_MS  (300U)
#define COURSE_CLEAR_PINGS    (3U)
#define COURSE_STOP_MM        (100U)
#define DATA_RECOVERY_PINGS   (3U)
#define COURSE_INNER_DECI     (OBSTACLE_SERVO_LIMIT_DECI / HALF_DIVISOR)
static const int16_t g_course_scan_angles[][3] = {
    {-OBSTACLE_SERVO_LIMIT_DECI, 0, OBSTACLE_SERVO_LIMIT_DECI},
    {COURSE_INNER_DECI, 0, -COURSE_INNER_DECI},
};
#define COURSE_SCAN_LIMIT (sizeof(g_course_scan_angles) / sizeof(g_course_scan_angles[0]))
#define COURSE_SCAN_END (COURSE_SCAN_FIRST + sizeof(g_course_scan_angles[0]) / sizeof(g_course_scan_angles[0][0]))

STATIC_ASSERT(OBSTACLE_CONSIST_TRAVEL_MM > (OBSTACLE_CONSIST_TOL_MM + OBSTACLE_AGREE_MM),
              "a stable range must disagree with travel by more than the sonar tolerance");

static void set_cmd(pilot_cmd_t * p_cmd, int32_t speed_mm_s,
                    int32_t turn_deci_s, pilot_behaviour_t behaviour,
                    pilot_reason_t reason)
{
    p_cmd->speed_mm_s  = (int16_t)clamp_i32(speed_mm_s, -PILOT_SPEED_MAX_MM_S,
                                            PILOT_SPEED_MAX_MM_S);
    p_cmd->turn_deci_s = (int16_t)clamp_i32(turn_deci_s, -PILOT_TURN_MAX_DECI_S,
                                            PILOT_TURN_MAX_DECI_S);
    p_cmd->behaviour   = (uint8_t)behaviour;
    p_cmd->reason      = (uint8_t)reason;
    p_cmd->ping_servo_deci = 0;
    p_cmd->ping_seq    = 0U;
    p_cmd->b_ping_req  = 0U;
    p_cmd->contact_mm  = 0U;
    p_cmd->b_scan_pivot = false;
    p_cmd->b_stop_left = false;
    p_cmd->b_stop_right = false;
    p_cmd->b_rejoined = false;
    p_cmd->course_scan_samples = 0U;
}

#define FLAG_MOVING          (0x01UL)  /* the last command asked for motion       */
#define FLAG_LOW_LEFT        (0x02UL)  /* left wheel commanded below the minimum  */
#define FLAG_LOW_RIGHT       (0x04UL)  /* right wheel commanded below the minimum */
#define FLAG_REVERSE         (0x08UL)  /* last command: line reverse */
/* A run is a stretch of sonar fault or stale data. */
#define FLAG_SONAR_BAD       (0x10UL)  /* a run is open                    */
#define FLAG_SONAR_LATCH     (0x20UL)  /* the run outlasted the window     */
#define FLAG_SONAR_STALE     (0x40UL)  /* the most recent bad cause was STALE */
#define FLAG_PING_REQUEST    (0x80UL)  /* the last command requested a ping   */
#define FLAGS_SONAR (FLAG_SONAR_BAD | FLAG_SONAR_LATCH | FLAG_SONAR_STALE)
#define FORWARD_SEQ_UNSEEN   (UINT8_MAX)
#define WINDOW_MAX_UM        (0x7FFFFFFFUL)

static void flag_set(pilot_state_t * p_state, uint32_t mask, bool b_on)
{
    uint32_t flags = p_state->flags;

    flags = b_on ? (flags | mask) : (flags & ~mask);
    p_state->flags = (uint8_t)flags;
}

static bool flag_get(pilot_state_t const * p_state, uint32_t mask)
{
    return 0U != ((uint32_t)p_state->flags & mask);
}

static void window_start(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                         pilot_phase_t phase)
{
    p_state->ref_left_um  = (uint32_t)p_meas->left_um;
    p_state->ref_right_um = (uint32_t)p_meas->right_um;
    p_state->since_ms     = p_meas->now_ms;
    p_state->phase        = (uint8_t)phase;
}

static bool backing_off(uint8_t phase)
{
    return ((uint8_t)PILOT_PHASE_BACKOFF == phase) ||
           ((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL == phase);
}

/* Travel since the start of the window as the unsigned difference of the low
 * 32 bits: right across the wrap, and no signed conversion. A value above
 * WINDOW_MAX_UM is travel backward. */
static uint32_t travel_um(int64_t now_um, uint32_t ref_um)
{
    return (uint32_t)now_um - ref_um;
}

static bool wheel_moved(uint32_t um)
{
    uint32_t const back_um = 0U - um;

    return (um >= (uint32_t)PILOT_MOVE_MIN_UM) &&
           (back_um >= (uint32_t)PILOT_MOVE_MIN_UM);
}

static bool wheel_back(uint32_t um, uint32_t min_um)
{
    uint32_t const back_um = 0U - um;

    return (back_um >= min_um) && (back_um <= WINDOW_MAX_UM);
}

static void course_stop_and_scan(pilot_state_t * p_state, uint32_t now_ms);

/* An unchanged range is evidence only during straight forward travel. Let
 * the normal hug handle nearer contacts and keep blind stalls separate. */
static bool slip_detected(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                          bool b_fresh_forward)
{
    bool b_slip = false;

    if (!p_state->b_slip_forward || p_meas->b_ping_reply || (0 != p_meas->ping_servo_deci) ||
        (LOOK_ECHO_VALID != p_meas->ping_echo) || (p_meas->ping_mm <= HUG_STANDOFF_MM) ||
        (p_meas->ping_mm >= OBSTACLE_CONSIST_RANGE_MM))
    {
        p_state->slip_range_mm = 0U;
    }
    else if (b_fresh_forward)
    {
        uint32_t const dl = travel_um(p_meas->left_um, p_state->slip_left_um);
        uint32_t const dr = travel_um(p_meas->right_um, p_state->slip_right_um);
        int32_t const diff_mm = (int32_t)p_meas->ping_mm - (int32_t)p_state->slip_range_mm;
        int32_t const heading = line_heading_deci(dl, dr);

        if ((0U == p_state->slip_range_mm) || (dl > WINDOW_MAX_UM) || (dr > WINDOW_MAX_UM) ||
            (heading < -HUG_HEAD_TOL_DECI) || (heading > HUG_HEAD_TOL_DECI) ||
            (diff_mm < -(int32_t)OBSTACLE_AGREE_MM) || (diff_mm > (int32_t)OBSTACLE_AGREE_MM))
        {
            p_state->slip_left_um = (uint32_t)p_meas->left_um;
            p_state->slip_right_um = (uint32_t)p_meas->right_um;
            p_state->slip_range_mm = p_meas->ping_mm;
        }
        else
        {
            b_slip = (dl >= (OBSTACLE_CONSIST_TRAVEL_MM * UM_PER_MM)) &&
                     (dr >= (OBSTACLE_CONSIST_TRAVEL_MM * UM_PER_MM));
        }
    }
    else
    {
        /* A cached ping cannot confirm a plateau. */
    }
    return b_slip;
}

/* A stall during line following supplies no obstacle range. Only an active
 * obstacle maneuver has the contact evidence needed for a recovery reverse. */
static bool behaviour_stuck(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                            bool b_fresh_forward, pilot_cmd_t * p_cmd)
{
    bool b_takes = false;
    uint32_t const dl = travel_um(p_meas->left_um, p_state->ref_left_um);
    uint32_t const dr = travel_um(p_meas->right_um, p_state->ref_right_um);
    bool const b_left = wheel_moved(dl);
    bool const b_right = wheel_moved(dr);
    uint32_t const age_ms = p_meas->now_ms - p_state->since_ms;

    if (!flag_get(p_state, FLAG_MOVING))
    {
        /* Nothing was commanded: the window starts again, and a retry stays one. */
        window_start(p_state, p_meas,
                     ((uint8_t)PILOT_PHASE_RETRY == p_state->phase) ? PILOT_PHASE_RETRY
                                                                    : PILOT_PHASE_TRY);
    }
    else if (!backing_off(p_state->phase))
    {
        /* A wheel commanded below the minimum may rightly stand still. */
        bool const b_need_l = !flag_get(p_state, FLAG_LOW_LEFT);
        bool const b_need_r = !flag_get(p_state, FLAG_LOW_RIGHT);
        bool const b_progress_l = b_left && (!p_state->b_follow_forward || (dl <= WINDOW_MAX_UM));
        bool const b_progress_r = b_right && (!p_state->b_follow_forward || (dr <= WINDOW_MAX_UM));
        bool const b_slip = slip_detected(p_state, p_meas, b_fresh_forward);
        /* The next commanded interval can confirm a silent encoder without
         * backing up just to test it. An intentionally idle wheel is exempt. */
        p_state->b_check_left = p_state->b_check_left && !b_left;
        p_state->b_check_right = p_state->b_check_right && !b_right;
        if (age_ms >= PILOT_STUCK_MS)
        {
            if (b_need_l && b_need_r && (b_left != b_right) &&
                (p_state->b_check_left || p_state->b_check_right))
            {
                p_state->b_encoder = 1U;
            }
            p_state->b_check_left = b_need_l && !b_left;
            p_state->b_check_right = b_need_r && !b_right;
        }
        if (!b_slip && (b_progress_l || !b_need_l) && (b_progress_r || !b_need_r))
        {
            window_start(p_state, p_meas, PILOT_PHASE_TRY);
        }
        else if ((age_ms >= PILOT_STUCK_MS) && flag_get(p_state, FLAG_REVERSE))
        {
            /* The back-off assumes the blockage is ahead. A blocked reverse is
             * ended by the follower; backing off would push further into it. */
            window_start(p_state, p_meas, PILOT_PHASE_TRY);
        }
        else if (b_slip || (age_ms >= PILOT_STUCK_MS))
        {
            if (p_meas->b_bounded_course)
            {
                p_state->b_course_stuck = true;
            }
            else if (hug_active(&p_state->hug))
            {
                window_start(p_state, p_meas,
                             ((b_left != b_right) && b_need_l && b_need_r)
                                 ? PILOT_PHASE_BACKOFF_ONE_WHEEL
                                 : PILOT_PHASE_BACKOFF);
            }
            else
            {
                course_stop_and_scan(p_state, p_meas->now_ms);
            }
        }
    }
    else if (wheel_back(dl, (uint32_t)PILOT_BACKOFF_UM) ||
             wheel_back(dr, (uint32_t)PILOT_BACKOFF_UM) ||
             (age_ms >= PILOT_BACKOFF_MAX_MS))
    {
        if (((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL == p_state->phase) && (b_left != b_right))
        {
            p_state->b_encoder = 1U;
        }
        window_start(p_state, p_meas, PILOT_PHASE_RETRY);
    }
    else
    {
        /* Still backing off. */
    }
    if (0U != p_state->b_encoder)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ENCODER);
        b_takes = true;
    }
    else if (backing_off(p_state->phase))
    {
        set_cmd(p_cmd, -PILOT_BACKOFF_MM_S, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_STUCK);
        b_takes = true;
    }
    else
    {
        /* Drive: the follower runs. */
    }
    return b_takes;
}

/* Request replies use a different sequence to the free-running stream. A
 * requested look pauses the forward deadline while the servo is occupied. */
static bool fresh_forward_ping(pilot_state_t const * p_state, pilot_meas_t const * p_meas)
{
    bool const b_seen = (FORWARD_SEQ_UNSEEN != p_state->forward_ping_seq);

    return !p_meas->b_ping_reply && (0 == p_meas->ping_servo_deci) &&
           (p_meas->ping_seq != p_state->forward_ping_seq) &&
           (b_seen || (0U != p_meas->ping_seq) ||
            (LOOK_ECHO_OPEN == p_meas->ping_echo) || (0U != p_meas->ping_mm));
}

static bool forward_ping_stale(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                               bool b_fresh_forward)
{
    if (b_fresh_forward)
    {
        uint8_t const next_seq = (uint8_t)((p_state->forward_ping_seq + 1U) & OBSTACLE_PING_SEQ_MK);

        /* The sonar publishes only its latest reading. A sequence gap can hide
         * an obstruction, and after a long pause the sequence can wrap. */
        if ((p_meas->ping_seq != next_seq) ||
            ((p_meas->now_ms - p_state->forward_ping_ms) >= PILOT_FORWARD_PING_STALE_MS))
        {
            p_state->course_clear_count = 0U;
            p_state->data_recovery_pings = 0U;
        }
        p_state->forward_ping_seq = p_meas->ping_seq;
        p_state->forward_ping_ms  = p_meas->now_ms;
        p_state->b_forward_timer_started = true;
    }
    else if (!p_state->b_forward_timer_started && !p_meas->b_held &&
             !p_meas->b_sonar_fault &&
             ((p_meas->now_ms - p_meas->stamp_ms) <= PILOT_STALE_MS))
    {
        /* Start the grace period when the other drive inputs become ready. */
        p_state->forward_ping_ms = p_meas->now_ms;
        p_state->b_forward_timer_started = true;
    }
    else if ((FORWARD_SEQ_UNSEEN != p_state->forward_ping_seq) &&
             flag_get(p_state, FLAG_PING_REQUEST))
    {
        p_state->forward_ping_ms = p_meas->now_ms;
    }

    return !p_meas->b_held && p_state->b_forward_timer_started &&
           ((p_meas->now_ms - p_state->forward_ping_ms) >= PILOT_FORWARD_PING_STALE_MS);
}

/* The stop for a sonar fault or stale data: its reason while the run
 * lasts, and while confirming recovery once the run has outlasted
 * PILOT_SONAR_RESUME_MS. The latched reason keeps the cause of the moment the
 * latch sets. Stale feedback while motion holds the wheels is no run: HELD
 * answers. The run starts at since_ms: a stop is no motion, so the stuck window
 * starts again after it and does not need the value. */
static pilot_reason_t data_stop(pilot_state_t * p_state,
                                pilot_meas_t const * p_meas, bool b_fresh_forward)
{
    /* The unsigned difference is right across the wrap of the clock. */
    bool const b_forward_stale = forward_ping_stale(p_state, p_meas, b_fresh_forward);
    bool const b_sonar = p_meas->b_sonar_fault || b_forward_stale;
    bool const b_stale = !p_meas->b_held &&
                         ((p_meas->now_ms - p_meas->stamp_ms) > PILOT_STALE_MS);
    bool const b_bad = b_sonar || b_stale;
    pilot_reason_t reason = PILOT_REASON_NONE;

    if (b_bad && !flag_get(p_state, FLAG_SONAR_LATCH))
    {
        flag_set(p_state, FLAG_SONAR_STALE, !b_sonar);
    }
    if (flag_get(p_state, FLAG_SONAR_BAD))
    {
        uint32_t const age_ms = p_meas->now_ms - p_state->since_ms;

        if ((b_bad && (age_ms >= PILOT_SONAR_RESUME_MS)) ||
            (!p_meas->b_held && (age_ms > PILOT_SONAR_RESUME_MS)))
        {
            flag_set(p_state, FLAG_SONAR_LATCH, true);
        }
        flag_set(p_state, FLAG_SONAR_BAD, b_bad);
    }
    else if (b_bad)
    {
        flag_set(p_state, FLAG_SONAR_BAD, true);
        p_state->since_ms = p_meas->now_ms;
    }
    else
    {
        /* No open dropout. */
    }
    if (flag_get(p_state, FLAG_SONAR_LATCH))
    {
        if (b_bad || p_meas->b_held)
        {
            p_state->data_recovery_pings = 0U;
        }
        else if (b_fresh_forward)
        {
            bool const b_usable = (LOOK_ECHO_OPEN == p_meas->ping_echo) ||
                (look_echo_is_point(p_meas->ping_echo) && (0U != p_meas->ping_mm));

            p_state->data_recovery_pings = b_usable ? (uint8_t)(p_state->data_recovery_pings + 1U) : 0U;
            if (p_state->data_recovery_pings >= DATA_RECOVERY_PINGS)
            {
                pilot_release_sonar(p_state);
                return PILOT_REASON_NONE;
            }
        }
        else
        {
            /* A duplicate ping cannot advance recovery. */
        }
        reason = flag_get(p_state, FLAG_SONAR_STALE)
                     ? PILOT_REASON_STALE_LATCHED
                     : PILOT_REASON_SONAR_LATCHED;
    }
    else if (b_bad)
    {
        reason = b_sonar ? PILOT_REASON_SONAR : PILOT_REASON_STALE;
    }
    else
    {
        /* Good data. */
    }
    return reason;
}

static bool behaviour_safety(pilot_state_t * p_state,
                             pilot_meas_t const * p_meas, bool b_fresh_forward,
                             pilot_cmd_t * p_cmd)
{
    pilot_reason_t const data = data_stop(p_state, p_meas, b_fresh_forward);
    bool b_takes = false;

    if (0U != p_state->b_estop)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ESTOP);
        b_takes = true;
    }
    else if (0U != p_state->b_encoder)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ENCODER);
        b_takes = true;
    }
    else if (0U != p_state->b_timeout)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HUG_TIMEOUT);
        b_takes = true;
    }
    else if ((PILOT_REASON_SONAR == data) ||
             (PILOT_REASON_SONAR_LATCHED == data))
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, data);
        b_takes = true;
    }
    /* Motion refuses commands while it holds the car: no travel is no proof
     * of a stuck car, so the stuck window restarts, as for stale data. */
    else if (p_meas->b_held)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HELD);
        p_state->phase = (uint8_t)PILOT_PHASE_TRY;
        b_takes = true;
    }
    else if ((PILOT_REASON_STALE == data) ||
             (PILOT_REASON_STALE_LATCHED == data))
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, data);
        p_state->phase = (uint8_t)PILOT_PHASE_TRY;
        b_takes = true;
    }
    else if (p_state->b_course_stuck)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
        b_takes = true;
    }
    else if (!p_meas->b_commanded_turn && !line_follow_local_search(&p_state->follow))
    {
        b_takes = behaviour_stuck(p_state, p_meas, b_fresh_forward, p_cmd);
        if (p_state->b_course_stuck || (p_meas->b_bounded_course && backing_off(p_state->phase)))
        {
            /* The obstacle recovery reverse has no floor evidence behind
             * the caster. A clear forward beam cannot diagnose a jam. */
            p_state->b_course_stuck = true;
            p_state->phase = (uint8_t)PILOT_PHASE_TRY;
            set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
            b_takes = true;
        }
    }
    else
    {
        /* An optical search beyond a box and a commanded turn each limit
         * their own pivot. A stall must not start a reverse beside the box. */
    }
    return b_takes;
}

static void behaviour_follow_line(pilot_state_t * p_state,
                                  pilot_meas_t const * p_meas,
                                  pilot_cmd_t * p_cmd)
{
    line_targets_t targets = {0, 0};
    line_follow_status_t status = (line_follow_status_t)p_state->follow.status;
    int64_t const travel_um = (p_meas->left_um + p_meas->right_um) / HALF_DIVISOR;
    int32_t const heading = line_heading_deci(p_meas->left_um, p_meas->right_um);
    if (!p_meas->b_barcode || !line_follow_marker_sample(&p_state->follow, p_meas->b_line_left,
            p_meas->b_line_right, p_meas->b_marker_progress, p_meas->marker_prefix_mm, heading, travel_um,
            p_meas->now_ms, &targets))
    {
        status = line_follow_update(
            &p_state->follow, p_meas->b_line_left, p_meas->b_line_right,
            heading, travel_um, p_meas->now_ms,
            &targets);
    }

    bool const b_ranged = (!p_meas->b_ping_reply) && (p_meas->ping_mm > 0U) &&
                          look_echo_is_point(p_meas->ping_echo);

    line_targets_cap(&targets, line_bumper_cap_um_s(b_ranged, p_meas->ping_mm));
    if (((targets.left_um_s + targets.right_um_s) > 0) && hug_reapproaching(&p_state->hug))
    {
        line_targets_cap(&targets, REAPPROACH_CAP_UM_S);
    }
    p_state->line_radius_mm = ((LINE_FOLLOW_FOLLOWING == status) &&
        (p_meas->b_line_left != p_meas->b_line_right)) ?
        drive_forward_radius_mm(targets.left_um_s, targets.right_um_s) : 0;
    if (LINE_FOLLOW_STOPPED == status)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_FOLLOW_LINE,
                PILOT_REASON_LINE_LOST_STOP);
    }
    else
    {
        int32_t const speed_um_s = (targets.left_um_s + targets.right_um_s) /
                                   HALF_DIVISOR;
        int32_t const diff_um_s = targets.right_um_s - targets.left_um_s;

        set_cmd(p_cmd, speed_um_s / UM_PER_MM,
                (int32_t)wheel_yaw_deci(diff_um_s),
                PILOT_BEHAVIOUR_FOLLOW_LINE, PILOT_REASON_NONE);
        p_cmd->b_stop_left = targets.left_um_s == 0;
        p_cmd->b_stop_right = targets.right_um_s == 0;
    }
}

static void course_forget_survey(pilot_state_t * p_state)
{
    look_clear(&p_state->look);
    p_state->course_side = 0;
    p_state->b_course_anchor = false;
}

void pilot_init(pilot_state_t * p_state)
{
    p_state->b_follow_forward = false;
    line_follow_init(&p_state->follow);
    p_state->line_radius_mm = 0;
    p_state->b_estop    = 0U;
    p_state->b_encoder  = 0U;
    p_state->b_timeout  = 0U;
    p_state->flags      = 0U;
    p_state->phase      = (uint8_t)PILOT_PHASE_TRY;
    p_state->since_ms   = 0U;
    p_state->forward_ping_ms  = 0U;
    p_state->forward_ping_seq = FORWARD_SEQ_UNSEEN;
    p_state->b_forward_timer_started = false;
    p_state->ref_left_um  = 0U;
    p_state->ref_right_um = 0U;
    p_state->slip_left_um = 0U;
    p_state->slip_right_um = 0U;
    p_state->slip_range_mm = 0U;
    p_state->b_slip_forward = false;
    p_state->b_course_hold = false;
    p_state->b_check_left = false;
    p_state->b_check_right = false;
    p_state->b_course_stuck = false;
    p_state->exit_width_mm = 0U;
    p_state->exit_breadth_mm = 0U;
    course_forget_survey(p_state);
    p_state->course_left_um = 0U;
    p_state->course_right_um = 0U;
    p_state->course_scan_ms = 0U;
    p_state->course_scan_step = COURSE_SCAN_IDLE;
    p_state->course_scan_seq = 0U;
    p_state->course_clear_count = 0U;
    p_state->course_scan_attempts = 0U;
    p_state->data_recovery_pings = 0U;
    hug_init(&p_state->hug);
}

/* Note which wheels the last command asks to run below the minimum speed. */
static void note_wheels(pilot_state_t * p_state, pilot_cmd_t const * p_cmd)
{
    int32_t const speed = p_cmd->speed_mm_s;
    int32_t const half = ((int32_t)p_cmd->turn_deci_s * CHASSIS_WHEEL_TRACK_MM) /
                         (DECI_PER_RAD * HALF_DIVISOR);
    int32_t const left = speed - half;
    int32_t const right = speed + half;

    flag_set(p_state, FLAG_LOW_LEFT,
             (left < PILOT_WHEEL_MIN_MM_S) && (left > -PILOT_WHEEL_MIN_MM_S));
    flag_set(p_state, FLAG_LOW_RIGHT,
             (right < PILOT_WHEEL_MIN_MM_S) && (right > -PILOT_WHEEL_MIN_MM_S));
}

static uint16_t traced_radius_mm(pilot_state_t const * p_state, pilot_meas_t const * p_meas)
{
    int32_t const side = (int32_t)p_meas->b_line_left - (int32_t)p_meas->b_line_right;
    int32_t const measured = side * p_meas->wheel_radius_mm;
    int32_t const requested = side * p_state->line_radius_mm;
    if ((requested <= 0) || (measured <= 0) || p_meas->b_commanded_turn ||
        (0U != (p_state->follow.flags & LINE_FOLLOW_HUMP)))
    {
        return 0U;
    }
    return (uint16_t)((measured > requested) ? measured : requested);
}

/* The hug watches the forward pings and tracks the pose on every tick; it
 * takes the wheels and the sonar when it has something to go round. */
static bool behaviour_hug(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                          bool b_other, bool b_stall, bool b_relook, pilot_cmd_t * p_cmd)
{
    hug_in_t in;
    hug_out_t out;
    bool const b_was = hug_active(&p_state->hug);
    uint16_t const width = p_state->hug.width_mm;
    uint16_t const breadth = p_state->hug.breadth_mm;
    bool b_takes = false;

    in.left_um = p_meas->left_um;
    in.right_um = p_meas->right_um;
    in.now_ms = p_meas->now_ms;
    in.ping_mm = p_meas->ping_mm;
    in.bend_radius_mm = traced_radius_mm(p_state, p_meas);
    in.ping_servo_deci = p_meas->ping_servo_deci;
    in.ping_echo = p_meas->ping_echo;
    in.ping_seq = p_meas->ping_seq;
    in.b_ping_reply = p_meas->b_ping_reply;
    in.b_line_left = p_meas->b_line_left;
    in.b_line_right = p_meas->b_line_right;
    in.b_stall = b_stall;
    in.b_other = b_other;
    in.b_contact_backoff = b_other && ((uint8_t)PILOT_REASON_STUCK == p_cmd->reason);
    in.b_stationary = p_meas->b_stationary;
    in.b_relook = b_relook;
    b_takes = hug_step(&p_state->hug, &p_state->look, &in, &out);
    if (out.b_no_tape || out.b_rejoined)
    {
        p_state->exit_width_mm = width;
        p_state->exit_breadth_mm = breadth;
    }
    if (0U != out.b_timeout)
    {
        /* The car is beside the obstacle and off the line: the follower must
         * not take over, so the stop holds. */
        p_state->b_timeout = 1U;
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HUG_TIMEOUT);
        b_takes = true;
    }
    else if (out.b_rejoined)
    {
        line_follow_init(&p_state->follow);
        behaviour_follow_line(p_state, p_meas, p_cmd);
        p_cmd->b_rejoined = true;
        b_takes = true;
    }
    else if (out.b_no_tape || out.b_no_exit)
    {
        line_follow_start_local_search(&p_state->follow);
        if (out.b_no_exit)
        {
            line_follow_stop_local_search(&p_state->follow);
        }
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_REJOIN_SEARCH,
                out.b_no_exit ? PILOT_REASON_REJOIN_STOP : PILOT_REASON_REJOIN_SEARCH);
        b_takes = true;
    }
    else if (b_takes)
    {
        set_cmd(p_cmd, out.speed_mm_s, out.turn_deci_s, PILOT_BEHAVIOUR_HUG,
                (0U != out.b_stop) ? PILOT_REASON_SONAR : PILOT_REASON_NONE);
        p_cmd->ping_servo_deci = out.servo_deci;
        p_cmd->ping_seq = out.seq;
        p_cmd->b_ping_req = out.b_req;
        p_cmd->b_scan_pivot = out.b_scan_pivot;
        if (!b_was && hug_active(&p_state->hug) && !b_stall && !b_relook &&
            !p_meas->b_ping_reply && (0 == p_meas->ping_servo_deci) &&
            (p_meas->ping_mm <= HUG_STANDOFF_MM) && look_echo_is_point(p_meas->ping_echo))
        {
            p_cmd->contact_mm = p_meas->ping_mm;
        }
    }
    else if (b_was && !hug_active(&p_state->hug))
    {
        /* The hug gave the wheels back: the follower starts afresh. */
        line_follow_init(&p_state->follow);
    }
    else
    {
        /* Not the hug's tick. */
    }
    return b_takes;
}

static void course_stop_and_scan(pilot_state_t * p_state, uint32_t now_ms)
{
    p_state->phase = (uint8_t)PILOT_PHASE_TRY;
    p_state->b_course_hold = true;
    course_forget_survey(p_state);
    p_state->course_scan_ms = now_ms;
    p_state->course_scan_step = COURSE_SCAN_SETTLE;
    p_state->course_clear_count = 0U;
    p_state->course_scan_attempts = 0U;
}

static bool behaviour_course_contact(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                                     bool b_safe, bool b_fresh_forward, bool b_relook,
                                     pilot_cmd_t * p_cmd)
{
    bool const b_contact = b_fresh_forward && (0U != p_meas->ping_mm) &&
        ((LOOK_ECHO_TOO_CLOSE == p_meas->ping_echo) ||
         ((LOOK_ECHO_VALID == p_meas->ping_echo) && (p_meas->ping_mm <= COURSE_STOP_MM)));

    if (!b_contact && (b_safe || !b_relook))
    {
        return false;
    }

    course_stop_and_scan(p_state, p_meas->now_ms);
    /* The hold can outlast a lift or calibration. Preserve the stronger
     * stop on this tick and remember the range for the release tick. */
    if (!b_safe)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_RECOVERY);
    }
    if (b_contact && (LOOK_ECHO_VALID == p_meas->ping_echo))
    {
        p_cmd->contact_mm = p_meas->ping_mm;
    }
    return true;
}

static bool course_hug_held(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                            bool b_fresh_forward, bool b_motion_ready, bool * p_start_contact)
{
    bool const b_moved = p_state->b_course_anchor &&
        ((p_state->course_left_um != (uint32_t)p_meas->left_um) ||
         (p_state->course_right_um != (uint32_t)p_meas->right_um));

    if (p_state->b_course_hold && (!b_motion_ready || !p_meas->b_stationary || b_moved))
    {
        course_forget_survey(p_state);
        if (!b_motion_ready)
        {
            p_state->course_scan_step = (p_state->course_scan_attempts < COURSE_SCAN_LIMIT)
                                            ? COURSE_SCAN_SETTLE : COURSE_SCAN_IDLE;
        }
        else if (p_state->course_scan_step >= COURSE_SCAN_FIRST)
        {
            /* Discard the outstanding reply. New centered readings can
             * establish clearance at the new stationary position. */
            p_state->course_scan_step = COURSE_SCAN_IDLE;
        }
        p_state->course_scan_ms = p_meas->now_ms;
        p_state->course_clear_count = 0U;
    }
    else if (p_state->b_course_hold && (COURSE_SCAN_IDLE == p_state->course_scan_step) &&
             b_fresh_forward && (p_meas->b_bounded_course ||
             (p_state->look.count == (COURSE_SCAN_END - COURSE_SCAN_FIRST))))
    {
        if (!p_meas->b_bounded_course)
        {
            for (uint8_t i = 0U; i < p_state->look.count; i++)
            {
                look_ray_t const * p_ray = &p_state->look.rays[i];
                if (look_echo_is_point(look_ray_echo(p_ray)) && (p_ray->range_mm <= HUG_STANDOFF_MM))
                {
                    *p_start_contact = true;
                }
            }
            if (*p_start_contact || (LOOK_ECHO_TOO_CLOSE == p_meas->ping_echo) ||
                ((LOOK_ECHO_VALID == p_meas->ping_echo) && (p_meas->ping_mm > 0U) &&
                 (p_meas->ping_mm <= HUG_STANDOFF_MM)))
            {
                *p_start_contact = true;
                p_state->b_course_hold = false;
                course_forget_survey(p_state);
                return false;
            }
        }
        bool const b_clear = (LOOK_ECHO_OPEN == p_meas->ping_echo) ||
            ((LOOK_ECHO_VALID == p_meas->ping_echo) &&
             (p_meas->ping_mm > COURSE_STOP_MM));

        p_state->course_clear_count = b_clear ? (uint8_t)(p_state->course_clear_count + 1U) : 0U;
        if (b_clear)
        {
            /* Give each fresh clear ping time to be confirmed before the servo leaves centre again. */
            p_state->course_scan_ms = p_meas->now_ms;
        }
        if (p_state->course_clear_count >= COURSE_CLEAR_PINGS)
        {
            p_state->b_course_hold = false;
            course_forget_survey(p_state);
        }
    }
    else if (p_state->b_course_hold && !p_meas->b_bounded_course &&
             (COURSE_SCAN_IDLE == p_state->course_scan_step) &&
             (p_state->course_scan_attempts == COURSE_SCAN_LIMIT) && b_fresh_forward &&
             (p_meas->now_ms - p_state->course_scan_ms >= COURSE_SCAN_RETRY_MS) &&
             ((LOOK_ECHO_OPEN == p_meas->ping_echo) ||
              (look_echo_is_point(p_meas->ping_echo) && (0U != p_meas->ping_mm))))
    {
        /* The centre is answering again. Reacquire the missing side rays
         * at rest before handing any wheel targets back to navigation. */
        course_stop_and_scan(p_state, p_meas->now_ms);
    }
    return p_state->b_course_hold;
}

/* A second pass samples the observed side at new bearings between the
 * forward ray and the servo limit. No return here measures floor support. */
static int16_t course_scan_angle(pilot_state_t const * p_state)
{
    uint32_t const ray = p_state->course_scan_step - COURSE_SCAN_FIRST;

    if ((p_state->course_scan_attempts > 1U) && (0 != p_state->course_side))
    {
        return (int16_t)(p_state->course_side * ((int32_t)ray + 1) * (COURSE_INNER_DECI / HALF_DIVISOR));
    }
    return g_course_scan_angles[p_state->course_scan_attempts - 1U][ray];
}

static int8_t course_edge_side(look_t const * p_look)
{
    look_pose_t const from = {0};
    look_face_t face;
    look_edge_t edges[HUG_SIDES];
    bool have[HUG_SIDES];
    uint16_t near_mm = UINT16_MAX;
    int8_t side = 0;

    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];

        if (look_echo_is_point(look_ray_echo(p_ray)) && (p_ray->range_mm < near_mm))
        {
            near_mm = p_ray->range_mm;
        }
    }
    /* A far wall cannot participate in fitting the contacted face. Its real
     * range is still checked against the fitted face by look_edges(). */
    if ((near_mm <= COURSE_STOP_MM) &&
        look_face_near(p_look, &from, (uint16_t)(near_mm + LOOK_EDGE_STEP_MM), &face))
    {
        look_edges(p_look, &face, edges, have);
        if (have[LOOK_SIDE_LEFT] && (!have[LOOK_SIDE_RIGHT] ||
            (geom_abs(edges[LOOK_SIDE_LEFT].y_mm) <= geom_abs(edges[LOOK_SIDE_RIGHT].y_mm))))
        {
            side = 1;
        }
        else if (have[LOOK_SIDE_RIGHT])
        {
            side = -1;
        }
    }
    return side;
}

/* All combined rays have the same axle origin and heading. A change in
 * either encoder or a motion hold retires the entire contact-local frame. */
static void course_scan_tick(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                             pilot_cmd_t * p_cmd)
{
    uint32_t const age_ms = p_meas->now_ms - p_state->course_scan_ms;
    uint8_t const step = p_state->course_scan_step;

    if ((COURSE_SCAN_SETTLE == step) || (COURSE_SCAN_IDLE == step))
    {
        uint32_t const wait_ms = (COURSE_SCAN_SETTLE == step) ? HUG_SETTLE_MS : COURSE_SCAN_RETRY_MS;

        if ((p_state->course_scan_attempts < COURSE_SCAN_LIMIT) && p_meas->b_stationary && (age_ms >= wait_ms))
        {
            /* An interrupted attempt uses its slot. Once spent, leave the
             * servo to the centered stream until the course is clear. */
            p_state->course_scan_attempts++;
            if (!p_meas->b_bounded_course)
            {
                /* Each startup attempt needs usable replies on both sides and at centre. */
                course_forget_survey(p_state);
            }
            p_state->course_left_um = (uint32_t)p_meas->left_um;
            p_state->course_right_um = (uint32_t)p_meas->right_um;
            p_state->b_course_anchor = true;
            p_state->course_clear_count = 0U;
            p_state->course_scan_step = COURSE_SCAN_FIRST;
        }
    }
    else
    {
        int16_t const angle = course_scan_angle(p_state);

        if (age_ms >= COURSE_SCAN_REPLY_MS)
        {
            /* The mailbox has no capture timestamp. A matching word first
             * read after the request expired cannot supply a survey ray. */
            p_state->course_scan_step++;
        }
        else if (p_meas->b_ping_reply && (p_meas->ping_seq == p_state->course_scan_seq) &&
                 (p_meas->ping_servo_deci == angle))
        {
            look_pose_t const pose = {0};

            (void)look_add(&p_state->look, &pose, angle, p_meas->ping_echo, p_meas->ping_mm);
            p_state->course_scan_step++;
        }
    }

    if (COURSE_SCAN_END == p_state->course_scan_step)
    {
        p_state->course_scan_step = COURSE_SCAN_IDLE;
        p_state->course_scan_ms = p_meas->now_ms;
        if (p_meas->b_bounded_course && (0 == p_state->course_side))
        {
            p_state->course_side = course_edge_side(&p_state->look);
        }
        p_cmd->course_scan_samples = p_state->look.count + 1U;
    }
    else if (p_state->course_scan_step >= COURSE_SCAN_FIRST)
    {
        if (step != p_state->course_scan_step)
        {
            p_state->course_scan_ms = p_meas->now_ms;
            p_state->course_scan_seq = (uint8_t)((p_state->course_scan_seq + 1U) & OBSTACLE_PING_SEQ_MK);
        }
        p_cmd->ping_servo_deci = course_scan_angle(p_state);
        p_cmd->ping_seq = p_state->course_scan_seq;
        p_cmd->b_ping_req = 1U;
    }
    else
    {
        /* The wheels are still settling. */
    }
}

#define CONTOUR_MAX_LENGTH_MM (3500U)
#define CONTOUR_MAX_STEP_MM   (1500U)
#define CONTOUR_RATE_DECI_S   (500)
#define CONTOUR_TRACK_MM      (25)
#define CONTOUR_TRACK_DECI    (80)
#define CONTOUR_TICK_MAX_UM   (PILOT_SPEED_MAX_MM_S * (int32_t)PILOT_STALE_MS)
#define CONTOUR_LINE_WAIT_MS  (1000U)
#define CONTOUR_HALF_TURN     (1800)
#define CONTOUR_QUARTER_TURN  (CONTOUR_HALF_TURN / HALF_DIVISOR)
#define CONTOUR_PERCENT_FULL  (100)

static bool contour_offer_valid(pilot_contour_plan_t const * p_offer, pilot_meas_t const * p_meas)
{
    uint32_t length = 0U;
    int32_t heading = 0;

    if ((p_offer == NULL) || (0U == p_offer->id) || (0U == p_offer->count) ||
        (p_offer->count > PILOT_CONTOUR_STEPS) || !p_meas->b_stationary ||
        ((p_meas->now_ms - p_offer->issued_ms) > PILOT_FORWARD_PING_STALE_MS) ||
        ((uint32_t)p_meas->left_um != p_offer->left_um) ||
        ((uint32_t)p_meas->right_um != p_offer->right_um))
    {
        return false;
    }
    for (uint8_t i = 0U; i < p_offer->count; i++)
    {
        pilot_contour_step_t const * const p_step = &p_offer->steps[i];
        int32_t const turn = geom_abs((int32_t)p_step->turn_deci);

        if ((p_step->length_mm > CONTOUR_MAX_STEP_MM) || (turn > CONTOUR_HALF_TURN) ||
            ((0U == p_step->length_mm) && (0 == turn)) ||
            ((0U != p_step->length_mm) && ((turn * HUG_LEG_MM_S) >
                ((int32_t)p_step->length_mm * CONTOUR_RATE_DECI_S))))
        {
            return false;
        }
        length += p_step->length_mm;
        heading += p_step->turn_deci;
    }
    return (0U != length) && (length <= CONTOUR_MAX_LENGTH_MM) && (0 == geom_wrap_deci(heading));
}

/* Integrate signed travel in the verified route's frame. The existing
 * in-place pivot slip correction applies only to opposed wheel travel. */
static bool contour_feedback(pilot_contour_t * p_run, pilot_meas_t const * p_meas)
{
    int32_t const dl = (int32_t)((uint32_t)p_meas->left_um - p_run->last_left_um);
    int32_t const dr = (int32_t)((uint32_t)p_meas->right_um - p_run->last_right_um);
    int32_t ds = 0;
    int32_t dd = 0;
    int32_t mid = 0;

    if ((dl < -CONTOUR_TICK_MAX_UM) || (dl > CONTOUR_TICK_MAX_UM) ||
        (dr < -CONTOUR_TICK_MAX_UM) || (dr > CONTOUR_TICK_MAX_UM))
    {
        return false;
    }
    ds = (dl + dr) / HALF_DIVISOR;
    dd = dr - dl;
    if (((dl < 0) && (dr > 0)) || ((dl > 0) && (dr < 0)))
    {
        dd = (dd * HUG_PIVOT_PCT) / CONTOUR_PERCENT_FULL;
    }
    mid = (int32_t)wheel_yaw_deci((int64_t)p_run->diff_um + (dd / HALF_DIVISOR));
    p_run->x_um += geom_mul_q14(ds, geom_cos_q14(mid));
    p_run->y_um += geom_mul_q14(ds, geom_sin_q14(mid));
    p_run->diff_um += dd;
    p_run->progress_um += ds;
    p_run->last_left_um = (uint32_t)p_meas->left_um;
    p_run->last_right_um = (uint32_t)p_meas->right_um;
    return true;
}

static void contour_point(pilot_contour_t const * p_run, pilot_contour_step_t const * p_step,
                          int32_t travel_mm, int32_t * p_x, int32_t * p_y, int32_t * p_head)
{
    int32_t const heading = p_run->route_head_deci;
    int32_t x = travel_mm;
    int32_t y = 0;
    int32_t turn = p_step->turn_deci;

    if ((0U != p_step->length_mm) && (0 != turn))
    {
        int32_t const radius = ((int32_t)p_step->length_mm * DECI_PER_RAD) / turn;

        turn = (turn * travel_mm) / (int32_t)p_step->length_mm;
        x = geom_mul_q14(radius, geom_sin_q14(turn));
        y = geom_mul_q14(radius, GEOM_Q14_ONE - geom_cos_q14(turn));
    }
    *p_x = p_run->route_x_mm + geom_dot_mm(-heading, x, y);
    *p_y = p_run->route_y_mm + geom_dot_mm(CONTOUR_QUARTER_TURN - heading, x, y);
    *p_head = geom_wrap_deci(heading + turn);
}

static bool contour_position_ok(pilot_contour_t const * p_run, int32_t x, int32_t y)
{
    int32_t const dx = p_run->x_um / UM_PER_MM - x;
    int32_t const dy = p_run->y_um / UM_PER_MM - y;

    return (geom_abs(dx) <= CONTOUR_TRACK_MM) && (geom_abs(dy) <= CONTOUR_TRACK_MM) &&
           ((dx * dx + dy * dy) <= (CONTOUR_TRACK_MM * CONTOUR_TRACK_MM));
}

static int32_t contour_head(pilot_contour_t const * p_run)
{
    return (int32_t)wheel_yaw_deci(p_run->diff_um);
}

static bool contour_endpoint_ok(pilot_contour_t const * p_run)
{
    int32_t const error = geom_wrap_deci((int32_t)p_run->route_head_deci - contour_head(p_run));

    return (geom_abs(error) <= CONTOUR_TRACK_DECI) &&
           contour_position_ok(p_run, p_run->route_x_mm, p_run->route_y_mm);
}

static bool forward_clear(pilot_meas_t const * p_meas, uint16_t min_range)
{
    return (LOOK_ECHO_OPEN == p_meas->ping_echo) ||
           ((LOOK_ECHO_VALID == p_meas->ping_echo) && (p_meas->ping_mm > min_range));
}

static bool contour_moves(pilot_contour_t * p_run, pilot_meas_t const * p_meas,
                          pilot_contour_step_t const * p_step, pilot_cmd_t * p_cmd)
{
    int32_t const length = p_step->length_mm;
    int32_t const travel = clamp_i32(p_run->progress_um / UM_PER_MM, 0, length);
    int32_t const head = contour_head(p_run);
    int32_t const plan_ms = (0 == length) ? (geom_abs(p_step->turn_deci) * 1000) / HUG_PIVOT_MIN_DECI_S
                                          : (length * 1000) / HUG_LEG_MM_S;
    int32_t x = 0;
    int32_t y = 0;
    int32_t wanted = 0;
    int32_t error = 0;
    bool finish = false;

    contour_point(p_run, p_step, travel, &x, &y, &wanted);
    error = geom_wrap_deci(wanted - head);
    if (!contour_position_ok(p_run, x, y) ||
        ((0 != length) && (geom_abs(error) > CONTOUR_TRACK_DECI)) ||
        ((p_meas->now_ms - p_run->step_ms) > (HUG_MOVE_TIMES * (uint32_t)plan_ms + HUG_MOVE_FLOOR_MS)))
    {
        return false;
    }
    if (0 == length)
    {
        finish = geom_abs(error) <= HUG_HEAD_TOL_DECI;
        if (!finish)
        {
            set_cmd(p_cmd, 0, (error > 0) ? HUG_PIVOT_MIN_DECI_S : -HUG_PIVOT_MIN_DECI_S,
                    PILOT_BEHAVIOUR_HUG, PILOT_REASON_NONE);
        }
    }
    else
    {
        int32_t const rate = ((int32_t)p_step->turn_deci * HUG_LEG_MM_S) / length;

        finish = travel >= length;
        if (!finish)
        {
            set_cmd(p_cmd, HUG_LEG_MM_S,
                    clamp_i32(rate + (error * HUG_HOLD_GAIN), -CONTOUR_RATE_DECI_S, CONTOUR_RATE_DECI_S),
                    PILOT_BEHAVIOUR_HUG, PILOT_REASON_NONE);
        }
    }
    if (finish)
    {
        p_run->route_x_mm = x;
        p_run->route_y_mm = y;
        p_run->route_head_deci = (int16_t)wanted;
        p_run->progress_um -= length * UM_PER_MM;
        p_run->index++;
        p_run->status = (p_run->index == p_run->plan.count) ? PILOT_CONTOUR_LINE : PILOT_CONTOUR_WAIT;
        p_run->step_ms = p_meas->now_ms;
    }
    return true;
}

/* The surveyed route ends over a tape sensor. Require a centered return
 * after settling and verify tape again on that same stationary tick. */
static void contour_terminal_line(pilot_state_t * p_state, pilot_contour_t * p_run,
                                  pilot_meas_t const * p_meas, bool b_fresh_forward)
{
    bool const b_hit = p_meas->b_line_left || p_meas->b_line_right;

    if (!contour_endpoint_ok(p_run) ||
        (b_fresh_forward && !forward_clear(p_meas, HUG_GUARD_MM)) ||
        ((p_meas->now_ms - p_run->step_ms) >= CONTOUR_LINE_WAIT_MS))
    {
        p_run->status = PILOT_CONTOUR_STOPPED;
    }
    else if (b_hit && p_meas->b_stationary && p_run->b_terminal_stationary && b_fresh_forward)
    {
        p_run->status = PILOT_CONTOUR_DONE;
        hug_init(&p_state->hug);
        line_follow_init(&p_state->follow);
    }
    else
    {
        p_run->b_terminal_stationary = p_meas->b_stationary;
    }
}

static bool behaviour_contour(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                              pilot_contour_t * p_run, pilot_contour_plan_t const * p_offer,
                              bool b_safe, bool b_fresh_forward, pilot_cmd_t * p_cmd)
{
    if (p_run == NULL)
    {
        return false;
    }
    if ((PILOT_CONTOUR_IDLE == p_run->status) || (PILOT_CONTOUR_DONE == p_run->status))
    {
        if (b_safe || p_meas->b_bounded_course || p_state->b_course_hold || p_meas->b_commanded_turn ||
            !contour_offer_valid(p_offer, p_meas) || (p_run->plan.id == p_offer->id))
        {
            return false;
        }
        *p_run = (pilot_contour_t){.plan = *p_offer, .last_left_um = p_offer->left_um,
            .last_right_um = p_offer->right_um, .step_ms = p_meas->now_ms, .status = PILOT_CONTOUR_WAIT};
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
        return true;
    }
    if (!b_safe)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
    }
    if (b_safe || p_meas->b_bounded_course || p_meas->b_commanded_turn || (p_offer == NULL) ||
        (p_offer->id != p_run->plan.id) || (PILOT_CONTOUR_STOPPED == p_run->status) ||
        !contour_feedback(p_run, p_meas))
    {
        p_run->status = PILOT_CONTOUR_STOPPED;
    }
    else if (PILOT_CONTOUR_LINE == p_run->status)
    {
        contour_terminal_line(p_state, p_run, p_meas, b_fresh_forward);
    }
    else
    {
        pilot_contour_step_t const * const p_step = &p_run->plan.steps[p_run->index];
        uint16_t const min_range = (0U == p_step->length_mm) ? HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM : HUG_GUARD_MM;
        if (b_fresh_forward && !forward_clear(p_meas, min_range))
        {
            p_run->status = PILOT_CONTOUR_STOPPED;
        }
        else if (PILOT_CONTOUR_WAIT == p_run->status)
        {
            if (!contour_endpoint_ok(p_run))
            {
                p_run->status = PILOT_CONTOUR_STOPPED;
            }
            else if (b_fresh_forward)
            {
                p_run->status = PILOT_CONTOUR_MOVE;
                p_run->step_ms = p_meas->now_ms;
            }
            else if ((p_meas->now_ms - p_run->step_ms) >= PILOT_FORWARD_PING_STALE_MS)
            {
                p_run->status = PILOT_CONTOUR_STOPPED;
            }
        }
        if ((PILOT_CONTOUR_MOVE == p_run->status) && !contour_moves(p_run, p_meas, p_step, p_cmd))
        {
            p_run->status = PILOT_CONTOUR_STOPPED;
        }
    }
    if (PILOT_CONTOUR_STOPPED == p_run->status)
    {
        /* The old stall back-off is outside the authorized geometry. */
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, (pilot_reason_t)p_cmd->reason);
        p_state->phase = (uint8_t)PILOT_PHASE_TRY;
    }
    return true;
}

static void rejoin_search_tick(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                               bool b_safe, bool b_fresh_forward, pilot_cmd_t * p_cmd)
{
    if (b_safe || p_meas->b_bounded_course || p_meas->b_commanded_turn ||
        (b_fresh_forward && !forward_clear(p_meas, HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM)))
    {
        hug_forget_approach(&p_state->hug);
        line_follow_stop_local_search(&p_state->follow);
    }
    if (b_safe)
    {
        return;
    }
    behaviour_follow_line(p_state, p_meas, p_cmd);
    if (line_follow_local_search(&p_state->follow))
    {
        if (line_follow_local_missed(&p_state->follow))
        {
            /* Rebind the retained edge frame only after the axle is back
             * at rest on the entry heading with a new forward observation. */
            if (!p_meas->b_stationary || !b_fresh_forward)
            {
                set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_REJOIN_SEARCH, PILOT_REASON_REJOIN_SEARCH);
                return;
            }
            line_follow_stop_local_search(&p_state->follow);
            if (hug_resume_remaining(&p_state->hug, p_meas->left_um, p_meas->right_um, p_meas->now_ms))
            {
                p_state->hug.width_mm = p_state->exit_width_mm;
                p_state->hug.breadth_mm = p_state->exit_breadth_mm;
                line_follow_init(&p_state->follow);
                set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_HUG, PILOT_REASON_NONE);
                return;
            }
        }
        p_cmd->behaviour = (uint8_t)PILOT_BEHAVIOUR_REJOIN_SEARCH;
        p_cmd->reason = (uint8_t)((LINE_FOLLOW_STOPPED == p_state->follow.status)
            ? PILOT_REASON_REJOIN_STOP : PILOT_REASON_REJOIN_SEARCH);
    }
    else
    {
        p_cmd->b_rejoined = true;
    }
}

void pilot_step_with_contour(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                             pilot_contour_t * p_contour,
                             pilot_contour_plan_t const * p_offer, pilot_cmd_t * p_cmd)
{
    bool const b_was_back = backing_off(p_state->phase);
    bool const b_fresh_forward = fresh_forward_ping(p_state, p_meas);
    bool b_safe = false;
    bool b_backoff_done = false;
    bool b_start_contact = false;
    bool b_moving = false;

    set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_NONE, PILOT_REASON_NONE);
    b_safe = behaviour_safety(p_state, p_meas, b_fresh_forward, p_cmd);
    b_backoff_done = b_was_back && ((uint8_t)PILOT_PHASE_RETRY == p_state->phase);
    if (line_follow_local_search(&p_state->follow))
    {
        rejoin_search_tick(p_state, p_meas, b_safe, b_fresh_forward, p_cmd);
    }
    else if (behaviour_contour(p_state, p_meas, p_contour, p_offer, b_safe, b_fresh_forward, p_cmd))
    {
        /* The contour supplies the sole motion target. */
    }
    else if (course_hug_held(p_state, p_meas, b_fresh_forward, !b_safe, &b_start_contact))
    {
        if (!b_safe)
        {
            set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_RECOVERY);
            course_scan_tick(p_state, p_meas, p_cmd);
        }
    }
    else
    {
        bool const b_obstacle = p_meas->b_bounded_course
            ? behaviour_course_contact(p_state, p_meas, b_safe, b_fresh_forward, b_backoff_done, p_cmd)
            : behaviour_hug(p_state, p_meas, b_safe, b_backoff_done, b_start_contact, p_cmd);

        if (!b_obstacle && !b_safe && !p_meas->b_commanded_turn)
        {
            behaviour_follow_line(p_state, p_meas, p_cmd);
        }
    }
    b_moving = (0 != p_cmd->speed_mm_s) || (0 != p_cmd->turn_deci_s);
    if ((PILOT_REASON_NONE == p_cmd->reason) && ((uint8_t)PILOT_PHASE_RETRY == p_state->phase) &&
        b_moving)
    {
        p_cmd->reason = (uint8_t)PILOT_REASON_STUCK;
    }
    flag_set(p_state, FLAG_MOVING, b_moving);
    if (((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE != p_cmd->behaviour) || (PILOT_REASON_NONE != p_cmd->reason))
    {
        p_state->line_radius_mm = 0;
    }
    flag_set(p_state, FLAG_PING_REQUEST, 0U != p_cmd->b_ping_req);
    flag_set(p_state, FLAG_REVERSE,
             ((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour) &&
                 (p_cmd->speed_mm_s < 0));
    p_state->b_follow_forward = ((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour) &&
                                (p_cmd->speed_mm_s > 0);
    p_state->b_slip_forward = p_state->b_follow_forward && (0 == p_cmd->turn_deci_s);
    if (!p_state->b_slip_forward)
    {
        p_state->slip_range_mm = 0U;
    }
    note_wheels(p_state, p_cmd);
}

void pilot_step(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                pilot_cmd_t * p_cmd)
{
    pilot_step_with_contour(p_state, p_meas, NULL, NULL, p_cmd);
}

void pilot_estop(pilot_state_t * p_state)
{
    hug_forget_approach(&p_state->hug);
    p_state->b_estop = 1U;
}

void pilot_clear_latches(pilot_state_t * p_state)
{
    hug_forget_approach(&p_state->hug);
    p_state->line_radius_mm = 0;
    p_state->b_encoder = 0U;
    p_state->b_check_left = false;
    p_state->b_check_right = false;
    p_state->b_timeout = 0U;
    /* No motion is recorded: the next tick opens a new stuck window. This
     * clears the sonar and stale latch too. */
    p_state->flags = 0U;
    p_state->phase = (uint8_t)PILOT_PHASE_TRY;
    p_state->slip_range_mm = 0U;
    p_state->b_slip_forward = false;
    p_state->course_clear_count = 0U;
    p_state->data_recovery_pings = 0U;
    /* The side of the line stays: a one-shot turn has just set it. */
    if (!line_follow_local_search(&p_state->follow))
    {
        line_follow_set_side(&p_state->follow, (line_side_t)p_state->follow.side);
    }
}

void pilot_release_sonar(pilot_state_t * p_state)
{
    flag_set(p_state, FLAGS_SONAR, false);
    p_state->data_recovery_pings = 0U;
}

void pilot_start_run(pilot_state_t * p_state)
{
    if (line_follow_local_search(&p_state->follow))
    {
        line_follow_init(&p_state->follow);
    }
    pilot_clear_latches(p_state);
    if (p_state->b_course_stuck)
    {
        p_state->b_course_stuck = false;
        p_state->b_course_hold = true;
        course_forget_survey(p_state);
        p_state->course_scan_step = COURSE_SCAN_IDLE;
        p_state->course_scan_attempts = 0U;
        p_state->course_clear_count = 0U;
    }
}

static void discard_course_motion(pilot_state_t * p_state)
{
    hug_init(&p_state->hug);
    /* The discarded contact cannot authorize another reverse. Start the
     * first motion window at the next measured pose on this course. */
    flag_set(p_state, FLAG_MOVING, false);
    p_state->phase = (uint8_t)PILOT_PHASE_TRY;
    p_state->b_check_left = false;
    p_state->b_check_right = false;
    p_state->slip_range_mm = 0U;
    p_state->b_slip_forward = false;
    p_state->b_follow_forward = false;
    p_state->line_radius_mm = 0;
    course_forget_survey(p_state);
    p_state->course_scan_step = COURSE_SCAN_IDLE;
    p_state->course_scan_attempts = 0U;
    p_state->course_clear_count = 0U;
    p_state->exit_width_mm = 0U;
    p_state->exit_breadth_mm = 0U;
}

void pilot_change_floor(pilot_state_t * p_state, bool b_continuous_floor)
{
    discard_course_motion(p_state);
    line_follow_init(&p_state->follow);
    p_state->b_course_hold = !b_continuous_floor;
    p_state->b_course_stuck = false;
}

void pilot_recover_pose(pilot_state_t * p_state, bool clear, uint32_t now_ms)
{
    bool const scan = !clear || hug_active(&p_state->hug) || p_state->b_course_hold;
    discard_course_motion(p_state);
    if ((LINE_FOLLOW_STOPPED != p_state->follow.status) &&
        (scan || !line_follow_local_search(&p_state->follow)))
    {
        line_follow_init(&p_state->follow);
    }
    if (scan)
    {
        course_stop_and_scan(p_state, now_ms);
    }
}

bool pilot_count_moved(uint8_t * p_seen, uint8_t count)
{
    bool const b_moved = (count != *p_seen);

    *p_seen = count;
    return b_moved;
}

void pilot_drive(drive_state_t * p_drive, pilot_cmd_t const * p_cmd, uint32_t dt_ms,
                 int32_t * p_left_um_s, int32_t * p_right_um_s)
{
    /* A stationary measurement and a safety hold both require zero wheel
     * targets immediately. The encoders determine when the chassis settles. */
    if ((0 == p_cmd->speed_mm_s) && (0 == p_cmd->turn_deci_s))
    {
        drive_safety_stop(p_drive, p_left_um_s, p_right_um_s);
    }
    else if (((uint8_t)PILOT_REASON_STUCK == p_cmd->reason) &&
             ((p_drive->left_um_s > 0) || (p_drive->right_um_s > 0)))
    {
        /* The wheels do not turn: back off from rest, not through a 0.6 s
         * slew down that pushes on the blockage. */
        drive_init(p_drive);
        drive_step(p_drive, p_cmd->speed_mm_s, p_cmd->turn_deci_s, dt_ms, p_left_um_s,
                   p_right_um_s);
    }
    else if (((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour) ||
             ((uint8_t)PILOT_BEHAVIOUR_REJOIN_SEARCH == p_cmd->behaviour))
    {
        drive_follow(p_drive, p_cmd->speed_mm_s, p_cmd->turn_deci_s, dt_ms, p_left_um_s,
                     p_right_um_s);
    }
    else
    {
        drive_step(p_drive, p_cmd->speed_mm_s, p_cmd->turn_deci_s, dt_ms, p_left_um_s,
                   p_right_um_s);
    }
    /* Speed and yaw have coarser units than the original wheel targets.
     * Preserve a true zero so the motor controller releases that wheel. */
    if (p_cmd->b_stop_left)
    {
        p_drive->left_um_s = 0;
        *p_left_um_s = 0;
    }
    if (p_cmd->b_stop_right)
    {
        p_drive->right_um_s = 0;
        *p_right_um_s = 0;
    }
}

/*** end of file ***/
