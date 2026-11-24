/** @file pilot.c
 * @brief Pilot core: safety stop and line follow, in priority order.
 *
 * Each tick produces one wheel command. Safety takes priority over obstacle
 * navigation, tape reacquisition and ordinary line following.
 */

#include "pilot/pilot.h"

#include "avoidance_internal.h"
#include "pilot_internal.h"

#include <stddef.h>

#include "navigation/obstacle_ping.h"
#include "platform/chassis.h"
#include "platform/units.h"

#define HALF_DIVISOR         (2)
#define REAPPROACH_CAP_UM_S  (70 * UM_PER_MM)

#define DATA_RECOVERY_PINGS   (3U)

STATIC_ASSERT(OBSTACLE_CONSIST_TRAVEL_MM > (OBSTACLE_CONSIST_TOL_MM + OBSTACLE_AGREE_MM),
              "a stable range must disagree with travel by more than the sonar tolerance");

void pilot_set_command(pilot_cmd_t * p_cmd, int32_t speed_mm_s,
                       int32_t turn_deci_s, pilot_behaviour_t behaviour,
                       pilot_reason_t reason)
{
    *p_cmd = (pilot_cmd_t){
        .speed_mm_s = (int16_t)clamp_i32(speed_mm_s, -PILOT_SPEED_MAX_MM_S, PILOT_SPEED_MAX_MM_S),
        .turn_deci_s = (int16_t)clamp_i32(turn_deci_s, -PILOT_TURN_MAX_DECI_S, PILOT_TURN_MAX_DECI_S),
        .behaviour = (uint8_t)behaviour,
        .reason = (uint8_t)reason,
    };
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

/* The pilot diagnoses encoder faults even while avoidance owns a maneuver. */
static bool check_wheel_progress(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                                 bool b_fresh_forward, bool * p_backoff_done, pilot_cmd_t * p_cmd)
{
    avoidance_progress_t progress = {
        .left_um = travel_um(p_meas->left_um, p_state->ref_left_um),
        .right_um = travel_um(p_meas->right_um, p_state->ref_right_um),
        .age_ms = p_meas->now_ms - p_state->since_ms,
        .b_moving = flag_get(p_state, FLAG_MOVING),
        .b_line_reverse = flag_get(p_state, FLAG_REVERSE),
    };
    bool const b_left = wheel_moved(progress.left_um);
    bool const b_right = wheel_moved(progress.right_um);
    bool const b_need_l = !flag_get(p_state, FLAG_LOW_LEFT);
    bool const b_need_r = !flag_get(p_state, FLAG_LOW_RIGHT);
    bool const b_was_backing_off = avoidance_backing_off(&p_state->avoidance);
    bool const b_one_wheel_backoff = (PILOT_PHASE_BACKOFF_ONE_WHEEL == p_state->avoidance.recovery_phase);

    progress.b_one_moved = b_left != b_right;
    progress.b_both_commanded = b_need_l && b_need_r;
    if (progress.b_moving && !b_was_backing_off)
    {
        bool const b_progress_l = b_left && (!p_state->b_follow_forward || (progress.left_um <= WINDOW_MAX_UM));
        bool const b_progress_r = b_right && (!p_state->b_follow_forward || (progress.right_um <= WINDOW_MAX_UM));
        progress.b_advanced = (b_progress_l || !b_need_l) && (b_progress_r || !b_need_r);
        progress.b_slip = slip_detected(p_state, p_meas, b_fresh_forward);
        p_state->b_check_left = p_state->b_check_left && !b_left;
        p_state->b_check_right = p_state->b_check_right && !b_right;
        if (progress.age_ms >= PILOT_STUCK_MS)
        {
            if (progress.b_both_commanded && progress.b_one_moved &&
                (p_state->b_check_left || p_state->b_check_right))
            {
                p_state->b_encoder = 1U;
            }
            p_state->b_check_left = b_need_l && !b_left;
            p_state->b_check_right = b_need_r && !b_right;
        }
    }
    if (avoidance_recover(&p_state->avoidance, &progress, p_meas, p_cmd))
    {
        p_state->ref_left_um = (uint32_t)p_meas->left_um;
        p_state->ref_right_um = (uint32_t)p_meas->right_um;
        p_state->since_ms = p_meas->now_ms;
    }
    *p_backoff_done = b_was_backing_off && (PILOT_PHASE_RETRY == p_state->avoidance.recovery_phase);
    if (*p_backoff_done && b_one_wheel_backoff && progress.b_one_moved)
    {
        p_state->b_encoder = 1U;
    }
    if (p_state->b_encoder && !p_state->avoidance.b_course_stuck)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ENCODER);
    }
    return PILOT_BEHAVIOUR_NONE != p_cmd->behaviour;
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
            p_state->avoidance.course_clear_count = 0U;
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
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ESTOP);
        b_takes = true;
    }
    else if (0U != p_state->b_encoder)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ENCODER);
        b_takes = true;
    }
    else if (0U != p_state->b_timeout)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HUG_TIMEOUT);
        b_takes = true;
    }
    else if ((PILOT_REASON_SONAR == data) ||
             (PILOT_REASON_SONAR_LATCHED == data))
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, data);
        b_takes = true;
    }
    /* Motion refuses commands while it holds the car: no travel is no proof
     * of a stuck car, so the stuck window restarts, as for stale data. */
    else if (p_meas->b_held)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HELD);
        p_state->avoidance.recovery_phase = (uint8_t)PILOT_PHASE_TRY;
        b_takes = true;
    }
    else if ((PILOT_REASON_STALE == data) ||
             (PILOT_REASON_STALE_LATCHED == data))
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, data);
        p_state->avoidance.recovery_phase = (uint8_t)PILOT_PHASE_TRY;
        b_takes = true;
    }
    else if (p_state->avoidance.b_course_stuck)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
        b_takes = true;
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
        status = line_follow_update_floor(
            &p_state->follow, p_meas->b_line_left, p_meas->b_line_right,
            heading, travel_um, p_meas->now_ms,
            !p_meas->b_bounded_course, &targets);
    }

    bool const b_ranged = (!p_meas->b_ping_reply) && (p_meas->ping_mm > 0U) &&
                          look_echo_is_point(p_meas->ping_echo);

    line_targets_cap(&targets, line_bumper_cap_um_s(b_ranged, p_meas->ping_mm));
    if (((targets.left_um_s + targets.right_um_s) > 0) && hug_reapproaching(&p_state->avoidance.hug))
    {
        line_targets_cap(&targets, REAPPROACH_CAP_UM_S);
    }
    p_state->line_radius_mm = ((LINE_FOLLOW_FOLLOWING == status) &&
        (p_meas->b_line_left != p_meas->b_line_right)) ?
        drive_forward_radius_mm(targets.left_um_s, targets.right_um_s) : 0;
    if (LINE_FOLLOW_STOPPED == status)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_FOLLOW_LINE,
                PILOT_REASON_LINE_LOST_STOP);
    }
    else
    {
        int32_t const speed_um_s = (targets.left_um_s + targets.right_um_s) /
                                   HALF_DIVISOR;
        int32_t const diff_um_s = targets.right_um_s - targets.left_um_s;

        pilot_set_command(p_cmd, speed_um_s / UM_PER_MM,
                (int32_t)wheel_yaw_deci(diff_um_s),
                PILOT_BEHAVIOUR_FOLLOW_LINE, PILOT_REASON_NONE);
        p_cmd->b_stop_left = targets.left_um_s == 0;
        p_cmd->b_stop_right = targets.right_um_s == 0;
    }
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
    p_state->b_check_left = false;
    p_state->b_check_right = false;
    p_state->data_recovery_pings = 0U;
    avoidance_init(&p_state->avoidance);
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

bool pilot_forward_clear(pilot_meas_t const * p_meas, uint16_t min_range)
{
    return (LOOK_ECHO_OPEN == p_meas->ping_echo) ||
           ((LOOK_ECHO_VALID == p_meas->ping_echo) && (p_meas->ping_mm > min_range));
}

static void rejoin_search_tick(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                               bool b_inhibited, bool b_fresh_forward, pilot_cmd_t * p_cmd)
{
    if (b_inhibited || p_meas->b_bounded_course || p_meas->b_commanded_turn ||
        (!line_follow_local_missed(&p_state->follow) && b_fresh_forward &&
         !pilot_forward_clear(p_meas, HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM)))
    {
        hug_forget_approach(&p_state->avoidance.hug);
        line_follow_stop_local_search(&p_state->follow);
    }
    if (b_inhibited)
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
                pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_REJOIN_SEARCH, PILOT_REASON_REJOIN_SEARCH);
                return;
            }
            if (pilot_forward_clear(p_meas, HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM) &&
                avoidance_resume_edges(&p_state->avoidance, p_meas))
            {
                line_follow_init(&p_state->follow);
                pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_HUG, PILOT_REASON_NONE);
                return;
            }
            line_follow_init(&p_state->follow);
            if (pilot_forward_clear(p_meas, HUG_STANDOFF_MM))
            {
                /* The local sweep returned to its entry heading. A clear
                 * forward observation permits travel without a tape fix. */
                behaviour_follow_line(p_state, p_meas, p_cmd);
            }
            else
            {
                avoidance_request_survey(&p_state->avoidance, p_meas->now_ms);
                pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_RECOVERY);
            }
            return;
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
    bool const b_fresh_forward = fresh_forward_ping(p_state, p_meas);
    bool b_inhibited = false;
    bool b_backoff_done = false;
    bool b_moving = false;

    pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_NONE, PILOT_REASON_NONE);
    b_inhibited = behaviour_safety(p_state, p_meas, b_fresh_forward, p_cmd);
    if (!b_inhibited && !p_meas->b_commanded_turn && !line_follow_local_search(&p_state->follow))
    {
        b_inhibited = check_wheel_progress(p_state, p_meas, b_fresh_forward, &b_backoff_done, p_cmd);
    }
    if (line_follow_local_search(&p_state->follow))
    {
        rejoin_search_tick(p_state, p_meas, b_inhibited, b_fresh_forward, p_cmd);
    }
    else if (pilot_contour_step(p_meas, p_contour, p_offer, b_inhibited, p_state->avoidance.b_course_hold,
                                b_fresh_forward, p_cmd))
    {
        if (PILOT_CONTOUR_DONE == p_contour->status)
        {
            hug_init(&p_state->avoidance.hug);
            line_follow_init(&p_state->follow);
        }
        else if (PILOT_CONTOUR_STOPPED == p_contour->status)
        {
            p_state->avoidance.recovery_phase = (uint8_t)PILOT_PHASE_TRY;
        }
    }
    else
    {
        avoidance_result_t const result = avoidance_step(&p_state->avoidance, p_meas,
            traced_radius_mm(p_state, p_meas), b_inhibited, b_fresh_forward, b_backoff_done, p_cmd);
        switch (result)
        {
            case AVOIDANCE_TIMEOUT:
            default:
                p_state->b_timeout = 1U;
                pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HUG_TIMEOUT);
                break;
            case AVOIDANCE_REJOINED:
                line_follow_init(&p_state->follow);
                behaviour_follow_line(p_state, p_meas, p_cmd);
                p_cmd->b_rejoined = true;
                break;
            case AVOIDANCE_FIND_TAPE:
            case AVOIDANCE_NO_EXIT:
                line_follow_start_local_search(&p_state->follow);
                if (AVOIDANCE_NO_EXIT == result)
                {
                    line_follow_stop_local_search(&p_state->follow);
                }
                pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_REJOIN_SEARCH,
                        (AVOIDANCE_NO_EXIT == result) ? PILOT_REASON_REJOIN_STOP : PILOT_REASON_REJOIN_SEARCH);
                break;
            case AVOIDANCE_RELEASE:
            case AVOIDANCE_IDLE:
                if (AVOIDANCE_RELEASE == result)
                {
                    line_follow_init(&p_state->follow);
                }
                if (!b_inhibited && !p_meas->b_commanded_turn)
                {
                    behaviour_follow_line(p_state, p_meas, p_cmd);
                }
                break;
            case AVOIDANCE_COMMAND:
                break;
        }
    }
    b_moving = (0 != p_cmd->speed_mm_s) || (0 != p_cmd->turn_deci_s);
    if ((PILOT_REASON_NONE == p_cmd->reason) && ((uint8_t)PILOT_PHASE_RETRY == p_state->avoidance.recovery_phase) &&
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
    hug_forget_approach(&p_state->avoidance.hug);
    p_state->b_estop = 1U;
}

void pilot_clear_latches(pilot_state_t * p_state)
{
    hug_forget_approach(&p_state->avoidance.hug);
    p_state->line_radius_mm = 0;
    p_state->b_encoder = 0U;
    p_state->b_check_left = false;
    p_state->b_check_right = false;
    p_state->b_timeout = 0U;
    /* No motion is recorded: the next tick opens a new stuck window. This
     * clears the sonar and stale latch too. */
    p_state->flags = 0U;
    p_state->avoidance.recovery_phase = (uint8_t)PILOT_PHASE_TRY;
    p_state->slip_range_mm = 0U;
    p_state->b_slip_forward = false;
    p_state->avoidance.course_clear_count = 0U;
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
    avoidance_restart_stalled(&p_state->avoidance);
}

static void discard_course_motion(pilot_state_t * p_state)
{
    avoidance_discard_route(&p_state->avoidance);
    /* The discarded contact cannot authorize another reverse. Start the
     * first motion window at the next measured pose on this course. */
    flag_set(p_state, FLAG_MOVING, false);
    p_state->b_check_left = false;
    p_state->b_check_right = false;
    p_state->slip_range_mm = 0U;
    p_state->b_slip_forward = false;
    p_state->b_follow_forward = false;
    p_state->line_radius_mm = 0;
}

void pilot_change_floor(pilot_state_t * p_state, bool b_continuous_floor)
{
    discard_course_motion(p_state);
    line_follow_init(&p_state->follow);
    p_state->avoidance.b_course_hold = !b_continuous_floor;
    p_state->avoidance.b_course_stuck = false;
}

void pilot_recover_pose(pilot_state_t * p_state, bool clear, uint32_t now_ms)
{
    bool const scan = !clear || hug_active(&p_state->avoidance.hug) || p_state->avoidance.b_course_hold;
    discard_course_motion(p_state);
    if ((LINE_FOLLOW_STOPPED != p_state->follow.status) &&
        (scan || !line_follow_local_search(&p_state->follow)))
    {
        line_follow_init(&p_state->follow);
    }
    if (scan)
    {
        avoidance_request_survey(&p_state->avoidance, now_ms);
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
