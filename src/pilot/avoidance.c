/** @file avoidance.c
 * @brief Selects the obstacle maneuver and its recovery from wheel and sonar evidence.
 */
#include "avoidance_internal.h"

#include "pilot_internal.h"
#include "geometry.h"
#include "navigation/obstacle_ping.h"
#include "platform/units.h"

#define HALF_DIVISOR (2)
#define COURSE_SCAN_IDLE      (0U)
#define COURSE_SCAN_SETTLE    (1U)
#define COURSE_SCAN_FIRST     (2U)
#define COURSE_SCAN_RETRY_MS  (320U)
#define COURSE_SCAN_REPLY_MS  (300U)
#define COURSE_CLEAR_PINGS    (3U)
#define COURSE_STOP_MM        (100U)
#define COURSE_INNER_DECI     (OBSTACLE_SERVO_LIMIT_DECI / HALF_DIVISOR)
static const int16_t g_course_scan_angles[][3] = {
    {-OBSTACLE_SERVO_LIMIT_DECI, 0, OBSTACLE_SERVO_LIMIT_DECI},
    {COURSE_INNER_DECI, 0, -COURSE_INNER_DECI},
};
#define COURSE_SCAN_LIMIT (sizeof(g_course_scan_angles) / sizeof(g_course_scan_angles[0]))
#define COURSE_SCAN_END (COURSE_SCAN_FIRST + sizeof(g_course_scan_angles[0]) / sizeof(g_course_scan_angles[0][0]))

static void course_forget_survey(avoidance_t * p_state)
{
    look_clear(&p_state->look);
    p_state->course_side = 0;
    p_state->b_course_anchor = false;
}

void avoidance_request_survey(avoidance_t * p_state, uint32_t now_ms)
{
    p_state->recovery_phase = (uint8_t)PILOT_PHASE_TRY;
    p_state->b_course_hold = true;
    course_forget_survey(p_state);
    p_state->course_scan_ms = now_ms;
    p_state->course_scan_step = COURSE_SCAN_SETTLE;
    p_state->course_clear_count = 0U;
    p_state->course_scan_attempts = 0U;
}

static bool behaviour_course_contact(avoidance_t * p_state, pilot_meas_t const * p_meas,
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

    avoidance_request_survey(p_state, p_meas->now_ms);
    /* The hold can outlast a lift or calibration. Preserve the stronger
     * stop on this tick and remember the range for the release tick. */
    if (!b_safe)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_RECOVERY);
    }
    if (b_contact && (LOOK_ECHO_VALID == p_meas->ping_echo))
    {
        p_cmd->contact_mm = p_meas->ping_mm;
    }
    return true;
}

static bool course_hug_held(avoidance_t * p_state, pilot_meas_t const * p_meas,
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
        *p_start_contact = true;
        p_state->b_course_hold = false;
        course_forget_survey(p_state);
        return false;
    }
    return p_state->b_course_hold;
}

/* A second pass samples the observed side at new bearings between the
 * forward ray and the servo limit. No return here measures floor support. */
static int16_t course_scan_angle(avoidance_t const * p_state)
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
static void course_scan_tick(avoidance_t * p_state, pilot_meas_t const * p_meas,
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
        else if (obstacle_ping_reply_matches(p_meas->b_ping_reply, p_meas->ping_seq, p_meas->ping_servo_deci,
                                             p_state->course_scan_seq, angle, 0U))
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

/* The hug watches the forward pings and tracks the pose on every tick; it
 * takes the wheels and the sonar when it has something to go round. */
static avoidance_result_t step_hug(avoidance_t * p_state, pilot_meas_t const * p_meas,
                                   uint16_t bend_radius_mm, int16_t drift_corr_um, bool b_other,
                                   bool b_stall, bool b_relook, pilot_cmd_t * p_cmd)
{
    hug_in_t in;
    hug_out_t out;
    bool const b_was = hug_active(&p_state->hug);
    uint16_t const width = p_state->hug.width_mm;
    uint16_t const breadth = p_state->hug.breadth_mm;

    in.left_um = p_meas->left_um;
    in.right_um = p_meas->right_um;
    in.now_ms = p_meas->now_ms;
    in.ping_mm = p_meas->ping_mm;
    in.bend_radius_mm = bend_radius_mm;
    in.ping_servo_deci = p_meas->ping_servo_deci;
    in.ping_echo = p_meas->ping_echo;
    in.ping_seq = p_meas->ping_seq;
    in.b_ping_reply = p_meas->b_ping_reply;
    in.b_line_left = p_meas->b_line_left;
    in.b_line_right = p_meas->b_line_right;
    in.b_stall = b_stall;
    in.b_other = b_other;
    in.b_contact_backoff = b_other && (p_cmd->speed_mm_s < 0) &&
        ((uint8_t)PILOT_REASON_STUCK == p_cmd->reason);
    in.b_independent_hold = b_other && (p_meas->b_held || p_meas->b_wheel_invalid ||
        ((uint8_t)PILOT_REASON_ESTOP == p_cmd->reason) ||
        ((p_meas->now_ms - p_meas->stamp_ms) > PILOT_STALE_MS));
    in.b_stationary = p_meas->b_stationary;
    in.b_relook = b_relook;
    in.pivot_scale_pct = (p_meas->pivot_scale_pct <= (uint8_t)PERCENT)
        ? (p_meas->pivot_scale_pct & HUG_PIVOT_WITNESS_MASK) : 0U;
    in.drift_corr_um = drift_corr_um;
    bool const b_takes = hug_step(&p_state->hug, &p_state->look, &in, &out);
    if (out.b_no_tape || out.b_rejoined)
    {
        p_state->exit_width_mm = width;
        p_state->exit_breadth_mm = breadth;
    }
    if (0U != out.b_timeout)
    {
        return AVOIDANCE_TIMEOUT;
    }
    if (out.b_rejoined)
    {
        return AVOIDANCE_REJOINED;
    }
    if (out.b_no_exit || out.b_no_tape)
    {
        return out.b_no_exit ? AVOIDANCE_NO_EXIT : AVOIDANCE_FIND_TAPE;
    }
    if (b_takes)
    {
        pilot_set_command(p_cmd, out.speed_mm_s, out.turn_deci_s, PILOT_BEHAVIOUR_HUG,
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
        return AVOIDANCE_COMMAND;
    }
    return (b_was && !hug_active(&p_state->hug)) ? AVOIDANCE_RELEASE : AVOIDANCE_IDLE;
}

avoidance_result_t avoidance_step(avoidance_t * p_state, pilot_meas_t const * p_meas,
                                  uint16_t bend_radius_mm, int16_t drift_corr_um, bool b_inhibited,
                                  bool b_fresh_forward, bool b_backoff_done, pilot_cmd_t * p_cmd)
{
    bool b_start_contact = false;

    if (course_hug_held(p_state, p_meas, b_fresh_forward, !b_inhibited, &b_start_contact))
    {
        if (!b_inhibited)
        {
            pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_RECOVERY);
            course_scan_tick(p_state, p_meas, p_cmd);
        }
        return AVOIDANCE_COMMAND;
    }
    if (p_meas->b_bounded_course)
    {
        return behaviour_course_contact(p_state, p_meas, b_inhibited, b_fresh_forward,
                                        b_backoff_done, p_cmd) ? AVOIDANCE_COMMAND : AVOIDANCE_IDLE;
    }
    return step_hug(p_state, p_meas, bend_radius_mm, drift_corr_um, b_inhibited, b_backoff_done, b_start_contact, p_cmd);
}

void avoidance_init(avoidance_t * p_state)
{
    *p_state = (avoidance_t){0};
    hug_init(&p_state->hug);
}

void avoidance_discard_route(avoidance_t * p_state)
{
    hug_init(&p_state->hug);
    p_state->recovery_phase = (uint8_t)PILOT_PHASE_TRY;
    course_forget_survey(p_state);
    p_state->course_scan_step = COURSE_SCAN_IDLE;
    p_state->course_scan_attempts = 0U;
    p_state->course_clear_count = 0U;
    p_state->exit_width_mm = 0U;
    p_state->exit_breadth_mm = 0U;
}

void avoidance_restart_stalled(avoidance_t * p_state)
{
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

bool avoidance_resume_edges(avoidance_t * p_state, pilot_meas_t const * p_meas)
{
    if (!hug_resume_remaining(&p_state->hug, p_meas->left_um, p_meas->right_um, p_meas->now_ms))
    {
        return false;
    }
    p_state->hug.width_mm = p_state->exit_width_mm;
    p_state->hug.breadth_mm = p_state->exit_breadth_mm;
    return true;
}

bool avoidance_backing_off(avoidance_t const * p_state)
{
    return ((uint8_t)PILOT_PHASE_BACKOFF == p_state->recovery_phase) ||
           ((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL == p_state->recovery_phase);
}

static bool backoff_distance_reached(uint32_t travel_um)
{
    uint32_t const backward_um = 0U - travel_um;
    return (backward_um >= (uint32_t)PILOT_BACKOFF_UM) && (backward_um <= INT32_MAX);
}

bool avoidance_recover(avoidance_t * p_state, avoidance_progress_t const * p_progress,
                       pilot_meas_t const * p_meas, pilot_cmd_t * p_cmd)
{
    bool b_new_window = false;

    if (!p_progress->b_moving)
    {
        if ((uint8_t)PILOT_PHASE_RETRY != p_state->recovery_phase)
        {
            p_state->recovery_phase = (uint8_t)PILOT_PHASE_TRY;
        }
        b_new_window = true;
    }
    else if (avoidance_backing_off(p_state))
    {
        if (backoff_distance_reached(p_progress->left_um) || backoff_distance_reached(p_progress->right_um) ||
            (p_progress->age_ms >= PILOT_BACKOFF_MAX_MS))
        {
            p_state->recovery_phase = (uint8_t)PILOT_PHASE_RETRY;
            b_new_window = true;
        }
    }
    else if ((!p_progress->b_slip && p_progress->b_advanced) ||
             ((p_progress->age_ms >= PILOT_STUCK_MS) &&
              (p_progress->b_line_reverse || (BYPASS_IDLE != p_state->hug.bypass.phase))))
    {
        /* The measured bypass already rechecks a stalled leg from rest. */
        p_state->recovery_phase = (uint8_t)PILOT_PHASE_TRY;
        b_new_window = true;
    }
    else if (p_progress->b_slip || (p_progress->age_ms >= PILOT_STUCK_MS))
    {
        if (p_meas->b_bounded_course)
        {
            p_state->b_course_stuck = true;
        }
        else if (hug_active(&p_state->hug))
        {
            p_state->recovery_phase = (uint8_t)((p_progress->b_one_moved && p_progress->b_both_commanded)
                             ? PILOT_PHASE_BACKOFF_ONE_WHEEL : PILOT_PHASE_BACKOFF);
            b_new_window = true;
        }
        else
        {
            avoidance_request_survey(p_state, p_meas->now_ms);
        }
    }
    if (p_state->b_course_stuck || (p_meas->b_bounded_course && avoidance_backing_off(p_state)))
    {
        p_state->b_course_stuck = true;
        p_state->recovery_phase = (uint8_t)PILOT_PHASE_TRY;
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
    }
    else if (avoidance_backing_off(p_state))
    {
        pilot_set_command(p_cmd, -PILOT_BACKOFF_MM_S, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_STUCK);
    }
    return b_new_window;
}
