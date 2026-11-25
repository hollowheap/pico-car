/** @file motion.c
 * @brief Hug odometry, bend clearance, and pivot and drive execution.
 */

#include "internal.h"

#include <stddef.h>

#include "obstacle/obstacle.h"
#include "motion/pilot/pilot.h"
#include "platform/units.h"
#include "ir/ir.h"

#define EXIT_CLEAR      (0U)
#define HIST_MASK       (0x07U)
#define HOP_LIMIT_MM    (400)
#define HUG_LANE_BUFFER_MM (30)
#define HUG_FLANK_GAP_MM (40)
#define MS_PER_S        (1000U)

/* The longer end of the chassis determines the swept diagonal. */
#define HUG_LONG_END_MM ((CHASSIS_FRONT_TIP_MM > CHASSIS_AXLE_TO_CASTER_MM) ? \
                          CHASSIS_FRONT_TIP_MM : CHASSIS_AXLE_TO_CASTER_MM)
#define HUG_SWEEP_SQ_MM (HUG_LONG_END_MM * HUG_LONG_END_MM + CHASSIS_LEFT_REACH_MM * CHASSIS_LEFT_REACH_MM)
#define BEND_FRONT_RESERVE_MM (HUG_LONG_END_MM - LOOK_SONAR_FWD_MM + HUG_FORWARD_RESERVE_MM)
#define BEND_OUTER_TRAVEL_UM ((uint32_t)(HUG_FORWARD_RESERVE_MM / HALVES) * UM_PER_MM)
STATIC_ASSERT(((HUG_SWEEP_RADIUS_MM - 1) * (HUG_SWEEP_RADIUS_MM - 1) < HUG_SWEEP_SQ_MM) &&
              (HUG_SWEEP_RADIUS_MM * HUG_SWEEP_RADIUS_MM >= HUG_SWEEP_SQ_MM),
              "turn radius includes sensors and caster");

/* Encoders only: no gyro; the uncalibrated compass by the motors is worse. */
int32_t hug_pose_heading_deci(hug_t const * p_hug)
{
    return geom_wrap_deci((int32_t)wheel_yaw_deci(p_hug->diff_um));
}

int32_t hug_pos_x_mm(hug_t const * p_hug)
{
    return p_hug->x_um / UM_PER_MM;
}

int32_t hug_pos_y_mm(hug_t const * p_hug)
{
    return p_hug->y_um / UM_PER_MM;
}

look_pose_t hug_axle_pose(hug_t const * p_hug)
{
    return (look_pose_t){hug_pos_x_mm(p_hug), hug_pos_y_mm(p_hug), hug_pose_heading_deci(p_hug)};
}

void hug_zero_out(hug_out_t * p_out)
{
    p_out->speed_mm_s = 0;
    p_out->turn_deci_s = 0;
    p_out->servo_deci = 0;
    p_out->seq = 0U;
    p_out->b_req = 0U;
    p_out->b_stop = 0U;
    p_out->b_timeout = 0U;
    p_out->b_scan_pivot = false;
    p_out->b_no_tape = false;
    p_out->b_no_exit = false;
    p_out->b_rejoined = false;
}

int32_t hug_pivot_turn_um(int32_t diff_um, uint8_t witness_pct)
{
    int32_t const pct = ((0U != witness_pct) && (witness_pct <= (uint8_t)PERCENT))
                            ? (int32_t)witness_pct : HUG_PIVOT_PCT;

    return (diff_um * pct) / PERCENT;
}

/* Wheel travel to pose. Wheels that run opposite turn the body less than
 * their travel says. */
void hug_odo_update(hug_t * p_hug, hug_in_t const * p_in)
{
    int32_t const dl = (int32_t)((uint32_t)p_in->left_um - p_hug->last_left_um);
    int32_t const dr = (int32_t)((uint32_t)p_in->right_um - p_hug->last_right_um);
    int32_t const ds = (dl + dr) / HALVES;
    int32_t dd = dr - dl;
    int32_t const before = hug_pose_heading_deci(p_hug);
    int32_t mid = 0;

    if ((BYPASS_IDLE == p_hug->bypass.phase) && ((0 != dl) || (0 != dr) || p_in->b_other))
    {
        p_hug->last_motion_ms = p_in->now_ms;
    }
    p_hug->last_left_um = (uint32_t)p_in->left_um;
    p_hug->last_right_um = (uint32_t)p_in->right_um;
    if ((BYPASS_IDLE == p_hug->bypass.phase) && (((dl > 0) && (dr < 0)) || ((dl < 0) && (dr > 0))))
    {
        dd = hug_pivot_turn_um(dd, p_in->pivot_scale_pct);
    }
    dd += (int32_t)p_in->drift_corr_um;
    p_hug->diff_um += dd;
    mid = before + (geom_wrap_deci(hug_pose_heading_deci(p_hug) - before) / HALVES);
    p_hug->x_um += (int32_t)(((int64_t)ds * geom_cos_q14(mid)) / GEOM_Q14_ONE);
    p_hug->y_um += (int32_t)(((int64_t)ds * geom_sin_q14(mid)) / GEOM_Q14_ONE);
}

/* After a stall hug the pose runs while the hug is idle: x is the travel along
 * the heading of the first hit, from the pose where the hug began, which is
 * the pilot's back-off short of the stall. The note clears at HUG_STALL_PAST_MM
 * and with time. Wheel slip against the face counts as travel. */
void hug_note_update(hug_t * p_hug, hug_in_t const * p_in)
{
    hug_odo_update(p_hug, p_in);
    if (hug_pos_x_mm(p_hug) >= HUG_STALL_PAST_MM)
    {
        p_hug->b_note = 0U;
    }
    if ((p_in->now_ms - p_hug->t_ms) > HUG_NOTE_MS)
    {
        p_hug->b_note = 0U;
        p_hug->n_rel = 0U;
    }
}

bool hug_forward_ping(hug_t * p_hug, hug_in_t const * p_in, bool * p_b_guard)
{
    bool const b_new = !p_in->b_ping_reply && (p_in->ping_seq != p_hug->last_seq);
    bool b_hit = false;

    *p_b_guard = false;
    if (b_new)
    {
        bool const b_valid = (LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > 0U);
        bool const b_too_close = (LOOK_ECHO_TOO_CLOSE == p_in->ping_echo);

        if (p_in->ping_seq != (((uint32_t)p_hug->last_seq + 1U) & SEQ_MASK))
        {
            p_hug->b_bend_check = false;
        }
        p_hug->last_seq = p_in->ping_seq;
        b_hit = b_too_close || (b_valid && (p_in->ping_mm <= (uint16_t)HUG_STANDOFF_MM));
        if (b_hit)
        {
            /* A saturated count survives hits so the trigger can see the contact that follows the quiet run. */
            p_hug->quiet_pings = (p_hug->quiet_pings < HIST_BITS) ? 0U : p_hug->quiet_pings;
        }
        else if (((LOOK_ECHO_OPEN == p_in->ping_echo) || b_valid) &&
                 (p_hug->quiet_pings < HIST_BITS))
        {
            p_hug->quiet_pings = (uint8_t)(p_hug->quiet_pings + 1U);
        }
        *p_b_guard = b_too_close || (b_valid && (p_in->ping_mm <= (uint16_t)HUG_GUARD_MM));
        p_hug->hist = (uint8_t)(((uint32_t)p_hug->hist << 1U) | (b_hit ? 1U : 0U));
        p_hug->hist = (uint8_t)((uint32_t)p_hug->hist & HIST_MASK);
    }
    return b_new;
}

bool hug_triggered(hug_t const * p_hug)
{
    uint32_t hits = 0U;

    for (uint32_t i = 0U; i < HIST_BITS; i++)
    {
        hits += ((uint32_t)p_hug->hist >> i) & 1U;
    }
    return hits >= HUG_TRIGGER_HITS;
}

/* Relative to the instantaneous turn centre, every body corner travels
 * within the circle through the outermost corner. Compare the nearest
 * possible echo in the beam to that circle, including tracking margin. */
static bool echo_outside_bend(uint16_t radius_mm, uint16_t range_mm, bool b_left_turn)
{
    int32_t const radius = radius_mm;
    int32_t const margin = (int32_t)OBSTACLE_PLAN_MARGIN_MM;
    int32_t const outer = radius + (int32_t)(b_left_turn ? CHASSIS_RIGHT_REACH_MM : CHASSIS_LEFT_REACH_MM) + margin;
    int32_t const end = HUG_LONG_END_MM + margin;
    int32_t const ahead = LOOK_SONAR_FWD_MM + geom_mul_q14((int32_t)range_mm - (int32_t)OBSTACLE_AGREE_MM,
                                                           geom_cos_q14(CHASSIS_SONAR_OUTER_HALF_DECI));
    int32_t const across = radius - geom_mul_q14((int32_t)range_mm + (int32_t)OBSTACLE_AGREE_MM,
                                                 geom_sin_q14(CHASSIS_SONAR_OUTER_HALF_DECI)) - 1;
    int32_t const nearest = (across > 0) ? across : 0;
    /* Difference of squares keeps even a nearly straight radius in int32. */
    return (ahead * ahead - end * end) > ((outer - nearest) * (outer + nearest));
}

bool hug_recheck_on_bend(hug_t * p_hug, hug_in_t const * p_in, bool b_forward)
{
    bool const b_center = !p_in->b_ping_reply && (0 == p_in->ping_servo_deci);
    bool const b_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
        ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > HUG_STANDOFF_MM));
    if (p_in->b_other || !b_center || (0U == p_in->bend_radius_mm))
    {
        p_hug->b_bend_check = false;
        return false;
    }
    if (p_hug->b_bend_check && b_clear)
    {
        /* The first clear ping can still have two old hits behind it. Two
         * consecutive clears end this observation window. */
        if (b_forward && ((p_hug->hist & (HIST_MASK >> 1U)) == 0U))
        {
            p_hug->b_bend_check = false;
        }
        return true;
    }
    if ((LOOK_ECHO_VALID != p_in->ping_echo) || (p_in->ping_mm <= BEND_FRONT_RESERVE_MM) ||
        (p_in->ping_mm > HUG_STANDOFF_MM) || !echo_outside_bend(p_in->bend_radius_mm, p_in->ping_mm, p_in->b_line_left))
    {
        p_hug->b_bend_check = false;
        return false;
    }
    uint32_t const outer = (uint32_t)(p_in->b_line_left ? p_in->right_um : p_in->left_um);
    if (!p_hug->b_bend_check && b_forward && (1U == p_hug->hist))
    {
        p_hug->anchor.bend.outer_um = outer;
        p_hug->anchor.bend.start_ms = p_in->now_ms;
        p_hug->b_bend_check = true;
    }
    /* A point return supplies no obstacle width. A surface that stays in
     * view as the wheels turn must get the ordinary stationary survey. */
    return p_hug->b_bend_check && (outer - p_hug->anchor.bend.outer_um < BEND_OUTER_TRAVEL_UM) &&
           (p_in->now_ms - p_hug->anchor.bend.start_ms < HUG_REPLY_TIMEOUT_MS);
}

void hug_start_pivot(hug_t * p_hug, int32_t head, uint32_t then)
{
    p_hug->b_pivot_reaimed = false;
    p_hug->state = (uint8_t)ST_PIVOT;
    p_hug->sub = (uint8_t)SUB_START;
    p_hug->goal_head = (int16_t)head;
    p_hug->then = (uint8_t)then;
}

void hug_start_drive(hug_t * p_hug, int32_t len_mm, int32_t head, uint32_t then)
{
    p_hug->state = (uint8_t)ST_DRIVE;
    p_hug->sub = (uint8_t)SUB_START;
    p_hug->goal_head = (int16_t)head;
    p_hug->goal_len_mm = (int16_t)len_mm;
    p_hug->origin.drive.x_mm = hug_pos_x_mm(p_hug);
    p_hug->origin.drive.y_mm = hug_pos_y_mm(p_hug);
    p_hug->then = (uint8_t)then;
}

/* The first contact nearer than the gap, or a stall: back straight off along
 * the heading of the contact to the standoff from the face, then look again.
 * The contact look has turned the car away from that heading, so it turns
 * back first (proceed starts the drive, which keeps its length here). The
 * range is from the sonar, as HUG_STANDOFF_MM is. */
void hug_start_back(hug_t * p_hug, int32_t range_mm)
{
    hug_flag_set(p_hug, F_BACKED, true);
    p_hug->goal_len_mm = (int16_t)(range_mm - HUG_STANDOFF_MM);
    hug_start_pivot(p_hug, p_hug->center, THEN_BACK);
}

/* The kind of a drive, from what follows it. A leg goes along a face and a
 * look follows it: the tape past the hit ends it, a face ahead under the
 * guard cuts it short, and it runs at HUG_LEG_MM_S. The other drives (the
 * back-off step, the approach and the run onto the tape) end at their length
 * only, at HUG_SHORT_MM_S. */
static bool hug_drive_is_leg(hug_t const * p_hug)
{
    return THEN_LOOK == p_hug->then;
}

/* Turn in place toward the target. True when within the tolerance. */
bool hug_pivot_cmd(hug_t const * p_hug, int32_t target, hug_out_t * p_out)
{
    int32_t const err = geom_wrap_deci(target - hug_pose_heading_deci(p_hug));
    bool const b_done = geom_abs(err) <= HUG_HEAD_TOL_DECI;

    if (!b_done)
    {
        int32_t const rate = clamp_i32(geom_abs(err) * HUG_PIVOT_GAIN,
                                       HUG_PIVOT_MIN_DECI_S, HUG_PIVOT_MAX_DECI_S);

        p_out->turn_deci_s = (int16_t)((err > 0) ? rate : -rate);
    }
    return b_done;
}

static int32_t hug_drive_remaining_mm(hug_t const * p_hug)
{
    int32_t const dx = hug_pos_x_mm(p_hug) - p_hug->origin.drive.x_mm;
    int32_t const dy = hug_pos_y_mm(p_hug) - p_hug->origin.drive.y_mm;
    int32_t const travelled = (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy)));
    int32_t const remaining = geom_abs(p_hug->goal_len_mm) - travelled;
    return (remaining > 0) ? remaining : 0;
}

static bool hug_drive_cmd(hug_t const * p_hug, int32_t speed_mm_s, hug_out_t * p_out)
{
    bool const b_done = (0 == hug_drive_remaining_mm(p_hug));

    if (!b_done)
    {
        int32_t const err = geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug));

        p_out->speed_mm_s = (int16_t)((p_hug->goal_len_mm < 0) ? -speed_mm_s : speed_mm_s);
        p_out->turn_deci_s = (int16_t)clamp_i32(err * HUG_HOLD_GAIN, -HUG_HOLD_MAX_DECI_S,
                                                HUG_HOLD_MAX_DECI_S);
    }
    return b_done;
}

bool hug_resume_remaining(hug_t * p_hug, int64_t left_um, int64_t right_um, uint32_t now_ms)
{
    bool const b_offered = p_hug->b_can_resume && !hug_active(p_hug);
    p_hug->b_can_resume = false;
    if (b_offered)
    {
        /* A local sweep pivots in place and returns to its starting heading.
         * A translated car no longer has this surveyed return pose. */
        int32_t const dl = (int32_t)((uint32_t)left_um - p_hug->last_left_um);
        int32_t const dr = (int32_t)((uint32_t)right_um - p_hug->last_right_um);
        int64_t const translation = (int64_t)dl + dr;
        int64_t const yaw = ((int64_t)dr - dl) * DECI_PER_RAD;
        int64_t const yaw_limit = (int64_t)HUG_HEAD_TOL_DECI * CHASSIS_WHEEL_TRACK_MM * UM_PER_MM;
        if ((translation < -(LINE_WIDTH_MM * UM_PER_MM)) || (translation > (LINE_WIDTH_MM * UM_PER_MM)) ||
            (yaw < -yaw_limit) || (yaw > yaw_limit))
        {
            return false;
        }
        p_hug->last_left_um = (uint32_t)left_um;
        p_hug->last_right_um = (uint32_t)right_um;
        p_hug->diff_um = 0; /* restored entry heading */
        p_hug->b_remaining = true;
        p_hug->last_motion_ms = now_ms;
        hug_flag_set(p_hug, F_EXIT | F_EXIT_TAPE | F_EDGE_LEG, false);
        hug_look_along_face(p_hug, PU_LOOK);
    }
    return b_offered;
}

/* The time the move that starts now may take: HUG_MOVE_TIMES times its plan
 * (hug.h). */
uint32_t hug_move_limit_ms(hug_t const * p_hug)
{
    uint32_t plan_ms = 0U;

    if (ST_DRIVE == p_hug->state)
    {
        int32_t const speed = hug_drive_is_leg(p_hug) ? HUG_LEG_MM_S : HUG_SHORT_MM_S;

        plan_ms = ((uint32_t)geom_abs(p_hug->goal_len_mm) * MS_PER_S) / (uint32_t)speed;
    }
    else
    {
        int32_t const target = (ST_SCAN == p_hug->state)
                                   ? (p_hug->center
                                      + hug_scan_body_aim(p_hug, hug_ray_offset(p_hug, p_hug->idx)))
                                   : p_hug->goal_head;
        uint32_t const angle = (uint32_t)geom_abs(geom_wrap_deci(target - hug_pose_heading_deci(p_hug)));

        plan_ms = (angle * MS_PER_S * (uint32_t)PERCENT)
                  / ((uint32_t)HUG_PIVOT_MIN_DECI_S * (uint32_t)HUG_PIVOT_PCT);
    }
    return HUG_MOVE_TIMES * ((plan_ms < HUG_MOVE_FLOOR_MS) ? HUG_MOVE_FLOOR_MS : plan_ms);
}

/* Distance from the saved vertex along an axis in the contact frame. */
int32_t hug_from_edge_mm(hug_t const * p_hug, int32_t axis)
{
    uint8_t const side = (p_hug->side > 0) ? LEFT : RIGHT;
    return geom_dot_mm(axis, hug_pos_x_mm(p_hug) - p_hug->keep_x_mm[side],
                       hug_pos_y_mm(p_hug) - p_hug->keep_y_mm[side]);
}

/* Where the lateral setup leg ends. The following straight line passes the
 * vertex at half the body width plus the working gap, measured perpendicular
 * to travel. The pivot here still has the full front standoff. */
/* Passing the obstacle on its right puts the left reader next to its face. */
int32_t hug_obstacle_side_reach_mm(hug_t const * p_hug)
{
    return (int32_t)((p_hug->side < 0) ? CHASSIS_LEFT_REACH_MM : CHASSIS_RIGHT_REACH_MM);
}

static int32_t flank_setup_mm(int32_t front_mm, int32_t side_reach_mm)
{
    int32_t const across = side_reach_mm + HUG_FLANK_GAP_MM
                          - geom_mul_q14(front_mm, geom_sin_q14(HUG_FLANK_DECI));
    int32_t const cosine = geom_cos_q14(HUG_FLANK_DECI);
    return (across * GEOM_Q14_ONE + cosine - 1) / cosine;
}

static int32_t approach_mm(hug_t const * p_hug, int32_t dist_mm)
{
    /* After a stall the measured approach must also clear the stall point. */
    int32_t const limit = hug_flag_get(p_hug, F_STALL) ? HUG_STANDOFF_MM - HUG_BODY_GAP_MM : HOP_LIMIT_MM;
    int32_t const travel = clamp_i32(dist_mm - HUG_AXLE_OFF_MM, 0, limit);
    return (travel >= HUG_APPROACH_MIN_MM) ? travel : 0;
}

/* From the face and its edges: the heading along the face, the length of the
 * leg, and for the first leg the approach to the gap. A face assumed at a stall
 * is not measured, and a narrow face gives a poor distance, so their legs are
 * not trimmed. */
void hug_plan_leg(hug_t * p_hug, look_face_t const * p_face,
                  hug_edges_t const * p_edges, bool b_first, look_t const * p_look)
{
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const along = geom_wrap_deci(p_face->normal_deci + (p_hug->side * QUARTER_DECI));
    int32_t const sonar_mm = p_face->dist_mm - LOOK_SONAR_FWD_MM;
    int32_t const min_leg = (p_hug->b_close_front || (p_hug->side_end_x_mm < 0)) ? HUG_APPROACH_MIN_MM : HUG_LEG_MIN_MM;
    int32_t const app = b_first ? approach_mm(p_hug, p_face->dist_mm) : 0;
    int32_t leg = 0;
    int32_t trim = 0;
    bool const b_measured = (NULL != p_look) && !p_edges->b_narrow;
    /* Width and trim need a well-conditioned normal. A bounded front
     * with two hit-to-open edges can still be approached for a fresh
     * side survey, which must independently confirm its next face. */
    bool const b_bounded = b_first && p_edges->have[LEFT] && p_edges->have[RIGHT];

    hug_flag_set(p_hug, F_ASSUME, false);
    hug_flag_set(p_hug, F_PAST, p_edges->have[s]);
    hug_flag_set(p_hug, F_EDGE_LEG, p_edges->have[s] && (!p_edges->b_narrow || b_bounded || p_hug->b_oblique));
    hug_flag_set(p_hug, F_MEASURED_FACE, b_measured);
    if (b_measured && (PU_CHECK != p_hug->purpose))
    {
        p_hug->face_line_mm = p_face->dist_mm + geom_dot_mm(p_face->normal_deci,
                                                           hug_pos_x_mm(p_hug), hug_pos_y_mm(p_hug));
    }
    if (hug_flag_get(p_hug, F_EDGE_LEG))
    {
        /* The refine buffers are free once the leg has been planned. */
        p_hug->keep_x_mm[s] = (int16_t)p_edges->edge[s].x_mm;
        p_hug->keep_y_mm[s] = (int16_t)p_edges->edge[s].y_mm;
    }
    if (p_edges->have[s])
    {
        leg = geom_dot_mm(along, p_edges->edge[s].x_mm - hug_pos_x_mm(p_hug),
                     p_edges->edge[s].y_mm - hug_pos_y_mm(p_hug));
        if (p_hug->b_close_front)
        {
            leg += flank_setup_mm(p_face->dist_mm - app, hug_obstacle_side_reach_mm(p_hug));
        }
        else
        {
            leg += HUG_AXLE_OFF_MM
                   + ((p_hug->b_oblique || (b_bounded && p_edges->b_narrow)) ? HUG_APPROACH_MIN_MM : 0);
        }
        if (b_first && b_measured && !p_hug->b_close_front)
        {
            /* The guarded approach to the first side can run inward if
             * the front is oblique to the entry heading. Reserve that
             * lateral displacement before turning around its endpoint. */
            int32_t const inward = -p_hug->side *
                geom_mul_q14(p_face->dist_mm - app, geom_sin_q14(p_face->normal_deci));
            leg += (inward > 0) ? inward : 0;
        }
    }
    else if (b_measured)
    {
        look_pose_t const pose = hug_axle_pose(p_hug);
        look_side_t const side = (p_hug->side > 0) ? LOOK_SIDE_LEFT : LOOK_SIDE_RIGHT;
        look_side_t const other = (LOOK_SIDE_LEFT == side) ? LOOK_SIDE_RIGHT : LOOK_SIDE_LEFT;
        int32_t const reach = look_face_reach_mm(p_look, &pose, p_face, side);
        int32_t const behind = look_face_reach_mm(p_look, &pose, p_face, other);
        /* Buddy 5 advances past the last on-face return by a half lane.
         * Keep successive views overlapping with this narrower servo.
         * While seeking the third vertex, stop at the observed extent. */
        int32_t const lane = (int32_t)OBSTACLE_CAR_HALF_W_MM + HUG_LANE_BUFFER_MM;
        leg = reach + ((p_hug->side_end_x_mm < 0) ? 0 : clamp_i32(behind / HALVES, 0, lane));
    }
    else
    {
        leg = geom_mul_q14(sonar_mm - app, geom_tan_q14(HUG_HOP_ANGLE_DECI));
    }
    leg = clamp_i32(leg, min_leg, HUG_LEG_MAX_MM);
    if (b_measured && !p_hug->b_close_front)
    {
        trim = clamp_i32(geom_atan_deci(p_face->dist_mm - app - HUG_AXLE_OFF_MM, leg),
                         -HUG_TRIM_MAX_DECI, HUG_TRIM_MAX_DECI);
    }
    /* The leg is the way along the face; leaning makes the way longer. */
    leg = clamp_i32((leg * GEOM_Q14_ONE) / geom_cos_q14(trim), min_leg, HUG_LEG_MAX_MM);
    p_hug->u_head = (int16_t)geom_wrap_deci(along - (p_hug->side * trim));
    p_hug->look_head = (int16_t)p_face->normal_deci;
    p_hug->leg_mm = (int16_t)leg;
    p_hug->app_mm = (int16_t)app;
    p_hug->face_p_mm = (int16_t)(p_face->dist_mm - app);
    p_hug->seen_mm = (int16_t)p_face->dist_mm;
    if (app > 0)
    {
        hug_start_pivot(p_hug, p_face->normal_deci, THEN_APPROACH);
    }
    else
    {
        hug_start_pivot(p_hug, p_hug->u_head, THEN_LEG);
    }
}

/* Tape under a sensor outside the line the car came on. The line may run at
 * any angle from here: it can turn under the obstacle. */
void hug_observe_exit_tape(hug_t * p_hug, hug_in_t const * p_in)
{
    bool const b_approach = (hug_pos_x_mm(p_hug) <= HUG_EXIT_PAST_MM)
                            && (geom_abs(hug_pos_y_mm(p_hug)) <= HUG_LINE_WINDOW_MM);

    if (b_approach || !(p_in->b_line_left || p_in->b_line_right))
    {
        return;
    }
    if (p_hug->b_require_rear && ((p_hug->side_end_x_mm <= 0) ||
        (hug_pos_x_mm(p_hug) - p_hug->side_end_x_mm <= HUG_SWEEP_RADIUS_MM)))
    {
        return;
    }
    int32_t const head = hug_pose_heading_deci(p_hug);
    int32_t const left_mm = ((int32_t)p_in->b_line_left - (int32_t)p_in->b_line_right)
                            * (LINE_SENSOR_SPACING_MM / HALVES);
    int32_t const cos_head = geom_cos_q14(head);
    int32_t const sin_head = geom_sin_q14(head);
    int32_t const y_mm = hug_pos_y_mm(p_hug) + geom_mul_q14(LINE_SENSOR_LEAD_MM, sin_head)
                         + geom_mul_q14(left_mm, cos_head);
    /* Two dark readers farther apart laterally than one stripe can be a
     * bent exit here. The normal optical alignment still has to confirm it. */
    bool const b_transverse = p_in->b_line_left && p_in->b_line_right &&
        (geom_abs(geom_mul_q14(LINE_SENSOR_SPACING_MM, cos_head)) > LINE_WIDTH_MM);
    if (!b_transverse && (geom_abs(geom_abs(y_mm) - HUG_PARALLEL_SPACING_MM) <= HUG_TAPE_TOL_MM))
    {
        return;
    }
    p_hug->tape.point.y_mm = y_mm;
    p_hug->tape.point.x_mm = hug_pos_x_mm(p_hug) + geom_mul_q14(LINE_SENSOR_LEAD_MM, cos_head)
                             - geom_mul_q14(left_mm, sin_head);
    hug_flag_set(p_hug, F_EXIT_TAPE, true);
}

/* Only shorten the forward guard for a measured, aligned straight leg.
 * Reserve the complete turning envelope, the working gap, and one settling
 * interval at leg speed beyond its endpoint. */
static bool side_leg_blocked(hug_in_t const * p_in, int32_t remaining_mm)
{
    int32_t const coast = (int32_t)HUG_SETTLE_MS * HUG_LEG_MM_S / (int32_t)MS_PER_S;
    /* Keep the head-on stopping reserve independent of the side working gap. */
    int32_t const clearance = remaining_mm + HUG_SWEEP_RADIUS_MM - LOOK_SONAR_FWD_MM
                              + HUG_FORWARD_RESERVE_MM + coast;
    return (LOOK_ECHO_VALID != p_in->ping_echo) || (0 != p_in->ping_servo_deci) ||
           ((int32_t)p_in->ping_mm <= clearance);
}

void hug_drive_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                    bool b_guard, hug_out_t * p_out)
{
    if (THEN_FLANK_LOOK == p_hug->then)
    {
        int32_t const side = hug_from_edge_mm(p_hug, hug_along_head(p_hug));
        bool const b_pivot_clear = side >= HUG_SCAN_CLEARANCE_MM;
        if (p_in->b_stall || b_guard || (!b_pivot_clear && (0 == hug_drive_remaining_mm(p_hug))))
        {
            /* A turn to investigate here could swing the nose into the post. */
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else if (b_pivot_clear && p_in->b_stationary && (SUB_SETTLE == p_hug->sub) &&
                 (p_in->now_ms - p_hug->t_ms >= HUG_SETTLE_MS))
        {
            hug_proceed(p_hug, p_look);
        }
        else if (b_pivot_clear)
        {
            if (SUB_SETTLE != p_hug->sub)
            {
                p_hug->sub = (uint8_t)SUB_SETTLE;
                p_hug->t_ms = p_in->now_ms;
            }
        }
        else
        {
            p_hug->sub = (uint8_t)SUB_MOVE;
            (void)hug_drive_cmd(p_hug, HUG_SHORT_MM_S, p_out);
        }
        return;
    }
    bool const b_leg = hug_drive_is_leg(p_hug);
    bool const b_blocked_leg = b_leg && b_guard &&
        (!hug_flag_get(p_hug, F_MEASURED_FACE) ||
         (geom_abs(geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug))) > HUG_HEAD_TOL_DECI) ||
         side_leg_blocked(p_in, hug_drive_remaining_mm(p_hug)));
    if (p_hug->b_probe_front && (p_in->b_stall || b_blocked_leg))
    {
        hug_finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else if (p_in->b_stall || b_blocked_leg || (b_guard &&
        ((THEN_EXIT_TURN == p_hug->then) || (THEN_REAR_ALIGN == p_hug->then))))
    {
        hug_flag_set(p_hug, F_STALL, p_in->b_stall);
        hug_flag_set(p_hug, F_EDGE_LEG | F_EXIT | F_EXIT_TAPE, false);
        hug_start_scan(p_hug, p_look, PU_INNER, hug_pose_heading_deci(p_hug));
    }
    else if (SUB_SETTLE == p_hug->sub)
    {
        if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
        {
            hug_proceed(p_hug, p_look);
        }
    }
    else if ((THEN_EXIT_TURN != p_hug->then) && hug_flag_get(p_hug, F_EXIT_TAPE))
    {
        int32_t const head = hug_pose_heading_deci(p_hug);
        /* A sensor can find the tape while the chassis is still turning.
         * The crossing distance belongs to that measured point. */
        int32_t const advance = geom_dot_mm(head, p_hug->tape.point.x_mm - hug_pos_x_mm(p_hug),
                                           p_hug->tape.point.y_mm - hug_pos_y_mm(p_hug));
        hug_flag_set(p_hug, F_EXIT, true);
        hug_flag_set(p_hug, F_EXIT_TAPE, false);
        hug_start_drive(p_hug, clamp_i32(advance, 0, LINE_SENSOR_LEAD_MM), head, THEN_EXIT_TURN);
    }
    else if (hug_drive_cmd(p_hug, hug_drive_is_leg(p_hug) ? HUG_LEG_MM_S : HUG_SHORT_MM_S, p_out))
    {
        p_hug->sub = (uint8_t)SUB_SETTLE;
        p_hug->t_ms = p_in->now_ms;
    }
    else
    {
        /* Driving. */
    }
}

/* A single detector must sweep across a measurable part of the stripe.
 * Both dark can be a broad patch. Both clear arms the next crossing. */
static bool exit_crossed(hug_t * p_hug, hug_in_t const * p_in)
{
    uint8_t const reader = (uint8_t)((p_in->b_line_left ? EXIT_LEFT : 0U) |
                                     (p_in->b_line_right ? EXIT_RIGHT : 0U));
    uint8_t const leading = (p_hug->side > 0) ? (uint8_t)EXIT_LEFT : (uint8_t)EXIT_RIGHT;
    int32_t const head = hug_pose_heading_deci(p_hug);
    if ((EXIT_CLEAR == reader) && (leading == p_hug->tape.crossing.reader))
    {
        int32_t const sweep = p_hug->side * geom_wrap_deci(head - p_hug->tape.crossing.heading_deci);
        int32_t const arc = sweep * LINE_SENSOR_LEAD_MM;
        p_hug->tape.crossing.reader = (uint8_t)EXIT_CLEAR;
        return (arc * HALVES >= LINE_WIDTH_MM * DECI_PER_RAD) &&
               (arc <= HALVES * LINE_WIDTH_MM * DECI_PER_RAD);
    }
    if (EXIT_CLEAR == reader)
    {
        p_hug->tape.crossing.reader = reader;
    }
    else if (leading != reader)
    {
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
    }
    else if ((reader != p_hug->tape.crossing.reader) && (EXIT_WAIT_CLEAR != p_hug->tape.crossing.reader))
    {
        p_hug->tape.crossing.reader = reader;
        p_hug->tape.crossing.heading_deci = (int16_t)head;
    }
    return false;
}

static bool exit_settled(hug_t const * p_hug)
{
    int32_t const dx = hug_pos_x_mm(p_hug) - p_hug->origin.drive.x_mm;
    int32_t const dy = hug_pos_y_mm(p_hug) - p_hug->origin.drive.y_mm;
    int32_t const angle = geom_abs(geom_wrap_deci(hug_pose_heading_deci(p_hug) - p_hug->tape.crossing.heading_deci));
    return (geom_abs(dx) + geom_abs(dy) <= LINE_WIDTH_MM / HALVES) &&
           (angle * LINE_SENSOR_LEAD_MM <= LINE_WIDTH_MM * DECI_PER_RAD);
}

/* Check fresh centered sonar after the wheels settle. A blocked or missing
 * result cannot transfer the wheels to the line search. */
static void rear_settled(hug_t * p_hug, hug_in_t const * p_in, bool b_forward, hug_out_t * p_out)
{
    bool const b_center = THEN_REAR_LATERAL == p_hug->then;
    int32_t const y = geom_abs(hug_pos_y_mm(p_hug));
    int32_t const min_range = b_center ? y + LINE_SENSOR_LEAD_MM - LOOK_SONAR_FWD_MM : HUG_GUARD_MM;
    bool const b_ping = b_forward && (0 == p_in->ping_servo_deci);
    bool const b_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
                         ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > min_range));
    bool const b_blocked = (LOOK_ECHO_TOO_CLOSE == p_in->ping_echo) ||
        ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > 0U) && !b_clear);
    if ((p_in->now_ms - p_hug->t_ms >= HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS) || (b_ping && b_blocked))
    {
        hug_finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else if (b_ping && b_clear && p_in->b_stationary)
    {
        if (geom_abs(geom_wrap_deci(hug_pose_heading_deci(p_hug) - p_hug->goal_head)) > HUG_HEAD_TOL_DECI)
        {
            p_hug->sub = (uint8_t)SUB_START;
        }
        else if (b_center)
        {
            hug_start_drive(p_hug, y, p_hug->goal_head, THEN_REAR_ALIGN);
        }
        else
        {
            hug_finish(p_hug);
            p_out->b_no_tape = y <= (LINE_WIDTH_MM / HALVES);
            p_out->b_timeout = (uint8_t)!p_out->b_no_tape;
            p_hug->b_can_resume = p_out->b_no_tape && p_hug->b_approach_tape;
            p_hug->b_approach_tape = false;
        }
    }
    else
    {
        /* Stand still until a centered reading arrives. */
    }
}

static void settle_optical_exit(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    if (!exit_settled(p_hug) || (p_in->b_line_left && p_in->b_line_right))
    {
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
        p_hug->sub = (uint8_t)SUB_START;
    }
    else
    {
        /* The leading detector crossed the stripe and the axle settled
         * within its measured footprint. Emit the transfer once. */
        hug_finish(p_hug);
        p_out->b_rejoined = true;
    }
}

/* A persistent small target may never supply a face normal from here.
 * Check both wide rays on the chosen side and measure one short step.
 * The normal leg admission checks the centered beam again after the pivot. */
static bool probe_sparse_front(hug_t * p_hug, look_t const * p_look, int32_t range_mm)
{
    if ((range_mm <= 0) || hug_flag_get(p_hug, F_RECHECK | F_STALL) ||
        !hug_prior_near_return(p_look, 0, range_mm))
    {
        return false;
    }
    int32_t const room_mm = HUG_SCAN_CLEARANCE_MM + HUG_LEG_MIN_MM;
    bool const b_left = !p_hug->b_left_scan_gap && (0U != p_hug->open_mm[LEFT]) &&
        ((0U == p_hug->wall_mm[LEFT]) || (p_hug->wall_mm[LEFT] > room_mm));
    bool const b_right = !p_hug->b_right_scan_gap && (0U != p_hug->open_mm[RIGHT]) &&
        ((0U == p_hug->wall_mm[RIGHT]) || (p_hug->wall_mm[RIGHT] > room_mm));
    if (!b_left && !b_right)
    {
        return false;
    }
    if (range_mm + LOOK_SONAR_FWD_MM < HUG_AXLE_OFF_MM)
    {
        if (hug_flag_get(p_hug, F_BACKED))
        {
            return false;
        }
        hug_start_back(p_hug, range_mm);
    }
    else
    {
        bool const b_prefer_left = (p_hug->open_mm[LEFT] >= p_hug->open_mm[RIGHT]);
        p_hug->side = (b_left && (!b_right || b_prefer_left)) ? (int8_t)1 : (int8_t)-1;
        p_hug->b_probe_front = true;
        p_hug->b_probe_retried = false;
        p_hug->face_p_mm = (int16_t)(LOOK_SONAR_FWD_MM + range_mm);
        p_hug->look_head = (int16_t)geom_atan_deci(-p_hug->side * HUG_LEG_MIN_MM, p_hug->face_p_mm);
        p_hug->leg_mm = HUG_LEG_MIN_MM;
        p_hug->u_head = (int16_t)(p_hug->side * QUARTER_DECI);
        hug_start_pivot(p_hug, p_hug->u_head, THEN_LEG);
    }
    return true;
}

static void settle_contact(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, bool b_forward)
{
    if ((p_in->now_ms - p_hug->t_ms) >= (HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS))
    {
        hug_restart_contact(p_hug, p_look);
    }
    else if (b_forward && p_in->b_stationary && (0 == p_in->ping_servo_deci) &&
             (geom_abs(hug_pose_heading_deci(p_hug)) <= HUG_HEAD_TOL_DECI))
    {
        if ((LOOK_ECHO_OPEN == p_in->ping_echo) ||
            ((LOOK_ECHO_VALID == p_in->ping_echo) && (p_in->ping_mm > HUG_GUARD_MM)))
        {
            hug_proceed(p_hug, p_look);
        }
        else if ((LOOK_ECHO_TOO_CLOSE == p_in->ping_echo) ||
                 ((LOOK_ECHO_VALID == p_in->ping_echo) && (0U != p_in->ping_mm) &&
                  !probe_sparse_front(p_hug, p_look, p_in->ping_mm)))
        {
            hug_restart_contact(p_hug, p_look);
        }
        else
        {
            /* An unresolved echo can still be replaced inside this reply window. */
        }
    }
    else
    {
        /* Only the settled beam on the approach heading can release this contact. */
    }
}

/* A side-looking ping from the survey cannot clear the forward leg. */
static void settle_leg(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                       bool b_forward, bool b_guard)
{
    int32_t const error = geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug));
    bool const b_aligned = (geom_abs(error) <= HUG_HEAD_TOL_DECI);
    bool const b_ready = b_aligned && b_forward && p_in->b_stationary && (0 == p_in->ping_servo_deci);
    bool const b_blocked = b_ready && b_guard &&
        (!hug_flag_get(p_hug, F_MEASURED_FACE) || side_leg_blocked(p_in, p_hug->leg_mm));
    if (b_blocked || ((p_in->now_ms - p_hug->t_ms) >= (HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS)))
    {
        if (p_hug->b_probe_front)
        {
            p_hug->b_probe_front = false;
            hug_restart_contact(p_hug, p_look);
        }
        else if (b_blocked)
        {
            hug_start_scan(p_hug, p_look, PU_INNER, hug_pose_heading_deci(p_hug));
        }
        else
        {
            hug_look_along_face(p_hug, PU_LOOK);
        }
    }
    else if (b_ready && ((LOOK_ECHO_OPEN == p_in->ping_echo) ||
             ((LOOK_ECHO_VALID == p_in->ping_echo) && (0U != p_in->ping_mm))))
    {
        hug_proceed(p_hug, p_look);
    }
    else
    {
        /* Still settling or waiting for an observation in the direction of travel. */
    }
}

void hug_pivot_tick(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                    bool b_forward, bool b_guard, hug_out_t * p_out)
{
    if (SUB_MOVE == p_hug->sub)
    {
        bool b_done = hug_pivot_cmd(p_hug, p_hug->goal_head, p_out);

        if (hug_flag_get(p_hug, F_EXIT))
        {
            /* Keep the braking travel inside the tape that the optical sweep measures. */
            p_out->turn_deci_s = (int16_t)clamp_i32(p_out->turn_deci_s, -HUG_PIVOT_MIN_DECI_S,
                                                   HUG_PIVOT_MIN_DECI_S);
        }
        /* The line may cross the way out at any angle, and the follower needs
         * a sensor on it: the turn stops when one sees it. */
        if (hug_flag_get(p_hug, F_EXIT) && exit_crossed(p_hug, p_in))
        {
            p_hug->tape.crossing.heading_deci = (int16_t)hug_pose_heading_deci(p_hug);
            p_hug->origin.drive.x_mm = hug_pos_x_mm(p_hug);
            p_hug->origin.drive.y_mm = hug_pos_y_mm(p_hug);
            p_out->turn_deci_s = 0;
            b_done = true;
        }
        else if (b_done && hug_flag_get(p_hug, F_EXIT))
        {
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
            return;
        }
        if (b_done)
        {
            p_hug->sub = (uint8_t)SUB_SETTLE;
            p_hug->t_ms = p_in->now_ms;
        }
        else
        {
            p_out->b_scan_pivot = THEN_SCAN == p_hug->then;
        }
    }
    else if ((p_in->now_ms - p_hug->t_ms) >= HUG_SETTLE_MS)
    {
        if (hug_flag_get(p_hug, F_EXIT))
        {
            settle_optical_exit(p_hug, p_in, p_out);
        }
        else if ((THEN_REAR_LATERAL == p_hug->then) || (THEN_REAR_READY == p_hug->then))
        {
            rear_settled(p_hug, p_in, b_forward, p_out);
        }
        else if (((THEN_RELEASE == p_hug->then) || (THEN_LEG == p_hug->then)) &&
                 p_in->b_stationary && !p_hug->b_pivot_reaimed &&
                 (p_in->now_ms - p_hug->t_ms < HUG_SETTLE_MS + HUG_REPLY_TIMEOUT_MS) &&
                 (geom_abs(geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug))) > HUG_HEAD_TOL_DECI))
        {
            p_hug->b_pivot_reaimed = true;
            p_hug->sub = (uint8_t)SUB_START;
        }
        else if (THEN_RELEASE == p_hug->then)
        {
            settle_contact(p_hug, p_look, p_in, b_forward);
        }
        else if (THEN_LEG != p_hug->then)
        {
            hug_proceed(p_hug, p_look);
        }
        else
        {
            settle_leg(p_hug, p_look, p_in, b_forward, b_guard);
        }
    }
    else
    {
        /* Standing still. */
    }
}
