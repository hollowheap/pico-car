/** @file hug.c
 * @brief Obstacle hug states and transitions between scans and motions.
 */

#include "internal.h"

#include <stddef.h>

#include "obstacle/obstacle.h"
#include "motion/pilot/pilot.h"
#include "ir/ir.h"

/* One perimeter plus the observation turn towards a face. */
#define REMAINING_LIMIT_DECI  (3600 + QUARTER_DECI)
#define ODD_SEQ         (0xFFU)
#define FACE_SPREAD_MM  (15)

bool hug_flag_get(hug_t const * p_hug, uint32_t mask)
{
    return 0U != ((uint32_t)p_hug->flags & mask);
}

void hug_flag_set(hug_t * p_hug, uint32_t mask, bool b_on)
{
    uint32_t flags = p_hug->flags;

    flags = b_on ? (flags | mask) : (flags & ~mask);
    p_hug->flags = (uint16_t)flags;
}

/* The heading along the face the last look was square to. */
int32_t hug_along_head(hug_t const * p_hug)
{
    return geom_wrap_deci(p_hug->look_head + (p_hug->side * QUARTER_DECI));
}

/* The lead-side probe must still see the part of the face measured before
 * the leg, even at the allowed aim error toward travel. */
static bool hug_side_check_supported(hug_t const * p_hug, look_t const * p_look)
{
    int32_t const normal = p_hug->look_head;
    look_pose_t const pose = hug_axle_pose(p_hug);
    int32_t const expected = p_hug->face_line_mm - geom_dot_mm(normal, pose.x_mm, pose.y_mm);
    look_face_t const face = {0, 0, normal, expected};
    look_side_t const side = (p_hug->side > 0) ? LOOK_SIDE_LEFT : LOOK_SIDE_RIGHT;
    int32_t const sonar_normal = geom_mul_q14(LOOK_SONAR_FWD_MM, geom_cos_q14(HUG_HEAD_TOL_DECI));
    int32_t const sonar_lead = geom_mul_q14(LOOK_SONAR_FWD_MM, geom_sin_q14(HUG_HEAD_TOL_DECI));
    int32_t const lead_mm = sonar_lead + geom_mul_q14(expected - sonar_normal,
        geom_tan_q14(PROBE_STEP_DECI + HUG_HEAD_TOL_DECI - LOOK_BEAM_DECI));

    int32_t const reach = look_face_reach_mm(p_look, &pose, &face, side);
    return (reach > 0) && (reach >= lead_mm);
}

static void hug_set_next_face(hug_t * p_hug)
{
    hug_flag_set(p_hug, F_PAST | F_MEASURED_FACE, false);
    hug_flag_set(p_hug, F_TURNED, true);
    p_hug->trav_head = p_hug->center;
    p_hug->look_head = (int16_t)geom_wrap_deci(p_hug->center - (p_hug->side * QUARTER_DECI));
}

static int32_t flank_drive_mm(hug_t const * p_hug)
{
    int32_t const outward = p_hug->side * QUARTER_DECI;
    int32_t const front = -hug_from_edge_mm(p_hug, p_hug->look_head);
    int32_t const nearest = hug_from_edge_mm(p_hug, p_hug->center + outward);
    if ((front < HUG_SCAN_CLEARANCE_MM) ||
        (nearest < hug_obstacle_side_reach_mm(p_hug) + (int32_t)OBSTACLE_PLAN_MARGIN_MM))
    {
        return 0;
    }
    int32_t const gain = HUG_SCAN_CLEARANCE_MM - hug_from_edge_mm(p_hug, p_hug->look_head + outward);
    /* Cover the shallowest allowed heading. Odometry ends the leg as soon
     * as the chassis can turn at its new observation station. */
    int32_t const sine = geom_sin_q14(HUG_FLANK_DECI - HUG_HEAD_TOL_DECI);
    return (gain * GEOM_Q14_ONE + sine - 1) / sine;
}

void hug_after_corner(hug_t * p_hug, look_t const * p_look, hug_out_t * p_out)
{
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    /* Stop abreast of the measured endpoint to look sideways. A fixed
     * advance past its plane can skip the whole next face of a small box. */
    bool const b_mid = hug_facet_clearance(p_hug);
    int32_t leg_mm = b_mid ?
        geom_dot_mm(p_hug->center, p_hug->origin.drive.x_mm - hug_pos_x_mm(p_hug),
                    p_hug->origin.drive.y_mm - hug_pos_y_mm(p_hug)) / HALVES :
        geom_dot_mm(p_hug->center, p_hug->keep_x_mm[s] - hug_pos_x_mm(p_hug),
                    p_hug->keep_y_mm[s] - hug_pos_y_mm(p_hug));
    if (p_hug->b_close_front)
    {
        leg_mm = flank_drive_mm(p_hug);
    }
    int32_t const min_mm = leg_mm + HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM;
    bool b_clear = (PROBE_RAYS == p_look->count) && (leg_mm >= HUG_LEG_MIN_MM) && (leg_mm <= HUG_LEG_MAX_MM);

    for (uint8_t i = 0U; b_clear && (i < p_look->count); i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(p_ray) - p_hug->center;
        int32_t const along_mm = geom_mul_q14(p_ray->range_mm, geom_cos_q14(angle));

        b_clear = (LOOK_ECHO_OPEN == look_ray_echo(p_ray)) ||
                  ((LOOK_ECHO_VALID == look_ray_echo(p_ray)) && (along_mm >= min_mm));
    }
    if (b_clear && p_hug->b_close_front)
    {
        hug_flag_set(p_hug, F_EXIT_TAPE, false);
        hug_start_drive(p_hug, leg_mm, p_hug->center, THEN_FLANK_LOOK);
    }
    else if (b_clear)
    {
        if (!b_mid)
        {
            hug_set_next_face(p_hug);
        }
        hug_start_drive(p_hug, (!b_mid && p_hug->b_oblique) ? leg_mm / HALVES : leg_mm, p_hug->center, THEN_LOOK);
    }
    else if (b_mid)
    {
        hug_finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else
    {
        hug_look_along_face(p_hug, PU_LOOK);
    }
}

/* Confirm the existing plane at separated bearings. A short check cannot
 * fit a new normal or infer a new edge, and it never advances the plane. */
void hug_after_side_check(hug_t * p_hug, look_t const * p_look)
{
    int32_t const normal = p_hug->look_head;
    int32_t const axle = geom_dot_mm(normal, hug_pos_x_mm(p_hug), hug_pos_y_mm(p_hug));
    int32_t const expected_mm = p_hug->face_line_mm - axle;
    int32_t near_mm = INT32_MAX;
    int32_t far_mm  = INT32_MIN;
    bool b_confirmed = (PROBE_RAYS == p_look->count);

    for (uint8_t i = 0U; b_confirmed && (i < p_look->count); i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(p_ray) - normal;
        int32_t const normal_mm = geom_dot_mm(normal, p_ray->x_mm, p_ray->y_mm) - axle
                            + geom_mul_q14(p_ray->range_mm, geom_cos_q14(angle));

        b_confirmed = (LOOK_ECHO_VALID == look_ray_echo(p_ray)) &&
                      (geom_abs(normal_mm - expected_mm) <= LOOK_FACE_MATCH_MM);
        near_mm = (normal_mm < near_mm) ? normal_mm : near_mm;
        far_mm = (normal_mm > far_mm) ? normal_mm : far_mm;
    }
    if (b_confirmed && (near_mm >= (HUG_AXLE_OFF_MM - LOOK_FACE_MATCH_MM)) &&
        ((far_mm - near_mm) <= FACE_SPREAD_MM))
    {
        look_face_t const face = {0, 0, normal, near_mm};
        hug_edges_t const edges = {0};

        hug_plan_leg(p_hug, &face, &edges, false, p_look);
    }
    else
    {
        hug_look_along_face(p_hug, PU_LOOK);
    }
}

/* A quadrilateral's measured side endpoint must lie on the observed rear
 * plane. The whole swept body must clear that plane at both ends of the
 * straight traverse. This also bounds clearance during the two pivots. */
static bool begin_rear_return(hug_t * p_hug, look_face_t const * p_face)
{
    int32_t const normal = p_face->normal_deci;
    int32_t const x = hug_pos_x_mm(p_hug);
    int32_t const y = hug_pos_y_mm(p_hug);
    int32_t const radius = HUG_SWEEP_RADIUS_MM;
    uint8_t const side = (p_hug->side > 0) ? LEFT : RIGHT;
    bool const b_joined = p_hug->b_oblique && (PU_MID_LOOK == p_hug->purpose) && hug_flag_get(p_hug, F_TURNED);
    /* A chamfer links the side endpoint to a second measured endpoint on
     * the rear plane. Use that connected endpoint for the plane check. */
    int32_t const end_x = b_joined ? p_hug->keep_x_mm[side] : p_hug->side_end_x_mm;
    int32_t const end_y = b_joined ? p_hug->keep_y_mm[side] : p_hug->side_end_y_mm;
    int32_t const end_plane = geom_dot_mm(normal, end_x - x, end_y - y);
    int32_t const target_gap = p_face->dist_mm + geom_mul_q14(y, geom_sin_q14(normal));
    bool const b_verified = !p_hug->b_remaining && (p_hug->side_end_x_mm > 0) && !hug_flag_get(p_hug, F_EXIT_TAPE) &&
        (geom_abs(geom_wrap_deci(normal - HALVES * QUARTER_DECI)) <= HUG_FACE_ANGLE_DECI) &&
        (geom_abs(end_plane - p_face->dist_mm) <= HUG_REFINE_TOL_MM) &&
        (p_face->dist_mm >= (radius + HUG_REFINE_TOL_MM)) && (target_gap >= (radius + HUG_REFINE_TOL_MM)) &&
        (geom_abs(y) <= HUG_LEG_MAX_MM);
    if (b_verified)
    {
        p_hug->look_head = (int16_t)normal;
        p_hug->face_p_mm = (int16_t)target_gap;
        if (geom_abs(y) <= (LINE_WIDTH_MM / HALVES))
        {
            hug_start_pivot(p_hug, 0, THEN_REAR_READY);
        }
        else
        {
            hug_start_pivot(p_hug, (y > 0) ? -QUARTER_DECI : QUARTER_DECI, THEN_REAR_LATERAL);
        }
    }
    return b_verified;
}

static void follow_measured_face(hug_t * p_hug, look_t const * p_look, look_face_t const * p_face)
{
    hug_edges_t edges;
    hug_find_edges(p_look, p_face, p_hug, &edges);
    /* Only a measured face running along the approach can supply the
     * rearward endpoint. The other front corner says nothing about it. */
    uint8_t const s = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const side_normal = -p_hug->side * QUARTER_DECI;
    bool const b_same_side = (p_hug->side_end_x_mm < 0) && !hug_flag_get(p_hug, F_TURNED) &&
        (geom_abs(geom_wrap_deci(p_hug->center - side_normal)) <= HUG_FACE_ANGLE_DECI);
    bool const b_side = b_same_side || (geom_abs(geom_wrap_deci(p_face->normal_deci - side_normal)) <= HUG_FACE_ANGLE_DECI);
    if (b_side && (0 >= p_hug->side_end_x_mm))
    {
        if (edges.have[s] && (edges.edge[s].x_mm > 0))
        {
            p_hug->side_end_x_mm = (int16_t)edges.edge[s].x_mm;
            p_hug->side_end_y_mm = (int16_t)edges.edge[s].y_mm;
        }
        else
        {
            p_hug->side_end_x_mm = -1;
        }
    }
    if (hug_flag_get(p_hug, F_TURNED) && edges.have[LEFT] && edges.have[RIGHT] && !edges.b_narrow)
    {
        p_hug->breadth_mm = (uint16_t)hug_edges_apart_mm(&edges);
    }
    hug_flag_set(p_hug, F_TURNED, false);
    hug_plan_leg(p_hug, p_face, &edges, false, p_look);
}

void hug_after_look(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                    bool b_found, hug_out_t * p_out)
{
    /* The look after an assumed face is the first one of the real face, which
     * may be turned as far as an INNER look takes it. */
    bool const b_near = b_found && (hug_valid_rays(p_look) >= HUG_FACE_RAYS_MIN)
                        && (geom_abs(geom_wrap_deci(p_face->normal_deci - p_hug->center))
                            <= ((p_hug->b_probe_front || hug_flag_get(p_hug, F_ASSUME)) ?
                                HUG_INNER_ANGLE_DECI : HUG_FACE_ANGLE_DECI))
                        && (hug_flag_get(p_hug, F_TURNED)
                            ? (p_face->dist_mm <= (HUG_AXLE_OFF_MM + HUG_FACE_TOL_MM))
                            : (geom_abs(p_face->dist_mm - p_hug->face_p_mm) <= HUG_FACE_TOL_MM));

    if (p_hug->b_probe_front && !b_near)
    {
        /* Repeat the look at this station before spending any more
         * lateral clearance toward the neighboring line. */
        if (p_hug->b_probe_retried)
        {
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else
        {
            p_hug->b_probe_retried = true;
            hug_look_along_face(p_hug, PU_LOOK);
        }
    }
    else if (b_near)
    {
        p_hug->b_probe_front = false;
        if (p_hug->b_remaining && (geom_abs(p_face->normal_deci) <= HUG_FACE_ANGLE_DECI) &&
            (hug_pos_x_mm(p_hug) < p_hug->side_end_x_mm))
        {
            /* The outstanding arc has led back to the entry face. */
            hug_finish(p_hug);
            p_out->b_no_exit = true;
            return;
        }
        if (begin_rear_return(p_hug, p_face))
        {
            return;
        }
        if (p_hug->b_oblique && !p_hug->b_remaining &&
            ((geom_abs(geom_wrap_deci(p_face->normal_deci - HALVES * QUARTER_DECI)) <= HUG_FACE_ANGLE_DECI) ||
             (p_hug->side * p_face->normal_deci > QUARTER_DECI)))
        {
            /* Reaching the rear without a qualified centered traverse cannot
             * justify another lap around the same obstacle. */
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
            return;
        }
        follow_measured_face(p_hug, p_look, p_face);
    }
    else if (p_hug->b_oblique)
    {
        hug_finish(p_hug);
        p_out->b_timeout = 1U;
    }
    else if (p_hug->side_end_x_mm < 0)
    {
        if (PU_EDGE_RETRY == p_hug->purpose)
        {
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else
        {
            hug_look_along_face(p_hug, PU_EDGE_RETRY);
        }
    }
    else
    {
        if (hug_flag_get(p_hug, F_TURNED))
        {
            /* The next face is short and behind the car: go on until the
             * body clears its end, then look again. */
            hug_flag_set(p_hug, F_TURNED, false);
            p_hug->face_p_mm = (int16_t)HUG_AXLE_OFF_MM;
            p_hug->leg_mm = (int16_t)HUG_AXLE_OFF_MM;
            hug_start_pivot(p_hug, p_hug->trav_head, THEN_ADVANCE);
        }
        else if (!hug_flag_get(p_hug, F_PAST))
        {
            /* The face is gone, but a look beside a corner can lose it with the
             * corner just behind the car: go on a body and a gap along the face
             * before the turn, then look again. */
            hug_flag_set(p_hug, F_PAST, true);
            p_hug->leg_mm = (int16_t)HUG_AXLE_OFF_MM;
            hug_start_pivot(p_hug, hug_along_head(p_hug), THEN_ADVANCE);
        }
        else
        {
            /* The face is gone: the corner is behind the car. The next face
             * lies across the old heading of the leg, on the side of the face. */
            hug_set_next_face(p_hug);
            hug_start_pivot(p_hug, p_hug->center, THEN_TURN);
        }
    }
}

/* A successor must meet the actual vertex, including when the two normals
 * are oblique. A parallel side offset from that point can hide a chamfer. */
void hug_after_edge_sweep(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                          bool b_found, hug_out_t * p_out)
{
    uint8_t const side = (p_hug->side > 0) ? LEFT : RIGHT;
    int32_t const expected = geom_dot_mm(p_face->normal_deci, p_hug->keep_x_mm[side] - hug_pos_x_mm(p_hug),
                                         p_hug->keep_y_mm[side] - hug_pos_y_mm(p_hug));
    int32_t const angle = p_hug->side * geom_wrap_deci(p_face->normal_deci - p_hug->look_head);
    bool const b_mid = (PU_MID_LOOK == p_hug->purpose);
    bool const b_face = b_found && (hug_valid_rays(p_look) >= HUG_FACE_RAYS_MIN);
    int32_t const min_angle = b_mid ? HUG_FACE_ANGLE_DECI + 1 : -HUG_FACE_ANGLE_DECI;
    int32_t const max_angle = (b_mid || p_hug->b_oblique) ? HUG_INNER_ANGLE_DECI : HUG_FACE_ANGLE_DECI;
    if (b_face && (angle >= min_angle) && (angle <= max_angle) &&
        (geom_abs(p_face->dist_mm - expected) <= HUG_REFINE_TOL_MM))
    {
        p_hug->b_oblique = p_hug->b_oblique || b_mid;
        if (p_hug->b_oblique)
        {
            p_hug->center = (int16_t)p_face->normal_deci;
        }
        hug_after_look(p_hug, p_look, p_face, true, p_out);
    }
    else if (!b_mid && b_face && hug_flag_get(p_hug, F_TURNED) &&
             (geom_abs(angle) <= HUG_FACE_ANGLE_DECI) && (p_face->dist_mm < expected - HUG_REFINE_TOL_MM))
    {
        /* The side is real, but misses the last endpoint. Check the return
         * corridor before taking a different view from its midpoint. */
        hug_start_scan(p_hug, p_look, PU_FACET_CLEAR, geom_wrap_deci(p_hug->trav_head + HALVES * QUARTER_DECI));
    }
    else if (PU_EDGE_LOOK == p_hug->purpose)
    {
        hug_look_along_face(p_hug, PU_EDGE_RETRY);
    }
    else
    {
        hug_finish(p_hug);
        p_out->b_timeout = 1U;
    }
}

void hug_release(hug_t * p_hug)
{
    hug_flag_set(p_hug, F_REAPPROACH, hug_flag_get(p_hug, F_STALL));
    hug_start_pivot(p_hug, 0, THEN_RELEASE);
}

/* Give the wheels back. The sizes belong to the obstacle just left. */
void hug_finish(hug_t * p_hug)
{
    if (hug_flag_get(p_hug, F_EXIT))
    {
        p_hug->b_note = 0U;
        p_hug->n_rel = 0U;
    }
    p_hug->state = (uint8_t)ST_IDLE;
    p_hug->hist = 0U;
    p_hug->width_mm = 0U;
    p_hug->breadth_mm = 0U;
    p_hug->seen_mm = 0;
}

/* A wheel stall is a face across the heading at the body radius, which the
 * sonar did not see. It has no edges, so it gives no width or breadth and the
 * leg is the short one of a face with no edge. The next look sees the real
 * faces at a better angle. The face is measured from the stall pose, where
 * the car stands. */
void hug_plan_assumed(hug_t * p_hug, bool b_first)
{
    int32_t const head = p_hug->center; /* the stall heading */
    /* The first stall backed off by the standoff before this look. */
    int32_t const ahead_mm = HUG_BODY_MM + (b_first ? HUG_STANDOFF_MM : 0);
    look_face_t face = {0, 0, 0, 0};
    hug_edges_t edges = {0};

    face.normal_deci = head;
    face.x_mm = hug_pos_x_mm(p_hug) + geom_mul_q14(ahead_mm, geom_cos_q14(head));
    face.y_mm = hug_pos_y_mm(p_hug) + geom_mul_q14(ahead_mm, geom_sin_q14(head));
    face.dist_mm = geom_dot_mm(head, face.x_mm - hug_pos_x_mm(p_hug), face.y_mm - hug_pos_y_mm(p_hug));
    if (b_first)
    {
        p_hug->width_mm = 0U;
        p_hug->side = hug_pick_side(p_hug, &edges);
    }
    else
    {
        /* The way round on this side is blocked by a face the sonar does not
         * see, so the car goes round the other way. */
        p_hug->side = (int8_t)(-p_hug->side);
    }
    hug_plan_leg(p_hug, &face, &edges, b_first, NULL);
    /* The look after the leg comes as after a turn: if the sonar does not see
     * the face either, the car goes on a body length before it turns back. */
    hug_flag_set(p_hug, F_ASSUME | F_TURNED, true);
}

void hug_after_inner(hug_t * p_hug, look_t * p_look, look_face_t const * p_face,
                     bool b_found)
{
    if (b_found
        && (geom_abs(geom_wrap_deci(p_face->normal_deci - p_hug->center)) <= HUG_INNER_ANGLE_DECI))
    {
        hug_edges_t edges;

        hug_find_edges(p_look, p_face, p_hug, &edges);
        hug_plan_leg(p_hug, p_face, &edges, false, p_look);
    }
    else if (hug_flag_get(p_hug, F_STALL))
    {
        hug_plan_assumed(p_hug, false);
    }
    else
    {
        p_hug->leg_mm = (int16_t)HUG_LEG_MIN_MM;
        hug_start_pivot(p_hug, p_hug->trav_head, THEN_ADVANCE);
    }
}

static void settle_fan_pose(hug_t * p_hug)
{
    if (p_hug->b_fan_pending)
    {
        /* Idle odo_update is off and b_can_resume is clear, so the wheel anchors are free. */
        p_hug->b_fan_pending = false;
        p_hug->last_left_um = p_hug->anchor.fan_pose.left_um;
        p_hug->last_right_um = p_hug->anchor.fan_pose.right_um;
        p_hug->b_settled_pose = true;
    }
}

void hug_init(hug_t * p_hug)
{
    uint8_t * const p_bytes = (uint8_t *)p_hug;
    settle_fan_pose(p_hug);
    uint32_t const left_um = p_hug->last_left_um;
    uint32_t const right_um = p_hug->last_right_um;
    bool const b_settled_pose = p_hug->b_settled_pose;

    for (size_t i = 0U; i < sizeof(*p_hug); i++)
    {
        p_bytes[i] = 0U;
    }
    p_hug->last_seq = (uint8_t)ODD_SEQ;
    p_hug->side = 1;
    p_hug->last_left_um = left_um;
    p_hug->last_right_um = right_um;
    p_hug->b_settled_pose = b_settled_pose;
}

void hug_forget_approach(hug_t * p_hug)
{
    p_hug->b_bend_check = false;
    p_hug->b_approach_tape = false;
    p_hug->b_can_resume = false;
}

bool hug_active(hug_t const * p_hug)
{
    return ST_IDLE != p_hug->state;
}

bool hug_reapproaching(hug_t const * p_hug)
{
    return (ST_IDLE == p_hug->state) && (0U != p_hug->b_note) &&
           hug_flag_get(p_hug, F_REAPPROACH);
}

int16_t hug_heading_deci(hug_t const * p_hug)
{
    /* Cast is safe: the heading is wrapped to -1800..1800. */
    return (ST_IDLE == p_hug->state) ? (int16_t)0 : (int16_t)hug_pose_heading_deci(p_hug);
}

void hug_proceed(hug_t * p_hug, look_t * p_look)
{
    switch (p_hug->then)
    {
    case THEN_BACK:
        if (ST_PIVOT == p_hug->state)
        {
            hug_start_drive(p_hug, p_hug->goal_len_mm, p_hug->goal_head, THEN_BACK);
        }
        else
        {
            hug_start_scan(p_hug, p_look, p_hug->purpose, p_hug->center);
        }
        break;
    case THEN_SCAN:
        hug_start_scan(p_hug, p_look, p_hug->purpose, p_hug->center);
        break;
    case THEN_APPROACH:
        hug_start_drive(p_hug, p_hug->app_mm, p_hug->goal_head, THEN_ALIGN);
        break;
    case THEN_ALIGN:
        hug_start_pivot(p_hug, p_hug->u_head, THEN_LEG);
        break;
    case THEN_LEG:
        p_hug->trav_head = p_hug->u_head;
        hug_start_drive(p_hug, p_hug->leg_mm, p_hug->u_head, THEN_LOOK);
        break;
    case THEN_LOOK:
        if (p_hug->b_close_front && hug_flag_get(p_hug, F_EDGE_LEG))
        {
            hug_start_scan(p_hug, p_look, PU_CORNER,
                       geom_wrap_deci(p_hug->look_head + p_hug->side * HUG_FLANK_DECI));
        }
        else if (hug_corner_scan(p_hug) && hug_flag_get(p_hug, F_TURNED))
        {
            hug_look_along_face(p_hug, (hug_facet_clearance(p_hug) || p_hug->b_oblique) ? PU_MID_LOOK : PU_EDGE_LOOK);
        }
        else if (hug_flag_get(p_hug, F_EDGE_LEG) && !hug_flag_get(p_hug, F_TURNED) &&
                 (hug_from_edge_mm(p_hug, hug_along_head(p_hug)) >= HUG_AXLE_OFF_MM))
        {
            hug_start_scan(p_hug, p_look, PU_CORNER, p_hug->look_head);
        }
        else if (hug_flag_get(p_hug, F_MEASURED_FACE) && !hug_flag_get(p_hug, F_PAST | F_TURNED)
                 && hug_side_check_supported(p_hug, p_look))
        {
            hug_start_scan(p_hug, p_look, PU_CHECK, p_hug->look_head);
        }
        else
        {
            hug_look_along_face(p_hug, PU_LOOK);
        }
        hug_flag_set(p_hug, F_EDGE_LEG, false);
        break;
    case THEN_FLANK_LOOK:
        p_hug->center = p_hug->look_head;
        hug_set_next_face(p_hug);
        hug_look_along_face(p_hug, PU_EDGE_LOOK);
        break;
    case THEN_TURN:
        hug_start_drive(p_hug, p_hug->face_p_mm + HUG_TURN_FWD_MM, p_hug->goal_head, THEN_LOOK);
        break;
    case THEN_ADVANCE:
        hug_start_drive(p_hug, p_hug->leg_mm, p_hug->goal_head, THEN_LOOK);
        break;
    case THEN_REAR_ALIGN:
        hug_start_pivot(p_hug, 0, THEN_REAR_READY);
        break;
    case THEN_EXIT_TURN:
        /* Away from the obstacle, which is on the other side than the way round. */
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
        hug_start_pivot(p_hug, geom_wrap_deci(p_hug->goal_head + (p_hug->side * LINE_SEARCH_SWEEP_DECI)),
                    THEN_RELEASE);
        break;
    default:
        hug_finish(p_hug);
        break;
    }
}

static void begin(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in)
{
    bool const b_recheck = hug_can_recheck(p_hug, p_look, p_in);

    p_hug->b_bend_check = false;
    p_hug->recheck_hits = 0U;
    p_hug->last_motion_ms = p_in->now_ms;
    p_hug->x_um = 0;
    p_hug->b_require_rear = !p_hug->b_approach_tape;
    p_hug->b_can_resume = false;
    p_hug->b_remaining = false;
    p_hug->b_oblique = false;
    p_hug->b_close_front = false;
    p_hug->b_survey_displaced = false;
    p_hug->b_probe_front = false;
    p_hug->side_end_x_mm = 0;
    p_hug->side_end_y_mm = 0;
    p_hug->y_um = 0;
    p_hug->diff_um = 0;
    p_hug->last_left_um = (uint32_t)p_in->left_um;
    p_hug->last_right_um = (uint32_t)p_in->right_um;
    p_hug->width_mm = 0U;
    p_hug->breadth_mm = 0U;
    p_hug->seen_mm = 0;
    p_hug->flags = 0U;
    hug_flag_set(p_hug, F_STALL, p_in->b_stall);
    hug_flag_set(p_hug, F_REPEAT,
             p_in->b_stall &&
                 ((0U != p_hug->b_note) || (p_hug->n_rel >= HUG_RELEASES_MAX)));
    p_hug->b_note = p_in->b_stall ? 1U : 0U;
    p_hug->n_rel = p_in->b_stall ? p_hug->n_rel : 0U;
    p_hug->side = 1;
    p_hug->trav_head = 0;
    p_hug->look_head = 0;
    p_hug->lost = 0U;
    p_hug->purpose = (uint8_t)PU_CONTACT;
    p_hug->center = 0;
    hug_flag_set(p_hug, F_RECHECK, b_recheck);
    if (!b_recheck)
    {
        p_hug->contact_scan_ms = p_in->now_ms;
        p_hug->origin.survey.left_um = (uint32_t)p_in->left_um;
        p_hug->origin.survey.right_um = (uint32_t)p_in->right_um;
        look_clear(p_look);
    }
    if (hug_flag_get(p_hug, F_REPEAT))
    {
        /* The look at the stall pose would not change the back-off. */
        hug_start_back(p_hug, 0);
    }
    else
    {
        hug_start_pivot(p_hug, 0, THEN_SCAN);
    }
}

/* A sweep or cached sighting belongs to the motion owner that observed it. */
static void interrupt_observations(hug_t * p_hug, look_t * p_look, bool b_contact_backoff)
{
    if (p_hug->b_probe_front && (ST_IDLE != p_hug->state))
    {
        p_hug->b_survey_displaced = true;
    }
    if (!b_contact_backoff)
    {
        if (hug_edge_sweep(p_hug) && ((ST_SCAN == p_hug->state) ||
            ((ST_PIVOT == p_hug->state) && (THEN_SCAN == p_hug->then))))
        {
            /* A wheel takeover can displace the chassis even when its
             * encoder position is unchanged. Discard both partial views. */
            look_clear(p_look);
            p_hug->b_survey_displaced = true;
        }
        hug_forget_approach(p_hug);
    }
    p_hug->b_can_resume = false;
    p_hug->side_end_x_mm = 0;
    p_hug->side_end_y_mm = 0;
    if ((ST_DRIVE == p_hug->state) && (THEN_FLANK_LOOK == p_hug->then))
    {
        p_hug->b_survey_displaced = true;
    }
    else if ((ST_IDLE != p_hug->state) && (p_hug->then >= THEN_REAR_LATERAL))
    {
        hug_look_along_face(p_hug, PU_LOOK);
    }
    p_hug->recheck_hits = 0U;
    hug_flag_set(p_hug, F_EXIT_TAPE, false);
    if ((ST_PIVOT == p_hug->state) && hug_flag_get(p_hug, F_EXIT))
    {
        p_hug->tape.crossing.reader = (uint8_t)EXIT_WAIT_CLEAR;
        p_hug->sub = (uint8_t)SUB_START;
    }
    if (((ST_SCAN == p_hug->state) && ((PU_CONTACT == p_hug->purpose) || (PU_CENTER == p_hug->purpose))) ||
        ((ST_IDLE != p_hug->state) && hug_flag_get(p_hug, F_RECHECK)))
    {
        hug_restart_contact(p_hug, p_look);
    }
    if (ST_IDLE != p_hug->state)
    {
        bool const b_probe_leg = ((PU_CHECK == p_hug->purpose) && (ST_PIVOT == p_hug->state) &&
                                  (THEN_LEG == p_hug->then)) ||
            (hug_probe_scan(p_hug) && (ST_DRIVE == p_hug->state) && (THEN_LOOK == p_hug->then));

        hug_flag_set(p_hug, F_EDGE_LEG | F_MEASURED_FACE, false);
        if (((ST_SCAN == p_hug->state) && hug_probe_scan(p_hug)) || b_probe_leg)
        {
            hug_look_along_face(p_hug, PU_LOOK);
        }
        else if (SUB_MOVE == p_hug->sub)
        {
            /* The clock of the move starts again when the wheels come back. */
            p_hug->sub = (uint8_t)SUB_START;
        }
    }
}

static void observe_approach_tape(hug_t * p_hug, hug_in_t const * p_in)
{
    if (!p_in->b_other && (p_in->b_line_left || p_in->b_line_right))
    {
        p_hug->b_approach_tape = true;
        p_hug->tape.idle_tape_ms = p_in->now_ms;
    }
    else if (p_hug->b_approach_tape && (p_in->now_ms - p_hug->tape.idle_tape_ms > HUG_NOTE_MS))
    {
        p_hug->b_approach_tape = false;
    }
}

bool hug_fan_renewable(hug_t const * p_hug, hug_in_t const * p_in)
{
    if (!p_hug->b_settled_pose)
    {
        return true;
    }
    int32_t const dl = (int32_t)((uint32_t)p_in->left_um - p_hug->last_left_um);
    int32_t const dr = (int32_t)((uint32_t)p_in->right_um - p_hug->last_right_um);
    return bypass_pose_moved(dl, dr) || (hug_triggered(p_hug) && (HIST_BITS == p_hug->quiet_pings));
}

void hug_begin_bypass(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, uint16_t range_mm)
{
    begin(p_hug, p_look, p_in);
    look_clear(p_look);
    bypass_begin(p_hug, p_in, range_mm);
}

static void observe_idle(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, bool b_forward)
{
    settle_fan_pose(p_hug);
    bool const b_rechecking = hug_recheck_on_bend(p_hug, p_in, b_forward);
    observe_approach_tape(p_hug, p_in);
    /* A stall with the follower driving is a contact: the sonar did not see
     * the face, so look from the backed-off pose. */
    if (!p_in->b_other && ((!b_rechecking && hug_triggered(p_hug)) || p_in->b_stall || p_in->b_relook))
    {
        uint16_t range = (!p_in->b_ping_reply && (0 == p_in->ping_servo_deci) &&
            look_echo_is_point(p_in->ping_echo) && (p_in->ping_mm <= HUG_STANDOFF_MM))
            ? p_in->ping_mm : 0U;
        if (!p_in->b_ping_reply && (p_in->ping_servo_deci == 0) && (p_in->ping_echo == LOOK_ECHO_TOO_CLOSE))
        {
            range = OBSTACLE_PING_TOO_CLOSE_MM;
        }
        if ((range == 0U) && !p_in->b_stall && !p_in->b_relook)
        {
            return;
        }
        if (p_in->b_relook)
        {
            /* A post-recovery contact may be on a side ray while the
             * latest centre ping is open. Save it before begin resets the look. */
            for (uint8_t i = 0U; i < p_look->count; i++)
            {
                look_ray_t const * const p_ray = &p_look->rays[i];
                if (look_echo_is_point(look_ray_echo(p_ray)) && p_ray->range_mm <= HUG_STANDOFF_MM &&
                    (range == 0U || p_ray->range_mm < range))
                {
                    range = p_ray->range_mm;
                }
            }
        }
        if (!hug_fan_renewable(p_hug, p_in))
        {
            return;
        }
        p_hug->quiet_pings = 0U;
        if (range == 0U)
        {
            begin(p_hug, p_look, p_in);
        }
        else
        {
            hug_begin_bypass(p_hug, p_look, p_in, range);
        }
    }
}

bool hug_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bool const b_was_active = (ST_IDLE != p_hug->state);
    bool b_guard = false;
    bool b_forward = false;
    bool b_takes = false;

    hug_zero_out(p_out);
    if (ST_IDLE != p_hug->state)
    {
        hug_odo_update(p_hug, p_in);
        if ((ST_SCAN == p_hug->state) && (p_hug->lost >= HUG_LOST_REPLIES))
        {
            /* The sonar-fault stop lasts until replies recover. */
            p_hug->last_motion_ms = p_in->now_ms;
        }
    }
    else if ((0U != p_hug->b_note) || (0U != p_hug->n_rel))
    {
        hug_note_update(p_hug, p_in);
    }
    else
    {
        /* Nothing to track while idle. */
    }
    if (p_hug->bypass.phase != BYPASS_IDLE)
    {
        bypass_step(p_hug, p_look, p_in, p_out);
        return !p_in->b_other && (ST_IDLE != p_hug->state);
    }
    if (p_in->b_other)
    {
        interrupt_observations(p_hug, p_look, p_in->b_contact_backoff);
    }
    else if ((ST_IDLE != p_hug->state) && hug_flag_get(p_hug, F_RECHECK) &&
        (((p_in->now_ms - p_hug->contact_scan_ms) > HUG_NOTE_MS) ||
         !hug_near_survey_pose(p_hug, p_in)))
    {
        hug_restart_contact(p_hug, p_look);
    }
    b_forward = hug_forward_ping(p_hug, p_in, &b_guard);
    if (ST_IDLE == p_hug->state)
    {
        observe_idle(p_hug, p_look, p_in, b_forward);
    }
    if (p_hug->bypass.phase != BYPASS_IDLE)
    {
        bypass_step(p_hug, p_look, p_in, p_out);
        return true;
    }
    if (!p_in->b_other && (ST_IDLE != p_hug->state))
    {
        if (!hug_flag_get(p_hug, F_EXIT) && (THEN_FLANK_LOOK != p_hug->then) &&
            ((ST_DRIVE != p_hug->state) || (SUB_SETTLE != p_hug->sub)))
        {
            hug_observe_exit_tape(p_hug, p_in);
        }
        if (SUB_START == p_hug->sub)
        {
            p_hug->t_ms = p_in->now_ms + hug_move_limit_ms(p_hug);
            p_hug->sub = (uint8_t)SUB_MOVE;
        }
        /* The difference is right across the wrap of the clock. */
        /* An incomplete contour cannot keep circulating if a surface is
         * lost. This odometric bound is a stop, never an inferred exit. */
        if (p_hug->b_remaining && (geom_abs(p_hug->diff_um) / UM_PER_MM >
            ((REMAINING_LIMIT_DECI * CHASSIS_WHEEL_TRACK_MM) / DECI_PER_RAD)))
        {
            hug_finish(p_hug);
            p_out->b_no_exit = true;
        }
        else if (p_hug->b_survey_displaced ||
                 ((p_in->now_ms - p_hug->last_motion_ms) >= HUG_STATIONARY_LIMIT_MS) ||
                 ((SUB_MOVE == p_hug->sub) && ((int32_t)(p_in->now_ms - p_hug->t_ms) > 0)))
        {
            /* A displaced survey or an expired movement ends under a hold. */
            hug_finish(p_hug);
            p_out->b_timeout = 1U;
        }
        else if (ST_SCAN == p_hug->state)
        {
            hug_scan_tick(p_hug, p_look, p_in, p_out);
        }
        else if (ST_PIVOT == p_hug->state)
        {
            hug_pivot_tick(p_hug, p_look, p_in, b_forward, b_guard, p_out);
        }
        else
        {
            hug_drive_tick(p_hug, p_look, p_in, b_guard, p_out);
        }
        b_takes = (ST_IDLE != p_hug->state);
    }
    if (b_was_active && (ST_IDLE == p_hug->state))
    {
        /* The note and the releases run from here. */
        p_hug->t_ms = p_in->now_ms;
    }
    return b_takes;
}
