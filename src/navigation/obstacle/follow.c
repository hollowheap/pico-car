/** @file follow.c
 *
 * @brief Bug2 face follow for the obstacle bypass.
 *        Pure C99, integer only, no libm and no kernel calls.
 */

#include <stdbool.h>
#include <stdint.h>

#include "follow.h"
#include "geometry.h"
#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "planner.h"

#define UM_PER_MM  (1000)
#define PERCENT_ALL (100)
#define DECI_RIGHT (900)

/* Phases of follow_t.phase. */
#define PHASE_LEG         (0U) /* parallel to the face                     */
#define PHASE_IN          (1U) /* arc toward the face and look             */
#define PHASE_OUT         (2U) /* arc back to parallel                     */
#define PHASE_CORNER      (3U) /* face lost: drive past the corner         */
#define PHASE_WRAP        (4U) /* turn on the spot toward the next face    */
#define PHASE_AWAY        (5U) /* face ahead: turn on the spot away        */
#define PHASE_LEAVE_CREEP (6U) /* on the tape: bring the axle over it      */
#define PHASE_LEAVE_TURN  (7U) /* turn to the original heading             */
#define PHASE_DONE        (8U) /* stand still: on the line, or gave up    */
#define PHASE_BACKUP      (9U) /* face lost: reverse before a deeper look */
#define PHASE_LOOK        (10U) /* turn further into the face and look    */

/* Widths of the follow_t bit-fields. */
#define PHASE_MASK  (0x0FU)
#define MISSES_MASK (0x03U)
#define TRIES_MASK  (0x03U)
#define REASON_MASK (0x07U)

/* +1 when the car passes on the left: "away" from the face is a left turn. */
static int32_t away_sign (follow_t const * p_follow)
{
    return (0U != p_follow->b_left) ? 1 : -1;
}

static int16_t clamp_i16 (int32_t value)
{
    int32_t out = value;

    if (out > INT16_MAX)
    {
        out = INT16_MAX;
    }
    else if (out < INT16_MIN)
    {
        out = INT16_MIN;
    }
    else
    {
        /* In range. */
    }
    return ((int16_t)out);
}

/* face_head_deci stays wrapped, so any number of corners fits int16_t. */
static void set_face_head (follow_t * p_follow, int32_t deci)
{
    p_follow->face_head_deci = (int16_t)geom_wrap_deci(deci);
}

/* The face direction plus offset, as a target next to the present heading
 * (the pose heading is not wrapped). */
static int32_t face_target (follow_t const * p_follow, follow_in_t const * p_in, int32_t offset_deci)
{
    return (p_in->head_deci +
            geom_wrap_deci(((int32_t)p_follow->face_head_deci + offset_deci) - p_in->head_deci));
}

static void set_phase (follow_t * p_follow, uint8_t phase, follow_in_t const * p_in)
{
    p_follow->phase     = (uint8_t)(phase & PHASE_MASK);
    p_follow->mark_x_mm = clamp_i16(p_in->x_mm);
    p_follow->mark_y_mm = clamp_i16(p_in->y_mm);
    p_follow->misses    = 0U;
    p_follow->b_settled = 0U;
}

/* Distance driven since the phase started, mm. */
static int32_t driven_mm (follow_t const * p_follow, follow_in_t const * p_in)
{
    int32_t const dx = p_in->x_mm - (int32_t)p_follow->mark_x_mm;
    int32_t const dy = p_in->y_mm - (int32_t)p_follow->mark_y_mm;
    int64_t const sq = ((int64_t)dx * dx) + ((int64_t)dy * dy);

    return ((int32_t)geom_isqrt((sq > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)sq));
}

/* Signed distance of the axle from the face line, positive on the car's
 * side of the face. */
static int32_t face_gap_mm (follow_t const * p_follow, follow_in_t const * p_in)
{
    int32_t const nx = geom_cos_q14((int32_t)p_follow->face_head_deci);  /* left normal x */
    int32_t const ny = -geom_sin_q14((int32_t)p_follow->face_head_deci); /* left normal y */
    int32_t const dx = p_in->x_mm - (int32_t)p_follow->face_x_mm;
    int32_t const dy = p_in->y_mm - (int32_t)p_follow->face_y_mm;

    return (away_sign(p_follow) * ((geom_mul_q14(dx, nx) + geom_mul_q14(dy, ny))));
}

/* Axle travel past the face point along the face direction, mm. */
static int32_t past_point_mm (follow_t const * p_follow, follow_in_t const * p_in)
{
    int32_t const dx = p_in->x_mm - (int32_t)p_follow->face_x_mm;
    int32_t const dy = p_in->y_mm - (int32_t)p_follow->face_y_mm;

    return (geom_mul_q14(dx, geom_sin_q14((int32_t)p_follow->face_head_deci)) +
            geom_mul_q14(dy, geom_cos_q14((int32_t)p_follow->face_head_deci)));
}

/* The new ping is a range closer than limit_mm. TOO_CLOSE always is. */
static bool echo_within (follow_in_t const * p_in, uint16_t limit_mm)
{
    return (p_in->b_ping &&
            ((OBSTACLE_CLASS_TOO_CLOSE == p_in->cls) ||
             ((OBSTACLE_CLASS_VALID == p_in->cls) && (p_in->dist_mm <= limit_mm))));
}

/* The point the echo came from, on the beam axis. */
static void echo_point (follow_in_t const * p_in, int32_t servo_deci, int32_t * p_x, int32_t * p_y)
{
    int32_t const beam   = p_in->head_deci + servo_deci;
    int32_t const dist   = (OBSTACLE_CLASS_TOO_CLOSE == p_in->cls) ? 0 : (int32_t)p_in->dist_mm;
    int32_t const sens_x = p_in->x_mm + geom_mul_q14((int32_t)OBSTACLE_SONAR_FWD_MM, geom_sin_q14(p_in->head_deci));
    int32_t const sens_y = p_in->y_mm + geom_mul_q14((int32_t)OBSTACLE_SONAR_FWD_MM, geom_cos_q14(p_in->head_deci));

    *p_x = sens_x + geom_mul_q14(dist, geom_sin_q14(beam));
    *p_y = sens_y + geom_mul_q14(dist, geom_cos_q14(beam));
}

/* Distance of a point from the face line, positive on the car's side. */
static int32_t toward_car_mm (follow_t const * p_follow, int32_t px, int32_t py)
{
    int32_t const nx = geom_cos_q14((int32_t)p_follow->face_head_deci);
    int32_t const ny = -geom_sin_q14((int32_t)p_follow->face_head_deci);

    return (away_sign(p_follow) * (geom_mul_q14(px - (int32_t)p_follow->face_x_mm, nx) +
                                   geom_mul_q14(py - (int32_t)p_follow->face_y_mm, ny)));
}

/* How far the face would turn to reach this point from the last one;
 * 0 if the two points are too close to tell. Positive turns left. Points
 * that creep along a face across the path each sit close to the last one,
 * but they bend the face by much more than a slant can. */
static int32_t bend_deci (follow_t const * p_follow, int32_t px, int32_t py)
{
    int32_t const dx   = px - (int32_t)p_follow->face_x_mm;
    int32_t const dy   = py - (int32_t)p_follow->face_y_mm;
    int32_t       bend = 0;

    if ((((int64_t)dx * dx) + ((int64_t)dy * dy)) >=
        ((int64_t)OBSTACLE_FOLLOW_SLANT_MIN_MM * OBSTACLE_FOLLOW_SLANT_MIN_MM))
    {
        bend = geom_wrap_deci(geom_atan_deci(dx, dy) - (int32_t)p_follow->face_head_deci);
    }
    return (bend);
}

/* Store the point the echo came from. A point far enough from the last one
 * of the same face gives the face direction (a slanted face). */
static void note_face_point (follow_t * p_follow, follow_in_t const * p_in, int32_t servo_deci)
{
    int32_t px = 0;
    int32_t py = 0;

    echo_point(p_in, servo_deci, &px, &py);

    if ((0U == p_follow->b_point) || (0U != p_follow->b_seed))
    {
        /* The first echo of a face, or the first after the scan estimate:
         * it becomes the anchor, with no direction change. */
        p_follow->b_seed    = 0U;
        p_follow->face_x_mm = clamp_i16(px);
        p_follow->face_y_mm = clamp_i16(py);
        p_follow->b_point   = 1U;
    }
    else
    {
        int32_t const dx = px - (int32_t)p_follow->face_x_mm;
        int32_t const dy = py - (int32_t)p_follow->face_y_mm;

        /* A point close to the stored one is kept out: the stored point is
         * the anchor until a new one is far enough to give a direction. */
        if ((((int64_t)dx * dx) + ((int64_t)dy * dy)) >=
            ((int64_t)OBSTACLE_FOLLOW_SLANT_MIN_MM * OBSTACLE_FOLLOW_SLANT_MIN_MM))
        {
            int32_t const turn = bend_deci(p_follow, px, py);

            if (geom_abs(turn) <= OBSTACLE_FOLLOW_SLANT_MAX_DECI)
            {
                /* Blend: one noisy pair of points must not swing the face. */
                set_face_head(p_follow, (int32_t)p_follow->face_head_deci +
                                            (turn / OBSTACLE_FOLLOW_SLANT_GAIN_DIV));
            }
            p_follow->face_x_mm = clamp_i16(px);
            p_follow->face_y_mm = clamp_i16(py);
        }
    }
}

/* The face has ended. The reference for the corner drive is where this
 * ping's beam meets the face line: the corner lies before it, so driving
 * OBSTACLE_FOLLOW_CORNER_PAST_MM past it clears the corner. With no face
 * point, or a beam that does not meet the line ahead, it is the car. */
static void set_corner_reference (follow_t * p_follow, follow_in_t const * p_in, int32_t servo_deci)
{
    int32_t rx = p_in->x_mm;
    int32_t ry = p_in->y_mm;

    if (0U != p_follow->b_point)
    {
        int32_t const beam = p_in->head_deci + servo_deci;
        int32_t const sx   = p_in->x_mm + geom_mul_q14((int32_t)OBSTACLE_SONAR_FWD_MM, geom_sin_q14(p_in->head_deci));
        int32_t const sy   = p_in->y_mm + geom_mul_q14((int32_t)OBSTACLE_SONAR_FWD_MM, geom_cos_q14(p_in->head_deci));
        int32_t const bx   = geom_sin_q14(beam);
        int32_t const by   = geom_cos_q14(beam);
        int32_t const fx   = geom_sin_q14((int32_t)p_follow->face_head_deci);
        int32_t const fy   = geom_cos_q14((int32_t)p_follow->face_head_deci);
        int64_t const den  = ((int64_t)bx * fy) - ((int64_t)by * fx);
        int64_t const num  = ((int64_t)((int32_t)p_follow->face_x_mm - sx) * fy) -
                             ((int64_t)((int32_t)p_follow->face_y_mm - sy) * fx);

        if (0 != den)
        {
            int64_t const t_mm = (num * GEOM_Q14_ONE) / den;

            if ((t_mm > 0) && (t_mm <= (int64_t)OBSTACLE_FOLLOW_SEEN_MAX_MM))
            {
                rx = sx + geom_mul_q14((int32_t)t_mm, bx);
                ry = sy + geom_mul_q14((int32_t)t_mm, by);
            }
        }
    }
    p_follow->face_x_mm = clamp_i16(rx);
    p_follow->face_y_mm = clamp_i16(ry);
}

/* Arc: both wheels forward, turning toward sign (+1 left). */
static void arc (int32_t sign, int32_t speed_mm_s, follow_out_t * p_out)
{
    p_out->left_um_s  = (speed_mm_s - (sign * OBSTACLE_FOLLOW_ARC_MM_S)) * UM_PER_MM;
    p_out->right_um_s = (speed_mm_s + (sign * OBSTACLE_FOLLOW_ARC_MM_S)) * UM_PER_MM;
    p_out->steer      = (sign > 0) ? OBSTACLE_STEER_LEFT : OBSTACLE_STEER_RIGHT;
}

/* Straight back at the look speed; the navigation pauses before a wheel
 * reverses. */
static void reverse (follow_out_t * p_out)
{
    p_out->left_um_s  = -(int32_t)OBSTACLE_FOLLOW_LOOK_MM_S * UM_PER_MM;
    p_out->right_um_s = -(int32_t)OBSTACLE_FOLLOW_LOOK_MM_S * UM_PER_MM;
    p_out->steer      = OBSTACLE_STEER_STRAIGHT;
}

/* Spin on the spot to target; true when there. */
static bool spin (follow_in_t const * p_in, int32_t target_deci, follow_out_t * p_out)
{
    bool b_done = false;

    obstacle_plan_wheel_pivot(p_in->head_deci, target_deci, &p_out->left_um_s,
                              &p_out->right_um_s, &p_out->steer, &b_done);
    p_out->b_turning = !b_done;
    return (b_done);
}

static void straight (follow_in_t const * p_in, int32_t target_deci, int32_t speed_mm_s,
                      follow_out_t * p_out)
{
    obstacle_plan_wheel_straight(p_in->head_deci, target_deci, speed_mm_s, &p_out->left_um_s,
                                 &p_out->right_um_s, &p_out->steer);
}

/* What the new echo is (classify_echo). */
#define ECHO_NONE  (0U) /* no echo inside the seen range                 */
#define ECHO_SAME  (1U) /* the face being followed: a new point          */
#define ECHO_AHEAD (2U) /* a face across the path: an inner corner       */

/* Sort the new echo and act on it. A point more than
 * OBSTACLE_FOLLOW_SAME_FACE_MM off the face line, on the car's side, is a
 * face across the path: it becomes the face to follow and the car turns
 * away. Before any point, an echo inside OBSTACLE_FOLLOW_INNER_MM outside a
 * glance is a face ahead too. */
static uint8_t classify_echo (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out,
                              bool b_glance)
{
    uint8_t kind = ECHO_NONE;

    if (echo_within(p_in, (uint16_t)OBSTACLE_FOLLOW_SEEN_MAX_MM))
    {
        if (0U != p_follow->b_point)
        {
            int32_t px = 0;
            int32_t py = 0;

            echo_point(p_in, (int32_t)p_out->servo_deci, &px, &py);
            kind = ((toward_car_mm(p_follow, px, py) > OBSTACLE_FOLLOW_SAME_FACE_MM) ||
                    ((0U == p_follow->b_seed) &&
                     ((away_sign(p_follow) * bend_deci(p_follow, px, py)) > OBSTACLE_FOLLOW_SLANT_MAX_DECI)))
                       ? ECHO_AHEAD
                       : ECHO_SAME;
        }
        else
        {
            kind = ((!b_glance) && echo_within(p_in, (uint16_t)OBSTACLE_FOLLOW_INNER_MM)) ? ECHO_AHEAD
                                                                                         : ECHO_SAME;
        }
    }

    if (ECHO_SAME == kind)
    {
        note_face_point(p_follow, p_in, (int32_t)p_out->servo_deci);
    }
    else if (ECHO_AHEAD == kind)
    {
        /* The point is on the new face; the old face must not slant it. */
        p_follow->b_point = 0U;
        note_face_point(p_follow, p_in, (int32_t)p_out->servo_deci);
        set_face_head(p_follow, (int32_t)p_follow->face_head_deci + (away_sign(p_follow) * DECI_RIGHT));
        p_follow->tries = 0U;
        set_phase(p_follow, PHASE_AWAY, p_in);
        (void)spin(p_in, face_target(p_follow, p_in, 0), p_out);
    }
    else
    {
        /* No echo. */
    }
    return (kind);
}

static void step_leg (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    int32_t const away = away_sign(p_follow);
    int32_t       trim = 0;

    if (ECHO_AHEAD == classify_echo(p_follow, p_in, p_out, false))
    {
        /* Turning away. */
    }
    else if (driven_mm(p_follow, p_in) > OBSTACLE_FOLLOW_LEG_MM)
    {
        set_phase(p_follow, PHASE_IN, p_in);
        arc(-away, (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
    }
    else
    {
        if (0U != p_follow->b_point)
        {
            /* Too far from the face: lean toward it; too close: away. */
            trim = (face_gap_mm(p_follow, p_in) - OBSTACLE_FOLLOW_HOLD_MM) * OBSTACLE_FOLLOW_TRIM_PER_MM;
            if (trim > OBSTACLE_FOLLOW_TRIM_MAX_DECI)
            {
                trim = OBSTACLE_FOLLOW_TRIM_MAX_DECI;
            }
            else if (trim < -OBSTACLE_FOLLOW_TRIM_MAX_DECI)
            {
                trim = -OBSTACLE_FOLLOW_TRIM_MAX_DECI;
            }
            else
            {
                /* Within the cap. */
            }
        }
        straight(p_in, face_target(p_follow, p_in, -(away * trim)), (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                 p_out);
    }
}

static void step_in (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    int32_t const away   = away_sign(p_follow);
    int32_t const target = face_target(p_follow, p_in, -(away * OBSTACLE_FOLLOW_TOWARD_DECI));
    /* Reached once the heading is at or past the target: the arc must not
     * restart after the wheel lag carries the car a little beyond it. */
    bool const    b_at   = (away * (p_in->head_deci - target)) <= OBSTACLE_PIVOT_TOL_DECI;

    uint8_t const kind   = classify_echo(p_follow, p_in, p_out, true);

    if (ECHO_AHEAD == kind)
    {
        /* Turning away. */
    }
    else if (ECHO_SAME == kind)
    {
        set_phase(p_follow, PHASE_OUT, p_in);
        arc(away, (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
    }
    else if (b_at && p_in->b_ping && (0U != p_follow->b_settled) &&
             (p_follow->misses >= (OBSTACLE_FOLLOW_MISS_MAX - 1U)) && (0U == p_follow->b_point))
    {
        /* This face has not been seen yet (it starts further on): no
         * corner, arc back and look again after the next leg. */
        set_phase(p_follow, PHASE_OUT, p_in);
        arc(away, (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
    }
    else if (b_at && p_in->b_ping && (0U != p_follow->b_settled) &&
             (p_follow->misses >= (OBSTACLE_FOLLOW_MISS_MAX - 1U)))
    {
        /* A face that was seen does not answer: it has ended, or the echo
         * is lost at a steep angle. Back up and look deeper to tell. */
        set_phase(p_follow, PHASE_BACKUP, p_in);
        reverse(p_out);
    }
    else if (b_at)
    {
        /* A ping that arrives on the first step at the glance heading was
         * taken before it was reached (up to one ping period earlier), so
         * only later pings count as misses. */
        if (p_in->b_ping && (0U != p_follow->b_settled))
        {
            p_follow->misses = (uint8_t)((p_follow->misses + 1U) & MISSES_MASK);
        }
        p_follow->b_settled = 1U;
        straight(p_in, target, (int32_t)OBSTACLE_FOLLOW_LOOK_MM_S, p_out);
    }
    else
    {
        arc(-away, (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
    }
}

static void step_out (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    int32_t const target = face_target(p_follow, p_in, 0);

    /* No face-ahead check here: the pings still see the face just found. */
    if ((away_sign(p_follow) * (p_in->head_deci - target)) >= -OBSTACLE_PIVOT_TOL_DECI)
    {
        /* Parallel again, or past it: the leg trims the rest. */
        set_phase(p_follow, PHASE_LEG, p_in);
        step_leg(p_follow, p_in, p_out);
    }
    else
    {
        arc(away_sign(p_follow), (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
    }
}

static void step_corner (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    if (ECHO_AHEAD == classify_echo(p_follow, p_in, p_out, false))
    {
        /* Turning away. */
    }
    else if (past_point_mm(p_follow, p_in) > OBSTACLE_FOLLOW_CORNER_PAST_MM)
    {
        /* The next face has no point yet: no leg trim, and no corner, until
         * a glance sees it. */
        set_face_head(p_follow, (int32_t)p_follow->face_head_deci - (away_sign(p_follow) * DECI_RIGHT));
        p_follow->b_point = 0U;
        p_follow->tries   = 0U;
        set_phase(p_follow, PHASE_WRAP, p_in);
        (void)spin(p_in, face_target(p_follow, p_in, 0), p_out);
    }
    else
    {
        straight(p_in, face_target(p_follow, p_in, 0), (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
    }
}

/* BACKUP: reverse a little, so the deeper look has room to turn. */
static void step_backup (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    if (driven_mm(p_follow, p_in) > OBSTACLE_FOLLOW_BACKUP_MM)
    {
        set_phase(p_follow, PHASE_LOOK, p_in);
        (void)spin(p_in, face_target(p_follow, p_in, -(away_sign(p_follow) *
                                                      (OBSTACLE_FOLLOW_TOWARD_DECI + OBSTACLE_FOLLOW_DEEPER_DECI))),
                   p_out);
    }
    else
    {
        reverse(p_out);
    }
}

static void give_up (follow_t * p_follow, follow_in_t const * p_in, uint8_t reason)
{
    p_follow->reason = (uint8_t)(reason & REASON_MASK);
    set_phase(p_follow, PHASE_DONE, p_in);
}

/* LOOK: on the spot, 15 degrees off the face normal. An echo means the face
 * goes on (the echo was lost at the glance angle); no echo means a corner. */
static void step_look (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    int32_t const target = face_target(p_follow, p_in, -(away_sign(p_follow) *
                                                         (OBSTACLE_FOLLOW_TOWARD_DECI + OBSTACLE_FOLLOW_DEEPER_DECI)));

    if (!spin(p_in, target, p_out))
    {
        /* Still turning: the pings are guard-only. */
    }
    else if (0U == p_follow->b_settled)
    {
        /* The first ping here was taken while turning. */
        p_follow->b_settled = 1U;
    }
    else
    {
        uint8_t const kind = classify_echo(p_follow, p_in, p_out, true);

        if (ECHO_SAME == kind)
        {
            p_follow->tries = (uint8_t)((p_follow->tries + 1U) & TRIES_MASK);
            if (p_follow->tries >= OBSTACLE_FOLLOW_TRIES_MAX)
            {
                give_up(p_follow, p_in, FOLLOW_REASON_ECHO_LOST);
            }
            else
            {
                set_phase(p_follow, PHASE_OUT, p_in);
                arc(away_sign(p_follow), (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
            }
        }
        else if ((ECHO_NONE == kind) && p_in->b_ping)
        {
            p_follow->misses = (uint8_t)((p_follow->misses + 1U) & MISSES_MASK);
            if (p_follow->misses >= OBSTACLE_FOLLOW_MISS_MAX)
            {
                /* A real corner: the reference is where this beam meets the
                 * face line. */
                set_corner_reference(p_follow, p_in, (int32_t)p_out->servo_deci);
                set_phase(p_follow, PHASE_CORNER, p_in);
            }
        }
        else
        {
            /* ECHO_AHEAD turns away; no new ping: wait. */
        }
    }
}

/* Hard stops: back at the hit point after going round, or no
 * gain along the original line for OBSTACLE_FOLLOW_NO_PROGRESS_MM. */
static void check_hard_stops (follow_t * p_follow, follow_in_t const * p_in)
{
    int64_t const r_sq = ((int64_t)p_in->x_mm * p_in->x_mm) + ((int64_t)p_in->y_mm * p_in->y_mm);

    if (p_in->y_mm > (int32_t)p_follow->best_y_mm)
    {
        p_follow->best_y_mm      = clamp_i16(p_in->y_mm);
        p_follow->best_travel_mm = p_in->travel_mm;
    }

    if ((p_in->travel_mm > OBSTACLE_FOLLOW_CIRCLE_MIN_MM) &&
        (r_sq < ((int64_t)OBSTACLE_FOLLOW_HIT_RADIUS_MM * OBSTACLE_FOLLOW_HIT_RADIUS_MM)))
    {
        give_up(p_follow, p_in, FOLLOW_REASON_CIRCLED);
    }
    else if ((uint32_t)(p_in->travel_mm - p_follow->best_travel_mm) > OBSTACLE_FOLLOW_NO_PROGRESS_MM)
    {
        give_up(p_follow, p_in, FOLLOW_REASON_NO_PROGRESS);
    }
    else
    {
        /* Going on. */
    }
}

bool follow_bump_update (follow_bump_t * p_bump, int32_t const p_cmd_um_s[FOLLOW_WHEELS],
                         int32_t const p_meas_um_s[FOLLOW_WHEELS], uint32_t dt_ms)
{
    uint32_t wheel   = 0U;
    bool     b_low   = false;
    bool     b_moved = false;

    for (wheel = 0U; wheel < FOLLOW_WHEELS; wheel++)
    {
        int32_t const cmd  = p_cmd_um_s[wheel] / UM_PER_MM;
        int32_t const meas = p_meas_um_s[wheel] / UM_PER_MM;

        if (geom_abs(cmd - (int32_t)p_bump->cmd_mm_s[wheel]) > OBSTACLE_BUMP_CHANGE_MM_S)
        {
            b_moved                 = true;
            p_bump->cmd_mm_s[wheel] = clamp_i16(cmd);
        }
        /* A wheel driven fast enough that turns under the percentage, or the
         * wrong way (the sign product), is too slow. */
        if ((geom_abs(cmd) >= OBSTACLE_BUMP_MIN_MM_S) &&
            (((meas * cmd) < 0) || ((geom_abs(meas) * PERCENT_ALL) < (geom_abs(cmd) * OBSTACLE_BUMP_PERCENT))))
        {
            b_low = true;
        }
    }

    if (b_moved)
    {
        p_bump->settle_ms = 0U;
    }
    else if (p_bump->settle_ms < OBSTACLE_BUMP_SETTLE_MS)
    {
        p_bump->settle_ms = (uint16_t)(p_bump->settle_ms + ((dt_ms > OBSTACLE_BUMP_SETTLE_MS) ? OBSTACLE_BUMP_SETTLE_MS : dt_ms));
    }
    else
    {
        /* Settled. */
    }

    if (b_low && (p_bump->settle_ms >= OBSTACLE_BUMP_SETTLE_MS))
    {
        p_bump->low_ms = (uint16_t)(p_bump->low_ms + ((dt_ms > OBSTACLE_BUMP_MS) ? OBSTACLE_BUMP_MS : dt_ms));
    }
    else
    {
        p_bump->low_ms = 0U;
    }

    return (p_bump->low_ms >= OBSTACLE_BUMP_MS);
}

/* Bug2 leave point: the tape, past the hit point along the original line,
 * and inside the window of the original line. */
static bool at_leave_point (follow_t const * p_follow, follow_in_t const * p_in)
{
    return (p_in->b_line && (p_in->y_mm > OBSTACLE_FOLLOW_LEAVE_AHEAD_MM) &&
            (geom_abs(p_in->x_mm) <= OBSTACLE_FOLLOW_LINE_WINDOW_MM) &&
            (PHASE_LEAVE_CREEP != p_follow->phase) && (PHASE_LEAVE_TURN != p_follow->phase) &&
            (PHASE_DONE != p_follow->phase));
}

void follow_start (follow_t * p_follow, bool b_left, follow_in_t const * p_in, int32_t hit_mm)
{
    /* The echo that stopped the car is the first point of the front face:
     * straight ahead of the scan origin. */
    p_follow->b_left    = b_left ? 1U : 0U;
    p_follow->b_point   = (hit_mm > 0) ? 1U : 0U;
    p_follow->b_seed    = p_follow->b_point;
    p_follow->tries     = 0U;
    p_follow->reason    = FOLLOW_REASON_NONE;
    p_follow->best_y_mm = clamp_i16(p_in->y_mm);
    p_follow->best_travel_mm = p_in->travel_mm;
    p_follow->face_x_mm = 0;
    p_follow->face_y_mm = clamp_i16((hit_mm > 0) ? hit_mm : 0);
    set_face_head(p_follow, away_sign(p_follow) * DECI_RIGHT);
    set_phase(p_follow, PHASE_LEG, p_in);
}

/* Leave the face on the tape: creep onto it, turn to the original heading,
 * then stand still and report the line. */
static void step_leave (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{
    int32_t const back_to_line = p_in->head_deci - geom_wrap_deci(p_in->head_deci);

    if (PHASE_LEAVE_CREEP == p_follow->phase)
    {
        if (driven_mm(p_follow, p_in) > OBSTACLE_FOLLOW_LINE_LEAD_MM)
        {
            set_phase(p_follow, PHASE_LEAVE_TURN, p_in);
            (void)spin(p_in, back_to_line, p_out);
        }
        else
        {
            straight(p_in, p_in->head_deci, (int32_t)OBSTACLE_FOLLOW_LOOK_MM_S, p_out);
        }
    }
    else if (PHASE_LEAVE_TURN == p_follow->phase)
    {
        if (spin(p_in, back_to_line, p_out))
        {
            set_phase(p_follow, PHASE_DONE, p_in);
            p_out->result = FOLLOW_LINE;
        }
    }
    else
    {
        /* PHASE_DONE: stand still and keep reporting the end. */
        p_out->result = FOLLOW_LINE;
    }
}

/* Gave up, in this step or before: the wheels stop at once. */
static void report_give_up (follow_t const * p_follow, follow_out_t * p_out)
{
    if ((PHASE_DONE == p_follow->phase) && (FOLLOW_REASON_NONE != p_follow->reason))
    {
        p_out->left_um_s  = 0;
        p_out->right_um_s = 0;
        p_out->steer      = OBSTACLE_STEER_HOLD;
        p_out->b_turning  = false;
        p_out->result     = FOLLOW_GAVE_UP;
        p_out->reason     = (uint8_t)p_follow->reason;
    }
}

void follow_step (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out)
{

    p_out->left_um_s  = 0;
    p_out->right_um_s = 0;
    p_out->steer      = OBSTACLE_STEER_HOLD;
    p_out->result     = FOLLOW_RUN;
    p_out->reason     = FOLLOW_REASON_NONE;
    p_out->b_turning  = false;
    p_out->servo_deci = (int16_t)(-away_sign(p_follow) * (int32_t)OBSTACLE_FOLLOW_LOOK_DECI);

    if (at_leave_point(p_follow, p_in))
    {
        set_phase(p_follow, PHASE_LEAVE_CREEP, p_in);
    }
    else if (PHASE_DONE != p_follow->phase)
    {
        check_hard_stops(p_follow, p_in);
    }
    else
    {
        /* Done: report below. */
    }

    switch (p_follow->phase)
    {
        case PHASE_LEG:
            step_leg(p_follow, p_in, p_out);
        break;

        case PHASE_IN:
            step_in(p_follow, p_in, p_out);
        break;

        case PHASE_OUT:
            step_out(p_follow, p_in, p_out);
        break;

        case PHASE_CORNER:
            step_corner(p_follow, p_in, p_out);
        break;

        case PHASE_WRAP:
            if (spin(p_in, face_target(p_follow, p_in, 0), p_out))
            {
                set_phase(p_follow, PHASE_IN, p_in);
                arc(-away_sign(p_follow), (int32_t)OBSTACLE_BYPASS_SPEED_MM_S, p_out);
            }
        break;

        case PHASE_BACKUP:
            step_backup(p_follow, p_in, p_out);
        break;

        case PHASE_LOOK:
            step_look(p_follow, p_in, p_out);
        break;

        case PHASE_AWAY:
            if (spin(p_in, face_target(p_follow, p_in, 0), p_out))
            {
                set_phase(p_follow, PHASE_LEG, p_in);
                step_leg(p_follow, p_in, p_out);
            }
        break;

        default:
            /* LEAVE_CREEP, LEAVE_TURN and DONE. */
            step_leave(p_follow, p_in, p_out);
        break;
    }

    report_give_up(p_follow, p_out);
}

/*** end of file ***/
