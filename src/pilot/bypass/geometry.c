/** @file geometry.c
 * @brief Bypass sonar fans and obstacle geometry in the contact frame.
 */

#include "internal.h"

#include <stddef.h>

#include "geometry.h"
#include "pilot/pilot.h"
#include "platform/units.h"

#define AIM_HOLD_TOL    (30)
#define CONTACT_JUMP_MM (250)
#define UNSEEN_EDGE     (INT16_MAX)
#define SIDE_STEP_MM    (60)
#define SIDE_CLEAR_CONFIRM (2U)
#define SLOT_COUNT      ((2 * OBSTACLE_SERVO_LIMIT_DECI / RAY_STEP) + 1)
#define LEVEL_BUFFER_MM (10)
#define ROOM_MAX_MM     (150)
#define LOOK_SLOTS      (1U + OBSTACLE_SERVO_LIMIT_DECI / RAY_STEP)
#define SIDE_MIN_HITS (2U)
#define SIDE_MIN_SPAN_MM (LOOK_FACE_MATCH_MM / HALF_DIVISOR)
#define LAST_SECTOR     (2U)
#define FAN_BUDGET      (9U)
#define FAN_MAX_WING_STEPS (6U)
#define FAN_GAP_SAT     (2U)
#define FAN_REPEAT_NONE (0U)
#define FAN_REPEAT_CENTER (1U)
#define FAN_REPEAT_LEFT_BASE (2U)
#define FAN_REPEAT_RIGHT_BASE (FAN_REPEAT_LEFT_BASE + FAN_MAX_WING_STEPS)
#define FAN_REPEAT_MAX  (15U)
#define PACKED_2BIT_MAX (3U)
#define PACKED_3BIT_MAX (7U)
#define WING_STEPS      ((uint8_t)(OBSTACLE_SERVO_LIMIT_DECI / RAY_STEP))
#define FAN_BUDGET_ACTIVE ((SLOT_COUNT < (int32_t)FAN_BUDGET) ? (uint8_t)SLOT_COUNT : (uint8_t)FAN_BUDGET)
STATIC_ASSERT(LAST_SECTOR <= PACKED_2BIT_MAX, "sector fits 2 bits");
STATIC_ASSERT(FAN_MAX_WING_STEPS <= PACKED_3BIT_MAX, "wing steps fit 3 bits");
STATIC_ASSERT(HUG_TRIGGER_HITS <= PACKED_2BIT_MAX, "blocked pings fit 2 bits");
STATIC_ASSERT((LEFT_BLOCKED | RIGHT_BLOCKED) <= PACKED_2BIT_MAX, "failed sides fit 2 bits");
STATIC_ASSERT(LOOK_VIEWS <= PACKED_2BIT_MAX, "views fit 2 bits");
STATIC_ASSERT((OBSTACLE_SERVO_LIMIT_DECI / RAY_STEP) <= (int32_t)FAN_MAX_WING_STEPS,
              "servo wing steps fit packed fan state");
STATIC_ASSERT((FAN_REPEAT_RIGHT_BASE + FAN_MAX_WING_STEPS - 1U) <= FAN_REPEAT_MAX,
              "repeat slot fits 4 bits");
STATIC_ASSERT(FAN_BUDGET <= LOOK_RAYS_MAX, "fan budget fits look buffer");

void bypass_reset_contact_fan(bypass_t * p_run)
{
    p_run->ray = 0U;
    p_run->unknown = 0U;
    p_run->b_front_seen = false;
    p_run->b_center_clear = false;
    p_run->b_blocked = false;
    p_run->b_swing_blocked = false;
    p_run->nearest_mm = UINT16_MAX;
    p_run->last_hit_deci = -1;
    p_run->edge_deci[0] = UNSEEN_EDGE;
    p_run->edge_deci[1] = UNSEEN_EDGE;
    p_run->b_edge_gap_left = false;
    p_run->b_edge_gap_right = false;
    p_run->fan_left_step = 0U;
    p_run->fan_right_step = 0U;
    p_run->fan_gap_left = 0U;
    p_run->fan_gap_right = 0U;
    p_run->fan_repeat_left = 0U;
    p_run->fan_repeat_right = 0U;
    p_run->fan_repeat_slot = FAN_REPEAT_NONE;
    p_run->b_fan_center_gap = false;
    p_run->b_fan_repeating = false;
}

static bool fan_pose_intact(hug_t const * p_hug, hug_in_t const * p_in)
{
    return p_hug->bypass.b_fan_pose_set &&
           ((uint32_t)p_in->left_um == p_hug->anchor.fan_pose.left_um) &&
           ((uint32_t)p_in->right_um == p_hug->anchor.fan_pose.right_um);
}

bool bypass_fan_pose_moved_usefully(hug_t const * p_hug, hug_in_t const * p_in)
{
    return !p_hug->bypass.b_fan_pose_set ||
           bypass_moved_usefully(p_in, p_hug->anchor.fan_pose.left_um, p_hug->anchor.fan_pose.right_um);
}

int32_t bypass_axis_mm(hug_t const * p_hug, int32_t angle)
{
    return geom_dot_mm(angle, p_hug->x_um / UM_PER_MM, p_hug->y_um / UM_PER_MM);
}

static void retire_contact_fan(hug_t * p_hug, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;

    p_run->b_pending = false;
    p_run->b_fan_repeating = false;
    if ((p_run->b_front_seen ||
         ((p_run->ray >= FAN_BUDGET_ACTIVE) && (p_run->contact_mm <= FRONT_RESERVE_MM))) &&
        (((int32_t)p_run->contact_mm + SHAFT_MM) < (SWING_MM + SPIN_GAP_MM)))
    {
        if (p_run->b_backed)
        {
            bypass_end_route(p_hug, p_out, false);
        }
        else
        {
            p_run->b_backed = true;
            p_run->b_wide = true;
            bypass_enter(p_hug, BYPASS_AHEAD, 0,
                  (int32_t)p_run->contact_mm + SHAFT_MM - (SWING_MM + SPIN_GAP_MM));
        }
        return;
    }
    p_run->b_fan_done = true;
    p_out->b_stop = 1U;
}

/* The scan service returns a request ID and quantized angle, without a
 * trigger timestamp. Keep the chassis still from request through reply. */
bool bypass_ray(hug_t * p_hug, hug_in_t const * p_in, int32_t aim, int32_t gaze,
                hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    bool const b_contact_fan = (BYPASS_CONTACT == p_run->phase) && (0U == p_run->sector);
    int32_t const servo = geom_wrap_deci(gaze - aim);
    int32_t const tol = (p_run->b_pending || p_run->b_look_ready) ? AIM_HOLD_TOL : AIM_TOL;
    if ((geom_abs(servo) > OBSTACLE_SERVO_LIMIT_DECI) || !bypass_turn_within(p_hug, aim, tol, p_out))
    {
        p_run->b_pending = false;
        p_out->b_scan_pivot = (p_out->turn_deci_s != 0);
        return false;
    }
    if (b_contact_fan && p_run->b_survey_timed &&
        ((uint32_t)(p_in->now_ms - p_run->survey_ms) >= BYPASS_FAN_DEADLINE_MS))
    {
        retire_contact_fan(p_hug, p_out);
        return false;
    }
    if (!p_in->b_stationary)
    {
        p_run->b_pending = false;
        return false;
    }
    bool const b_pose_held = !b_contact_fan || fan_pose_intact(p_hug, p_in);
    if (!b_pose_held)
    {
        p_run->b_pending = false;
    }
    bool const b_matched = p_run->b_pending && b_pose_held &&
        obstacle_ping_reply_matches(p_in->b_ping_reply, p_in->ping_seq, p_in->ping_servo_deci,
                                    p_run->seq, servo, OBSTACLE_PING_MAX_QUANTIZATION_DECI) &&
        (geom_abs(geom_wrap_deci(hug_pose_heading_deci(p_hug) - p_run->requested_head)) <= AIM_HOLD_TOL);
    bool const b_expired = p_run->b_pending &&
        ((uint32_t)(p_in->now_ms - p_run->request_ms) >= HUG_REPLY_TIMEOUT_MS);
    if (b_matched && !b_expired)
    {
        p_run->b_pending = false;
        return true;
    }
    if (b_contact_fan && b_expired)
    {
        if (p_run->b_fan_repeated)
        {
            retire_contact_fan(p_hug, p_out);
            return false;
        }
        p_run->b_fan_repeated = true;
    }
    if (!p_run->b_pending || b_expired)
    {
        if (!p_run->b_survey_timed || (b_expired && !b_contact_fan))
        {
            p_run->survey_ms = p_in->now_ms;
            p_run->b_survey_timed = true;
        }
        if (b_contact_fan && !p_run->b_fan_pose_set)
        {
            p_hug->anchor.fan_pose.left_um = (uint32_t)p_in->left_um;
            p_hug->anchor.fan_pose.right_um = (uint32_t)p_in->right_um;
            p_run->b_fan_pose_set = true;
            p_hug->b_fan_pending = true;
        }
        p_run->request_ms = p_in->now_ms;
        p_run->seq = (uint8_t)((p_run->seq + 1U) & OBSTACLE_PING_SEQ_MK);
        p_run->requested_head = (int16_t)hug_pose_heading_deci(p_hug);
        p_run->b_pending = true;
    }
    p_out->seq = p_run->seq;
    p_out->b_req = 1U;
    p_out->servo_deci = (int16_t)servo;
    p_out->b_stop = b_expired ? 1U : 0U;
    return false;
}

static void remember_contact(bypass_t * p_run, look_ray_t const * p_ray)
{
    if (!p_run->b_anchor && (look_ray_echo(p_ray) == LOOK_ECHO_VALID) &&
        (p_ray->range_mm <= HUG_STANDOFF_MM))
    {
        int32_t const angle = look_ray_angle_deci(p_ray);
        p_run->contact_x_mm = (int16_t)(p_ray->x_mm + geom_mul_q14(p_ray->range_mm, geom_cos_q14(angle)));
        p_run->contact_y_mm = (int16_t)(p_ray->y_mm + geom_mul_q14(p_ray->range_mm, geom_sin_q14(angle)));
        p_run->b_anchor = true;
    }
}

static void contact_ray(hug_t * p_hug, hug_in_t const * p_in, int32_t bearing)
{
    bypass_t * const p_run = &p_hug->bypass;
    bool const b_point = bypass_is_point(p_in);
    bool const b_open = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
        (b_point && ((int32_t)p_in->ping_mm > p_run->contact_mm + CONTACT_JUMP_MM));
    if (geom_abs(bearing) <= AIM_TOL)
    {
        p_run->b_center_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
            (b_point && (p_in->ping_mm > HUG_STANDOFF_MM));
    }
    p_run->b_front_seen = p_run->b_front_seen || (b_point && (p_in->ping_mm <= HUG_STANDOFF_MM));
    if (!b_open && !b_point && p_run->unknown < UINT8_MAX)
    {
        p_run->unknown++;
    }
    if (b_point && (p_in->ping_mm < p_run->contact_mm))
    {
        p_run->contact_mm = p_in->ping_mm;
    }
    /* Each edge is the first cleared bearing beyond a contact. Unknown
     * samples before it break the chain on that side only. */
    unsigned const side = (bearing >= 0) ? 0U : 1U;
    if (!b_open && !b_point)
    {
        p_run->b_edge_gap_left = p_run->b_edge_gap_left || (bearing >= -AIM_TOL);
        p_run->b_edge_gap_right = p_run->b_edge_gap_right || (bearing <= AIM_TOL);
        if (geom_abs(bearing) <= AIM_TOL)
        {
            p_run->b_fan_center_gap = true;
        }
        else if (0U == side)
        {
            uint8_t const step = (uint8_t)(geom_abs(bearing) / RAY_STEP);
            if ((0U == p_run->fan_gap_left) && (step <= FAN_MAX_WING_STEPS))
            {
                p_run->fan_repeat_left = (uint8_t)(step & PACKED_3BIT_MAX);
            }
            if (p_run->fan_gap_left < FAN_GAP_SAT)
            {
                p_run->fan_gap_left = (uint8_t)((p_run->fan_gap_left + 1U) & PACKED_2BIT_MAX);
            }
        }
        else
        {
            uint8_t const step = (uint8_t)(geom_abs(bearing) / RAY_STEP);
            if ((0U == p_run->fan_gap_right) && (step <= FAN_MAX_WING_STEPS))
            {
                p_run->fan_repeat_right = (uint8_t)(step & PACKED_3BIT_MAX);
            }
            if (p_run->fan_gap_right < FAN_GAP_SAT)
            {
                p_run->fan_gap_right = (uint8_t)((p_run->fan_gap_right + 1U) & PACKED_2BIT_MAX);
            }
        }
    }
    else if (b_open && (geom_abs(bearing) > AIM_TOL))
    {
        if (p_run->edge_deci[side] == UNSEEN_EDGE)
        {
            p_run->edge_deci[side] = (int16_t)geom_abs(bearing);
        }
    }
    else if (b_point && (geom_abs(bearing) > AIM_TOL))
    {
        /* A second obstruction on this side takes precedence over a gap. */
        p_run->edge_deci[side] = UNSEEN_EDGE;
    }
}

/* Keep measured edge geometry available to telemetry. Route selection can
 * proceed from a clearance transition even when a round post gives no fit. */
static void report_front(hug_t * p_hug, look_t const * p_look)
{
    look_pose_t const pose = hug_axle_pose(p_hug);
    look_face_t face;
    look_edge_t edge[BYPASS_SIDES];
    bool have[BYPASS_SIDES];
    p_hug->width_mm = 0U;
    p_hug->seen_mm = (int16_t)(p_hug->bypass.contact_mm + SHAFT_MM);
    if (look_face_near(p_look, &pose, (uint16_t)(p_hug->bypass.contact_mm + LOOK_FACE_MATCH_MM), &face))
    {
        look_edges(p_look, &face, edge, have);
        if (have[LOOK_SIDE_LEFT] && have[LOOK_SIDE_RIGHT])
        {
            int32_t const dx = edge[LOOK_SIDE_LEFT].x_mm - edge[LOOK_SIDE_RIGHT].x_mm;
            int32_t const dy = edge[LOOK_SIDE_LEFT].y_mm - edge[LOOK_SIDE_RIGHT].y_mm;
            p_hug->width_mm = (uint16_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy)));
            p_hug->seen_mm = (int16_t)face.dist_mm;
        }
    }
}

/* An isolated long or missing echo does not end a face supported by returns
 * ahead and on both wings of the stationary fan. The side leg has its own
 * centered clearance check after the car turns. */
static bool spans_front(hug_t const * p_hug, look_t const * p_look)
{
    look_face_t face;
    look_pose_t const pose = hug_axle_pose(p_hug);
    if (!look_face_near(p_look, &pose, (uint16_t)(p_hug->bypass.contact_mm + LOOK_FACE_MATCH_MM), &face))
    {
        return false;
    }
    bool center = false;
    bool near_left = false;
    bool near_right = false;
    int32_t left = 0;
    int32_t right = 0;
    unsigned hits = 0U;
    for (unsigned i = 0U; i < p_look->count; ++i)
    {
        look_ray_t const * const ray = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(ray);
        if ((look_ray_echo(ray) != LOOK_ECHO_VALID) || (geom_abs(angle) > OBSTACLE_SERVO_LIMIT_DECI + AIM_TOL))
        {
            continue;
        }
        int32_t const x = ray->x_mm + geom_mul_q14(ray->range_mm, geom_cos_q14(angle));
        int32_t const y = ray->y_mm + geom_mul_q14(ray->range_mm, geom_sin_q14(angle));
        if (geom_abs(geom_dot_mm(face.normal_deci, x - face.x_mm, y - face.y_mm)) <= LOOK_FACE_MATCH_MM)
        {
            hits++;
            center = center || (geom_abs(angle) <= AIM_TOL);
            near_left = near_left || (geom_abs(angle - RAY_STEP) <= AIM_TOL);
            near_right = near_right || (geom_abs(angle + RAY_STEP) <= AIM_TOL);
            left = (angle > left) ? angle : left;
            right = (angle < right) ? angle : right;
        }
    }
    int32_t const wing = OBSTACLE_SERVO_LIMIT_DECI - RAY_STEP - AIM_TOL;
    bool const supported = center || (!p_hug->bypass.b_center_clear && near_left && near_right);
    return supported && (hits >= SLOT_COUNT / HALF_DIVISOR + 1U) && (left >= wing) && (right <= -wing);
}

/* A post wholly across the entry axis can have a clear side that starts
 * before the centre ray. Preserve that signed bound inside the tape corridor. */
static int32_t front_room(hug_t const * p_hug, look_t const * p_look, int32_t angle)
{
    bypass_t const * const run = &p_hug->bypass;
    int32_t const range = run->contact_mm;
    if (p_hug->b_approach_tape && run->b_anchor && run->b_center_clear)
    {
        int32_t outer_hit = INT32_MIN;
        int32_t front = run->contact_x_mm;
        look_ray_t const * p_clear = NULL;
        for (uint8_t i = 0U; i < p_look->count; i++)
        {
            look_ray_t const * const p_ray = &p_look->rays[i];
            int32_t const bearing = p_hug->side * look_ray_angle_deci(p_ray);
            if (look_echo_is_point(look_ray_echo(p_ray)) && (p_ray->range_mm <= range + CONTACT_JUMP_MM))
            {
                int32_t const x = p_ray->x_mm + geom_mul_q14(p_ray->range_mm, geom_cos_q14(bearing));
                outer_hit = (bearing > outer_hit) ? bearing : outer_hit;
                front = (x < front) ? x : front;
            }
        }
        for (uint8_t i = 0U; i < p_look->count; i++)
        {
            look_ray_t const * const p_ray = &p_look->rays[i];
            int32_t const bearing = p_hug->side * look_ray_angle_deci(p_ray);
            if ((outer_hit >= -OBSTACLE_SERVO_LIMIT_DECI - AIM_TOL) && (bearing <= 0) &&
                (geom_abs(bearing - outer_hit - RAY_STEP) <= AIM_TOL) &&
                ((p_clear == NULL) || (bearing > p_hug->side * look_ray_angle_deci(p_clear))))
            {
                p_clear = p_ray;
            }
        }
        if (p_clear != NULL)
        {
            int32_t const bearing = look_ray_angle_deci(p_clear);
            int32_t const depth = clamp_i32(front - p_clear->x_mm - LOOK_FACE_MATCH_MM, 0, TRAVEL_LIMIT_MM);
            int32_t const edge = p_hug->side * (p_clear->y_mm + geom_mul_q14(depth, geom_tan_q14(bearing)));
            return clamp_i32(edge + 1, -ROOM_MAX_MM, ROOM_MAX_MM);
        }
    }
    return clamp_i32(geom_mul_q14(range, geom_tan_q14(angle)), 0, ROOM_MAX_MM);
}

static bool choose_side(hug_t * p_hug, look_t const * p_look)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const left = (p_run->b_edge_gap_left || (p_run->failed_sides & LEFT_BLOCKED) != 0U) ? UNSEEN_EDGE : p_run->edge_deci[0];
    int32_t const right = (p_run->b_edge_gap_right || (p_run->failed_sides & RIGHT_BLOCKED) != 0U) ? UNSEEN_EDGE : p_run->edge_deci[1];
    int32_t const extent = (p_run->sector == 0U) ? OBSTACLE_SERVO_LIMIT_DECI : FULL_PAN;
    bool const complete = (p_run->sector == LAST_SECTOR) ||
        ((p_run->sector == 0U) && (p_run->ray >= FAN_BUDGET_ACTIVE));
    bool const wall = complete && (left > extent - RAY_STEP) && (right > extent - RAY_STEP) &&
        (p_run->failed_sides != (LEFT_BLOCKED | RIGHT_BLOCKED)) && spans_front(p_hug, p_look);
    if (!wall && (left == UNSEEN_EDGE) && (right == UNSEEN_EDGE))
    {
        return false;
    }
    p_run->b_saw_side = false;
    p_run->b_retried = false;
    p_hug->side_end_x_mm = 0;
    p_hug->side_end_y_mm = 0;
    p_hug->breadth_mm = 0U;
    p_run->b_wall = wall;
    p_run->b_wall_margin = false;
    report_front(p_hug, p_look);
    if (wall)
    {
        p_hug->width_mm = 0U;
    }
    p_hug->side = wall ? (((p_run->failed_sides & LEFT_BLOCKED) == 0U) ? 1 : -1) :
                        ((left <= right) ? 1 : -1);
    p_run->out_origin_mm = (int16_t)bypass_axis_mm(p_hug, p_hug->side * RIGHT_ANGLE);
    p_run->views = (p_run->b_anchor && !wall) ? 0U : LOOK_VIEWS;
    int32_t const angle = wall ? 0 : ((left <= right) ? left : right);
    int32_t const range = p_run->contact_mm;
    int32_t const ahead = clamp_i32(range + SHAFT_MM - (SWING_MM + SPIN_GAP_MM), -SWING_MM, range / HALF_DIVISOR);
    int32_t const room = front_room(p_hug, p_look, angle);
    p_run->ahead_mm = (int16_t)ahead;
    p_run->out_mm = wall ? TRACK_STEP : (int16_t)(room + SWING_MM + SPIN_GAP_MM);
    p_run->level_mm = (int16_t)(range + LEVEL_BUFFER_MM + (range / HALF_DIVISOR) - ahead);
    bypass_enter(p_hug, BYPASS_ALIGN, 0, 0);
    return true;
}

static bool cleared_contact_fan(bypass_t const * p_run, unsigned count)
{
    /* A previously close post can disappear alongside the front tips.
     * Reobserve it from the backed-off pose before releasing the approach. */
    return (0U == p_run->sector) && (p_run->ray == count) &&
           !p_run->b_front_seen && p_run->b_center_clear && (0U == p_run->unknown) &&
           (p_run->contact_mm > FRONT_RESERVE_MM);
}

static bool repeat_front_fan(bypass_t const * p_run)
{
    if (!p_run->b_front_seen && !p_run->b_backed && (p_run->contact_mm <= FRONT_RESERVE_MM))
    {
        /* A fresh view requires separation from the last close contact. */
        return false;
    }
    return (!p_run->b_front_seen && (p_run->unknown != 0U)) ||
           (p_run->b_edge_gap_left && p_run->b_edge_gap_right);
}

static bool contact_wing_usable(bypass_t const * p_run, unsigned wing)
{
    if (0U == wing)
    {
        return ((p_run->failed_sides & LEFT_BLOCKED) == 0U) && !p_run->b_edge_gap_left;
    }
    return ((p_run->failed_sides & RIGHT_BLOCKED) == 0U) && !p_run->b_edge_gap_right;
}

static uint8_t contact_wing_step(bypass_t const * p_run, unsigned wing)
{
    return (0U == wing) ? p_run->fan_left_step : p_run->fan_right_step;
}

static int32_t contact_repeat_offset(uint8_t slot)
{
    if (FAN_REPEAT_CENTER == slot)
    {
        return 0;
    }
    if (slot >= FAN_REPEAT_RIGHT_BASE)
    {
        return -((int32_t)(slot - FAN_REPEAT_RIGHT_BASE) + 1) * RAY_STEP;
    }
    return ((int32_t)(slot - FAN_REPEAT_LEFT_BASE) + 1) * RAY_STEP;
}

static int32_t contact_fan_offset(bypass_t const * p_run)
{
    unsigned wing = 0U;

    if (p_run->b_fan_repeating)
    {
        return contact_repeat_offset(p_run->fan_repeat_slot);
    }
    if (0U == p_run->ray)
    {
        return 0;
    }
    unsigned const pref = ((p_run->failed_sides & LEFT_BLOCKED) != 0U) ? 1U : 0U;
    unsigned const other = 1U - pref;
    if ((contact_wing_step(p_run, pref) < WING_STEPS) && contact_wing_usable(p_run, pref))
    {
        wing = pref;
    }
    else if ((contact_wing_step(p_run, other) < WING_STEPS) && contact_wing_usable(p_run, other))
    {
        wing = other;
    }
    else if (contact_wing_step(p_run, pref) < WING_STEPS)
    {
        wing = pref;
    }
    else if (contact_wing_step(p_run, other) < WING_STEPS)
    {
        wing = other;
    }
    else
    {
        return 0;
    }
    int32_t const step = (int32_t)contact_wing_step(p_run, wing) + 1;
    return (0U == wing) ? (step * RAY_STEP) : -(step * RAY_STEP);
}

static bool contact_fan_gate(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                             hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;

    if (p_run->b_fan_done)
    {
        if (bypass_fan_pose_moved_usefully(p_hug, p_in))
        {
            p_hug->last_motion_ms = p_in->now_ms;
            bypass_enter(p_hug, BYPASS_CONTACT, 0, 0);
            look_clear(p_look);
        }
        else
        {
            p_run->b_pending = false;
            if ((uint32_t)(p_in->now_ms - p_hug->last_motion_ms) >= PILOT_RECOVER_MS)
            {
                /* A repeated stop or reverse would alternate with a re-triggered fan forever. */
                bypass_end_route(p_hug, p_out, false);
            }
            else
            {
                p_out->b_stop = 1U;
            }
            return false;
        }
    }
    if (p_run->b_survey_timed &&
        ((uint32_t)(p_in->now_ms - p_run->survey_ms) >= BYPASS_FAN_DEADLINE_MS))
    {
        retire_contact_fan(p_hug, p_out);
        return false;
    }
    if (!p_in->b_stationary)
    {
        p_run->b_pending = false;
        return false;
    }
    if (p_run->b_survey_timed && !fan_pose_intact(p_hug, p_in))
    {
        if (bypass_fan_pose_moved_usefully(p_hug, p_in))
        {
            p_hug->last_motion_ms = p_in->now_ms;
            bypass_enter(p_hug, BYPASS_CONTACT, 0, 0);
            look_clear(p_look);
        }
        else
        {
            retire_contact_fan(p_hug, p_out);
            return false;
        }
    }
    return true;
}

static uint8_t contact_fan_repeat_slot(bypass_t const * p_run)
{
    uint8_t repeat_slot = FAN_REPEAT_NONE;
    if (!p_run->b_fan_repeated && (0U != p_run->unknown))
    {
        bool const b_left_repeat = (1U == p_run->fan_gap_left) && (0U != p_run->fan_repeat_left);
        bool const b_right_repeat = (1U == p_run->fan_gap_right) && (0U != p_run->fan_repeat_right);
        if (!p_run->b_fan_center_gap && b_left_repeat && ((p_run->failed_sides & LEFT_BLOCKED) == 0U) &&
            (p_run->edge_deci[0] != UNSEEN_EDGE))
        {
            repeat_slot = (uint8_t)(FAN_REPEAT_LEFT_BASE + p_run->fan_repeat_left - 1U);
        }
        else if (!p_run->b_fan_center_gap && b_right_repeat && ((p_run->failed_sides & RIGHT_BLOCKED) == 0U) &&
                 (p_run->edge_deci[1] != UNSEEN_EDGE))
        {
            repeat_slot = (uint8_t)(FAN_REPEAT_RIGHT_BASE + p_run->fan_repeat_right - 1U);
        }
        else if (1U == p_run->unknown)
        {
            if (p_run->b_fan_center_gap)
            {
                repeat_slot = FAN_REPEAT_CENTER;
            }
            else if (b_left_repeat)
            {
                repeat_slot = (uint8_t)(FAN_REPEAT_LEFT_BASE + p_run->fan_repeat_left - 1U);
            }
            else if (b_right_repeat)
            {
                repeat_slot = (uint8_t)(FAN_REPEAT_RIGHT_BASE + p_run->fan_repeat_right - 1U);
            }
        }
    }
    return repeat_slot;
}

static void contact_fan_record(hug_t * p_hug, hug_in_t const * p_in, int32_t offset)
{
    bypass_t * const p_run = &p_hug->bypass;

    if (p_run->b_fan_repeating)
    {
        bool const b_point = bypass_is_point(p_in);
        bool const b_open = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
            (b_point && ((int32_t)p_in->ping_mm > (int32_t)p_run->contact_mm + CONTACT_JUMP_MM));
        if (b_open || b_point)
        {
            if (p_run->unknown > 0U)
            {
                p_run->unknown--;
            }
            p_run->b_front_seen = p_run->b_front_seen || (b_point && (p_in->ping_mm <= HUG_STANDOFF_MM));
            if (b_point && (p_in->ping_mm < p_run->contact_mm))
            {
                p_run->contact_mm = p_in->ping_mm;
            }
            if (geom_abs(offset) <= AIM_TOL)
            {
                p_run->b_center_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
                    (b_point && (p_in->ping_mm > HUG_STANDOFF_MM));
                p_run->b_fan_center_gap = false;
                p_run->b_edge_gap_left = (p_run->fan_gap_left != 0U);
                p_run->b_edge_gap_right = (p_run->fan_gap_right != 0U);
            }
            else
            {
                unsigned const side = (offset > 0) ? 0U : 1U;
                int32_t const mag = geom_abs(offset);
                if (0U == side)
                {
                    if (p_run->fan_gap_left > 0U)
                    {
                        p_run->fan_gap_left = (uint8_t)((p_run->fan_gap_left - 1U) & PACKED_2BIT_MAX);
                    }
                    p_run->b_edge_gap_left = p_run->b_fan_center_gap || (p_run->fan_gap_left != 0U);
                }
                else
                {
                    if (p_run->fan_gap_right > 0U)
                    {
                        p_run->fan_gap_right = (uint8_t)((p_run->fan_gap_right - 1U) & PACKED_2BIT_MAX);
                    }
                    p_run->b_edge_gap_right = p_run->b_fan_center_gap || (p_run->fan_gap_right != 0U);
                }
                if (b_open)
                {
                    if ((p_run->edge_deci[side] != UNSEEN_EDGE) && (mag < p_run->edge_deci[side]))
                    {
                        p_run->edge_deci[side] = (int16_t)mag;
                    }
                    else if ((p_run->edge_deci[side] == UNSEEN_EDGE) &&
                             (mag == ((int32_t)WING_STEPS * RAY_STEP)))
                    {
                        p_run->edge_deci[side] = (int16_t)mag;
                    }
                }
                else if ((p_run->edge_deci[side] != UNSEEN_EDGE) && (mag > p_run->edge_deci[side]))
                {
                    p_run->edge_deci[side] = UNSEEN_EDGE;
                }
            }
        }
        p_run->b_fan_repeating = false;
    }
    else
    {
        contact_ray(p_hug, p_in, offset);
        if (offset > 0)
        {
            p_run->fan_left_step = (uint8_t)((p_run->fan_left_step + 1U) & PACKED_3BIT_MAX);
        }
        else if (offset < 0)
        {
            p_run->fan_right_step = (uint8_t)((p_run->fan_right_step + 1U) & PACKED_3BIT_MAX);
        }
        p_run->ray++;
    }
}

static bool contact_fan_decide(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                               hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;

    bool b_ready = false;
    for (unsigned wing = 0U; wing < BYPASS_SIDES; wing++)
    {
        int32_t const edge = p_run->edge_deci[wing];
        int32_t const reached = (int32_t)contact_wing_step(p_run, wing) * RAY_STEP;
        b_ready = b_ready || (p_run->b_front_seen && contact_wing_usable(p_run, wing) &&
                              (edge != UNSEEN_EDGE) && (reached >= (edge + RAY_STEP)));
    }
    if (b_ready && choose_side(p_hug, p_look))
    {
        return true;
    }
    if (p_run->ray < FAN_BUDGET_ACTIVE)
    {
        /* Emit the next bearing immediately so servo settle overlaps the 60 ms spacing. */
        int32_t const next_offset = contact_fan_offset(p_run);
        (void)bypass_ray(p_hug, p_in, 0, next_offset, p_out);
        return true;
    }
    if (cleared_contact_fan(p_run, FAN_BUDGET_ACTIVE))
    {
        p_hug->state = 0U;
        p_hug->hist = 0U;
        p_run->phase = BYPASS_IDLE;
        return true;
    }
    if (p_run->b_front_seen && choose_side(p_hug, p_look))
    {
        return true;
    }
    if (p_run->b_front_seen && (p_run->failed_sides == (LEFT_BLOCKED | RIGHT_BLOCKED)))
    {
        bypass_end_route(p_hug, p_out, false);
        return true;
    }
    return false;
}

static void contact_fan_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in,
                             hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;

    if (!contact_fan_gate(p_hug, p_look, p_in, p_out))
    {
        return;
    }
    int32_t const offset = contact_fan_offset(p_run);
    if (!bypass_ray(p_hug, p_in, 0, offset, p_out))
    {
        return;
    }
    look_pose_t const pose = hug_axle_pose(p_hug);
    if (look_add(p_look, &pose, p_in->ping_servo_deci, p_in->ping_echo, p_in->ping_mm))
    {
        remember_contact(p_run, &p_look->rays[p_look->count - 1U]);
    }
    contact_fan_record(p_hug, p_in, offset);
    if (contact_fan_decide(p_hug, p_look, p_in, p_out))
    {
        return;
    }
    uint8_t const repeat_slot = contact_fan_repeat_slot(p_run);
    if ((FAN_REPEAT_NONE != repeat_slot) &&
        ((uint32_t)(p_in->now_ms - p_run->survey_ms) < BYPASS_FAN_DEADLINE_MS))
    {
        p_run->b_fan_repeated = true;
        p_run->b_fan_repeating = true;
        p_run->fan_repeat_slot = repeat_slot & FAN_REPEAT_MAX;
        (void)bypass_ray(p_hug, p_in, 0, contact_repeat_offset(repeat_slot), p_out);
        return;
    }
    if (repeat_front_fan(p_run) || !p_run->b_front_seen ||
        (((int32_t)p_run->contact_mm + SHAFT_MM) < (SWING_MM + SPIN_GAP_MM)))
    {
        retire_contact_fan(p_hug, p_out);
        return;
    }
    p_run->sector = (uint8_t)((p_run->sector + 1U) & PACKED_2BIT_MAX);
    p_run->ray = 0U;
    p_run->b_survey_timed = false;
    p_run->b_fan_pose_set = false;
}

/* Sample the centre first, then progressively farther on each side. The
 * second and third sectors use chassis yaw to supply the missing bearings. */
void bypass_sweep(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    bool const b_contact = (BYPASS_CONTACT == p_run->phase);
    if (b_contact && (0U == p_run->sector))
    {
        contact_fan_step(p_hug, p_look, p_in, p_out);
        return;
    }
    int32_t const base = b_contact ? 0 : -p_hug->side * RIGHT_ANGLE;
    int32_t const sector = (0 == p_run->sector) ? 0 :
        ((1 == p_run->sector) ? 1 : -1) * (FULL_PAN - OBSTACLE_SERVO_LIMIT_DECI);
    int32_t const half = OBSTACLE_SERVO_LIMIT_DECI / RAY_STEP;
    int32_t const offset = (0U == p_run->sector) ?
        (((int32_t)p_run->ray <= half) ? ((int32_t)p_run->ray * RAY_STEP) :
         -((int32_t)p_run->ray - half) * RAY_STEP) :
        ((sector > 0) ? 1 : -1) * (OBSTACLE_SERVO_LIMIT_DECI + (p_run->ray + 1) * RAY_STEP);
    int32_t const aim = base + sector;
    if (!bypass_ray(p_hug, p_in, aim, base + offset, p_out))
    {
        return;
    }
    look_pose_t const pose = hug_axle_pose(p_hug);
    if (look_add(p_look, &pose, p_in->ping_servo_deci, p_in->ping_echo, p_in->ping_mm) && b_contact)
    {
        remember_contact(p_run, &p_look->rays[p_look->count - 1U]);
    }
    if (b_contact)
    {
        /* The matched request supplies the angular bin for this reading. */
        contact_ray(p_hug, p_in, offset);
    }
    else
    {
        bypass_lane_ray(p_hug, p_in, base, false);
    }
    p_run->ray++;
    unsigned const n = (0U == p_run->sector) ? SLOT_COUNT :
                       (FULL_PAN - OBSTACLE_SERVO_LIMIT_DECI) / RAY_STEP;
    bool const b_slow = (p_in->now_ms - p_run->survey_ms >= HUG_MOVE_FLOOR_MS) &&
        (p_run->ray >= 3U) && (p_run->sector == 0U);
    if ((p_run->ray < n) && !b_slow)
    {
        return;
    }
    if (b_contact && p_run->b_front_seen && (LAST_SECTOR == p_run->sector) && choose_side(p_hug, p_look))
    {
        return;
    }
    if (!b_contact && (p_run->b_swing_blocked || p_run->b_blocked || (p_run->unknown > 0U)))
    {
        bypass_move_on(p_hug, true);
        return;
    }
    if (b_contact && p_run->b_front_seen && (p_run->failed_sides == (LEFT_BLOCKED | RIGHT_BLOCKED)))
    {
        bypass_end_route(p_hug, p_out, false);
        return;
    }
    if (b_slow && !b_contact && (p_run->ray < n))
    {
        bypass_move_on(p_hug, true);
    }
    else if (p_run->sector < LAST_SECTOR)
    {
        p_run->sector = (uint8_t)((p_run->sector + 1U) & PACKED_2BIT_MAX);
        p_run->ray = 0U;
        p_run->b_survey_timed = false;
        p_run->b_fan_pose_set = false;
    }
    else if (b_contact)
    {
        bypass_end_route(p_hug, p_out, false);
    }
    else
    {
        bypass_enter(p_hug, BYPASS_TRACK_OUT, base, 0);
    }
}

/* The first ray must find the original contact from the displaced axle.
 * Bracket the chosen edge between that surface and a farther or open ray.
 * An isolated gap, a nearer obstruction or an incomplete sweep is ambiguous. */
static bool refine_outward(hug_t * p_hug, look_t const * p_look)
{
    bypass_t * const run = &p_hug->bypass;
    int32_t edge = INT32_MIN;
    if (p_look->count != LOOK_SLOTS)
    {
        return false;
    }
    for (unsigned i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const r = &p_look->rays[i];
        int32_t const angle = look_ray_angle_deci(r);
        int32_t const dx = run->contact_x_mm - r->x_mm;
        if ((dx <= 0) || (geom_abs(angle) > FULL_PAN) ||
            ((i != 0U) && (p_hug->side * (angle - look_ray_angle_deci(&p_look->rays[i - 1U])) <= 0)))
        {
            return false;
        }
        int32_t const depth = geom_mul_q14(r->range_mm, geom_cos_q14(angle)) - dx;
        bool const on_surface = (look_ray_echo(r) == LOOK_ECHO_VALID) && (geom_abs(depth) <= LOOK_FACE_MATCH_MM);
        if (i == 0U)
        {
            int32_t const y = r->y_mm + geom_mul_q14(r->range_mm, geom_sin_q14(angle));
            if (!on_surface || geom_abs(y - run->contact_y_mm) > LOOK_FACE_MATCH_MM)
            {
                return false;
            }
        }
        if (on_surface && (edge == INT32_MIN))
        {
            continue;
        }
        if ((look_ray_echo(r) != LOOK_ECHO_OPEN) &&
            ((look_ray_echo(r) != LOOK_ECHO_VALID) || (depth < LOOK_EDGE_STEP_MM)))
        {
            return false;
        }
        if (edge == INT32_MIN)
        {
            /* Use the clear bearing and the outward extreme of the allowed
             * contact depth. Both coordinates are relative to the entry axle. */
            int32_t const slope = geom_tan_q14(angle);
            edge = p_hug->side * (r->y_mm + geom_mul_q14(dx, slope)) +
                   geom_mul_q14(LOOK_FACE_MATCH_MM, geom_abs(slope)) + 1;
        }
    }
    if (edge == INT32_MIN)
    {
        return false;
    }
    run->out_mm = (int16_t)(clamp_i32(edge - run->out_origin_mm, -ROOM_MAX_MM, TRAVEL_LIMIT_MM - SWING_MM - SPIN_GAP_MM)
                            + SWING_MM + SPIN_GAP_MM);
    return true;
}

void bypass_look_back(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    look_pose_t const pose = hug_axle_pose(p_hug);
    if ((p_in->now_ms - run->look_start_ms) < HUG_STATIONARY_LIMIT_MS)
    {
        if (!run->b_look_ready)
        {
            /* Check the spin envelope ahead at the stopped side-step pose.
             * The approach leg already established clearance to the near face. */
            bool const b_aimed = bypass_turn_within(p_hug, p_hug->side * RIGHT_ANGLE, AIM_HOLD_TOL, p_out);
            if (!b_aimed || !p_in->b_stationary)
            {
                run->last_forward = p_in->ping_seq;
                p_out->b_scan_pivot = !b_aimed;
                return;
            }
            if (p_in->b_ping_reply || p_in->ping_servo_deci != 0 ||
                p_in->ping_seq == run->last_forward)
            {
                return;
            }
            run->last_forward = p_in->ping_seq;
            run->b_look_ready = (p_in->ping_echo == LOOK_ECHO_OPEN) ||
                ((p_in->ping_echo == LOOK_ECHO_VALID) &&
                 (p_in->ping_mm > SWING_MM + SPIN_GAP_MM - SHAFT_MM - SONAR_BOARD_MM));
            if (!run->b_look_ready)
            {
                return;
            }
            p_hug->goal_head = (int16_t)geom_atan_deci(run->contact_y_mm - pose.y_mm, run->contact_x_mm - pose.x_mm);
        }
        int32_t const aim = p_hug->goal_head;
        int32_t const gaze = aim + p_hug->side * run->ray * RAY_STEP;
        if (!bypass_ray(p_hug, p_in, aim, gaze, p_out))
        {
            return;
        }
        if (look_add(p_look, &pose, p_in->ping_servo_deci, p_in->ping_echo, p_in->ping_mm) &&
            ((run->ray != 0U) || (p_in->ping_echo == LOOK_ECHO_VALID)))
        {
            if (++run->ray < LOOK_SLOTS)
            {
                return;
            }
            if (refine_outward(p_hug, p_look))
            {
                run->views = LOOK_VIEWS;
            }
        }
    }
    bypass_enter(p_hug, BYPASS_AWAY, p_hug->side * RIGHT_ANGLE, 0);
}

/* From a parallel lane, look diagonally inward. The forward end of the fan
 * samples progressively farther along the hugged boundary. Each fan has a
 * single stationary axle origin, so its measured points can also be plotted. */
int32_t bypass_side_reach(hug_t const * p_hug, int32_t aim)
{
    int32_t const nose = p_hug->side > 0 ? CHASSIS_NOSE_WIDTH_MM / 2 : CHASSIS_NOSE_LEFT_MM;
    int32_t const ir_half = (CHASSIS_IR_WIDTH_MM + 1) / 2;
    int32_t reach = geom_dot_mm(aim, nose, CHASSIS_NOSE_FORWARD_MM);
    int32_t tip = geom_dot_mm(aim, ir_half, CHASSIS_FRONT_TIP_MM);
    if (aim == RIGHT_ANGLE)
    {
        /* The widest point can pass the obstacle before the pivot ends. */
        reach = (int32_t)geom_isqrt((uint32_t)(nose * nose + CHASSIS_NOSE_FORWARD_MM * CHASSIS_NOSE_FORWARD_MM)) + 1;
        tip = (int32_t)geom_isqrt((uint32_t)(ir_half * ir_half + CHASSIS_FRONT_TIP_MM * CHASSIS_FRONT_TIP_MM)) + 1;
    }
    if (tip > reach)
    {
        reach = tip;
    }
    /* The left board reaches its maximum projection between these headings. */
    int32_t const board = (p_hug->side < 0) ? (int32_t)geom_isqrt((uint32_t)(HALF_LEFT_MM * HALF_LEFT_MM +
                                      CHASSIS_BARCODE_FORWARD_MM * CHASSIS_BARCODE_FORWARD_MM)) + 1 : HALF_RIGHT_MM;
    return ((board > reach) ? board : reach) + SPIN_GAP_MM;
}

static int32_t side_gap(hug_t const * p_hug)
{
    bypass_t const * const run = &p_hug->bypass;
    /* The stored outward target includes a full pivot allowance. */
    int32_t const edge = run->out_origin_mm + run->out_mm - SWING_MM - SPIN_GAP_MM;
    return bypass_axis_mm(p_hug, p_hug->side * RIGHT_ANGLE) - edge;
}

int32_t bypass_side_aim(hug_t const * p_hug)
{
    bypass_t const * const run = &p_hug->bypass;
    if (run->b_wall || !p_hug->b_approach_tape)
    {
        return SIDE_AIM_DECI;
    }
    int32_t const gap = side_gap(p_hug);
    if (gap >= bypass_side_reach(p_hug, SIDE_AIM_DECI))
    {
        return SIDE_AIM_DECI;
    }
    return (gap >= bypass_side_reach(p_hug, OBSTACLE_SERVO_LIMIT_DECI)) ? OBSTACLE_SERVO_LIMIT_DECI : 0;
}

void bypass_start_side_stations(hug_t * p_hug, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    int32_t const lateral = bypass_axis_mm(p_hug, p_hug->side * RIGHT_ANGLE);
    int32_t const front = run->b_anchor ? run->contact_x_mm : p_hug->seen_mm;
    int32_t const aim = bypass_side_aim(p_hug);
    if (aim == 0)
    {
        bypass_end_route(p_hug, p_out, false);
        return;
    }
    int32_t const first = front - geom_mul_q14(lateral, geom_tan_q14(RIGHT_ANGLE - aim)) - bypass_axis_mm(p_hug, 0);
    bypass_enter(p_hug, BYPASS_SIDE_STEP, 0, clamp_i32(first, MIN_MOVE_MM, TRAVEL_LIMIT_MM));
}

static bool far_end(hug_t * p_hug, look_t const * p_look, unsigned min_clear)
{
    bypass_t * const run = &p_hug->bypass;
    unsigned hits = 0U;
    unsigned clear_after = 0U;
    int32_t first_x = 0;
    int32_t previous_x = 0;
    int32_t previous_y = 0;
    int32_t previous_angle = 0;
    uint16_t previous_range = 0U;
    look_edge_t candidate = {0};
    int16_t candidate_angle = 0;
    bool bracket = false;
    int32_t const front = run->b_anchor ? run->contact_x_mm : p_hug->seen_mm;
    for (unsigned i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const r = &p_look->rays[i];
        int32_t const a = look_ray_angle_deci(r);
        int32_t const x = r->x_mm + geom_mul_q14(r->range_mm, geom_cos_q14(a));
        int32_t const y = r->y_mm + geom_mul_q14(r->range_mm, geom_sin_q14(a));
        bool const adjacent = geom_abs(geom_wrap_deci(a - previous_angle) - p_hug->side * RAY_STEP) <= AIM_TOL;
        bool const past = (look_ray_echo(r) == LOOK_ECHO_OPEN) ||
            (look_ray_echo(r) == LOOK_ECHO_VALID && r->range_mm >= previous_range + LOOK_EDGE_STEP_MM &&
             p_hug->side * (previous_y - y) > LOOK_FACE_MATCH_MM);
        if (hits >= SIDE_MIN_HITS && adjacent && past && previous_x - first_x >= SIDE_MIN_SPAN_MM && previous_x > front)
        {
            candidate.x_mm = previous_x;
            candidate.y_mm = previous_y;
            candidate_angle = (int16_t)previous_angle;
            bracket = true;
            clear_after = 1U;
        }
        else if (bracket && adjacent && ((look_ray_echo(r) == LOOK_ECHO_OPEN) ||
                 (look_ray_echo(r) == LOOK_ECHO_VALID && (p_hug->side * (candidate.y_mm - y) > LOOK_FACE_MATCH_MM))))
        {
            clear_after++;
        }
        else if (bracket)
        {
            bracket = false;
            clear_after = 0U;
        }
        /* Keep background returns and the original front plane out of the side fit,
         * while allowing curved boundaries on round obstacles. */
        bool const on_side = look_ray_echo(r) == LOOK_ECHO_VALID && x >= front - LOOK_FACE_MATCH_MM &&
            p_hug->side * (r->y_mm - y) > 0 && r->range_mm <= HUG_STANDOFF_MM &&
            (!bracket || p_hug->side * (candidate.y_mm - y) <= LOOK_FACE_MATCH_MM);
        if (on_side)
        {
            bracket = false;
            clear_after = 0U;
            hits = adjacent && x >= previous_x && x - previous_x <= LOOK_EDGE_STEP_MM &&
                geom_abs(y - previous_y) <= LOOK_EDGE_STEP_MM ? hits + 1U : 1U;
            if (hits == 1U)
            {
                first_x = x;
            }
            run->b_saw_side = run->b_saw_side || (hits >= SIDE_MIN_HITS && x - first_x >= SIDE_MIN_SPAN_MM);
        }
        else
        {
            hits = 0U;
        }
        previous_angle = a;
        previous_range = r->range_mm;
        previous_x = x;
        previous_y = y;
    }
    if (bracket && (clear_after >= min_clear))
    {
        p_hug->side_end_x_mm = (int16_t)candidate.x_mm;
        p_hug->side_end_y_mm = (int16_t)candidate.y_mm;
        run->last_hit_deci = candidate_angle;
        return true;
    }
    return false;
}

static bool side_fan_clear(look_t const * p_look)
{
    if (p_look->count != SLOT_COUNT)
    {
        return false;
    }
    for (uint8_t i = 0U; i < p_look->count; i++)
    {
        look_ray_t const * const p_ray = &p_look->rays[i];
        if ((look_ray_echo(p_ray) != LOOK_ECHO_OPEN) &&
            ((look_ray_echo(p_ray) != LOOK_ECHO_VALID) || (p_ray->range_mm <= HUG_STANDOFF_MM)))
        {
            return false;
        }
    }
    return true;
}

void bypass_side_fan(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    int32_t const aim = p_hug->goal_head;
    int32_t const tol = run->b_look_ready ? AIM_HOLD_TOL : AIM_TOL;
    if (run->b_look_ready && p_in->now_ms - run->look_start_ms >= HUG_STATIONARY_LIMIT_MS)
    {
        bypass_end_route(p_hug, p_out, false);
        return;
    }
    if (!bypass_turn_within(p_hug, aim, tol, p_out) || !p_in->b_stationary)
    {
        run->b_pending = false;
        run->b_survey_timed = false;
        run->ray = 0U;
        look_clear(p_look);
        p_out->b_scan_pivot = (p_out->turn_deci_s != 0);
        return;
    }
    int32_t const gaze = aim + p_hug->side * ((int32_t)run->ray * RAY_STEP - OBSTACLE_SERVO_LIMIT_DECI);
    if (!bypass_ray(p_hug, p_in, aim, gaze, p_out))
    {
        if (run->ray == 0U && !run->b_look_ready)
        {
            run->look_start_ms = p_in->now_ms;
            run->b_look_ready = true;
        }
        return;
    }
    look_pose_t const pose = hug_axle_pose(p_hug);
    (void)look_add(p_look, &pose, p_in->ping_servo_deci, p_in->ping_echo, p_in->ping_mm);
    bool const b_done = (++run->ray >= SLOT_COUNT);
    bool const b_clear = b_done && side_fan_clear(p_look);
    int32_t const x = bypass_axis_mm(p_hug, 0);
    int32_t const front = run->b_anchor ? run->contact_x_mm : p_hug->seen_mm;
    bool const had_side = run->b_saw_side;
    run->b_saw_side = false;
    if (far_end(p_hug, p_look, b_done ? 1U : SIDE_CLEAR_CONFIRM))
    {
        int32_t const far_x = p_hug->side_end_x_mm;
        p_hug->breadth_mm = (uint16_t)(far_x - front);
        int32_t const ahead = far_x + HALF_LEFT_MM + SPIN_GAP_MM - x;
        bypass_enter(p_hug, BYPASS_LEVEL, 0, clamp_i32(ahead, MIN_MOVE_MM, TRAVEL_LIMIT_MM));
    }
    else if (!b_done)
    {
        run->b_saw_side = had_side;
    }
    else if (p_look->count != SLOT_COUNT)
    {
        if (run->b_retried)
        {
            bypass_end_route(p_hug, p_out, false);
        }
        else
        {
            run->b_retried = true;
            run->b_saw_side = had_side || run->b_saw_side;
            run->b_look_ready = false;
            look_clear(p_look);
            bypass_enter(p_hug, BYPASS_SIDE_FAN, aim, 0);
        }
    }
    else if (b_clear && (aim != (-p_hug->side * RIGHT_ANGLE)) &&
             (run->b_wall || !p_hug->b_approach_tape || (side_gap(p_hug) >= bypass_side_reach(p_hug, RIGHT_ANGLE))))
    {
        /* A round side can slip behind the oblique fan between stations.
         * Check broadside before deciding the obstacle has ended. */
        look_clear(p_look);
        run->b_look_ready = false;
        run->b_saw_side = had_side;
        bypass_enter(p_hug, BYPASS_SIDE_FAN, -p_hug->side * RIGHT_ANGLE, 0);
    }
    else if (b_clear && had_side && (aim == -p_hug->side * RIGHT_ANGLE))
    {
        /* The clear station bounds the geometry for the next leg. Only a
         * bracketed return supplies a measured breadth. */
        p_hug->side_end_x_mm = (int16_t)x;
        bypass_enter(p_hug, BYPASS_LEVEL, 0, clamp_i32(HALF_LEFT_MM + SPIN_GAP_MM, MIN_MOVE_MM, TRAVEL_LIMIT_MM));
    }
    else if (x + MIN_MOVE_MM > TRAVEL_LIMIT_MM ||
             (!run->b_saw_side && (run->b_retried ||
               (!had_side && x > front + TRACK_STEP))))
    {
        bypass_end_route(p_hug, p_out, false);
    }
    else
    {
        int32_t const step = run->b_saw_side ? SIDE_STEP_MM : MIN_MOVE_MM;
        run->b_retried = had_side && !run->b_saw_side;
        run->b_saw_side = had_side || run->b_saw_side;
        bypass_enter(p_hug, BYPASS_SIDE_STEP, 0, clamp_i32(step, MIN_MOVE_MM, TRAVEL_LIMIT_MM - x));
    }
}
