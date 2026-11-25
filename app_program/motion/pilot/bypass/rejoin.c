/** @file rejoin.c
 * @brief Bypass tape detection, alignment and release.
 */

#include "internal.h"
#include "obstacle/obstacle.h"
#include "platform/units.h"
#include "ir/ir.h"

#define LANE_MARGIN_MM  (30)
#define LANE_EXTRA_MM   (100)
#define TRACK_TILT      (100)
#define BLACK_MIN_MM    (8)
#define CENTRE_MM       (127)
#define ASSUMED_ANGLE   (300)
STATIC_ASSERT(OBSTACLE_SERVO_LIMIT_DECI >= TRACK_TILT, "two return scan gazes fit servo");

void bypass_lane_ray(hug_t * p_hug, hug_in_t const * p_in, int32_t base, bool b_track)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (!bypass_is_point(p_in))
    {
        if ((LOOK_ECHO_OPEN != p_in->ping_echo) && (p_run->unknown < UINT8_MAX))
        {
            p_run->unknown++;
        }
        return;
    }
    int32_t const psi = hug_pose_heading_deci(p_hug) - base;
    int32_t const gaze = psi + p_in->ping_servo_deci;
    int32_t const beam = SONAR_BOARD_MM + p_in->ping_mm;
    int32_t const fwd = geom_mul_q14(SHAFT_MM, geom_cos_q14(psi)) + geom_mul_q14(beam, geom_cos_q14(gaze));
    int32_t const side = geom_mul_q14(SHAFT_MM, geom_sin_q14(psi)) + geom_mul_q14(beam, geom_sin_q14(gaze));
    bool const b_inside = (fwd > 0) && (fwd < p_hug->side * p_hug->y_um / UM_PER_MM + LANE_EXTRA_MM) &&
        (side < HALF_LEFT_MM + LANE_MARGIN_MM) && (side > -HALF_RIGHT_MM - LANE_MARGIN_MM);
    p_run->b_swing_blocked = p_run->b_swing_blocked || ((fwd * fwd + side * side < SWING_MM * SWING_MM));
    p_run->b_blocked = p_run->b_blocked || (b_inside);
    if (b_inside)
    {
        if (p_in->ping_mm < p_run->nearest_mm)
        {
            p_run->nearest_mm = p_in->ping_mm;
        }
        int32_t const alpha = p_hug->side * gaze;
        if (alpha > p_run->last_hit_deci)
        {
            p_run->last_hit_deci = (int16_t)alpha;
        }
        if (b_track)
        {
            int32_t const gap = (side >= 0) ? HALF_LEFT_MM + LANE_MARGIN_MM - side :
                                             HALF_RIGHT_MM + LANE_MARGIN_MM + side;
            if (gap > p_run->move_mm)
            {
                p_run->move_mm = (int16_t)gap;
            }
        }
    }
}

void bypass_move_on(hug_t * p_hug, bool b_sweep)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (b_sweep && p_run->b_blocked)
    {
        int32_t const angle = (p_run->last_hit_deci < 0) ? 0 :
            ((p_run->last_hit_deci + RAY_STEP > FULL_PAN) ? ASSUMED_ANGLE : p_run->last_hit_deci + RAY_STEP);
        int32_t const extent = geom_mul_q14(p_run->nearest_mm, geom_tan_q14(angle));
        int32_t const half = (p_hug->side > 0) ? HALF_RIGHT_MM : HALF_LEFT_MM;
        p_run->move_mm = (int16_t)((extent + half + SPIN_GAP_MM) / HALF_DIVISOR);
    }
    p_run->move_mm = (int16_t)clamp_i32(p_run->move_mm, MIN_MOVE_MM, TRAVEL_LIMIT_MM);
    bypass_enter(p_hug, BYPASS_BACK, 0, 0);
}

/* Check the lane a second time with the sensor shaft at different positions.
 * Sample and hold the two gazes at achievable counter-steer angles. */
void bypass_track(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const base = -p_hug->side * RIGHT_ANGLE;
    bool const b_out = (BYPASS_TRACK_OUT == p_run->phase);
    int32_t const endpoint = OBSTACLE_SERVO_LIMIT_DECI;
    int32_t const rotated = b_out ? p_run->ray * TRACK_STEP : endpoint - p_run->ray * TRACK_STEP;
    int32_t const aim = base - p_hug->side * rotated;
    int32_t const gaze = base - (b_out ? 0 : p_hug->side * TRACK_TILT);
    if (!bypass_ray(p_hug, p_in, aim, gaze, p_out))
    {
        return;
    }
    bypass_lane_ray(p_hug, p_in, base, true);
    if (p_run->b_blocked || p_run->b_swing_blocked)
    {
        bypass_move_on(p_hug, false);
    }
    else if (++p_run->ray > (unsigned)(endpoint / TRACK_STEP))
    {
        if (b_out)
        {
            bypass_enter(p_hug, BYPASS_TRACK_BACK, base, 0);
        }
        else if (p_run->unknown && !p_run->b_retried)
        {
            p_run->b_retried = true;
            bypass_enter(p_hug, BYPASS_TRACK_OUT, base, 0);
        }
        else if (p_run->unknown)
        {
            bypass_move_on(p_hug, false);
        }
        else
        {
            p_run->return_origin_mm = (int16_t)bypass_axis_mm(p_hug, base);
            bypass_enter(p_hug, BYPASS_TO_LINE, base, TRAVEL_LIMIT_MM);
        }
    }
}

void bypass_to_line(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    bool const b_black = p_in->b_line_left || p_in->b_line_right;
    int32_t const axis = bypass_axis_mm(p_hug, p_hug->goal_head);
    int32_t const ir = p_hug->side * (p_hug->y_um / UM_PER_MM +
        geom_mul_q14(LINE_SENSOR_LEAD_MM, geom_sin_q14(hug_pose_heading_deci(p_hug))));
    if (ir < -HUG_TAPE_TOL_MM)
    {
        /* Stop looking across the course before reaching neighbouring tape. */
        int32_t const to_axis = p_hug->side * p_hug->y_um / UM_PER_MM;
        bypass_enter(p_hug, BYPASS_CENTRE, p_hug->goal_head, clamp_i32(to_axis, 0, TRAVEL_LIMIT_MM));
    }
    else if (!b_black)
    {
        p_run->b_white = true;
        p_run->b_black = false;
    }
    else if (ir > HUG_TAPE_TOL_MM)
    {
        /* A stripe entered at the neighbouring route cannot become a new
         * crossing by staying dark all the way into the original lane. */
        p_run->b_white = false;
        p_run->b_black = false;
    }
    else if (p_run->b_white && !p_run->b_black)
    {
        p_run->b_black = true;
        p_run->black_mm = (int16_t)axis;
    }
    else if (p_run->b_black && (axis - p_run->black_mm >= BLACK_MIN_MM) && (geom_abs(ir) <= HUG_TAPE_TOL_MM))
    {
        p_run->b_exit_tape = true;
        bypass_enter(p_hug, BYPASS_CENTRE, p_hug->goal_head, CENTRE_MM);
    }
    if (p_run->phase == BYPASS_TO_LINE)
    {
        (void)bypass_moved(p_hug, p_in, p_out, SLOW_MM_S);
    }
}

void bypass_map_line(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const swept = (int32_t)wheel_yaw_deci(p_hug->diff_um - p_run->map_diff_um) * p_hug->side;
    bool const b_black = p_in->b_line_left || p_in->b_line_right;
    int32_t const ir = p_hug->y_um / UM_PER_MM + geom_mul_q14(LINE_SENSOR_LEAD_MM, geom_sin_q14(hug_pose_heading_deci(p_hug)));
    if (b_black && p_run->b_white && (geom_abs(ir) < HUG_PARALLEL_SPACING_MM - HUG_TAPE_TOL_MM))
    {
        bypass_end_route(p_hug, p_out, true);
    }
    else if (swept >= MAP_LIMIT_DECI)
    {
        bypass_enter(p_hug, BYPASS_EXIT_AHEAD, 0, 0);
    }
    else
    {
        p_run->b_white = p_run->b_white || (!b_black);
        p_out->turn_deci_s = (int16_t)(p_hug->side * SPIN_SLOW_DECI_S);
    }
}

void bypass_exit_ahead(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    if (!bypass_ray(p_hug, p_in, 0, 0, p_out))
    {
        return;
    }
    if ((p_in->ping_echo == LOOK_ECHO_OPEN) ||
        ((p_in->ping_echo == LOOK_ECHO_VALID) && (p_in->ping_mm > HUG_STANDOFF_MM)))
    {
        /* Release on the original heading without an optical rejoin. */
        p_hug->state = 0U;
        p_hug->hist = 0U;
        p_hug->bypass.phase = BYPASS_IDLE;
    }
    else if ((LOOK_ECHO_TOO_CLOSE == p_in->ping_echo) || bypass_is_point(p_in))
    {
        uint16_t const range = (LOOK_ECHO_TOO_CLOSE == p_in->ping_echo) ?
            OBSTACLE_PING_TOO_CLOSE_MM : p_in->ping_mm;
        if (p_hug->b_fan_pending &&
            !bypass_moved_usefully(p_in, p_hug->anchor.fan_pose.left_um, p_hug->anchor.fan_pose.right_um))
        {
            bypass_end_route(p_hug, p_out, false);
            return;
        }
        hug_forget_approach(p_hug);
        hug_begin_bypass(p_hug, p_look, p_in, range);
    }
}
