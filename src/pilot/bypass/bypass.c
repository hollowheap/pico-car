/** @file bypass.c
 * @brief Bypass leg states and wheel progress in the contact frame.
 */

#include "internal.h"
#include "geometry.h"
#include "pilot/pilot.h"
#include "platform/units.h"
#include "track/line_control.h"

#define PIVOT_WHEEL_MM_S (100)
#define SPIN_DECI_S     ((2 * PIVOT_WHEEL_MM_S * DECI_PER_RAD) / CHASSIS_WHEEL_TRACK_MM)
#define FORWARD_AGE_MS  (400U)
#define LOOK_FIRST_STEP_MM (SWING_MM / HALF_DIVISOR)

/* Reserve the entire optical sweep while the axle turns at a lateral station. */
#define READER_RADIUS_MM (LINE_SENSOR_LEAD_MM + 1)
STATIC_ASSERT(READER_RADIUS_MM * READER_RADIUS_MM >= LINE_SENSOR_LEAD_MM * LINE_SENSOR_LEAD_MM +
              ((LINE_SENSOR_SPACING_MM + 1) / 2) * ((LINE_SENSOR_SPACING_MM + 1) / 2), "reader pivot radius");
#define GUIDED_LATERAL_MM (HUG_PARALLEL_SPACING_MM - (LINE_WIDTH_MM + 1) / 2 - READER_RADIUS_MM - SPIN_GAP_MM)
#define MS_PER_SECOND   (1000U)

/* Keep Buddy 5's larger turning allowance. Check each measured station
 * around the axle, including the asymmetric barcode board behind the sonar. */
STATIC_ASSERT(SWING_MM * SWING_MM >= CHASSIS_BARCODE_FORWARD_MM * CHASSIS_BARCODE_FORWARD_MM +
              HALF_LEFT_MM * HALF_LEFT_MM, "pivot clears barcode overhang");
STATIC_ASSERT(SWING_MM * SWING_MM >= CHASSIS_FRONT_TIP_MM * CHASSIS_FRONT_TIP_MM +
              ((CHASSIS_IR_WIDTH_MM + 1) / 2) * ((CHASSIS_IR_WIDTH_MM + 1) / 2), "pivot clears IR tips");
STATIC_ASSERT(SWING_MM * SWING_MM >= CHASSIS_NOSE_FORWARD_MM * CHASSIS_NOSE_FORWARD_MM +
              CHASSIS_NOSE_LEFT_MM * CHASSIS_NOSE_LEFT_MM, "pivot clears nose");
STATIC_ASSERT(CHASSIS_NOSE_LEFT_MM >= CHASSIS_NOSE_WIDTH_MM / 2, "left side covers nominal nose");

bool bypass_pose_moved(int32_t dl_um, int32_t dr_um)
{
    return (geom_abs(dl_um) >= PILOT_MOVE_MIN_UM) && (geom_abs(dr_um) >= PILOT_MOVE_MIN_UM);
}

bool bypass_moved_usefully(hug_in_t const * p_in, uint32_t left_um, uint32_t right_um)
{
    int32_t const dl = (int32_t)((uint32_t)p_in->left_um - left_um);
    int32_t const dr = (int32_t)((uint32_t)p_in->right_um - right_um);
    return bypass_pose_moved(dl, dr);
}

void bypass_enter(hug_t * p_hug, bypass_phase_t phase, int32_t aim, int32_t distance)
{
    bypass_t * const p_run = &p_hug->bypass;
    p_run->phase = (uint8_t)phase;
    p_run->b_motor = false;
    p_hug->goal_head = (int16_t)aim;
    p_run->axis_start_mm = (int16_t)bypass_axis_mm(p_hug, aim);
    p_run->target_mm = (int16_t)distance;
    p_run->b_clear = false;
    p_run->blocked_pings = 0U;
    p_run->b_drive_aligned = false;
    p_run->b_pending = false;
    p_run->ray = 0U;
    p_run->b_survey_timed = false;
    p_run->sector = 0U;
    p_run->b_white = false;
    p_run->b_black = false;
    p_run->b_fan_repeating = false;
    p_run->b_fan_pose_set = false;
    if ((BYPASS_CONTACT == phase) || (BYPASS_SWEEP == phase) || (BYPASS_TRACK_OUT == phase))
    {
        bypass_reset_contact_fan(p_run);
        p_run->b_fan_done = false;
        p_run->b_fan_repeated = false;
    }
    if (BYPASS_MAP == phase)
    {
        p_run->map_diff_um = p_hug->diff_um;
    }
}

void bypass_begin(hug_t * p_hug, hug_in_t const * p_in, uint16_t range_mm)
{
    p_hug->bypass = (bypass_t){.seq = p_in->ping_seq, .last_forward = p_in->ping_seq,
                                .contact_mm = range_mm};
    p_hug->seen_mm = (int16_t)(SHAFT_MM + range_mm);
    bypass_enter(p_hug, BYPASS_CONTACT, 0, 0);
}

bool bypass_turn_within(hug_t const * p_hug, int32_t aim, int32_t tol, hug_out_t * p_out)
{
    int32_t const err = geom_wrap_deci(aim - hug_pose_heading_deci(p_hug));
    int32_t const rate = (geom_abs(err) <= TRACK_STEP) ? SPIN_SLOW_DECI_S : SPIN_DECI_S;
    if (geom_abs(err) <= tol)
    {
        return true;
    }
    p_out->turn_deci_s = (int16_t)((err > 0) ? rate : -rate);
    return false;
}

static bool turn(hug_t const * p_hug, int32_t aim, hug_out_t * p_out)
{
    return bypass_turn_within(p_hug, aim, AIM_TOL, p_out);
}

bool bypass_is_point(hug_in_t const * p_in)
{
    return look_echo_is_point(p_in->ping_echo) && (0U != p_in->ping_mm);
}

void bypass_end_route(hug_t * p_hug, hug_out_t * p_out, bool b_rejoined)
{
    p_hug->bypass.phase = BYPASS_IDLE;
    p_hug->state = 0U;
    p_hug->hist = 0U;
    p_out->b_rejoined = b_rejoined;
    p_out->b_no_exit = !b_rejoined;
}

static bool observe_clearance(hug_t * p_hug, hug_in_t const * p_in, int32_t remaining)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (!p_in->b_ping_reply && (p_in->ping_servo_deci == 0) && (p_run->last_forward != p_in->ping_seq))
    {
        if (p_in->ping_seq != ((p_run->last_forward + 1U) & OBSTACLE_PING_SEQ_MK))
        {
            p_run->blocked_pings = 0U;
        }
        p_run->last_forward = p_in->ping_seq;
        p_run->clearance_ms = p_in->now_ms;
        /* A fresh centre echo must leave room to stop. AHEAD terminates
         * at the planned spin clearance even when the obstacle stays visible. */
        int32_t const guard = (remaining < SPIN_GAP_MM) ? remaining : SPIN_GAP_MM;
        p_run->b_clear = (LOOK_ECHO_OPEN == p_in->ping_echo) ||
            ((LOOK_ECHO_VALID == p_in->ping_echo) && ((int32_t)p_in->ping_mm > FRONT_RESERVE_MM + guard));
        /* The limit is in the same leg-relative coordinates as remaining.
         * An OPEN reply imposes only the ordinary freshness limit. */
        p_run->clear_until_remaining_mm = (LOOK_ECHO_VALID == p_in->ping_echo) ?
            (int16_t)clamp_i32(remaining - (int32_t)p_in->ping_mm + FRONT_RESERVE_MM,
                               -1, INT16_MAX) : -1;
        if (p_run->b_clear || (!bypass_is_point(p_in) && (p_in->ping_echo != LOOK_ECHO_TOO_CLOSE)))
        {
            p_run->blocked_pings = 0U;
        }
        else if (p_run->blocked_pings < HUG_TRIGGER_HITS)
        {
            p_run->blocked_pings++;
        }
    }
    return p_run->b_clear && (remaining > p_run->clear_until_remaining_mm) && (p_in->now_ms - p_run->clearance_ms <= FORWARD_AGE_MS);
}

static void blocked_leg(hug_t * p_hug, hug_in_t const * p_in)
{
    bypass_t * const p_run = &p_hug->bypass;
    switch (p_run->phase)
    {
    case BYPASS_AHEAD:
        /* An object closed on the straight approach. Recompute the signed
         * spin clearance from the current axle rather than driving into it. */
        p_run->b_wide = true;
        p_run->contact_mm = (p_in->ping_echo == LOOK_ECHO_TOO_CLOSE) ?
                            OBSTACLE_PING_TOO_CLOSE_MM : p_in->ping_mm;
        p_run->b_anchor = false;
        bypass_enter(p_hug, BYPASS_AHEAD, 0, (int32_t)p_run->contact_mm + SHAFT_MM - SWING_MM - SPIN_GAP_MM);
        break;
    case BYPASS_OUT:
        p_run->failed_sides |= (uint8_t)((p_hug->side > 0) ? LEFT_BLOCKED : RIGHT_BLOCKED);
        bypass_enter(p_hug, BYPASS_RETRACE, p_hug->goal_head,
              -clamp_i32(bypass_axis_mm(p_hug, p_hug->goal_head) - p_run->out_origin_mm, 0, TRAVEL_LIMIT_MM));
        break;
    case BYPASS_SIDE_STEP:
    case BYPASS_LEVEL:
        p_run->b_wall = true;
        p_run->b_wall_margin = false;
        bypass_enter(p_hug, BYPASS_WALL_BACK, 0,
              -clamp_i32(bypass_axis_mm(p_hug, 0) - p_run->parallel_origin_mm, 0, TRAVEL_LIMIT_MM));
        break;
    case BYPASS_MOVE:
        /* The inward turn was planned at the far end. A new object in its
         * swing circle invalidates that pivot. Return along the travelled leg. */
        bypass_enter(p_hug, BYPASS_RETRACE, 0,
              -clamp_i32(bypass_axis_mm(p_hug, 0) - p_run->parallel_origin_mm, 0, TRAVEL_LIMIT_MM));
        break;
    case BYPASS_TO_LINE:
    case BYPASS_CENTRE:
        p_run->move_mm = MIN_MOVE_MM;
        bypass_enter(p_hug, BYPASS_RETRACE, p_hug->goal_head,
              -clamp_i32(bypass_axis_mm(p_hug, p_hug->goal_head) - p_run->return_origin_mm, 0, TRAVEL_LIMIT_MM));
        break;
    default:
        break;
    }
}

bool bypass_moved(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out, int32_t speed)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const travel = bypass_axis_mm(p_hug, p_hug->goal_head) - p_run->axis_start_mm;
    int32_t const direction = (p_run->target_mm < 0) ? -1 : 1;
    int32_t const remaining = direction * (p_run->target_mm - travel);
    if (remaining <= 0)
    {
        return true;
    }
    int32_t const err = geom_wrap_deci(p_hug->goal_head - hug_pose_heading_deci(p_hug));
    if (direction > 0 && (!p_run->b_drive_aligned || geom_abs(err) > RAY_STEP))
    {
        p_run->b_clear = false;
        p_run->b_drive_aligned = turn(p_hug, p_hug->goal_head, p_out) && p_in->b_stationary;
        p_run->last_forward = p_in->ping_seq;
        return false;
    }
    if ((direction < 0) || observe_clearance(p_hug, p_in, remaining))
    {
        p_run->b_survey_timed = false;
        p_out->speed_mm_s = (int16_t)(direction * speed);
        p_out->turn_deci_s = (geom_abs(err) <= HUG_HEAD_TOL_DECI) ? 0 :
            (int16_t)clamp_i32(err * HUG_HOLD_GAIN, -HUG_HOLD_MAX_DECI_S, HUG_HOLD_MAX_DECI_S);
    }
    else
    {
        p_out->b_stop = 1U;
        if (p_run->blocked_pings >= HUG_TRIGGER_HITS)
        {
            blocked_leg(p_hug, p_in);
        }
        else if (!p_run->b_survey_timed)
        {
            p_run->survey_ms = p_in->now_ms;
            p_run->b_survey_timed = true;
        }
        else if (p_in->now_ms - p_run->survey_ms >= HUG_STATIONARY_LIMIT_MS)
        {
            bypass_end_route(p_hug, p_out, false);
        }
    }
    return false;
}

static void approach(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const away = p_hug->side * RIGHT_ANGLE;
    if (bypass_moved(p_hug, p_in, p_out, (p_run->target_mm < 0) ? SLOW_MM_S : SPEED_MM_S))
    {
        if (p_run->b_wide)
        {
            p_run->b_wide = false;
            p_run->contact_mm = (uint16_t)((int32_t)p_run->contact_mm - p_run->target_mm);
            bypass_enter(p_hug, BYPASS_CONTACT, 0, 0);
            look_clear(p_look);
        }
        else
        {
            bypass_enter(p_hug, BYPASS_AWAY, away, 0);
        }
    }
}

static int32_t outward_target(hug_t const * p_hug)
{
    bypass_t const * const run = &p_hug->bypass;
    int32_t const room = p_hug->b_approach_tape ? GUIDED_LATERAL_MM - run->out_origin_mm : TRAVEL_LIMIT_MM;
    return clamp_i32(run->out_mm, 0, clamp_i32(room, 0, TRAVEL_LIMIT_MM));
}

static void outward(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    if (bypass_moved(p_hug, p_in, p_out, SPEED_MM_S))
    {
        int32_t const lateral = bypass_axis_mm(p_hug, p_hug->side * RIGHT_ANGLE) - run->out_origin_mm;
        if ((lateral < outward_target(p_hug)) && (run->contact_x_mm - bypass_axis_mm(p_hug, 0) < SWING_MM))
        {
            /* Coasting beyond the planned approach can put the front face
             * inside the pivot circle. Finish the straight departure first. */
            run->views = LOOK_VIEWS;
            bypass_enter(p_hug, BYPASS_AWAY, p_hug->side * RIGHT_ANGLE, 0);
        }
        else if ((run->views < LOOK_VIEWS) && (lateral < outward_target(p_hug)))
        {
            run->views++;
            run->look_start_ms = p_in->now_ms;
            run->b_look_ready = false;
            run->last_forward = p_in->ping_seq;
            bypass_enter(p_hug, BYPASS_LOOK_BACK, p_hug->side * RIGHT_ANGLE, 0);
            look_clear(p_look);
        }
        else
        {
            run->parallel_origin_mm = (int16_t)bypass_axis_mm(p_hug, 0);
            bypass_enter(p_hug, BYPASS_PARALLEL, 0, 0);
        }
    }
}

static void retrace(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const away = p_hug->side * RIGHT_ANGLE;
    switch (p_run->phase)
    {
    case BYPASS_RETRACE:
        if (bypass_moved(p_hug, p_in, p_out, SLOW_MM_S))
        {
            if (p_hug->goal_head == 0)
            {
                /* The parallel leg returned to its own origin. Retrace the
                 * outward leg next before restarting the contact survey. */
                p_run->failed_sides |= (uint8_t)((p_hug->side > 0) ? LEFT_BLOCKED : RIGHT_BLOCKED);
                bypass_enter(p_hug, BYPASS_RESCAN, away, 0);
            }
            else if (p_hug->goal_head == -away)
            {
                bypass_enter(p_hug, BYPASS_BACK, 0, 0);
            }
            else
            {
                bypass_enter(p_hug, BYPASS_RESCAN, 0, 0);
            }
        }
        break;
    case BYPASS_RESCAN:
        if (turn(p_hug, p_hug->goal_head, p_out))
        {
            if (p_hug->goal_head != 0)
            {
                bypass_enter(p_hug, BYPASS_RETRACE, away, -clamp_i32(bypass_axis_mm(p_hug, away) - p_run->out_origin_mm, 0, TRAVEL_LIMIT_MM));
            }
            else
            {
                p_run->contact_mm = HUG_STANDOFF_MM;
                p_run->b_anchor = false;
                bypass_enter(p_hug, BYPASS_CONTACT, 0, 0);
                look_clear(p_look);
            }
        }
        break;
    default:
        break;
    }
}

/* Keep the contact frame across steps so crossing adjacent tape cannot move
 * the return axis. Each outward move still needs a new centered clearance. */
static void retreat_side(hug_t * p_hug)
{
    bypass_enter(p_hug, BYPASS_RETREAT_ALIGN, 0, 0);
}

static void wall_step(hug_t * p_hug, hug_out_t * p_out, int32_t step)
{
    bypass_t * const run = &p_hug->bypass;
    int32_t const away = p_hug->side * RIGHT_ANGLE;
    int32_t const next = bypass_axis_mm(p_hug, away) - run->out_origin_mm + step;
    if (p_hug->b_approach_tape && ((next + run->out_origin_mm > GUIDED_LATERAL_MM) &&
        (run->b_wall_margin || bypass_axis_mm(p_hug, away) >= GUIDED_LATERAL_MM - 1)))
    {
        /* A partial outward step cannot establish the full body-width margin. */
        retreat_side(p_hug);
    }
    else if (next > TRAVEL_LIMIT_MM)
    {
        bypass_end_route(p_hug, p_out, false);
    }
    else
    {
        run->out_mm = (int16_t)next;
        run->views = LOOK_VIEWS;
        bypass_enter(p_hug, BYPASS_AWAY, away, 0);
    }
}

static void wall_check(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    if (p_hug->bypass.phase == BYPASS_WALL_BACK)
    {
        if (bypass_moved(p_hug, p_in, p_out, SLOW_MM_S) && p_in->b_stationary)
        {
            wall_step(p_hug, p_out, TRACK_STEP);
        }
        return;
    }
    if (!bypass_ray(p_hug, p_in, 0, 0, p_out))
    {
        return;
    }
    bypass_t * const run = &p_hug->bypass;
    bool const clear = (p_in->ping_echo == LOOK_ECHO_OPEN) ||
        (bypass_is_point(p_in) && ((int32_t)p_in->ping_mm > FRONT_RESERVE_MM + run->level_mm));
    if (clear && run->b_wall_margin)
    {
        bypass_start_side_stations(p_hug, p_out);
        run->last_forward = p_in->ping_seq;
    }
    else if (clear || bypass_is_point(p_in) || (p_in->ping_echo == LOOK_ECHO_TOO_CLOSE))
    {
        run->b_wall_margin = clear;
        wall_step(p_hug, p_out, clear ? bypass_side_reach(p_hug, SIDE_AIM_DECI) : TRACK_STEP);
    }
}

static void turn_outward(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    int32_t const away = p_hug->side * RIGHT_ANGLE;
    if (turn(p_hug, away, p_out))
    {
        int32_t const progress = bypass_axis_mm(p_hug, away) - run->out_origin_mm;
        int32_t const remaining = clamp_i32(outward_target(p_hug) - progress, 0, TRAVEL_LIMIT_MM);
        int32_t const step = (run->views == 0U) ? LOOK_FIRST_STEP_MM - progress : MIN_MOVE_MM;
        bypass_enter(p_hug, BYPASS_OUT, away, (run->views == LOOK_VIEWS) ? remaining : clamp_i32(step, 0, remaining));
        run->last_forward = p_in->ping_seq;
    }
}

static void turn_parallel(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    if (turn(p_hug, 0, p_out))
    {
        if (run->phase == BYPASS_BACK)
        {
            bypass_enter(p_hug, BYPASS_MOVE, 0, run->move_mm);
        }
        else if (run->b_wall)
        {
            bypass_enter(p_hug, BYPASS_WALL_CHECK, 0, run->level_mm);
        }
        else
        {
            bypass_start_side_stations(p_hug, p_out);
        }
        run->last_forward = p_in->ping_seq;
    }
}

static void approach_side_station(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const run = &p_hug->bypass;
    if (bypass_moved(p_hug, p_in, p_out, SLOW_MM_S))
    {
        int32_t const aim = bypass_side_aim(p_hug);
        look_clear(p_look);
        run->b_look_ready = false;
        if (aim == 0)
        {
            bypass_end_route(p_hug, p_out, false);
        }
        else
        {
            bypass_enter(p_hug, BYPASS_SIDE_FAN, -p_hug->side * aim, 0);
        }
    }
}

static void run_phase(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    int32_t const away = p_hug->side * RIGHT_ANGLE;
    switch (p_run->phase)
    {
    case BYPASS_CONTACT:
    case BYPASS_SWEEP:
        bypass_sweep(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_ALIGN:
        if (turn(p_hug, 0, p_out))
        {
            bypass_enter(p_hug, BYPASS_AHEAD, 0, p_run->ahead_mm);
            p_run->last_forward = p_in->ping_seq;
        }
        break;
    case BYPASS_AHEAD:
        approach(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_AWAY:
        turn_outward(p_hug, p_in, p_out);
        break;
    case BYPASS_OUT:
        outward(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_LOOK_BACK:
        bypass_look_back(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_BACK:
    case BYPASS_PARALLEL:
        turn_parallel(p_hug, p_in, p_out);
        break;
    case BYPASS_SIDE_STEP:
        approach_side_station(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_SIDE_FAN:
        bypass_side_fan(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_RETREAT_ALIGN:
        if (turn(p_hug, 0, p_out))
        {
            bypass_enter(p_hug, BYPASS_RETRACE, 0,
                  -clamp_i32(bypass_axis_mm(p_hug, 0) - p_run->parallel_origin_mm, 0, TRAVEL_LIMIT_MM));
        }
        break;
    case BYPASS_LEVEL:
    case BYPASS_MOVE:
        if (p_hug->x_um / UM_PER_MM > TRAVEL_LIMIT_MM)
        {
            bypass_end_route(p_hug, p_out, false);
        }
        else if (bypass_moved(p_hug, p_in, p_out, SPEED_MM_S))
        {
            bypass_enter(p_hug, BYPASS_FACE, -away, 0);
        }
        break;
    case BYPASS_FACE:
        if (turn(p_hug, -away, p_out))
        {
            p_run->b_retried = false;
            p_run->move_mm = MIN_MOVE_MM;
            bypass_enter(p_hug, BYPASS_SWEEP, -away, 0);
            look_clear(p_look);
        }
        break;
    case BYPASS_TRACK_OUT:
    case BYPASS_TRACK_BACK:
        bypass_track(p_hug, p_in, p_out);
        break;
    case BYPASS_TO_LINE:
        bypass_to_line(p_hug, p_in, p_out);
        break;
    case BYPASS_CENTRE:
        if (bypass_moved(p_hug, p_in, p_out, SLOW_MM_S))
        {
            bool const guide = p_hug->b_approach_tape || p_run->b_exit_tape;
            bypass_enter(p_hug, guide ? BYPASS_MAP : BYPASS_EXIT_AHEAD, guide ? -away : 0, 0);
        }
        break;
    case BYPASS_RETRACE:
    case BYPASS_RESCAN:
        retrace(p_hug, p_look, p_in, p_out);
        break;
    case BYPASS_MAP:
        bypass_map_line(p_hug, p_in, p_out);
        break;
    case BYPASS_WALL_BACK:
    case BYPASS_WALL_CHECK:
        wall_check(p_hug, p_in, p_out);
        break;
    case BYPASS_EXIT_AHEAD:
        bypass_exit_ahead(p_hug, p_look, p_in, p_out);
        break;
    default:
        break;
    }
}

static void set_progress_reference(hug_t * p_hug, hug_in_t const * p_in)
{
    p_hug->origin.survey.left_um = (uint32_t)p_in->left_um;
    p_hug->origin.survey.right_um = (uint32_t)p_in->right_um;
}

static void anchor_progress(hug_t * p_hug, hug_in_t const * p_in)
{
    set_progress_reference(p_hug, p_in);
    p_hug->last_motion_ms = p_in->now_ms;
}

static void observe_wheel_progress(hug_t * p_hug, hug_in_t const * p_in,
                                   bool b_left_reverse, bool b_right_reverse)
{
    int32_t const dl = (int32_t)((uint32_t)p_in->left_um - p_hug->origin.survey.left_um);
    int32_t const dr = (int32_t)((uint32_t)p_in->right_um - p_hug->origin.survey.right_um);
    bool const b_left = b_left_reverse ? dl <= -PILOT_MOVE_MIN_UM : dl >= PILOT_MOVE_MIN_UM;
    bool const b_right = b_right_reverse ? dr <= -PILOT_MOVE_MIN_UM : dr >= PILOT_MOVE_MIN_UM;
    if (b_left && b_right)
    {
        anchor_progress(p_hug, p_in);
        p_hug->bypass.b_stalled = false;
    }
}

static void observe_progress(hug_t * p_hug, hug_in_t const * p_in)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (p_run->b_scan_pause)
    {
        p_hug->last_motion_ms += p_in->now_ms - p_hug->t_ms;
    }
    if (p_run->b_motor)
    {
        observe_wheel_progress(p_hug, p_in, p_run->b_left_reverse, p_run->b_right_reverse);
    }
}

static bool settled_fan_running(bypass_t const * p_run, hug_in_t const * p_in)
{
    return p_in->b_stationary && p_run->b_survey_timed && !p_run->b_fan_done &&
        ((BYPASS_CONTACT == p_run->phase) || (BYPASS_SWEEP == p_run->phase) ||
         (BYPASS_TRACK_OUT == p_run->phase) || (BYPASS_TRACK_BACK == p_run->phase) ||
         (BYPASS_LOOK_BACK == p_run->phase) || (BYPASS_SIDE_FAN == p_run->phase));
}

static void resume_bypass(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (p_run->b_contact_moving)
    {
        observe_wheel_progress(p_hug, p_in, true, true);
        p_run->b_contact_moving = false;
    }
    set_progress_reference(p_hug, p_in);
    p_run->b_interrupted = false;
    p_run->last_forward = p_in->ping_seq;
    p_run->b_drive_aligned = false;
    if (BYPASS_CONTACT == p_run->phase)
    {
        if (bypass_fan_pose_moved_usefully(p_hug, p_in))
        {
            bypass_enter(p_hug, BYPASS_CONTACT, 0, 0);
            look_clear(p_look);
        }
        else
        {
            /* A pose that did not move must not restart the deadline or clear the completion latch. */
            p_run->b_pending = false;
            p_run->b_fan_repeating = false;
            if (p_run->b_survey_timed &&
                ((uint32_t)(p_in->now_ms - p_run->survey_ms) >= BYPASS_FAN_DEADLINE_MS))
            {
                p_run->b_fan_done = true;
            }
        }
        return;
    }
    p_run->b_survey_timed = false;
    if (p_run->phase == BYPASS_SIDE_FAN)
    {
        bypass_enter(p_hug, BYPASS_SIDE_FAN, p_hug->goal_head, 0);
        p_run->b_look_ready = false;
        look_clear(p_look);
    }
    else if (p_run->phase == BYPASS_LOOK_BACK)
    {
        bypass_enter(p_hug, BYPASS_AWAY, p_hug->side * RIGHT_ANGLE, 0);
        look_clear(p_look);
    }
    else if ((p_run->phase == BYPASS_SWEEP) ||
             (p_run->phase == BYPASS_TRACK_OUT) ||
             (p_run->phase == BYPASS_TRACK_BACK))
    {
        bypass_enter(p_hug, BYPASS_FACE, -p_hug->side * RIGHT_ANGLE, 0);
        look_clear(p_look);
    }
}

static void suspend_bypass(hug_t * p_hug, hug_in_t const * p_in)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (p_run->b_scan_pause)
    {
        p_hug->last_motion_ms += p_in->now_ms - p_hug->t_ms;
    }
    bool const b_contact_moving = p_run->b_progress_timed && p_in->b_contact_backoff &&
        !p_in->b_independent_hold;
    if (p_run->b_contact_moving && !p_in->b_independent_hold)
    {
        observe_wheel_progress(p_hug, p_in, true, true);
    }
    else if (b_contact_moving)
    {
        set_progress_reference(p_hug, p_in);
    }
    p_run->b_contact_moving = b_contact_moving;
    p_run->b_scan_pause = p_run->b_progress_timed && !p_run->b_probe && p_in->b_independent_hold;
    if (p_run->b_scan_pause)
    {
        p_hug->t_ms = p_in->now_ms;
    }
    p_run->b_interrupted = true;
    p_run->b_motor = false;
    p_run->b_pending = false;
    p_run->b_clear = false;
}

void bypass_step(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out)
{
    bypass_t * const p_run = &p_hug->bypass;
    if (p_in->b_other)
    {
        suspend_bypass(p_hug, p_in);
        return;
    }
    if (p_run->b_interrupted)
    {
        resume_bypass(p_hug, p_look, p_in);
    }
    if (p_run->b_probe)
    {
        if ((int32_t)(p_in->now_ms - p_hug->t_ms) >= 0)
        {
            p_hug->state = 0U;
            p_run->phase = BYPASS_IDLE;
            p_out->b_timeout = 1U;
            return;
        }
        int32_t const aim = p_run->requested_head;
        if (bypass_ray(p_hug, p_in, aim, aim, p_out))
        {
            bool const b_clear = (p_in->ping_echo == LOOK_ECHO_OPEN) ||
                ((p_in->ping_echo == LOOK_ECHO_VALID) && (p_in->ping_mm >= FRONT_RESERVE_MM));
            if (b_clear)
            {
                p_run->b_probe = false;
                p_run->b_clear = false;
                p_run->last_forward = p_in->ping_seq;
            }
        }
        return;
    }
    observe_progress(p_hug, p_in);
    run_phase(p_hug, p_look, p_in, p_out);
    bool const b_motor = (p_out->speed_mm_s != 0) || (p_out->turn_deci_s != 0);
    /* A settled fine scan pauses the recovery clock before its first motor leg too. */
    p_run->b_scan_pause = !b_motor && settled_fan_running(p_run, p_in);
    if (p_run->b_scan_pause)
    {
        p_hug->t_ms = p_in->now_ms;
    }
    if (b_motor && !p_run->b_motor)
    {
        if (!p_run->b_progress_timed)
        {
            p_run->b_progress_timed = true;
            anchor_progress(p_hug, p_in);
        }
        uint32_t const distance = (uint32_t)geom_abs(p_run->target_mm);
        uint32_t const motion_ms = (0 != p_out->turn_deci_s && 0 == p_out->speed_mm_s) ?
            ((uint32_t)MAP_LIMIT_DECI * MS_PER_SECOND) / (uint32_t)SPIN_SLOW_DECI_S :
            (distance * MS_PER_SECOND) / (uint32_t)SLOW_MM_S;
        uint32_t const plan_ms = (motion_ms < HUG_MOVE_FLOOR_MS) ? HUG_MOVE_FLOOR_MS : motion_ms;
        p_hug->t_ms = p_in->now_ms + HUG_MOVE_TIMES * plan_ms;
    }
    if (b_motor && ((p_in->now_ms - p_hug->last_motion_ms >= PILOT_STUCK_MS) ||
                    ((int32_t)(p_in->now_ms - p_hug->t_ms) >= 0)))
    {
        p_out->speed_mm_s = 0;
        p_out->turn_deci_s = 0;
        if (p_run->b_stalled)
        {
            p_hug->state = 0U;
            p_run->phase = BYPASS_IDLE;
            p_out->b_timeout = 1U;
        }
        else
        {
            p_run->b_stalled = true;
            p_run->b_probe = true;
            p_run->requested_head = (int16_t)hug_pose_heading_deci(p_hug);
            p_run->b_pending = false;
            p_hug->t_ms = p_in->now_ms + PILOT_STUCK_MS;
        }
    }
    if (b_motor)
    {
        /* Measured bypass legs either translate both wheels or spin in place. */
        bool const b_pivot = (0 == p_out->speed_mm_s);
        p_run->b_left_reverse = b_pivot ? (p_out->turn_deci_s > 0) : (p_out->speed_mm_s < 0);
        p_run->b_right_reverse = b_pivot ? (p_out->turn_deci_s < 0) : (p_out->speed_mm_s < 0);
    }
    p_run->b_motor = b_motor && !p_run->b_probe;
}
