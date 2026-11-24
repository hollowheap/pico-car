/** @file contour.c
 * @brief Execute a caller-supplied contour at a verified stationary pose.
 */
#include "pilot_internal.h"

#include <stddef.h>

#include "geometry.h"
#include "platform/chassis.h"
#include "platform/units.h"

#define HALF_DIVISOR (2)
#define CONTOUR_MAX_LENGTH_MM (3500U)
#define CONTOUR_MAX_STEP_MM   (1500U)
#define CONTOUR_RATE_DECI_S   (500)
#define CONTOUR_TRACK_MM      (25)
#define CONTOUR_TRACK_DECI    (80)
#define CONTOUR_TICK_MAX_UM   (PILOT_SPEED_MAX_MM_S * (int32_t)PILOT_STALE_MS)
#define CONTOUR_LINE_WAIT_MS  (1000U)
#define CONTOUR_HALF_TURN     (1800)
#define CONTOUR_QUARTER_TURN  (CONTOUR_HALF_TURN / HALF_DIVISOR)

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
        dd = hug_pivot_turn_um(dd, p_meas->pivot_scale_pct);
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
            pilot_set_command(p_cmd, 0, (error > 0) ? HUG_PIVOT_MIN_DECI_S : -HUG_PIVOT_MIN_DECI_S,
                    PILOT_BEHAVIOUR_HUG, PILOT_REASON_NONE);
        }
    }
    else
    {
        int32_t const rate = ((int32_t)p_step->turn_deci * HUG_LEG_MM_S) / length;

        finish = travel >= length;
        if (!finish)
        {
            pilot_set_command(p_cmd, HUG_LEG_MM_S,
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
static void contour_terminal_line(pilot_contour_t * p_run,
                                  pilot_meas_t const * p_meas, bool b_fresh_forward)
{
    bool const b_hit = p_meas->b_line_left || p_meas->b_line_right;

    if (!contour_endpoint_ok(p_run) ||
        (b_fresh_forward && !pilot_forward_clear(p_meas, HUG_GUARD_MM)) ||
        ((p_meas->now_ms - p_run->step_ms) >= CONTOUR_LINE_WAIT_MS))
    {
        p_run->status = PILOT_CONTOUR_STOPPED;
    }
    else if (b_hit && p_meas->b_stationary && p_run->b_terminal_stationary && b_fresh_forward)
    {
        p_run->status = PILOT_CONTOUR_DONE;
    }
    else
    {
        p_run->b_terminal_stationary = p_meas->b_stationary;
    }
}

bool pilot_contour_step(pilot_meas_t const * p_meas,
                        pilot_contour_t * p_run, pilot_contour_plan_t const * p_offer,
                        bool b_inhibited, bool b_course_hold, bool b_fresh_forward, pilot_cmd_t * p_cmd)
{
    if (p_run == NULL)
    {
        return false;
    }
    if ((PILOT_CONTOUR_IDLE == p_run->status) || (PILOT_CONTOUR_DONE == p_run->status))
    {
        if (b_inhibited || p_meas->b_bounded_course || b_course_hold || p_meas->b_commanded_turn ||
            !contour_offer_valid(p_offer, p_meas) || (p_run->plan.id == p_offer->id))
        {
            return false;
        }
        *p_run = (pilot_contour_t){.plan = *p_offer, .last_left_um = p_offer->left_um,
            .last_right_um = p_offer->right_um, .step_ms = p_meas->now_ms, .status = PILOT_CONTOUR_WAIT};
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
        return true;
    }
    if (!b_inhibited)
    {
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_COURSE_CLEARANCE);
    }
    if (b_inhibited || p_meas->b_bounded_course || p_meas->b_commanded_turn || (p_offer == NULL) ||
        (p_offer->id != p_run->plan.id) || (PILOT_CONTOUR_STOPPED == p_run->status) ||
        !contour_feedback(p_run, p_meas))
    {
        p_run->status = PILOT_CONTOUR_STOPPED;
    }
    else if (PILOT_CONTOUR_LINE == p_run->status)
    {
        contour_terminal_line(p_run, p_meas, b_fresh_forward);
    }
    else
    {
        pilot_contour_step_t const * const p_step = &p_run->plan.steps[p_run->index];
        uint16_t const min_range = (0U == p_step->length_mm) ? HUG_AXLE_OFF_MM - LOOK_SONAR_FWD_MM : HUG_GUARD_MM;
        if (b_fresh_forward && !pilot_forward_clear(p_meas, min_range))
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
        pilot_set_command(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, (pilot_reason_t)p_cmd->reason);
    }
    return true;
}
