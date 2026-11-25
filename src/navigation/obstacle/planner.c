/** @file planner.c
 *
 * @brief Obstacle planner and stop-and-look bypass control laws (D96, S40).
 *        Pure C99, integer only, no libm and no kernel calls.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "geometry.h"
#include "planner.h"
#include "scan_algo.h"

#define HALF          (2)
#define ROUND_DECI    (5)
#define AXIS_X        (0)
#define AXIS_Y        (1)
#define AXES          (2)
#define UM_PER_MM     (1000)
#define MAX_ATAN_DECI (600)
#define OUT_ANGLES    (4U)
#define PERCENT_FULL  (100)
#define PI_APPROX_NUM (22U)
#define PI_APPROX_DEN (7U)

static int16_t const g_out_angles_deci[OUT_ANGLES] = {
    OBSTACLE_OUT_ANGLE_A_DECI,
    OBSTACLE_OUT_ANGLE_B_DECI,
    OBSTACLE_OUT_ANGLE_C_DECI,
    OBSTACLE_OUT_ANGLE_D_DECI};

static int32_t  side_sign (uint8_t side);
static bool     slot_blocks (scan_algo_ctx_t const * p_scan, int32_t slot,
                             int32_t sign, int32_t band_x, int32_t band_y);
static bool     corridor_blocked (scan_algo_ctx_t const * p_scan,
                                  obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                                  uint32_t need_mm, int32_t first_slot);
static void     plan_wall_left (scan_algo_ctx_t const * p_scan,
                                obstacle_plan_cfg_t const * p_cfg,
                                obstacle_plan_result_t * p_res);
static void     plan_side (scan_algo_ctx_t const * p_scan,
                           obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                           int32_t p_mid_xy[AXES], obstacle_plan_result_t * p_res);
static uint8_t  clear_or_unsafe (scan_algo_ctx_t const * p_scan,
                                 obstacle_plan_cfg_t const * p_cfg);
static uint16_t chord_width (scan_algo_ctx_t const * p_scan,
                             obstacle_plan_cfg_t const * p_cfg,
                             int32_t const p_left[AXES],
                             int32_t const p_right[AXES]);
static uint8_t  pick_start_deg (obstacle_plan_cfg_t const * p_cfg,
                                uint16_t need_mm, uint16_t closest_mm);

/*!
 * @brief Sine of an angle.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Sine in Q14 (16384 = 1.0).
 */
int32_t obstacle_plan_sin_q14 (int32_t deci)
{
    return (geom_sin_q14(deci));
}

/*!
 * @brief Cosine of an angle.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Cosine in Q14 (16384 = 1.0).
 */
int32_t obstacle_plan_cos_q14 (int32_t deci)
{
    return (geom_cos_q14(deci));
}

/*!
 * @brief Tangent of an angle, clamped to +-70 degrees.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Tangent in Q14 (16384 = 1.0).
 */
int32_t obstacle_plan_tan_q14 (int32_t deci)
{
    return (geom_tan_q14(deci));
}

/*!
 * @brief Arctangent of num / den in the first quadrant, clamped to 60 deg.
 *
 * @param[in] num Opposite side (mm).
 * @param[in] den Adjacent side (mm); 0 gives 600 deci-degrees.
 *
 * @return Angle in deci-degrees, 0..600.
 */
int32_t obstacle_plan_atan_deci (uint32_t num, uint32_t den)
{
    int32_t deci = MAX_ATAN_DECI;

    if (0U != den)
    {
        deci = geom_atan_deci((int32_t)num, (int32_t)den);
        if (deci > MAX_ATAN_DECI)
        {
            deci = MAX_ATAN_DECI;
        }
    }

    return (deci);
}

/*!
 * @brief Integer square root.
 *
 * @param[in] value Number to take the root of.
 *
 * @return floor(sqrt(value)).
 */
uint32_t obstacle_plan_isqrt (uint32_t value)
{
    return (geom_isqrt(value));
}

/*!
 * @brief Fill a planner configuration from obstacle_config.h.
 *
 * @param[out] p_cfg Configuration to fill.
 */
void obstacle_plan_cfg_default (obstacle_plan_cfg_t * p_cfg)
{
    p_cfg->half_width_mm   = (uint16_t)OBSTACLE_CAR_HALF_W_MM;
    p_cfg->side_margin_mm  = (uint16_t)OBSTACLE_SIDE_MARGIN_MM;
    p_cfg->box_gap_mm      = (uint16_t)OBSTACLE_BOX_GAP_MM;
    p_cfg->free_floor_mm   = (uint16_t)OBSTACLE_FREE_FLOOR_MM;
    p_cfg->depth_mm        = (uint16_t)OBSTACLE_DEPTH_ASSUMED_MM;
    p_cfg->rear_mm         = (uint16_t)OBSTACLE_SERVO_TO_REAR_MM;
    p_cfg->beam_half_deci  = (int16_t)OBSTACLE_BEAM_HALF_DECI;
    p_cfg->path_half_deci  = (int16_t)OBSTACLE_PATH_HALF_DECI;
    p_cfg->edge_trust_deci = (int16_t)OBSTACLE_EDGE_TRUST_DECI;
}

/*!
 * @brief Turn a finished scan into a plan for the statechart.
 *
 * Picks the usable side that needs the smaller sideways offset (ties and
 * walls go left). An apparent edge beyond +-OBSTACLE_EDGE_TRUST_DECI is not
 * trusted as a bounded box edge because a flat face seen further off normal
 * loses its echo: if neither side has a trusted bounded edge, the obstacle
 * is planned as a wall on the left.
 *
 * @param[in]  p_scan   Finished scan.
 * @param[in]  p_cfg    Car and course numbers.
 * @param[out] p_result Scan result.
 */
void obstacle_plan_build (scan_algo_ctx_t const * p_scan, obstacle_plan_cfg_t const * p_cfg,
                          obstacle_plan_result_t * p_result)
{
    int32_t left_xy[AXES]  = {0, 0};
    int32_t right_xy[AXES] = {0, 0};

    p_result->status                       = OBSTACLE_RESULT_NO_SAFE_PATH;
    p_result->side                         = OBSTACLE_SIDE_LEFT;
    p_result->width_mm                     = 0U;
    p_result->start_deg                    = 0U;
    p_result->closest_mm                   = p_scan->closest_mm;
    p_result->b_wall                       = false;
    p_result->need_mm[OBSTACLE_SIDE_LEFT]  = OBSTACLE_PLAN_SIDE_BLOCKED;
    p_result->need_mm[OBSTACLE_SIDE_RIGHT] = OBSTACLE_PLAN_SIDE_BLOCKED;

    if (!scan_algo_has_region(p_scan))
    {
        p_result->status = clear_or_unsafe(p_scan, p_cfg);
    }
    else
    {
        uint16_t need_mm = OBSTACLE_PLAN_SIDE_BLOCKED;

        plan_side(p_scan, p_cfg, SCAN_ALGO_LEFT, left_xy, p_result);
        plan_side(p_scan, p_cfg, SCAN_ALGO_RIGHT, right_xy, p_result);
        plan_wall_left(p_scan, p_cfg, p_result);
        p_result->width_mm = chord_width(p_scan, p_cfg, left_xy, right_xy);

        if (p_result->need_mm[OBSTACLE_SIDE_LEFT] <= p_result->need_mm[OBSTACLE_SIDE_RIGHT])
        {
            p_result->side = (uint8_t)OBSTACLE_SIDE_LEFT;
        }
        else
        {
            p_result->side = (uint8_t)OBSTACLE_SIDE_RIGHT;
        }
        need_mm = p_result->need_mm[p_result->side];

        if (OBSTACLE_PLAN_SIDE_BLOCKED != need_mm)
        {
            p_result->status = OBSTACLE_RESULT_OK;
            if (p_result->b_wall)
            {
                p_result->start_deg =
                    (uint8_t)(OBSTACLE_WALL_PIVOT_DECI / OBSTACLE_DECI_PER_DEG);
            }
            else
            {
                p_result->start_deg = pick_start_deg(p_cfg, need_mm, p_scan->closest_mm);
            }
        }
    }
}

/*!
 * @brief Escape phase: tell whether the forward reading no longer sees
 *        the obstacle.
 *
 * @param[in] cls        Ping class.
 * @param[in] dist_mm    Ping distance.
 * @param[in] closest_mm Closest distance found by the scan.
 *
 * @return True for OPEN, or VALID beyond closest + 250 mm.
 */
bool obstacle_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                                    uint16_t closest_mm)
{
    uint32_t const limit_mm = (uint32_t)closest_mm + OBSTACLE_REGION_EXTRA_MM;
    bool           b_clear  = false;

    if (OBSTACLE_CLASS_OPEN == cls)
    {
        b_clear = true;
    }
    else if (OBSTACLE_CLASS_VALID == cls)
    {
        b_clear = ((uint32_t)dist_mm > limit_mm);
    }
    else
    {
        b_clear = false;
    }

    return (b_clear);
}

/*!
 * @brief Distance budget for the bypass.
 *
 * @param[in] p_result Scan result.
 * @param[in] p_cfg    Car and course numbers.
 *
 * @return Distance limit in mm.
 */
uint32_t obstacle_plan_hug_limit_mm (obstacle_plan_result_t const * p_result,
                                     obstacle_plan_cfg_t const * p_cfg)
{
    uint32_t const radius = (uint32_t)p_result->closest_mm +
                            ((uint32_t)p_result->width_mm / (uint32_t)HALF) +
                            (uint32_t)p_cfg->side_margin_mm +
                            (uint32_t)p_cfg->half_width_mm;
    uint32_t need = p_result->need_mm[p_result->side];
    uint32_t limit = 0U;

    if (need >= OBSTACLE_PLAN_SIDE_BLOCKED)
    {
        need = 0U;
    }

    limit = need + ((radius * PI_APPROX_NUM) / PI_APPROX_DEN) +
            OBSTACLE_HUG_EXTRA_MM;

    if (limit < OBSTACLE_HUG_MIN_MM)
    {
        limit = OBSTACLE_HUG_MIN_MM;
    }
    else if (limit > OBSTACLE_HUG_MAX_MM)
    {
        limit = OBSTACLE_HUG_MAX_MM;
    }
    else
    {
        /* Within range. */
    }

    return (limit);
}

/*!
 * @brief Steering that bends away from the obstacle.
 *
 * @param[in] side Side the car passes on (OBSTACLE_SIDE_*).
 *
 * @return OBSTACLE_STEER_LEFT or OBSTACLE_STEER_RIGHT.
 */
uint8_t obstacle_plan_steer_away (uint8_t side)
{
    return ((OBSTACLE_SIDE_LEFT == side) ? OBSTACLE_STEER_LEFT : OBSTACLE_STEER_RIGHT);
}

/*!
 * @brief Spin on the spot toward target_head_deci.
 *
 * @param[in]  cur_head_deci    Current heading in deci-degrees.
 * @param[in]  target_head_deci Wanted heading in deci-degrees.
 * @param[out] p_left_um_s      Left wheel speed in um/s.
 * @param[out] p_right_um_s     Right wheel speed in um/s.
 * @param[out] p_steer          OBSTACLE_STEER_* code.
 * @param[out] p_b_done         True when within OBSTACLE_PIVOT_TOL_DECI.
 */
void obstacle_plan_wheel_pivot (int32_t cur_head_deci, int32_t target_head_deci,
                                int32_t * p_left_um_s, int32_t * p_right_um_s,
                                uint8_t * p_steer, bool * p_b_done)
{
    int32_t const err_deci = target_head_deci - cur_head_deci;
    int32_t const abs_err  = geom_abs(err_deci);

    if (abs_err <= OBSTACLE_PIVOT_TOL_DECI)
    {
        *p_left_um_s  = 0;
        *p_right_um_s = 0;
        *p_steer      = OBSTACLE_STEER_HOLD;
        *p_b_done     = true;
    }
    else
    {
        int32_t speed_mm_s = (int32_t)OBSTACLE_PIVOT_SPEED_MM_S;

        if (abs_err < OBSTACLE_PIVOT_RAMP_DECI)
        {
            speed_mm_s = (int32_t)OBSTACLE_PIVOT_MIN_MM_S +
                         (((speed_mm_s - (int32_t)OBSTACLE_PIVOT_MIN_MM_S) * abs_err) /
                          OBSTACLE_PIVOT_RAMP_DECI);
        }

        if (err_deci > 0)
        {
            /* Turn left: right wheel forward, left wheel backward. */
            *p_left_um_s  = -speed_mm_s * UM_PER_MM;
            *p_right_um_s = speed_mm_s * UM_PER_MM;
            *p_steer      = OBSTACLE_STEER_LEFT;
        }
        else
        {
            /* Turn right: left wheel forward, right wheel backward. */
            *p_left_um_s  = speed_mm_s * UM_PER_MM;
            *p_right_um_s = -speed_mm_s * UM_PER_MM;
            *p_steer      = OBSTACLE_STEER_RIGHT;
        }
        *p_b_done = false;
    }
}

/*!
 * @brief Drive forward while holding target_head_deci with proportional
 *        wheel difference.
 *
 * @param[in]  cur_head_deci    Current heading in deci-degrees.
 * @param[in]  target_head_deci Wanted heading in deci-degrees.
 * @param[in]  speed_mm_s       Base forward speed in mm/s.
 * @param[out] p_left_um_s      Left wheel speed in um/s.
 * @param[out] p_right_um_s     Right wheel speed in um/s.
 * @param[out] p_steer          OBSTACLE_STEER_* code.
 */
void obstacle_plan_wheel_straight (int32_t cur_head_deci, int32_t target_head_deci,
                                   int32_t speed_mm_s, int32_t * p_left_um_s,
                                   int32_t * p_right_um_s, uint8_t * p_steer)
{
    int32_t const err_deci    = target_head_deci - cur_head_deci;
    int32_t const base_um_s   = speed_mm_s * UM_PER_MM;
    int32_t const max_diff_um = (int32_t)OBSTACLE_HEAD_MAX_DIFF_MM_S * UM_PER_MM;
    int32_t       diff_um_s   = err_deci * OBSTACLE_HEAD_GAIN_UM_S_PER_DECI;

    if (diff_um_s > max_diff_um)
    {
        diff_um_s = max_diff_um;
    }
    else if (diff_um_s < -max_diff_um)
    {
        diff_um_s = -max_diff_um;
    }
    else
    {
        /* Within the differential cap. */
    }

    *p_left_um_s  = base_um_s - diff_um_s;
    *p_right_um_s = base_um_s + diff_um_s;

    if (err_deci > OBSTACLE_PIVOT_TOL_DECI)
    {
        *p_steer = OBSTACLE_STEER_LEFT;
    }
    else if (err_deci < -OBSTACLE_PIVOT_TOL_DECI)
    {
        *p_steer = OBSTACLE_STEER_RIGHT;
    }
    else
    {
        *p_steer = OBSTACLE_STEER_STRAIGHT;
    }
}

/*!
 * @brief Compute the heading correction (deci-degrees away from the obstacle)
 *        after a stationary side check.
 *
 * @param[in] gap_mm        Measured axle-to-face gap in mm.
 * @param[in] prev_gap_mm   Previous check's axle-to-face gap in mm.
 * @param[in] b_have_prev   True if prev_gap_mm is valid.
 * @param[in] target_gap_mm Target axle-to-face gap in mm.
 *
 * @return Heading correction in deci-degrees (positive = away from obstacle),
 *         clamped to +-OBSTACLE_LEAN_MAX_DECI.
 */
int32_t obstacle_plan_side_lean_deci (int32_t gap_mm, int32_t prev_gap_mm,
                                      bool b_have_prev, int32_t target_gap_mm)
{
    int32_t const pos_err_mm = target_gap_mm - gap_mm;
    int32_t       lean_deci  = geom_atan_deci(pos_err_mm, (int32_t)OBSTACLE_LEAN_REF_MM);

    if (b_have_prev)
    {
        int32_t const dg_mm      = gap_mm - prev_gap_mm;
        int32_t const slope_deci = geom_atan_deci(-dg_mm, (int32_t)OBSTACLE_SIDE_LOOK_MM);

        lean_deci += (slope_deci * (int32_t)OBSTACLE_LEAN_GAIN_PCT) / PERCENT_FULL;
    }

    if (lean_deci > OBSTACLE_LEAN_MAX_DECI)
    {
        lean_deci = OBSTACLE_LEAN_MAX_DECI;
    }
    else if (lean_deci < -OBSTACLE_LEAN_MAX_DECI)
    {
        lean_deci = -OBSTACLE_LEAN_MAX_DECI;
    }
    else
    {
        /* Within lean cap. */
    }

    return (lean_deci);
}

/*!
 * @brief Heading drift trim (deci-degrees outward) from two side checks
 *        separated by OBSTACLE_SIDE_LOOK_MM.
 *
 * @param[in] gap_mm         Latest axle-to-face gap in mm.
 * @param[in] prev_gap_mm    Previous check's axle-to-face gap in mm.
 * @param[in] prev_lean_deci Outward lean commanded during the leg.
 *
 * @return Outward heading drift to subtract from the reference heading,
 *         clamped to +-OBSTACLE_LEAN_MAX_DECI.
 */
int32_t obstacle_plan_side_trim_deci (int32_t gap_mm, int32_t prev_gap_mm,
                                      int32_t prev_lean_deci)
{
    int32_t const dg_mm           = gap_mm - prev_gap_mm;
    int32_t const actual_out_deci = geom_atan_deci(dg_mm, (int32_t)OBSTACLE_SIDE_LOOK_MM);
    int32_t const drift_deci      = actual_out_deci - prev_lean_deci;
    int32_t       trim_deci       = 0;

    if (geom_abs(drift_deci) > (int32_t)OBSTACLE_PIVOT_TOL_DECI)
    {
        trim_deci = drift_deci;
    }

    if (trim_deci > OBSTACLE_LEAN_MAX_DECI)
    {
        trim_deci = OBSTACLE_LEAN_MAX_DECI;
    }
    else if (trim_deci < -OBSTACLE_LEAN_MAX_DECI)
    {
        trim_deci = -OBSTACLE_LEAN_MAX_DECI;
    }
    else
    {
        /* Within trim cap. */
    }

    return (trim_deci);
}

/*!
 * @brief Sign of an edge's outward direction (+1 left, -1 right).
 */
static int32_t side_sign (uint8_t side)
{
    return ((SCAN_ALGO_LEFT == side) ? 1 : -1);
}

/*!
 * @brief Tell whether one slot beyond an edge blocks the swept band.
 */
static bool slot_blocks (scan_algo_ctx_t const * p_scan, int32_t slot,
                         int32_t sign, int32_t band_x, int32_t band_y)
{
    uint16_t const slot_val  = p_scan->profile[slot];
    uint8_t const  cls       = obstacle_pack_slot_class(slot_val);
    bool           b_blocked = false;

    if (!obstacle_pack_slot_is_measured(slot_val))
    {
        b_blocked = false;
    }
    else if (obstacle_pack_class_is_unresolved(cls))
    {
        /* NO_ECHO, GLITCH and STUCK prove nothing: assume the corridor is
         * blocked. */
        b_blocked = true;
    }
    else if (obstacle_pack_class_is_ranged(cls))
    {
        int32_t const  deci     = scan_algo_slot_deci((uint8_t)slot);
        uint32_t const range_mm = obstacle_pack_slot_dist(slot_val);
        int32_t const  x_mm     = sign * geom_mul_q14((int32_t)range_mm, geom_sin_q14(deci));
        int32_t const  y_mm     = geom_mul_q14((int32_t)range_mm, geom_cos_q14(deci));

        b_blocked = (x_mm <= band_x) && (y_mm <= band_y);
    }
    else
    {
        b_blocked = false;
    }

    return (b_blocked);
}

/*!
 * @brief Tell whether any reading outward of an edge sits inside the swept
 *        band, or cannot be trusted.
 */
static bool corridor_blocked (scan_algo_ctx_t const * p_scan,
                              obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                              uint32_t need_mm, int32_t first_slot)
{
    int32_t const sign   = side_sign(side);
    int32_t const band_x = (int32_t)need_mm + (int32_t)p_cfg->half_width_mm;
    int32_t const band_y = (int32_t)p_scan->closest_mm +
                           (int32_t)p_cfg->depth_mm + (int32_t)p_cfg->rear_mm;
    int32_t slot      = first_slot;
    bool    b_blocked = false;

    while ((!b_blocked) && (slot >= 0) && (slot < (int32_t)OBSTACLE_SLOT_COUNT))
    {
        b_blocked = slot_blocks(p_scan, slot, sign, band_x, band_y);
        slot += sign;
    }

    return (b_blocked);
}

/*!
 * @brief Work out one side: its middle edge point (for the width) and the
 *        sideways offset it needs.
 *
 * S40: an apparent edge whose beam-corrected bearing exceeds
 * +-p_cfg->edge_trust_deci is not trusted as a bounded edge because a flat
 * face seen beyond the trusted incidence angle loses its echo.
 */
static void plan_side (scan_algo_ctx_t const * p_scan,
                       obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                       int32_t p_mid_xy[AXES], obstacle_plan_result_t * p_res)
{
    int32_t const  sign      = side_sign(side);
    uint8_t const  hit_slot  = p_scan->edge_hit[side];
    uint8_t const  miss_slot = p_scan->edge_miss[side];
    uint32_t const range_mm  = obstacle_pack_slot_dist(p_scan->profile[hit_slot]);
    int32_t const  beam      = (int32_t)p_cfg->beam_half_deci;
    int32_t const  hit_deci  = (int32_t)scan_algo_slot_deci(hit_slot);
    int32_t const  mid_deci =
        (((int32_t)scan_algo_slot_deci(hit_slot) +
          (int32_t)scan_algo_slot_deci(miss_slot)) /
         HALF) -
        (sign * beam);
    int32_t const out_deci = (int32_t)scan_algo_slot_deci(miss_slot) - (sign * beam);
    int32_t const edge_x   = sign * geom_mul_q14((int32_t)range_mm, geom_tan_q14(out_deci));
    int32_t       need_mm  = edge_x + (int32_t)p_cfg->box_gap_mm;

    p_mid_xy[AXIS_X] = geom_mul_q14((int32_t)range_mm, geom_sin_q14(mid_deci));
    p_mid_xy[AXIS_Y] = geom_mul_q14((int32_t)range_mm, geom_cos_q14(mid_deci));

    if (need_mm < (int32_t)p_cfg->box_gap_mm)
    {
        need_mm = (int32_t)p_cfg->box_gap_mm;
    }

    p_res->need_mm[side] = OBSTACLE_PLAN_SIDE_BLOCKED;

    if ((SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[side]) &&
        ((sign * hit_deci) <= (int32_t)p_cfg->edge_trust_deci) &&
        (need_mm <= (int32_t)p_cfg->free_floor_mm) &&
        (!corridor_blocked(p_scan, p_cfg, side, (uint32_t)need_mm,
                           (int32_t)miss_slot)))
    {
        p_res->need_mm[side] = (uint16_t)need_mm;
    }
}

/*!
 * @brief Wall fallback: when neither side has a trusted bounded edge, pass on
 *        the left if the left corridor is not blocked by an untrusted reading.
 */
static void plan_wall_left (scan_algo_ctx_t const * p_scan,
                            obstacle_plan_cfg_t const * p_cfg,
                            obstacle_plan_result_t * p_res)
{
    if ((OBSTACLE_PLAN_SIDE_BLOCKED == p_res->need_mm[SCAN_ALGO_LEFT]) &&
        (OBSTACLE_PLAN_SIDE_BLOCKED == p_res->need_mm[SCAN_ALGO_RIGHT]) &&
        (p_cfg->free_floor_mm > p_cfg->half_width_mm))
    {
        uint32_t const need_mm =
            (uint32_t)p_cfg->free_floor_mm - (uint32_t)p_cfg->half_width_mm;
        int32_t const first = (int32_t)p_scan->edge_hit[SCAN_ALGO_LEFT] + 1;

        /* Only NO_ECHO, GLITCH and STUCK slots beyond the last hit block a wall fallback;
         * if an edge was bounded beyond edge_trust_deci, start checking at
         * edge_miss so VALID hits on the wall face do not block it. */
        int32_t const check_from =
            (SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[SCAN_ALGO_LEFT])
                ? (int32_t)p_scan->edge_miss[SCAN_ALGO_LEFT]
                : first;

        if (!corridor_blocked(p_scan, p_cfg, SCAN_ALGO_LEFT, need_mm, check_from))
        {
            p_res->need_mm[SCAN_ALGO_LEFT] = (uint16_t)need_mm;
            p_res->b_wall                  = true;
        }
    }
}

/*!
 * @brief No region was found: report CLEAR only if every slot in the
 *        car's path gave a real reading.
 */
static uint8_t clear_or_unsafe (scan_algo_ctx_t const * p_scan,
                                obstacle_plan_cfg_t const * p_cfg)
{
    uint8_t status = OBSTACLE_RESULT_CLEAR;
    uint8_t slot   = 0U;

    for (slot = 0U; slot < OBSTACLE_SLOT_COUNT; slot++)
    {
        uint16_t const slot_val  = p_scan->profile[slot];
        int32_t const  magnitude = geom_abs(scan_algo_slot_deci(slot));
        uint8_t const  cls       = obstacle_pack_slot_class(slot_val);

        if ((obstacle_pack_slot_is_measured(slot_val)) &&
            (magnitude <= (int32_t)p_cfg->path_half_deci) &&
            (obstacle_pack_class_is_unresolved(cls)))
        {
            status = OBSTACLE_RESULT_NO_SAFE_PATH;
        }
    }

    return (status);
}

/*!
 * @brief Reported width from the two middle edge points (Cartesian chord).
 */
static uint16_t chord_width (scan_algo_ctx_t const * p_scan,
                             obstacle_plan_cfg_t const * p_cfg,
                             int32_t const p_left[AXES],
                             int32_t const p_right[AXES])
{
    uint32_t width_mm = OBSTACLE_WIDTH_MAX;
    int32_t const left_hit =
        (int32_t)scan_algo_slot_deci(p_scan->edge_hit[SCAN_ALGO_LEFT]) -
        (int32_t)p_cfg->beam_half_deci;
    int32_t const right_hit =
        -(int32_t)scan_algo_slot_deci(p_scan->edge_hit[SCAN_ALGO_RIGHT]) -
        (int32_t)p_cfg->beam_half_deci;

    if ((SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[SCAN_ALGO_LEFT]) &&
        (SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[SCAN_ALGO_RIGHT]) &&
        (left_hit <= (int32_t)p_cfg->edge_trust_deci) &&
        (right_hit <= (int32_t)p_cfg->edge_trust_deci))
    {
        int32_t const dx_mm = p_left[AXIS_X] - p_right[AXIS_X];
        int32_t const dy_mm = p_left[AXIS_Y] - p_right[AXIS_Y];

        if (dx_mm <= 0)
        {
            width_mm = 0U;
        }
        else
        {
            width_mm = geom_isqrt((uint32_t)((dx_mm * dx_mm) + (dy_mm * dy_mm)));
        }
    }

    if (width_mm > OBSTACLE_WIDTH_MAX)
    {
        width_mm = OBSTACLE_WIDTH_MAX;
    }

    return ((uint16_t)width_mm);
}

/*!
 * @brief Pick the smallest out-leg pivot angle in degrees from
 *        OBSTACLE_OUT_ANGLE_{A..D}_DECI whose straight leg leaves at least
 *        OBSTACLE_PLAN_MARGIN_MM to the front corner of the box.
 */
static uint8_t pick_start_deg (obstacle_plan_cfg_t const * p_cfg,
                               uint16_t need_mm, uint16_t closest_mm)
{
    int32_t const edge_x   = (int32_t)need_mm - (int32_t)p_cfg->box_gap_mm;
    int32_t const front_y  = (int32_t)closest_mm + (int32_t)OBSTACLE_SONAR_FWD_MM;
    int32_t const min_perp = (int32_t)p_cfg->half_width_mm + (int32_t)OBSTACLE_PLAN_MARGIN_MM;
    int32_t       chosen   = OBSTACLE_OUT_ANGLE_D_DECI;
    uint32_t      idx      = 0U;

    for (idx = 0U; idx < OUT_ANGLES; idx++)
    {
        int32_t const theta   = g_out_angles_deci[idx];
        int32_t const perp_mm = geom_mul_q14(front_y, geom_sin_q14(theta)) -
                                geom_mul_q14(edge_x, geom_cos_q14(theta));

        if (perp_mm >= min_perp)
        {
            chosen = theta;
            break;
        }
    }

    return ((uint8_t)((chosen + ROUND_DECI) / OBSTACLE_DECI_PER_DEG));
}

/*** end of file ***/
