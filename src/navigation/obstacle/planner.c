/** @file planner.c
 *
 * @brief Buddy 5 planner. Pure C99, integer only, no libm.
 *
 * Beam-cone correction (Tribunal decision): an echo heard at servo angle
 * t can come from anywhere in [t - b, t + b], where b is half the cone.
 * For an edge whose last hit is at angle h and first miss at angle m
 * (d = +1 for the left edge, -1 for the right edge), the true edge lies
 * between h - d*b and m - d*b.
 *   - Reported width: both edges at their middle estimate,
 *     ((h + m) / 2) - d*b, joined by the law of cosines (worked as the
 *     Cartesian chord, which is the same thing).
 *   - Side clearance: the safe outer bound, x = r * tan(m - d*b).
 *
 * Trigonometry uses Q14 tables (16384 = 1.0) with linear interpolation
 * between whole degrees.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "scan_algo.h"

#define Q14_ONE       (16384)
#define DECI_PER_DEG  (10)
#define SIN_TABLE_MAX (90)
#define TAN_TABLE_MAX (70)
#define DECI_RIGHT    (900)
#define DECI_TAN_MAX  (TAN_TABLE_MAX * DECI_PER_DEG)
#define DECI_ATAN_MAX ((int32_t)OBSTACLE_ANGLE_MAX * DECI_PER_DEG)
#define HALF          (2)
#define ROUND_DECI    (5)
#define AXIS_X        (0)
#define AXIS_Y        (1)
#define AXES          (2)
#define ISQRT_TOP_BIT (1UL << 30U)
#define ISQRT_STEP    (2U)

/* round(16384 * sin(deg)) for deg = 0..90. */
static uint16_t const g_sine_table[SIN_TABLE_MAX + 1] = {
    0U,     286U,   572U,   857U,   1143U,  1428U,  1713U,  1997U,  2280U,
    2563U,  2845U,  3126U,  3406U,  3686U,  3964U,  4240U,  4516U,  4790U,
    5063U,  5334U,  5604U,  5872U,  6138U,  6402U,  6664U,  6924U,  7182U,
    7438U,  7692U,  7943U,  8192U,  8438U,  8682U,  8923U,  9162U,  9397U,
    9630U,  9860U,  10087U, 10311U, 10531U, 10749U, 10963U, 11174U, 11381U,
    11585U, 11786U, 11982U, 12176U, 12365U, 12551U, 12733U, 12911U, 13085U,
    13255U, 13421U, 13583U, 13741U, 13894U, 14044U, 14189U, 14330U, 14466U,
    14598U, 14726U, 14849U, 14968U, 15082U, 15191U, 15296U, 15396U, 15491U,
    15582U, 15668U, 15749U, 15826U, 15897U, 15964U, 16026U, 16083U, 16135U,
    16182U, 16225U, 16262U, 16294U, 16322U, 16344U, 16362U, 16374U, 16382U,
    16384U};

/* round(16384 * tan(deg)) for deg = 0..70. */
static uint16_t const g_tangent_table[TAN_TABLE_MAX + 1] = {
    0U,     286U,   572U,   859U,   1146U,  1433U,  1722U,  2012U,  2303U,
    2595U,  2889U,  3185U,  3483U,  3783U,  4085U,  4390U,  4698U,  5009U,
    5323U,  5641U,  5963U,  6289U,  6620U,  6955U,  7295U,  7640U,  7991U,
    8348U,  8712U,  9082U,  9459U,  9845U,  10238U, 10640U, 11051U, 11472U,
    11904U, 12346U, 12801U, 13268U, 13748U, 14242U, 14752U, 15278U, 15822U,
    16384U, 16966U, 17570U, 18196U, 18848U, 19526U, 20233U, 20971U, 21742U,
    22551U, 23399U, 24290U, 25229U, 26220U, 27268U, 28378U, 29558U, 30814U,
    32155U, 33592U, 35136U, 36799U, 38598U, 40552U, 42682U, 45015U};

static int32_t  table_lookup (uint16_t const * p_table, int32_t table_max,
                              int32_t abs_deci);
static int32_t  abs_deci_of (int32_t deci);
static int32_t  side_sign (uint8_t side);
static int32_t  mul_q14 (uint32_t range_mm, int32_t q14);
static bool     slot_blocks (scan_algo_ctx_t const * p_scan, int32_t slot,
                             int32_t sign, int32_t band_x, int32_t band_y);
static bool     corridor_blocked (scan_algo_ctx_t const * p_scan,
                                  obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                                  uint32_t need_mm);
static void     plan_side (scan_algo_ctx_t const * p_scan,
                           obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                           int32_t p_mid_xy[AXES], obstacle_plan_result_t * p_res);
static uint8_t  clear_or_unsafe (scan_algo_ctx_t const * p_scan,
                                 obstacle_plan_cfg_t const *   p_cfg);
static uint16_t chord_width (scan_algo_ctx_t const * p_scan,
                             int32_t const           p_left[AXES],
                             int32_t const           p_right[AXES]);

/*!
 * @brief Sine of an angle.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Sine in Q14 (16384 = 1.0).
 */
int32_t obstacle_plan_sin_q14 (int32_t deci)
{
    int32_t const value =
        table_lookup(g_sine_table, SIN_TABLE_MAX, abs_deci_of(deci));

    return ((deci < 0) ? -value : value);
}

/*!
 * @brief Cosine of an angle (0..90 degrees either side).
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Cosine in Q14 (16384 = 1.0).
 */
int32_t obstacle_plan_cos_q14 (int32_t deci)
{
    int32_t const magnitude = abs_deci_of(deci);
    int32_t const clamped   = (magnitude > DECI_RIGHT) ? DECI_RIGHT : magnitude;

    return (table_lookup(g_sine_table, SIN_TABLE_MAX, DECI_RIGHT - clamped));
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
    int32_t const magnitude = abs_deci_of(deci);
    int32_t const clamped =
        (magnitude > DECI_TAN_MAX) ? DECI_TAN_MAX : magnitude;
    int32_t const value = table_lookup(g_tangent_table, TAN_TABLE_MAX, clamped);

    return ((deci < 0) ? -value : value);
}

/*!
 * @brief Arctangent of num / den.
 *
 * @param[in] num Opposite side (mm).
 * @param[in] den Adjacent side (mm); 0 gives the maximum angle.
 *
 * @return Angle in deci-degrees, 0..600.
 */
int32_t obstacle_plan_atan_deci (uint32_t num, uint32_t den)
{
    int32_t  deci    = DECI_ATAN_MAX;
    uint32_t ratio   = 0U;
    int32_t  deg     = 0;
    bool     b_found = false;

    if (0U != den)
    {
        ratio = (num * (uint32_t)Q14_ONE) / den;

        for (deg = 0; (deg < (int32_t)OBSTACLE_ANGLE_MAX) && (!b_found); deg++)
        {
            uint32_t const low  = g_tangent_table[deg];
            uint32_t const high = g_tangent_table[deg + 1];

            if (ratio < high)
            {
                /* Cast is safe: the fraction is 0..9. */
                deci = (deg * DECI_PER_DEG) +
                       (int32_t)(((ratio - low) * (uint32_t)DECI_PER_DEG) /
                                 (high - low));
                b_found = true;
            }
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
    uint32_t remainder = value;
    uint32_t root      = 0U;
    uint32_t bit       = ISQRT_TOP_BIT;

    while (bit > remainder)
    {
        bit >>= ISQRT_STEP;
    }

    while (0U != bit)
    {
        if (remainder >= (root + bit))
        {
            remainder -= root + bit;
            root = (root >> 1U) + bit;
        }
        else
        {
            root >>= 1U;
        }

        bit >>= ISQRT_STEP;
    }

    return (root);
}

/*!
 * @brief Fill a planner configuration from obstacle_config.h.
 *
 * @param[out] p_cfg Configuration to fill.
 */
void obstacle_plan_cfg_default (obstacle_plan_cfg_t * p_cfg)
{
    /* Casts are safe: every value is checked against its width in
     * obstacle_config.h and is far below 32767. */
    p_cfg->half_width_mm  = (uint16_t)OBSTACLE_CAR_HALF_W_MM;
    p_cfg->side_margin_mm = (uint16_t)OBSTACLE_SIDE_MARGIN_MM;
    p_cfg->free_floor_mm  = (uint16_t)OBSTACLE_FREE_FLOOR_MM;
    p_cfg->depth_mm       = (uint16_t)OBSTACLE_DEPTH_ASSUMED_MM;
    p_cfg->rear_mm        = (uint16_t)OBSTACLE_SERVO_TO_REAR_MM;
    p_cfg->beam_half_deci = (int16_t)OBSTACLE_BEAM_HALF_DECI;
    p_cfg->path_half_deci = (int16_t)OBSTACLE_PATH_HALF_DECI;
}

/*!
 * @brief Turn a finished scan into a result for Navigation.
 *
 * Picks the usable side that needs the smaller sideways offset (least
 * deviation; a tie goes left) and the angle that aims just past that edge.
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

    p_result->status                 = OBSTACLE_RESULT_NO_SAFE_PATH;
    p_result->side                   = OBSTACLE_SIDE_LEFT;
    p_result->width_mm               = 0U;
    p_result->start_deg              = 0U;
    p_result->closest_mm             = p_scan->closest_mm;
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
        p_result->width_mm = chord_width(p_scan, left_xy, right_xy);

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
            int32_t const deci = obstacle_plan_atan_deci(need_mm, p_scan->closest_mm);

            p_result->status = OBSTACLE_RESULT_OK;

            /* Cast is safe: deci is 0..600, so the result is 0..60. */
            p_result->start_deg = (uint8_t)((deci + ROUND_DECI) / DECI_PER_DEG);
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
 * @return True for OPEN, or VALID beyond closest + 250 mm. UNKNOWN and
 *         STUCK never count as clear.
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
 * @brief Proportional clearance controller for the HUG loop.
 *
 * The look-ray reading is compared with OBSTACLE_HUG_TARGET_MM. Outside
 * the deadband the car steers back towards the target, and the inner wheel
 * slows in proportion to the error, so the path is a gentle correction
 * instead of a switch between two fixed arcs.
 *
 * OPEN means nothing was heard, so the car steers towards the obstacle to
 * reacquire it. UNKNOWN and STUCK steer straight: the health counters in
 * the statechart stop the car if the sensor stays bad.
 *
 * @param[in]  cls        Echo class (OBSTACLE_CLASS_*).
 * @param[in]  reading_mm Filtered look-ray distance in mm.
 * @param[out] p_cmd      Direction and inner wheel speed.
 */
void obstacle_plan_hug_ctrl (uint8_t cls, uint16_t reading_mm,
                             obstacle_hug_cmd_t * p_cmd)
{
    uint32_t const target = OBSTACLE_HUG_TARGET_MM;
    uint32_t const dead   = OBSTACLE_HUG_DEADBAND_MM;
    uint32_t const span   = OBSTACLE_HUG_SPAN_MM;
    uint32_t       dist   = reading_mm;
    uint32_t       err    = 0U;

    p_cmd->dir             = OBSTACLE_HUG_STRAIGHT;
    p_cmd->inside_permille = 1000U;

    if ((OBSTACLE_CLASS_UNKNOWN == cls) || (OBSTACLE_CLASS_STUCK == cls))
    {
        /* A blind sensor must not steer the car. */
    }
    else
    {
        if (OBSTACLE_CLASS_OPEN == cls)
        {
            /* Nothing heard: treat it as the far edge of the band. */
            dist = target + dead + span;
        }

        if (dist > (target + dead))
        {
            p_cmd->dir = OBSTACLE_HUG_TOWARD;
            err        = dist - (target + dead);
        }
        else if (dist < (target - dead))
        {
            p_cmd->dir = OBSTACLE_HUG_AWAY;
            err        = (target - dead) - dist;
        }
        else
        {
            /* Inside the deadband: hold the target. */
        }

        if (0U != err)
        {
            if (err > span)
            {
                err = span;
            }

            p_cmd->inside_permille = (uint16_t)(1000U -
                ((OBSTACLE_HUG_TURN_PERMILLE * err) / span));
        }
    }
}

/*!
 * @brief Distance budget for the whole bypass, from the plan.
 *
 * The car drives out by need_mm, then half-way round the obstacle. The arc
 * length is about pi * (closest + width / 2 + clearance). The fixed
 * 2500 mm budget cut this short for rocks wider than about 0.7 m.
 *
 * @param[in] p_result Plan from obstacle_plan_build().
 * @param[in] p_cfg    Car and course numbers.
 *
 * @return Bypass distance limit in mm, clamped to the configured bounds.
 */
uint32_t obstacle_plan_hug_limit_mm (obstacle_plan_result_t const * p_result,
                                     obstacle_plan_cfg_t const * p_cfg)
{
    uint32_t radius = (uint32_t)p_result->closest_mm +
                      ((uint32_t)p_result->width_mm / 2U) +
                      (uint32_t)p_cfg->side_margin_mm +
                      (uint32_t)p_cfg->half_width_mm;
    uint32_t need   = p_result->need_mm[p_result->side];
    uint32_t limit  = 0U;

    if (need >= OBSTACLE_PLAN_SIDE_BLOCKED)
    {
        need = 0U;
    }

    /* 22/7 approximates pi without libm. */
    limit = need + ((radius * 22U) / 7U) + OBSTACLE_HUG_EXTRA_MM;

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
        /* Within bounds. */
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
 * @brief Steering that bends towards the obstacle.
 *
 * @param[in] side Side the car passes on (OBSTACLE_SIDE_*).
 *
 * @return OBSTACLE_STEER_LEFT or OBSTACLE_STEER_RIGHT.
 */
uint8_t obstacle_plan_steer_toward (uint8_t side)
{
    return ((OBSTACLE_SIDE_LEFT == side) ? OBSTACLE_STEER_RIGHT : OBSTACLE_STEER_LEFT);
}

/*!
 * @brief Hug rule for one ping on the 45 degree look-ray.
 *
 * Under NEAR bends away, NEAR..FAR goes straight, beyond FAR or OPEN bends
 * towards, and UNKNOWN or STUCK keep the last choice (the fault counters
 * handle a dead sensor).
 *
 * @param[in] side       Side the car passes on (OBSTACLE_SIDE_*).
 * @param[in] cls        Ping class.
 * @param[in] dist_mm    Ping distance.
 * @param[in] last_steer Previous steering choice.
 *
 * @return Steering for this ping (OBSTACLE_STEER_*).
 */
uint8_t obstacle_plan_hug_steer (uint8_t side, uint8_t cls, uint16_t dist_mm,
                           uint8_t last_steer)
{
    uint8_t steer = last_steer;

    if (OBSTACLE_CLASS_OPEN == cls)
    {
        steer = obstacle_plan_steer_toward(side);
    }
    else if (OBSTACLE_CLASS_VALID != cls)
    {
        steer = last_steer;
    }
    else if (dist_mm < OBSTACLE_HUG_NEAR_MM)
    {
        steer = obstacle_plan_steer_away(side);
    }
    else if (dist_mm <= OBSTACLE_HUG_FAR_MM)
    {
        steer = OBSTACLE_STEER_STRAIGHT;
    }
    else
    {
        steer = obstacle_plan_steer_toward(side);
    }

    return (steer);
}

/*!
 * @brief Table lookup with linear interpolation between whole degrees.
 *
 * @param[in] p_table   Q14 table, one entry per degree.
 * @param[in] table_max Last valid index of the table.
 * @param[in] abs_deci  Non-negative angle in deci-degrees.
 *
 * @return Interpolated Q14 value.
 */
static int32_t table_lookup (uint16_t const * p_table, int32_t table_max,
                             int32_t abs_deci)
{
    int32_t const deg   = abs_deci / DECI_PER_DEG;
    int32_t const frac  = abs_deci % DECI_PER_DEG;
    int32_t       value = 0;

    if (deg >= table_max)
    {
        value = (int32_t)p_table[table_max];
    }
    else
    {
        int32_t const low  = (int32_t)p_table[deg];
        int32_t const high = (int32_t)p_table[deg + 1];

        value = low + (((high - low) * frac) / DECI_PER_DEG);
    }

    return (value);
}

/*!
 * @brief Magnitude of an angle.
 *
 * @param[in] deci Angle in deci-degrees (never INT32_MIN here).
 *
 * @return |deci|.
 */
static int32_t abs_deci_of (int32_t deci)
{
    return ((deci < 0) ? -deci : deci);
}

/*!
 * @brief Sign of an edge's outward direction.
 *
 * @param[in] side SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 *
 * @return +1 for the left edge, -1 for the right edge.
 */
static int32_t side_sign (uint8_t side)
{
    return ((SCAN_ALGO_LEFT == side) ? 1 : -1);
}

/*!
 * @brief Multiply a range by a Q14 factor.
 *
 * @param[in] range_mm Range, at most 4095 mm.
 * @param[in] q14      Q14 factor, at most 45015 in magnitude.
 *
 * @return range_mm * q14 / 16384 (fits in 32 bits).
 */
static int32_t mul_q14 (uint32_t range_mm, int32_t q14)
{
    /* Cast is safe: range_mm is at most 4095. */
    return (((int32_t)range_mm * q14) / Q14_ONE);
}

/*!
 * @brief Tell whether one slot beyond an edge blocks the swept band.
 *
 * @param[in] p_scan    Finished scan.
 * @param[in] slot      Slot number.
 * @param[in] sign      +1 for the left side, -1 for the right side.
 * @param[in] band_x    Sideways extent of the band, mm.
 * @param[in] band_y    Forward extent of the band, mm.
 *
 * @return True for UNKNOWN or STUCK, or a VALID point inside the band.
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
    else if ((OBSTACLE_CLASS_UNKNOWN == cls) || (OBSTACLE_CLASS_STUCK == cls))
    {
        b_blocked = true;
    }
    else if (OBSTACLE_CLASS_VALID == cls)
    {
        /* Cast is safe: slot is 0..24 here. */
        int32_t const  deci     = scan_algo_slot_deci((uint8_t)slot);
        uint32_t const range_mm = obstacle_pack_slot_dist(slot_val);
        int32_t const  x_mm = sign * mul_q14(range_mm, obstacle_plan_sin_q14(deci));
        int32_t const  y_mm = mul_q14(range_mm, obstacle_plan_cos_q14(deci));

        b_blocked = (x_mm <= band_x) && (y_mm <= band_y);
    }
    else
    {
        /* OPEN never blocks. */
        b_blocked = false;
    }

    return (b_blocked);
}

/*!
 * @brief Tell whether any reading outward of an edge sits inside the swept
 *        band, or cannot be trusted.
 *
 * @param[in] p_scan  Finished scan.
 * @param[in] p_cfg   Car and course numbers.
 * @param[in] side    SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 * @param[in] need_mm Sideways offset this side needs.
 *
 * @return True if the side is blocked.
 */
static bool corridor_blocked (scan_algo_ctx_t const * p_scan,
                              obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                              uint32_t need_mm)
{
    int32_t const sign = side_sign(side);

    /* Cast is safe: need_mm is at most the free floor (<= 1023). */
    int32_t const band_x = (int32_t)need_mm + (int32_t)p_cfg->half_width_mm;
    int32_t const band_y = (int32_t)p_scan->closest_mm +
                           (int32_t)p_cfg->depth_mm + (int32_t)p_cfg->rear_mm;
    int32_t slot      = (int32_t)p_scan->edge_miss[side];
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
 * @param[in]  p_scan   Finished scan.
 * @param[in]  p_cfg    Car and course numbers.
 * @param[in]  side     SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 * @param[out] p_mid_xy Middle edge point (x, y) in mm.
 * @param[out] p_res    Result; need_mm[side] is filled in.
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
    int32_t const  mid_deci =
        ((scan_algo_slot_deci(hit_slot) + scan_algo_slot_deci(miss_slot)) /
         HALF) -
        (sign * beam);
    int32_t const out_deci = scan_algo_slot_deci(miss_slot) - (sign * beam);
    int32_t need_mm = (sign * mul_q14(range_mm, obstacle_plan_tan_q14(out_deci))) +
                      (int32_t)p_cfg->half_width_mm +
                      (int32_t)p_cfg->side_margin_mm;

    p_mid_xy[AXIS_X] = mul_q14(range_mm, obstacle_plan_sin_q14(mid_deci));
    p_mid_xy[AXIS_Y] = mul_q14(range_mm, obstacle_plan_cos_q14(mid_deci));

    if (need_mm < 0)
    {
        need_mm = 0;
    }

    p_res->need_mm[side] = OBSTACLE_PLAN_SIDE_BLOCKED;

    /* Casts are safe: need_mm is non-negative and, when stored, no larger
     * than the free floor (<= 1023). */
    if ((SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[side]) &&
        ((need_mm + (int32_t)p_cfg->half_width_mm) <=
         (int32_t)p_cfg->free_floor_mm) &&
        (!corridor_blocked(p_scan, p_cfg, side, (uint32_t)need_mm)))
    {
        p_res->need_mm[side] = (uint16_t)need_mm;
    }
}

/*!
 * @brief No region was found: report CLEAR only if every slot in the
 *        car's path gave a real reading.
 *
 * @param[in] p_scan Finished scan.
 * @param[in] p_cfg  Car and course numbers.
 *
 * @return OBSTACLE_RESULT_CLEAR or OBSTACLE_RESULT_NO_SAFE_PATH.
 */
static uint8_t clear_or_unsafe (scan_algo_ctx_t const * p_scan,
                                obstacle_plan_cfg_t const *   p_cfg)
{
    uint8_t status = OBSTACLE_RESULT_CLEAR;
    uint8_t slot   = 0U;

    for (slot = 0U; slot < OBSTACLE_SLOT_COUNT; slot++)
    {
        uint16_t const slot_val  = p_scan->profile[slot];
        int32_t const  magnitude = abs_deci_of(scan_algo_slot_deci(slot));
        uint8_t const  cls       = obstacle_pack_slot_class(slot_val);

        if ((obstacle_pack_slot_is_measured(slot_val)) &&
            (magnitude <= (int32_t)p_cfg->path_half_deci) &&
            (OBSTACLE_CLASS_VALID != cls) && (OBSTACLE_CLASS_OPEN != cls))
        {
            status = OBSTACLE_RESULT_NO_SAFE_PATH;
        }
    }

    return (status);
}

/*!
 * @brief Reported width from the two middle edge points (law of cosines).
 *
 * @param[in] p_scan  Finished scan.
 * @param[in] p_left  Left middle edge point (x, y).
 * @param[in] p_right Right middle edge point (x, y).
 *
 * @return Width in mm; 1023 means an edge is open ("1023 or wider").
 */
static uint16_t chord_width (scan_algo_ctx_t const * p_scan,
                             int32_t const           p_left[AXES],
                             int32_t const           p_right[AXES])
{
    uint32_t width_mm = OBSTACLE_WIDTH_MAX;

    if ((SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[SCAN_ALGO_LEFT]) &&
        (SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[SCAN_ALGO_RIGHT]))
    {
        int32_t const dx_mm = p_left[AXIS_X] - p_right[AXIS_X];
        int32_t const dy_mm = p_left[AXIS_Y] - p_right[AXIS_Y];

        if (dx_mm <= 0)
        {
            /* The corrected edges crossed: a very thin object. */
            width_mm = 0U;
        }
        else
        {
            /* Cast is safe: both terms are below 2^26. */
            width_mm =
                obstacle_plan_isqrt((uint32_t)((dx_mm * dx_mm) + (dy_mm * dy_mm)));
        }
    }

    if (width_mm > OBSTACLE_WIDTH_MAX)
    {
        width_mm = OBSTACLE_WIDTH_MAX;
    }

    /* Cast is safe: width_mm is at most 1023 here. */
    return ((uint16_t)width_mm);
}

/*** end of file ***/

