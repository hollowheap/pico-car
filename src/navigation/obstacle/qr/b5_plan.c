/** @file b5_plan.c
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

#include "b5_cfg.h"
#include "b5_pack.h"
#include "b5_plan.h"
#include "scan_algo.h"

#define Q14_ONE       (16384)
#define DECI_PER_DEG  (10)
#define SIN_TABLE_MAX (90)
#define TAN_TABLE_MAX (70)
#define DECI_RIGHT    (900)
#define DECI_STRAIGHT (1800)
#define DECI_TAN_MAX  (TAN_TABLE_MAX * DECI_PER_DEG)
#define DECI_ATAN_MAX ((int32_t)B5_ANGLE_MAX * DECI_PER_DEG)
#define HALF          (2)
#define ROUND_DECI    (5)
#define AXIS_X        (0)
#define AXIS_Y        (1)
#define AXES          (2)
#define ISQRT_TOP_BIT (1UL << 30U)
#define ISQRT_STEP    (2U)

B5_CT_ASSERT(clear_unbounded_past_limit,
             B5_PLAN_CLEAR_UNBOUNDED ==
                 (B5_SERVO_LIMIT_DECI + B5_SLOT_STEP_DECI));

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
                                  b5_plan_cfg_t const * p_cfg, uint8_t side,
                                  uint32_t need_mm);
static void     plan_side (scan_algo_ctx_t const * p_scan,
                           b5_plan_cfg_t const * p_cfg, uint8_t side,
                           int32_t p_mid_xy[AXES], b5_plan_result_t * p_res);
static uint8_t  clear_or_unsafe (scan_algo_ctx_t const * p_scan,
                                 b5_plan_cfg_t const *   p_cfg);
static uint16_t chord_width (scan_algo_ctx_t const * p_scan,
                             int32_t const           p_left[AXES],
                             int32_t const           p_right[AXES]);

/*!
 * @brief Sine of an angle (up to 180 degrees either side; beyond that the
 *        angle is treated as 180).
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Sine in Q14 (16384 = 1.0).
 */
int32_t b5_plan_sin_q14 (int32_t deci)
{
    int32_t const magnitude = abs_deci_of(deci);
    int32_t const clamped =
        (magnitude > DECI_STRAIGHT) ? DECI_STRAIGHT : magnitude;
    int32_t const folded =
        (clamped > DECI_RIGHT) ? (DECI_STRAIGHT - clamped) : clamped;
    int32_t const value = table_lookup(g_sine_table, SIN_TABLE_MAX, folded);

    return ((deci < 0) ? -value : value);
}

/*!
 * @brief Cosine of an angle (up to 180 degrees either side; beyond that the
 *        angle is treated as 180).
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Cosine in Q14 (16384 = 1.0).
 */
int32_t b5_plan_cos_q14 (int32_t deci)
{
    int32_t const magnitude = abs_deci_of(deci);
    int32_t const clamped =
        (magnitude > DECI_STRAIGHT) ? DECI_STRAIGHT : magnitude;
    int32_t value = 0;

    if (clamped > DECI_RIGHT)
    {
        value =
            -table_lookup(g_sine_table, SIN_TABLE_MAX, clamped - DECI_RIGHT);
    }
    else
    {
        value = table_lookup(g_sine_table, SIN_TABLE_MAX, DECI_RIGHT - clamped);
    }

    return (value);
}

/*!
 * @brief Tangent of an angle, clamped to +-70 degrees.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Tangent in Q14 (16384 = 1.0).
 */
int32_t b5_plan_tan_q14 (int32_t deci)
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
int32_t b5_plan_atan_deci (uint32_t num, uint32_t den)
{
    int32_t  deci    = DECI_ATAN_MAX;
    uint32_t ratio   = 0U;
    int32_t  deg     = 0;
    bool     b_found = false;

    if (0U != den)
    {
        ratio = (num * (uint32_t)Q14_ONE) / den;

        for (deg = 0; (deg < (int32_t)B5_ANGLE_MAX) && (!b_found); deg++)
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
uint32_t b5_plan_isqrt (uint32_t value)
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
 * @brief Fill a planner configuration from b5_cfg.h.
 *
 * @param[out] p_cfg Configuration to fill.
 */
void b5_plan_cfg_default (b5_plan_cfg_t * p_cfg)
{
    /* Casts are safe: every value is checked against its width in
     * b5_cfg.h and is far below 32767. */
    p_cfg->half_width_mm  = (uint16_t)B5_CAR_HALF_W_MM;
    p_cfg->side_margin_mm = (uint16_t)B5_SIDE_MARGIN_MM;
    p_cfg->free_floor_mm  = (uint16_t)B5_FREE_FLOOR_MM;
    p_cfg->depth_mm       = (uint16_t)B5_DEPTH_ASSUMED_MM;
    p_cfg->rear_mm        = (uint16_t)B5_SERVO_TO_REAR_MM;
    p_cfg->beam_half_deci = (int16_t)B5_BEAM_HALF_DECI;
    p_cfg->path_half_deci = (int16_t)B5_PATH_HALF_DECI;
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
void b5_plan_build (scan_algo_ctx_t const * p_scan, b5_plan_cfg_t const * p_cfg,
                    b5_plan_result_t * p_result)
{
    int32_t left_xy[AXES]  = {0, 0};
    int32_t right_xy[AXES] = {0, 0};

    p_result->status                 = B5_RESULT_NO_SAFE_PATH;
    p_result->side                   = B5_SIDE_LEFT;
    p_result->width_mm               = 0U;
    p_result->start_deg              = 0U;
    p_result->closest_mm             = p_scan->closest_mm;
    p_result->need_mm[B5_SIDE_LEFT]  = B5_PLAN_SIDE_BLOCKED;
    p_result->need_mm[B5_SIDE_RIGHT] = B5_PLAN_SIDE_BLOCKED;

    /* Cast is safe: B5_PLAN_CLEAR_UNBOUNDED fits in int16_t. */
    p_result->clear_deci[B5_SIDE_LEFT]  = (int16_t)B5_PLAN_CLEAR_UNBOUNDED;
    p_result->clear_deci[B5_SIDE_RIGHT] = (int16_t)B5_PLAN_CLEAR_UNBOUNDED;

    if (!scan_algo_has_region(p_scan))
    {
        p_result->status = clear_or_unsafe(p_scan, p_cfg);
    }
    else
    {
        uint16_t need_mm = B5_PLAN_SIDE_BLOCKED;

        plan_side(p_scan, p_cfg, SCAN_ALGO_LEFT, left_xy, p_result);
        plan_side(p_scan, p_cfg, SCAN_ALGO_RIGHT, right_xy, p_result);
        p_result->width_mm = chord_width(p_scan, left_xy, right_xy);

        if (p_result->need_mm[B5_SIDE_LEFT] <= p_result->need_mm[B5_SIDE_RIGHT])
        {
            p_result->side = (uint8_t)B5_SIDE_LEFT;
        }
        else
        {
            p_result->side = (uint8_t)B5_SIDE_RIGHT;
        }
        need_mm = p_result->need_mm[p_result->side];

        if (B5_PLAN_SIDE_BLOCKED != need_mm)
        {
            int32_t const deci = b5_plan_atan_deci(need_mm, p_scan->closest_mm);

            p_result->status = B5_RESULT_OK;

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
bool b5_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                              uint16_t closest_mm)
{
    uint32_t const limit_mm = (uint32_t)closest_mm + B5_REGION_EXTRA_MM;
    bool           b_clear  = false;

    if (B5_CLASS_OPEN == cls)
    {
        b_clear = true;
    }
    else if (B5_CLASS_VALID == cls)
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
 * @brief Steering that bends away from the obstacle.
 *
 * @param[in] side Side the car passes on (B5_SIDE_*).
 *
 * @return B5_STEER_LEFT or B5_STEER_RIGHT.
 */
uint8_t b5_plan_steer_away (uint8_t side)
{
    return ((B5_SIDE_LEFT == side) ? B5_STEER_LEFT : B5_STEER_RIGHT);
}

/*!
 * @brief Steering that bends towards the obstacle.
 *
 * @param[in] side Side the car passes on (B5_SIDE_*).
 *
 * @return B5_STEER_LEFT or B5_STEER_RIGHT.
 */
uint8_t b5_plan_steer_toward (uint8_t side)
{
    return ((B5_SIDE_LEFT == side) ? B5_STEER_RIGHT : B5_STEER_LEFT);
}

/*!
 * @brief Revised bypass, before the first pivot: how far to back up so the
 *        front corner swinging round the stopped wheel keeps
 *        B5_NAV_CLEAR_MM from the obstacle face.
 *
 * Frame for the bypass helpers: the middle of the wheel axle where the
 * car stopped is the origin, "ahead" runs along the original line and
 * "side" towards the side being passed.
 *
 * @param[in] closest_mm Closest scan reading, measured from the sensor.
 *
 * @return Distance to reverse, mm (0 when there is room already).
 */
uint32_t b5_plan_backup_mm (uint32_t closest_mm)
{
    uint32_t const needed_mm =
        (B5_PIVOT_REACH_MM + B5_NAV_CLEAR_MM) - B5_AXLE_TO_SONAR_MM;

    return ((closest_mm >= needed_mm) ? 0U : (needed_mm - closest_mm));
}

/*!
 * @brief Revised bypass: A, the room the obstacle takes up on the side
 *        being passed, from the first scan: B x tan(cleared angle), at
 *        least 0 and at most B5_ROOM_MAX_MM.
 *
 * @param[in] closest_mm B, the closest scan reading (from the sensor).
 * @param[in] clear_deci Raw cleared angle of that side, positive =
 *                       outwards (b5_plan_result_t.clear_deci).
 *
 * @return A in mm.
 */
uint32_t b5_plan_room_mm (uint32_t closest_mm, int32_t clear_deci)
{
    int32_t room_mm = 0;

    if (clear_deci > 0)
    {
        room_mm = mul_q14(closest_mm, b5_plan_tan_q14(clear_deci));
    }

    if (room_mm > (int32_t)B5_ROOM_MAX_MM)
    {
        room_mm = (int32_t)B5_ROOM_MAX_MM;
    }

    /* Cast is safe: room_mm is 0..B5_ROOM_MAX_MM here. */
    return ((uint32_t)room_mm);
}

/*!
 * @brief Revised bypass: E, how far the car moves out after the first
 *        pivot: A + car length + B5_OUT_BUFFER_MM.
 *
 * @param[in] room_mm A (b5_plan_room_mm()).
 *
 * @return E in mm.
 */
uint32_t b5_plan_out_mm (uint32_t room_mm)
{
    return (room_mm + B5_CAR_LEN_MM + B5_OUT_BUFFER_MM);
}

/*!
 * @brief Revised bypass: how far the car drives on after the pivot back,
 *        to come level with the obstacle's near face: B + B5_LEVEL_BUFFER_MM.
 *
 * @param[in] closest_mm B, the closest scan reading (from the sensor).
 *
 * @return Distance in mm.
 */
uint32_t b5_plan_level_mm (uint32_t closest_mm)
{
    return (closest_mm + B5_LEVEL_BUFFER_MM);
}

/*!
 * @brief Revised bypass: judge one side scan, taken with the car facing
 *        the line.
 *
 * Each reading at servo angle t (alpha = fwd_sign x t, positive = forward
 * along the line) is a point f = r cos(alpha) towards the line and
 * s = r sin(alpha) along it. Points with f beyond lane_mm are past the line
 * and ignored; the rest are "hits".
 *   - Lane: a hit with |s| <= B5_LANE_HALF_MM blocks the way to the line.
 *   - d: the closest hit.
 *   - Forward cleared angle: one slot past the last hit with alpha >= 0;
 *     0 with no forward hit; none (B5_SIDE_ASSUMED_DECI is used) when the
 *     hits reach the servo limit.
 *   - Move on: d x tan(cleared angle) + B5_LANE_HALF_MM (D + C).
 *
 * @param[in]  p_mm     B5_SLOT_COUNT readings in slot order (mm, or
 *                      B5_PLAN_NO_ECHO).
 * @param[in]  fwd_sign +1 if a left (positive) servo angle looks forward
 *                      along the line, -1 if a right one does.
 * @param[in]  lane_mm  How far the lane runs from the sensor towards (and
 *                      past) the line.
 * @param[out] p_side   The judgement.
 */
void b5_plan_side_scan (uint16_t const * p_mm, int32_t fwd_sign,
                        uint32_t lane_mm, b5_plan_side_t * p_side)
{
    uint32_t closest_mm = B5_PLAN_NO_ECHO;
    int32_t  last_deci  = -1;
    bool     b_blocked  = false;
    uint32_t slot       = 0U;

    for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
    {
        uint32_t const range_mm = p_mm[slot];

        /* Cast is safe: slot is 0..24. */
        int32_t const alpha = fwd_sign * scan_algo_slot_deci((uint8_t)slot);

        if (B5_PLAN_NO_ECHO != range_mm)
        {
            int32_t const to_line_mm =
                mul_q14(range_mm, b5_plan_cos_q14(alpha));
            int32_t const along_mm = mul_q14(range_mm, b5_plan_sin_q14(alpha));

            /* Cast is safe: lane_mm is a few hundred mm. */
            if (to_line_mm <= (int32_t)lane_mm)
            {
                closest_mm = (range_mm < closest_mm) ? range_mm : closest_mm;
                b_blocked =
                    b_blocked || ((along_mm <= (int32_t)B5_LANE_HALF_MM) &&
                                  (along_mm >= -(int32_t)B5_LANE_HALF_MM));
                last_deci =
                    ((alpha >= 0) && (alpha > last_deci)) ? alpha : last_deci;
            }
        }
    }

    p_side->b_lane_clear = !b_blocked;
    p_side->b_cleared    = (last_deci < B5_SERVO_LIMIT_DECI);

    /* Casts are safe: the angles are 0..600 and closest_mm is 12 bits or
     * B5_PLAN_NO_ECHO. */
    if (last_deci < 0)
    {
        p_side->clear_deci = 0;
    }
    else if (p_side->b_cleared)
    {
        p_side->clear_deci = (int16_t)(last_deci + B5_SLOT_STEP_DECI);
    }
    else
    {
        p_side->clear_deci = (int16_t)B5_SIDE_ASSUMED_DECI;
    }

    if (B5_PLAN_NO_ECHO == closest_mm)
    {
        p_side->closest_mm = 0U;
        p_side->move_mm    = B5_LANE_HALF_MM;
    }
    else
    {
        p_side->closest_mm = (uint16_t)closest_mm;
        p_side->move_mm =
            (uint32_t)mul_q14(closest_mm, b5_plan_tan_q14(p_side->clear_deci)) +
            B5_LANE_HALF_MM;
    }
}

/*!
 * @brief Side position of a point straight ahead of the axle (an IR
 *        sensor), for a car at side_mm pointing away_deci from the line
 *        (positive = away from the obstacle).
 *
 * @param[in] side_mm   Side position of the middle of the axle.
 * @param[in] away_deci Heading, deci-degrees, positive = away.
 * @param[in] ahead_mm  How far the point is ahead of the axle.
 *
 * @return Side position of the point, mm.
 */
int32_t b5_plan_point_side_mm (int32_t side_mm, int32_t away_deci,
                               uint32_t ahead_mm)
{
    return (side_mm +
            (((int32_t)ahead_mm * b5_plan_sin_q14(away_deci)) / Q14_ONE));
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
    uint8_t const  cls       = b5_pack_slot_class(slot_val);
    bool           b_blocked = false;

    if (!b5_pack_slot_is_measured(slot_val))
    {
        b_blocked = false;
    }
    else if ((B5_CLASS_UNKNOWN == cls) || (B5_CLASS_STUCK == cls))
    {
        b_blocked = true;
    }
    else if (B5_CLASS_VALID == cls)
    {
        /* Cast is safe: slot is 0..24 here. */
        int32_t const  deci     = scan_algo_slot_deci((uint8_t)slot);
        uint32_t const range_mm = b5_pack_slot_dist(slot_val);
        int32_t const  x_mm = sign * mul_q14(range_mm, b5_plan_sin_q14(deci));
        int32_t const  y_mm = mul_q14(range_mm, b5_plan_cos_q14(deci));

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
                              b5_plan_cfg_t const * p_cfg, uint8_t side,
                              uint32_t need_mm)
{
    int32_t const sign = side_sign(side);

    /* Cast is safe: need_mm is at most the free floor (<= 1023). */
    int32_t const band_x = (int32_t)need_mm + (int32_t)p_cfg->half_width_mm;
    int32_t const band_y = (int32_t)p_scan->closest_mm +
                           (int32_t)p_cfg->depth_mm + (int32_t)p_cfg->rear_mm;
    int32_t slot      = (int32_t)p_scan->edge_miss[side];
    bool    b_blocked = false;

    while ((!b_blocked) && (slot >= 0) && (slot < (int32_t)B5_SLOT_COUNT))
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
                       b5_plan_cfg_t const * p_cfg, uint8_t side,
                       int32_t p_mid_xy[AXES], b5_plan_result_t * p_res)
{
    int32_t const  sign      = side_sign(side);
    uint8_t const  hit_slot  = p_scan->edge_hit[side];
    uint8_t const  miss_slot = p_scan->edge_miss[side];
    uint32_t const range_mm  = b5_pack_slot_dist(p_scan->profile[hit_slot]);
    int32_t const  beam      = (int32_t)p_cfg->beam_half_deci;
    int32_t const  mid_deci =
        ((scan_algo_slot_deci(hit_slot) + scan_algo_slot_deci(miss_slot)) /
         HALF) -
        (sign * beam);
    int32_t const out_deci = scan_algo_slot_deci(miss_slot) - (sign * beam);
    int32_t need_mm = (sign * mul_q14(range_mm, b5_plan_tan_q14(out_deci))) +
                      (int32_t)p_cfg->half_width_mm +
                      (int32_t)p_cfg->side_margin_mm;

    p_mid_xy[AXIS_X] = mul_q14(range_mm, b5_plan_sin_q14(mid_deci));
    p_mid_xy[AXIS_Y] = mul_q14(range_mm, b5_plan_cos_q14(mid_deci));

    if (need_mm < 0)
    {
        need_mm = 0;
    }

    p_res->need_mm[side] = B5_PLAN_SIDE_BLOCKED;

    /* Cast is safe: a slot angle is within +-600. */
    p_res->clear_deci[side] =
        (SCAN_ALGO_EDGE_BOUNDED == p_scan->edge_state[side])
            ? (int16_t)(sign * scan_algo_slot_deci(miss_slot))
            : (int16_t)B5_PLAN_CLEAR_UNBOUNDED;

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
 * @return B5_RESULT_CLEAR or B5_RESULT_NO_SAFE_PATH.
 */
static uint8_t clear_or_unsafe (scan_algo_ctx_t const * p_scan,
                                b5_plan_cfg_t const *   p_cfg)
{
    uint8_t status = B5_RESULT_CLEAR;
    uint8_t slot   = 0U;

    for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
    {
        uint16_t const slot_val  = p_scan->profile[slot];
        int32_t const  magnitude = abs_deci_of(scan_algo_slot_deci(slot));
        uint8_t const  cls       = b5_pack_slot_class(slot_val);

        if ((b5_pack_slot_is_measured(slot_val)) &&
            (magnitude <= (int32_t)p_cfg->path_half_deci) &&
            (B5_CLASS_VALID != cls) && (B5_CLASS_OPEN != cls))
        {
            status = B5_RESULT_NO_SAFE_PATH;
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
    uint32_t width_mm = B5_WIDTH_MAX;

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
                b5_plan_isqrt((uint32_t)((dx_mm * dx_mm) + (dy_mm * dy_mm)));
        }
    }

    if (width_mm > B5_WIDTH_MAX)
    {
        width_mm = B5_WIDTH_MAX;
    }

    /* Cast is safe: width_mm is at most 1023 here. */
    return ((uint16_t)width_mm);
}

/*** end of file ***/
