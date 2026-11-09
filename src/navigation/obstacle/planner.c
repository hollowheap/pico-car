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
#define PERMILLE_FULL (1000U)
#define HALF_DIVISOR  (2U)
#define LOOK_AHEAD_MASK (OBSTACLE_LOOK_AHEAD_EVERY - 1U)
#define TRIM_CDECI_PER_DECI (100)
#define HUG_FULL_TURN_PERMILLE (PERMILLE_FULL - OBSTACLE_HUG_TURN_PERMILLE)
/* 22/7 approximates pi without libm. */
#define PI_APPROX_NUM (22U)
#define PI_APPROX_DEN (7U)

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
static uint32_t wall_distance_mm (uint32_t reading_mm, int32_t head_deci);
static void     set_turn (obstacle_hug_cmd_t * p_cmd, uint8_t dir, uint32_t err,
                          uint32_t span);
static void     seek_heading (int32_t head_deci, int32_t want_deci,
                              obstacle_hug_cmd_t * p_cmd);
static int32_t  wall_heading (uint8_t cls, uint32_t dist_mm);
static void     hug_band (uint32_t dist, obstacle_hug_cmd_t * p_cmd);
static void     cap_heading (obstacle_hug_in_t const * p_in, obstacle_hug_cmd_t * p_cmd);
static int32_t  pivot_deci (bool b_wall, uint8_t start_deg);
static uint8_t  steer_for (uint8_t side, uint8_t dir);
static void     fill_drive (obstacle_drive_in_t const * p_in, obstacle_drive_out_t * p_out);
static int32_t  abs_deci_of (int32_t deci);
static int32_t  side_sign (uint8_t side);
static int32_t  mul_q14 (uint32_t range_mm, int32_t q14);
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
    p_result->b_wall                 = false;
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
 * @brief Proportional clearance controller for the HUG loop (the only hug
 *        law; the STEER-word law of the first version is gone).
 *
 * Box mode: the look-ray reading is compared with OBSTACLE_HUG_TARGET_MM.
 * Outside the deadband the car steers back towards the target, and the
 * inner wheel slows in proportion to the error. OPEN means nothing was
 * heard, so the car steers towards the obstacle to reacquire it. UNKNOWN
 * and STUCK steer straight (the health counters stop the car if the sensor
 * stays bad). One close ping (under OBSTACLE_HUG_NEAR_MM) wins over the
 * median. The heading is capped both ways: no turn away beyond
 * OBSTACLE_HUG_AWAY_MAX_DECI and no turn towards beyond
 * -OBSTACLE_HUG_TOWARD_MAX_DECI.
 *
 * Wall mode: the reading is first turned into the distance to the wall
 * (wall_distance_mm). The distance error does not steer the wheels
 * directly, because the reading is old by the time the car has turned (the
 * servo spends about 0.4 s of every 0.65 s looking ahead). It sets a wanted
 * heading instead (wall_heading), and the wheels follow the heading error,
 * which the dead reckoning measures every 20 ms. An OPEN, blind or very
 * far reading wants +90 degrees. There is no separate one-ping override as
 * in box mode: a close wall already wants the full lean, which turns the
 * wheels at the full rate. The heading stays in +60..+120 degrees.
 *
 * @param[in]  p_in  Readings and heading.
 * @param[out] p_cmd Direction and inner wheel speed.
 */
void obstacle_plan_hug_ctrl (obstacle_hug_in_t const * p_in,
                             obstacle_hug_cmd_t * p_cmd)
{
    p_cmd->dir             = OBSTACLE_HUG_STRAIGHT;
    p_cmd->inside_permille = PERMILLE_FULL;

    if (p_in->b_wall)
    {
        uint32_t const dist = wall_distance_mm(p_in->reading_mm, p_in->head_deci);

        seek_heading(p_in->head_deci, wall_heading(p_in->cls, dist), p_cmd);
    }
    else if ((OBSTACLE_CLASS_UNKNOWN == p_in->cls) || (OBSTACLE_CLASS_STUCK == p_in->cls))
    {
        /* A blind sensor must not steer the car by distance. */
    }
    else if (p_in->latest_mm < OBSTACLE_HUG_NEAR_MM)
    {
        /* One close ping wins over the median. */
        p_cmd->dir             = OBSTACLE_HUG_AWAY;
        p_cmd->inside_permille = (uint16_t)HUG_FULL_TURN_PERMILLE;
    }
    else if (OBSTACLE_CLASS_OPEN == p_in->cls)
    {
        /* Nothing heard: treat it as the far edge of the band. */
        hug_band(OBSTACLE_HUG_TARGET_MM + OBSTACLE_HUG_DEADBAND_MM + OBSTACLE_HUG_SPAN_MM, p_cmd);
    }
    else
    {
        hug_band(p_in->reading_mm, p_cmd);
    }

    cap_heading(p_in, p_cmd);
}

/*!
 * @brief Distance from the wall to the car, from a reading of the wall ray.
 *
 * The ray is OBSTACLE_HUG_SERVO_DECI to the wall side of the heading, so it
 * meets the wall (which stands across the original line) at the angle
 * |heading - ray| off the wall normal. The nearest echo in the cone is
 * OBSTACLE_BEAM_HALF_DECI closer to the normal, hence
 * distance = reading * cos(angle - beam).
 *
 * @param[in] reading_mm Reading; OBSTACLE_SIDE_FAR_MM for no echo.
 * @param[in] head_deci  Heading from the original line (left positive).
 *
 * @return Distance in mm, or OBSTACLE_SIDE_FAR_MM when there is no echo or
 *         the ray is too slanted to trust.
 */
static uint32_t wall_distance_mm (uint32_t reading_mm, int32_t head_deci)
{
    int32_t const off_deci = abs_deci_of(head_deci - OBSTACLE_HUG_SERVO_DECI) -
                             OBSTACLE_BEAM_HALF_DECI;
    int32_t const eff_deci = (off_deci > 0) ? off_deci : 0;
    uint32_t       dist    = OBSTACLE_SIDE_FAR_MM;

    if ((reading_mm < OBSTACLE_SIDE_FAR_MM) && (eff_deci <= OBSTACLE_WALL_RAY_MAX_DECI))
    {
        /* Cast is safe: cos is 0..16384 and mul_q14 does not go negative. */
        dist = (uint32_t)mul_q14(reading_mm, obstacle_plan_cos_q14(eff_deci));
    }

    return (dist);
}

/*!
 * @brief Heading a wall-following car wants for a distance to the wall.
 *
 * Inside the deadband, or for an OPEN, UNKNOWN, STUCK or very far reading
 * (beyond target + deadband + span, where the ray may be seeing something
 * else), the car wants +90 degrees: parallel, neither towards nor away.
 * Closer than the band it leans away, up to span * gain; a little beyond the
 * band (a VALID reading, so a real wall) it leans in by the same rule. The
 * lean is at most 16 degrees, so the heading window +60..+120 is never
 * reached by this law.
 *
 * @param[in] cls     Class of the newest side ping.
 * @param[in] dist_mm Distance to the wall in mm (OBSTACLE_SIDE_FAR_MM for none).
 *
 * @return Wanted heading in deci-degrees.
 */
static int32_t wall_heading (uint8_t cls, uint32_t dist_mm)
{
    int32_t const near_edge = (int32_t)(OBSTACLE_HUG_TARGET_MM - OBSTACLE_HUG_DEADBAND_MM);
    int32_t const far_edge  = (int32_t)(OBSTACLE_HUG_TARGET_MM + OBSTACLE_HUG_DEADBAND_MM);
    int32_t const span      = (int32_t)OBSTACLE_HUG_SPAN_MM;
    int32_t const dist      = (int32_t)dist_mm;
    int32_t       want      = OBSTACLE_WALL_PIVOT_DECI;
    int32_t       err       = 0;

    if (OBSTACLE_CLASS_VALID != cls)
    {
        /* OPEN or blind: nothing to steer by. */
    }
    else if (dist < near_edge)
    {
        err  = ((near_edge - dist) > span) ? span : (near_edge - dist);
        want = OBSTACLE_WALL_PIVOT_DECI + (err * OBSTACLE_WALL_LEAN_DECI_PER_MM);
    }
    else if ((dist > far_edge) && (dist <= (far_edge + span)))
    {
        want = OBSTACLE_WALL_PIVOT_DECI - ((dist - far_edge) * OBSTACLE_WALL_LEAN_DECI_PER_MM);
    }
    else
    {
        /* In the band, or too far to trust: parallel. */
    }

    return (want);
}

/*!
 * @brief Turn the car towards a wanted heading, in proportion to the error.
 *
 * Above the wanted heading the car is turned away from the wall, so it
 * turns back towards it; below, it turns away.
 *
 * @param[in]  head_deci Heading from the original line (left positive).
 * @param[in]  want_deci Wanted heading.
 * @param[out] p_cmd     Command to fill.
 */
static void seek_heading (int32_t head_deci, int32_t want_deci, obstacle_hug_cmd_t * p_cmd)
{
    int32_t const err = head_deci - want_deci;

    if (err > OBSTACLE_WALL_HEAD_DEAD_DECI)
    {
        set_turn(p_cmd, OBSTACLE_HUG_TOWARD, (uint32_t)err, OBSTACLE_WALL_HEAD_SPAN_DECI);
    }
    else if (err < -OBSTACLE_WALL_HEAD_DEAD_DECI)
    {
        set_turn(p_cmd, OBSTACLE_HUG_AWAY, (uint32_t)(-err), OBSTACLE_WALL_HEAD_SPAN_DECI);
    }
    else
    {
        /* Close enough to the wanted heading: straight on. */
    }
}

/*!
 * @brief Set a turn proportional to an error.
 *
 * @param[out] p_cmd Command to fill.
 * @param[in]  dir   OBSTACLE_HUG_AWAY or OBSTACLE_HUG_TOWARD.
 * @param[in]  err   Error, same unit as span; larger values count as span.
 * @param[in]  span  Error that gives the full turn.
 */
static void set_turn (obstacle_hug_cmd_t * p_cmd, uint8_t dir, uint32_t err,
                      uint32_t span)
{
    uint32_t const used = (err > span) ? span : err;

    p_cmd->dir             = dir;
    p_cmd->inside_permille =
        (uint16_t)(PERMILLE_FULL - ((OBSTACLE_HUG_TURN_PERMILLE * used) / span));
}

/*!
 * @brief Box mode: compare a distance with the target band and set the turn.
 *
 * @param[in]  dist  Distance in mm.
 * @param[out] p_cmd Command to fill.
 */
static void hug_band (uint32_t dist, obstacle_hug_cmd_t * p_cmd)
{
    uint32_t const target = OBSTACLE_HUG_TARGET_MM;
    uint32_t const dead   = OBSTACLE_HUG_DEADBAND_MM;
    uint32_t const span   = OBSTACLE_HUG_SPAN_MM;

    if (dist < (target - dead))
    {
        set_turn(p_cmd, OBSTACLE_HUG_AWAY, (target - dead) - dist, span);
    }
    else if (dist > (target + dead))
    {
        set_turn(p_cmd, OBSTACLE_HUG_TOWARD, dist - (target + dead), span);
    }
    else
    {
        /* Inside the deadband: hold the target. */
    }
}

/*!
 * @brief Stop a turn that has reached its heading cap.
 *
 * @param[in]     p_in  Hug inputs (mode and heading).
 * @param[in,out] p_cmd Command; set to straight when capped.
 */
static void cap_heading (obstacle_hug_in_t const * p_in, obstacle_hug_cmd_t * p_cmd)
{
    int32_t const hi = p_in->b_wall ? OBSTACLE_WALL_HEAD_MAX_DECI : OBSTACLE_HUG_AWAY_MAX_DECI;
    int32_t const lo = p_in->b_wall ? OBSTACLE_WALL_HEAD_MIN_DECI : -OBSTACLE_HUG_TOWARD_MAX_DECI;

    if (((OBSTACLE_HUG_AWAY == p_cmd->dir) && (p_in->head_deci >= hi)) ||
        ((OBSTACLE_HUG_TOWARD == p_cmd->dir) && (p_in->head_deci <= lo)))
    {
        p_cmd->dir             = OBSTACLE_HUG_STRAIGHT;
        p_cmd->inside_permille = PERMILLE_FULL;
    }
}

/*!
 * @brief Angle the escape pivot has to reach.
 *
 * @param[in] b_wall    Wall plan.
 * @param[in] start_deg Planned start angle of a box plan, degrees.
 *
 * @return Deci-degrees away from the obstacle.
 */
static int32_t pivot_deci (bool b_wall, uint8_t start_deg)
{
    return (b_wall ? OBSTACLE_WALL_PIVOT_DECI : ((int32_t)start_deg * DECI_PER_DEG));
}

/*!
 * @brief Steering for a hug direction.
 *
 * @param[in] side Side the car passes on (OBSTACLE_SIDE_*).
 * @param[in] dir  OBSTACLE_HUG_*.
 *
 * @return OBSTACLE_STEER_STRAIGHT, LEFT or RIGHT.
 */
static uint8_t steer_for (uint8_t side, uint8_t dir)
{
    uint8_t steer = OBSTACLE_STEER_STRAIGHT;

    if (OBSTACLE_HUG_AWAY == dir)
    {
        steer = obstacle_plan_steer_away(side);
    }
    else if (OBSTACLE_HUG_TOWARD == dir)
    {
        steer = (OBSTACLE_STEER_LEFT == obstacle_plan_steer_away(side)) ? OBSTACLE_STEER_RIGHT
                                                                        : OBSTACLE_STEER_LEFT;
    }
    else
    {
        steer = OBSTACLE_STEER_STRAIGHT;
    }

    return (steer);
}

/*!
 * @brief Escape pivot: bend away on the inside wheel until the forward beam
 *        is clear (the statechart decides that), and report when the planned
 *        angle is reached.
 *
 * The pivot stands still at the heading cap (OBSTACLE_PIVOT_MAX_DECI, or
 * OBSTACLE_WALL_HEAD_MAX_DECI for a wall) instead of spinning on.
 *
 * @param[in]  p_in  Side, plan and heading.
 * @param[out] p_out Steering and speeds; b_turned is the TURNED condition.
 */
void obstacle_plan_pivot (obstacle_drive_in_t const * p_in, obstacle_drive_out_t * p_out)
{
    int32_t const cap = p_in->b_wall ? OBSTACLE_WALL_HEAD_MAX_DECI : OBSTACLE_PIVOT_MAX_DECI;

    p_out->phase        = OBSTACLE_PHASE_TURN;
    p_out->dir          = OBSTACLE_HUG_AWAY;
    p_out->steer        = obstacle_plan_steer_away(p_in->side);
    p_out->inside_um_s  = OBSTACLE_NAV_ESCAPE_INSIDE_UM_S;
    p_out->outside_um_s = (int32_t)OBSTACLE_NAV_SPEED_UM_S;
    p_out->b_turned     = (p_in->head_deci >= pivot_deci(p_in->b_wall, p_in->start_deg));

    if (p_in->head_deci >= cap)
    {
        p_out->steer = OBSTACLE_STEER_HOLD;
    }
}

/*!
 * @brief REACQUIRE driver: advance the bypass phase and choose the wheels.
 *
 * Box plan: TURN (until the start angle) -> OUT (straight to the planned
 * offset) -> ALIGN (turn back to parallel with the original line) -> HUG.
 * Wall plan: TURN (to +90 degrees) -> HUG. Several changes can happen in
 * one call.
 *
 * @param[in]  p_in  Phase, plan, heading, offset and readings.
 * @param[out] p_out Phase after this call, steering and speeds.
 */
void obstacle_plan_drive (obstacle_drive_in_t const * p_in, obstacle_drive_out_t * p_out)
{
    obstacle_drive_in_t in = *p_in;

    obstacle_plan_pivot(p_in, p_out);

    if ((OBSTACLE_PHASE_TURN == in.phase) && (p_out->b_turned))
    {
        in.phase = in.b_wall ? OBSTACLE_PHASE_HUG : OBSTACLE_PHASE_OUT;
    }

    if ((OBSTACLE_PHASE_OUT == in.phase) && (in.lateral_mm >= in.need_mm))
    {
        in.phase = OBSTACLE_PHASE_ALIGN;
    }

    if ((OBSTACLE_PHASE_ALIGN == in.phase) && (in.head_deci <= OBSTACLE_ALIGN_DONE_DECI))
    {
        in.phase = OBSTACLE_PHASE_HUG;
    }

    fill_drive(&in, p_out);
}

/*!
 * @brief Wheels for the phase reached by obstacle_plan_drive().
 *
 * @param[in]  p_in  Inputs with the new phase.
 * @param[out] p_out Output; the pivot answer stays for TURN.
 */
static void fill_drive (obstacle_drive_in_t const * p_in, obstacle_drive_out_t * p_out)
{
    obstacle_hug_cmd_t hug = {OBSTACLE_HUG_STRAIGHT, (uint16_t)PERMILLE_FULL};

    p_out->phase = p_in->phase;

    if (OBSTACLE_PHASE_TURN != p_in->phase)
    {
        int32_t const outside = (int32_t)OBSTACLE_NAV_HUG_OUTSIDE_UM_S;

        if (OBSTACLE_PHASE_ALIGN == p_in->phase)
        {
            hug.dir             = OBSTACLE_HUG_TOWARD;
            hug.inside_permille = (uint16_t)HUG_FULL_TURN_PERMILLE;
        }
        else if (OBSTACLE_PHASE_HUG == p_in->phase)
        {
            obstacle_hug_in_t hug_in;

            hug_in.cls        = p_in->side_cls;
            hug_in.reading_mm = p_in->median_mm;
            hug_in.latest_mm  = p_in->latest_mm;
            hug_in.head_deci  = p_in->head_deci;
            hug_in.b_wall     = p_in->b_wall;
            obstacle_plan_hug_ctrl(&hug_in, &hug);
        }
        else
        {
            /* OUT: straight on at the planned angle. */
        }

        p_out->dir          = hug.dir;
        p_out->steer        = steer_for(p_in->side, hug.dir);
        p_out->inside_um_s  = (outside * (int32_t)hug.inside_permille) / (int32_t)PERMILLE_FULL;
        p_out->outside_um_s = outside;
    }
}

/*!
 * @brief Tell whether the ping with this sequence number looks ahead.
 *
 * In REACQUIRE every OBSTACLE_LOOK_AHEAD_EVERY-th ping (seq 3, 7, 11 ...)
 * aims the servo straight ahead; the others aim at the hug ray. The 6-bit
 * sequence number wraps on a whole cycle.
 *
 * @param[in] seq Ping sequence number from the status word (6 bits).
 *
 * @return True for a look-ahead ping.
 */
bool obstacle_plan_is_look_ahead (uint8_t seq)
{
    return (LOOK_AHEAD_MASK == ((uint32_t)seq & LOOK_AHEAD_MASK));
}

/*!
 * @brief Start the side filter afresh: forget earlier readings and count
 *        only pings newer than seq.
 *
 * @param[out] p_filt Filter.
 * @param[in]  seq    Sequence number of the newest ping seen so far.
 */
void obstacle_plan_side_reset (obstacle_side_filter_t * p_filt, uint8_t seq)
{
    uint32_t slot = 0U;

    for (slot = 0U; slot < OBSTACLE_SIDE_FILTER_LEN; slot++)
    {
        p_filt->mm[slot] = (uint16_t)OBSTACLE_SIDE_FAR_MM;
    }

    p_filt->cls      = OBSTACLE_CLASS_UNKNOWN;
    p_filt->last_seq = seq;
    p_filt->count      = 0U;
    p_filt->next       = 0U;
    p_filt->b_fresh    = false;
    p_filt->trim_cdeci = 0;
}

/*!
 * @brief Offer the newest ping to the side filter. A ping already seen and
 *        a look-ahead ping are ignored, so a forward reading never reaches
 *        the hug law.
 *
 * @param[in,out] p_filt  Filter.
 * @param[in]     seq     Sequence number of the ping (6 bits).
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance (VALID only).
 */
void obstacle_plan_side_feed (obstacle_side_filter_t * p_filt, uint8_t seq, uint8_t cls,
                              uint16_t dist_mm)
{
    if (seq != p_filt->last_seq)
    {
        p_filt->last_seq = seq;

        if (!obstacle_plan_is_look_ahead(seq))
        {
            p_filt->cls     = cls;
            p_filt->b_fresh = true;
            p_filt->mm[p_filt->next] =
                (uint16_t)((OBSTACLE_CLASS_VALID == cls) ? dist_mm : OBSTACLE_SIDE_FAR_MM);
            p_filt->next = (uint8_t)((p_filt->next + 1U) % OBSTACLE_SIDE_FILTER_LEN);

            if (p_filt->count < OBSTACLE_SIDE_FILTER_LEN)
            {
                p_filt->count = (uint8_t)(p_filt->count + 1U);
            }
        }
    }
}

/*!
 * @brief Median of the last three side readings, so one stray echo cannot
 *        steer the car.
 *
 * @param[in] p_filt Filter.
 *
 * @return Median in mm; OBSTACLE_SIDE_FAR_MM until three readings arrived
 *         (OPEN, UNKNOWN and STUCK count as OBSTACLE_SIDE_FAR_MM).
 */
uint16_t obstacle_plan_side_median (obstacle_side_filter_t const * p_filt)
{
    uint32_t const first  = p_filt->mm[0];
    uint32_t const second = p_filt->mm[1];
    uint32_t const third  = p_filt->mm[OBSTACLE_SIDE_FILTER_LEN - 1U];
    uint32_t       median = OBSTACLE_SIDE_FAR_MM;

    if (OBSTACLE_SIDE_FILTER_LEN == p_filt->count)
    {
        median = second;

        if (((first >= second) && (first <= third)) || ((first <= second) && (first >= third)))
        {
            median = first;
        }
        else if (((third >= first) && (third <= second)) || ((third <= first) && (third >= second)))
        {
            median = third;
        }
        else
        {
            /* The second value is the middle one. */
        }
    }

    /* Cast is safe: every value is at most OBSTACLE_SIDE_FAR_MM. */
    return ((uint16_t)median);
}

/*!
 * @brief Newest side reading.
 *
 * @param[in] p_filt Filter.
 *
 * @return Distance in mm; OBSTACLE_SIDE_FAR_MM before the first reading or
 *         for OPEN, UNKNOWN and STUCK.
 */
uint16_t obstacle_plan_side_latest (obstacle_side_filter_t const * p_filt)
{
    uint16_t latest = (uint16_t)OBSTACLE_SIDE_FAR_MM;

    if (p_filt->count > 0U)
    {
        latest = p_filt->mm[(p_filt->next + (OBSTACLE_SIDE_FILTER_LEN - 1U)) %
                            OBSTACLE_SIDE_FILTER_LEN];
    }

    return (latest);
}

/*!
 * @brief Wall mode: heading for the hug law, trimmed by the wall distance.
 *
 * Once per new side ping, while tracking, a VALID reading whose distance to
 * the wall lies within the lean range (not beyond target + deadband + span,
 * where the ray may see something else) moves the trim by
 * OBSTACLE_WALL_TRIM_CDECI_PER_MM per mm of error from
 * OBSTACLE_HUG_TARGET_MM, limited to +-OBSTACLE_HUG_SPAN_MM. The distance
 * itself is computed with the trimmed heading. OPEN, blind and far readings
 * leave the trim alone. See OBSTACLE_WALL_TRIM_CDECI_PER_MM.
 *
 * @param[in,out] p_filt    Filter holding the trim.
 * @param[in]     head_deci Dead-reckoned heading from the original line.
 * @param[in]     b_track   True in the wall-follow phase; false leaves the
 *                          trim unchanged (it is 0 until tracking starts).
 *
 * @return Heading plus trim, deci-degrees.
 */
int32_t obstacle_plan_wall_head (obstacle_side_filter_t * p_filt, int32_t head_deci,
                                 bool b_track)
{
    int32_t const span = (int32_t)OBSTACLE_HUG_SPAN_MM;
    int32_t const trim_max_cdeci = OBSTACLE_WALL_TRIM_MAX_DECI * TRIM_CDECI_PER_DECI;

    if (b_track && p_filt->b_fresh && (OBSTACLE_CLASS_VALID == p_filt->cls) &&
        (p_filt->count > 0U))
    {
        int32_t const eff  = head_deci + (p_filt->trim_cdeci / TRIM_CDECI_PER_DECI);
        int32_t const dist = (int32_t)wall_distance_mm(obstacle_plan_side_latest(p_filt), eff);
        int32_t const far_edge =
            (int32_t)(OBSTACLE_HUG_TARGET_MM + OBSTACLE_HUG_DEADBAND_MM + OBSTACLE_HUG_SPAN_MM);

        if (dist <= far_edge)
        {
            int32_t err = dist - (int32_t)OBSTACLE_HUG_TARGET_MM;

            if (err > span)
            {
                err = span;
            }
            else if (err < -span)
            {
                err = -span;
            }
            else
            {
                /* Inside the span. */
            }

            p_filt->trim_cdeci += err * OBSTACLE_WALL_TRIM_CDECI_PER_MM;

            if (p_filt->trim_cdeci > trim_max_cdeci)
            {
                p_filt->trim_cdeci = trim_max_cdeci;
            }
            else if (p_filt->trim_cdeci < -trim_max_cdeci)
            {
                p_filt->trim_cdeci = -trim_max_cdeci;
            }
            else
            {
                /* Inside the limit. */
            }
        }
    }

    p_filt->b_fresh = false;

    return (head_deci + (p_filt->trim_cdeci / TRIM_CDECI_PER_DECI));
}

/*!
 * @brief Class of the newest side ping.
 *
 * @param[in] p_filt Filter.
 *
 * @return OBSTACLE_CLASS_*; UNKNOWN before the first side ping.
 */
uint8_t obstacle_plan_side_class (obstacle_side_filter_t const * p_filt)
{
    return (p_filt->cls);
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
                      ((uint32_t)p_result->width_mm / HALF_DIVISOR) +
                      (uint32_t)p_cfg->side_margin_mm +
                      (uint32_t)p_cfg->half_width_mm;
    uint32_t need   = p_result->need_mm[p_result->side];
    uint32_t limit  = 0U;

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
 * @param[in] p_scan     Finished scan.
 * @param[in] p_cfg      Car and course numbers.
 * @param[in] side       SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 * @param[in] need_mm    Sideways offset this side needs.
 * @param[in] first_slot First slot to check, then outward from there.
 *
 * @return True if the side is blocked.
 */
static bool corridor_blocked (scan_algo_ctx_t const * p_scan,
                              obstacle_plan_cfg_t const * p_cfg, uint8_t side,
                              uint32_t need_mm, int32_t first_slot)
{
    int32_t const sign = side_sign(side);

    /* Cast is safe: need_mm is at most the free floor (<= 1023). */
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
        (!corridor_blocked(p_scan, p_cfg, side, (uint32_t)need_mm,
                           (int32_t)miss_slot)))
    {
        p_res->need_mm[side] = (uint16_t)need_mm;
    }
}

/*!
 * @brief Wall: neither edge ends inside the sweep, so pass on the left with
 *        a fixed offset.
 *
 * The offset is the largest need plan_side would still accept (free floor
 * minus half the car width), as for a bounded edge that only just fits at
 * the +60 degree slot. The left corridor is blocked when a reading after
 * the last confirmed hit (UNKNOWN or STUCK) cannot be trusted. A right
 * pass needs a bounded edge, so this never touches need_mm[RIGHT].
 *
 * @param[in]     p_scan Finished scan.
 * @param[in]     p_cfg  Car and course numbers.
 * @param[in,out] p_res  Result; need_mm[LEFT] and b_wall are set for a wall.
 */
static void plan_wall_left (scan_algo_ctx_t const * p_scan,
                            obstacle_plan_cfg_t const * p_cfg,
                            obstacle_plan_result_t * p_res)
{
    if ((SCAN_ALGO_EDGE_UNBOUNDED == p_scan->edge_state[SCAN_ALGO_LEFT]) &&
        (SCAN_ALGO_EDGE_UNBOUNDED == p_scan->edge_state[SCAN_ALGO_RIGHT]) &&
        (p_cfg->free_floor_mm > p_cfg->half_width_mm))
    {
        /* Cast is safe: the difference is positive and at most 500. */
        uint32_t const need_mm =
            (uint32_t)p_cfg->free_floor_mm - (uint32_t)p_cfg->half_width_mm;
        int32_t const first = (int32_t)p_scan->edge_hit[SCAN_ALGO_LEFT] + 1;

        if (!corridor_blocked(p_scan, p_cfg, SCAN_ALGO_LEFT, need_mm, first))
        {
            /* Cast is safe: need_mm is at most the free floor (<= 1023). */
            p_res->need_mm[SCAN_ALGO_LEFT] = (uint16_t)need_mm;
            p_res->b_wall                  = true;
        }
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

