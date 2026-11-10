/** @file geometry.c
 *
 * @brief Integer trigonometry, dead-reckoned pose and local frames.
 *        Pure C99, no libm and no kernel calls.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "geometry.h"

#define SIN_TABLE_MAX (90)
#define TAN_TABLE_MAX (70)
#define DECI_RIGHT    (900)
#define DECI_STRAIGHT (1800)
#define DECI_FULL     (3600)
#define DECI_TAN_MAX  (TAN_TABLE_MAX * OBSTACLE_DECI_PER_DEG)
#define ISQRT_TOP_BIT (1UL << 30U)
#define ISQRT_STEP    (2U)
#define HALF_DIVISOR    (2)
#define UM_PER_MM       (1000)
#define OCTANT_DEG_MAX  (45)
#define OCTANT_DECI_MAX (450)

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

static int32_t table_lookup (uint16_t const * p_table, int32_t table_max,
                             int32_t abs_deci);
static int32_t atan_first_octant (uint32_t num, uint32_t den);

/*!
 * @brief Magnitude of a signed 32-bit value.
 *
 * @param[in] value Input value (never INT32_MIN).
 *
 * @return |value|.
 */
int32_t geom_abs (int32_t value)
{
    return ((value < 0) ? -value : value);
}

/*!
 * @brief Wrap an angle into (-1800, +1800] deci-degrees.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Wrapped angle in deci-degrees.
 */
int32_t geom_wrap_deci (int32_t deci)
{
    int32_t wrapped = deci % DECI_FULL;

    if (wrapped > DECI_STRAIGHT)
    {
        wrapped -= DECI_FULL;
    }
    else if (wrapped <= -DECI_STRAIGHT)
    {
        wrapped += DECI_FULL;
    }
    else
    {
        /* Already inside (-1800, +1800]. */
    }

    return (wrapped);
}

/*!
 * @brief Sine of any angle in deci-degrees.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Sine in Q14 (16384 = 1.0).
 */
int32_t geom_sin_q14 (int32_t deci)
{
    int32_t const wrapped = geom_wrap_deci(deci);
    int32_t const mag     = geom_abs(wrapped);
    int32_t const acute   = (mag > DECI_RIGHT) ? (DECI_STRAIGHT - mag) : mag;
    int32_t const val     = table_lookup(g_sine_table, SIN_TABLE_MAX, acute);

    return ((wrapped < 0) ? -val : val);
}

/*!
 * @brief Cosine of any angle in deci-degrees.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Cosine in Q14 (16384 = 1.0).
 */
int32_t geom_cos_q14 (int32_t deci)
{
    return (geom_sin_q14(DECI_RIGHT - deci));
}

/*!
 * @brief Tangent of an angle, clamped to +-70 degrees.
 *
 * @param[in] deci Angle in deci-degrees.
 *
 * @return Tangent in Q14 (16384 = 1.0).
 */
int32_t geom_tan_q14 (int32_t deci)
{
    int32_t const mag     = geom_abs(deci);
    int32_t const clamped = (mag > DECI_TAN_MAX) ? DECI_TAN_MAX : mag;
    int32_t const val     = table_lookup(g_tangent_table, TAN_TABLE_MAX, clamped);

    return ((deci < 0) ? -val : val);
}

/*!
 * @brief Four-quadrant arctangent of num / den in deci-degrees.
 *
 * @param[in] num Opposite side (signed).
 * @param[in] den Adjacent side (signed).
 *
 * @return Angle in deci-degrees (-1800..+1800).
 */
int32_t geom_atan_deci (int32_t num, int32_t den)
{
    uint32_t const abs_num = (uint32_t)geom_abs(num);
    uint32_t const abs_den = (uint32_t)geom_abs(den);
    int32_t        acute   = 0;
    int32_t        deci    = 0;

    if ((0U == abs_num) && (0U == abs_den))
    {
        deci = 0;
    }
    else
    {
        if (abs_num <= abs_den)
        {
            acute = atan_first_octant(abs_num, abs_den);
        }
        else
        {
            acute = DECI_RIGHT - atan_first_octant(abs_den, abs_num);
        }

        deci = (den < 0) ? (DECI_STRAIGHT - acute) : acute;
        if (num < 0)
        {
            deci = -deci;
        }
    }

    return (deci);
}

/*!
 * @brief Multiply a value by a Q14 factor.
 *
 * @param[in] value Multiplicand.
 * @param[in] q14   Q14 factor (16384 = 1.0).
 *
 * @return value * q14 / 16384.
 */
int32_t geom_mul_q14 (int32_t value, int32_t q14)
{
    int64_t const prod = (int64_t)value * (int64_t)q14;

    return ((int32_t)(prod / (int64_t)GEOM_Q14_ONE));
}

/*!
 * @brief Integer square root.
 *
 * @param[in] value Number to take the root of.
 *
 * @return floor(sqrt(value)).
 */
uint32_t geom_isqrt (uint32_t value)
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
 * @brief Reset the dead-reckoned pose to the origin at the current wheel
 *        displacements.
 *
 * @param[out] p_pose   Pose to reset.
 * @param[in]  left_um  Left wheel displacement in um.
 * @param[in]  right_um Right wheel displacement in um.
 */
void geom_pose_reset (geom_pose_t * p_pose, int64_t left_um, int64_t right_um)
{
    p_pose->base_um[GEOM_WHEEL_LEFT]  = left_um;
    p_pose->base_um[GEOM_WHEEL_RIGHT] = right_um;
    p_pose->prev_um[GEOM_WHEEL_LEFT]  = left_um;
    p_pose->prev_um[GEOM_WHEEL_RIGHT] = right_um;
    p_pose->x_um                      = 0;
    p_pose->y_um                      = 0;
    p_pose->head_deci                 = 0;
}

/*!
 * @brief Advance the dead-reckoned pose from the latest wheel displacements.
 *
 * Heading comes from the total wheel difference since geom_pose_reset(), so
 * rounding never accumulates across steps.
 *
 * @param[in,out] p_pose   Pose to update.
 * @param[in]     left_um  Left wheel displacement in um.
 * @param[in]     right_um Right wheel displacement in um.
 */
void geom_pose_update (geom_pose_t * p_pose, int64_t left_um, int64_t right_um)
{
    int64_t const dl_um   = left_um - p_pose->prev_um[GEOM_WHEEL_LEFT];
    int64_t const dr_um   = right_um - p_pose->prev_um[GEOM_WHEEL_RIGHT];
    int64_t const ds_um   = (dl_um + dr_um) / HALF_DIVISOR;
    int64_t const tot_l   = left_um - p_pose->base_um[GEOM_WHEEL_LEFT];
    int64_t const tot_r   = right_um - p_pose->base_um[GEOM_WHEEL_RIGHT];
    int64_t const diff_um = tot_r - tot_l;
    int64_t const den_um  = (int64_t)OBSTACLE_WHEEL_TRACK_MM * (int64_t)UM_PER_MM;
    int32_t const new_head =
        (int32_t)((diff_um * (int64_t)OBSTACLE_NAV_DECI_PER_RAD) / den_um);
    int32_t const mid_head = (p_pose->head_deci + new_head) / HALF_DIVISOR;

    p_pose->prev_um[GEOM_WHEEL_LEFT]  = left_um;
    p_pose->prev_um[GEOM_WHEEL_RIGHT] = right_um;
    p_pose->head_deci                 = new_head;
    p_pose->x_um += (ds_um * (int64_t)geom_sin_q14(mid_head)) / (int64_t)GEOM_Q14_ONE;
    p_pose->y_um += (ds_um * (int64_t)geom_cos_q14(mid_head)) / (int64_t)GEOM_Q14_ONE;
}

/*!
 * @brief Sideways position of the pose in mm (positive = left).
 *
 * @param[in] p_pose Pose.
 *
 * @return x in mm.
 */
int32_t geom_pose_x_mm (geom_pose_t const * p_pose)
{
    return ((int32_t)(p_pose->x_um / (int64_t)UM_PER_MM));
}

/*!
 * @brief Forward position of the pose in mm.
 *
 * @param[in] p_pose Pose.
 *
 * @return y in mm.
 */
int32_t geom_pose_y_mm (geom_pose_t const * p_pose)
{
    return ((int32_t)(p_pose->y_um / (int64_t)UM_PER_MM));
}

/*!
 * @brief Mean wheel travel since geom_pose_reset(), in um.
 *
 * @param[in] p_pose Pose.
 *
 * @return Mean displacement in um.
 */
int64_t geom_pose_travel_um (geom_pose_t const * p_pose)
{
    int64_t const dl = p_pose->prev_um[GEOM_WHEEL_LEFT] - p_pose->base_um[GEOM_WHEEL_LEFT];
    int64_t const dr = p_pose->prev_um[GEOM_WHEEL_RIGHT] - p_pose->base_um[GEOM_WHEEL_RIGHT];

    return ((dl + dr) / HALF_DIVISOR);
}

/*!
 * @brief Capture the current pose as a local reference frame.
 *
 * @param[out] p_frame Frame to set.
 * @param[in]  p_pose  Current pose.
 */
void geom_frame_set (geom_frame_t * p_frame, geom_pose_t const * p_pose)
{
    p_frame->x_mm      = geom_pose_x_mm(p_pose);
    p_frame->y_mm      = geom_pose_y_mm(p_pose);
    p_frame->head_deci = p_pose->head_deci;
}

/*!
 * @brief Convert a world point (mm) into a local frame's (lat, along) coordinates.
 *
 * @param[in]  p_frame    Local frame.
 * @param[in]  world_x_mm World x (mm, positive = left).
 * @param[in]  world_y_mm World y (mm, positive = ahead).
 * @param[out] p_lat_mm   Sideways offset in the frame (mm, positive = left).
 * @param[out] p_along_mm Forward offset in the frame (mm, positive = ahead).
 */
void geom_to_local (geom_frame_t const * p_frame, int32_t world_x_mm, int32_t world_y_mm,
                    int32_t * p_lat_mm, int32_t * p_along_mm)
{
    int32_t const dx  = world_x_mm - p_frame->x_mm;
    int32_t const dy  = world_y_mm - p_frame->y_mm;
    int32_t const sin = geom_sin_q14(p_frame->head_deci);
    int32_t const cos = geom_cos_q14(p_frame->head_deci);

    *p_lat_mm   = geom_mul_q14(dx, cos) - geom_mul_q14(dy, sin);
    *p_along_mm = geom_mul_q14(dx, sin) + geom_mul_q14(dy, cos);
}

/*!
 * @brief Convert a local frame's (lat, along) point into world coordinates (mm).
 *
 * @param[in]  p_frame      Local frame.
 * @param[in]  lat_mm       Sideways offset in the frame (mm, positive = left).
 * @param[in]  along_mm     Forward offset in the frame (mm, positive = ahead).
 * @param[out] p_world_x_mm World x (mm).
 * @param[out] p_world_y_mm World y (mm).
 */
void geom_to_world (geom_frame_t const * p_frame, int32_t lat_mm, int32_t along_mm,
                    int32_t * p_world_x_mm, int32_t * p_world_y_mm)
{
    int32_t const sin = geom_sin_q14(p_frame->head_deci);
    int32_t const cos = geom_cos_q14(p_frame->head_deci);

    *p_world_x_mm = p_frame->x_mm + geom_mul_q14(lat_mm, cos) + geom_mul_q14(along_mm, sin);
    *p_world_y_mm = p_frame->y_mm - geom_mul_q14(lat_mm, sin) + geom_mul_q14(along_mm, cos);
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
    int32_t const deg   = abs_deci / OBSTACLE_DECI_PER_DEG;
    int32_t const frac  = abs_deci % OBSTACLE_DECI_PER_DEG;
    int32_t       value = 0;

    if (deg >= table_max)
    {
        value = (int32_t)p_table[table_max];
    }
    else
    {
        int32_t const low  = (int32_t)p_table[deg];
        int32_t const high = (int32_t)p_table[deg + 1];

        value = low + (((high - low) * frac) / OBSTACLE_DECI_PER_DEG);
    }

    return (value);
}

/*!
 * @brief Arctangent in the first octant (0 <= num <= den, den > 0).
 *
 * @param[in] num Opposite side.
 * @param[in] den Adjacent side.
 *
 * @return Angle in deci-degrees (0..450).
 */
static int32_t atan_first_octant (uint32_t num, uint32_t den)
{
    uint32_t const ratio   = (uint32_t)(((uint64_t)num * (uint64_t)GEOM_Q14_ONE) / (uint64_t)den);
    int32_t        deci    = OCTANT_DECI_MAX;
    int32_t        deg     = 0;
    bool           b_found = false;

    for (deg = 0; (deg < OCTANT_DEG_MAX) && (!b_found); deg++)
    {
        uint32_t const low  = g_tangent_table[deg];
        uint32_t const high = g_tangent_table[deg + 1];

        if (ratio < high)
        {
            deci = (deg * OBSTACLE_DECI_PER_DEG) +
                   (int32_t)(((ratio - low) * (uint32_t)OBSTACLE_DECI_PER_DEG) /
                             (high - low));
            b_found = true;
        }
    }

    return (deci);
}

/*** end of file ***/
