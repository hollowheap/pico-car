/** @file avoid.c
 * @brief Reactive obstacle avoidance from a memory of recent sonar points.
 *
 * The candidate headings step away from the bearing to the goal, one step to
 * each side at a time. The first free candidate wins, so the car keeps the
 * free heading nearest the goal and so follows an obstacle edge without any
 * special case.
 */

#include "pilot/avoid.h"

#include "geometry.h"

#define DECI_STRAIGHT   (1800)
#define STEPS_HALF_TURN (DECI_STRAIGHT / AVOID_STEP_DECI)
#define CORRIDOR_MM     (AVOID_HALF_WIDTH_MM + AVOID_MARGIN_MM)
#define INT16_LIMIT_MM  (32767)

static int32_t clamp_mm(int32_t value);
static bool    heading_free(avoid_map_t const * p_map, avoid_pose_t const * p_pose,
                            int32_t head_deci);
static int32_t nearest_ahead_mm(avoid_map_t const * p_map,
                                avoid_pose_t const * p_pose, int32_t head_deci);
static int32_t speed_for(int32_t nearest_mm);

void avoid_clear(avoid_map_t * p_map)
{
    p_map->next  = 0U;
    p_map->count = 0U;
}

bool avoid_add_point(avoid_map_t * p_map, avoid_pose_t const * p_pose,
                     int32_t servo_deci, int32_t range_mm)
{
    bool stored = false;

    if ((range_mm > 0) && (range_mm < AVOID_MAX_RANGE_MM))
    {
        int32_t const car_cos  = geom_cos_q14(p_pose->head_deci);
        int32_t const car_sin  = geom_sin_q14(p_pose->head_deci);
        int32_t const beam     = p_pose->head_deci + servo_deci;
        int32_t const sensor_x = p_pose->x_mm + geom_mul_q14(AVOID_SONAR_FWD_MM, car_cos);
        int32_t const sensor_y = p_pose->y_mm + geom_mul_q14(AVOID_SONAR_FWD_MM, car_sin);
        int32_t const point_x  = sensor_x + geom_mul_q14(range_mm, geom_cos_q14(beam));
        int32_t const point_y  = sensor_y + geom_mul_q14(range_mm, geom_sin_q14(beam));

        p_map->points[p_map->next].x_mm = (int16_t)clamp_mm(point_x);
        p_map->points[p_map->next].y_mm = (int16_t)clamp_mm(point_y);
        p_map->next                     = (uint8_t)((p_map->next + 1U) % AVOID_MAP_POINTS);
        if (p_map->count < AVOID_MAP_POINTS)
        {
            p_map->count++;
        }
        stored = true;
    }
    return (stored);
}

void avoid_steer(avoid_map_t const * p_map, avoid_pose_t const * p_pose,
                 int32_t goal_x_mm, int32_t goal_y_mm, avoid_out_t * p_out)
{
    int32_t const bearing = geom_atan_deci(goal_y_mm - p_pose->y_mm,
                                           goal_x_mm - p_pose->x_mm);
    bool          found   = false;
    int32_t       chosen  = bearing;

    for (int32_t step = 0; (step <= STEPS_HALF_TURN) && !found; step++)
    {
        int32_t const left  = geom_wrap_deci(bearing + (step * AVOID_STEP_DECI));
        int32_t const right = geom_wrap_deci(bearing - (step * AVOID_STEP_DECI));
        bool const    left_free  = heading_free(p_map, p_pose, left);
        bool const    right_free = (right != left) && heading_free(p_map, p_pose, right);

        if (left_free && right_free)
        {
            /* Equal detours: take the smaller turn from the current heading. */
            int32_t const turn_left  = geom_abs(geom_wrap_deci(left - p_pose->head_deci));
            int32_t const turn_right = geom_abs(geom_wrap_deci(right - p_pose->head_deci));

            chosen = (turn_right < turn_left) ? right : left;
            found  = true;
        }
        else if (left_free || right_free)
        {
            chosen = left_free ? left : right;
            found  = true;
        }
        else
        {
            /* Both sides blocked: widen the detour. */
        }
    }

    if (!found)
    {
        p_out->head_deci  = geom_wrap_deci(p_pose->head_deci);
        p_out->speed_mm_s = 0;
        p_out->status     = AVOID_BLOCKED;
    }
    else
    {
        p_out->head_deci  = chosen;
        p_out->speed_mm_s = speed_for(nearest_ahead_mm(p_map, p_pose, chosen));
        p_out->status     = (chosen == bearing) ? AVOID_GOAL_CLEAR : AVOID_DETOUR;
    }
}

static int32_t clamp_mm(int32_t value)
{
    int32_t out = value;

    if (out > INT16_LIMIT_MM)
    {
        out = INT16_LIMIT_MM;
    }
    else if (out < -INT16_LIMIT_MM)
    {
        out = -INT16_LIMIT_MM;
    }
    else
    {
        /* In range. */
    }
    return (out);
}

/* A heading is free when no point lies in the corridor ahead of the axle.
 * Points beyond the look-ahead box are skipped first, so the products below
 * stay inside 32 bits. */
static bool heading_free(avoid_map_t const * p_map, avoid_pose_t const * p_pose,
                         int32_t head_deci)
{
    int32_t const head_cos = geom_cos_q14(head_deci);
    int32_t const head_sin = geom_sin_q14(head_deci);
    bool          free_way = true;

    for (uint32_t i = 0U; (i < p_map->count) && free_way; i++)
    {
        int32_t const dx = (int32_t)p_map->points[i].x_mm - p_pose->x_mm;
        int32_t const dy = (int32_t)p_map->points[i].y_mm - p_pose->y_mm;

        if ((geom_abs(dx) <= AVOID_LOOK_MM) && (geom_abs(dy) <= AVOID_LOOK_MM))
        {
            int32_t const along = ((dx * head_cos) + (dy * head_sin)) / GEOM_Q14_ONE;
            int32_t const side  = ((dy * head_cos) - (dx * head_sin)) / GEOM_Q14_ONE;

            if ((along >= 0) && (along <= AVOID_LOOK_MM) && (geom_abs(side) < CORRIDOR_MM))
            {
                free_way = false;
            }
        }
    }
    return (free_way);
}

/* Distance to the nearest point in the half plane ahead of the axle. A point
 * at the stop distance counts from any side: the car is touching it. */
static int32_t nearest_ahead_mm(avoid_map_t const * p_map,
                                avoid_pose_t const * p_pose, int32_t head_deci)
{
    int32_t const head_cos = geom_cos_q14(head_deci);
    int32_t const head_sin = geom_sin_q14(head_deci);
    int32_t       nearest  = AVOID_LOOK_MM;

    for (uint32_t i = 0U; i < p_map->count; i++)
    {
        int32_t const dx = (int32_t)p_map->points[i].x_mm - p_pose->x_mm;
        int32_t const dy = (int32_t)p_map->points[i].y_mm - p_pose->y_mm;

        if ((geom_abs(dx) <= AVOID_LOOK_MM) && (geom_abs(dy) <= AVOID_LOOK_MM))
        {
            int32_t const dist = (int32_t)geom_isqrt((uint32_t)((dx * dx) + (dy * dy)));
            bool const    ahead = ((dx * head_cos) + (dy * head_sin)) > 0;

            if ((ahead || (dist <= AVOID_STOP_MM)) && (dist < nearest))
            {
                nearest = dist;
            }
        }
    }
    return (nearest);
}

static int32_t speed_for(int32_t nearest_mm)
{
    int32_t speed = 0;

    if (nearest_mm > AVOID_STOP_MM)
    {
        speed = AVOID_MIN_SPEED_MM_S +
                (((AVOID_MAX_SPEED_MM_S - AVOID_MIN_SPEED_MM_S) * (nearest_mm - AVOID_STOP_MM)) /
                 (AVOID_LOOK_MM - AVOID_STOP_MM));
    }
    return (speed);
}
