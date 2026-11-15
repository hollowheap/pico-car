/** @file avoid.h
 * @brief Reactive obstacle avoidance from a memory of recent sonar points.
 *
 * Pure C99, integer only, no kernel and no driver calls. The caller owns the
 * clock, feeds each sonar reading to avoid_add_point(), and calls
 * avoid_steer() for the heading and speed to command.
 *
 * Frame: the odometry frame. x and y in mm, heading in deci-degrees,
 * counter-clockwise positive, 0 along +x, 900 along +y. A servo angle is
 * relative to the car front, positive to the left.
 *
 * Trigonometry comes from src/navigation/obstacle/geometry.c, which must be
 * linked next to avoid.c (include path src/navigation/obstacle).
 */

#ifndef PILOT_AVOID_H
#define PILOT_AVOID_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/** Points in the memory. The oldest point is overwritten first. */
#define AVOID_MAP_POINTS (32U)

/**
 * Sonar position ahead of the axle. Copy of OBSTACLE_SONAR_FWD_MM in
 * src/navigation/obstacle/obstacle_config.h; keep the two in step.
 */
#define AVOID_SONAR_FWD_MM (70)

/** A reading at this range or more is free space and stores no point. */
#define AVOID_MAX_RANGE_MM (600)

/** Car half width, and the extra room kept beside it. Tune on the car. */
#define AVOID_HALF_WIDTH_MM (60)
#define AVOID_MARGIN_MM     (50)

/** Distance from the axle checked for points, along a candidate heading. */
#define AVOID_LOOK_MM (400)

/**
 * Candidate headings are spaced this far apart around the bearing to the
 * goal, which is always the first candidate.
 */
#define AVOID_STEP_DECI (100)

/**
 * Speed output. At AVOID_STOP_MM and closer to the nearest point ahead the
 * speed is 0. It then rises in a straight line from AVOID_MIN_SPEED_MM_S to
 * AVOID_MAX_SPEED_MM_S at AVOID_LOOK_MM. The stop distance is below the
 * corridor half width, so a car that follows an obstacle edge at the margin
 * never stops itself.
 */
#define AVOID_STOP_MM        (100)
#define AVOID_MIN_SPEED_MM_S (60)
#define AVOID_MAX_SPEED_MM_S (200)

typedef struct
{
    int16_t x_mm;
    int16_t y_mm;
} avoid_point_t;

typedef struct
{
    avoid_point_t points[AVOID_MAP_POINTS];
    uint8_t       next;  /* slot written by the next point */
    uint8_t       count; /* valid points, at most AVOID_MAP_POINTS */
} avoid_map_t;

STATIC_ASSERT(sizeof(avoid_point_t) == 4U, "avoid_point_t must stay 4 bytes");
STATIC_ASSERT(sizeof(avoid_map_t) == 130U, "avoid_map_t must stay 130 bytes");

typedef struct
{
    int32_t x_mm;
    int32_t y_mm;
    int32_t head_deci;
} avoid_pose_t;

typedef enum
{
    AVOID_GOAL_CLEAR = 0, /* the bearing to the goal is free */
    AVOID_DETOUR,         /* another heading is free */
    AVOID_BLOCKED         /* no heading is free; the caller backs off */
} avoid_status_t;

typedef struct
{
    int32_t        head_deci;  /* wanted heading, (-1800, 1800] */
    int32_t        speed_mm_s; /* forward speed, 0 when BLOCKED */
    avoid_status_t status;
} avoid_out_t;

void avoid_clear(avoid_map_t * p_map);

/**
 * Store the point of one sonar reading.
 *
 * @return true when a point was stored; false for a reading that is not
 *         positive or reaches AVOID_MAX_RANGE_MM (free space).
 */
bool avoid_add_point(avoid_map_t * p_map, avoid_pose_t const * p_pose,
                     int32_t servo_deci, int32_t range_mm);

void avoid_steer(avoid_map_t const * p_map, avoid_pose_t const * p_pose,
                 int32_t goal_x_mm, int32_t goal_y_mm, avoid_out_t * p_out);

#endif /* PILOT_AVOID_H */
