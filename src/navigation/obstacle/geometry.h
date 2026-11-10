/** @file geometry.h
 *
 * @brief Integer trigonometry, the dead-reckoned pose and local frames.
 *        Pure C99, no libm and no kernel calls.
 *
 * Frame: x is sideways, positive to the LEFT; y is straight ahead; a
 * heading is positive when the car has turned left, so heading h points
 * along (sin h, cos h). Angles are deci-degrees (0.1 degree), distances
 * mm (pose: um). Trigonometry uses Q14 values (16384 = 1.0).
 */

#ifndef OBSTACLE_GEOMETRY_H
#define OBSTACLE_GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Q14 value of 1.0. */
#define GEOM_Q14_ONE (16384)

/* Wheel indexes of the pose. */
#define GEOM_WHEEL_LEFT  (0U)
#define GEOM_WHEEL_RIGHT (1U)
#define GEOM_WHEELS      (2U)

/* Dead-reckoned pose of the axle middle. The heading is not wrapped, so a
 * pivot can be followed through any number of turns. */
typedef struct
{
    int64_t base_um[GEOM_WHEELS];
    int64_t prev_um[GEOM_WHEELS];
    int64_t x_um;
    int64_t y_um;
    int32_t head_deci;
} geom_pose_t;

/* A local frame: origin and forward direction of a stationary scan. */
typedef struct
{
    int32_t x_mm;
    int32_t y_mm;
    int32_t head_deci;
} geom_frame_t;

int32_t  geom_sin_q14 (int32_t deci);
int32_t  geom_cos_q14 (int32_t deci);
int32_t  geom_tan_q14 (int32_t deci);
int32_t  geom_atan_deci (int32_t num, int32_t den);
int32_t  geom_wrap_deci (int32_t deci);
int32_t  geom_abs (int32_t value);
int32_t  geom_mul_q14 (int32_t value, int32_t q14);
uint32_t geom_isqrt (uint32_t value);

void    geom_pose_reset (geom_pose_t * p_pose, int64_t left_um, int64_t right_um);
void    geom_pose_update (geom_pose_t * p_pose, int64_t left_um, int64_t right_um);
int32_t geom_pose_x_mm (geom_pose_t const * p_pose);
int32_t geom_pose_y_mm (geom_pose_t const * p_pose);
int64_t geom_pose_travel_um (geom_pose_t const * p_pose);

void geom_frame_set (geom_frame_t * p_frame, geom_pose_t const * p_pose);
void geom_to_local (geom_frame_t const * p_frame, int32_t world_x_mm, int32_t world_y_mm,
                    int32_t * p_lat_mm, int32_t * p_along_mm);
void geom_to_world (geom_frame_t const * p_frame, int32_t lat_mm, int32_t along_mm,
                    int32_t * p_world_x_mm, int32_t * p_world_y_mm);

#endif /* OBSTACLE_GEOMETRY_H */

/*** end of file ***/
