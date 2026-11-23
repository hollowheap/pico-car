/** @file geometry.h
 *
 * @brief Integer trigonometry and vector arithmetic. Angles use deci-degrees
 *        and trigonometric results use Q14 (16384 = 1.0).
 */

#ifndef OBSTACLE_GEOMETRY_H
#define OBSTACLE_GEOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Q14 value of 1.0. */
#define GEOM_Q14_ONE (16384)

int32_t  geom_sin_q14 (int32_t deci);
int32_t  geom_cos_q14 (int32_t deci);
int32_t  geom_tan_q14 (int32_t deci);
int32_t  geom_atan_deci (int32_t num, int32_t den);
int32_t  geom_wrap_deci (int32_t deci);
int32_t  geom_abs (int32_t value);
int32_t  geom_mul_q14 (int32_t value, int32_t q14);
uint32_t geom_isqrt (uint32_t value);
/* Dot product of (dx, dy) with a unit vector: given as Q14 cos and sin, or as
 * an angle. */
int32_t  geom_dot_q14 (int32_t cos_q14, int32_t sin_q14, int32_t dx_mm, int32_t dy_mm);
int32_t  geom_dot_mm (int32_t angle_deci, int32_t dx_mm, int32_t dy_mm);

#endif /* OBSTACLE_GEOMETRY_H */

/*** end of file ***/
