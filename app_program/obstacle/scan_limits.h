#ifndef OBSTACLE_SCAN_LIMITS_H
#define OBSTACLE_SCAN_LIMITS_H

#include "platform/chassis.h"

/* Round the outside tire edge away from the axle midpoint for clearance. */
#define OBSTACLE_CAR_HALF_W_MM ((CHASSIS_OUTER_WIDTH_MM + 1U) / 2U)

/* [Default] Smallest clearance the out-leg may leave to the obstacle. */
#define OBSTACLE_PLAN_MARGIN_MM (25U)

#endif /* OBSTACLE_SCAN_LIMITS_H */
