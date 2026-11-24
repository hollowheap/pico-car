/** @file obstacle_nav.h
 * @brief Initialize the optional legacy route controller and its sonar links.
 */
#ifndef OBSTACLE_NAV_H
#define OBSTACLE_NAV_H

#include "navigation/obstacle.h"

ER obstacle_nav_init (obstacle_distance_fn_t p_get_distance);

#endif /* OBSTACLE_NAV_H */
