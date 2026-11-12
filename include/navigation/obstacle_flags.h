/*
 *	obstacle_flags.h
 *	Warning flags in the obstacle snapshot (the flags byte of the o packet).
 *	Kernel-free, so host tests and the supervisor can include it. The flags
 *	never stop the car.
 */

#ifndef NAVIGATION_OBSTACLE_FLAGS_H
#define NAVIGATION_OBSTACLE_FLAGS_H

#define OBSTACLE_FLAG_WALL         (1U << 0U) /* the plan follows a wall           */
#define OBSTACLE_FLAG_ECHO_NOISY   (1U << 3U) /* P1: ranges jump while still       */
#define OBSTACLE_FLAG_ECHO_INCONSISTENT (1U << 4U) /* P2: range disagrees with travel */
#define OBSTACLE_FLAG_BEAM_BLOCKED (1U << 5U) /* P3: something is in the beam at boot */
#define OBSTACLE_FLAG_SCAN_NOISY   (1U << 6U) /* P5: many slots needed retries     */

#endif /* NAVIGATION_OBSTACLE_FLAGS_H */
