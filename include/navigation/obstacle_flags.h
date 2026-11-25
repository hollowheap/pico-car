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

/* Fault word, 9 bits: the obstacle task is its only writer. */
#define OBSTACLE_FAULT_NO_ECHO      (1U << 0U)
#define OBSTACLE_FAULT_ECHO_STUCK   (1U << 1U)
#define OBSTACLE_FAULT_SCAN_TIMEOUT (1U << 2U)
#define OBSTACLE_FAULT_NO_SAFE_PATH (1U << 3U)
#define OBSTACLE_FAULT_COLLISION    (1U << 4U)
#define OBSTACLE_FAULT_HANDOVER     (1U << 5U)
#define OBSTACLE_FAULT_BYPASS_LIMIT (1U << 6U)
#define OBSTACLE_FAULT_LINE_TIMEOUT (1U << 7U)
#define OBSTACLE_FAULT_ECHO_NOISE   (1U << 8U)

/* Telemetry obstacle faults bits 12 to 14: obstacle_status_t.stop_reason,
 * why the bypass gave up when the cause is BYPASS_GAVE_UP. The fault bits
 * stay in bits 0 to 8. */
#define OBSTACLE_STOP_REASON_SH (12U)
#define OBSTACLE_STOP_REASON_MK (0x7U)

#endif /* NAVIGATION_OBSTACLE_FLAGS_H */
