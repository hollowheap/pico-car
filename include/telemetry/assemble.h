/*
 *	assemble.h
 *	Fill telemetry packet structs from the public snapshots of the other
 *	subsystems (D110, S46). Pure C: no kernel call, no hardware, so a host
 *	test covers every mapping. The composition root reads the snapshots and
 *	calls these; nothing here reads a subsystem itself.
 *
 *	Every snapshot pointer may be NULL (the subsystem is not running). The
 *	packet then carries zeros, and the heading carries 0xFFFF (invalid).
 */

#ifndef TELEMETRY_ASSEMBLE_H
#define TELEMETRY_ASSEMBLE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "physics/physics.h"
#include "telemetry/packet.h"

/* Heading value that means "no valid heading". */
#define TELEMETRY_HEADING_INVALID (0xFFFFu)

/* 5 Hz status frame.
   faults is the level word of the fault register. The estop bit is added
   here from the motion latch. line_bits: bit 0 left, bit 1 right, bit 2
   barcode. b_turning is true while a one-shot turn runs. */
void telemetry_assemble_status (telemetry_status_t * p_out,
                                const motion_status_t * p_motion,
                                const motion_feedback_t * p_feedback,
                                const physics_status_t * p_physics,
                                const obstacle_status_t * p_obstacle,
                                uint16_t faults, uint16_t seq,
                                uint8_t line_bits, bool b_turning);

/* 5 Hz motion frame. Speeds are zero when the feedback is stale. */
void telemetry_assemble_motion (telemetry_motion_t * p_out,
                                const motion_status_t * p_motion,
                                const motion_feedback_t * p_feedback,
                                const motion_diag_t * p_diag);

/* 5 Hz physics frame. roll, az, amag and attitude_state are zero until the
   attitude module fills them. */
void telemetry_assemble_physics (telemetry_physics_t * p_out,
                                 const physics_status_t * p_physics);

/* 2 Hz obstacle frame. A NULL plan leaves the plan fields at zero (no scan
   has finished yet). */
void telemetry_assemble_obstacle (telemetry_obstacle_t * p_out,
                                  const obstacle_status_t * p_obstacle,
                                  const obstacle_plan_summary_t * p_plan);

/* Mean wheel travel in whole millimetres, never negative. */
uint32_t telemetry_odometer_mm (const motion_feedback_t * p_feedback);

#endif /* TELEMETRY_ASSEMBLE_H */
