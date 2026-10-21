/** @file motion_request.h
 * @brief Decoded sensor intents; Buddy3 owns the IR polarity/truth table.
 */
#ifndef MOTION_REQUEST_H
#define MOTION_REQUEST_H

#include "motion_control.h"

typedef enum
{
    MOTION_DRIVE_STRAIGHT = 0,
    MOTION_STEER_LEFT,
    MOTION_STEER_RIGHT,
    MOTION_DRIVE_HOLD
} motion_drive_action_t;

typedef struct
{
    motion_drive_action_t action;
    int32_t speed_um_s;
    int32_t inside_um_s; /* Left/right steering: 0..speed, no pivot/reverse. */
    uint32_t sensed_us;  /* Timestamp when the decision's sensor data was read. */
    uint32_t valid_for_us;
} motion_drive_request_t;

/* Produces signed wheel targets with the ORIGINAL sensor timestamp/lease.
 * Straight always gives identical targets. Left slows left, right slows right.
 * No wheel geometry/distance input. Failure leaves output unchanged.
 * Limits are per-wheel motor-service limits, not a guessed operating speed. */
bool motion_request_plan (const motion_drive_request_t * p_request,
    const uint32_t limits_um_s[ENCODER_WHEELS], uint32_t max_ttl_us,
    uint32_t now_us, motion_command_t * p_command);

#endif /* MOTION_REQUEST_H */
