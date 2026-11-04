#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

#include <stdint.h>

typedef enum
{
    MOTION_IDLE = 0,
    MOTION_RUNNING,
    MOTION_REVERSING,
    MOTION_STOPPED,
    MOTION_FAULT
} motion_state_t;

typedef enum
{
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_COMMAND,
    MOTION_FAULT_EXPIRED,
    MOTION_FAULT_FEEDBACK,
    MOTION_FAULT_GAP,
    MOTION_FAULT_NO_EDGES,
    MOTION_FAULT_OUTPUT
} motion_fault_t;

typedef enum
{
    GUIDANCE_STRAIGHT = 0,
    GUIDANCE_STEER_LEFT,
    GUIDANCE_STEER_RIGHT,
    GUIDANCE_HOLD
} guidance_action_t;

typedef struct
{
    guidance_action_t action;
    int32_t           speed_um_s;
    int32_t           inside_um_s;
    uint32_t          sensed_us;
    uint32_t          valid_for_us;
} guidance_request_t;

#define MOTION_HOLD_OBSTACLE  UINT32_C(1)
#define MOTION_HOLD_LINE_LOST UINT32_C(2)
#define MOTION_HOLD_OPERATOR  UINT32_C(4)
#define MOTION_HOLD_ALL       UINT32_C(7)

#endif /* MOTION_TYPES_H */
