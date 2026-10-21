/** @file guidance_replay.c
 * @brief Fixed bench inputs, never linked into the team's default main.
 */
#include <stddef.h>
#include "guidance_replay.h"

bool guidance_replay_frame (uint32_t elapsed_us, guidance_replay_frame_t * p_frame)
{
    static const uint32_t ends[GUIDANCE_REPLAY_STAGES] = {
        600000U, 1200000U, 1800000U, 2000000U, 2200000U, 2400000U, 3000000U
    };
    static const motion_drive_action_t actions[GUIDANCE_REPLAY_STAGES] = {
        MOTION_DRIVE_STRAIGHT, MOTION_STEER_LEFT, MOTION_STEER_RIGHT,
        MOTION_DRIVE_STRAIGHT, MOTION_DRIVE_STRAIGHT, MOTION_DRIVE_STRAIGHT,
        MOTION_DRIVE_STRAIGHT
    };
    static const uint32_t holds[GUIDANCE_REPLAY_STAGES] = {
        0U, 0U, 0U, MOTION_HOLD_OBSTACLE,
        MOTION_HOLD_OBSTACLE | MOTION_HOLD_LINE_LOST, MOTION_HOLD_LINE_LOST, 0U
    };
    unsigned int stage;

    if (p_frame == NULL)
    {
        return false;
    }
    for (stage = 0U; stage < GUIDANCE_REPLAY_STAGES; stage++)
    {
        if (elapsed_us < ends[stage])
        {
            *p_frame = (guidance_replay_frame_t){actions[stage], holds[stage], stage};
            return true;
        }
    }
    return false;
}
