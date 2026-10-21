/** @file guidance_replay.h
 * @brief Synthetic decisions for the opt-in wheels-up service check only.
 */
#ifndef GUIDANCE_REPLAY_H
#define GUIDANCE_REPLAY_H

#include "motion_request.h"
#include "motion_inbox.h"

#define GUIDANCE_REPLAY_STAGES 7U

typedef struct
{
    motion_drive_action_t action;
    uint32_t hold_mask;
    unsigned int stage;
} guidance_replay_frame_t;

/* Three seconds: straight, left, right, obstacle, both holds, line-only,
 * fresh straight. No real sensor polarity, geometry or turn-angle claim.
 * False at/after the deadline; output is unchanged on failure. */
bool guidance_replay_frame (uint32_t elapsed_us, guidance_replay_frame_t * p_frame);

#endif /* GUIDANCE_REPLAY_H */
