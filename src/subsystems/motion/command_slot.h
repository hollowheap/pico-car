/** @file command_slot.h
 * @brief Bounded command/hold/stop policy; caller supplies synchronisation.
 */
#ifndef COMMAND_SLOT_H
#define COMMAND_SLOT_H

#include "controller.h"
#include "motion/types.h"

typedef struct
{
    motion_command_t pending;
    uint32_t         hold_mask;
    uint32_t         released_us;
    uint32_t         max_ttl_us;
    bool             has_pending;
    bool             brake_pending;
    bool             require_fresh;
    bool             stopped;
} command_slot_t;

typedef enum
{
    COMMAND_SLOT_ACCEPTED = 0,
    COMMAND_SLOT_HELD,
    COMMAND_SLOT_STOPPED,
    COMMAND_SLOT_STALE
} command_slot_result_t;

bool command_slot_init (command_slot_t * p_slot, uint32_t max_ttl_us);
void command_slot_cancel (command_slot_t * p_slot); /* One normal brake. */
void command_slot_stop (command_slot_t * p_slot);   /* Terminal. */
bool command_slot_hold (command_slot_t * p_slot, uint32_t reason, bool held,
                        uint32_t now_us);
command_slot_result_t command_slot_post (command_slot_t *         p_slot,
                                         const motion_command_t * p_command,
                                         uint32_t                 now_us);

/* Priority: terminal stop -> held/latched normal brake -> latest motion.
 * A hold that was set AND cleared between polls still delivers one brake.
 * A fresh post-clear motion request waits in the slot until a later take.
 * No post-clear cached request from before release can resume old movement.
 * now/issued/released times must be within the documented half-range window;
 * producer leases must be nonzero <=INT32_MAX. Not thread-safe by itself. */
bool command_slot_take (command_slot_t * p_slot, uint32_t now_us,
                        motion_command_t * p_command, bool * p_stop);

#endif /* COMMAND_SLOT_H */
