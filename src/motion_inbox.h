/** @file motion_inbox.h
 * @brief Bounded command/hold/stop policy; caller supplies synchronisation.
 */
#ifndef MOTION_INBOX_H
#define MOTION_INBOX_H

#include "motion_control.h"

#define MOTION_HOLD_OBSTACLE UINT32_C(1)
#define MOTION_HOLD_LINE_LOST UINT32_C(2)
#define MOTION_HOLD_OPERATOR UINT32_C(4)
#define MOTION_HOLD_ALL UINT32_C(7)

typedef struct
{
    motion_command_t pending;
    uint32_t hold_mask;
    uint32_t released_us;
    uint32_t max_ttl_us;
    bool has_pending;
    bool brake_pending;
    bool require_fresh;
    bool stopped;
} motion_inbox_t;

typedef enum
{
    MOTION_INBOX_ACCEPTED = 0,
    MOTION_INBOX_HELD,
    MOTION_INBOX_STOPPED,
    MOTION_INBOX_STALE
} motion_inbox_result_t;

bool motion_inbox_init (motion_inbox_t * p_inbox, uint32_t max_ttl_us);
void motion_inbox_cancel (motion_inbox_t * p_inbox); /* One normal brake. */
void motion_inbox_stop (motion_inbox_t * p_inbox);   /* Terminal. */
bool motion_inbox_hold (motion_inbox_t * p_inbox, uint32_t reason,
                        bool held, uint32_t now_us);
motion_inbox_result_t motion_inbox_post (motion_inbox_t * p_inbox,
    const motion_command_t * p_command, uint32_t now_us);

/* Priority: terminal stop -> held/latched normal brake -> latest motion.
 * A hold that was set AND cleared between polls still delivers one brake.
 * A fresh post-clear motion request waits in the slot until a later take.
 * No post-clear cached request from before release can resume old movement.
 * now/issued/released times must be within the documented half-range window;
 * producer leases must be nonzero <=INT32_MAX. Not thread-safe by itself. */
bool motion_inbox_take (motion_inbox_t * p_inbox, uint32_t now_us,
    motion_command_t * p_command, bool * p_stop);

#endif /* MOTION_INBOX_H */
