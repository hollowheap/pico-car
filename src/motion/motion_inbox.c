/** @file motion_inbox.c
 * @brief Sensor holds cannot be overwritten by ordinary motion commands.
 */
#include <limits.h>
#include <stddef.h>
#include "motion_inbox.h"

bool motion_inbox_init (motion_inbox_t * p_inbox, uint32_t max_ttl_us)
{
    if ((p_inbox == NULL) || (max_ttl_us == 0U) ||
        (max_ttl_us > (uint32_t)INT32_MAX))
    {
        return false;
    }
    *p_inbox            = (motion_inbox_t){0};
    p_inbox->max_ttl_us = max_ttl_us;
    return true;
}

void motion_inbox_cancel (motion_inbox_t * p_inbox)
{
    if (p_inbox != NULL)
    {
        p_inbox->has_pending   = false;
        p_inbox->brake_pending = true;
    }
}

void motion_inbox_stop (motion_inbox_t * p_inbox)
{
    if (p_inbox != NULL)
    {
        motion_inbox_cancel(p_inbox);
        p_inbox->stopped = true;
    }
}

bool motion_inbox_hold (motion_inbox_t * p_inbox, uint32_t reason, bool held,
                        uint32_t now_us)
{
    uint32_t before;

    if ((p_inbox == NULL) || (reason == 0U) ||
        ((reason & ~MOTION_HOLD_ALL) != 0U) || ((reason & (reason - 1U)) != 0U))
    {
        return false;
    }
    before             = p_inbox->hold_mask;
    p_inbox->hold_mask = held ? before | reason : before & ~reason;
    if (before != p_inbox->hold_mask)
    {
        motion_inbox_cancel(p_inbox);
        if (p_inbox->hold_mask == 0U)
        {
            p_inbox->released_us   = now_us;
            p_inbox->require_fresh = true;
        }
    }
    return true;
}

motion_inbox_result_t motion_inbox_post (motion_inbox_t *         p_inbox,
                                         const motion_command_t * p_command,
                                         uint32_t                 now_us)
{
    if ((p_inbox == NULL) || p_inbox->stopped)
    {
        return MOTION_INBOX_STOPPED;
    }
    if (p_inbox->hold_mask != 0U)
    {
        return MOTION_INBOX_HELD;
    }
    if (p_inbox->require_fresh &&
        ((uint32_t)(now_us - p_inbox->released_us) >= p_inbox->max_ttl_us))
    {
        /* Every possible pre-release lease has expired. Keeping the barrier
         * forever would otherwise confuse long uptime with timestamp wrap. */
        p_inbox->require_fresh = false;
    }
    if ((p_command == NULL) || (p_command->ttl_us == 0U) ||
        (p_command->ttl_us > p_inbox->max_ttl_us) ||
        ((uint32_t)(now_us - p_command->issued_us) >= p_command->ttl_us) ||
        (p_inbox->require_fresh && ((uint32_t)(now_us - p_command->issued_us) >
                                    (uint32_t)(now_us - p_inbox->released_us))))
    {
        motion_inbox_cancel(p_inbox);
        return MOTION_INBOX_STALE;
    }
    p_inbox->pending     = *p_command;
    p_inbox->has_pending = true;
    return MOTION_INBOX_ACCEPTED;
}

bool motion_inbox_take (motion_inbox_t * p_inbox, uint32_t now_us,
                        motion_command_t * p_command, bool * p_stop)
{
    if ((p_inbox == NULL) || (p_command == NULL) || (p_stop == NULL))
    {
        return false;
    }
    *p_stop = p_inbox->stopped;
    if (*p_stop)
    {
        return false;
    }
    if (p_inbox->brake_pending || p_inbox->hold_mask != 0U)
    {
        *p_command =
            (motion_command_t){0U, now_us, p_inbox->max_ttl_us, {0, 0}};
        p_inbox->brake_pending = false;
        return true;
    }
    if (p_inbox->has_pending)
    {
        *p_command           = p_inbox->pending;
        p_inbox->has_pending = false;
        return true;
    }
    return false;
}
