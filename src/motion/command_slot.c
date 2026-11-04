/** @file command_slot.c
 * @brief Sensor holds cannot be overwritten by ordinary motion commands.
 */
#include <limits.h>
#include <stddef.h>
#include "command_slot.h"

bool command_slot_init (command_slot_t * p_slot, uint32_t max_ttl_us)
{
    if ((p_slot == NULL) || (max_ttl_us == 0U) ||
        (max_ttl_us > (uint32_t)INT32_MAX))
    {
        return false;
    }
    *p_slot            = (command_slot_t){0};
    p_slot->max_ttl_us = max_ttl_us;
    return true;
}

void command_slot_cancel (command_slot_t * p_slot)
{
    if (p_slot != NULL)
    {
        p_slot->has_pending   = false;
        p_slot->brake_pending = true;
    }
}

void command_slot_stop (command_slot_t * p_slot)
{
    if (p_slot != NULL)
    {
        command_slot_cancel(p_slot);
        p_slot->stopped = true;
    }
}

bool command_slot_hold (command_slot_t * p_slot, uint32_t reason, bool held,
                        uint32_t now_us)
{
    uint32_t before;

    if ((p_slot == NULL) || (reason == 0U) ||
        ((reason & ~MOTION_HOLD_ALL) != 0U) || ((reason & (reason - 1U)) != 0U))
    {
        return false;
    }
    before             = p_slot->hold_mask;
    p_slot->hold_mask = held ? before | reason : before & ~reason;
    if (before != p_slot->hold_mask)
    {
        command_slot_cancel(p_slot);
        if (p_slot->hold_mask == 0U)
        {
            p_slot->released_us   = now_us;
            p_slot->require_fresh = true;
        }
    }
    return true;
}

command_slot_result_t command_slot_post (command_slot_t *         p_slot,
                                         const motion_command_t * p_command,
                                         uint32_t                 now_us)
{
    if ((p_slot == NULL) || p_slot->stopped)
    {
        return COMMAND_SLOT_STOPPED;
    }
    if (p_slot->hold_mask != 0U)
    {
        return COMMAND_SLOT_HELD;
    }
    if (p_slot->require_fresh &&
        ((uint32_t)(now_us - p_slot->released_us) >= p_slot->max_ttl_us))
    {
        /* Every possible pre-release lease has expired. Keeping the barrier
         * forever would otherwise confuse long uptime with timestamp wrap. */
        p_slot->require_fresh = false;
    }
    if ((p_command == NULL) || (p_command->ttl_us == 0U) ||
        (p_command->ttl_us > p_slot->max_ttl_us) ||
        ((uint32_t)(now_us - p_command->issued_us) >= p_command->ttl_us) ||
        (p_slot->require_fresh && ((uint32_t)(now_us - p_command->issued_us) >
                                    (uint32_t)(now_us - p_slot->released_us))))
    {
        command_slot_cancel(p_slot);
        return COMMAND_SLOT_STALE;
    }
    p_slot->pending     = *p_command;
    p_slot->has_pending = true;
    return COMMAND_SLOT_ACCEPTED;
}

bool command_slot_take (command_slot_t * p_slot, uint32_t now_us,
                        motion_command_t * p_command, bool * p_stop)
{
    if ((p_slot == NULL) || (p_command == NULL) || (p_stop == NULL))
    {
        return false;
    }
    *p_stop = p_slot->stopped;
    if (*p_stop)
    {
        return false;
    }
    if (p_slot->brake_pending || p_slot->hold_mask != 0U)
    {
        *p_command =
            (motion_command_t){0U, now_us, p_slot->max_ttl_us, {0, 0}};
        p_slot->brake_pending = false;
        return true;
    }
    if (p_slot->has_pending)
    {
        *p_command           = p_slot->pending;
        p_slot->has_pending = false;
        return true;
    }
    return false;
}
