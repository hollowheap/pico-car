#include "navigation/mission_events.h"

static bool apply_event (mission_state_t * p_state, mission_event_t event)
{
    mission_state_t next = *p_state;

    switch (event)
    {
        case MISSION_EVENT_START:
            if (*p_state == MISSION_STATE_READY)
            {
                next = MISSION_STATE_RUNNING;
            }
            break;
        case MISSION_EVENT_STOP:
            if (*p_state != MISSION_STATE_STOPPED)
            {
                next = MISSION_STATE_STOPPED;
            }
            break;
        case MISSION_EVENT_RESET:
            if (*p_state == MISSION_STATE_STOPPED)
            {
                next = MISSION_STATE_READY;
            }
            break;
        case MISSION_EVENT_FAULT:
            next = MISSION_STATE_STOPPED;
            break;
        default:
            return false;
    }

    if (next == *p_state)
    {
        return false;
    }
    *p_state = next;
    return true;
}

mission_state_t mission_apply_events (mission_state_t state, uint32_t events,
                                      bool * p_stop_requested)
{
    const bool dominant =
        (events & (uint32_t)(MISSION_EVENT_STOP | MISSION_EVENT_FAULT)) != 0U;
    mission_state_t next = state;

    if (p_stop_requested != 0)
    {
        *p_stop_requested = dominant && (state != MISSION_STATE_STOPPED);
    }

    if (dominant)
    {
        if ((events & (uint32_t)MISSION_EVENT_STOP) != 0U)
        {
            (void)apply_event(&next, MISSION_EVENT_STOP);
        }
        if ((events & (uint32_t)MISSION_EVENT_FAULT) != 0U)
        {
            (void)apply_event(&next, MISSION_EVENT_FAULT);
        }
    }
    else
    {
        if ((events & (uint32_t)MISSION_EVENT_RESET) != 0U)
        {
            (void)apply_event(&next, MISSION_EVENT_RESET);
        }
        if ((events & (uint32_t)MISSION_EVENT_START) != 0U)
        {
            (void)apply_event(&next, MISSION_EVENT_START);
        }
    }

    return next;
}
