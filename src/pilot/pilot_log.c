/** @file pilot_log.c
 * @brief Pilot decision log. See pilot_log.h.
 */

#include "pilot/pilot_log.h"

void pilot_log_init(pilot_log_t * p_log)
{
    p_log->behaviour = (uint8_t)PILOT_LOG_NONE;
    p_log->reason    = (uint8_t)PILOT_REASON_NONE;
}

bool pilot_log_note(pilot_log_t * p_log, pilot_cmd_t const * p_cmd)
{
    bool const b_changed = (p_log->behaviour != p_cmd->behaviour) ||
                           (p_log->reason != p_cmd->reason);

    p_log->behaviour = p_cmd->behaviour;
    p_log->reason    = p_cmd->reason;

    return b_changed;
}

bool pilot_log_is_following(pilot_log_t const * p_log)
{
    return ((uint8_t)PILOT_LOG_NONE == p_log->behaviour) ||
           (((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_log->behaviour) &&
            ((uint8_t)PILOT_REASON_NONE == p_log->reason));
}

/*** end of file ***/
