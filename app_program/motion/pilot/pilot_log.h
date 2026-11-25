/** @file pilot_log.h
 * @brief Pilot decision log: the last behaviour and reason.
 *
 * Pure C, no kernel, driver or floating point. The caller prints a decision
 * and posts its event only when the behaviour or the reason of the command
 * differs from the last one noted, so the output holds decisions and not
 * ticks.
 */

#ifndef PILOT_PILOT_LOG_H
#define PILOT_PILOT_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "motion/pilot/pilot.h"
#include "platform/static_assert.h"

typedef struct
{
    uint8_t behaviour;      /* pilot_behaviour_t, or PILOT_LOG_NONE before the first note */
    uint8_t reason;         /* pilot_reason_t    */
    uint8_t recoveries;     /* pilot_state_t.recoveries at the last call */
} pilot_log_t;

STATIC_ASSERT(sizeof(pilot_log_t) == 3U, "pilot_log_t size");

/** Nothing noted yet: no behaviour has this value. */
#define PILOT_LOG_NONE       (0xFFU)

/** Forget the last decision. */
void pilot_log_init(pilot_log_t * p_log);

/**
 * Offer the latest command.
 *
 * @return true when its behaviour or reason differs from the last one noted
 *         (the caller prints it and posts the event); the first call always
 *         does.
 */
bool pilot_log_note(pilot_log_t * p_log, pilot_cmd_t const * p_cmd);

/**
 * Offer the pilot's automatic recovery count.
 *
 * @return the attempt number when the count rose since the last call (the
 *         caller posts the recovery event), else 0.
 */
uint8_t pilot_log_recovery(pilot_log_t * p_log, uint8_t recoveries);

/**
 * Whether the last decision was plain line following. True before the first
 * note.
 */
bool pilot_log_is_following(pilot_log_t const * p_log);

#endif /* PILOT_PILOT_LOG_H */

/*** end of file ***/
