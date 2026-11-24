/** @file test_pilot_log.c
 * @brief Host tests for the pilot decision log.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "pilot/pilot_log.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static pilot_cmd_t make_cmd(int16_t speed, int16_t turn, uint8_t beh,
                            uint8_t reason)
{
    pilot_cmd_t c;
    c.speed_mm_s = speed;
    c.turn_deci_s = turn;
    c.behaviour = beh;
    c.reason = reason;
    return c;
}

static void test_first_call_notes(void)
{
    pilot_log_t log;
    pilot_cmd_t c = make_cmd(120, -300, PILOT_BEHAVIOUR_NONE, PILOT_REASON_NONE);

    pilot_log_init(&log);
    assert(sizeof(log) == 3U);
    /* Even the all-zero pair is new before the first note. */
    assert(pilot_log_note(&log, &c));
    assert(!pilot_log_note(&log, &c));
}

static void test_same_pair_does_not_note(void)
{
    pilot_log_t log;
    pilot_cmd_t c = make_cmd(100, 0, PILOT_BEHAVIOUR_FOLLOW_LINE,
                             PILOT_REASON_NONE);

    pilot_log_init(&log);
    assert(pilot_log_note(&log, &c));
    c.speed_mm_s = 200;
    c.turn_deci_s = 50;
    assert(!pilot_log_note(&log, &c));
}

static void test_new_behaviour_or_reason_notes(void)
{
    pilot_log_t log;
    pilot_cmd_t c = make_cmd(100, 0, PILOT_BEHAVIOUR_FOLLOW_LINE,
                             PILOT_REASON_NONE);

    pilot_log_init(&log);
    assert(pilot_log_note(&log, &c));
    c.reason = PILOT_REASON_LINE_LOST_STOP;
    c.speed_mm_s = 0;
    assert(pilot_log_note(&log, &c));
    assert(!pilot_log_note(&log, &c));
    c.behaviour = PILOT_BEHAVIOUR_SAFETY;
    assert(pilot_log_note(&log, &c));
    /* Back to an earlier pair is a change again. */
    c.behaviour = PILOT_BEHAVIOUR_FOLLOW_LINE;
    c.reason = PILOT_REASON_NONE;
    assert(pilot_log_note(&log, &c));
}

static void test_is_following(void)
{
    pilot_log_t log;
    pilot_cmd_t c = make_cmd(100, 0, PILOT_BEHAVIOUR_FOLLOW_LINE,
                             PILOT_REASON_NONE);

    pilot_log_init(&log);
    assert(pilot_log_is_following(&log));
    (void)pilot_log_note(&log, &c);
    assert(pilot_log_is_following(&log));
    c.reason = PILOT_REASON_STUCK;
    (void)pilot_log_note(&log, &c);
    assert(!pilot_log_is_following(&log));
    c.behaviour = PILOT_BEHAVIOUR_HUG;
    c.reason = PILOT_REASON_NONE;
    (void)pilot_log_note(&log, &c);
    assert(!pilot_log_is_following(&log));
    c.behaviour = PILOT_BEHAVIOUR_FOLLOW_LINE;
    (void)pilot_log_note(&log, &c);
    assert(pilot_log_is_following(&log));
}

/* Each new automatic recovery attempt is reported once; a count that falls
 * back (clean travel, START or RESET) reports nothing. */
static void test_recovery_attempts(void)
{
    pilot_log_t log;

    pilot_log_init(&log);
    assert(0U == pilot_log_recovery(&log, 0U));
    assert(1U == pilot_log_recovery(&log, 1U));
    assert(0U == pilot_log_recovery(&log, 1U));
    assert(2U == pilot_log_recovery(&log, 2U));
    assert(0U == pilot_log_recovery(&log, 0U));
    assert(1U == pilot_log_recovery(&log, 1U));
    pilot_log_init(&log);
    assert(1U == pilot_log_recovery(&log, 1U));
}

int main(void)
{
    test_first_call_notes();
    test_same_pair_does_not_note();
    test_new_behaviour_or_reason_notes();
    test_is_following();
    test_recovery_attempts();
    puts("test_pilot_log: 5 tests passed");
    return 0;
}
