/** @file test_pilot_latches.c
 * @brief Host tests for the pilot latches that RESET clears.
 */

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pilot/pilot.h"
#include "pilot/pilot_log.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static uint32_t g_now_ms;

#define TICK_MS      (10U)
#define SHORT_MS     (PILOT_SONAR_RESUME_MS - (5U * TICK_MS))
#define LONG_MS      (PILOT_SONAR_RESUME_MS + (5U * TICK_MS))
#define WINDOW_TICKS (PILOT_SONAR_RESUME_MS / TICK_MS)

/* One tick with a fresh clock and the line under both sensors. */
static void tick(pilot_state_t * p_state, pilot_cmd_t * p_cmd)
{
    pilot_meas_t meas = {0};

    g_now_ms += 10U;
    meas.now_ms       = g_now_ms;
    meas.stamp_ms     = g_now_ms;
    meas.b_line_left  = true;
    meas.b_line_right = true;
    pilot_step(p_state, &meas, p_cmd);
}

/* One tick at the given clock, the line under both sensors. Stale data is
 * wheel feedback older than the limit. */
static void tick_at(pilot_state_t * p_state, pilot_cmd_t * p_cmd,
                    uint32_t now_ms, bool b_fault, bool b_stale, bool b_held)
{
    pilot_meas_t meas = {0};

    g_now_ms = now_ms;
    meas.now_ms        = now_ms;
    meas.stamp_ms      = b_stale ? (now_ms - PILOT_STALE_MS - 1U) : now_ms;
    meas.b_sonar_fault = b_fault;
    meas.b_held        = b_held;
    meas.b_line_left   = true;
    meas.b_line_right  = true;
    pilot_step(p_state, &meas, p_cmd);
}

static void tick_bad(pilot_state_t * p_state, pilot_cmd_t * p_cmd, bool b_fault,
                     bool b_stale)
{
    tick_at(p_state, p_cmd, g_now_ms + TICK_MS, b_fault, b_stale, false);
}

/* Ticks of a bad run that has not reached the window: each keeps its reason. */
static void ticks_bad(pilot_state_t * p_state, pilot_cmd_t * p_cmd, uint32_t ms,
                      bool b_fault, bool b_stale, pilot_reason_t reason)
{
    uint32_t n;

    for (n = 0U; n < (ms / TICK_MS); n++)
    {
        tick_bad(p_state, p_cmd, b_fault, b_stale);
        assert((uint8_t)reason == p_cmd->reason);
        assert((0 == p_cmd->speed_mm_s) && (0 == p_cmd->turn_deci_s));
    }
}

static void ticks_held(pilot_state_t * p_state, pilot_cmd_t * p_cmd, uint32_t n,
                       bool b_stale)
{
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        tick_at(p_state, p_cmd, g_now_ms + TICK_MS, false, b_stale, true);
        assert((uint8_t)PILOT_REASON_HELD == p_cmd->reason);
    }
}

static pilot_reason_t base_of(bool b_fault)
{
    return b_fault ? PILOT_REASON_SONAR : PILOT_REASON_STALE;
}

static pilot_reason_t latched_of(bool b_fault)
{
    return b_fault ? PILOT_REASON_SONAR_LATCHED : PILOT_REASON_STALE_LATCHED;
}

/* A bad run of LONG_MS: the cause is the reason until the window ends, then
 * the latched reason of the same cause. */
static void latch_bad(pilot_state_t * p_state, pilot_cmd_t * p_cmd,
                      bool b_fault)
{
    uint32_t n;

    for (n = 0U; n < (LONG_MS / TICK_MS); n++)
    {
        uint8_t const want = (uint8_t)((n < WINDOW_TICKS) ? base_of(b_fault)
                                       : latched_of(b_fault));

        tick_bad(p_state, p_cmd, b_fault, !b_fault);
        assert(want == p_cmd->reason);
        assert((0 == p_cmd->speed_mm_s) && (0 == p_cmd->turn_deci_s));
    }
}

static void assert_drives(pilot_state_t * p_state, pilot_cmd_t * p_cmd)
{
    tick(p_state, p_cmd);
    assert((uint8_t)PILOT_REASON_NONE == p_cmd->reason);
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour);
    assert(0 != p_cmd->speed_mm_s);
}

static void assert_stopped(pilot_state_t * p_state, pilot_cmd_t * p_cmd,
                           pilot_reason_t reason)
{
    tick(p_state, p_cmd);
    assert((uint8_t)reason == p_cmd->reason);
    assert((uint8_t)PILOT_BEHAVIOUR_SAFETY == p_cmd->behaviour);
    assert((0 == p_cmd->speed_mm_s) && (0 == p_cmd->turn_deci_s));
}

static void test_encoder_cleared(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    pilot_log_t   log;

    pilot_init(&st);
    pilot_log_init(&log);
    st.b_encoder = 1U;
    tick(&st, &cmd);
    assert((uint8_t)PILOT_BEHAVIOUR_SAFETY == cmd.behaviour);
    assert((uint8_t)PILOT_REASON_ENCODER == cmd.reason);
    assert(pilot_log_note(&log, &cmd));

    pilot_clear_latches(&st);
    tick(&st, &cmd);
    assert((uint8_t)PILOT_REASON_ENCODER != cmd.reason);
    /* The decision event fires when the reason leaves ENCODER. */
    assert(pilot_log_note(&log, &cmd));
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
    assert(0 != cmd.speed_mm_s);
    assert(0U == st.b_encoder);
}

static void test_timeout_cleared(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    pilot_log_t   log;

    pilot_init(&st);
    pilot_log_init(&log);
    st.b_timeout = 1U;
    tick(&st, &cmd);
    assert((uint8_t)PILOT_REASON_HUG_TIMEOUT == cmd.reason);
    assert(pilot_log_note(&log, &cmd));

    pilot_clear_latches(&st);
    tick(&st, &cmd);
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
    assert(pilot_log_note(&log, &cmd));
    assert(0U == st.b_timeout);
}

static void test_estop_kept(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    pilot_estop(&st);
    st.b_encoder = 1U;
    st.b_timeout = 1U;
    pilot_clear_latches(&st);
    assert(0U == st.b_encoder);
    assert(0U == st.b_timeout);
    tick(&st, &cmd);
    assert((uint8_t)PILOT_REASON_ESTOP == cmd.reason);
    assert(0 == cmd.speed_mm_s);
}

/* A RESET that arrives while the follower had stopped for a lost line, or
 * after a one-shot turn, starts the follower afresh but keeps the side. */
static void test_clear_keeps_the_side(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    line_follow_set_side(&st.follow, LINE_SIDE_RIGHT);
    st.follow.status = (uint8_t)LINE_FOLLOW_STOPPED;
    st.b_encoder = 1U;
    pilot_clear_latches(&st);
    assert((uint8_t)LINE_SIDE_RIGHT == st.follow.side);
    assert((uint8_t)LINE_FOLLOW_FOLLOWING == st.follow.status);
    tick(&st, &cmd);
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
}

/* RESET while the car runs: the supervisor holds the wheels until START, so
 * the pilot answers HELD with no wheel command, and then follows the line. */
static void test_reset_then_start(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    pilot_meas_t  meas = {0};

    pilot_init(&st);
    st.b_encoder = 1U;
    tick(&st, &cmd);
    assert((uint8_t)PILOT_REASON_ENCODER == cmd.reason);

    pilot_clear_latches(&st);
    g_now_ms += 10U;
    meas.now_ms       = g_now_ms;
    meas.stamp_ms     = g_now_ms;
    meas.b_held       = true;
    meas.b_line_left  = true;
    meas.b_line_right = true;
    pilot_step(&st, &meas, &cmd);
    assert((uint8_t)PILOT_REASON_HELD == cmd.reason);
    assert((0 == cmd.speed_mm_s) && (0 == cmd.turn_deci_s));

    tick(&st, &cmd);
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
    assert(0 != cmd.speed_mm_s);

    pilot_estop(&st);
    st.b_encoder = 1U;
    pilot_clear_latches(&st);
    tick(&st, &cmd);
    assert((uint8_t)PILOT_REASON_ESTOP == cmd.reason);
    assert(0 == cmd.speed_mm_s);
}

/* A fault or stale data that ends inside the window: the pilot drives on by
 * itself, and two such runs do not add up. */
static void test_short_bad_resumes(bool b_fault, pilot_reason_t reason)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    ticks_bad(&st, &cmd, SHORT_MS, b_fault, !b_fault, reason);
    assert_drives(&st, &cmd);
    ticks_bad(&st, &cmd, SHORT_MS, b_fault, !b_fault, reason);
    assert_drives(&st, &cmd);
    assert_drives(&st, &cmd);
}

/* A fault or stale data that outlasts the window latches the stop: good data
 * does not end it, the clear does. */
static void test_long_bad_latches(bool b_fault)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    uint32_t      n;

    pilot_init(&st);
    latch_bad(&st, &cmd, b_fault);
    for (n = 0U; n < 50U; n++)
    {
        assert_stopped(&st, &cmd, latched_of(b_fault));
    }
    pilot_clear_latches(&st);
    assert_drives(&st, &cmd);
    assert_drives(&st, &cmd);
}

/* The window is 1000 ms from the first bad tick: 999 ms resumes. */
static void test_window_edge_resumes(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    tick_at(&st, &cmd, 1000U, true, false, false);
    tick_at(&st, &cmd, 1999U, true, false, false);
    assert((uint8_t)PILOT_REASON_SONAR == cmd.reason);
    tick_at(&st, &cmd, 2000U, false, false, false);
    assert((uint8_t)PILOT_REASON_NONE == cmd.reason);
    assert(0 != cmd.speed_mm_s);
}

/* The window is 1000 ms from the first bad tick: 1000 ms latches. */
static void test_window_edge_latches(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    tick_at(&st, &cmd, 1000U, true, false, false);
    tick_at(&st, &cmd, 1999U, true, false, false);
    assert((uint8_t)PILOT_REASON_SONAR == cmd.reason);
    tick_at(&st, &cmd, 2000U, true, false, false);
    assert((uint8_t)PILOT_REASON_SONAR_LATCHED == cmd.reason);
    tick_at(&st, &cmd, 2010U, false, false, false);
    assert((uint8_t)PILOT_REASON_SONAR_LATCHED == cmd.reason);
    assert(0 == cmd.speed_mm_s);
}

/* A fault that turns into stale data keeps the run. The latch names the cause
 * at the moment it sets: here stale data, as the window ends. */
static void test_fault_then_stale_latches_stale(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    tick_at(&st, &cmd, 1000U, true, false, false);
    assert((uint8_t)PILOT_REASON_SONAR == cmd.reason);
    tick_at(&st, &cmd, 1500U, false, true, false);
    assert((uint8_t)PILOT_REASON_STALE == cmd.reason);
    tick_at(&st, &cmd, 2000U, false, true, false);
    assert((uint8_t)PILOT_REASON_STALE_LATCHED == cmd.reason);
    tick_at(&st, &cmd, 2010U, false, false, false);
    assert((uint8_t)PILOT_REASON_STALE_LATCHED == cmd.reason);
    assert(0 == cmd.speed_mm_s);
}

/* A latched sonar stop: the reason holds across good data, and a fault that
 * comes later in the same run does not change it. */
static void test_sonar_latch_then_stale(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    tick_bad(&st, &cmd, false, true);
    assert((uint8_t)PILOT_REASON_SONAR_LATCHED == cmd.reason);
}

/* After good data the sonar latch still holds, so a stale tick keeps the
 * sonar reason. */
static void test_sonar_latch_good_then_stale(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    assert_stopped(&st, &cmd, PILOT_REASON_SONAR_LATCHED);
    tick_bad(&st, &cmd, false, true);
    assert((uint8_t)PILOT_REASON_SONAR_LATCHED == cmd.reason);
    assert(0 == cmd.speed_mm_s);
}

/* A stale latch: a sonar fault later in the same run does not change it. */
static void test_stale_latch_then_sonar(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, false);
    tick_bad(&st, &cmd, true, false);
    assert((uint8_t)PILOT_REASON_STALE_LATCHED == cmd.reason);
}

/* After good data the stale latch still holds, so a sonar fault keeps the
 * stale reason. */
static void test_stale_latch_good_then_sonar(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, false);
    assert_stopped(&st, &cmd, PILOT_REASON_STALE_LATCHED);
    tick_bad(&st, &cmd, true, false);
    assert((uint8_t)PILOT_REASON_STALE_LATCHED == cmd.reason);
    assert(0 == cmd.speed_mm_s);
}

/* A clear does not undo the emergency stop over a latched sonar stop. */
static void test_latched_bad_keeps_estop(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    pilot_estop(&st);
    pilot_clear_latches(&st);
    assert_stopped(&st, &cmd, PILOT_REASON_ESTOP);
}

/* The emergency stop answers over a latched sonar stop. */
static void test_estop_over_latch(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    pilot_estop(&st);
    assert_stopped(&st, &cmd, PILOT_REASON_ESTOP);
}

/* Stale data while motion holds the wheels is no run: HELD answers, and the
 * car drives on when the hold ends. */
static void test_held_stale_is_no_run(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    tick(&st, &cmd);
    ticks_held(&st, &cmd, 300U, true);
    assert_drives(&st, &cmd);
}

/* A fault that begins during a hold has its own window. */
static void test_fault_after_hold_has_own_window(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    tick(&st, &cmd);
    ticks_held(&st, &cmd, 500U, false);
    ticks_bad(&st, &cmd, SHORT_MS, true, false, PILOT_REASON_SONAR);
    assert_drives(&st, &cmd);
}

/* A run that closed in a hold does not count toward the next one. */
static void test_hold_closes_the_run(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    tick(&st, &cmd);
    ticks_bad(&st, &cmd, SHORT_MS, true, false, PILOT_REASON_SONAR);
    ticks_held(&st, &cmd, 500U, false);
    ticks_bad(&st, &cmd, SHORT_MS, true, false, PILOT_REASON_SONAR);
    assert_drives(&st, &cmd);
}

/* The latch moment is a decision: the log reports it, once. */
static void test_latch_is_reported(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    pilot_log_t   log;
    uint32_t      n;
    uint32_t      reports = 0U;

    pilot_init(&st);
    pilot_log_init(&log);
    for (n = 0U; n < (LONG_MS / TICK_MS); n++)
    {
        tick_bad(&st, &cmd, true, false);
        reports += pilot_log_note(&log, &cmd) ? 1U : 0U;
        if (n == ((PILOT_SONAR_RESUME_MS / TICK_MS) - 2U))
        {
            assert(1U == reports);
        }
    }
    assert(2U == reports);
    assert_stopped(&st, &cmd, PILOT_REASON_SONAR_LATCHED);
    assert(!pilot_log_note(&log, &cmd));
    pilot_clear_latches(&st);
    assert_drives(&st, &cmd);
    assert(pilot_log_note(&log, &cmd));
}

/* START releases the sonar and stale latches. */
static void test_release_sonar(bool b_fault)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, b_fault);
    assert_stopped(&st, &cmd, latched_of(b_fault));
    pilot_release_sonar(&st);
    assert_drives(&st, &cmd);
    assert_drives(&st, &cmd);
}

/* The release leaves the latches that only RESET clears, and the emergency
 * stop. */
static void test_release_keeps_the_others(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    st.b_encoder = 1U;
    pilot_release_sonar(&st);
    assert_stopped(&st, &cmd, PILOT_REASON_ENCODER);

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    st.b_timeout = 1U;
    pilot_release_sonar(&st);
    assert_stopped(&st, &cmd, PILOT_REASON_HUG_TIMEOUT);

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    pilot_estop(&st);
    pilot_release_sonar(&st);
    assert_stopped(&st, &cmd, PILOT_REASON_ESTOP);
}

/* A sonar that is still bad after the release latches again only after a new
 * full window. */
static void test_release_then_still_bad(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    pilot_release_sonar(&st);
    ticks_bad(&st, &cmd, SHORT_MS, true, false, PILOT_REASON_SONAR);
    assert_drives(&st, &cmd);

    latch_bad(&st, &cmd, true);
    assert_stopped(&st, &cmd, PILOT_REASON_SONAR_LATCHED);
}

/* Each supervisor count is compared with its own last-seen value, wrap
 * included: a START never reads as a RESET, or the other way round. */
static void test_count_moved(void)
{
    uint8_t reset_seen = 0U;
    uint8_t start_seen = 0U;

    assert(!pilot_count_moved(&reset_seen, 0U));
    assert(pilot_count_moved(&start_seen, 1U));
    assert(!pilot_count_moved(&reset_seen, 0U));
    assert(!pilot_count_moved(&start_seen, 1U));
    assert(pilot_count_moved(&reset_seen, 1U));
    assert(!pilot_count_moved(&start_seen, 1U));
    assert(!pilot_count_moved(&reset_seen, 1U));

    reset_seen = 255U;
    assert(pilot_count_moved(&reset_seen, 0U));
    assert(reset_seen == 0U);
}

static int run_isolated_case(char const * p_name)
{
    if (strcmp(p_name, "reset-encoder") == 0)
    {
        test_encoder_cleared();
    }
    else if (strcmp(p_name, "reset-hug-timeout") == 0)
    {
        test_timeout_cleared();
    }
    else if (strcmp(p_name, "short-sonar-fault") == 0)
    {
        test_short_bad_resumes(true, PILOT_REASON_SONAR);
    }
    else if (strcmp(p_name, "sonar-window-boundary") == 0)
    {
        test_window_edge_resumes();
        test_window_edge_latches();
    }
    else if (strcmp(p_name, "latched-sonar-fault") == 0)
    {
        test_long_bad_latches(true);
    }
    else if (strcmp(p_name, "held-is-not-sonar-fault") == 0)
    {
        test_fault_after_hold_has_own_window();
    }
    else if (strcmp(p_name, "start-releases-sonar") == 0)
    {
        test_release_sonar(true);
    }
    else
    {
        (void)fprintf(stderr, "Unknown pilot latch case: %s\n", p_name);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int main(int argc, char * argv[])
{
    if (2 == argc)
    {
        return run_isolated_case(argv[1]);
    }
    if (1 != argc)
    {
        return EXIT_FAILURE;
    }
    test_encoder_cleared();
    test_timeout_cleared();
    test_estop_kept();
    test_clear_keeps_the_side();
    test_reset_then_start();
    test_short_bad_resumes(true, PILOT_REASON_SONAR);
    test_short_bad_resumes(false, PILOT_REASON_STALE);
    test_long_bad_latches(true);
    test_long_bad_latches(false);
    test_window_edge_resumes();
    test_window_edge_latches();
    test_fault_then_stale_latches_stale();
    test_latched_bad_keeps_estop();
    test_estop_over_latch();
    test_held_stale_is_no_run();
    test_fault_after_hold_has_own_window();
    test_hold_closes_the_run();
    test_latch_is_reported();
    test_sonar_latch_then_stale();
    test_sonar_latch_good_then_stale();
    test_stale_latch_then_sonar();
    test_stale_latch_good_then_sonar();
    test_release_sonar(true);
    test_release_sonar(false);
    test_release_keeps_the_others();
    test_release_then_still_bad();
    test_count_moved();
    (void)printf("pilot_latches: ok\n");
    return 0;
}

/*** end of file ***/
