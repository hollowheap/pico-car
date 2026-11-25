/** @file test_pilot_latches.c
 * @brief Host tests for the pilot latches that RESET clears.
 */

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "navigation/obstacle_ping.h"
#include "pilot/pilot.h"
#include "pilot/pilot_log.h"
#include "telemetry/hug_word.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static uint32_t g_now_ms;

#define TICK_MS      (10U)
#define TEST_PING_MS (60U)
#define SHORT_MS     (PILOT_SONAR_RESUME_MS - (5U * TICK_MS))
#define LONG_MS      (PILOT_SONAR_RESUME_MS + (5U * TICK_MS))
#define WINDOW_TICKS (PILOT_SONAR_RESUME_MS / TICK_MS)
/* Automatic recovery: the clock at the latch, a wait past the rest, the
 * crawl ticks a contact needs at most, the pings that end the latch and the
 * back-off bound. */
#define RECOVER_START_MS   (1000U)
#define RECOVER_WAIT_MS    (2U * PILOT_RECOVER_MS)
#define CRAWL_TICKS_MAX    (100U)
#define PING_RESUME_TICKS  (40U)
#define BACKOFF_LIMIT_UM   (2 * PILOT_BACKOFF_UM)

/* A fresh wheel update without a new sonar observation. */
static void tick(pilot_state_t * p_state, pilot_cmd_t * p_cmd)
{
    pilot_meas_t meas = {0};

    g_now_ms += 10U;
    meas.now_ms       = g_now_ms;
    meas.stamp_ms     = g_now_ms;
    meas.b_line_left  = true;
    meas.b_line_right = true;
    meas.b_bounded_course = true;
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
    meas.b_bounded_course = true;
    if (!b_fault)
    {
        meas.ping_echo = LOOK_ECHO_OPEN;
        meas.ping_seq = (uint8_t)((now_ms / TEST_PING_MS) & OBSTACLE_PING_SEQ_MK);
    }
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
    tick_at(p_state, p_cmd, g_now_ms + TICK_MS, false, false, false);
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

    st.b_encoder = 1U;
    st.b_timeout = 1U;
    pilot_start_run(&st);
    assert_drives(&st, &cmd);

    pilot_estop(&st);
    pilot_start_run(&st);
    assert_stopped(&st, &cmd, PILOT_REASON_ESTOP);
}

static void test_no_guide_after_completed_sweep(void)
{
    int32_t const wheel_um[] = {0, 28150, -28150, 0};
    for (unsigned near = 0; near < 2; ++near)
    {
        pilot_state_t pilot;
        pilot_cmd_t cmd;
        pilot_meas_t m = {.ping_echo = LOOK_ECHO_OPEN, .ping_seq = 1U};
        pilot_init(&pilot);
        line_follow_start_local_search(&pilot.follow);
        for (unsigned i = 0; i < sizeof(wheel_um) / sizeof(wheel_um[0]); ++i)
        {
            m.left_um = -wheel_um[i];
            m.right_um = wheel_um[i];
            m.now_ms = m.stamp_ms = i * 10U;
            pilot_step(&pilot, &m, &cmd);
        }
        assert(line_follow_local_missed(&pilot.follow));
        m.now_ms = m.stamp_ms = 40U;
        m.b_stationary = true;
        m.ping_seq++;
        m.ping_echo = (uint8_t)(near ? LOOK_ECHO_VALID : LOOK_ECHO_OPEN);
        m.ping_mm = near ? 205 : 0;
        pilot_step(&pilot, &m, &cmd);
        assert(!cmd.b_rejoined);
        if (near)
        {
            assert(!cmd.speed_mm_s && !cmd.turn_deci_s && pilot.avoidance.b_course_hold);
            m.now_ms = m.stamp_ms = 50U;
            pilot_step(&pilot, &m, &cmd);
            assert(cmd.reason == PILOT_REASON_COURSE_RECOVERY);
        }
        else
        {
            assert(cmd.speed_mm_s > 0 && cmd.reason == PILOT_REASON_NONE);
        }
    }
}

static void test_local_search_interruption(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t meas = {.ping_echo = LOOK_ECHO_OPEN, .ping_seq = 1U};

    pilot_init(&pilot);
    line_follow_start_local_search(&pilot.follow);
    pilot_step(&pilot, &meas, &cmd);
    assert(PILOT_REASON_REJOIN_SEARCH == cmd.reason && 0 == cmd.speed_mm_s && 0 != cmd.turn_deci_s);
    meas.now_ms = 10U;
    meas.stamp_ms = 10U;
    meas.ping_seq++;
    meas.ping_echo = LOOK_ECHO_TOO_CLOSE;
    meas.b_line_left = true;
    pilot_step(&pilot, &meas, &cmd);
    assert(PILOT_REASON_REJOIN_STOP == cmd.reason && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
    pilot_clear_latches(&pilot);
    meas.now_ms = 20U;
    meas.stamp_ms = 20U;
    meas.ping_seq++;
    meas.ping_echo = LOOK_ECHO_OPEN;
    pilot_step(&pilot, &meas, &cmd);
    assert(PILOT_REASON_REJOIN_STOP == cmd.reason && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
    pilot_start_run(&pilot);
    pilot_step(&pilot, &meas, &cmd);
    assert(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
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

/* One tick with fresh pings and the given mean travel. */
static void tick_travel(pilot_state_t * p_state, pilot_cmd_t * p_cmd, int64_t travel_um)
{
    pilot_meas_t meas = {0};

    g_now_ms += TICK_MS;
    meas.now_ms       = g_now_ms;
    meas.stamp_ms     = g_now_ms;
    meas.left_um      = travel_um;
    meas.right_um     = travel_um;
    meas.b_line_left  = true;
    meas.b_line_right = true;
    meas.ping_echo    = LOOK_ECHO_OPEN;
    meas.ping_seq     = (uint8_t)((g_now_ms / TEST_PING_MS) & OBSTACLE_PING_SEQ_MK);
    pilot_step(p_state, &meas, p_cmd);
}

typedef enum
{
    LATCH_ENCODER = 0,
    LATCH_TIMEOUT,
    LATCH_COURSE
} latch_kind_t;

static void set_latch(pilot_state_t * p_state, latch_kind_t kind)
{
    if (LATCH_ENCODER == kind)
    {
        p_state->b_encoder = 1U;
    }
    else if (LATCH_TIMEOUT == kind)
    {
        p_state->b_timeout = 1U;
    }
    else
    {
        p_state->avoidance.b_course_stuck = true;
    }
}

static bool latch_set(pilot_state_t const * p_state)
{
    return (0U != p_state->b_encoder) || (0U != p_state->b_timeout) || p_state->avoidance.b_course_stuck;
}

/* Hold the latch for the rest time; return whether it ended by itself. */
static bool rest_in_latch(pilot_state_t * p_state, pilot_cmd_t * p_cmd, int64_t travel_um)
{
    for (uint32_t ms = 0U; ms < RECOVER_WAIT_MS; ms += TICK_MS)
    {
        tick_travel(p_state, p_cmd, travel_um);
        if (!latch_set(p_state))
        {
            assert(ms + TICK_MS >= PILOT_RECOVER_MS);
            return true;
        }
        assert((uint8_t)PILOT_BEHAVIOUR_SAFETY == p_cmd->behaviour);
        assert((0 == p_cmd->speed_mm_s) && (0 == p_cmd->turn_deci_s));
    }
    return false;
}

/* ENCODER, HUG_TIMEOUT and COURSE_CLEARANCE end by themselves after a rest,
 * a bounded number of times; then START is needed. Clean travel restores
 * the attempts. */
static void test_latches_recover(latch_kind_t kind)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    g_now_ms = RECOVER_START_MS;
    pilot_init(&st);
    for (unsigned attempt = 0U; attempt < PILOT_RECOVERIES_MAX; attempt++)
    {
        set_latch(&st, kind);
        assert(rest_in_latch(&st, &cmd, 0));
        assert((uint8_t)PILOT_REASON_ENCODER != cmd.reason);
        assert((uint8_t)PILOT_REASON_HUG_TIMEOUT != cmd.reason);
    }
    set_latch(&st, kind);
    assert(rest_in_latch(&st, &cmd, 0));

    pilot_start_run(&st);
    assert(!latch_set(&st));
    for (unsigned attempt = 0U; attempt < PILOT_RECOVERIES_MAX; attempt++)
    {
        set_latch(&st, kind);
        assert(rest_in_latch(&st, &cmd, 0));
    }
    set_latch(&st, kind);
    assert(rest_in_latch(&st, &cmd, (int64_t)PILOT_RECOVER_CLEAR_MM * UM_PER_MM));
}

static void test_recover_waits_for_higher_stop(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;

    g_now_ms = RECOVER_START_MS;
    pilot_init(&st);
    set_latch(&st, LATCH_COURSE);
    for (uint32_t ms = 0U; ms <= PILOT_RECOVER_MS + TICK_MS; ms += TICK_MS)
    {
        pilot_meas_t meas = {0};

        g_now_ms += TICK_MS;
        meas.now_ms           = g_now_ms;
        meas.stamp_ms         = g_now_ms;
        meas.b_line_left      = true;
        meas.b_line_right     = true;
        meas.b_sonar_fault    = true;
        meas.b_bounded_course = true;
        pilot_step(&st, &meas, &cmd);
    }
    assert(st.avoidance.b_course_stuck);
    assert(0U == st.recoveries);

    pilot_init(&st);
    set_latch(&st, LATCH_ENCODER);
    pilot_estop(&st);
    for (uint32_t ms = 0U; ms <= PILOT_RECOVER_MS + TICK_MS; ms += TICK_MS)
    {
        pilot_meas_t meas = {0};

        g_now_ms += TICK_MS;
        meas.now_ms       = g_now_ms;
        meas.stamp_ms     = g_now_ms;
        meas.b_line_left  = true;
        meas.b_line_right = true;
        meas.ping_seq     = 1U;
        meas.ping_echo    = OBSTACLE_CLASS_OPEN;
        pilot_step(&st, &meas, &cmd);
    }
    assert(0U != st.b_encoder);
    assert(0U == st.recoveries);
}

/* One tick with a dead sonar on the line, at the given travel. */
static void tick_blind(pilot_state_t * p_state, pilot_cmd_t * p_cmd, int64_t travel_um,
                       bool b_loaded, bool b_bounded)
{
    pilot_meas_t meas = {0};

    g_now_ms += TICK_MS;
    meas.now_ms           = g_now_ms;
    meas.stamp_ms         = g_now_ms;
    meas.left_um          = travel_um;
    meas.right_um         = travel_um;
    meas.b_line_left      = true;
    meas.b_line_right     = true;
    meas.b_sonar_fault    = true;
    meas.b_loaded         = b_loaded;
    meas.b_bounded_course = b_bounded;
    pilot_step(p_state, &meas, p_cmd);
}

/* Once the outage interval has elapsed, a supported crawl starts on this pass. */
static void start_supported_crawl(pilot_state_t * p_state, pilot_cmd_t * p_cmd, int64_t travel_um)
{
    tick_blind(p_state, p_cmd, travel_um, false, false);
    assert((uint8_t)PILOT_REASON_BLIND_CRAWL == p_cmd->reason);
    assert((p_cmd->speed_mm_s > 0) && (p_cmd->speed_mm_s <= PILOT_BLIND_CRAWL_MM_S));
}

/* Crawl forward until a contact; return the travel at the contact. */
static int64_t crawl_to_contact(pilot_state_t * p_state, pilot_cmd_t * p_cmd, int64_t travel_um)
{
    for (unsigned n = 0U; n < CRAWL_TICKS_MAX; n++)
    {
        assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour);
        assert((uint8_t)PILOT_REASON_BLIND_CRAWL == p_cmd->reason);
        assert((p_cmd->speed_mm_s > 0) && (p_cmd->speed_mm_s <= PILOT_BLIND_CRAWL_MM_S));
        travel_um += (int64_t)p_cmd->speed_mm_s * TICK_MS;
        tick_blind(p_state, p_cmd, travel_um, false, false);
    }
    tick_blind(p_state, p_cmd, travel_um, true, false);
    assert((uint8_t)PILOT_BEHAVIOUR_SAFETY == p_cmd->behaviour);
    assert((uint8_t)PILOT_REASON_STUCK == p_cmd->reason);
    assert((-PILOT_BACKOFF_MM_S == p_cmd->speed_mm_s) && (0 == p_cmd->turn_deci_s));
    return travel_um;
}

/* Back off until the stop; return the travel. */
static int64_t back_off(pilot_state_t * p_state, pilot_cmd_t * p_cmd, int64_t travel_um)
{
    int64_t const from_um = travel_um;

    while ((p_cmd->speed_mm_s < 0) && ((from_um - travel_um) <= BACKOFF_LIMIT_UM))
    {
        travel_um += (int64_t)p_cmd->speed_mm_s * TICK_MS;
        tick_blind(p_state, p_cmd, travel_um, false, false);
    }
    assert((from_um - travel_um) >= PILOT_BACKOFF_UM);
    assert((uint8_t)PILOT_REASON_SONAR_LATCHED == p_cmd->reason);
    assert(0 == p_cmd->speed_mm_s);
    return travel_um;
}

/* Measured contact permits a retreat and the next safe crawl. A blind crawl
 * without motion remains stopped. Bounded courses never use a blind crawl. */
static void test_blind_crawl(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    int64_t       travel = 0;

    g_now_ms = RECOVER_START_MS;
    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    for (unsigned contact = 0U; contact < 3U; contact++)
    {
        start_supported_crawl(&st, &cmd, travel);
        travel = back_off(&st, &cmd, crawl_to_contact(&st, &cmd, travel));
    }
    start_supported_crawl(&st, &cmd, travel);
    for (uint32_t ms = 0U; ms < PILOT_STUCK_MS; ms += TICK_MS)
    {
        tick_blind(&st, &cmd, travel, false, false);
    }
    assert(PILOT_REASON_STUCK == cmd.reason && !cmd.speed_mm_s && !cmd.turn_deci_s);
    for (uint32_t ms = 0U; ms < RECOVER_WAIT_MS; ms += TICK_MS)
    {
        tick_blind(&st, &cmd, travel, false, false);
        assert(PILOT_REASON_STUCK == cmd.reason && !cmd.speed_mm_s && !cmd.turn_deci_s);
    }

    /* Usable forward pings end the latch and the crawl. */
    for (unsigned n = 0U; n < PING_RESUME_TICKS; n++)
    {
        tick_at(&st, &cmd, g_now_ms + TICK_MS, false, false, false);
    }
    assert((uint8_t)PILOT_REASON_NONE == cmd.reason);
    assert(cmd.speed_mm_s > PILOT_BLIND_CRAWL_MM_S);

    pilot_init(&st);
    latch_bad(&st, &cmd, true);
    for (uint32_t ms = 0U; ms < RECOVER_WAIT_MS; ms += TICK_MS)
    {
        tick_blind(&st, &cmd, 0, false, true);
        assert((uint8_t)PILOT_REASON_SONAR_LATCHED == cmd.reason);
    }
}

static void test_dropout_uses_one_interval(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    uint32_t const loss_ms = UINT32_MAX - 500U;
    pilot_meas_t meas = {
        .now_ms = loss_ms - TICK_MS,
        .stamp_ms = loss_ms - TICK_MS,
        .b_line_left = true,
        .b_line_right = true,
        .ping_echo = LOOK_ECHO_OPEN,
        .ping_seq = 1U,
    };

    pilot_init(&pilot);
    pilot_step(&pilot, &meas, &cmd);
    assert(cmd.speed_mm_s > 0);
    meas.now_ms = meas.stamp_ms = loss_ms;
    meas.left_um = meas.right_um = 2000;
    meas.ping_echo = LOOK_ECHO_NONE;
    meas.b_sonar_fault = true;
    pilot_step(&pilot, &meas, &cmd);
    assert(cmd.speed_mm_s == 0 && cmd.turn_deci_s == 0);

    meas.b_stationary = true;
    for (uint32_t elapsed = TICK_MS; elapsed <= 1000U; elapsed += TICK_MS)
    {
        meas.now_ms = meas.stamp_ms = loss_ms + elapsed;
        pilot_step(&pilot, &meas, &cmd);
        if (cmd.speed_mm_s != 0 || cmd.turn_deci_s != 0)
        {
            break;
        }
    }
    assert(cmd.reason == PILOT_REASON_BLIND_CRAWL && cmd.speed_mm_s > 0 &&
           cmd.speed_mm_s <= 80 && "eligible crawl must start by 1000 ms from the dropout");
}

static void test_blind_retreat_keeps_verified_margin(void)
{
    int64_t const approaches_um[] = {2000, 44000};
    for (size_t i = 0U; i < sizeof(approaches_um) / sizeof(approaches_um[0]); i++)
    {
        pilot_state_t pilot;
        pilot_cmd_t cmd;
        int64_t travel = 0;

        g_now_ms = RECOVER_START_MS;
        pilot_init(&pilot);
        latch_bad(&pilot, &cmd, true);
        start_supported_crawl(&pilot, &cmd, travel);
        travel = approaches_um[i];
        tick_blind(&pilot, &cmd, travel, true, false);
        bool const b_short = (i == 0U);
        assert((cmd.speed_mm_s < 0) == !b_short &&
               "reverse is permitted only when the verified path can cover the stop reserve");
        for (uint32_t ms = 0U; ms < PILOT_BACKOFF_MAX_MS && cmd.speed_mm_s < 0; ms += TICK_MS)
        {
            travel += (int64_t)cmd.speed_mm_s * TICK_MS;
            tick_blind(&pilot, &cmd, travel, false, false);
        }
        assert(!cmd.speed_mm_s && !cmd.turn_deci_s);
        assert(b_short ? (travel == approaches_um[i]) : (travel >= 25000));
    }
}

static void test_measured_retreat_keeps_recovery_eligible(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd = {0};
    int64_t travel = 0;

    pilot_init(&pilot);
    g_now_ms = RECOVER_START_MS;
    for (unsigned encounter = 0U; encounter < 4U; ++encounter)
    {
        for (uint32_t waited = 0U; waited < 3000U && cmd.speed_mm_s <= 0; waited += TICK_MS)
        {
            tick_blind(&pilot, &cmd, travel, false, false);
        }
        if (cmd.speed_mm_s <= 0 || cmd.reason != PILOT_REASON_BLIND_CRAWL)
        {
            fprintf(stderr, "Crawl did not start for encounter %u after %lld mm of net travel\n",
                    encounter + 1U, (long long)(travel / 1000));
        }
        assert(cmd.speed_mm_s > 0 && cmd.reason == PILOT_REASON_BLIND_CRAWL &&
               "verified movement before each new encounter must keep crawl eligible");
        if (encounter == 3U)
        {
            return;
        }
        for (unsigned tick_index = 0U; tick_index < CRAWL_TICKS_MAX; ++tick_index)
        {
            travel += (int64_t)cmd.speed_mm_s * TICK_MS;
            tick_blind(&pilot, &cmd, travel, false, false);
            assert(cmd.speed_mm_s > 0);
        }
        int64_t const contact_um = travel;
        tick_blind(&pilot, &cmd, travel, true, false);
        assert(cmd.speed_mm_s < 0);
        for (uint32_t reversed = 0U; reversed < 1000U && cmd.speed_mm_s < 0; reversed += TICK_MS)
        {
            travel += (int64_t)cmd.speed_mm_s * TICK_MS;
            tick_blind(&pilot, &cmd, travel, false, false);
        }
        assert(contact_um - travel >= 40000 && cmd.speed_mm_s >= 0);
    }
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

static void test_data_recovers_without_start(bool b_fault)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t meas = {
        .b_line_left = true, .b_line_right = true, .ping_echo = LOOK_ECHO_OPEN, .b_bounded_course = true,
    };

    pilot_init(&st);
    latch_bad(&st, &cmd, b_fault);
    for (uint8_t sample = 1U; sample <= 8U; sample++)
    {
        g_now_ms += TEST_PING_MS;
        if (5U == sample)
        {
            /* The sonar mailbox overwrote an unread sample. */
            continue;
        }
        meas.now_ms = g_now_ms;
        meas.stamp_ms = g_now_ms;
        meas.ping_seq = (uint8_t)((g_now_ms / TEST_PING_MS) & OBSTACLE_PING_SEQ_MK);
        meas.ping_echo = (2U == sample) ? LOOK_ECHO_NONE : LOOK_ECHO_OPEN;
        pilot_step(&st, &meas, &cmd);
        if (sample < 8U)
        {
            assert((uint8_t)latched_of(b_fault) == cmd.reason);
            assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
        }
    }
    assert((uint8_t)PILOT_REASON_NONE == cmd.reason);
    assert(cmd.speed_mm_s > 0);
}

typedef enum
{
    STILL, LEFT_ROLLBACK, RIGHT_ROLLBACK, BOTH_ROLLBACK, DROVE_THEN_STALLED,
    ONE_WHEEL_FORWARD, REPEATED_ONE_WHEEL
} stall_travel_t;

/* Prescribed counts are independent of the wheel targets. Four encoder edges
 * per sample give about 0.32 mm of wheel travel on the installed chassis. */
static void stall_encoders(pilot_meas_t * m, stall_travel_t travel, unsigned tick, unsigned retry_tick)
{
    int64_t const left = m->left_um;
    int64_t const right = m->right_um;
    bool const burst = tick >= 24U && tick <= 30U;
    bool const rolling = travel == BOTH_ROLLBACK && tick >= 24U && tick <= 105U;
    if (travel == DROVE_THEN_STALLED && tick == 11U)
    {
        m->left_um = m->right_um = 5000;
    }
    if ((travel == LEFT_ROLLBACK && burst) || rolling)
    {
        m->left_um -= 323;
    }
    if ((travel == RIGHT_ROLLBACK && burst) || rolling)
    {
        m->right_um -= 323;
    }
    if (((travel == ONE_WHEEL_FORWARD || travel == REPEATED_ONE_WHEEL) && burst) ||
        (travel == REPEATED_ONE_WHEEL && retry_tick != 0U && tick <= retry_tick + 7U))
    {
        m->left_um += 323;
    }
    m->b_stationary = left == m->left_um && right == m->right_um;
}

static void test_forward_stall_scan(stall_travel_t travel, bool obstacle, bool missing_reply)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = {.b_line_left = true, .b_line_right = true, .b_stationary = true};
    drive_state_t drive;
    int32_t left, right;
    drive_init(&drive);
    unsigned directions = 0U;
    bool stopped = false;
    unsigned retry_tick = 0U;
    pilot_init(&pilot);
    pilot_change_floor(&pilot, true);
    pilot_start_run(&pilot);
    unsigned const last_progress_tick = (travel == DROVE_THEN_STALLED) ? 11U : 1U;
    for (unsigned tick = 1; tick <= 450; tick++)
    {
        stall_encoders(&m, travel, tick, retry_tick);
        m.now_ms = tick * TICK_MS;
        m.stamp_ms = m.now_ms;
        m.b_ping_reply = cmd.b_ping_req;
        m.ping_servo_deci = cmd.b_ping_req ? cmd.ping_servo_deci : 0;
        m.ping_seq = cmd.b_ping_req ? cmd.ping_seq : (uint8_t)((tick / 5U) & OBSTACLE_PING_SEQ_MK);
        m.ping_echo = LOOK_ECHO_OPEN;
        m.ping_mm = 0U;
        if (m.b_ping_reply)
        {
            assert(stopped);
            directions |= (m.ping_servo_deci < 0) ? 1U : (m.ping_servo_deci > 0) ? 4U : 2U;
            if (missing_reply && m.ping_servo_deci > 0)
            {
                m.ping_echo = LOOK_ECHO_NONE;
            }
            else if (obstacle && m.ping_servo_deci == 0)
            {
                m.ping_echo = LOOK_ECHO_VALID;
                m.ping_mm = 90U;
            }
        }
        pilot_step(&pilot, &m, &cmd);
        pilot_drive(&drive, &cmd, TICK_MS, &left, &right);
        if (retry_tick != 0U)
        {
            assert(left >= 0 && right >= 0);
            if (cmd.reason == PILOT_REASON_ENCODER)
            {
                assert(!left && !right && tick >= retry_tick + PILOT_STUCK_MS / TICK_MS);
                return;
            }
            continue;
        }
        if (m.now_ms < last_progress_tick * TICK_MS + PILOT_STUCK_MS)
        {
            assert(left > 0 && right > 0);
        }
        if (m.now_ms == last_progress_tick * TICK_MS + PILOT_STUCK_MS)
        {
            assert(!left && !right && cmd.reason == PILOT_REASON_COURSE_RECOVERY);
        }
        if (tick == 1U)
        {
            assert(cmd.speed_mm_s > 0 && cmd.turn_deci_s == 0);
        }
        if (cmd.behaviour == PILOT_BEHAVIOUR_HUG || (stopped && cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE))
        {
            if (cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE)
            {
                assert(!missing_reply && directions == 7U);
            }
            if (!obstacle && !missing_reply)
            {
                assert(cmd.speed_mm_s > 0);
            }
            if (travel == REPEATED_ONE_WHEEL)
            {
                retry_tick = tick;
                continue;
            }
            return;
        }
        stopped |= cmd.speed_mm_s == 0;
        assert(cmd.speed_mm_s >= 0);
        if (stopped)
        {
            assert(cmd.speed_mm_s == 0 && cmd.turn_deci_s == 0 && left == 0 && right == 0);
        }
    }
    assert(missing_reply && stopped && directions == 7U);
}

/* On unqualified floor even a measured stall cannot authorize reversing. */
static void test_course_recovery_brakes(bool b_slip)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t meas = {
        .b_line_left = true, .b_line_right = true, .b_bounded_course = true,
        .ping_echo = LOOK_ECHO_VALID, .ping_mm = 400U,
    };
    drive_state_t drive;
    int32_t left = 0;
    int32_t right = 0;
    bool held = false;

    pilot_init(&pilot);
    drive_init(&drive);
    for (unsigned tick = 1U; tick <= 150U; tick++)
    {
        meas.now_ms = tick * TICK_MS;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq = (uint8_t)((meas.now_ms / TEST_PING_MS) & OBSTACLE_PING_SEQ_MK);
        meas.b_stationary = !b_slip || held;
        if (b_slip && !held)
        {
            meas.left_um += 1500;
            meas.right_um += 1500;
        }
        pilot_step(&pilot, &meas, &cmd);
        pilot_drive(&drive, &cmd, TICK_MS, &left, &right);
        if (tick == 1U)
        {
            assert(left > 0 && right > 0);
        }
        assert(left >= 0 && right >= 0);
        if (pilot.avoidance.b_course_stuck)
        {
            assert(!left && !right && cmd.reason == PILOT_REASON_COURSE_CLEARANCE);
        }
        if (PILOT_REASON_COURSE_CLEARANCE == cmd.reason)
        {
            assert(left == 0 && right == 0);
            held = true;
            /* A requested pivot has to pass the same guard while the
             * sonar reports an open forward beam. */
            meas.b_commanded_turn = true;
            meas.ping_echo = LOOK_ECHO_OPEN;
            meas.ping_mm = 0U;
        }
        if (held)
        {
            assert(PILOT_BEHAVIOUR_SAFETY == cmd.behaviour && left == 0 && right == 0);
        }
    }
    assert(held);
    /* Restored telemetry and RESET can clear other latches. The operator
     * still has to clear the obstruction and START on the original route. */
    pilot_release_sonar(&pilot);
    pilot_clear_latches(&pilot);
    meas.b_commanded_turn = false;
    meas.ping_seq++;
    pilot_step(&pilot, &meas, &cmd);
    assert((PILOT_REASON_COURSE_CLEARANCE == cmd.reason) || (PILOT_REASON_COURSE_RECOVERY == cmd.reason));
    pilot_start_run(&pilot);
    for (unsigned sample = 1U; sample <= 3U; sample++)
    {
        meas.now_ms += TEST_PING_MS;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq++;
        pilot_step(&pilot, &meas, &cmd);
        if (sample < 3U)
        {
            assert(PILOT_REASON_COURSE_RECOVERY == cmd.reason);
            assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
        }
    }
    assert(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour && cmd.speed_mm_s > 0);
}

static void test_course_held_contact(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t meas = {
        .b_line_left = true, .b_line_right = true, .b_bounded_course = true,
        .b_stationary = true, .b_held = true, .stamp_ms = TICK_MS, .now_ms = TICK_MS,
        .ping_echo = LOOK_ECHO_VALID, .ping_seq = 1U, .ping_mm = 80U,
    };

    pilot_init(&pilot);
    pilot_step(&pilot, &meas, &cmd);
    assert(PILOT_REASON_HELD == cmd.reason && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
    meas.b_held = false;
    meas.now_ms += TICK_MS;
    meas.stamp_ms = meas.now_ms;
    pilot_step(&pilot, &meas, &cmd);
    assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
    assert(PILOT_REASON_COURSE_RECOVERY == cmd.reason);
}

static void start_course_sweep(pilot_state_t * p_state, pilot_meas_t * p_meas,
                               pilot_cmd_t * p_cmd)
{
    *p_meas = (pilot_meas_t){
        .b_line_left = true, .b_line_right = true,
        .b_bounded_course = true, .b_stationary = true,
        .now_ms = 10U, .stamp_ms = 10U,
        .ping_mm = 100U, .ping_echo = LOOK_ECHO_VALID, .ping_seq = 1U,
    };

    pilot_init(p_state);
    pilot_step(p_state, p_meas, p_cmd);
    p_meas->now_ms += 50U;
    p_meas->stamp_ms = p_meas->now_ms;
    p_meas->ping_seq++;
    pilot_step(p_state, p_meas, p_cmd);
    assert((uint8_t)PILOT_REASON_COURSE_RECOVERY == p_cmd->reason);

    p_meas->now_ms += HUG_SETTLE_MS;
    p_meas->stamp_ms = p_meas->now_ms;
    pilot_step(p_state, p_meas, p_cmd);
    assert(p_cmd->b_ping_req);
}

static void check_course_clear_streak(uint8_t first_seq, unsigned missed)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    pilot_meas_t  meas;
    unsigned sequence = first_seq;

    start_course_sweep(&st, &meas, &cmd);
    for (unsigned ray = 0U; ray < 3U; ray++)
    {
        assert(cmd.b_ping_req);
        meas.now_ms += TICK_MS;
        meas.stamp_ms = meas.now_ms;
        meas.b_ping_reply = true;
        meas.ping_seq = cmd.ping_seq;
        meas.ping_servo_deci = cmd.ping_servo_deci;
        pilot_step(&st, &meas, &cmd);
        assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
    }
    assert(0U != cmd.course_scan_samples);

    meas.b_ping_reply = false;
    meas.ping_servo_deci = 0;
    for (unsigned sample = 0U; sample < 6U; sample++)
    {
        bool const b_blocked = (2U == sample);

        if (b_blocked && (missed > 0U))
        {
            /* An unread close return and any following pings were overwritten. */
            meas.now_ms += missed * TEST_PING_MS;
            sequence += missed;
            continue;
        }
        meas.now_ms += TEST_PING_MS;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq = (uint8_t)(sequence++ & OBSTACLE_PING_SEQ_MK);
        meas.ping_echo = (uint8_t)(b_blocked ? LOOK_ECHO_VALID : LOOK_ECHO_OPEN);
        meas.ping_mm = b_blocked ? 100U : 0U;
        pilot_step(&st, &meas, &cmd);
        if (sample < 5U)
        {
            assert((uint8_t)PILOT_REASON_COURSE_RECOVERY == cmd.reason);
            assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
        }
    }
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
    assert(cmd.speed_mm_s > 0);
}

static void test_course_blocked_ping_breaks_clear_streak(void)
{
    check_course_clear_streak(3U, 0U);
    check_course_clear_streak(3U, 1U);
    check_course_clear_streak(62U, 1U); /* Miss seq 0 after seq 63. */
    check_course_clear_streak(59U, 1U); /* The three new clears span 62, 63, 0. */
    check_course_clear_streak(3U, OBSTACLE_PING_SEQ_MK + 1U);
}

static void test_course_aborted_sweep_confirms_clearance(void)
{
    pilot_state_t st;
    pilot_cmd_t   cmd;
    pilot_meas_t  meas;

    start_course_sweep(&st, &meas, &cmd);
    uint8_t const request_seq = cmd.ping_seq;
    int16_t const request_servo_deci = cmd.ping_servo_deci;

    meas.now_ms += TICK_MS;
    meas.stamp_ms = meas.now_ms;
    meas.b_stationary = false;
    pilot_step(&st, &meas, &cmd);
    assert(!cmd.b_ping_req);
    assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);

    /* The sonar can finish the side ray after the pilot stops requesting it. */
    meas.now_ms += 50U;
    meas.stamp_ms = meas.now_ms;
    meas.b_stationary = true;
    meas.b_ping_reply = true;
    meas.ping_seq = request_seq;
    meas.ping_servo_deci = request_servo_deci;
    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_mm = 0U;
    pilot_step(&st, &meas, &cmd);
    assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);

    meas.now_ms += 100U;
    meas.b_ping_reply = false;
    meas.ping_servo_deci = 0;
    for (uint8_t seq = 3U; seq <= 5U; seq++)
    {
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq = seq;
        pilot_step(&st, &meas, &cmd);
        if (seq < 5U)
        {
            assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
        }
        meas.now_ms += 50U;
    }
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
    assert(cmd.speed_mm_s > 0);
}

static void test_recovery_survey_retries_missing_edges(void)
{
    pilot_state_t pilot;
    pilot_meas_t meas = {.b_stationary = true, .b_line_left = true};
    pilot_cmd_t cmd = {0};
    unsigned sweeps = 0U;
    uint8_t forward_seq = 0U;
    pilot_init(&pilot);
    pilot_recover_pose(&pilot, false, 0U);
    for (unsigned tick = 1U; tick <= 400U; tick++)
    {
        meas.now_ms = tick * TICK_MS;
        meas.stamp_ms = meas.now_ms;
        meas.b_ping_reply = cmd.b_ping_req != 0U;
        meas.ping_servo_deci = meas.b_ping_reply ? cmd.ping_servo_deci : 0;
        if (meas.b_ping_reply)
        {
            meas.ping_seq = cmd.ping_seq;
            meas.ping_echo = sweeps < 2U ? LOOK_ECHO_NONE : LOOK_ECHO_OPEN;
        }
        else
        {
            if ((tick % 5U) == 0U) forward_seq = (uint8_t)((forward_seq + 1U) & OBSTACLE_PING_SEQ_MK);
            meas.ping_seq = forward_seq;
            meas.ping_echo = LOOK_ECHO_OPEN;
        }
        pilot_step(&pilot, &meas, &cmd);
        if (cmd.course_scan_samples != 0U) sweeps++;
        if (cmd.speed_mm_s != 0 || cmd.turn_deci_s != 0)
        {
            assert((PILOT_BEHAVIOUR_HUG == cmd.behaviour) || (PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour));
            return;
        }
    }
    assert(0 && "Fresh sonar after incomplete surveys must recover the run");
}

static void test_course_movement_breaks_clear_streak(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t meas;
    static uint8_t const sequence[] = {62U, 63U, 63U, 0U, 1U, 2U};

    start_course_sweep(&st, &meas, &cmd);
    /* Cancel the sweep with movement, then verify the separate parked streak. */
    meas.now_ms += TICK_MS;
    meas.stamp_ms = meas.now_ms;
    meas.b_stationary = false;
    pilot_step(&st, &meas, &cmd);
    assert(!cmd.b_ping_req && 0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);

    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_mm = 0U;
    for (size_t i = 0U; i < sizeof sequence / sizeof sequence[0]; i++)
    {
        meas.b_stationary = (i != 2U);
        meas.now_ms += 40U;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq = sequence[i];
        pilot_step(&st, &meas, &cmd);
        if (i < 5U)
        {
            assert((uint8_t)PILOT_REASON_COURSE_RECOVERY == cmd.reason);
            assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);
        }
    }
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour);
    assert(cmd.speed_mm_s > 0);
}

static void test_never_seen_forward_has_startup_deadline(void)
{
    uint32_t const started_ms = UINT32_MAX - 80U;
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t meas = {.b_line_left = true, .b_line_right = true, .b_held = true};

    pilot_init(&st);
    meas.now_ms = started_ms - 1000U;
    meas.stamp_ms = meas.now_ms;
    pilot_step(&st, &meas, &cmd);
    assert((uint8_t)PILOT_REASON_HELD == cmd.reason);
    meas.now_ms = started_ms - 1U;
    meas.stamp_ms = meas.now_ms;
    pilot_step(&st, &meas, &cmd);
    assert((uint8_t)PILOT_REASON_HELD == cmd.reason);
    meas.b_held = false;
    meas.now_ms = started_ms;
    meas.stamp_ms = meas.now_ms;
    pilot_step(&st, &meas, &cmd);
    assert(cmd.speed_mm_s > 0);
    meas.now_ms += PILOT_FORWARD_PING_STALE_MS - 1U;
    meas.stamp_ms = meas.now_ms;
    pilot_step(&st, &meas, &cmd);
    assert(cmd.speed_mm_s > 0);
    meas.now_ms++;
    meas.stamp_ms = meas.now_ms;
    pilot_step(&st, &meas, &cmd);
    assert((uint8_t)PILOT_REASON_SONAR == cmd.reason);
    assert(0 == cmd.speed_mm_s && 0 == cmd.turn_deci_s);

    /* An actual centered frame ends this short dropout, including wrap. */
    meas.now_ms += TICK_MS;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq = 63U;
    meas.ping_echo = LOOK_ECHO_OPEN;
    pilot_step(&st, &meas, &cmd);
    assert((uint8_t)PILOT_REASON_NONE == cmd.reason && cmd.speed_mm_s > 0);
    meas.now_ms += PILOT_FORWARD_PING_STALE_MS;
    meas.stamp_ms = meas.now_ms;
    meas.ping_seq = 0U;
    pilot_step(&st, &meas, &cmd);
    assert((uint8_t)PILOT_REASON_NONE == cmd.reason && cmd.speed_mm_s > 0);
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

/* Boot 25568, profile 6. A zero slot carries no observation. */
static void test_recorded_face_reach(void)
{
    static int32_t const ranges[] = {103, 99, 93, 93, 94};
    look_pose_t const pose = {310, -180, 900};
    look_face_t const face = {0, 0, 900, 163};
    look_t look;
    look_clear(&look);
    for (int i = 0; i < 5; i++)
    {
        assert(look_add(&look, &pose, (i - 2) * 100, LOOK_ECHO_VALID, ranges[i]));
    }
    assert(look_add(&look, &pose, 200, LOOK_ECHO_VALID, 700));
    assert(look_add(&look, &pose, 250, LOOK_ECHO_OPEN, 0));
    assert(17 == look_face_reach_mm(&look, &pose, &face, LOOK_SIDE_LEFT));
    assert(18 == look_face_reach_mm(&look, &pose, &face, LOOK_SIDE_RIGHT));
    look_clear(&look);
    assert(0 == look_face_reach_mm(&look, &pose, &face, LOOK_SIDE_RIGHT));
}

static uint32_t begin_stationary_contact(hug_t *hug, look_t *look, hug_in_t *in, hug_out_t *out)
{
    hug_init(hug);
    look_clear(look);
    in->now_ms = UINT32_MAX - 1000U;
    in->b_stationary = true;
    in->ping_echo = LOOK_ECHO_VALID;
    in->ping_mm = 99U;
    in->ping_seq = 1U;
    assert(!hug_step(hug, look, in, out));
    in->now_ms += 50U;
    in->ping_seq++;
    assert(hug_step(hug, look, in, out));
    return in->now_ms;
}

static void test_hug_stationary_feedback(void)
{
    hug_t hug = {0};
    look_t look;
    hug_in_t in = {0};
    hug_out_t out;
    (void)begin_stationary_contact(&hug, &look, &in, &out);
    assert(out.b_req);
    in.b_stationary = false;
    in.b_ping_reply = true;
    in.ping_seq = out.seq;
    in.ping_servo_deci = out.servo_deci;
    in.ping_echo = LOOK_ECHO_OPEN;
    unsigned zero_ticks = 0U;
    bool b_handoff = false;
    for (unsigned i = 0; i < 350 && !b_handoff; ++i)
    {
        in.now_ms += TICK_MS;
        b_handoff = !hug_step(&hug, &look, &in, &out);
        if (!b_handoff)
        {
            assert(!out.b_req && !out.turn_deci_s && !out.speed_mm_s && look.count == 0);
            ++zero_ticks;
        }
    }
    assert(b_handoff && out.b_no_exit && "a missing reply hands off the expired fan");
    assert(zero_ticks <= (BYPASS_FAN_DEADLINE_MS + PILOT_RECOVER_MS) / TICK_MS &&
           "zero demand lasts no longer than the deadline plus the recovery budget");
    in.b_stationary = true;
    in.now_ms += TICK_MS;
    (void)hug_step(&hug, &look, &in, &out);
    assert(!out.b_req && !out.speed_mm_s && !out.turn_deci_s && look.count == 0 &&
           "a glitch cannot renew the expired fan deadline at the same pose");
}

static void test_fan_deadline_survives_missing_replies(void)
{
    hug_t hug = {0};
    look_t look;
    hug_in_t in = {0};
    hug_out_t out;
    uint32_t const start = begin_stationary_contact(&hug, &look, &in, &out);
    assert(out.b_req);

    in.ping_echo = LOOK_ECHO_NONE;
    in.ping_mm = 0U;
    for (uint32_t elapsed = TICK_MS; elapsed < 1000U; elapsed += TICK_MS)
    {
        in.now_ms = start + elapsed;
        (void)hug_step(&hug, &look, &in, &out);
        assert(out.speed_mm_s <= 0 && !out.b_rejoined);
    }

    /* The queue delivers the pending request's matching OPEN at the fan deadline. */
    in.b_ping_reply = out.b_req != 0U;
    in.ping_seq = out.seq;
    in.ping_servo_deci = out.servo_deci;
    in.ping_echo = LOOK_ECHO_OPEN;
    in.now_ms = start + 1000U;
    (void)hug_step(&hug, &look, &in, &out);
    assert(!out.b_req && out.speed_mm_s <= 0 && !out.b_rejoined && look.count == 0U &&
           "the original fine fan must retire at 1000 ms, including a matching late reply");
}

static void test_hug_slow_scan_pause(void)
{
    hug_t hug = {0};
    look_t look;
    hug_in_t in = {0};
    hug_out_t out;
    uint32_t const start = begin_stationary_contact(&hug, &look, &in, &out);
    unsigned replies = 0;
    while (in.now_ms - start < HUG_STATIONARY_LIMIT_MS)
    {
        assert(out.b_req);
        in.b_ping_reply = true;
        in.ping_seq = out.seq;
        in.ping_servo_deci = (int16_t)(out.servo_deci + 100);
        in.now_ms += 150;
        assert(hug_step(&hug, &look, &in, &out) && out.b_req && out.seq == in.ping_seq);
        assert(look.count == replies);
        in.now_ms += 200;
        in.ping_servo_deci = out.servo_deci;
        in.ping_echo = LOOK_ECHO_VALID;
        in.ping_mm = 99;
        assert(hug_step(&hug, &look, &in, &out));
        ++replies;
        in.b_ping_reply = false;
        in.now_ms += TICK_MS;
        assert(hug_step(&hug, &look, &in, &out));
        if (out.speed_mm_s != 0 || out.turn_deci_s != 0)
        {
            assert(replies >= 3 && in.now_ms - start < HUG_STATIONARY_LIMIT_MS);
            assert(out.speed_mm_s < 0 && !out.turn_deci_s);
            return;
        }
    }
    assert(!"usable close readings must produce movement within the stationary budget");
}

static void test_short_leg_clearance(void)
{
    hug_t hug = {0};
    look_t look;
    hug_in_t in = {.now_ms = 10U, .ping_echo = LOOK_ECHO_VALID, .ping_mm = 320U, .ping_seq = 1U};
    hug_out_t out;
    hug_init(&hug);
    look_clear(&look);
    /* Resume an already planned 60 mm side leg with a fresh forward echo.
     * The measured leg and the chassis turning envelope both fit. */
    hug.state = HUG_WORD_ST_DRIVE;
    hug.then = HUG_WORD_THEN_LOOK;
    hug.flags = UINT16_C(0x8000); /* A full side scan supplied the measured face. */
    hug.goal_len_mm = 60;
    hug.t_ms = 1000U;
    assert(hug_step(&hug, &look, &in, &out) && out.speed_mm_s > 0);
    hug_t blocked = hug;
    hug_t too_close = hug;
    in.ping_seq++;
    in.ping_mm = 290U;
    assert(hug_step(&blocked, &look, &in, &out) && !out.speed_mm_s);
    assert(blocked.state == HUG_WORD_ST_SCAN);
    in.ping_echo = LOOK_ECHO_TOO_CLOSE;
    in.ping_mm = 0U;
    assert(hug_step(&too_close, &look, &in, &out) && !out.speed_mm_s);
    assert(too_close.state == HUG_WORD_ST_SCAN);

    in.ping_echo = LOOK_ECHO_VALID;
    in.left_um = 60000;
    in.right_um = 60000;
    in.now_ms += 600U;
    in.ping_mm = 290U;
    assert(hug_step(&hug, &look, &in, &out) && !out.speed_mm_s);
    in.now_ms += HUG_SETTLE_MS;
    assert(hug_step(&hug, &look, &in, &out));
    in.now_ms += TICK_MS;
    assert(hug_step(&hug, &look, &in, &out) && out.b_scan_pivot && out.turn_deci_s != 0);
}

/* During an encoder-measured leg, supply the pose and the front IR contact
 * directly. A 90 degree heading puts the reader 130 mm out from the axle. */
static void check_tape_on_leg(int side, int axle_lateral_mm, int heading_deci, bool both, bool expect_exit)
{
    hug_t hug = {0};
    look_t look;
    hug_in_t in = {.now_ms = 10U, .b_line_left = true, .b_line_right = both};
    hug_out_t out;
    hug_init(&hug);
    look_clear(&look);
    hug.state = HUG_WORD_ST_DRIVE;
    hug.then = HUG_WORD_THEN_LOOK;
    hug.side = (int8_t)side;
    hug.goal_len_mm = 60;
    hug.t_ms = 1000U;
    hug.goal_head = (int16_t)(side * heading_deci);
    hug.diff_um = hug.goal_head * CHASSIS_WHEEL_TRACK_MM * 1000 / 573;
    hug.x_um = 400000;
    hug.y_um = side * axle_lateral_mm * 1000;
    hug.origin.drive.x_mm = 400;
    hug.origin.drive.y_mm = side * axle_lateral_mm;
    assert(hug_step(&hug, &look, &in, &out));
    assert(hug.then == (expect_exit ? HUG_WORD_THEN_EXIT : HUG_WORD_THEN_LOOK));
    if (!expect_exit)
    {
        assert(out.speed_mm_s > 0);
    }
}

static void test_hug_adjacent_tape(void)
{
    for (int side = -1; side <= 1; side += 2)
    {
        check_tape_on_leg(side, 170, 900, false, false);   /* reader at 300 */
        check_tape_on_leg(side, 430, -900, false, false);  /* same tape from the far side */
        check_tape_on_leg(side, 170, 900, true, false);    /* both readers can fit on the neighbor */
        check_tape_on_leg(side, 300, 0, true, true);       /* transverse tape spans the readers */
        check_tape_on_leg(side, 300, -900, false, true);   /* bent exit at 170 */
        check_tape_on_leg(side, 130, -900, false, true);   /* original line */
    }
}

/* At the stationary refinement budget, the pilot uses the two saved
 * hit-to-open front edges. Feed those endpoints directly in millimetres. */
static void check_flank_drive(hug_t hug, look_t look, hug_in_t in, int half)
{
    hug_out_t out;
    look_pose_t const pose = {hug.x_um / 1000, hug.y_um / 1000, hug.center};
    /* A close return on any of the three rays vetoes the slant. */
    for (int blocked = 0; blocked <= 1; blocked++)
    {
        hug_t pass = hug;
        look_t probe = look;
        uint8_t const first_echo = (uint8_t)(blocked ? LOOK_ECHO_VALID : LOOK_ECHO_OPEN);
        assert(look_add(&probe, &pose, -150, first_echo, blocked ? 50 : 0));
        assert(look_add(&probe, &pose, 0, LOOK_ECHO_OPEN, 0));
        pass.idx = 2;
        pass.sub = 2; /* two bearings acquired, waiting for the third */
        pass.diff_um = pass.center * CHASSIS_WHEEL_TRACK_MM * 1000 / 573;
        pass.last_motion_ms = pass.t_ms = in.now_ms;
        in.b_ping_reply = true;
        in.ping_servo_deci = 150;
        in.ping_seq = pass.req_seq;
        assert(hug_step(&pass, &probe, &in, &out));
        if (blocked)
        {
            assert(pass.state != HUG_WORD_ST_DRIVE && out.speed_mm_s == 0);
            continue;
        }
        assert(pass.state == HUG_WORD_ST_DRIVE && pass.goal_head == pose.head_deci);
        in.now_ms++;
        in.b_ping_reply = false;
        in.b_stationary = false;
        in.ping_servo_deci = 0;
        assert(hug_step(&pass, &probe, &in, &out) && out.speed_mm_s > 0);
        hug_t obstructed = pass;
        hug_in_t near = in;
        near.ping_seq++;
        near.ping_echo = LOOK_ECHO_TOO_CLOSE;
        assert(!hug_step(&obstructed, &probe, &near, &out));
        assert(out.b_timeout && !out.turn_deci_s && !out.speed_mm_s);

        /* Supply the pose at the front plane and then at the next survey.
         * A close pass continues straight until there is room to pivot. */
        double const angle = abs(pass.goal_head) * acos(-1.0) / 1800.0;
        int const at_front = abs(pose.y_mm) + (int)lround((320 - pose.x_mm) * tan(angle));
        pass.x_um = 320000;
        pass.y_um = pass.side * at_front * 1000;
        assert(at_front - half < HUG_SWEEP_RADIUS_MM);
        assert(hug_step(&pass, &probe, &in, &out) && out.speed_mm_s > 0 && !out.b_scan_pivot);
        pass.y_um = pass.side * (half + HUG_SWEEP_RADIUS_MM + 26) * 1000;
        pass.x_um += (int32_t)lround((abs(pass.y_um) - at_front * 1000) / tan(angle));
        assert(hug_step(&pass, &probe, &in, &out) && !out.turn_deci_s && !out.speed_mm_s);
        in.now_ms += HUG_SETTLE_MS;
        in.b_stationary = true;
        assert(hug_step(&pass, &probe, &in, &out) && pass.state != HUG_WORD_ST_DRIVE);
        in.now_ms++;
        assert(hug_step(&pass, &probe, &in, &out) && out.b_scan_pivot);
    }
}

static void test_front_edge_leg(void)
{
    int const half_widths[] = {20, 25, 100};
    for (size_t i = 0; i < sizeof(half_widths) / sizeof(half_widths[0]); i++)
    {
        int const half = half_widths[i];
        for (int sparse = 0; sparse <= 1; sparse++)
        {
            hug_t hug = {0};
            look_t look;
            hug_in_t in = {.now_ms = HUG_SCAN_BATCH_MS, .b_ping_reply = true, .b_stationary = true,
                           .ping_echo = LOOK_ECHO_OPEN, .ping_servo_deci = -HUG_REFINE_ZONE_DECI};
            hug_out_t out;
            hug_init(&hug);
            look_clear(&look);
            hug.state = HUG_WORD_ST_SCAN;
            hug.purpose = 3U; /* refine */
            hug.sub = 2U;     /* waiting for the requested ray */
            hug.body = 10U;   /* servo-only refinement */
            hug.t_ms = in.now_ms;
            hug.flags = sparse ? UINT16_C(0x4010) : UINT16_C(0x0010);
            hug.keep_x_mm[0] = hug.keep_x_mm[1] = hug.keep_dist_mm = 320;
            hug.keep_y_mm[LOOK_SIDE_LEFT] = (int16_t)half;
            hug.keep_y_mm[LOOK_SIDE_RIGHT] = (int16_t)-half;
            if (sparse)
            {
                hug.wall_mm[LOOK_SIDE_LEFT] = 200U;
            }
            assert(hug_step(&hug, &look, &in, &out));
            assert(hug.state != HUG_WORD_ST_SCAN);
            int const front_gap = 320 - hug.app_mm;
            assert(front_gap > HUG_SWEEP_RADIUS_MM);
            if (half == 100)
            {
                assert(hug.leg_mm - half >= HUG_SWEEP_RADIUS_MM);
                continue;
            }
            /* Supply the settled axle pose at the end of the lateral leg.
             * The next sonar look points down the proposed passing path. */
            hug.state = HUG_WORD_ST_DRIVE;
            hug.then = HUG_WORD_THEN_LOOK;
            hug.sub = 1U;
            hug.x_um = hug.app_mm * 1000;
            hug.y_um = hug.side * hug.leg_mm * 1000;
            hug.t_ms = in.now_ms;
            hug.last_motion_ms = in.now_ms;
            in.now_ms += HUG_SETTLE_MS;
            in.b_ping_reply = false;
            assert(hug_step(&hug, &look, &in, &out));
            assert(hug.state == HUG_WORD_ST_SCAN);
            double const radians = hug.side * hug.center * acos(-1.0) / 1800.0;
            double const corner_distance = front_gap * sin(radians) + (hug.leg_mm - half) * cos(radians);
            /* From the obstacle-facing outer edge, including the left reader. */
            double const side_reach = hug.side < 0 ? 107.0 : 63.0;
            double const gap = corner_distance + half * (sin(radians) + cos(radians) - 1.0) - side_reach;
            assert(radians > 0.0 && radians < acos(-1.0) / 2);
            assert(gap >= 40.0 && gap <= 50.0);
            check_flank_drive(hug, look, in, half);
        }
    }
}

/* Wheel totals are prescribed independently of the command. */
static void check_steering_handoff(bool first_left, bool check_zero)
{
    pilot_state_t pilot;
    drive_state_t drive;
    pilot_cmd_t command;
    pilot_meas_t m = {.ping_echo = LOOK_ECHO_OPEN};
    int32_t left = 0, right = 0;
    pilot_init(&pilot);
    drive_init(&drive);
    for (unsigned tick_no = 0; tick_no <= 60U; ++tick_no)
    {
        m.now_ms = m.stamp_ms = 1000U + TICK_MS * tick_no;
        m.ping_seq = (uint8_t)(tick_no / 5U);
        m.left_um = m.right_um = (int64_t)tick_no * 1000;
        m.b_line_left = tick_no < 30U || ((tick_no < 60U) == first_left);
        m.b_line_right = tick_no < 30U || ((tick_no < 60U) != first_left);
        pilot_step(&pilot, &m, &command);
        pilot_drive(&drive, &command, TICK_MS, &left, &right);
        assert(command.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE);
        if (check_zero && tick_no == 59U)
        {
            assert((first_left ? left : right) == 0);
            assert((first_left ? right : left) > 0);
        }
    }
    assert(first_left ? left > right : right > left);
}

/* An 8 mm narrow Code 39 start symbol and separator occupy 128 mm.
 * The decoder cannot confirm the delimiter at the normal 100 mm probe. */
static void check_wide_marker_prefix(void)
{
    static const uint8_t ends_mm[] = {8, 32, 40, 48, 72, 80, 104, 112, 120, 128};
    pilot_state_t pilot;
    pilot_cmd_t command;
    pilot_meas_t m = {.b_barcode = true, .ping_echo = LOOK_ECHO_OPEN};
    pilot_init(&pilot);
    size_t elements = 0;
    for (uint32_t mm = 0; mm <= 128U; ++mm)
    {
        m.now_ms = m.stamp_ms = mm * TICK_MS;
        m.ping_seq = (uint8_t)(mm / 5U);
        m.left_um = m.right_um = (int64_t)mm * 1000;
        if (elements < sizeof(ends_mm) && mm == ends_mm[elements])
        {
            ++elements;
        }
        m.b_marker_progress = elements == sizeof(ends_mm);
        m.marker_prefix_mm = elements == 0U || m.b_marker_progress ? 0U : ends_mm[elements - 1U];
        pilot_step(&pilot, &m, &command);
        assert(command.turn_deci_s == 0 && command.speed_mm_s > 0);
    }
}

static void check_unconfirmed_marker(void)
{
    pilot_state_t pilot;
    pilot_cmd_t command;
    pilot_meas_t m = {.b_bounded_course = true, .b_barcode = true, .ping_echo = LOOK_ECHO_OPEN};
    pilot_init(&pilot);
    bool searched = false;
    for (unsigned tick_no = 0; tick_no <= 200U; ++tick_no)
    {
        m.now_ms = m.stamp_ms = TICK_MS * tick_no;
        m.ping_seq = (uint8_t)(tick_no / 5U);
        m.left_um = m.right_um = (int64_t)tick_no * 1600;
        pilot_step(&pilot, &m, &command);
        searched = searched || pilot.follow.status == LINE_FOLLOW_SEARCHING;
    }
    assert(searched);
}

/* The side reader can stay active between bends while both front readers
 * briefly agree. Keep the corner's speed through the marker and its release. */
static void test_marker_between_bends(void)
{
    pilot_state_t pilot;
    pilot_cmd_t cmd;
    pilot_meas_t m = {.b_line_left = true, .ping_echo = LOOK_ECHO_OPEN};
    pilot_init(&pilot);
    for (uint32_t tick_no = 0U; tick_no <= 16U; tick_no++)
    {
        m.stamp_ms = m.now_ms = 1000U + tick_no * TICK_MS;
        m.ping_seq = (uint8_t)tick_no;
        m.left_um = m.right_um = (int64_t)tick_no * 1000;
        pilot_step(&pilot, &m, &cmd);
    }
    assert(cmd.turn_deci_s > 0 && cmd.speed_mm_s > 0);
    for (unsigned pass = 0U; pass < 3U; pass++)
    {
        m.b_barcode = pass < 2U;
        m.b_line_left = false;
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        m.ping_seq++;
        pilot_step(&pilot, &m, &cmd);
        assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE && cmd.turn_deci_s == 0);
        assert(cmd.speed_mm_s == LINE_CORNER_UM_S / 1000);
    }
    m.b_barcode = true;
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms;
    pilot_step(&pilot, &m, &cmd);
    m.b_line_right = true;
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms;
    pilot_step(&pilot, &m, &cmd);
    assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE && cmd.turn_deci_s < 0);
}

/* One 50 ms forward ping and the corresponding encoder feedback. */
static void bend_sample(pilot_state_t *pilot, pilot_meas_t *m, pilot_cmd_t *cmd,
                        unsigned echo, unsigned mm, int32_t left_um_s, int32_t right_um_s)
{
    m->stamp_ms = m->now_ms += 50U;
    m->ping_seq = (uint8_t)((m->ping_seq + 1U) & OBSTACLE_PING_SEQ_MK);
    m->ping_echo = (uint8_t)echo;
    m->ping_mm = (uint16_t)mm;
    m->left_um += left_um_s / 20;
    m->right_um += right_um_s / 20;
    m->wheel_radius_mm = drive_forward_radius_mm(left_um_s, right_um_s);
    pilot_step(pilot, m, cmd);
}

static void test_hug_line_bend(void)
{
    for (int side = -1; side <= 1; side += 2)
    {
        pilot_state_t pilot;
        pilot_cmd_t cmd;
        pilot_meas_t m = {.b_line_left = side > 0, .b_line_right = side < 0};
        int32_t const left = side > 0 ? 20000 : 100000;
        int32_t const right = side > 0 ? 100000 : 20000;
        pilot_init(&pilot);
        for (unsigned tick = 0U; tick < 16U; tick++)
        {
            bend_sample(&pilot, &m, &cmd, LOOK_ECHO_OPEN, 0, left, right);
        }
        assert(cmd.speed_mm_s > 0 && side * cmd.turn_deci_s > 0);

        /* 86/100 mm/s puts a centred 225 mm echo 58.8 mm outside the axle's
         * arc, between the outer wheel centre (53.5 mm) and tire edge (63 mm).
         * At 53/100 mm/s the near end of the echo range at the 13-degree beam
         * edge enters the padded front corner only with the full tire width. */
        int32_t const clip_inner_um_s[] = {86000, 53000};
        for (size_t i = 0U; i < sizeof(clip_inner_um_s) / sizeof(clip_inner_um_s[0]); i++)
        {
            pilot_state_t approach = pilot;
            pilot_meas_t observation = m;
            int32_t const clip_left = side > 0 ? clip_inner_um_s[i] : 100000;
            int32_t const clip_right = side > 0 ? 100000 : clip_inner_um_s[i];
            bend_sample(&approach, &observation, &cmd, LOOK_ECHO_VALID, 225, clip_left, clip_right);
            bend_sample(&approach, &observation, &cmd, LOOK_ECHO_VALID, 225, clip_left, clip_right);
            assert(cmd.behaviour == PILOT_BEHAVIOUR_HUG);
        }
        for (unsigned tick = 0U; tick < 3U; tick++)
        {
            bend_sample(&pilot, &m, &cmd, LOOK_ECHO_VALID, 225, left, right);
            assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE && side * cmd.turn_deci_s > 0);
        }

        pilot_state_t saved = pilot;
        pilot_meas_t sample = m;
        bend_sample(&pilot, &m, &cmd, LOOK_ECHO_OPEN, 0, left, right);
        assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE);
        bend_sample(&pilot, &m, &cmd, LOOK_ECHO_OPEN, 0, left, right);
        assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE);
        /* A new obstacle directly ahead still takes control after the bend. */
        m.b_line_left = true;
        m.b_line_right = true;
        bend_sample(&pilot, &m, &cmd, LOOK_ECHO_VALID, 160, 100000, 100000);
        bend_sample(&pilot, &m, &cmd, LOOK_ECHO_VALID, 160, 100000, 100000);
        assert(cmd.behaviour == PILOT_BEHAVIOUR_HUG);

        /* Each change below happens after the same established bend and hits. */
        for (unsigned which = 0; which < 8U; which++)
        {
            pilot = saved;
            m = sample;
            unsigned range = 225;
            unsigned echo = LOOK_ECHO_VALID;
            int32_t l = left;
            int32_t r = right;
            switch (which)
            {
            case 0: range = 180; break;                    /* front overhang and stopping room */
            case 1: l = r = 100000; break;                 /* wheels moving straight */
            case 2: l = side > 0 ? 60000 : 100000;         /* outer corner can clip the beam edge */
                    r = side > 0 ? 100000 : 60000; break;
            case 3: m.b_line_left = m.b_line_right; break; /* lost directional IR evidence */
            case 4: echo = LOOK_ECHO_TOO_CLOSE; range = 0; break;
            case 5: l = right; r = left; break;            /* wheels turning the opposite way */
            case 6: m.b_commanded_turn = true; break;
            default: m.ping_seq++; break;                  /* a forward observation was missed */
            }
            bend_sample(&pilot, &m, &cmd, echo, range, l, r);
            assert(cmd.behaviour == PILOT_BEHAVIOUR_HUG);
        }
        /* A broad surface remains in the beam at both speeds. Faster travel
         * reaches the distance bound first. Slower travel reaches the clock bound. */
        for (int speed_divisor = 1; speed_divisor <= 2; speed_divisor++)
        {
            pilot = saved;
            m = sample;
            unsigned const remaining_ms = (1 == speed_divisor) ? 500U : 700U;
            while (m.now_ms - sample.now_ms < remaining_ms)
            {
                bend_sample(&pilot, &m, &cmd, LOOK_ECHO_VALID, 225, left / speed_divisor, right / speed_divisor);
                if (m.now_ms - sample.now_ms < remaining_ms)
                {
                    assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE);
                }
            }
            assert(cmd.behaviour == PILOT_BEHAVIOUR_HUG);
        }
    }
}

static int run_isolated_case(char const * p_name)
{
    if (strcmp(p_name, "dropout-uses-one-interval") == 0)
    {
        test_dropout_uses_one_interval();
        return EXIT_SUCCESS;
    }
    if (strcmp(p_name, "blind-retreat-keeps-verified-margin") == 0)
    {
        test_blind_retreat_keeps_verified_margin();
        return EXIT_SUCCESS;
    }
    if (strcmp(p_name, "fan-deadline-survives-missing-replies") == 0)
    {
        test_fan_deadline_survives_missing_replies();
        return EXIT_SUCCESS;
    }
    if (strcmp(p_name, "measured-retreat-keeps-recovery-eligible") == 0)
    {
        test_measured_retreat_keeps_recovery_eligible();
        return EXIT_SUCCESS;
    }
    if (strcmp(p_name, "steering-handoff") == 0 || strcmp(p_name, "inner-wheel-stop") == 0)
    {
        bool const check_zero = strcmp(p_name, "inner-wheel-stop") == 0;
        check_steering_handoff(true, check_zero);
        check_steering_handoff(false, check_zero);
        return EXIT_SUCCESS;
    }
    if (strcmp(p_name, "unconfirmed-marker") == 0)
    {
        check_unconfirmed_marker();
        check_wide_marker_prefix();
        return EXIT_SUCCESS;
    }
    if (strcmp(p_name, "marker-between-bends") == 0)
    {
        test_marker_between_bends();
    }
    else if (strcmp(p_name, "hug-adjacent-tape") == 0)
    {
        test_hug_adjacent_tape();
    }
    else if (strcmp(p_name, "hug-front-edge") == 0)
    {
        test_front_edge_leg();
    }
    else if (strcmp(p_name, "hug-line-bend") == 0)
    {
        test_hug_line_bend();
    }
    else if (strcmp(p_name, "hug-short-leg") == 0)
    {
        test_short_leg_clearance();
    }
    else if (strcmp(p_name, "hug-slow-scan") == 0)
    {
        test_hug_slow_scan_pause();
    }
    else if (strcmp(p_name, "hug-frozen-pivot") == 0)
    {
        test_hug_stationary_feedback();
    }
    else if (strcmp(p_name, "hug-recorded-face") == 0)
    {
        test_recorded_face_reach();
    }
    else if (strcmp(p_name, "reset-encoder") == 0)
    {
        test_encoder_cleared();
    }
    else if (strcmp(p_name, "reset-hug-timeout") == 0)
    {
        test_timeout_cleared();
    }
    else if (strcmp(p_name, "blind-crawl") == 0)
    {
        test_blind_crawl();
    }
    else if (strcmp(p_name, "recover-encoder") == 0)
    {
        test_latches_recover(LATCH_ENCODER);
    }
    else if (strcmp(p_name, "recover-hug-timeout") == 0)
    {
        test_latches_recover(LATCH_TIMEOUT);
    }
    else if (strcmp(p_name, "recover-course-clearance") == 0)
    {
        test_latches_recover(LATCH_COURSE);
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
    else if (strcmp(p_name, "startup-stall-scan") == 0)
    {
        test_forward_stall_scan(LEFT_ROLLBACK, false, false);
        test_forward_stall_scan(RIGHT_ROLLBACK, false, false);
        test_forward_stall_scan(BOTH_ROLLBACK, false, false);
        test_forward_stall_scan(DROVE_THEN_STALLED, false, false);
        test_forward_stall_scan(ONE_WHEEL_FORWARD, false, false);
        test_forward_stall_scan(REPEATED_ONE_WHEEL, false, false);
        test_forward_stall_scan(STILL, false, false);
        test_forward_stall_scan(STILL, true, false);
        test_forward_stall_scan(STILL, false, true);
    }
    else if (strcmp(p_name, "course-stall-floor") == 0)
    {
        test_course_recovery_brakes(false);
    }
    else if (strcmp(p_name, "course-slip-floor") == 0)
    {
        test_course_recovery_brakes(true);
    }
    else if (strcmp(p_name, "course-held-contact") == 0)
    {
        test_course_held_contact();
    }
    else if (strcmp(p_name, "course-blocked-ping-breaks-clear-streak") == 0)
    {
        test_course_blocked_ping_breaks_clear_streak();
    }
    else if (strcmp(p_name, "course-aborted-sweep-confirms-clearance") == 0)
    {
        test_course_aborted_sweep_confirms_clearance();
    }
    else if (strcmp(p_name, "course-movement-breaks-clear-streak") == 0)
    {
        test_course_movement_breaks_clear_streak();
    }
    else if (strcmp(p_name, "never-seen-forward-startup-deadline") == 0)
    {
        test_never_seen_forward_has_startup_deadline();
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
    test_blind_retreat_keeps_verified_margin();
    test_encoder_cleared();
    test_local_search_interruption();
    test_no_guide_after_completed_sweep();
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
    test_data_recovers_without_start(true);
    test_data_recovers_without_start(false);
    test_course_blocked_ping_breaks_clear_streak();
    test_course_aborted_sweep_confirms_clearance();
    test_recovery_survey_retries_missing_edges();
    test_course_movement_breaks_clear_streak();
    test_never_seen_forward_has_startup_deadline();
    test_release_keeps_the_others();
    test_release_then_still_bad();
    test_recover_waits_for_higher_stop();
    test_count_moved();
    (void)printf("pilot_latches: ok\n");
    return 0;
}

/*** end of file ***/
