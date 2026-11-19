/* Pilot tests: direct cases, then closed-loop properties on the simulated car
 * over several courses and fixed seeds. */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pilot/hug.h"
#include "telemetry/hug_word.h"
#include "pilot/look.h"
#include "pilot/pilot.h"
#include "platform/chassis.h"
#include "pilot_sim.h"

#define SEEDS            (6U)
#define STILL_SPEED      (5.0)
#define STILL_LIMIT_S    (3.0)
#define ON_LINE_MM       (25.0)
#define ON_LINE_MIN_PCT  (99.0)
#define START_S          (0.5)
#define RUN_S            (20.0)
#define STALE_FROM_S     (4.0)
#define STALE_LEN_S      (0.3)
#define RECOVER_S        (2.0)

static pilot_sim_t g_sim;

/* A robustness run reports its failure and the suite goes on, so that one
 * run does not hide the others; the suite fails at the end. */
static bool g_soft;
static bool g_run_failed;
static unsigned g_soft_fails;
static unsigned g_budget_overruns;
static unsigned g_slip_releasing_seeds;

static uint32_t const g_seeds[SEEDS] = {1U, 2U, 3U, 17U, 42U, 12345U};

static void fail(char const * p_what, uint32_t seed)
{
    (void)printf("FAIL: %s (seed %u)\n", p_what, (unsigned)seed);
    if (g_soft)
    {
        g_run_failed = true;
        g_soft_fails++;
        return;
    }
    pilot_sim_dump(&g_sim, seed, stdout);
    exit(EXIT_FAILURE);
}

static void check(bool b_ok, char const * p_what)
{
    if (!b_ok)
    {
        (void)printf("FAIL: %s\n", p_what);
        exit(EXIT_FAILURE);
    }
}

static pilot_meas_t fresh_meas(uint32_t now_ms)
{
    pilot_meas_t m = {0};

    m.now_ms = now_ms;
    m.stamp_ms = now_ms;
    m.b_line_left = true;

    return m;
}

static void test_direct(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = fresh_meas(1000U);

    pilot_init(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "fresh data drives");
    check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "follow line wins");
    check(cmd.speed_mm_s > 0, "drives forward");

    m.now_ms = m.stamp_ms + PILOT_STALE_MS;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "at the limit is still fresh");
    m.now_ms = m.stamp_ms + PILOT_STALE_MS + 1U;
    pilot_step(&st, &m, &cmd);
    check((PILOT_REASON_STALE == cmd.reason) && (0 == cmd.speed_mm_s) &&
              (0 == cmd.turn_deci_s) && (PILOT_BEHAVIOUR_SAFETY == cmd.behaviour),
          "stale data stops");
    m = fresh_meas(1200U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "stale ends by itself");

    /* The clock wraps. */
    pilot_init(&st);
    m = fresh_meas(0xFFFFFFF0U);
    m.now_ms = 5U;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "a wrapped clock is fresh");

    pilot_estop(&st);
    m = fresh_meas(1300U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_ESTOP == cmd.reason, "estop stops");
    m = fresh_meas(1400U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_ESTOP == cmd.reason, "estop latches");
    pilot_init(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "init (a reboot) ends estop");
    (void)printf("direct cases ok\n");
}

#define TICK_MS          (10U)
#define SLOW_OUTER_MM_S  (20)  /* travel of the outer wheel, mm/s */
#define UM_PER_MM_S_TICK (10)  /* travel of 1 mm/s over one tick, um */

/* Wheel speeds the command asks for, mm/s. */
static void wheel_speeds(pilot_cmd_t const * p_cmd, int32_t * p_left, int32_t * p_right)
{
    int32_t const half = ((int32_t)p_cmd->turn_deci_s * CHASSIS_WHEEL_TRACK_MM) / 1146;

    *p_left = p_cmd->speed_mm_s - half;
    *p_right = p_cmd->speed_mm_s + half;
}

/* The inner wheel is commanded below the deadband and does not turn, while
 * the outer wheel turns slowly: a pivot about one wheel with healthy
 * encoders. Neither a dead encoder nor stuck. */
static void test_slow_wheel_seam(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    unsigned t;
    unsigned n_slow = 0U;

    pilot_init(&st);
    for (t = 0U; t < 500U; t++)
    {
        int32_t l, r;

        m.now_ms = 1000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        m.b_line_left = false;
        m.b_line_right = true;
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_ENCODER != cmd.reason, "slow wheel read as a dead encoder");
        check(PILOT_REASON_STUCK != cmd.reason, "slow wheel read as stuck");
        wheel_speeds(&cmd, &l, &r);
        n_slow += ((r < PILOT_WHEEL_MIN_MM_S) && (l >= PILOT_WHEEL_MIN_MM_S)) ? 1U : 0U;
        m.left_um += (int64_t)SLOW_OUTER_MM_S * UM_PER_MM_S_TICK;
    }
    check(n_slow > ((1000U / TICK_MS) + 100U), "no slow inner wheel for over a second");
    (void)printf("slow wheel seam ok\n");
}

/* After a back-off and the retry, a stale window ends the retry: later
 * commands are not labelled STUCK once both wheels move again. */
static void test_retry_label_seam(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    unsigned t;
    bool b_back = false, b_retry = false;

    pilot_init(&st);
    check((uint8_t)PILOT_PHASE_TRY == st.phase, "a fresh pilot tries");
    for (t = 0U; t < 350U; t++)
    {
        m.now_ms = 1000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        pilot_step(&st, &m, &cmd);
        check((PILOT_REASON_STUCK != cmd.reason) || (0 != cmd.speed_mm_s) ||
                  (0 != cmd.turn_deci_s), "a zero command is labelled STUCK");
        if ((PILOT_REASON_STUCK == cmd.reason) && (cmd.speed_mm_s < 0))
        {
            b_back = true;
            check((PILOT_BEHAVIOUR_SAFETY != cmd.behaviour) ||
                      ((uint8_t)PILOT_PHASE_BACKOFF == st.phase),
                  "a back-off of both wheels is BACKOFF");
        }
        /* The retry after a stall is a contact look first (a pivot), the follower
         * after it. */
        if (b_back && (PILOT_REASON_STUCK == cmd.reason) &&
            ((cmd.speed_mm_s > 0) || (0 != cmd.turn_deci_s)))
        {
            b_retry = true;
            check((uint8_t)PILOT_PHASE_RETRY == st.phase, "the try after a back-off is RETRY");
        }
    }
    check(b_back && b_retry, "no back-off and retry labelled STUCK");
    /* This seam is the follower's label: drop the look, as when it ends. */
    hug_init(&st.hug);
    look_clear(&st.look);
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms - PILOT_STALE_MS - 50U;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_STALE == cmd.reason, "stale window stops");
    check((uint8_t)PILOT_PHASE_TRY == st.phase, "a stale window ends the retry");
    for (t = 0U; t < 100U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        m.left_um += 3000;
        m.right_um += 3000;
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_NONE == cmd.reason, "STUCK left on after a stale window");
        check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "follower runs again");
    }
    (void)printf("retry label seam ok\n");
}

/* While motion holds the wheels the pilot stops with HELD and does not read
 * the missing travel as stuck. */
static void test_held(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    unsigned t;

    pilot_init(&st);
    for (t = 0U; t < 50U; t++)
    {
        m.now_ms = 1000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        m.left_um += 3000;
        m.right_um += 3000;
        pilot_step(&st, &m, &cmd);
    }
    m.b_held = true;
    for (t = 0U; t < 300U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        pilot_step(&st, &m, &cmd);
        check((PILOT_REASON_HELD == cmd.reason) && (0 == cmd.speed_mm_s) &&
                  (0 == cmd.turn_deci_s) && (PILOT_BEHAVIOUR_SAFETY == cmd.behaviour),
              "a hold stops with HELD");
    }
    m.b_held = false;
    for (t = 0U; t < 150U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        m.left_um += 3000;
        m.right_um += 3000;
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_NONE == cmd.reason, "STUCK or ENCODER after a hold");
        check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "follower runs after a hold");
    }

    pilot_init(&st);
    pilot_estop(&st);
    m = fresh_meas(5000U);
    m.b_held = true;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_ESTOP == cmd.reason, "estop wins over a hold");

    /* A held tick restarts a stuck window that is nearly over. */
    pilot_init(&st);
    m = fresh_meas(8000U);
    for (t = 0U; t < 90U; t++)
    {
        m.now_ms = 8000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        pilot_step(&st, &m, &cmd);
    }
    m.now_ms += TICK_MS;
    m.stamp_ms = m.now_ms;
    m.b_held = true;
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_HELD == cmd.reason, "held tick in a stuck window");
    m.b_held = false;
    for (t = 0U; t < 50U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        pilot_step(&st, &m, &cmd);
        check(PILOT_REASON_NONE == cmd.reason, "a hold did not restart the stuck window");
    }
    (void)printf("held ok\n");
}

/* A tight turn: the inner wheel is commanded slowly for longer than the stuck
 * window while the encoders are healthy. Neither ENCODER nor STUCK. */
static void test_tight_turn(uint32_t seed)
{
    static pilot_sim_seg_t const tight[] = {
        {300.0, 0.0, 0.0}, {0.0, 25.0, 270.0}, {300.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    unsigned long n_slow = 0UL, n_run = 0UL;

    pilot_sim_init(&g_sim, tight, seed);
    while (g_sim.t_s < RUN_S)
    {
        pilot_sim_step(&g_sim);
        n_run++;
        if ((g_sim.vl < PILOT_WHEEL_MIN_MM_S) || (g_sim.vr < PILOT_WHEEL_MIN_MM_S))
        {
            n_slow++;
        }
        if ((PILOT_REASON_ENCODER == g_sim.cmd.reason) ||
            (PILOT_REASON_STUCK == g_sim.cmd.reason))
        {
            fail("a tight turn read as stuck or a dead encoder", seed);
        }
    }
    (void)printf("tight turn seed %5u: slow wheel %lu of %lu ticks\n", (unsigned)seed, n_slow, n_run);
}

/* One run. Properties 1 and 2 on every run; with b_stale also property 3. */
static void run_course(char const * p_name, pilot_sim_seg_t const * p_segs,
                       uint32_t seed, bool b_stale)
{
    unsigned long ticks = 0UL, off = 0UL;
    double still_s = 0.0, speed_sum = 0.0;
    double stale_seen_at = -1.0, recovered_at = -1.0;

    pilot_sim_init(&g_sim, p_segs, seed);
    if (b_stale)
    {
        g_sim.stale_from_s = STALE_FROM_S;
        g_sim.stale_len_s = STALE_LEN_S;
    }
    while (g_sim.t_s < RUN_S)
    {
        double const v = pilot_sim_speed_mm_s(&g_sim);
        bool const b_still = (g_sim.vl < STILL_SPEED) && (g_sim.vl > -STILL_SPEED) &&
                             (g_sim.vr < STILL_SPEED) && (g_sim.vr > -STILL_SPEED);

        pilot_sim_step(&g_sim);
        if (b_still && (PILOT_REASON_NONE == g_sim.cmd.reason))
        {
            still_s += 0.001;
            if (still_s > STILL_LIMIT_S)
            {
                fail("car stood still without a stop reason", seed);
            }
        }
        else
        {
            still_s = 0.0;
        }
        if (g_sim.t_s > START_S)
        {
            ticks++;
            speed_sum += v;
            if (pilot_sim_off_mm(&g_sim) > ON_LINE_MM)
            {
                off++;
            }
        }
        if (b_stale)
        {
            bool const b_in = (g_sim.t_s >= STALE_FROM_S) && (g_sim.t_s < (STALE_FROM_S + STALE_LEN_S));

            if (b_in && (PILOT_REASON_STALE == g_sim.cmd.reason) && (stale_seen_at < 0.0))
            {
                stale_seen_at = g_sim.t_s;
            }
            if (!b_in && (g_sim.t_s < STALE_FROM_S) && (PILOT_REASON_NONE != g_sim.cmd.reason))
            {
                fail("stop reason before the stale window", seed);
            }
            if ((g_sim.t_s >= (STALE_FROM_S + STALE_LEN_S + 0.02)) &&
                (PILOT_REASON_STALE == g_sim.cmd.reason))
            {
                fail("stale did not end by itself", seed);
            }
            if ((recovered_at < 0.0) && (g_sim.t_s > (STALE_FROM_S + STALE_LEN_S)) &&
                (pilot_sim_speed_mm_s(&g_sim) > 100.0))
            {
                recovered_at = g_sim.t_s;
            }
        }
    }
    {
        double const on_pct = 100.0 * (1.0 - ((double)off / (double)ticks));

        (void)printf("%-14s seed %5u: %.2f%% on line, mean %.0f mm/s%s\n", p_name,
                     (unsigned)seed, on_pct, speed_sum / (double)ticks,
                     b_stale ? " (stale window)" : "");
        if (on_pct < ON_LINE_MIN_PCT)
        {
            fail("car left the line", seed);
        }
    }
    if (b_stale)
    {
        if (stale_seen_at < 0.0)
        {
            fail("no STALE reason in the stale window", seed);
        }
        if ((recovered_at < 0.0) || (recovered_at > (STALE_FROM_S + STALE_LEN_S + RECOVER_S)))
        {
            fail("no recovery after the stale window", seed);
        }
    }
}

#define FAULT_AT_S       (3.0)
#define DEAD_HOLD_S      (11.0)
#define DEAD_END_S       (13.0)
#define BLOCK_LEN_S      (4.0)
#define BLOCK_END_S      (FAULT_AT_S + BLOCK_LEN_S)
#define BLOCK_LOOK_S     (30.0)
#define OBS_REJOIN_OFF_MM (25.0)

/* Run to t_s, calling the check of every step. */
static void run_to(double t_s)
{
    while (g_sim.t_s < t_s)
    {
        pilot_sim_step(&g_sim);
    }
}

static void test_dead_encoder(uint32_t seed, unsigned wheel)
{
    static pilot_sim_seg_t const straight[] = {{8000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    bool b_seen = false;
    bool b_one_wheel = false;

    pilot_sim_init(&g_sim, straight, seed);
    run_to(FAULT_AT_S);
    if (PILOT_REASON_NONE != g_sim.cmd.reason)
    {
        fail("stop reason before the encoder died", seed);
    }
    pilot_sim_kill_encoder(&g_sim, wheel);
    while (g_sim.t_s < DEAD_HOLD_S)
    {
        pilot_sim_step(&g_sim);
        b_one_wheel = b_one_wheel || ((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL == g_sim.pilot.phase);
        if ((PILOT_REASON_ENCODER == g_sim.cmd.reason) && !b_seen && !b_one_wheel)
        {
            fail("ENCODER without a one-wheel back-off", seed);
        }
        b_seen = b_seen || (PILOT_REASON_ENCODER == g_sim.cmd.reason);
        if ((g_sim.t_s > (FAULT_AT_S + 6.0)) && (PILOT_REASON_ENCODER != g_sim.cmd.reason))
        {
            fail("dead encoder stop did not hold", seed);
        }
    }
    if (!b_seen || (0 != g_sim.cmd.speed_mm_s) || (0 != g_sim.cmd.turn_deci_s))
    {
        fail("dead encoder did not stop with ENCODER", seed);
    }
    pilot_sim_fix_encoders(&g_sim);
    run_to(DEAD_HOLD_S + 0.5);
    if (PILOT_REASON_ENCODER != g_sim.cmd.reason)
    {
        fail("dead encoder stop ended without a reboot", seed);
    }
    pilot_init(&g_sim.pilot);
    run_to(DEAD_END_S);
    /* The car has drifted off the line while the encoder lied, so it may be
     * searching rather than going straight; it must command motion and not
     * stop again. */
    if ((PILOT_REASON_ENCODER == g_sim.cmd.reason) ||
        ((0 == g_sim.cmd.speed_mm_s) && (0 == g_sim.cmd.turn_deci_s)))
    {
        fail("no driving after the reboot", seed);
    }
}

static void test_blocked(uint32_t seed)
{
    static pilot_sim_seg_t const straight[] = {{8000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    bool b_back = false, b_retry = false;
    double x_block = 0.0;

    pilot_sim_init(&g_sim, straight, seed);
    g_sim.block_from_s = FAULT_AT_S;
    g_sim.block_len_s = BLOCK_LEN_S;
    run_to(FAULT_AT_S);
    x_block = g_sim.x;
    while (g_sim.t_s < BLOCK_END_S)
    {
        pilot_sim_step(&g_sim);
        if (PILOT_REASON_STUCK == g_sim.cmd.reason)
        {
            b_back = b_back || (g_sim.cmd.speed_mm_s < 0);
            /* The retry is a contact look first (a pivot), the follower after it. */
            b_retry = b_retry || (g_sim.cmd.speed_mm_s > 0) || (0 != g_sim.cmd.turn_deci_s);
            /* The block stops forward travel only: in a pivot one wheel still
             * turns, and the back-off is a one-wheel one. */
            if ((PILOT_BEHAVIOUR_SAFETY == g_sim.cmd.behaviour) &&
                ((uint8_t)PILOT_PHASE_BACKOFF != g_sim.pilot.phase) &&
                ((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL != g_sim.pilot.phase))
            {
                fail("a back-off is not in a back-off phase", seed);
            }
        }
        if (PILOT_REASON_ENCODER == g_sim.cmd.reason)
        {
            fail("blocked wheels read as a dead encoder", seed);
        }
    }
    if (!b_back || !b_retry)
    {
        fail("no back-off and retry with STUCK", seed);
    }
    /* A contact look comes before the drive on: the car backs and scans. */
    run_to(BLOCK_END_S + BLOCK_LOOK_S);
    if ((PILOT_REASON_NONE != g_sim.cmd.reason) || (g_sim.x < (x_block + 100.0)))
    {
        fail("no driving on after the block", seed);
    }
    /* Past the block is not enough: the car must be on its line again, not
     * part-way round a face that was never there. */
    if ((PILOT_BEHAVIOUR_FOLLOW_LINE != g_sim.cmd.behaviour) ||
        (fabs(g_sim.y) >= OBS_REJOIN_OFF_MM))
    {
        (void)printf("  behaviour %u, y %.0f mm\n",
                     (unsigned)g_sim.cmd.behaviour, g_sim.y);
        fail("the car is not back on its line after the block", seed);
    }
    (void)printf("blocked seed %5u: ok\n", (unsigned)seed);
}

#define DEG(a)  ((a) * 3.14159265358979323846 / 180.0)
#define TOL_MM  (PILOT_SIM_SONAR_NOISE_MM + 0.5)

static double sonar_at(unsigned want, char const * p_what)
{
    double mm = 0.0;
    unsigned const got = pilot_sim_sonar_from(&g_sim, 0.0, 0.0, 0.0, &mm);

    if (got != want)
    {
        (void)printf("FAIL: %s (class %u, want %u, %.1f mm)\n", p_what, got, want, mm);
        exit(EXIT_FAILURE);
    }
    return mm;
}

static bool near_mm(double got, double want)
{
    return (got > (want - TOL_MM)) && (got < (want + TOL_MM));
}

/* A long flat face 250 mm ahead of the origin, tilted by tilt_deg. */
static void face_ahead(double tilt_deg, double dist_mm)
{
    double const t = DEG(tilt_deg);

    (void)pilot_sim_add_wall(&g_sim, dist_mm - (1000.0 * sin(t)), -1000.0 * cos(t),
                             dist_mm + (1000.0 * sin(t)), 1000.0 * cos(t));
}

static void test_sonar(void)
{
    double mm;
    double v[16];
    double w[16];
    unsigned i;
    bool b_differ = false;

    pilot_sim_world_reset(&g_sim, 1U);
    face_ahead(0.0, 250.0);
    check(near_mm(sonar_at(PILOT_SIM_ECHO_VALID, "head-on face echoes"), 250.0), "head-on distance");

    pilot_sim_world_reset(&g_sim, 1U);
    face_ahead(25.0, 250.0);
    mm = sonar_at(PILOT_SIM_ECHO_VALID, "25 degree face echoes");
    check(mm > 200.0 && mm < 260.0, "25 degree distance");

    pilot_sim_world_reset(&g_sim, 1U);
    face_ahead(35.0, 250.0);
    (void)sonar_at(PILOT_SIM_ECHO_OPEN, "35 degree face is invisible");

    pilot_sim_world_reset(&g_sim, 1U);
    face_ahead(0.0, 250.0);
    face_ahead(0.0, 600.0);
    check(near_mm(sonar_at(PILOT_SIM_ECHO_VALID, "two faces"), 250.0), "nearer of two faces");

    pilot_sim_world_reset(&g_sim, 1U);
    face_ahead(35.0, 250.0);
    face_ahead(0.0, 600.0);
    check(near_mm(sonar_at(PILOT_SIM_ECHO_VALID, "far face behind grazing"), 600.0),
          "grazing near face is skipped");

    pilot_sim_world_reset(&g_sim, 1U);
    (void)pilot_sim_add_box(&g_sim, 250.0 + 20.0, 0.0, 40.0, 40.0, 0.0);
    check(near_mm(sonar_at(PILOT_SIM_ECHO_VALID, "narrow box"), 250.0), "narrow box nearest face");

    pilot_sim_world_reset(&g_sim, 1U);
    (void)pilot_sim_add_box(&g_sim, 250.0 + 20.0, 50.0, 40.0, 40.0, 0.0);
    check(near_mm(sonar_at(PILOT_SIM_ECHO_VALID, "box corner in beam"), sqrt((250.0 * 250.0) + 900.0)),
          "box corner region distance");

    pilot_sim_world_reset(&g_sim, 1U);
    (void)sonar_at(PILOT_SIM_ECHO_OPEN, "empty world is open");
    face_ahead(0.0, 10.0);
    (void)sonar_at(PILOT_SIM_ECHO_TOO_CLOSE, "face at 10 mm is too close");

    pilot_sim_world_reset(&g_sim, 77U);
    face_ahead(0.0, 250.0);
    for (i = 0U; i < 16U; i++)
    {
        pilot_sim_sonar_from(&g_sim, 0.0, 0.0, 0.0, &v[i]);
    }
    pilot_sim_world_reset(&g_sim, 77U);
    face_ahead(0.0, 250.0);
    for (i = 0U; i < 16U; i++)
    {
        pilot_sim_sonar_from(&g_sim, 0.0, 0.0, 0.0, &w[i]);
        check(v[i] == w[i], "noise repeats for the same seed");
        check(fabs(w[i] - 250.0) <= PILOT_SIM_SONAR_NOISE_MM + 1e-9, "noise within bound");
        b_differ = b_differ || (w[i] != w[0]);
    }
    check(b_differ, "noise varies");
}

/* ---- obstacles: the hug ---- */

#define OBS_SEEDS          (3U)
#define TOUCH_SEEDS        (12U)
#define TOUCH_N            (TOUCH_SEEDS + 1U)
#define OBS_RUN_S          (120.0)
#define OBS_FACE_X_MM      (1000.0)
#define OBS_CLEAR_MM       (50.0)
#define OBS_REJOIN_PAST_MM (300.0)
#define OBS_REJOIN_YAW_DEG (12.0)
#define OBS_WIDTH_TOL_MM   (25.0)
#define OBS_SIDE_MM        (100.0)
#define OBS_OFF_CENTRE_MM  (35.0)
#define OBS_LOOK_REACH_MM  (100.0)
#define OBS_LINE_END_MM    (8000.0)
#define OBS_WALL_HALF_MM   (3000.0)
#define OBS_PARALLEL_MM    (300.0)
#define OBS_SEEN_TOL_MM    (25.0)

/* State and then codes of hug.c, for a look at the hug by hand. */
#define HUG_CODE_ST_PIVOT    (2U)
#define HUG_CODE_THEN_APPROACH (2U)
#define HUG_CODE_SUB_SETTLE  (1U)
#define HUG_BUDGET_S       (30.0)

typedef enum
{
    OBS_BOX,       /* a square box, width and offset from the line */
    OBS_WALL,      /* a wall across the line                       */
    OBS_BOX_WALL,  /* a box with a wall alongside, at the left     */
    OBS_L,         /* an arm across the line with a foot           */
    OBS_ROOM       /* four walls around the line                   */
} obs_kind_t;

typedef struct
{
    char const * p_name;
    obs_kind_t   kind;
    double       width_mm;
    double       off_mm;
} obs_t;

/* What the run changes in the world or the sim; all zero is the plain run,
 * which also holds the width and the side to the truth. */
typedef struct
{
    char const * p_what;
    double       dropout;   /* chance that a VALID ray reads OPEN               */
    double       beam_deg;  /* sim beam half-angle, 0 keeps the pilot's         */
    double       rot_rad;   /* rotation of a box                                */
    uint32_t     clock_ms0; /* clock of the pilot at sim time 0                 */
} obs_opt_t;

static uint32_t const g_obs_seeds[OBS_SEEDS] = {1U, 7U, 12345U};

/* Seeds 1 to TOUCH_SEEDS, then 12345. */
static uint32_t touch_seed(unsigned i)
{
    return (i < TOUCH_SEEDS) ? (i + 1U) : g_obs_seeds[OBS_SEEDS - 1U];
}

static obs_t const g_obs[] = {
    {"box 60",              OBS_BOX, 60.0, 0.0},
    {"box 60 off",          OBS_BOX, 60.0, OBS_OFF_CENTRE_MM},
    {"box 120",             OBS_BOX, 120.0, 0.0},
    {"box 120 off",         OBS_BOX, 120.0, OBS_OFF_CENTRE_MM},
    {"box 200",             OBS_BOX, 200.0, 0.0},
    {"box 200 off",         OBS_BOX, 200.0, OBS_OFF_CENTRE_MM},
    {"box 300",             OBS_BOX, 300.0, 0.0},
    {"box 300 off",         OBS_BOX, 300.0, OBS_OFF_CENTRE_MM},
    {"box and side wall",   OBS_BOX_WALL, 200.0, 0.0},
    {"L shape",             OBS_L, 0.0, 0.0},
    {"wall across",         OBS_WALL, 0.0, 0.0},
    {"room",                OBS_ROOM, 0.0, 0.0},
};

#define OBS_N (sizeof(g_obs) / sizeof(g_obs[0]))

static obs_opt_t const g_obs_plain = {NULL, 0.0, 0.0, 0.0, 0U};

/* Robustness: the pilot assumes a 9.5 degree beam, a clean sonar and a face
 * square to the line. Each run keeps the clearance, shows no stop reason and
 * rejoins; the width is not held. The clock of the last one wraps (2^32 ms)
 * 30 s into the run, in the middle of the hug. */
#define ROB_BOXES        (4U)
#define ROB_ROTS         (2U)
#define ROB_WRAP_BEFORE_MS (30000U)

static unsigned const g_rob_box[ROB_BOXES] = {0U, 2U, 4U, 6U};

/* The box turned about the line. */
static double const g_rob_rot_deg[ROB_ROTS] = {10.0, 20.0};
static char const * const g_rob_rot_name[ROB_ROTS] = {"box turned 10 deg", "box turned 20 deg"};

static obs_opt_t const g_rob[] = {
    {"sonar dropout 5 %", 0.05, 0.0, 0.0, 0U},
    {"sim beam 8 deg", 0.0, 8.0, 0.0, 0U},
    {"sim beam 12 deg", 0.0, 12.0, 0.0, 0U},
    {"clock wraps", 0.0, 0.0, 0.0, (uint32_t)(0U - ROB_WRAP_BEFORE_MS)},
};

#define ROB_N (sizeof(g_rob) / sizeof(g_rob[0]))

/* Build the obstacle; returns the x the car must pass to count as rejoined. */
static double obs_build(obs_t const * p_obs, double rot_rad)
{
    double const x0 = OBS_FACE_X_MM;
    double far_x = x0;

    switch (p_obs->kind)
    {
    case OBS_BOX:
        (void)pilot_sim_add_box(&g_sim, x0 + (p_obs->width_mm / 2.0), p_obs->off_mm,
                                p_obs->width_mm, p_obs->width_mm, rot_rad);
        far_x = x0 + ((p_obs->width_mm / 2.0) * (1.0 + cos(rot_rad) + sin(rot_rad)));
        break;
    case OBS_BOX_WALL:
        (void)pilot_sim_add_box(&g_sim, x0 + (p_obs->width_mm / 2.0), 0.0, p_obs->width_mm,
                                p_obs->width_mm, 0.0);
        (void)pilot_sim_add_wall(&g_sim, 0.0, (p_obs->width_mm / 2.0) + OBS_PARALLEL_MM, 2500.0,
                                 (p_obs->width_mm / 2.0) + OBS_PARALLEL_MM);
        far_x = x0 + p_obs->width_mm;
        break;
    case OBS_L:
        (void)pilot_sim_add_box(&g_sim, x0 + 50.0, -100.0, 500.0, 100.0, 0.0);
        (void)pilot_sim_add_box(&g_sim, x0 + 300.0, -300.0, 100.0, 400.0, 0.0);
        far_x = x0 + 500.0;
        break;
    case OBS_WALL:
        (void)pilot_sim_add_wall(&g_sim, x0, -OBS_WALL_HALF_MM, x0, OBS_WALL_HALF_MM);
        break;
    default:
        (void)pilot_sim_add_room(&g_sim, -500.0, -700.0, 1500.0, 700.0);
        break;
    }
    return far_x;
}

static bool obs_forever(obs_kind_t kind)
{
    return (OBS_WALL == kind) || (OBS_ROOM == kind);
}

static double obs_truth_width(obs_t const * p_obs)
{
    return (OBS_BOX == p_obs->kind) ? p_obs->width_mm : 0.0;
}

static uint32_t g_hug_states_seen;

/* Time from the hug start to the follower back on the line; -1 without a rejoin. */
static double hug_span_s(double start_s, double rejoin_s)
{
    return (rejoin_s >= 0.0) ? (rejoin_s - start_s) : -1.0;
}

static void obs_run(obs_t const * p_obs, uint32_t seed, obs_opt_t const * p_opt)
{
    static pilot_sim_seg_t const straight[] = {{OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double const rad_deg = 180.0 / 3.14159265358979323846;
    double clear_min = 1e9, still_s = 0.0, y_min = 0.0, y_max = 0.0;
    double rejoin_s = -1.0, far_x;
    double w_seen = 0.0, hug_start_s = -1.0, hug_s = 0.0;
    bool b_hug = false, b_seen_checked = false;
    unsigned long hug_ticks = 0UL;

    pilot_sim_init(&g_sim, straight, seed);
    g_sim.dropout = p_opt->dropout;
    g_sim.sonar_half_deg = (p_opt->beam_deg > 0.0) ? p_opt->beam_deg : g_sim.sonar_half_deg;
    g_sim.clock_ms0 = p_opt->clock_ms0;
    far_x = obs_build(p_obs, p_opt->rot_rad);
    g_soft = (NULL != p_opt->p_what);
    g_run_failed = false;
    while ((g_sim.t_s < OBS_RUN_S) && (rejoin_s < 0.0) && !g_run_failed)
    {
        bool const b_still = (g_sim.vl < STILL_SPEED) && (g_sim.vl > -STILL_SPEED) &&
                             (g_sim.vr < STILL_SPEED) && (g_sim.vr > -STILL_SPEED);
        double const clear = pilot_sim_wall_dist_mm(&g_sim) - PILOT_SIM_CAR_RADIUS_MM;

        pilot_sim_step(&g_sim);
        clear_min = (clear < clear_min) ? clear : clear_min;
        y_min = (g_sim.y < y_min) ? g_sim.y : y_min;
        y_max = (g_sim.y > y_max) ? g_sim.y : y_max;
        if ((clear < OBS_CLEAR_MM) && !g_soft)
        {
            (void)printf("  clearance %.0f mm at x %.0f y %.0f\n", clear, g_sim.x, g_sim.y);
            fail("car body closer than the clearance to a face", seed);
        }
        if (PILOT_REASON_NONE != g_sim.cmd.reason)
        {
            fail("stop reason from an obstacle", seed);
        }
        if (g_sim.b_contact)
        {
            fail("the car touched a face", seed);
        }
        g_hug_states_seen |= 1UL << hug_word_state(&g_sim.pilot.hug);
        /* The first leg plan with an approach: the distance the frame draws
         * is the axle to the face as measured, not what is left after the
         * approach. */
        if (!b_seen_checked && (OBS_BOX == p_obs->kind) &&
            (0.0 == p_opt->rot_rad) && (g_sim.pilot.hug.app_mm > 0) &&
            (HUG_CODE_ST_PIVOT == g_sim.pilot.hug.state) &&
            (HUG_CODE_THEN_APPROACH == g_sim.pilot.hug.then))
        {
            double const truth = OBS_FACE_X_MM - g_sim.x;

            b_seen_checked = true;
            if (fabs((double)g_sim.pilot.hug.seen_mm - truth) > OBS_SEEN_TOL_MM)
            {
                (void)printf("  seen %d mm, axle to face %.0f mm, "
                             "approach %d mm\n",
                             (int)g_sim.pilot.hug.seen_mm, truth,
                             (int)g_sim.pilot.hug.app_mm);
                fail("the frame face distance is not the measured one", seed);
            }
        }
        if (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour)
        {
            hug_start_s = b_hug ? hug_start_s : g_sim.t_s;
            b_hug = true;
            hug_ticks++;
            if (g_sim.cmd.speed_mm_s < 0)
            {
                fail("the hug reversed", seed);
            }
            w_seen = (0U != g_sim.pilot.hug.width_mm) ? (double)g_sim.pilot.hug.width_mm : w_seen;
        }
        still_s = b_still ? (still_s + 0.001) : 0.0;
        if (still_s > STILL_LIMIT_S)
        {
            fail("car stood still too long during the hug", seed);
        }
        if (!obs_forever(p_obs->kind) && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) &&
            b_hug && (g_sim.x > (far_x + OBS_REJOIN_PAST_MM)) &&
            (fabs(g_sim.y) < OBS_REJOIN_OFF_MM) && (fabs(g_sim.psi * rad_deg) < OBS_REJOIN_YAW_DEG))
        {
            rejoin_s = g_sim.t_s;
        }
    }
    if (g_soft && (clear_min < OBS_CLEAR_MM))
    {
        fail("car body closer than the clearance to a face", seed);
    }
    if (!b_hug)
    {
        fail("the hug never took control", seed);
    }
    if (obs_forever(p_obs->kind))
    {
        if (hug_ticks < (unsigned long)(OBS_RUN_S * 1000.0 / 10.0 / 2.0))
        {
            fail("the car did not keep hugging the wall", seed);
        }
    }
    else if (rejoin_s < 0.0)
    {
        fail("the car did not rejoin the line past the obstacle", seed);
    }
    else if ((0U != g_sim.pilot.hug.width_mm) || (0U != g_sim.pilot.hug.breadth_mm) ||
             (0 != g_sim.pilot.hug.seen_mm))
    {
        fail("the hug kept a width or a face after it ended", seed);
    }
    else if (NULL != p_opt->p_what)
    {
        /* A run of the robustness set: no width, no side. */
    }
    else if (OBS_BOX == p_obs->kind)
    {
        /* The car no longer backs off to see the box whole, so a wide box may
         * never show both edges: no width (0) is allowed, a reported one is true. */
        if ((w_seen > 0.0) && (fabs(w_seen - obs_truth_width(p_obs)) > OBS_WIDTH_TOL_MM))
        {
            (void)printf("  width %.0f mm, truth %.0f mm\n", w_seen, obs_truth_width(p_obs));
            fail("reported width off the truth", seed);
        }
        if ((0.0 == p_obs->off_mm) && (y_max < OBS_SIDE_MM))
        {
            fail("a centred box was not passed on the left", seed);
        }
        /* The contact look at the standoff sees 250 mm * tan(12.5 + 9.5 deg),
         * about 100 mm to the side; a nearer edge farther out cannot be seen,
         * so the side choice cannot know which side is nearer. Not measured. */
        if ((p_obs->off_mm > 0.0) && (((p_obs->width_mm / 2.0) - p_obs->off_mm) <= OBS_LOOK_REACH_MM) &&
            (y_min > -OBS_SIDE_MM))
        {
            fail("an off-centre box was not passed on the nearer side", seed);
        }
    }
    else if (OBS_BOX_WALL == p_obs->kind)
    {
        if (y_min > -OBS_SIDE_MM)
        {
            fail("the car did not pass on the open side", seed);
        }
    }
    else
    {
        /* L shape: rejoined. */
    }
    if (OBS_WALL == p_obs->kind)
    {
        if (w_seen > 0.0)
        {
            fail("a wall reported a width", seed);
        }
    }
    hug_s = hug_span_s(hug_start_s, rejoin_s);
    if ((NULL == p_opt->p_what) && (OBS_BOX == p_obs->kind) && (hug_s > HUG_BUDGET_S))
    {
        (void)printf("BUDGET: %s seed %u: hug %.1f s, budget %.1f s\n", p_obs->p_name,
                     (unsigned)seed, hug_s, HUG_BUDGET_S);
        g_budget_overruns++;
    }
    (void)printf("%-18s seed %5u: clearance min %4.0f mm, rejoin %6.1f s, hug %5.1f s, width %3u mm (truth %.0f)%s%s\n",
                 p_obs->p_name, (unsigned)seed, clear_min, rejoin_s, hug_s,
                 (unsigned)w_seen, obs_truth_width(p_obs),
                 (NULL != p_opt->p_what) ? " : " : "", (NULL != p_opt->p_what) ? p_opt->p_what : "");
    g_soft = false;
}

/* ---- obstacles on a line that does not go straight on ---- */

/* The line goes under the box and turns there, or the box is long: the car
 * must go round as far as the line takes it, whatever the side it chose, and
 * follow the line away from the box. The line leaves the obstacle along
 * (course.dir_x, course.dir_y) from (OBS_FACE_X_MM + 100, 0). */
#define CRS_SEEDS        (3U)
#define CRS_RUN_S        (240.0)
#define CRS_BOX_MM       (200.0)
#define CRS_LONG_MM      (800.0)
#define CRS_TURN_X_MM    (OBS_FACE_X_MM + 100.0)
#define CRS_TURN_R_MM    (150.0)
#define CRS_YAW_DEG      (12.0)
#define CRS_LINE_MM      (25.0)
#define CRS_WRAP_SLACK   (35.0)

typedef struct
{
    char const *            p_name;
    pilot_sim_seg_t const * p_segs;
    double                  depth_mm;  /* of the box along the line            */
    double                  off_mm;    /* of the box from the line, left plus  */
    double                  dir_x;     /* the line leaves the obstacle this way */
    double                  dir_y;
    double                  s_goal_mm; /* the rejoin counts this far along it */
    double                  wrap_deg;     /* the corners turned, 90 degrees each   */
    double                  alt_wrap_deg; /* the other side, when the car may take it */
} course_t;

/* Straight to the turn, then a quarter circle, then straight on. */
static pilot_sim_seg_t const g_crs_right[] = {
    {CRS_TURN_X_MM - CRS_TURN_R_MM + 200.0, 0.0, 0.0}, {0.0, CRS_TURN_R_MM, -90.0},
    {OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};
static pilot_sim_seg_t const g_crs_left[] = {
    {CRS_TURN_X_MM - CRS_TURN_R_MM + 200.0, 0.0, 0.0}, {0.0, CRS_TURN_R_MM, 90.0},
    {OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};
static pilot_sim_seg_t const g_crs_straight[] = {{OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};

/* A left wrap turns the car right at every corner. The wrap is the turning
 * from the first leg to the exit: 90 degrees a corner. A box on the line has
 * no nearer edge, so the side follows the few mm the car is off the line, and
 * either wrap is right; a box 35 mm off the line sends the car the long way. */
static course_t const g_course[] = {
    {"line turns right",       g_crs_right,    CRS_BOX_MM,  0.0,   0.0, -1.0,  400.0, 270.0,  90.0},
    {"line turns left",        g_crs_left,     CRS_BOX_MM,  0.0,   0.0,  1.0,  400.0,  90.0, 270.0},
    {"long box",               g_crs_straight, CRS_LONG_MM, 0.0,   1.0,  0.0, 1000.0, 180.0, 180.0},
    {"wrong side, turns right", g_crs_right,   CRS_BOX_MM, -OBS_OFF_CENTRE_MM, 0.0, -1.0, 400.0, 270.0, 270.0},
    {"wrong side, turns left", g_crs_left,     CRS_BOX_MM,  OBS_OFF_CENTRE_MM, 0.0,  1.0, 400.0, 270.0, 270.0},
};

#define CRS_N (sizeof(g_course) / sizeof(g_course[0]))

static double unwrap_step(double now, double before)
{
    double d = now - before;

    d = (d > 3.14159265358979323846) ? (d - (2.0 * 3.14159265358979323846)) : d;
    d = (d < -3.14159265358979323846) ? (d + (2.0 * 3.14159265358979323846)) : d;
    return d;
}

static void course_run(course_t const * p_crs, uint32_t seed)
{
    double const rad_deg = 180.0 / 3.14159265358979323846;
    double const goal_psi = atan2(p_crs->dir_y, p_crs->dir_x);
    double clear_min = 1e9, still_s = 0.0, rejoin_s = -1.0, psi_u = 0.0, last_psi = 0.0;
    double psi_leg = 0.0, psi_exit = 0.0, hug_start_s = -1.0;
    bool b_hug = false, b_leg = false, b_exit = false;

    pilot_sim_init(&g_sim, p_crs->p_segs, seed);
    (void)pilot_sim_add_box(&g_sim, OBS_FACE_X_MM + (p_crs->depth_mm / 2.0), p_crs->off_mm,
                            CRS_BOX_MM, p_crs->depth_mm, 0.0);
    last_psi = g_sim.psi;
    g_soft = false;
    while ((g_sim.t_s < CRS_RUN_S) && (rejoin_s < 0.0))
    {
        bool const b_still = (g_sim.vl < STILL_SPEED) && (g_sim.vl > -STILL_SPEED) &&
                             (g_sim.vr < STILL_SPEED) && (g_sim.vr > -STILL_SPEED);
        double const clear = pilot_sim_wall_dist_mm(&g_sim) - PILOT_SIM_CAR_RADIUS_MM;
        uint8_t wire = 0U;

        pilot_sim_step(&g_sim);
        psi_u += unwrap_step(g_sim.psi, last_psi);
        last_psi = g_sim.psi;
        wire = hug_word_state(&g_sim.pilot.hug);
        clear_min = (clear < clear_min) ? clear : clear_min;
        if (clear < OBS_CLEAR_MM)
        {
            (void)printf("  clearance %.0f mm at x %.0f y %.0f\n", clear, g_sim.x, g_sim.y);
            fail("car body closer than the clearance to a face", seed);
        }
        if (PILOT_REASON_NONE != g_sim.cmd.reason)
        {
            fail("stop reason on a course", seed);
        }
        if (g_sim.b_contact)
        {
            fail("the car touched a face", seed);
        }
        if (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour)
        {
            hug_start_s = b_hug ? hug_start_s : g_sim.t_s;
            b_hug = true;
            if (g_sim.cmd.speed_mm_s < 0)
            {
                fail("the hug reversed", seed);
            }
        }
        if (!b_leg && (TELEMETRY_HUG_STATE_LEG == wire))
        {
            b_leg = true;
            psi_leg = psi_u;
        }
        if (!b_exit && (TELEMETRY_HUG_STATE_EXIT == wire))
        {
            b_exit = true;
            psi_exit = psi_u;
        }
        still_s = b_still ? (still_s + 0.001) : 0.0;
        if (still_s > STILL_LIMIT_S)
        {
            fail("car stood still too long on a course", seed);
        }
        if (b_hug && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour))
        {
            double const rx = g_sim.x - CRS_TURN_X_MM;
            double const s = (rx * p_crs->dir_x) + (g_sim.y * p_crs->dir_y);
            double const lat = (rx * p_crs->dir_y) - (g_sim.y * p_crs->dir_x);
            double const dpsi = fabs(unwrap_step(g_sim.psi, goal_psi) * rad_deg);

            if ((s > p_crs->s_goal_mm) && (fabs(lat) < CRS_LINE_MM) && (dpsi < CRS_YAW_DEG))
            {
                rejoin_s = g_sim.t_s;
            }
        }
    }
    {
        double const wrap = -(double)g_sim.pilot.hug.side * (psi_exit - psi_leg) * rad_deg;

        (void)printf("%-22s seed %5u: clearance min %4.0f mm, rejoin %6.1f s, hug %5.1f s, wrapped %4.0f deg\n",
                     p_crs->p_name, (unsigned)seed, clear_min, rejoin_s,
                     hug_span_s(hug_start_s, rejoin_s), wrap);
        if (rejoin_s < 0.0)
        {
            fail("the car did not rejoin the line past the obstacle", seed);
        }
        if (!b_leg || !b_exit)
        {
            fail("the hug did not end by the line", seed);
        }
        if ((fabs(wrap - p_crs->wrap_deg) > CRS_WRAP_SLACK) &&
            (fabs(wrap - p_crs->alt_wrap_deg) > CRS_WRAP_SLACK))
        {
            fail("the car wrapped the wrong angle round the obstacle", seed);
        }
    }
}

static void test_courses(void)
{
    unsigned c, s;

    for (c = 0U; c < CRS_N; c++)
    {
        for (s = 0U; s < CRS_SEEDS; s++)
        {
            course_run(&g_course[c], g_obs_seeds[s]);
        }
    }
}

/* ---- a first contact nearer than the gap ---- */

/* The line jogs sideways and turns back toward the box, so the face first shows
 * nearer than the HUG_BODY_GAP_MM the wrap keeps. The car backs straight off by
 * the standoff minus the face range, looks again, and wraps. The tolerance is
 * the odometry and the sonar noise; it is not measured on the car. */
#define NEAR_BACK_TOL_MM (30.0)
#define NEAR_BOX_Y_MM    (325.0)

static pilot_sim_seg_t const g_near_segs[] = {
    {790.0, 0.0, 0.0}, {0.0, 150.0, 90.0}, {100.0, 0.0, 0.0}, {0.0, 75.0, -90.0},
    {OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};

/* A stall is the first contact with a blind wall across the line: the pilot backs
 * off and the hug gets the stall as a face at range 0. */
#define NEAR_BLIND_HALF_MM (300.0)

/* A stop reason is one that stops the car; STUCK labels the pilot's back-off. */
static bool near_stop(uint8_t reason)
{
    return (PILOT_REASON_NONE != reason) && (PILOT_REASON_STUCK != reason);
}

static void near_run(uint32_t seed, bool b_stall)
{
    double const rad_deg = 180.0 / 3.14159265358979323846;
    double clear_min = 1e9, back_mm = 0.0, want_mm = 0.0, rejoin_s = -1.0;
    double last_x = 0.0, last_y = 0.0, hug_start_s = -1.0;
    bool b_hug = false, b_back = false, b_out = false, b_contact_before = false, b_contact_after = false;
    bool b_leg_after = false, b_was_hug = false, b_drove = false;
    unsigned ends = 0U;
    double const line_y = b_stall ? 0.0 : NEAR_BOX_Y_MM;

    pilot_sim_init(&g_sim, b_stall ? g_crs_straight : g_near_segs, seed);
    if (b_stall)
    {
        (void)pilot_sim_add_blind_wall(&g_sim, OBS_FACE_X_MM, -NEAR_BLIND_HALF_MM, OBS_FACE_X_MM,
                                       NEAR_BLIND_HALF_MM);
    }
    else
    {
        (void)pilot_sim_add_box(&g_sim, OBS_FACE_X_MM + (CRS_BOX_MM / 2.0), NEAR_BOX_Y_MM,
                                CRS_BOX_MM, CRS_BOX_MM, 0.0);
    }
    g_soft = false;
    g_run_failed = false;
    while ((g_sim.t_s < CRS_RUN_S) && (rejoin_s < 0.0) && !g_run_failed)
    {
        double const clear = pilot_sim_wall_dist_mm(&g_sim) - PILOT_SIM_CAR_RADIUS_MM;
        uint8_t wire = 0U;

        pilot_sim_step(&g_sim);
        wire = hug_word_state(&g_sim.pilot.hug);
        /* A hug that gave the wheels back without a wrap: the look from the
         * backed-off pose saw no face, and the follower goes to the wall again,
         * which stalls it a second time. */
        if (b_stall && b_was_hug && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) &&
            !b_leg_after)
        {
            check(fabs(back_mm - want_mm) <= NEAR_BACK_TOL_MM,
                  "the back-off is the standoff minus the range");
            check(b_contact_after, "the car looks again after the back-off");
            ends++;
            back_mm = 0.0;
            b_back = false;
            b_out = false;
            b_drove = false;
            b_contact_before = false;
            b_contact_after = false;
        }
        b_was_hug = (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour);
        /* The stall touches the wall by definition, and the pilot's own
         * back-off leaves the car under the clearance: the check counts from
         * the first time the hug's back-off brings the car out to it, and the
         * minimum reported is the one after that drive has ended. */
        b_out = b_out || (b_back && (clear >= OBS_CLEAR_MM));
        b_drove = b_drove || (b_back && (TELEMETRY_HUG_STATE_BACKOFF != wire));
        if (!b_stall || b_drove)
        {
            clear_min = (clear < clear_min) ? clear : clear_min;
        }
        if (!b_stall || b_out)
        {
            if (clear < OBS_CLEAR_MM)
            {
                (void)printf("  clearance %.0f mm at x %.0f y %.0f\n", clear, g_sim.x, g_sim.y);
                fail("car body closer than the clearance to a face", seed);
            }
        }
        if (near_stop(g_sim.cmd.reason))
        {
            fail("stop reason on the near contact", seed);
        }
        if (g_sim.b_contact && !b_stall)
        {
            fail("the car touched a face", seed);
        }
        if (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour)
        {
            if (!b_hug)
            {
                (void)printf("  hug starts at x %.0f y %.0f psi %.0f deg\n", g_sim.x, g_sim.y,
                             g_sim.psi * rad_deg);
                hug_start_s = g_sim.t_s;
            }
            b_hug = true;
        }
        if (TELEMETRY_HUG_STATE_BACKOFF == wire)
        {
            if (!b_back)
            {
                /* The face range from the sonar, 70 mm ahead of the axle; the
                 * face is the plane x = OBS_FACE_X_MM. */
                want_mm = b_stall ? (double)HUG_STANDOFF_MM
                                  : ((double)HUG_STANDOFF_MM -
                                     (OBS_FACE_X_MM - g_sim.x - (double)LOOK_SONAR_FWD_MM));
                b_back = true;
            }
            back_mm += hypot(g_sim.x - last_x, g_sim.y - last_y);
        }
        else if (TELEMETRY_HUG_STATE_CONTACT == wire)
        {
            b_contact_before = b_contact_before || !b_back;
            b_contact_after = b_contact_after || b_back;
        }
        else if (b_back && (TELEMETRY_HUG_STATE_LEG == wire))
        {
            b_leg_after = true;
        }
        else
        {
            /* Other states of the hug. */
        }
        last_x = g_sim.x;
        last_y = g_sim.y;
        if (b_hug && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) &&
            (g_sim.x > (OBS_FACE_X_MM + CRS_BOX_MM + OBS_REJOIN_PAST_MM)) &&
            (fabs(g_sim.y - line_y) < OBS_REJOIN_OFF_MM) &&
            (fabs(unwrap_step(g_sim.psi, 0.0) * rad_deg) < OBS_REJOIN_YAW_DEG))
        {
            rejoin_s = g_sim.t_s;
        }
    }
    (void)printf("%-18s seed %5u: back-off %4.0f mm (want %4.0f), clearance min %4.0f mm, "
                 "rejoin %6.1f s, hug %5.1f s\n", b_stall ? "stall, blind wall" : "near contact",
                 (unsigned)seed, back_mm, want_mm, clear_min, rejoin_s,
                 hug_span_s(hug_start_s, rejoin_s));
    check(ends == (b_stall ? 1U : 0U), "a stall the look does not see a face after ends the hug once");
    check(b_back, "a first contact nearer than the gap backs off (wire BACKOFF seen)");
    check(fabs(back_mm - want_mm) <= NEAR_BACK_TOL_MM, "the back-off is the standoff minus the range");
    check((b_stall || b_contact_before) && b_contact_after, "the car looks again after the back-off");
    check(b_leg_after, "the car wraps after the second look");
    check(rejoin_s >= 0.0, "the car rejoins the line past the box");
}

/* The trigger: two of the last three forward pings are VALID hits at or
 * below the standoff. */
static void hug_feed(hug_t * p_hug, look_t * p_look, uint8_t seq, uint8_t echo,
                     uint16_t mm, bool b_reply, bool * p_b_takes)
{
    hug_in_t in = {0};
    hug_out_t out;

    in.now_ms = (uint32_t)seq * 60U;
    in.ping_seq = seq;
    in.ping_echo = echo;
    in.ping_mm = mm;
    in.b_ping_reply = b_reply;
    *p_b_takes = hug_step(p_hug, p_look, &in, &out);
}

static void test_hug_trigger(void)
{
    static look_t look;
    hug_t hug;
    bool b_takes = false;

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, HUG_STANDOFF_MM, false, &b_takes);
    check(!b_takes, "one hit does not trigger");
    hug_feed(&hug, &look, 2U, LOOK_ECHO_VALID, HUG_STANDOFF_MM + 1, false, &b_takes);
    check(!b_takes, "a miss does not trigger");
    hug_feed(&hug, &look, 3U, LOOK_ECHO_VALID, HUG_STANDOFF_MM, false, &b_takes);
    check(b_takes && hug_active(&hug), "two of the last three trigger");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    hug_feed(&hug, &look, 2U, LOOK_ECHO_OPEN, 0U, false, &b_takes);
    hug_feed(&hug, &look, 3U, LOOK_ECHO_OPEN, 0U, false, &b_takes);
    hug_feed(&hug, &look, 4U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    check(!b_takes, "hits three pings apart do not trigger");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, true, &b_takes);
    hug_feed(&hug, &look, 2U, LOOK_ECHO_VALID, 100U, true, &b_takes);
    hug_feed(&hug, &look, 3U, LOOK_ECHO_VALID, 100U, true, &b_takes);
    check(!b_takes, "replies to requests do not trigger");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_VALID, 100U, false, &b_takes);
    check(!b_takes, "one ping seen again does not count twice");

    hug_init(&hug);
    hug_feed(&hug, &look, 1U, LOOK_ECHO_TOO_CLOSE, 15U, false, &b_takes);
    hug_feed(&hug, &look, 2U, LOOK_ECHO_VALID, HUG_STANDOFF_MM, false, &b_takes);
    check(b_takes && hug_active(&hug), "a TOO_CLOSE ping counts as a hit");
    (void)printf("hug trigger ok\n");
}

static void test_sonar_fault(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = fresh_meas(1000U);

    pilot_init(&st);
    m.b_sonar_fault = true;
    pilot_step(&st, &m, &cmd);
    check((PILOT_REASON_SONAR == cmd.reason) && (0 == cmd.speed_mm_s) && (0 == cmd.turn_deci_s),
          "a sonar fault stops with SONAR");
    m.b_sonar_fault = false;
    m = fresh_meas(1010U);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "SONAR ends when the fault clears");
    (void)printf("sonar fault ok\n");
}

/* The sonar stops answering requests once the hug has taken the wheels. The
 * car must stand with SONAR within MUTE_STOP_S of the hug taking over, and never stand still without
 * a reason. With b_back the sonar answers again after MUTE_S and the hug goes
 * on round the box. */
#define MUTE_S       (6.0)
#define MUTE_STOP_S  (4.0)
#define MUTE_TAIL_S  (1.0)

static void test_sonar_silent(bool b_back, uint32_t seed)
{
    static pilot_sim_seg_t const straight[] = {{OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double const rad_deg = 180.0 / 3.14159265358979323846;
    double mute_s = -1.0, stop_s = -1.0, still_s = 0.0;
    double const far_x = OBS_FACE_X_MM + g_obs[4].width_mm;
    bool b_rejoined = false;

    pilot_sim_init(&g_sim, straight, seed);
    (void)obs_build(&g_obs[4], 0.0);
    while ((g_sim.t_s < OBS_RUN_S) && !b_rejoined && (b_back || (mute_s < 0.0) ||
                                                      (g_sim.t_s < (mute_s + MUTE_S))))
    {
        bool const b_still = (g_sim.vl < STILL_SPEED) && (g_sim.vl > -STILL_SPEED) &&
                             (g_sim.vr < STILL_SPEED) && (g_sim.vr > -STILL_SPEED);

        pilot_sim_step(&g_sim);
        if ((mute_s < 0.0) && hug_active(&g_sim.pilot.hug))
        {
            mute_s = g_sim.t_s;
            g_sim.b_mute = true;
        }
        if (g_sim.b_mute && b_back && (g_sim.t_s >= (mute_s + MUTE_S)))
        {
            g_sim.b_mute = false;
        }
        if (PILOT_REASON_SONAR == g_sim.cmd.reason)
        {
            stop_s = (stop_s < 0.0) ? g_sim.t_s : stop_s;
            if ((0 != g_sim.cmd.speed_mm_s) || (0 != g_sim.cmd.turn_deci_s))
            {
                fail("the SONAR stop moves the car", seed);
            }
            if (b_back && (g_sim.t_s > (mute_s + MUTE_S + MUTE_TAIL_S)))
            {
                fail("SONAR lasts after the sonar answers again", seed);
            }
        }
        else if (PILOT_REASON_NONE != g_sim.cmd.reason)
        {
            fail("a reason other than SONAR with a silent sonar", seed);
        }
        else
        {
            /* No reason. */
        }
        still_s = (b_still && (PILOT_REASON_NONE == g_sim.cmd.reason)) ? (still_s + 0.001) : 0.0;
        if (still_s > STILL_LIMIT_S)
        {
            fail("car stood still without a reason", seed);
        }
        if ((mute_s >= 0.0) && (stop_s < 0.0) && (g_sim.t_s > (mute_s + MUTE_STOP_S)))
        {
            fail("no SONAR stop within the bound", seed);
        }
        b_rejoined = b_back && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) &&
                     (g_sim.x > (far_x + OBS_REJOIN_PAST_MM)) && (fabs(g_sim.y) < OBS_REJOIN_OFF_MM) &&
                     (fabs(g_sim.psi * rad_deg) < OBS_REJOIN_YAW_DEG);
    }
    if (stop_s < 0.0)
    {
        fail("a silent sonar never gave SONAR", seed);
    }
    if (b_back && !b_rejoined)
    {
        fail("the hug did not go on when the sonar came back", seed);
    }
    (void)printf("sonar silent %s seed %5u: SONAR %.1f s after the hug took over\n",
                 b_back ? "returns" : "stays  ", (unsigned)seed, stop_s - mute_s);
}

/* A look of a hug that has gone round a corner sees one edge of its face: the
 * breadth an earlier look measured stays. The hug is put in its last ray of
 * such a look by hand (state, sub, purpose and flag codes as in hug.c), with
 * the face 250 mm from the sonar, ending 40 mm left of the axis and running on
 * to the right past the window of the look. */
#define HUG_CODE_ST_SCAN   (1U)
#define HUG_CODE_SUB_ASK   (2U)
#define HUG_CODE_PU_LOOK   (1U)
#define HUG_CODE_F_TURNED  (0x08U)
#define BREADTH_RAYS       (15U)
#define BREADTH_STEP_DECI  (50)
#define BREADTH_FACE_MM    (250.0)
#define BREADTH_LEFT_MM    (40.0)
#define BREADTH_MEASURED   (55U)

static void breadth_ray(int32_t servo_deci, uint8_t * p_echo, int32_t * p_mm)
{
    double const rad = 3.14159265358979323846 / 1800.0;
    double const hi = BREADTH_FACE_MM * tan(((double)servo_deci + (double)LOOK_BEAM_DECI) * rad);
    double const lo = BREADTH_FACE_MM * tan(((double)servo_deci - (double)LOOK_BEAM_DECI) * rad);
    double const a = lo;
    double const b = (hi < BREADTH_LEFT_MM) ? hi : BREADTH_LEFT_MM;
    double y = 0.0;

    *p_echo = (uint8_t)LOOK_ECHO_OPEN;
    *p_mm = 0;
    if (a <= b)
    {
        y = (a > 0.0) ? a : ((b < 0.0) ? b : 0.0);
        *p_echo = (uint8_t)LOOK_ECHO_VALID;
        *p_mm = (int32_t)(sqrt((BREADTH_FACE_MM * BREADTH_FACE_MM) + (y * y)) + 0.5);
    }
}

static void test_breadth_kept(void)
{
    static look_t look;
    look_pose_t const pose = {0, 0, 0};
    hug_t hug;
    hug_in_t in = {0};
    hug_out_t out;
    uint8_t echo = 0U;
    int32_t mm = 0;

    hug_init(&hug);
    look_clear(&look);
    for (uint32_t i = 0U; i < (BREADTH_RAYS - 1U); i++)
    {
        int32_t const servo = ((int32_t)i * BREADTH_STEP_DECI) - 350;

        breadth_ray(servo, &echo, &mm);
        (void)look_add(&look, &pose, servo, echo, mm);
    }
    hug.state = (uint8_t)HUG_CODE_ST_SCAN;
    hug.sub = (uint8_t)HUG_CODE_SUB_ASK;
    hug.purpose = (uint8_t)HUG_CODE_PU_LOOK;
    hug.flags = (uint8_t)HUG_CODE_F_TURNED;
    hug.idx = (uint8_t)(BREADTH_RAYS - 1U);
    hug.body = 11U;
    hug.req_seq = 5U;
    hug.breadth_mm = BREADTH_MEASURED;
    breadth_ray(350, &echo, &mm);
    in.now_ms = 1000U;
    in.b_ping_reply = true;
    in.ping_seq = 5U;
    in.ping_echo = echo;
    in.ping_mm = (uint16_t)mm;
    in.ping_servo_deci = 350;
    (void)hug_step(&hug, &look, &in, &out);
    check(hug.state != (uint8_t)HUG_CODE_ST_SCAN, "the last ray ends the look");
    check(BREADTH_MEASURED == hug.breadth_mm, "a look with one edge keeps the breadth");
    (void)printf("breadth kept ok\n");
}

/* The pivot before a look has stopped at the first body step and settled, so
 * the scan does not settle again: only when the car is already on that step at
 * the first tick of the scan. A pivot that has to turn settles after it. The
 * hug is put in the first move of a look by hand (codes as in hug.c). */
#define HUG_CODE_SUB_START   (3U)
#define HUG_CODE_F_AIMED     (0x800U)
#define AIM_STEP_DECI        (-250)
#define AIM_TICK_MS          (10U)

static void aim_head(hug_t * p_hug, int32_t deci)
{
    double const rad = ((double)deci / 1800.0) * 3.14159265358979323846;

    p_hug->diff_um = (int32_t)(rad * (double)CHASSIS_WHEEL_TRACK_MM * 1000.0);
}

/* Ticks from the pivot done to the request for a ping; -1 for none. */
static int aim_run(int32_t head_deci)
{
    static look_t look;
    hug_t hug;
    hug_in_t in = {0};
    hug_out_t out;
    int done_ms = -1;
    int req_ms = -1;
    unsigned t;

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_SCAN;
    hug.sub = (uint8_t)HUG_CODE_SUB_START;
    hug.purpose = (uint8_t)HUG_CODE_PU_LOOK;
    hug.flags = (uint16_t)HUG_CODE_F_AIMED;
    aim_head(&hug, head_deci);
    in.now_ms = 1000U;
    for (t = 0U; (t < 60U) && (req_ms < 0); t++)
    {
        (void)hug_step(&hug, &look, &in, &out);
        if ((done_ms < 0) && (0 == out.turn_deci_s))
        {
            done_ms = (int)(t * AIM_TICK_MS);
            aim_head(&hug, AIM_STEP_DECI);
        }
        else if (0U != out.b_req)
        {
            req_ms = (int)(t * AIM_TICK_MS);
        }
        else if (0 != out.turn_deci_s)
        {
            aim_head(&hug, AIM_STEP_DECI);
        }
        in.now_ms += AIM_TICK_MS;
    }
    return (req_ms < 0) ? -1 : (req_ms - done_ms);
}

static void test_aimed_settle(void)
{
    int const settled = aim_run(AIM_STEP_DECI);
    int const turned = aim_run(0);

    (void)printf("aimed settle: on the step %d ms, turned %d ms "
                 "(settle %u ms)\n",
                 settled, turned, (unsigned)HUG_SETTLE_MS);
    check((settled >= 0) && (settled < (int)HUG_SETTLE_MS),
          "a scan already on its step does not settle twice");
    check(turned >= (int)HUG_SETTLE_MS,
          "a scan that had to turn settles after the turn");
}

/* The note of a stall hug that gave the wheels back: the car moves about while
 * the hug is idle (wheel travel only), then stalls. A stall at the place of the
 * last one is a repeat (flag code as in hug.c); one anywhere else is a first
 * stall. The car is put at the pose the release leaves it in, 260 mm before the
 * stall. */
#define HUG_CODE_F_REPEAT    (0x200U)
#define HUG_CODE_F_EXIT      (0x04U)
#define HUG_CODE_THEN_RELEASE (8U)
#define NOTE_RELEASE_X_MM    (-260)
#define NOTE_TICK_MS         (50U)
#define NOTE_PIVOT_MS        (10U)
#define NOTE_STEP_MM         (10)
#define NOTE_STEP_UM         ((int64_t)NOTE_STEP_MM * 1000)
#define NOTE_WAIT_MS         (60000U)
#define NOTE_ARC_RADIUS_MM   (300)
#define NOTE_LONG_HUG_MS     (20000U)
#define NOTE_HIST_HITS       (0x07U)

typedef struct
{
    hug_t    hug;
    hug_in_t in;
} note_car_t;

static look_t g_note_look;

static void note_start(note_car_t * p_car)
{
    hug_init(&p_car->hug);
    look_clear(&g_note_look);
    p_car->in = (hug_in_t){0};
    p_car->in.now_ms = 1000U;
    p_car->hug.b_note = 1U;
    p_car->hug.t_ms = p_car->in.now_ms;
    p_car->hug.x_um = NOTE_RELEASE_X_MM * 1000;
}

static void note_tick(note_car_t * p_car, int64_t left_um, int64_t right_um,
                      uint32_t dt_ms)
{
    hug_out_t out;

    p_car->in.left_um += left_um;
    p_car->in.right_um += right_um;
    p_car->in.now_ms += dt_ms;
    (void)hug_step(&p_car->hug, &g_note_look, &p_car->in, &out);
}

static void note_forward(note_car_t * p_car, int mm)
{
    for (int i = 0; i < (mm / NOTE_STEP_MM); i++)
    {
        note_tick(p_car, NOTE_STEP_UM, NOTE_STEP_UM, NOTE_TICK_MS);
    }
}

/* Turn in place, degrees to the left, a degree a tick; the hug reads 80 percent
 * of the wheel travel of a pivot as turning. */
static void note_turn(note_car_t * p_car, int deg)
{
    double const pct = (double)HUG_PIVOT_PCT / 100.0;
    double const um = (3.14159265358979323846 / 180.0) *
                      (double)CHASSIS_WHEEL_TRACK_MM * 1000.0 / pct;
    int const n = (deg < 0) ? -deg : deg;
    int64_t const half = (int64_t)(um / 2.0) * ((deg < 0) ? -1 : 1);

    for (int i = 0; i < n; i++)
    {
        note_tick(p_car, -half, half, NOTE_PIVOT_MS);
    }
}

static void note_wait(note_car_t * p_car, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += NOTE_TICK_MS)
    {
        note_tick(p_car, 0, 0, NOTE_TICK_MS);
    }
}

/* A gentle arc to the left, a tick a step, the wheels apart by the track over
 * the radius. The car circles near the place of the first stall. */
static void note_arc(note_car_t * p_car, int ticks)
{
    double const half = (double)NOTE_STEP_UM * (double)CHASSIS_WHEEL_TRACK_MM /
                        (2.0 * (double)NOTE_ARC_RADIUS_MM);

    for (int i = 0; i < ticks; i++)
    {
        note_tick(p_car, NOTE_STEP_UM - (int64_t)half,
                  NOTE_STEP_UM + (int64_t)half, NOTE_TICK_MS);
    }
}

/* The hug is put in the settle before its release by hand. */
static void note_release(note_car_t * p_car, uint16_t flags)
{
    p_car->hug.state = (uint8_t)HUG_CODE_ST_PIVOT;
    p_car->hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    p_car->hug.then = (uint8_t)HUG_CODE_THEN_RELEASE;
    p_car->hug.flags = flags;
    note_tick(p_car, 0, 0, HUG_SETTLE_MS);
}

static bool note_stall_is_repeat(note_car_t * p_car)
{
    p_car->in.b_stall = true;
    note_tick(p_car, 0, 0, NOTE_TICK_MS);
    p_car->in.b_stall = false;
    return hug_active(&p_car->hug) &&
           (0U != (p_car->hug.flags & HUG_CODE_F_REPEAT));
}

static void test_note(void)
{
    static note_car_t car;
    bool b_repeat = false;

    /* Round and back to the wall: the same place, though the wheels travelled
     * 900 mm. */
    note_start(&car);
    note_turn(&car, 180);
    note_forward(&car, 300);
    note_turn(&car, 180);
    note_forward(&car, 600);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall at the same place after a turn round is a repeat");

    /* 450 mm on, past the 150 mm: a new place. */
    note_start(&car);
    note_forward(&car, 450);
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall far past the first is not a repeat");

    /* The line turns away and the car goes far along it, within the note's
     * time. */
    note_start(&car);
    note_turn(&car, 90);
    note_forward(&car, 800);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall within the note's life, line turned away: repeat");

    /* The same path, and the note has run out. */
    note_start(&car);
    note_turn(&car, 90);
    note_forward(&car, 800);
    note_wait(&car, NOTE_WAIT_MS);
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall long after the first is not a repeat");

    /* A hug that ends at the tape clears the note, whatever the car did. */
    note_start(&car);
    car.hug.state = (uint8_t)HUG_CODE_ST_PIVOT;
    car.hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    car.hug.then = (uint8_t)HUG_CODE_THEN_RELEASE;
    car.hug.flags = (uint16_t)HUG_CODE_F_EXIT;
    note_tick(&car, 0, 0, HUG_SETTLE_MS);
    check(!hug_active(&car.hug) && (0U == car.hug.b_note),
          "a hug that ends at the tape clears the note");
    note_start(&car);
    car.hug.state = (uint8_t)HUG_CODE_ST_PIVOT;
    car.hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    car.hug.then = (uint8_t)HUG_CODE_THEN_RELEASE;
    note_tick(&car, 0, 0, HUG_SETTLE_MS);
    check(!hug_active(&car.hug) && (0U != car.hug.b_note),
          "a release with no wrap keeps the note");

    /* A hug that ends at the tape clears the count of releases as well. */
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    note_release(&car, (uint16_t)HUG_CODE_F_EXIT);
    check(!hug_active(&car.hug) && (0U == car.hug.n_rel),
          "a hug that ends at the tape clears the count of releases");
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall after the tape is not a repeat, count or not");

    /* Past the releases allowed, the next stall wraps whatever the note says;
     * the count outlives the note cleared by travel, and times out alone. */
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    note_forward(&car, 450);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "the count stands after the note was cleared by travel");
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    note_forward(&car, 450);
    note_wait(&car, NOTE_WAIT_MS);
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "the count times out after the note was cleared");

    /* A hug that began with no stall (the sonar) starts the count again. */
    note_start(&car);
    car.hug.n_rel = (uint8_t)HUG_RELEASES_MAX;
    car.hug.hist = (uint8_t)NOTE_HIST_HITS;
    note_tick(&car, 0, 0, NOTE_TICK_MS);
    check(hug_active(&car.hug) && (0U == car.hug.n_rel),
          "a hug started by the sonar starts the count again");
    car.hug.state = 0U;
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat, "a stall after a hug the sonar started is not a repeat");

    /* The note's time runs from the end of the hug, not from its start. */
    note_start(&car);
    car.in.now_ms += NOTE_LONG_HUG_MS;
    note_release(&car, 0U);
    note_wait(&car, 1000U);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall just after a long hug is still a repeat");

    /* The car circles near the first stall and only the time clears the note:
     * a repeat at exactly HUG_NOTE_MS, none one tick past it. */
    note_start(&car);
    note_arc(&car, (int)(HUG_NOTE_MS / NOTE_TICK_MS) - 1);
    b_repeat = note_stall_is_repeat(&car);
    check(b_repeat, "a stall at exactly HUG_NOTE_MS is still a repeat");
    note_start(&car);
    note_arc(&car, (int)(HUG_NOTE_MS / NOTE_TICK_MS));
    b_repeat = note_stall_is_repeat(&car);
    check(!b_repeat,
          "a stall one tick after the note runs out is not a repeat");
    (void)printf("stall note ok\n");
}

/* A move of the hug that makes no headway ends the hug after HUG_MOVE_TIMES
 * times its planned time, which is at least HUG_MOVE_FLOOR_MS. The hug is put
 * in the settle before the move by hand. With other_ms > 0, the tick that
 * many ms into the move is another behaviour's, and the clock of the move
 * starts again after it. Returns the time from the first tick of the move, or
 * of the restart, to the timeout; 0 for none within a minute. */
#define HUG_CODE_THEN_ALIGN  (3U)
#define APPROACH_MM          (100)
#define MOVE_TICKS_MAX       (6000U)

static uint32_t move_limit(uint32_t plan_ms)
{
    return HUG_MOVE_TIMES * ((plan_ms < HUG_MOVE_FLOOR_MS) ? HUG_MOVE_FLOOR_MS : plan_ms);
}

static uint32_t pivot_plan_ms(uint32_t angle_deci)
{
    return (angle_deci * 1000U * 100U) / (uint32_t)(HUG_PIVOT_MIN_DECI_S * HUG_PIVOT_PCT);
}

static uint32_t stall_ms(uint8_t then, int16_t u_head, uint32_t other_ms)
{
    static look_t look;
    hug_t hug;
    hug_in_t in = {0};
    hug_out_t out;
    uint32_t start_ms = 0U;
    uint32_t got = 0U;
    unsigned t;

    hug_init(&hug);
    look_clear(&look);
    hug.state = (uint8_t)HUG_CODE_ST_PIVOT;
    hug.sub = (uint8_t)HUG_CODE_SUB_SETTLE;
    hug.then = then;
    hug.u_head = u_head;
    hug.app_mm = APPROACH_MM;
    in.now_ms = HUG_SETTLE_MS;
    check(hug_step(&hug, &look, &in, &out), "the settle ends in the move");
    start_ms = in.now_ms + TICK_MS;
    for (t = 0U; (t < MOVE_TICKS_MAX) && (0U == got); t++)
    {
        in.now_ms += TICK_MS;
        in.b_other = (other_ms > 0U) && ((in.now_ms - start_ms) == other_ms);
        (void)hug_step(&hug, &look, &in, &out);
        if (in.b_other)
        {
            start_ms = in.now_ms + TICK_MS;
            other_ms = 0U;
        }
        else if (0U != out.b_timeout)
        {
            check(!hug_active(&hug), "a move timeout ends the hug");
            check((0 == out.speed_mm_s) && (0 == out.turn_deci_s), "a move timeout stands");
            got = in.now_ms - start_ms;
        }
        else
        {
            check(hug_active(&hug) && ((0 != out.speed_mm_s) || (0 != out.turn_deci_s)),
                  "a stalled move goes on until it times out");
        }
    }
    return got;
}

static void check_limit(uint32_t got_ms, uint32_t limit_ms, char const * p_what)
{
    (void)printf("move timeout %s: %u ms (limit %u ms)\n", p_what, (unsigned)got_ms,
                 (unsigned)limit_ms);
    check((got_ms > limit_ms) && (got_ms <= (limit_ms + TICK_MS)), p_what);
}

static void test_move_timeout(void)
{
    uint32_t const pivot_90 = move_limit(pivot_plan_ms(900U));
    uint32_t const pivot_10 = move_limit(pivot_plan_ms(100U));
    uint32_t const approach = move_limit(((uint32_t)APPROACH_MM * 1000U) /
                                            (uint32_t)HUG_SHORT_MM_S);

    check(pivot_10 == (HUG_MOVE_TIMES * HUG_MOVE_FLOOR_MS), "a short pivot gets the floor");
    check_limit(stall_ms(HUG_CODE_THEN_ALIGN, 900, 0U), pivot_90, "pivot of 90 degrees");
    check_limit(stall_ms(HUG_CODE_THEN_ALIGN, 100, 0U), pivot_10, "pivot of 10 degrees");
    check_limit(stall_ms(HUG_CODE_THEN_APPROACH, 0, 0U), approach, "approach drive");
    check_limit(stall_ms(HUG_CODE_THEN_ALIGN, 900, pivot_90 - 100U), pivot_90,
                "pivot restarted by another behaviour");
}

/* A pivot that crawls: both wheels turn at 3 mm/s, which the stuck check takes
 * for travel, so the first pivot of the contact look never ends. The move
 * timeout stops the car with HUG_TIMEOUT, and the stop holds. */
#define CRAWL_UM_TICK   (30)
#define HIT_MM          (200U)

static void test_hug_timeout(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd = {0};
    pilot_meas_t m = fresh_meas(1000U);
    unsigned t;
    bool b_hug = false;
    bool b_out = false;

    pilot_init(&st);
    m.ping_echo = LOOK_ECHO_VALID;
    m.ping_mm = HIT_MM;
    for (t = 0U; (t < 3000U) && !b_out; t++)
    {
        int64_t const step_um = (int64_t)cmd.speed_mm_s * UM_PER_MM_S_TICK;

        m.now_ms = 1000U + (t * TICK_MS);
        m.stamp_ms = m.now_ms;
        if (!b_hug)
        {
            m.ping_seq = (uint8_t)((m.ping_seq + 1U) & 0x3FU);
        }
        m.left_um += step_um + ((cmd.turn_deci_s > 0) ? -CRAWL_UM_TICK
                                : ((cmd.turn_deci_s < 0) ? CRAWL_UM_TICK : 0));
        m.right_um += step_um + ((cmd.turn_deci_s > 0) ? CRAWL_UM_TICK
                                 : ((cmd.turn_deci_s < 0) ? -CRAWL_UM_TICK : 0));
        pilot_step(&st, &m, &cmd);
        b_hug = b_hug || (PILOT_BEHAVIOUR_HUG == cmd.behaviour);
        b_out = (PILOT_REASON_HUG_TIMEOUT == cmd.reason);
        check(b_out || (PILOT_REASON_NONE == cmd.reason), "a crawling pivot gives another reason");
    }
    check(b_hug, "the hug did not take the wheels");
    check(b_out, "a crawling pivot did not time out");
    for (t = 0U; t < 100U; t++)
    {
        m.now_ms += TICK_MS;
        m.stamp_ms = m.now_ms;
        m.left_um += 3000;
        m.right_um += 3000;
        pilot_step(&st, &m, &cmd);
        check((PILOT_REASON_HUG_TIMEOUT == cmd.reason) && (PILOT_BEHAVIOUR_SAFETY == cmd.behaviour) &&
                  (0 == cmd.speed_mm_s) && (0 == cmd.turn_deci_s),
              "the HUG_TIMEOUT stop does not hold");
    }
    (void)printf("hug timeout ok\n");
}

static void test_robust(void)
{
    unsigned o, s, b;

    for (o = 0U; o < ROB_N; o++)
    {
        for (b = 0U; b < ROB_BOXES; b++)
        {
            for (s = 0U; s < OBS_SEEDS; s++)
            {
                obs_run(&g_obs[g_rob_box[b]], g_obs_seeds[s], &g_rob[o]);
            }
        }
    }
    for (b = 0U; b < ROB_ROTS; b++)
    {
        for (s = 0U; s < OBS_SEEDS; s++)
        {
            obs_opt_t const rotated = {g_rob_rot_name[b], 0.0, 0.0,
                                       g_rob_rot_deg[b] * 3.14159265358979323846 / 180.0, 0U};

            obs_run(&g_obs[4], g_obs_seeds[s], &rotated);
        }
    }
    (void)printf("robustness runs failed: %u\n", g_soft_fails);
    check(0U == g_soft_fails, "robustness runs failed");
}

/* ---- contact: a face the sonar cannot see ---- */

#define TOUCH_BOX_MM       (200.0)
#define TOUCH_BLIND_Y_MM   (250.0)
#define TOUCH_BLIND_X0_MM  (450.0)
#define TOUCH_BLIND_X1_MM  (700.0)
#define TOUCH_SLIP_S       (1.5)
#define TOUCH_SLIP_GAP_S   (0.5)
#define TOUCH_SLIP_RELEASES_MAX (2U)

typedef enum
{
    TOUCH_BOX_TURNED,  /* a 200 mm box turned about the line: both front faces
                          read OPEN */
    TOUCH_BLIND_LEG,   /* a 200 mm box, and a low face across the leg path */
    TOUCH_SLIP_STALL   /* a 45 degree box; at every contact the wheels slip,
                          then stall */
} touch_kind_t;

/* A stop reason is one that stops the car; STUCK labels the back-off. */
static bool stop_reason(uint8_t reason)
{
    return (PILOT_REASON_NONE != reason) && (PILOT_REASON_STUCK != reason);
}

/* Run to the rejoin. Report the contact, the clearance after the first
 * back-off, and the rejoin. With b_assert the run fails unless the car touched,
 * backed off, kept the clearance from then on and rejoined. */
static void touch_run(char const * p_name, touch_kind_t kind, double rot_deg, uint32_t seed,
                      bool b_slip, bool b_assert)
{
    static pilot_sim_seg_t const straight[] = {{OBS_LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double const rad = 3.14159265358979323846 / 180.0;
    double clear_after = 1e9, clear_min = 1e9, rejoin_s = -1.0, far_x;
    double stop_s = -1.0;
    bool b_back = false, b_backed = false, b_hug = false, b_clear = false;
    bool b_hug_back = false, b_drove = false;
    bool b_was_hug = false, b_wrap = false, b_wall = false;
    unsigned stop = 0U, releases = 0U, last_contacts = 0U, slip_ticks = 0U;
    double slip_until_s = -1.0, last_contact_s = -10.0;
    /* Slip at every contact costs the releases the hug allows before the wrap,
     * and the case is about that bound, not about the time. */
    double const run_s = (TOUCH_SLIP_STALL == kind) ? CRS_RUN_S : OBS_RUN_S;

    pilot_sim_init(&g_sim, straight, seed);
    g_sim.b_slip = b_slip;
    far_x = obs_build(&g_obs[4], rot_deg * rad);
    g_soft = true;
    g_run_failed = false;
    while ((g_sim.t_s < run_s) && (rejoin_s < 0.0) && !g_run_failed)
    {
        bool const b_stuck_cmd = (PILOT_REASON_STUCK == g_sim.cmd.reason) && (g_sim.cmd.speed_mm_s < 0);
        uint8_t wire = 0U;

        /* The slip starts with each contact, counts as travel and is not a
         * stall; the stall comes when it ends. */
        if (g_sim.contacts != last_contacts)
        {
            slip_until_s = ((g_sim.t_s - last_contact_s) > TOUCH_SLIP_GAP_S)
                               ? (g_sim.t_s + TOUCH_SLIP_S) : slip_until_s;
            last_contact_s = g_sim.t_s;
            last_contacts = g_sim.contacts;
        }
        g_sim.b_slip = b_slip || ((TOUCH_SLIP_STALL == kind) &&
                                  (g_sim.t_s < slip_until_s));
        slip_ticks += (g_sim.b_slip && (TOUCH_SLIP_STALL == kind)) ? 1U : 0U;
        pilot_sim_step(&g_sim);
        wire = hug_word_state(&g_sim.pilot.hug);
        b_wrap = b_wrap || (TELEMETRY_HUG_STATE_LEG == wire);
        /* The side of a box on the line comes from the noise, so the low face
         * goes on the side the hug has chosen, across the first leg. */
        if ((TOUCH_BLIND_LEG == kind) && !b_wall &&
            (TELEMETRY_HUG_STATE_ALIGN == wire))
        {
            double const y = (double)g_sim.pilot.hug.side * TOUCH_BLIND_Y_MM;

            (void)pilot_sim_add_blind_wall(&g_sim, TOUCH_BLIND_X0_MM, y,
                                           TOUCH_BLIND_X1_MM, y);
            b_wall = true;
        }
        /* A hug that gave the wheels back without a wrap saw no face from the
         * backed-off pose: the follower goes to the face again and stalls a
         * second time, which counts as a new contact. */
        if (b_was_hug && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) && !b_wrap)
        {
            if (b_assert && (!b_clear || (clear_after < OBS_CLEAR_MM)))
            {
                fail("car body closer than the clearance after the back-off", seed);
            }
            b_back = false;
            b_backed = false;
            b_hug_back = false;
            b_drove = false;
            b_clear = false;
            clear_after = 1e9;
            clear_min = 1e9;
            releases++;
        }
        b_was_hug = (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour);
        if (g_sim.b_contact && b_stuck_cmd)
        {
            b_back = true;
        }
        else if (b_back && !b_stuck_cmd)
        {
            b_backed = true;
        }
        else
        {
            /* Before the first back-off, or in it. */
        }
        if (b_backed)
        {
            double const clear = pilot_sim_wall_dist_mm(&g_sim) - PILOT_SIM_CAR_RADIUS_MM;

            /* The back-off ends PILOT_BACKOFF_UM from the face, below the
             * clearance, so the clearance counts once the car has got to it. */
            b_clear = b_clear || (clear >= OBS_CLEAR_MM);
            if (b_clear)
            {
                clear_after = (clear < clear_after) ? clear : clear_after;
            }
            /* What is reported is the minimum once the hug's back-off drive has
             * ended, or, for a stall in a leg where the hug does not reverse,
             * once the car stands to look after the first leg of the wrap. */
            b_drove = b_drove ||
                      (b_hug_back && (TELEMETRY_HUG_STATE_BACKOFF != wire)) ||
                      (TELEMETRY_HUG_STATE_LOOK == wire);
            b_hug_back = b_hug_back || (TELEMETRY_HUG_STATE_BACKOFF == wire);
            if (b_drove)
            {
                clear_min = (clear < clear_min) ? clear : clear_min;
            }
        }
        if (stop_reason(g_sim.cmd.reason))
        {
            stop++;
            stop_s = (stop_s < 0.0) ? g_sim.t_s : stop_s;
        }
        b_hug = b_hug || (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour);
        if (b_hug && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) &&
            (g_sim.x > (far_x + OBS_REJOIN_PAST_MM)) && (fabs(g_sim.y) < OBS_REJOIN_OFF_MM) &&
            (fabs(g_sim.psi / rad) < OBS_REJOIN_YAW_DEG))
        {
            rejoin_s = g_sim.t_s;
        }
    }
    (void)printf("%-22s seed %5u: contact %s (%u ms), clearance after back-off %4.0f mm, "
                 "rejoin %6.1f s, stop reasons %u%s\n",
                 p_name, (unsigned)seed, g_sim.b_contact ? "yes" : "no ", g_sim.contacts,
                 b_drove ? clear_min : -999.0, rejoin_s, stop,
                 b_slip ? " (wheel slip)" : "");
    if (b_assert)
    {
        if (!g_sim.b_contact)
        {
            fail("the car never touched the face", seed);
        }
        if (!b_backed)
        {
            fail("the car did not back off after the contact", seed);
        }
        if (0U != stop)
        {
            (void)printf("  first stop reason at %.2f s\n", stop_s);
            fail("stop reason after a contact", seed);
        }
        if (!b_clear || (clear_after < OBS_CLEAR_MM))
        {
            fail("car body closer than the clearance after the back-off", seed);
        }
        if (rejoin_s < 0.0)
        {
            fail("the car did not rejoin the line past the obstacle", seed);
        }
        if (TOUCH_SLIP_STALL == kind)
        {
            (void)printf("  slipped %u ticks, %u releases\n", slip_ticks,
                         releases);
            g_slip_releasing_seeds += (releases > 0U) ? 1U : 0U;
        }
        if ((TOUCH_SLIP_STALL == kind) && (0U == slip_ticks))
        {
            fail("the slip case never slipped", seed);
        }
        if ((TOUCH_SLIP_STALL == kind) && (releases > TOUCH_SLIP_RELEASES_MAX))
        {
            (void)printf("  %u releases without a wrap\n", releases);
            fail("the hug gave the wheels back again and again", seed);
        }
    }
    g_soft = false;
}

static void test_touch(void)
{
    unsigned const before = g_soft_fails;
    unsigned s;

    for (s = 0U; s < TOUCH_N; s++)
    {
        uint32_t const seed = touch_seed(s);

        touch_run("slip at every contact", TOUCH_SLIP_STALL, 45.0, seed, false,
                  true);
        touch_run("box 200 turned 35 deg", TOUCH_BOX_TURNED, 35.0, seed, false,
                  true);
        touch_run("box 200 turned 45 deg", TOUCH_BOX_TURNED, 45.0, seed, false,
                  true);
        touch_run("low face across a leg", TOUCH_BLIND_LEG, 0.0, seed, false,
                  true);
    }
    check(before == g_soft_fails, "contact runs failed");
    /* One seed's only slip window ends in a stall that wraps, so it never
     * releases; every other seed must. */
    check(g_slip_releasing_seeds >= (TOUCH_N - 1U),
          "the slip case did not give the wheels back on its seeds");
}

/* The risk: wheel slip instead of a stall. Reported, not asserted. */
static void test_touch_slip(void)
{
    unsigned s;

    for (s = 0U; s < OBS_SEEDS; s++)
    {
        touch_run("box 200 turned 35 deg", TOUCH_BOX_TURNED, 35.0, g_obs_seeds[s], true, false);
    }
}

/* A range just outside the normal hug trigger first falls, then stops
 * changing even as both encoders keep counting. The pilot gets only its
 * usual samples: no chassis position or physical contact flag. */
static void test_continuous_slip(bool b_check_restart)
{
    uint32_t const approach_ms = 1200U;
    uint32_t const reverse_deadline_ms = approach_ms + 110000U;
    uint32_t const look_window_ms = 15000U;
    uint16_t const plateau_mm = HUG_STANDOFF_MM + 20U;
    pilot_state_t pilot;
    drive_state_t drive;
    pilot_cmd_t command = {0};
    pilot_meas_t meas = fresh_meas(1000U);
    uint32_t reverse_at_ms = 0U;
    bool b_new_look = false;
    int32_t left_um_s = 0;
    int32_t right_um_s = 0;

    pilot_init(&pilot);
    drive_init(&drive);
    meas.b_line_right = true;
    meas.ping_echo = LOOK_ECHO_VALID;
    for (uint32_t elapsed_ms = 0U;
         elapsed_ms <= ((reverse_at_ms == 0U) ? reverse_deadline_ms
                                             : (reverse_at_ms + look_window_ms));
         elapsed_ms += TICK_MS)
    {
        uint8_t const old_request = command.b_ping_req;
        uint8_t const old_sequence = command.ping_seq;

        meas.now_ms = 1000U + elapsed_ms;
        meas.stamp_ms = meas.now_ms;
        if ((elapsed_ms % 60U) == 0U)
        {
            uint32_t const remaining_ms = (elapsed_ms < approach_ms) ?
                                           (approach_ms - elapsed_ms) : 0U;

            meas.ping_mm = (uint16_t)(plateau_mm + (remaining_ms / 10U));
            meas.ping_seq = (uint8_t)((meas.ping_seq + 1U) & 0x3FU);
        }
        if (elapsed_ms == approach_ms)
        {
            check(left_um_s > 0 && right_um_s > 0 && meas.left_um > 0 && meas.right_um > 0,
                  "both wheels were moving as the forward range stopped falling");
        }
        pilot_step(&pilot, &meas, &command);
        if (elapsed_ms >= approach_ms && reverse_at_ms == 0U && command.speed_mm_s < 0)
        {
            reverse_at_ms = elapsed_ms;
            if (!b_check_restart)
            {
                break;
            }
        }
        else if (reverse_at_ms != 0U && command.speed_mm_s >= 0 &&
                 command.behaviour == PILOT_BEHAVIOUR_HUG && command.b_ping_req != 0U &&
                 (old_request == 0U || old_sequence != command.ping_seq))
        {
            b_new_look = true;
            break;
        }
        pilot_drive(&drive, &command, TICK_MS, &left_um_s, &right_um_s);
        meas.left_um += ((int64_t)left_um_s * TICK_MS) / 1000;
        meas.right_um += ((int64_t)right_um_s * TICK_MS) / 1000;
    }

    if (b_check_restart)
    {
        check(b_new_look, "unchanged obstacle range and rolling wheels did not yield reverse then a new hug ping request");
    }
    else
    {
        check(reverse_at_ms != 0U,
              "fresh, unchanged obstacle range and rolling wheels did not trigger reverse within 110 seconds");
    }
}

static void test_unseen_face_approach(bool b_check_retry)
{
    double first_speed = 0.0;
    double first_time_s = 0.0;
    double retry_speed = 0.0;
    bool b_reversed = false;
    bool b_released = false;

    pilot_sim_init(&g_sim, g_crs_straight, 1U);
    check(pilot_sim_add_blind_wall(&g_sim, OBS_FACE_X_MM, -NEAR_BLIND_HALF_MM,
                                  OBS_FACE_X_MM, NEAR_BLIND_HALF_MM),
          "install the unseen face across the track");
    while ((g_sim.t_s < CRS_RUN_S) && (retry_speed <= 0.0))
    {
        unsigned const contacts = g_sim.contacts;
        double const approach_speed = pilot_sim_speed_mm_s(&g_sim);

        pilot_sim_step(&g_sim);
        if (g_sim.contacts != contacts)
        {
            if (first_speed <= 0.0)
            {
                first_speed = approach_speed;
                first_time_s = g_sim.t_s - 0.001;
            }
            else if (b_released)
            {
                retry_speed = approach_speed;
            }
        }
        b_reversed = b_reversed || (g_sim.cmd.speed_mm_s < 0);
        b_released = b_released || (b_reversed && (g_sim.cmd.speed_mm_s > 0) &&
            (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour));
    }
    (void)printf("unseen face: first impact %.1f mm/s, second impact %.1f mm/s\n",
                 first_speed, retry_speed);
    check(first_speed > 0.0, "the car did not drive to the unseen face");
    if (b_check_retry)
    {
        check(b_released && (retry_speed > 0.0), "the car did not reapproach the unseen face");
        check(retry_speed + 1.0 < first_speed, "the second impact was at normal cruise speed");
    }
    else
    {
        pilot_sim_init(&g_sim, g_crs_straight, 1U);
        run_to(first_time_s);
        check(fabs(first_speed - pilot_sim_speed_mm_s(&g_sim)) < 1.0,
              "the unseen face altered the first approach before contact");
        check(first_speed > 0.8 * (double)LINE_CRUISE_UM_S / 1000.0,
              "the first approach did not reach normal driving speed");
    }
}

static void test_stale_forward_ping(bool b_check_latch)
{
    pilot_state_t state;
    pilot_cmd_t command = {0};
    pilot_meas_t meas = fresh_meas(1000U);
    bool b_stopped = false;

    pilot_init(&state);
    meas.b_line_right = true;
    meas.ping_echo = LOOK_ECHO_OPEN;
    meas.ping_seq = 1U;
    pilot_step(&state, &meas, &command);
    check(command.speed_mm_s > 0, "fresh forward data should permit motion");

    /* The wheel source stays fresh, and the sonar task emits no fault. Its
     * last sequence is republished unchanged while the car approaches. */
    for (uint32_t elapsed = 10U; elapsed <= 4000U; elapsed += 10U)
    {
        meas.now_ms = 1000U + elapsed;
        meas.stamp_ms = meas.now_ms;
        meas.left_um += 1250;
        meas.right_um += 1250;
        pilot_step(&state, &meas, &command);
        b_stopped = b_stopped || ((PILOT_REASON_SONAR == command.reason ||
                                   PILOT_REASON_SONAR_LATCHED == command.reason) &&
                                  0 == command.speed_mm_s && 0 == command.turn_deci_s);
    }
    if (b_check_latch)
    {
        meas.now_ms += 10U;
        meas.stamp_ms = meas.now_ms;
        meas.ping_seq++;
        pilot_step(&state, &meas, &command);
        check((PILOT_REASON_SONAR_LATCHED == command.reason) &&
              (0 == command.speed_mm_s) && (0 == command.turn_deci_s),
              "a new forward ping did not leave a long dropout latched");
    }
    else
    {
        check(b_stopped, "the car drove with a silent forward sonar for four seconds");
    }
}

static void test_do_not_release_on_approach(void)
{
    bool b_hug_started = false;
    double const front_x = OBS_FACE_X_MM;
    double const back_x = front_x + 200.0;

    pilot_sim_init(&g_sim, g_crs_straight, 1U);
    check(pilot_sim_add_box(&g_sim, front_x + 100.0, 0.0, 200.0, 200.0, 0.0),
          "install the obstacle across the line");
    while (g_sim.t_s < OBS_RUN_S)
    {
        pilot_sim_step(&g_sim);
        if (b_hug_started && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour))
        {
            check(g_sim.x > back_x, "the hug released to the approach line");
            (void)printf("Hug released past the box at %.0f mm\n", g_sim.x);
            return;
        }
        b_hug_started = b_hug_started || (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour);
    }
    check(false, "the hug did not release to the line after the box");
}

static int run_isolated_case(char const * p_name)
{
    if (0 == strcmp(p_name, "slip-reverse"))
    {
        test_continuous_slip(false);
    }
    else if (0 == strcmp(p_name, "slip-restart"))
    {
        test_continuous_slip(true);
    }
    else if (0 == strcmp(p_name, "initial-approach"))
    {
        test_unseen_face_approach(false);
    }
    else if (0 == strcmp(p_name, "crawl-reapproach"))
    {
        test_unseen_face_approach(true);
    }
    else if (0 == strcmp(p_name, "stale-forward-ping"))
    {
        test_stale_forward_ping(false);
    }
    else if (0 == strcmp(p_name, "stale-forward-ping-latches"))
    {
        test_stale_forward_ping(true);
    }
    else if (0 == strcmp(p_name, "hug-timeout"))
    {
        test_hug_timeout();
    }
    else if (0 == strcmp(p_name, "hug-silent-stop"))
    {
        test_sonar_silent(false, 1U);
    }
    else if (0 == strcmp(p_name, "near-sonar-contact"))
    {
        near_run(1U, false);
    }
    else if (0 == strcmp(p_name, "near-blind-contact"))
    {
        near_run(1U, true);
    }
    else if (0 == strcmp(p_name, "recoverable-slip"))
    {
        touch_run("slip then stall", TOUCH_SLIP_STALL, 45.0, 1U, false, true);
        return (0U == g_soft_fails) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    else if (0 == strcmp(p_name, "box-60"))
    {
        obs_run(&g_obs[0], 1U, &g_obs_plain);
        return (0U == g_soft_fails && 0U == g_budget_overruns) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    else if (0 == strcmp(p_name, "box-60-off-centre"))
    {
        obs_run(&g_obs[1], 12345U, &g_obs_plain);
        return (0U == g_soft_fails && 0U == g_budget_overruns) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    else if (0 == strcmp(p_name, "box-200"))
    {
        obs_run(&g_obs[4], 1U, &g_obs_plain);
        return (0U == g_soft_fails && 0U == g_budget_overruns) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    else if (0 == strcmp(p_name, "box-300-off-centre"))
    {
        obs_run(&g_obs[7], 1U, &g_obs_plain);
        return (0U == g_soft_fails && 0U == g_budget_overruns) ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    else if (0 == strcmp(p_name, "blocked-wheel"))
    {
        test_blocked(1U);
    }
    else if (0 == strcmp(p_name, "approach-line-release"))
    {
        test_do_not_release_on_approach();
    }
    else if (0 == strcmp(p_name, "nearer-edge-within-reach"))
    {
        obs_run(&g_obs[1], 1U, &g_obs_plain);
    }
    else if (0 == strcmp(p_name, "nearer-edge-out-of-reach"))
    {
        obs_run(&g_obs[7], 1U, &g_obs_plain);
    }
    else if (0 == strcmp(p_name, "release-note-expiry"))
    {
        test_note();
    }
    else
    {
        (void)fprintf(stderr, "Unknown pilot sim case: %s\n", p_name);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static void test_obstacles(void)
{
    unsigned o, s;

    test_hug_trigger();
    test_sonar_fault();
    for (s = 0U; s < OBS_SEEDS; s++)
    {
        test_sonar_silent(false, g_obs_seeds[s]);
        test_sonar_silent(true, g_obs_seeds[s]);
    }
    test_breadth_kept();
    test_aimed_settle();
    test_note();
    test_move_timeout();
    test_hug_timeout();
    for (o = 0U; o < OBS_N; o++)
    {
        for (s = 0U; s < OBS_SEEDS; s++)
        {
            obs_run(&g_obs[o], g_obs_seeds[s], &g_obs_plain);
        }
    }
    /* The wire state is made from the private codes of hug.c: every state a
     * hug passes through must show up, or the mapping has drifted. */
    (void)printf("hug wire states seen 0x%03lx\n", (unsigned long)g_hug_states_seen);
    check(0UL != (g_hug_states_seen & (1UL << TELEMETRY_HUG_STATE_IDLE)), "wire state idle seen");
    check(0UL != (g_hug_states_seen & (1UL << TELEMETRY_HUG_STATE_CONTACT)), "wire state contact seen");
    check(0UL != (g_hug_states_seen & (1UL << TELEMETRY_HUG_STATE_ALIGN)), "wire state align seen");
    check(0UL != (g_hug_states_seen & (1UL << TELEMETRY_HUG_STATE_LEG)), "wire state leg seen");
    check(0UL != (g_hug_states_seen & (1UL << TELEMETRY_HUG_STATE_LOOK)), "wire state look seen");
    check(0UL != (g_hug_states_seen & (1UL << TELEMETRY_HUG_STATE_EXIT)), "wire state exit seen");
    test_courses();
    for (s = 0U; s < TOUCH_N; s++)
    {
        near_run(touch_seed(s), false);
        near_run(touch_seed(s), true);
    }
    test_robust();
    test_touch_slip();
    test_touch();
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
    test_sonar();
    static pilot_sim_seg_t const straight[] = {
        {8000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    static pilot_sim_seg_t const corner[] = {
        {800.0, 0.0, 0.0}, {0.0, 150.0, 90.0}, {8000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    static pilot_sim_seg_t const u_two[] = {
        {800.0, 0.0, 0.0}, {0.0, 150.0, 90.0}, {100.0, 0.0, 0.0}, {0.0, 150.0, 90.0},
        {300.0, 0.0, 0.0}, {0.0, 150.0, -90.0}, {8000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    static pilot_sim_seg_t const s_bend[] = {
        {800.0, 0.0, 0.0}, {0.0, 150.0, 90.0}, {0.0, 150.0, -90.0}, {8000.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};
    pilot_sim_seg_t const * const courses[] = {straight, corner, u_two, s_bend};
    char const * const names[] = {"straight", "R150 corner", "U of two", "S bend"};
    unsigned c, s;

    test_direct();
    test_slow_wheel_seam();
    test_retry_label_seam();
    test_held();
    for (c = 0U; c < 4U; c++)
    {
        for (s = 0U; s < SEEDS; s++)
        {
            run_course(names[c], courses[c], g_seeds[s], false);
        }
    }
    for (s = 0U; s < SEEDS; s++)
    {
        run_course("straight", straight, g_seeds[s], true);
    }
    for (s = 0U; s < SEEDS; s++)
    {
        test_dead_encoder(g_seeds[s], s & 1U);
        test_blocked(g_seeds[s]);
        test_tight_turn(g_seeds[s]);
    }
    test_obstacles();
    if (0U != g_budget_overruns)
    {
        (void)printf("FAIL: %u hugs took longer than the budget\n", g_budget_overruns);
        return EXIT_FAILURE;
    }
    (void)printf("pilot sim ok\n");
    return 0;
}
