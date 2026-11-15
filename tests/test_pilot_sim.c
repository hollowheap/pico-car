/* Pilot tests: direct cases, then closed-loop properties on the simulated car
 * over several courses and fixed seeds. */

#include <stdio.h>
#include <stdlib.h>

#include "pilot/pilot.h"
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

static uint32_t const g_seeds[SEEDS] = {1U, 2U, 3U, 17U, 42U, 12345U};

static void fail(char const * p_what, uint32_t seed)
{
    (void)printf("FAIL: %s\n", p_what);
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
    pilot_meas_t m = {0, 0, now_ms, now_ms, true, false};

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
    pilot_reset(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_REASON_NONE == cmd.reason, "reset ends estop");
    (void)printf("direct cases ok\n");
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

int main(void)
{
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
    (void)printf("pilot sim ok\n");
    return 0;
}
