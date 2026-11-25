/* Line behaviour of the pilot path: a barcode is driven straight through, and
 * the line-lost bookkeeping counts a lost line only while the follower has the
 * wheels. */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "pilot/pilot.h"
#include "platform/chassis.h"
#include "platform/units.h"
#include "pilot_sim.h"

#define SEEDS             (4U)
#define LINE_END_MM       (4000.0)
#define WIDE_END_MM       (2000.0)
#define WIDE_ROW_MM       (8.0)
#define PT_MM             (2.0)
#define BARCODE_HOLD_MS   (250.0)
#define STRIPES           (6)
#define STRIPE_PITCH_MM   (30.0)
#define STRIPE_WIDTH_MM   (10.0)
#define STRIPE_HALF_MM    (40.0)
#define BARCODE_X_MM      (800.0)
#define HEADING_BOUND_DEG (6.0)
#define RAD_DEG           (57.29577951308232)
#define GAP_X_MM          (1000.0)
#define GAP_LEN_MM        (400.0)
#define PATCH_AHEAD_MM    (60.0)
#define PATCH_HALF_MM     (30.0)
#define BOX_X_MM          (900.0)
#define BOX_W_MM          (250.0)
#define BOX_D_MM          (150.0)
#define RUN_MAX_S         (12.0)
#define RETRY_RUN_MAX_S   (30.0)
#define TICK_MS           (10U)
#define MS_PER_S_I        (1000)
#define BLIND_RUN_TICKS   (6000U)
#define BARCODE_TICKS     (25U)
#define REVERSE_TOL_MM    (10)

static pilot_sim_t g_sim;

static uint32_t const g_seeds[SEEDS] = {1U, 2U, 3U, 17U};

static void check(bool b_ok, char const * p_what, uint32_t seed)
{
    if (!b_ok)
    {
        (void)printf("FAIL: %s (seed %u)\n", p_what, (unsigned)seed);
        exit(EXIT_FAILURE);
    }
}

static void add_pt(double x, double y)
{
    if (g_sim.line_n < PILOT_SIM_MAX_PTS)
    {
        g_sim.line_x[g_sim.line_n] = x;
        g_sim.line_y[g_sim.line_n] = y;
        g_sim.line_n++;
    }
}

/* Black patch over x0..x1, -half..half across the line. */
static void add_patch(double x0, double x1, double half)
{
    double x;
    double y;

    for (x = x0; x <= x1; x += PT_MM)
    {
        for (y = -half; y <= half; y += PT_MM)
        {
            add_pt(x, y);
        }
    }
    pilot_sim_reindex(&g_sim);
}

/* Remove the line points in x0..x1. */
static void cut_line(double x0, double x1)
{
    int i;
    int n = 0;

    for (i = 0; i < g_sim.line_n; i++)
    {
        if ((g_sim.line_x[i] < x0) || (g_sim.line_x[i] > x1))
        {
            g_sim.line_x[n] = g_sim.line_x[i];
            g_sim.line_y[n] = g_sim.line_y[i];
            n++;
        }
    }
    g_sim.line_n = n;
    pilot_sim_reindex(&g_sim);
}

static double sensor_x(void)
{
    return g_sim.x + (LINE_SENSOR_LEAD_MM * cos(g_sim.psi));
}

static line_follow_status_t status_now(void)
{
    return (line_follow_status_t)g_sim.pilot.follow.status;
}

static line_follow_status_t status_of(pilot_state_t const * p_st)
{
    return (line_follow_status_t)p_st->follow.status;
}

static void init_straight(uint32_t seed)
{
    static pilot_sim_seg_t const straight[] = {{LINE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    pilot_sim_init(&g_sim, straight, seed);
}

/* A tape wide enough that a car a few mm off the middle always has a sensor on
 * it, so that the only line loss in a run is the one the test makes. */
static void init_wide(uint32_t seed)
{
    static pilot_sim_seg_t const straight[] = {{WIDE_END_MM, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    int n;
    int i;

    pilot_sim_init(&g_sim, straight, seed);
    n = g_sim.line_n;
    for (i = 0; i < n; i++)
    {
        add_pt(g_sim.line_x[i], WIDE_ROW_MM);
        add_pt(g_sim.line_x[i], -WIDE_ROW_MM);
        add_pt(g_sim.line_x[i], 2.0 * WIDE_ROW_MM);
        add_pt(g_sim.line_x[i], -2.0 * WIDE_ROW_MM);
    }
    pilot_sim_reindex(&g_sim);
}

/* (a) Stripes across the line under both sensors, with b_barcode set as the
 * barcode sensor sets it: the car keeps its heading and the search state
 * stays where it was. */
static void test_barcode(uint32_t seed)
{
    double const end_x = BARCODE_X_MM + ((STRIPES - 1) * STRIPE_PITCH_MM) + STRIPE_WIDTH_MM;
    double last_edge_s = -1.0;
    double psi0 = 0.0;
    bool b_in = false;
    bool b_past = false;
    line_follow_status_t status0 = LINE_FOLLOW_FOLLOWING;
    double max_dev_deg = 0.0;
    int i;

    init_straight(seed);
    for (i = 0; i < STRIPES; i++)
    {
        double const x0 = BARCODE_X_MM + (i * STRIPE_PITCH_MM);

        add_patch(x0, x0 + STRIPE_WIDTH_MM, STRIPE_HALF_MM);
    }
    while ((g_sim.t_s < RUN_MAX_S) &&
           (!b_past || ((g_sim.t_s - last_edge_s) <= (BARCODE_HOLD_MS / 1000.0))))
    {
        double const sx = sensor_x();

        b_past = b_past || (sx > (end_x + PT_MM));
        if (!b_past && (sx >= (BARCODE_X_MM - PT_MM)))
        {
            last_edge_s = g_sim.t_s;
        }
        g_sim.meas.b_barcode = (last_edge_s >= 0.0) &&
                               ((g_sim.t_s - last_edge_s) <= (BARCODE_HOLD_MS / 1000.0));
        if (g_sim.meas.b_barcode && !b_in)
        {
            b_in = true;
            psi0 = g_sim.psi;
            status0 = status_now();
        }
        pilot_sim_step(&g_sim);
        if (b_in && g_sim.meas.b_barcode)
        {
            double const dev = fabs((g_sim.psi - psi0) * RAD_DEG);

            max_dev_deg = (dev > max_dev_deg) ? dev : max_dev_deg;
            check(status_now() == status0, "search state changed in a barcode", seed);
        }
    }
    (void)printf("barcode seed %5u: heading change %.2f deg\n", (unsigned)seed, max_dev_deg);
    check(b_in, "car reached the barcode", seed);
    check(max_dev_deg < HEADING_BOUND_DEG, "barcode turned the car", seed);
}

static unsigned g_lost_in;
static unsigned g_lost_out;
static unsigned g_quiet;
static line_watch_t g_watch;

static void watch_tick(void)
{
    bool const b_was = (0U != g_watch.b_lost);
    uint8_t const bits = line_watch_step(
        &g_watch, PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour,
        PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour, status_now());

    g_lost_in += ((0U != g_watch.b_lost) && !b_was) ? 1U : 0U;
    g_lost_out += ((0U == g_watch.b_lost) && b_was && (0U == (bits & LINE_WATCH_QUIET))) ? 1U : 0U;
    g_quiet += (0U != (bits & LINE_WATCH_QUIET)) ? 1U : 0U;
}

/* (b) A gap longer than the gap distance: one entry into lost, and one return
 * when the tape comes back under the sensors. */
static void test_gap(uint32_t seed)
{
    bool b_patched = false;

    g_watch = (line_watch_t){0U, 0U};
    g_lost_in = 0U;
    g_lost_out = 0U;
    g_quiet = 0U;
    init_wide(seed);
    cut_line(GAP_X_MM, GAP_X_MM + GAP_LEN_MM);
    while (g_sim.t_s < RUN_MAX_S)
    {
        pilot_sim_step(&g_sim);
        watch_tick();
        if (!b_patched && (LINE_FOLLOW_SEARCHING == status_now()))
        {
            b_patched = true;
            add_patch(sensor_x() - PT_MM, sensor_x() + PATCH_AHEAD_MM, PATCH_HALF_MM);
        }
        if (b_patched && (g_sim.t_s > 0.0) && (LINE_FOLLOW_SEARCHING != status_now()) &&
            (0U == g_watch.b_lost))
        {
            break;
        }
    }
    (void)printf("gap seed %5u: lost %u found %u\n", (unsigned)seed, g_lost_in, g_lost_out);
    check(b_patched, "the gap did not give a search", seed);
    check(1U == g_lost_in, "not exactly one line-lost entry", seed);
    check(1U == g_lost_out, "not exactly one line-found", seed);
}

/* (b2) A gap longer than the gap distance, and no tape in reach of the first
 * search. The tape comes back under the sensors only when the first search has
 * failed and the car has begun its retry: one line-lost entry, one line-found,
 * and the car never stops. */
static void test_gap_retry(uint32_t seed)
{
    bool b_patched = false;
    bool b_found = false;

    g_watch = (line_watch_t){0U, 0U};
    g_lost_in = 0U;
    g_lost_out = 0U;
    g_quiet = 0U;
    init_wide(seed);
    cut_line(GAP_X_MM, GAP_X_MM + GAP_LEN_MM);
    while ((g_sim.t_s < RETRY_RUN_MAX_S) && !b_found)
    {
        pilot_sim_step(&g_sim);
        watch_tick();
        check(LINE_FOLLOW_STOPPED != status_now(), "the car stopped before the retry found the tape", seed);
        if (!b_patched && (g_sim.pilot.follow.retry > 0U))
        {
            b_patched = true;
            add_patch(sensor_x() - PT_MM, sensor_x() + PATCH_AHEAD_MM, PATCH_HALF_MM);
        }
        b_found = b_patched && (LINE_FOLLOW_SEARCHING != status_now()) && (0U == g_watch.b_lost);
    }
    (void)printf("gap retry seed %5u: lost %u found %u\n", (unsigned)seed, g_lost_in, g_lost_out);
    check(b_patched, "the first search did not fail into a retry", seed);
    check(b_found, "the retry did not find the tape", seed);
    check(1U == g_lost_in, "not exactly one line-lost entry", seed);
    check(1U == g_lost_out, "not exactly one line-found", seed);
}

/* (c) A box on the line: the hug has the wheels and the line is off the
 * sensors; that is not a lost line. */
static void test_hug(uint32_t seed)
{
    bool b_hug = false;
    bool b_back = false;

    g_watch = (line_watch_t){0U, 0U};
    g_lost_in = 0U;
    g_lost_out = 0U;
    g_quiet = 0U;
    init_wide(seed);
    (void)pilot_sim_add_box(&g_sim, BOX_X_MM, 0.0, BOX_W_MM, BOX_D_MM, 0.0);
    while ((g_sim.t_s < RUN_MAX_S) && !b_back)
    {
        pilot_sim_step(&g_sim);
        watch_tick();
        b_hug = b_hug || (PILOT_BEHAVIOUR_HUG == g_sim.cmd.behaviour);
        b_back = b_hug && (PILOT_BEHAVIOUR_FOLLOW_LINE == g_sim.cmd.behaviour) &&
                 (g_sim.x > (BOX_X_MM + BOX_D_MM));
    }
    (void)printf("hug seed %5u: lost %u found %u\n", (unsigned)seed, g_lost_in, g_lost_out);
    check(b_hug, "the car did not hug the box", seed);
    check(0U == g_lost_in, "a hug counted as a lost line", seed);
    check(0U == g_lost_out, "a hug raised a line-found", seed);
}

/* A hug that ends while the follower was lost: the follower starts afresh, and
 * that is no line-found. A stop is reported once. */
static void test_watch_direct(void)
{
    line_watch_t w = {0U, 0U};

    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_FOLLOWING), "idle", 0U);
    check(0U == w.b_lost, "following is not lost", 0U);
    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_SEARCHING), "search", 0U);
    check(0U != w.b_lost, "searching is lost", 0U);
    check(0U == line_watch_step(&w, false, false, LINE_FOLLOW_FOLLOWING), "safety tick", 0U);
    check(0U != w.b_lost, "a safety stop does not end a lost line", 0U);
    check(0U != (LINE_WATCH_QUIET & line_watch_step(&w, false, true, LINE_FOLLOW_SEARCHING)),
          "hug ends the lost state quietly", 0U);
    check(0U == w.b_lost, "hug clears the lost state", 0U);
    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_FOLLOWING), "after the hug", 0U);
    check(0U == w.b_lost, "no lost line after the hug", 0U);
    check(0U == line_watch_step(&w, true, false, LINE_FOLLOW_SEARCHING), "search again", 0U);
    check(0U != (LINE_WATCH_STOP & line_watch_step(&w, true, false, LINE_FOLLOW_STOPPED)),
          "first stop is reported", 0U);
    check(0U == (LINE_WATCH_STOP & line_watch_step(&w, true, false, LINE_FOLLOW_STOPPED)),
          "stop is reported once", 0U);
}

/* No line under the sensors, a barcode edge seen: straight at cruise, and the
 * search state stays. A stopped follower stays stopped. */
static void test_barcode_direct(void)
{
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = {0};

    m.now_ms = 1000U;
    m.stamp_ms = 1000U;
    m.b_barcode = true;
    pilot_init(&st);
    pilot_step(&st, &m, &cmd);
    check(PILOT_BEHAVIOUR_FOLLOW_LINE == cmd.behaviour, "barcode: follower drives", 0U);
    check(cmd.speed_mm_s == (LINE_CRUISE_UM_S / 1000), "barcode: cruise speed", 0U);
    check(0 == cmd.turn_deci_s, "barcode: straight", 0U);
    check(0U == (st.follow.flags & LINE_FOLLOW_STARTED), "barcode: follower not updated", 0U);

    st.follow.status = (uint8_t)LINE_FOLLOW_STOPPED;
    pilot_step(&st, &m, &cmd);
    check((0 == cmd.speed_mm_s) && (PILOT_REASON_LINE_LOST_STOP == cmd.reason),
          "barcode: a stopped follower stays stopped", 0U);
}

/* The straight through a barcode does not count as blind travel: after it the
 * follower has the whole LINE_GAP_MM before it searches. */
static void test_barcode_straight_keeps_the_gap(void)
{
    enum { STEP_MS = 10, STRAIGHT_STEPS = 25, MAX_STEPS = 400 };
    pilot_state_t st;
    pilot_cmd_t cmd;
    pilot_meas_t m = {0};
    int32_t travel_mm = 0;
    int32_t resume_mm;
    int i;

    pilot_init(&st);
    m.now_ms = 1000U;
    m.stamp_ms = 1000U;
    m.b_line_left = true;
    m.b_line_right = true;
    pilot_step(&st, &m, &cmd);
    m.b_line_left = false;
    m.b_line_right = false;
    m.b_barcode = true;
    for (i = 0; i < STRAIGHT_STEPS; i++)
    {
        m.now_ms += STEP_MS;
        m.stamp_ms = m.now_ms;
        travel_mm++;
        m.left_um = (int64_t)travel_mm * 1000;
        m.right_um = m.left_um;
        pilot_step(&st, &m, &cmd);
    }
    m.b_barcode = false;
    resume_mm = travel_mm;
    for (i = 0; (i < MAX_STEPS) && ((uint8_t)LINE_FOLLOW_SEARCHING != st.follow.status); i++)
    {
        m.now_ms += STEP_MS;
        m.stamp_ms = m.now_ms;
        travel_mm++;
        m.left_um = (int64_t)travel_mm * 1000;
        m.right_um = m.left_um;
        pilot_step(&st, &m, &cmd);
    }
    check((uint8_t)LINE_FOLLOW_SEARCHING == st.follow.status, "barcode gap: searches", 0U);
    check((travel_mm - resume_mm) >= (LINE_GAP_MM - 2), "barcode gap: full bridge after the straight", 0U);
}

/* The pilot on an ideal car: each command moves the wheels exactly, and
 * b_block holds the car still while it is told to reverse. */
typedef struct
{
    pilot_state_t st;
    pilot_meas_t  m;
    pilot_cmd_t   cmd;
    int64_t       travel_um;
    bool          b_block;
    bool          b_jam_left;
    int64_t       left_um;
    int64_t       right_um;
} ideal_t;

static void ideal_init(ideal_t * p_i)
{
    *p_i = (ideal_t){0};
    pilot_init(&p_i->st);
    p_i->m.now_ms = 1000U;
    p_i->m.stamp_ms = 1000U;
    p_i->m.b_line_left = true;
    p_i->m.b_line_right = true;
    pilot_step(&p_i->st, &p_i->m, &p_i->cmd);
    p_i->m.b_line_left = false;
    p_i->m.b_line_right = false;
}

static bool ideal_reversing(ideal_t const * p_i)
{
    return (PILOT_BEHAVIOUR_FOLLOW_LINE == p_i->cmd.behaviour) &&
           (p_i->cmd.speed_mm_s < 0);
}

static void ideal_step(ideal_t * p_i)
{
    p_i->m.now_ms += TICK_MS;
    p_i->m.stamp_ms = p_i->m.now_ms;
    pilot_step(&p_i->st, &p_i->m, &p_i->cmd);
    if (!(p_i->b_block && ideal_reversing(p_i)))
    {
        int64_t const dist_um = ((int64_t)p_i->cmd.speed_mm_s * UM_PER_MM *
                                 TICK_MS) / MS_PER_S_I;
        int64_t const turn_um = ((int64_t)p_i->cmd.turn_deci_s *
                                 CHASSIS_WHEEL_TRACK_MM * UM_PER_MM * TICK_MS) /
                                ((int64_t)DECI_PER_RAD * MS_PER_S_I);

        p_i->left_um += p_i->b_jam_left ? 0 : (dist_um - (turn_um / 2));
        p_i->right_um += dist_um + (turn_um / 2);
    }
    p_i->travel_um = (p_i->left_um + p_i->right_um) / 2;
    p_i->m.left_um = p_i->left_um;
    p_i->m.right_um = p_i->right_um;
}

/* The car is against something behind it: every reverse of the search stands
 * still. The pilot must not back off into it, must not hand it to the hug,
 * and the follower must give up after its retries. */
static void test_blocked_reverse_stops_after_the_retries(void)
{
    ideal_t i;
    unsigned tick;
    unsigned reversing = 0U;

    ideal_init(&i);
    i.b_block = true;
    for (tick = 0U;
         (tick < BLIND_RUN_TICKS) && (LINE_FOLLOW_STOPPED != status_of(&i.st));
         tick++)
    {
        ideal_step(&i);
        reversing += ideal_reversing(&i) ? 1U : 0U;
        check(PILOT_REASON_STUCK != i.cmd.reason,
              "blocked reverse: a back-off", 0U);
        check(PILOT_BEHAVIOUR_HUG != i.cmd.behaviour,
              "blocked reverse: the hug has the wheels", 0U);
        check(0U == i.st.b_encoder, "blocked reverse: an encoder stop", 0U);
    }
    check(0U != reversing, "blocked reverse: the car never reversed", 0U);
    check(LINE_FOLLOW_STOPPED == status_of(&i.st),
          "blocked reverse: no stop after the retries", 0U);
    check(LINE_RETRY_COUNT == i.st.follow.retry,
          "blocked reverse: not all retries used", 0U);
    ideal_step(&i);
    check((0 == i.cmd.speed_mm_s) &&
              (PILOT_REASON_LINE_LOST_STOP == i.cmd.reason),
          "blocked reverse: the stop is not the line-lost stop", 0U);
}

/* The left wheel jams as the first reverse of the search begins. The pilot does
 * not back off in the reverse, the follower ends it as a failed retry, and the
 * spin that follows shows one wheel still: ENCODER. */
static void test_one_wheel_jam_after_a_reverse_latches_encoder(void)
{
    ideal_t i;
    unsigned tick;
    unsigned reverse_ticks = 0U;
    bool b_jam = false;

    ideal_init(&i);
    for (tick = 0U; (tick < BLIND_RUN_TICKS) && (0U == i.st.b_encoder); tick++)
    {
        ideal_step(&i);
        if (ideal_reversing(&i))
        {
            b_jam = true;
            i.b_jam_left = true;
            reverse_ticks++;
            check(PILOT_REASON_STUCK != i.cmd.reason,
                  "one wheel jam: a back-off in the reverse", 0U);
        }
    }
    check(b_jam, "one wheel jam: the car never reversed", 0U);
    check(0U != i.st.b_encoder, "one wheel jam: no encoder stop", 0U);
    check(reverse_ticks <= ((LINE_REVERSE_STALL_MS / TICK_MS) + 2U),
          "one wheel jam: the reverse did not end in time", 0U);
}

/* A barcode edge while the car reverses in a search: the reverse goes on, and
 * it covers its distance. */
static void test_barcode_during_the_reverse(void)
{
    ideal_t i;
    unsigned tick;
    unsigned barcode_left = 0U;
    int64_t from_um = 0;
    int64_t back_um = 0;
    bool b_seen = false;
    bool b_done = false;

    ideal_init(&i);
    for (tick = 0U; (tick < BLIND_RUN_TICKS) && !b_done; tick++)
    {
        bool const b_was_reversing = ideal_reversing(&i);

        if (b_was_reversing && !b_seen)
        {
            b_seen = true;
            from_um = i.travel_um;
            barcode_left = BARCODE_TICKS;
        }
        i.m.b_barcode = (0U != barcode_left);
        barcode_left -= (0U != barcode_left) ? 1U : 0U;
        ideal_step(&i);
        if (b_seen && !b_done)
        {
            back_um = from_um - i.travel_um;
            if (i.m.b_barcode)
            {
                check(i.cmd.speed_mm_s < 0,
                      "barcode in the reverse: the car drove forward", 0U);
            }
            b_done = !ideal_reversing(&i) && (back_um > 0);
        }
    }
    check(b_seen, "barcode in the reverse: no reverse", 0U);
    check(b_done, "barcode in the reverse: the reverse did not end", 0U);
    check(back_um >= ((int64_t)(LINE_REVERSE_MM - REVERSE_TOL_MM) * UM_PER_MM),
          "barcode in the reverse: it did not cover its distance", 0U);
}

int main(void)
{
    unsigned i;

    test_watch_direct();
    test_barcode_direct();
    test_barcode_straight_keeps_the_gap();
    test_blocked_reverse_stops_after_the_retries();
    test_one_wheel_jam_after_a_reverse_latches_encoder();
    test_barcode_during_the_reverse();
    for (i = 0U; i < SEEDS; i++)
    {
        test_barcode(g_seeds[i]);
        test_gap(g_seeds[i]);
        test_gap_retry(g_seeds[i]);
        test_hug(g_seeds[i]);
    }
    (void)printf("pilot_line: ok\n");
    return EXIT_SUCCESS;
}
