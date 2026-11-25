/** @file test_line_control.c
 * @brief Host tests for the line follower and one-shot turns (D92, D93).
 *
 * Includes a small differential-drive simulator (107 mm track, a straight
 * line, two sensors 2 cm apart ahead of the axle) for the 30 degree entry.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "track/line_control.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define STEP_MS         10U
#define UM_PER_MM       1000
#define CRUISE          LINE_CRUISE_UM_S
#define SEARCH_UM       ((int64_t)LINE_SEARCH_MAX_MM * UM_PER_MM)
#define TRACK_UM        ((int64_t)LINE_WHEEL_TRACK_MM * UM_PER_MM)
#define DECI_PER_RAD    573
#define PI_RAD          3.14159265358979323846
#define LCG_MUL         1103515245U
#define LCG_ADD         12345U
#define LCG_SHIFT       16U
#define RANDOM_STEPS    5000

/* ---- helpers ---------------------------------------------------------- */

static line_follow_status_t follow(line_follow_t * p_f, bool b_left,
                                   bool b_right, int64_t travel_um,
                                   uint32_t now_ms, line_targets_t * p_t)
{
    return line_follow_update(p_f, b_left, b_right, travel_um, now_ms, p_t);
}

static void expect_targets(line_targets_t const * p_t, int32_t left,
                           int32_t right)
{
    assert(left == p_t->left_um_s);
    assert(right == p_t->right_um_s);
}

/* Wheel displacement of each wheel (magnitude) for a pivot of deci-degrees. */
static int64_t pivot_half_um(int32_t deci)
{
    return ((int64_t)deci * (TRACK_UM / 2)) / DECI_PER_RAD + 1;
}

/* Left turn: left wheel back, right wheel forward. */
static void left_turn_disp(int32_t deci, int64_t * p_l, int64_t * p_r)
{
    *p_l = 1000 - pivot_half_um(deci);
    *p_r = 2000 + pivot_half_um(deci);
}

static void right_turn_disp(int32_t deci, int64_t * p_l, int64_t * p_r)
{
    *p_l = 1000 + pivot_half_um(deci);
    *p_r = 2000 - pivot_half_um(deci);
}

/* ---- follower --------------------------------------------------------- */

static void test_init(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, false, false, 0, 0U, &t));
    expect_targets(&t, CRUISE, CRUISE);
}

static void test_one_sensor_correction(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, false, 0, 0U, &t));
    expect_targets(&t, 0, CRUISE);

    line_follow_init(&f);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, false, true, 0, 0U, &t));
    expect_targets(&t, CRUISE, 0);
}

static void test_both_and_neither_straight(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, true, 0, 0U, &t));
    expect_targets(&t, CRUISE, CRUISE);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, false, false, 0, 10U, &t));
    expect_targets(&t, CRUISE, CRUISE);
}

static void test_both_keeps_recorded_side(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0U, &t);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, true, 0, 10U, &t));
    expect_targets(&t, CRUISE, CRUISE);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 20U, &t));
    expect_targets(&t, 0, CRUISE);
}

/* 30 degree entry as a sensor sequence: the right sensor touches the line,
 * the car rolls off the far edge, and neither sensor sees it. */
static void test_entry_sequence_keeps_turning(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t i;

    line_follow_init(&f);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, false, true, 0, 0U, &t));
    expect_targets(&t, CRUISE, 0);
    for (i = 1U; i <= 20U; i++)
    {
        assert(LINE_FOLLOW_SEARCHING ==
               follow(&f, false, false, (int64_t)i * 1000, i * STEP_MS, &t));
        expect_targets(&t, CRUISE, 0);
    }
    assert(LINE_FOLLOW_FOLLOWING ==
           follow(&f, false, true, 21000, 210U, &t));
    expect_targets(&t, CRUISE, 0);
}

/* Hairpin: left side sees the line, a long gap, then the right side. */
static void test_hairpin(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t i;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0U, &t);
    for (i = 1U; i <= 50U; i++)
    {
        assert(LINE_FOLLOW_SEARCHING ==
               follow(&f, false, false, (int64_t)i * 500, i * STEP_MS, &t));
        expect_targets(&t, 0, CRUISE);
    }
    assert(LINE_FOLLOW_FOLLOWING ==
           follow(&f, false, true, 25000, 510U, &t));
    expect_targets(&t, CRUISE, 0);
    assert(LINE_FOLLOW_SEARCHING ==
           follow(&f, false, false, 26000, 520U, &t));
    expect_targets(&t, CRUISE, 0);
}

static void test_search_limit_distance(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t base = 5000;
    uint32_t i;

    line_follow_init(&f);
    (void)follow(&f, false, true, base, 0U, &t);
    /* First step without a line starts the search at this travel. */
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, base, 10U, &t));
    for (i = 1U; i < (uint32_t)LINE_SEARCH_MAX_MM; i++)
    {
        int64_t const travel = base + ((int64_t)i * UM_PER_MM);
        assert(LINE_FOLLOW_SEARCHING ==
               follow(&f, false, false, travel, 10U + (i * STEP_MS), &t));
        expect_targets(&t, CRUISE, 0);
    }
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, base + SEARCH_UM,
                  10U + ((uint32_t)LINE_SEARCH_MAX_MM * STEP_MS), &t));
    expect_targets(&t, 0, 0);
    /* Sticky while no sensor sees the line. */
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, base, 20U, &t));
    expect_targets(&t, 0, 0);
    /* Travel backwards counts too. */
    line_follow_init(&f);
    (void)follow(&f, true, false, base, 0U, &t);
    (void)follow(&f, false, false, base, 10U, &t);
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, base - SEARCH_UM, 20U, &t));
}

static void test_search_limit_time(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t start = 1000U;
    uint32_t el;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, start - STEP_MS, &t);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, start, &t));
    for (el = STEP_MS; el < LINE_SEARCH_MAX_MS; el += STEP_MS)
    {
        assert(LINE_FOLLOW_SEARCHING ==
               follow(&f, false, false, 0, start + el, &t));
    }
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, 0, start + LINE_SEARCH_MAX_MS, &t));
    expect_targets(&t, 0, 0);
}

static void test_search_limit_time_wraps(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t const start = 0xFFFFFF00U;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, start - STEP_MS, &t);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, start, &t));
    assert(LINE_FOLLOW_SEARCHING ==
           follow(&f, false, false, 0, start + 1000U, &t));
    assert(LINE_FOLLOW_SEARCHING ==
           follow(&f, false, false, 0, start + LINE_SEARCH_MAX_MS - 1U, &t));
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, 0, start + LINE_SEARCH_MAX_MS, &t));
}

static void test_recovery_after_lost(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0U, &t);
    (void)follow(&f, false, false, 0, 10U, &t);
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, 0, 10U + LINE_SEARCH_MAX_MS, &t));

    assert(LINE_FOLLOW_FOLLOWING ==
           follow(&f, false, true, 0, 2000U, &t));
    expect_targets(&t, CRUISE, 0);

    /* A new search gets a fresh budget toward the new side. */
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 2010U, &t));
    expect_targets(&t, CRUISE, 0);
    assert(LINE_FOLLOW_SEARCHING ==
           follow(&f, false, false, 0, 2010U + LINE_SEARCH_MAX_MS - 10U, &t));

    /* Both sensors also end LOST. */
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, 0, 2010U + LINE_SEARCH_MAX_MS, &t));
    assert(LINE_FOLLOW_FOLLOWING ==
           follow(&f, true, true, 0, 4000U, &t));
    expect_targets(&t, CRUISE, CRUISE);
}

static void test_targets_never_exceed_cruise(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t rnd = 1U;
    uint32_t now = 0U;
    int64_t travel = 0;
    int i;

    line_follow_init(&f);
    for (i = 0; i < RANDOM_STEPS; i++)
    {
        rnd = (rnd * LCG_MUL) + LCG_ADD;
        {
            uint32_t const bits = (rnd >> LCG_SHIFT) & 3U;
            (void)follow(&f, 0U != (bits & 1U), 0U != (bits & 2U), travel,
                         now, &t);
        }
        assert(t.left_um_s >= 0);
        assert(t.right_um_s >= 0);
        assert(t.left_um_s <= CRUISE);
        assert(t.right_um_s <= CRUISE);
        now += STEP_MS;
        travel += 800;
    }
}

/* ---- simulator -------------------------------------------------------- */

#define SENSOR_AHEAD_MM   60.0
#define SENSOR_HALF_GAP   10.0 /* sensors 2 cm apart */
#define SIM_DT_S          0.01

typedef struct
{
    double x_mm;
    double y_mm;
    double th_rad;
    int64_t left_um;
    int64_t right_um;
} sim_t;

typedef struct
{
    double x0_mm;
    double y0_mm;
    double phi_rad;
    double half_width_mm;
} sim_line_t;

static double line_dist(sim_line_t const * p_ln, double px, double py)
{
    return ((px - p_ln->x0_mm) * sin(p_ln->phi_rad)) -
           ((py - p_ln->y0_mm) * cos(p_ln->phi_rad));
}

static bool sensor(sim_t const * p_s, sim_line_t const * p_ln, double side)
{
    double const lx = SENSOR_AHEAD_MM;
    double const ly = side * SENSOR_HALF_GAP;
    double const px = p_s->x_mm + (lx * cos(p_s->th_rad)) -
                      (ly * sin(p_s->th_rad));
    double const py = p_s->y_mm + (lx * sin(p_s->th_rad)) +
                      (ly * cos(p_s->th_rad));

    return fabs(line_dist(p_ln, px, py)) <= p_ln->half_width_mm;
}

static void sim_move(sim_t * p_s, line_targets_t const * p_t)
{
    double const vl = (double)p_t->left_um_s / UM_PER_MM;
    double const vr = (double)p_t->right_um_s / UM_PER_MM;
    double const v = (vl + vr) / 2.0;
    double const w = (vr - vl) / (double)LINE_WHEEL_TRACK_MM;
    double const mid = p_s->th_rad + (w * SIM_DT_S / 2.0);

    p_s->x_mm += v * cos(mid) * SIM_DT_S;
    p_s->y_mm += v * sin(mid) * SIM_DT_S;
    p_s->th_rad += w * SIM_DT_S;
    p_s->left_um += (int64_t)(vl * SIM_DT_S * UM_PER_MM);
    p_s->right_um += (int64_t)(vr * SIM_DT_S * UM_PER_MM);
}

typedef struct
{
    int  lost_steps;
    int  neither_straight_steps;
    int  neither_steps;
    bool b_seen;
} sim_stats_t;

static void sim_run(sim_t * p_s, sim_line_t const * p_ln, int steps,
                    sim_stats_t * p_st)
{
    line_follow_t f;
    uint32_t now = 0U;
    int i;

    line_follow_init(&f);
    p_st->lost_steps = 0;
    p_st->neither_straight_steps = 0;
    p_st->neither_steps = 0;
    p_st->b_seen = false;
    for (i = 0; i < steps; i++)
    {
        bool const l = sensor(p_s, p_ln, 1.0);
        bool const r = sensor(p_s, p_ln, -1.0);
        line_targets_t t;
        line_follow_status_t status;

        status = follow(&f, l, r, (p_s->left_um + p_s->right_um) / 2, now, &t);
        if (l || r)
        {
            p_st->b_seen = true;
        }
        if (!l && !r && p_st->b_seen)
        {
            p_st->neither_steps++;
            if (t.left_um_s == t.right_um_s && t.left_um_s != 0)
            {
                p_st->neither_straight_steps++;
            }
        }
        if (LINE_FOLLOW_LOST == status)
        {
            p_st->lost_steps++;
        }
        sim_move(p_s, &t);
        now += STEP_MS;
    }
}

static double cross_track(sim_t const * p_s, sim_line_t const * p_ln)
{
    return fabs(line_dist(p_ln, p_s->x_mm, p_s->y_mm));
}

static double heading_error_deg(sim_t const * p_s, sim_line_t const * p_ln)
{
    double e = fmod(p_s->th_rad - p_ln->phi_rad, 2.0 * PI_RAD);

    if (e > PI_RAD)
    {
        e -= 2.0 * PI_RAD;
    }
    if (e < -PI_RAD)
    {
        e += 2.0 * PI_RAD;
    }
    return fabs(e) * 180.0 / PI_RAD;
}

/* Line at 30 degrees to the car, descending (right sensor first) and
 * rising (left sensor first), 14 mm and 20 mm tape, three entry offsets.
 * The car must settle on the line, never drive straight while it has no
 * sensor on the line, and never give up. */
static void test_sim_entry_30deg(void)
{
    static double const widths_mm[] = {7.0, 10.0};
    static double const offsets_mm[] = {-20.0, 0.0, 20.0};
    int dir;
    size_t w;
    size_t o;

    for (dir = -1; dir <= 1; dir += 2)
    {
        for (w = 0U; w < (sizeof(widths_mm) / sizeof(widths_mm[0])); w++)
        {
            for (o = 0U; o < (sizeof(offsets_mm) / sizeof(offsets_mm[0])); o++)
            {
                sim_t s = {0.0, 0.0, 0.0, 0, 0};
                sim_line_t ln = {300.0, offsets_mm[o],
                                 (double)dir * PI_RAD / 6.0, widths_mm[w]};
                sim_stats_t st;

                sim_run(&s, &ln, 500, &st);
                assert(st.b_seen);
                assert(0 == st.neither_straight_steps);
                assert(0 == st.lost_steps);
                assert(cross_track(&s, &ln) < 25.0);
                assert(heading_error_deg(&s, &ln) < 25.0);
            }
        }
    }
}

/* ---- turns ------------------------------------------------------------ */

static line_turn_status_t turn_step(line_turn_state_t * p_tn, int64_t l,
                                    int64_t r, bool b_fresh, bool sl, bool sr,
                                    line_targets_t * p_t)
{
    return line_turn_update(p_tn, l, r, b_fresh, sl, sr, p_t);
}

static void test_heading_sign(void)
{
    line_turn_state_t tn;
    int64_t l;
    int64_t r;

    line_turn_start(&tn, LINE_TURN_KIND_LEFT, 1000, 2000);
    left_turn_disp(900, &l, &r);
    assert(line_turn_heading_deci(&tn, l, r) >= 900);
    assert(line_turn_heading_deci(&tn, l, r) <= 905);
    right_turn_disp(900, &l, &r);
    assert(line_turn_heading_deci(&tn, l, r) <= -900);
    assert(line_turn_heading_deci(&tn, l, r) >= -905);
    assert(0 == line_turn_heading_deci(&tn, 1000, 2000));
    /* Straight driving does not change the heading. */
    assert(0 == line_turn_heading_deci(&tn, 51000, 52000));
}

static void test_left_turn_phases(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_turn_start(&tn, LINE_TURN_KIND_LEFT, 1000, 2000);
    assert(LINE_TURN_TURNING ==
           turn_step(&tn, 1000, 2000, true, false, false, &t));
    expect_targets(&t, -LINE_TURN_FAST_UM_S, LINE_TURN_FAST_UM_S);

    /* Just before the capture window: still fast. */
    left_turn_disp(LINE_TURN_QUARTER_DECI - LINE_TURN_CAPTURE_DECI - 20, &l,
                   &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, -LINE_TURN_FAST_UM_S, LINE_TURN_FAST_UM_S);

    /* Inside the window: slow. */
    left_turn_disp(LINE_TURN_QUARTER_DECI - LINE_TURN_CAPTURE_DECI + 20, &l,
                   &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, -LINE_TURN_SLOW_UM_S, LINE_TURN_SLOW_UM_S);

    /* Slow until a sensor sees the line, even near the target. */
    left_turn_disp(LINE_TURN_QUARTER_DECI + 100, &l, &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, -LINE_TURN_SLOW_UM_S, LINE_TURN_SLOW_UM_S);

    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, false, true, &t));
    expect_targets(&t, 0, 0);
    assert(LINE_SIDE_LEFT == line_turn_side(&tn));
}

static void test_right_turn_and_either_sensor(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_turn_start(&tn, LINE_TURN_KIND_RIGHT, 1000, 2000);
    assert(LINE_TURN_TURNING ==
           turn_step(&tn, 1000, 2000, true, false, false, &t));
    expect_targets(&t, LINE_TURN_FAST_UM_S, -LINE_TURN_FAST_UM_S);

    right_turn_disp(LINE_TURN_QUARTER_DECI - LINE_TURN_CAPTURE_DECI + 20, &l,
                    &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, LINE_TURN_SLOW_UM_S, -LINE_TURN_SLOW_UM_S);
    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, true, false, &t));
    expect_targets(&t, 0, 0);
    assert(LINE_SIDE_RIGHT == line_turn_side(&tn));
    /* DONE is sticky. */
    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, 0, 0);
}

static void test_sensors_ignored_before_capture(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_turn_start(&tn, LINE_TURN_KIND_RIGHT, 1000, 2000);
    /* At the start the sensors sit on the line the car is leaving. */
    assert(LINE_TURN_TURNING ==
           turn_step(&tn, 1000, 2000, true, true, true, &t));
    expect_targets(&t, LINE_TURN_FAST_UM_S, -LINE_TURN_FAST_UM_S);
    right_turn_disp(LINE_TURN_QUARTER_DECI - LINE_TURN_CAPTURE_DECI - 20, &l,
                    &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, true, true, &t));
    expect_targets(&t, LINE_TURN_FAST_UM_S, -LINE_TURN_FAST_UM_S);
}

static void test_uturn(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_turn_start(&tn, LINE_TURN_KIND_UTURN, 1000, 2000);
    assert(LINE_TURN_TURNING ==
           turn_step(&tn, 1000, 2000, true, false, false, &t));
    expect_targets(&t, LINE_TURN_FAST_UM_S, -LINE_TURN_FAST_UM_S);

    /* The 90 degree capture point is not the end of a U-turn. */
    right_turn_disp(LINE_TURN_QUARTER_DECI + 100, &l, &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, true, true, &t));
    expect_targets(&t, LINE_TURN_FAST_UM_S, -LINE_TURN_FAST_UM_S);

    right_turn_disp(LINE_TURN_HALF_DECI - LINE_TURN_CAPTURE_DECI + 20, &l,
                    &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, LINE_TURN_SLOW_UM_S, -LINE_TURN_SLOW_UM_S);
    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, false, true, &t));
    assert(LINE_SIDE_RIGHT == line_turn_side(&tn));

    /* A U-turn aborts at 180 + margin, not at 90 + margin. */
    line_turn_start(&tn, LINE_TURN_KIND_UTURN, 1000, 2000);
    right_turn_disp(LINE_TURN_HALF_DECI + LINE_TURN_ABORT_DECI - 20, &l, &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    right_turn_disp(LINE_TURN_HALF_DECI + LINE_TURN_ABORT_DECI + 20, &l, &r);
    assert(LINE_TURN_ABORTED == turn_step(&tn, l, r, true, false, false, &t));
}

static void test_abort_past_margin(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_turn_start(&tn, LINE_TURN_KIND_LEFT, 1000, 2000);
    left_turn_disp(LINE_TURN_QUARTER_DECI + LINE_TURN_ABORT_DECI - 20, &l, &r);
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, -LINE_TURN_SLOW_UM_S, LINE_TURN_SLOW_UM_S);
    left_turn_disp(LINE_TURN_QUARTER_DECI + LINE_TURN_ABORT_DECI + 20, &l, &r);
    assert(LINE_TURN_ABORTED == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, 0, 0);
    /* Sticky, even if a sensor fires afterwards. */
    assert(LINE_TURN_ABORTED == turn_step(&tn, l, r, true, true, true, &t));
    expect_targets(&t, 0, 0);
}

static void test_abort_on_stale_feedback(void)
{
    line_turn_state_t tn;
    line_targets_t t;

    line_turn_start(&tn, LINE_TURN_KIND_RIGHT, 1000, 2000);
    assert(LINE_TURN_ABORTED ==
           turn_step(&tn, 1000, 2000, false, false, false, &t));
    expect_targets(&t, 0, 0);
}

static void test_wrong_direction_never_completes(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;
    int32_t deci;
    line_turn_status_t st = LINE_TURN_TURNING;

    /* Crossed encoders or a reversed wheel: heading grows the wrong way. */
    line_turn_start(&tn, LINE_TURN_KIND_LEFT, 1000, 2000);
    for (deci = 0; (deci <= 2000) && (LINE_TURN_TURNING == st); deci += 50)
    {
        right_turn_disp(deci, &l, &r);
        st = turn_step(&tn, l, r, true, true, true, &t);
        assert(LINE_TURN_DONE != st);
    }
    assert(LINE_TURN_ABORTED == st);
}

static void test_wrong_direction_one_sensor_never_completes(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;
    int32_t deci;
    line_turn_status_t st = LINE_TURN_TURNING;

    line_turn_start(&tn, LINE_TURN_KIND_RIGHT, 1000, 2000);
    for (deci = 0; (deci <= 2000) && (LINE_TURN_TURNING == st); deci += 50)
    {
        left_turn_disp(deci, &l, &r);
        st = turn_step(&tn, l, r, true, false, true, &t);
        assert(LINE_TURN_DONE != st);
    }
    assert(LINE_TURN_ABORTED == st);
}

static void test_step_without_start_aborts(void)
{
    line_turn_state_t tn = {0, 0, 0, 0U, 0U, 0U};
    line_targets_t t;

    assert(LINE_TURN_ABORTED == turn_step(&tn, 0, 0, true, false, false, &t));
    expect_targets(&t, 0, 0);
}

static void test_recorded_side_after_turn(void)
{
    line_follow_t f;
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0U, &t); /* side = RIGHT */

    line_turn_start(&tn, LINE_TURN_KIND_LEFT, 0, 0);
    left_turn_disp(LINE_TURN_QUARTER_DECI - LINE_TURN_CAPTURE_DECI + 20, &l,
                   &r);
    (void)turn_step(&tn, l, r, true, false, false, &t);
    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, true, false, &t));
    line_follow_set_side(&f, line_turn_side(&tn));

    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 0U, &t));
    expect_targets(&t, 0, CRUISE);
}

/* After an aborted turn the caller holds the car until a sensor sees the
 * line, with or without a recorded side. */
static void test_hold_lost(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    line_follow_hold_lost(&f);
    assert(LINE_FOLLOW_LOST == follow(&f, false, false, 0, 0U, &t));
    expect_targets(&t, 0, 0);
    assert(LINE_FOLLOW_LOST == follow(&f, false, false, 0, 5000U, &t));
    expect_targets(&t, 0, 0);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, false, 0, 5010U, &t));
    expect_targets(&t, 0, CRUISE);

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0U, &t);
    line_follow_hold_lost(&f);
    assert(LINE_FOLLOW_LOST == follow(&f, false, false, 0, 10U, &t));
    expect_targets(&t, 0, 0);
}

/* Pin the starting values; a change must be a deliberate edit of the test
 * and of the tuning notes. */
static void test_constants(void)
{
    assert(163364 == LINE_CRUISE_UM_S);
    assert(107 == LINE_WHEEL_TRACK_MM);
    assert(150 == LINE_SEARCH_MAX_MM);
    assert(1500U == LINE_SEARCH_MAX_MS);
    assert(180000 == LINE_TURN_FAST_UM_S);
    assert(80000 == LINE_TURN_SLOW_UM_S);
    assert(300 == LINE_TURN_CAPTURE_DECI);
    assert(450 == LINE_TURN_ABORT_DECI);
    assert(900 == LINE_TURN_QUARTER_DECI);
    assert(1800 == LINE_TURN_HALF_DECI);
}

static void test_set_side_clears_lost(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0U, &t);
    (void)follow(&f, false, false, 0, 10U, &t);
    assert(LINE_FOLLOW_LOST ==
           follow(&f, false, false, 0, 10U + LINE_SEARCH_MAX_MS, &t));
    line_follow_set_side(&f, LINE_SIDE_RIGHT);
    assert(LINE_FOLLOW_SEARCHING ==
           follow(&f, false, false, 0, 2000U, &t));
    expect_targets(&t, CRUISE, 0);
}

int main(void)
{
    test_constants();
    test_init();
    test_one_sensor_correction();
    test_both_and_neither_straight();
    test_both_keeps_recorded_side();
    test_entry_sequence_keeps_turning();
    test_hairpin();
    test_search_limit_distance();
    test_search_limit_time();
    test_search_limit_time_wraps();
    test_recovery_after_lost();
    test_targets_never_exceed_cruise();
    test_sim_entry_30deg();
    test_heading_sign();
    test_left_turn_phases();
    test_right_turn_and_either_sensor();
    test_sensors_ignored_before_capture();
    test_uturn();
    test_abort_past_margin();
    test_abort_on_stale_feedback();
    test_wrong_direction_never_completes();
    test_wrong_direction_one_sensor_never_completes();
    test_step_without_start_aborts();
    test_recorded_side_after_turn();
    test_hold_lost();
    test_set_side_clears_lost();
    (void)puts("line_control: all tests passed");
    return 0;
}
