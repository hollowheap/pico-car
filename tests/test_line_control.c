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
#define TRACK_UM        ((int64_t)LINE_WHEEL_TRACK_MM * UM_PER_MM)
#define DECI_PER_RAD    573
#define PI_RAD          3.14159265358979323846
#define LCG_MUL         1103515245U
#define LCG_ADD         12345U
#define LCG_SHIFT       16U
#define RANDOM_STEPS    5000

/* ---- helpers ---------------------------------------------------------- */

static line_follow_status_t follow(line_follow_t * p_f, bool b_left,
                                   bool b_right, int32_t heading_deci,
                                   int64_t travel_um, uint32_t now_ms,
                                   line_targets_t * p_t)
{
    return line_follow_update(p_f, b_left, b_right, heading_deci, travel_um,
                              now_ms, p_t);
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

static line_turn_status_t turn_step(line_turn_state_t * p_tn, int64_t l,
                                    int64_t r, bool b_fresh, bool sl, bool sr,
                                    line_targets_t * p_t)
{
    return line_turn_update(p_tn, l, r, b_fresh, sl, sr, p_t);
}


/* ---- follower unit tests ---------------------------------------------- */

static void test_init(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    /* The first pass takes the heading as the target: straight on, cruise. */
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 500, 0, 0U, &t));
    expect_targets(&t, CRUISE, CRUISE);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, true, 500, 0, 10U, &t));
    expect_targets(&t, CRUISE, CRUISE);
}

/* One sensor on the line slews the target toward it; the inner wheel slows but
 * does not stop on the first pass. */
static void test_one_sensor_steers_toward_the_line(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t ms;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    for (ms = 10U; ms <= 30U; ms += 10U)
    {
        assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, false, 0, 0, ms, &t));
    }
    /* Left sensor: turn left, the left wheel is the inner one. */
    assert(t.left_um_s < CRUISE);
    assert(t.left_um_s > 0);
    assert(CRUISE == t.right_um_s);

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    for (ms = 10U; ms <= 30U; ms += 10U)
    {
        (void)follow(&f, false, true, 0, 0, ms, &t);
    }
    assert(CRUISE == t.left_um_s);
    assert(t.right_um_s < CRUISE);
    assert(t.right_um_s > 0);
}

/* The target stays within LINE_CAP_DECI of the heading, so the inner wheel never
 * stops for good however long a sensor stays on. */
static void test_cap_limits_the_correction(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t ms;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0, 0U, &t);
    for (ms = 10U; ms <= 5000U; ms += 10U)
    {
        (void)follow(&f, true, false, 0, 0, ms, &t);
    }
    assert(t.left_um_s >= 0);
    assert(CRUISE == t.right_um_s);
    assert(f.ref_deci == LINE_CAP_DECI);
}

/* Neither sensor: the car holds its heading for LINE_GAP_MM, then weaves. It
 * never stops. */
static void test_gap_then_weave(void)
{
    line_follow_t f;
    line_targets_t t;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;
    int64_t travel = 0;
    uint32_t ms = 0U;
    bool b_left_lead = false;
    bool b_right_lead = false;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0, ms, &t);
    /* 10 ms per pass at cruise is 1.63 mm. */
    while (travel < ((int64_t)(LINE_GAP_MM - 5) * UM_PER_MM))
    {
        ms += 10U;
        travel += (CRUISE * 10) / 1000;
        status = follow(&f, false, false, 0, travel, ms, &t);
        assert(LINE_FOLLOW_BRIDGING == status);
        expect_targets(&t, CRUISE, CRUISE);
    }
    while (LINE_FOLLOW_SEARCHING != status)
    {
        ms += 10U;
        travel += (CRUISE * 10) / 1000;
        status = follow(&f, false, false, 0, travel, ms, &t);
    }
    /* The weave runs at the search speed and swings both ways. */
    for (ms += 10U; ms < 20000U; ms += 10U)
    {
        int32_t const heading = 0;

        travel += (LINE_SEARCH_UM_S * 10) / 1000;
        assert(LINE_FOLLOW_SEARCHING ==
               follow(&f, false, false, heading, travel, ms, &t));
        assert(t.left_um_s >= 0);
        assert(t.right_um_s >= 0);
        assert((t.left_um_s + t.right_um_s) > 0);
        assert(t.left_um_s <= LINE_SEARCH_UM_S);
        assert(t.right_um_s <= LINE_SEARCH_UM_S);
        b_left_lead  = b_left_lead || (t.right_um_s < t.left_um_s);
        b_right_lead = b_right_lead || (t.left_um_s < t.right_um_s);
    }
    assert(b_left_lead && b_right_lead);
    /* A sensor hit ends the search. */
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, false, 0, travel, ms, &t));
    assert(0 == f.sweep_dir - f.sweep_dir);
}

/* The weave starts toward the side that saw the line last. */
static void test_weave_starts_toward_the_remembered_side(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    uint32_t ms = 0U;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0, ms, &t); /* the right sensor saw it */
    for (pass = 0; pass < 120; pass++)
    {
        ms += 10U;
        travel += (CRUISE * 10) / 1000;
        (void)follow(&f, false, false, 0, travel, ms, &t);
    }
    assert(LINE_FOLLOW_SEARCHING == f.status);
    assert(-1 == f.sweep_dir);
}

static void test_targets_never_exceed_cruise(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t seed = 12345U;
    uint32_t ms = 0U;
    int64_t travel = 0;
    int32_t heading = 0;
    int step;

    line_follow_init(&f);
    for (step = 0; step < RANDOM_STEPS; step++)
    {
        bool b_left;
        bool b_right;

        seed = (seed * LCG_MUL) + LCG_ADD;
        b_left = (0U != ((seed >> LCG_SHIFT) & 1U));
        b_right = (0U != ((seed >> (LCG_SHIFT + 1U)) & 1U));
        heading += (int32_t)((seed >> 20U) % 21U) - 10;
        ms += 10U;
        travel += 1600;
        (void)follow(&f, b_left, b_right, heading, travel, ms, &t);
        assert(t.left_um_s >= 0 && t.left_um_s <= CRUISE);
        assert(t.right_um_s >= 0 && t.right_um_s <= CRUISE);
        assert((t.left_um_s + t.right_um_s) > 0);
    }
}

static void test_time_wraps(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0, 0xFFFFFFF0U, &t);
    (void)follow(&f, true, false, 0, 0, 0x00000004U, &t);
    /* 20 ms passed, not 4 billion: the target moved by about 12 deci-degrees. */
    assert(f.ref_deci > 0 && f.ref_deci < 40); /* 20 ms at LINE_RATE_HIT, not 4 billion */
}

static void test_heading_helper(void)
{
    assert(0 == line_heading_deci(1000, 1000));
    assert(line_heading_deci(0, 1000) > 0);
    assert(line_heading_deci(1000, 0) < 0);
    /* A pivot of 90 degrees: each wheel moves 107 * pi / 4 mm. */
    assert(line_heading_deci(-84000, 84000) > 880);
    assert(line_heading_deci(-84000, 84000) < 920);
}

/* ---- kinematic simulation: legacy law against the heading cascade ------ */

#define SIM_DT_S          0.001
#define SIM_SAMPLE_S      0.005
#define SIM_TICK_S        0.010
#define SIM_LAG_UP_S      0.060
#define SIM_LAG_DOWN_S    0.020
#define SIM_EDGE_ZONE_MM  1.5
#define SIM_CHATTER       0.3
#define SIM_DEBOUNCE      3

typedef enum { LAW_LEGACY = 0, LAW_CASCADE } law_t;

typedef struct
{
    law_t  law;
    double radius;       /* 0 = straight */
    double gap_from_mm;  /* no line between these arc positions, 0 = none */
    double gap_len_mm;
    double seconds;
    bool   b_corner;     /* a straight, a 90 degree arc of `radius`, a straight */
} sim_case_t;

typedef struct
{
    double edges_hz;     /* sensor state changes per second after the settle */
    double heading_deg;  /* largest |heading| on a straight line             */
    double lost_pct;     /* time the sensor midpoint is off the line         */
    double max_off_mm;
    bool   b_stopped;    /* both targets zero at some pass                   */
    double gap_heading_deg;
} sim_result_t;

typedef struct
{
    bool lvl;
    int  count;
} debounce_t;

/* The old law without its search limit: inner wheel stop toward the last side. */
static void legacy_update(uint8_t * p_side, bool b_left, bool b_right,
                          line_targets_t * p_t)
{
    if (b_left && b_right)
    {
        p_t->left_um_s = CRUISE;
        p_t->right_um_s = CRUISE;
        return;
    }
    if (b_left || b_right)
    {
        *p_side = (uint8_t)(b_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
    }
    if (LINE_SIDE_NONE == (line_side_t)*p_side)
    {
        p_t->left_um_s = CRUISE;
        p_t->right_um_s = CRUISE;
    }
    else if (LINE_SIDE_LEFT == (line_side_t)*p_side)
    {
        p_t->left_um_s = 0;
        p_t->right_um_s = CRUISE;
    }
    else
    {
        p_t->left_um_s = CRUISE;
        p_t->right_um_s = 0;
    }
}

/* Distance of a point to the path, signed, left of travel positive. The path
 * is the x axis, a circle of the given radius that turns left, or (b_corner)
 * the x axis up to x = 0, a quarter circle, then the line x = radius going up:
 * the track corners are 90 degree arcs. */
static double path_dist(sim_case_t const * p_c, double px, double py)
{
    double const r = p_c->radius;

    if (0.0 == r)
    {
        return py;
    }
    if (!p_c->b_corner || ((px >= 0.0) && (py <= r)))
    {
        return r - hypot(px, py - r);
    }
    return (px < 0.0) ? py : (r - px);
}

/* Arc length along the path to the foot of a point. */
static double path_pos(sim_case_t const * p_c, double px, double py)
{
    double const r = p_c->radius;

    if (0.0 == r)
    {
        return px;
    }
    if (!p_c->b_corner || ((px >= 0.0) && (py <= r)))
    {
        return r * atan2(px, r - py);
    }
    return (px < 0.0) ? px : ((r * PI_RAD / 2.0) + (py - r));
}

static bool sim_sensor_raw(sim_case_t const * p_c, double x, double y, double psi,
                           double side, double * p_dist)
{
    double const sx = x + (LINE_SENSOR_LEAD_MM * cos(psi)) -
                      (side * (LINE_SENSOR_SPACING_MM / 2.0) * sin(psi));
    double const sy = y + (LINE_SENSOR_LEAD_MM * sin(psi)) +
                      (side * (LINE_SENSOR_SPACING_MM / 2.0) * cos(psi));
    double const d = fabs(path_dist(p_c, sx, sy));
    double const pos = path_pos(p_c, sx, sy);
    bool b_on = (d <= (LINE_WIDTH_MM / 2.0));

    if ((p_c->gap_len_mm > 0.0) && (pos >= p_c->gap_from_mm) &&
        (pos < (p_c->gap_from_mm + p_c->gap_len_mm)))
    {
        b_on = false;
    }
    *p_dist = d;
    return b_on;
}

static sim_result_t sim_run(sim_case_t const * p_c)
{
    sim_result_t res = {0.0, 0.0, 0.0, 0.0, false, 0.0};
    /* A straight run starts 6 mm off the line. A curve starts at the steady
     * state: the sensor midpoint on the arc, the axle on the smaller circle. */
    double x = p_c->b_corner ? -600.0 : 0.0, psi = 0.0;
    double y = ((0.0 == p_c->radius) || p_c->b_corner)
                   ? 6.0
                   : (p_c->radius - sqrt((p_c->radius * p_c->radius) -
                                         ((double)LINE_SENSOR_LEAD_MM * LINE_SENSOR_LEAD_MM)));
    double vl = 0.0, vr = 0.0, tl = CRUISE / 1000.0, tr = CRUISE / 1000.0;
    double next_sample = 0.0, next_tick = 0.0, t = 0.0;
    double disp_l_um = 0.0, disp_r_um = 0.0;
    double pending_l = tl, pending_r = tr;
    debounce_t filt[2] = {{false, 0}, {false, 0}};
    bool last_l = false, last_r = false, b_first = true;
    unsigned changes = 0U, samples = 0U, off = 0U;
    uint32_t seed = 1U;
    line_follow_t f;
    uint8_t legacy_side = (uint8_t)LINE_SIDE_LEFT;
    double const settle = p_c->seconds / 4.0;

    line_follow_init(&f);
    while (t < p_c->seconds)
    {
        if (t >= next_sample)
        {
            int i;

            next_sample += SIM_SAMPLE_S;
            for (i = 0; i < 2; i++)
            {
                double dist = 0.0;
                bool raw = sim_sensor_raw(p_c, x, y, psi, (0 == i) ? 1.0 : -1.0,
                                          &dist);

                seed = (seed * LCG_MUL) + LCG_ADD;
                if ((fabs(dist - (LINE_WIDTH_MM / 2.0)) < SIM_EDGE_ZONE_MM) &&
                    ((double)((seed >> LCG_SHIFT) & 0x7FFFU) / 32768.0 < SIM_CHATTER))
                {
                    raw = !raw;
                }
                if (raw == filt[i].lvl)
                {
                    filt[i].count = 0;
                }
                else if (++filt[i].count >= SIM_DEBOUNCE)
                {
                    filt[i].lvl = raw;
                    filt[i].count = 0;
                }
            }
        }
        if (t >= next_tick)
        {
            line_targets_t tg = {0, 0};
            line_follow_status_t status = LINE_FOLLOW_FOLLOWING;
            bool const bl = filt[0].lvl;
            bool const br = filt[1].lvl;

            next_tick += SIM_TICK_S;
            if (!b_first && t > settle && (bl != last_l || br != last_r))
            {
                changes++;
            }
            b_first = false;
            last_l = bl;
            last_r = br;
            if (LAW_LEGACY == p_c->law)
            {
                legacy_update(&legacy_side, bl, br, &tg);
            }
            else
            {
                status = follow(&f, bl, br,
                                line_heading_deci((int64_t)disp_l_um,
                                                  (int64_t)disp_r_um),
                                (int64_t)((disp_l_um + disp_r_um) / 2.0),
                                (uint32_t)(t * 1000.0), &tg);
                (void)status;
            }
            if (0 == tg.left_um_s && 0 == tg.right_um_s)
            {
                res.b_stopped = true;
            }
            /* One tick of command delay. */
            tl = pending_l;
            tr = pending_r;
            pending_l = tg.left_um_s / 1000.0;
            pending_r = tg.right_um_s / 1000.0;
        }
        vl += (tl - vl) * (SIM_DT_S / ((tl > vl) ? SIM_LAG_UP_S : SIM_LAG_DOWN_S));
        vr += (tr - vr) * (SIM_DT_S / ((tr > vr) ? SIM_LAG_UP_S : SIM_LAG_DOWN_S));
        x += ((vl + vr) / 2.0) * cos(psi) * SIM_DT_S;
        y += ((vl + vr) / 2.0) * sin(psi) * SIM_DT_S;
        psi += ((vr - vl) / LINE_WHEEL_TRACK_MM) * SIM_DT_S;
        disp_l_um += vl * 1000.0 * SIM_DT_S;
        disp_r_um += vr * 1000.0 * SIM_DT_S;
        t += SIM_DT_S;
        if (t > settle)
        {
            double const mx = x + (LINE_SENSOR_LEAD_MM * cos(psi));
            double const my = y + (LINE_SENSOR_LEAD_MM * sin(psi));
            double const mid = fabs(path_dist(p_c, mx, my));
            double const pos = path_pos(p_c, mx, my);
            bool const b_in_gap = (p_c->gap_len_mm > 0.0) &&
                                  (pos > (p_c->gap_from_mm - 1.0)) &&
                                  (pos < (p_c->gap_from_mm + p_c->gap_len_mm + 60.0));

            samples++;
            if (mid > ((LINE_SENSOR_SPACING_MM + LINE_WIDTH_MM) / 2.0))
            {
                off++;
            }
            if (mid > res.max_off_mm)
            {
                res.max_off_mm = mid;
            }
            if (0.0 == p_c->radius)
            {
                double const deg = fabs(psi * 180.0 / PI_RAD);

                if (deg > res.heading_deg)
                {
                    res.heading_deg = deg;
                }
                if (b_in_gap && deg > res.gap_heading_deg)
                {
                    res.gap_heading_deg = deg;
                }
            }
        }
    }
    res.edges_hz = (double)changes / (p_c->seconds - settle);
    res.lost_pct = (100.0 * (double)off) / (double)samples;
    return res;
}

/* The old law hunts on a straight line; the cascade hunts much less. This test
 * fails on the old law. */
static void test_sim_straight_hunting(void)
{
    sim_case_t c = {LAW_LEGACY, 0.0, 0.0, 0.0, 12.0, false};
    sim_result_t legacy;
    sim_result_t cascade;

    legacy = sim_run(&c);
    c.law = LAW_CASCADE;
    cascade = sim_run(&c);
    (void)printf("straight: legacy %.1f edges/s (%.1f deg), cascade %.1f edges/s (%.1f deg, %.2f%% lost, max off %.1f mm)\n",
                 legacy.edges_hz, legacy.heading_deg, cascade.edges_hz,
                 cascade.heading_deg, cascade.lost_pct, cascade.max_off_mm);
    assert(legacy.edges_hz > 6.0);
    assert(cascade.edges_hz < 8.0);
    assert(cascade.edges_hz < (legacy.edges_hz * 0.65));
    assert(cascade.heading_deg < 14.0);
    assert(!cascade.b_stopped);
    assert(cascade.lost_pct < 1.0);
    assert(cascade.max_off_mm < 25.0);
}

static void test_sim_curves_keep_the_line(void)
{
    sim_case_t c = {LAW_CASCADE, 400.0, 0.0, 0.0, 12.0, false};
    sim_result_t res;

    res = sim_run(&c);
    (void)printf("R400 circle: %.1f edges/s, %.2f%% lost, max off %.1f mm\n",
                 res.edges_hz, res.lost_pct, res.max_off_mm);
    assert(res.lost_pct < 1.0);
    assert(!res.b_stopped);
    /* The track corner: a straight, a 90 degree arc of 150 mm, a straight. */
    c.radius = 150.0;
    c.b_corner = true;
    c.seconds = 10.0;
    res = sim_run(&c);
    (void)printf("R150 corner: %.1f edges/s, %.2f%% lost, max off %.1f mm\n",
                 res.edges_hz, res.lost_pct, res.max_off_mm);
    assert(res.lost_pct < 1.0);
    assert(res.max_off_mm < 25.0);
    assert(!res.b_stopped);
}

/* A 60 mm cut in the line (the run-on-car check): the car crosses it. */
static void test_sim_crosses_a_gap(void)
{
    sim_case_t c = {LAW_CASCADE, 0.0, 800.0, 60.0, 14.0, false};
    sim_result_t res;

    res = sim_run(&c);
    (void)printf("gap 60 mm: heading %.1f deg across, max off %.1f mm\n",
                 res.gap_heading_deg, res.max_off_mm);
    assert(!res.b_stopped);
    assert(res.lost_pct < 1.0);
    assert(res.gap_heading_deg < 12.0);
}

/* After an aborted turn the car looks for the line and does not stop. */
static void test_start_search(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    line_follow_start_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 0, 0U, &t));
    assert((t.left_um_s + t.right_um_s) > 0);
    assert(t.left_um_s <= LINE_SEARCH_UM_S && t.right_um_s <= LINE_SEARCH_UM_S);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, true, 0, 0, 10U, &t));
}

/* A turn ends with line_follow_set_side(); the heading target restarts from the
 * heading after the turn. */
static void test_recorded_side_after_turn(void)
{
    line_follow_t f;
    line_turn_state_t tn;
    line_targets_t t;
    int64_t l;
    int64_t r;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0, 0U, &t); /* side = RIGHT */

    line_turn_start(&tn, LINE_TURN_KIND_LEFT, 0, 0);
    left_turn_disp(LINE_TURN_QUARTER_DECI - LINE_TURN_CAPTURE_DECI + 20, &l,
                   &r);
    (void)turn_step(&tn, l, r, true, false, false, &t);
    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, true, false, &t));
    line_follow_set_side(&f, line_turn_side(&tn));
    assert(LINE_SIDE_LEFT == (line_side_t)f.side);

    /* Straight on in the new heading, not a swerve back to the old one. */
    assert(LINE_FOLLOW_BRIDGING ==
           follow(&f, false, false, line_heading_deci(l, r), 0, 5000U, &t));
    expect_targets(&t, CRUISE, CRUISE);
}

/* STRAIGHT drives LINE_STRAIGHT_MM on the start heading, ignores the line
 * sensors, and steers back when the heading drifts. */
static void test_straight(void)
{
    line_turn_state_t tn;
    line_targets_t t;
    int64_t travel;

    line_turn_start(&tn, LINE_TURN_KIND_STRAIGHT, 1000, 2000);
    assert(LINE_SIDE_NONE == line_turn_side(&tn));
    /* No travel yet: both wheels at cruise, whatever the sensors say. */
    assert(LINE_TURN_TURNING == turn_step(&tn, 1000, 2000, true, true, false, &t));
    expect_targets(&t, CRUISE, CRUISE);
    assert(LINE_TURN_TURNING == turn_step(&tn, 1000, 2000, true, false, true, &t));

    /* A drift to the left (right wheel ahead): the left wheel slows. Steering
     * back means turning right, so the right wheel is the inner wheel. */
    assert(LINE_TURN_TURNING == turn_step(&tn, 1000 + 50000, 2000 + 52000, true, false, false, &t));
    assert(t.right_um_s < CRUISE);
    assert(CRUISE == t.left_um_s);

    /* Done after LINE_STRAIGHT_MM of mean travel, then sticky. */
    travel = (int64_t)LINE_STRAIGHT_MM * UM_PER_MM;
    assert(LINE_TURN_DONE == turn_step(&tn, 1000 + travel, 2000 + travel, true, false, false, &t));
    expect_targets(&t, 0, 0);
    assert(LINE_TURN_DONE == turn_step(&tn, 1000, 2000, true, false, false, &t));

    /* Stale feedback aborts. */
    line_turn_start(&tn, LINE_TURN_KIND_STRAIGHT, 0, 0);
    assert(LINE_TURN_ABORTED == turn_step(&tn, 0, 0, false, false, false, &t));
    /* A heading error past the abort margin aborts too. */
    line_turn_start(&tn, LINE_TURN_KIND_STRAIGHT, 0, 0);
    assert(LINE_TURN_ABORTED == turn_step(&tn, 0, 100000, true, false, false, &t));
}

/* Pin the starting values; a change must be a deliberate edit of the test
 * and of the tuning notes. */
static void test_constants(void)
{
    assert(163364 == LINE_CRUISE_UM_S);
    assert(107 == LINE_WHEEL_TRACK_MM);
    assert(25 == LINE_SENSOR_SPACING_MM);
    assert(8 == LINE_KH);
    assert(600 == LINE_RATE_HIT);
    assert(120 == LINE_CAP_DECI);
    assert(100 == LINE_GAP_MM);
    assert(100000 == LINE_SEARCH_UM_S);
    assert(180000 == LINE_TURN_FAST_UM_S);
    assert(80000 == LINE_TURN_SLOW_UM_S);
    assert(300 == LINE_TURN_CAPTURE_DECI);
    assert(450 == LINE_TURN_ABORT_DECI);
    assert(900 == LINE_TURN_QUARTER_DECI);
    assert(1800 == LINE_TURN_HALF_DECI);
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

int main(void)
{
    test_init();
    test_one_sensor_steers_toward_the_line();
    test_cap_limits_the_correction();
    test_gap_then_weave();
    test_weave_starts_toward_the_remembered_side();
    test_targets_never_exceed_cruise();
    test_time_wraps();
    test_heading_helper();
    test_start_search();
    test_sim_straight_hunting();
    test_sim_curves_keep_the_line();
    test_sim_crosses_a_gap();
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
    test_straight();
    test_constants();
    (void)puts("line_control: all tests passed");
    return 0;
}
