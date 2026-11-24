/** @file test_line_control.c
 * @brief Host tests for the line follower and one-shot turns.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "track/line_control.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define STEP_MS         10U
#define CRUISE          LINE_CRUISE_UM_S
#define TRACK_UM        ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
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

/* Feed recorded-heading style samples directly to the controller. A sustained
 * edge while the heading lags should tighten smoothly in either direction,
 * including after a previous 270 degree bend. */
static void test_corner_wheel_demands(void)
{
    for (int side = -1; side <= 1; side += 2)
    {
        line_follow_t f;
        line_targets_t t;
        int32_t const heading = -side * 2700;
        line_follow_init(&f);
        (void)follow(&f, true, true, heading, 0, 0U, &t);
        for (uint32_t ms = 10U; ms <= 160U; ms += 10U)
        {
            (void)follow(&f, side > 0, side < 0, heading, (int64_t)ms * 100, ms, &t);
            int32_t const inner = side > 0 ? t.left_um_s : t.right_um_s;
            int32_t const outer = side > 0 ? t.right_um_s : t.left_um_s;
            assert(inner >= 0 && inner < outer);
            if (ms == 10U)
            {
                assert(inner > outer / 2);
            }
            if (ms == 60U)
            {
                assert(inner <= outer / 2);
            }
            if (ms == 160U)
            {
                assert(inner == 0 && outer == 100000);
            }
        }
        (void)follow(&f, true, true, heading, 18000, 180U, &t);
        assert(t.left_um_s == t.right_um_s && t.left_um_s > 100000 && t.left_um_s <= 102000);
        (void)follow(&f, side < 0, side > 0, heading, 19000, 190U, &t);
        assert((side > 0) ? (t.left_um_s > t.right_um_s) : (t.right_um_s > t.left_um_s));
    }

    /* Prescribed tape and encoder samples, independent of the wheel commands.
     * A brief centered sample joins two opposite 270 degree arcs. */
    line_follow_t f;
    line_targets_t t;
    int32_t heading = 0;
    int64_t travel = 0;
    uint32_t ms = 0U;
    line_follow_init(&f);
    (void)follow(&f, true, true, heading, travel, ms, &t);
    for (int side = 1; side >= -1; side -= 2)
    {
        for (int angle = 0; angle < 2700; angle += 4)
        {
            heading += side * 4;
            travel += 650;
            ms += STEP_MS;
            assert(follow(&f, side > 0, side < 0, heading, travel, ms, &t) == LINE_FOLLOW_FOLLOWING);
            assert((side > 0) ? (t.left_um_s < t.right_um_s) : (t.right_um_s < t.left_um_s));
            if (angle >= 900)
            {
                assert((side > 0 ? t.right_um_s : t.left_um_s) == LINE_CORNER_UM_S);
            }
        }
        ms += STEP_MS;
        (void)follow(&f, true, true, heading, travel, ms, &t);
        expect_targets(&t, LINE_CORNER_UM_S + LINE_ACCEL_MM_S2 * 10,
                       LINE_CORNER_UM_S + LINE_ACCEL_MM_S2 * 10);
    }
}

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
    assert(t.left_um_s < t.right_um_s);
    assert(t.left_um_s > 0);
    assert(t.right_um_s <= CRUISE);

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    for (ms = 10U; ms <= 30U; ms += 10U)
    {
        (void)follow(&f, false, true, 0, 0, ms, &t);
    }
    assert(t.left_um_s <= CRUISE);
    assert(t.right_um_s < t.left_um_s);
    assert(t.right_um_s > 0);
}

/* The speed drops with the heading error: full cruise on the line, the corner
 * speed at LINE_SLOW_DECI and more. A long correction is a corner. */
static void test_speed_follows_heading_error(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t ms;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    expect_targets(&t, CRUISE, CRUISE);
    /* 10 ms on the left sensor at cruise: the slew per m times the travel. */
    (void)follow(&f, true, false, 0, 0, 10U, &t);
    assert(((((LINE_HIT_DECI_PER_M * CRUISE) / 1000000) * 10) / 1000) == f.ref_deci);
    assert((CRUISE - (((CRUISE - LINE_CORNER_UM_S) * f.ref_deci) / LINE_SLOW_DECI)) ==
           t.right_um_s);
    assert(t.left_um_s < t.right_um_s);
    for (ms = 20U; ms <= 100U; ms += 10U)
    {
        (void)follow(&f, true, false, 0, 0, ms, &t);
    }
    assert(f.ref_deci >= LINE_SLOW_DECI);
    assert(LINE_CORNER_UM_S == t.right_um_s);
    /* Both sensors: the target is the heading again. The speed climbs back
     * at LINE_ACCEL_MM_S2, not at once, so the car does not run into the next
     * corner at cruise. */
    (void)follow(&f, true, true, 0, 0, 110U, &t);
    assert(t.left_um_s == t.right_um_s);
    assert(t.left_um_s == (LINE_CORNER_UM_S + (LINE_ACCEL_MM_S2 * 10)));
    for (ms = 120U; ms <= 2000U; ms += 10U)
    {
        int32_t const before = t.left_um_s;

        (void)follow(&f, true, true, 0, 0, ms, &t);
        assert(t.left_um_s >= before);
        assert((t.left_um_s - before) <= (LINE_ACCEL_MM_S2 * 10));
    }
    expect_targets(&t, CRUISE, CRUISE);
}

/* After a hit, the line between the sensors means the car points along it:
 * the target relaxes to the heading, so the car stops turning and does not
 * swing across to the other sensor (the weave). */
static void test_blind_relaxes_toward_heading(void)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t ms;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    int32_t before;

    for (ms = 10U; ms <= 100U; ms += 10U)
    {
        (void)follow(&f, true, false, 0, 0, ms, &t);
    }
    before = f.ref_deci;
    assert(before > 0);
    (void)follow(&f, false, false, 0, 1000, 110U, &t);
    assert(f.ref_deci < before);
    assert(f.ref_deci > 0);
    for (ms = 120U; ms <= 300U; ms += 10U)
    {
        assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 0, 1000, ms, &t));
    }
    assert(0 == f.ref_deci);
    assert(t.left_um_s == t.right_um_s);
}

/* A drift during the early gap must get a heading correction. A barcode
 * straight (or other reanchor) discards the old heading while still bridging. */
static void test_blind_heading_reanchors(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    (void)follow(&f, true, true, 200, 0, 0U, &t);
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 200, 1000, 10U, &t));
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 204, 2000, 20U, &t));
    assert(f.ref_deci == 200 && t.left_um_s > t.right_um_s);

    line_follow_reanchor(&f);
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 700, 50000, 50U, &t));
    assert(f.ref_deci == 700 && t.left_um_s == t.right_um_s);
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 704, 51000, 60U, &t));
    assert(f.ref_deci == 700 && t.left_um_s > t.right_um_s);

    /* Relax back to the new heading before the forward probe begins. */
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 704,
                                         50000 + (LINE_PROBE_FROM_MM - 1) * 1000, 70U, &t));
    assert(f.ref_deci == 704 && t.left_um_s == t.right_um_s);
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 704,
                                         50000 + LINE_PROBE_FROM_MM * 1000, 80U, &t));
    assert(f.ref_deci == 704 && t.left_um_s > 0 && t.left_um_s < t.right_um_s);
    line_targets_t const before = t;
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 704,
                                         50000 + (LINE_GAP_MM - 1) * 1000, 90U, &t));
    int32_t const rise = LINE_ACCEL_MM_S2 * 10;
    int32_t const outer = (before.right_um_s + rise < CRUISE) ? before.right_um_s + rise : CRUISE;
    expect_targets(&t, outer - (before.right_um_s - before.left_um_s), outer);
}

/* The sonar bumper: full cruise with nothing close, the bumper speed at the
 * obstacle detect distance and closer, a straight ramp between. */
static void test_bumper_cap(void)
{
    int32_t const mid_mm = (LINE_BUMPER_FAR_MM + LINE_BUMPER_NEAR_MM) / 2;
    int32_t const mid = line_bumper_cap_um_s(true, (uint32_t)mid_mm);

    assert(CRUISE == line_bumper_cap_um_s(false, 0U));
    assert(CRUISE == line_bumper_cap_um_s(true, 5000U));
    assert(CRUISE == line_bumper_cap_um_s(true, (uint32_t)LINE_BUMPER_FAR_MM));
    assert(LINE_BUMPER_MIN_UM_S == line_bumper_cap_um_s(true, (uint32_t)LINE_BUMPER_NEAR_MM));
    assert(LINE_BUMPER_MIN_UM_S == line_bumper_cap_um_s(true, 0U));
    assert(abs(mid - ((CRUISE + LINE_BUMPER_MIN_UM_S) / 2)) <= 1000);
    assert(line_bumper_cap_um_s(true, (uint32_t)LINE_BUMPER_NEAR_MM + 10U) >
           LINE_BUMPER_MIN_UM_S);
}

/* A cap scales both wheels, so the car keeps its curve while it slows. */
static void test_targets_cap(void)
{
    line_targets_t t = {300000, 150000};

    line_targets_cap(&t, 120000);
    expect_targets(&t, 120000, 60000);
    t.left_um_s  = 100000;
    t.right_um_s = 50000;
    line_targets_cap(&t, 120000);
    expect_targets(&t, 100000, 50000);
    t.left_um_s  = 0;
    t.right_um_s = 0;
    line_targets_cap(&t, 120000);
    expect_targets(&t, 0, 0);
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
    assert(LINE_CORNER_UM_S == t.right_um_s);
    assert(f.ref_deci == LINE_CAP_DECI);
}

/* The near end of a blind bridge holds heading, then a forward probe seeks
 * the edge. At the limit it turns on the spot toward the side that saw the line
 * last, then back the other way, and stops when both turns find nothing. */
static void test_gap_then_search_on_the_spot(void)
{
    line_follow_t f;
    line_targets_t t;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;
    int64_t travel = 0;
    uint32_t ms = 0U;
    int32_t heading = 0;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0, ms, &t); /* last seen on the right */
    (void)follow(&f, true, true, 0, 0, ms, &t);
    while (travel < ((int64_t)(LINE_GAP_MM - 5) * UM_PER_MM))
    {
        ms += 10U;
        travel += (CRUISE * 10) / 1000;
        status = follow(&f, false, false, 0, travel, ms, &t);
        assert(LINE_FOLLOW_BRIDGING == status);
        if (travel < (int64_t)LINE_PROBE_FROM_MM * UM_PER_MM)
        {
            expect_targets(&t, CRUISE, CRUISE);
        }
        else
        {
            assert(t.left_um_s == CRUISE && t.right_um_s > 0 && t.right_um_s < t.left_um_s);
        }
    }
    while (LINE_FOLLOW_SEARCHING != status)
    {
        ms += 10U;
        travel += (CRUISE * 10) / 1000;
        status = follow(&f, false, false, 0, travel, ms, &t);
    }
    /* First turn: clockwise, toward the right, on the spot. */
    expect_targets(&t, LINE_SEARCH_UM_S, -LINE_SEARCH_UM_S);
    while (heading > -LINE_SEARCH_SWEEP_DECI)
    {
        ms += 10U;
        heading -= 10;
        assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, heading, travel, ms, &t));
    }
    /* Second turn: back past the start, as far the other way. */
    ms += 10U;
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, heading, travel, ms, &t));
    expect_targets(&t, -LINE_SEARCH_UM_S, LINE_SEARCH_UM_S);
    while (heading < (LINE_SEARCH_SWEEP_DECI - 10))
    {
        ms += 10U;
        heading += 10;
        assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, heading, travel, ms, &t));
    }
    /* At the end of the second turn the search is not over: the car turns back
     * toward the heading it had before the loss, to retry. */
    ms += 10U;
    assert(LINE_FOLLOW_SEARCHING ==
           follow(&f, false, false, LINE_SEARCH_SWEEP_DECI, travel, ms, &t));
    assert(t.left_um_s > 0);
    assert(t.right_um_s < 0);
    assert(1U == f.retry);
}

/* A closed-loop stand-in for the car: the targets turn the heading and move the
 * travel over one pass of STEP_MS. */
static void plant(line_targets_t const * p_t, int32_t * p_heading, int64_t * p_travel)
{
    *p_heading += (int32_t)((((int64_t)p_t->right_um_s - p_t->left_um_s) * STEP_MS / 1000 *
                             DECI_PER_RAD) / TRACK_UM);
    *p_travel  += ((int64_t)p_t->left_um_s + p_t->right_um_s) / 2 * (int64_t)STEP_MS / 1000;
}

#define RETRY_PASS_MAX 20000

/* Lose the line after a hit on the right and run the closed loop until the
 * search is over. Each reverse drives about LINE_REVERSE_MM, and there are
 * LINE_RETRY_COUNT of them before the stop. */
static void test_failed_search_turns_back_reverses_and_searches_again(void)
{
    line_follow_t f;
    line_targets_t t;
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    uint32_t reverses = 0U;
    int64_t reverse_from_um = 0;
    bool b_reversing = false;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0, ms, &t);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    for (pass = 0; (pass < RETRY_PASS_MAX) && (LINE_FOLLOW_STOPPED != status); pass++)
    {
        bool const b_back = (t.left_um_s < 0) && (t.right_um_s < 0);

        ms += STEP_MS;
        status = follow(&f, false, false, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
        if ((t.left_um_s < 0) && (t.right_um_s < 0))
        {
            if (!b_reversing)
            {
                b_reversing = true;
                reverses++;
                reverse_from_um = travel;
                /* The car is back on the heading it had before the loss. */
                assert(abs(heading) < 100);
                assert(reverses == f.retry);
            }
            assert(LINE_FOLLOW_SEARCHING == status);
            assert(abs(heading) < 100);
        }
        else if (b_reversing || b_back)
        {
            int64_t const back_um = reverse_from_um - travel;

            b_reversing = false;
            assert(back_um >= ((int64_t)(LINE_REVERSE_MM - 10) * UM_PER_MM));
            assert(back_um <= ((int64_t)(LINE_REVERSE_MM + 10) * UM_PER_MM));
        }
        else
        {
            assert((LINE_FOLLOW_STOPPED == status) || (LINE_FOLLOW_SEARCHING == status) ||
                   (LINE_FOLLOW_BRIDGING == status));
        }
        if (LINE_FOLLOW_STOPPED != status)
        {
            assert(reverses <= LINE_RETRY_COUNT);
        }
    }
    /* Stopped only after the last retry, and then for good. */
    assert(LINE_FOLLOW_STOPPED == status);
    assert(LINE_RETRY_COUNT == reverses);
    assert(LINE_RETRY_COUNT == f.retry);
    for (pass = 0; pass < 200; pass++)
    {
        ms += STEP_MS;
        assert(LINE_FOLLOW_STOPPED == follow(&f, false, false, heading, travel, ms, &t));
        expect_targets(&t, 0, 0);
    }
    /* A sensor hit ends the stop. */
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, false, heading, travel, ms + STEP_MS, &t));
}

/* Run the closed loop until the retry count reaches want_retry; returns the
 * pass state in the arguments. */
static void run_to_retry(line_follow_t * p_f, line_targets_t * p_t, int32_t * p_heading,
                         int64_t * p_travel, uint32_t * p_ms, uint8_t want_retry)
{
    int pass;

    for (pass = 0; (pass < RETRY_PASS_MAX) && (p_f->retry < want_retry); pass++)
    {
        *p_ms += STEP_MS;
        (void)follow(p_f, false, false, *p_heading, *p_travel, *p_ms, p_t);
        plant(p_t, p_heading, p_travel);
    }
    assert(p_f->retry == want_retry);
}

/* A hit while the car turns back or reverses ends the recovery. */
static void test_hit_during_retry_resumes_following(void)
{
    uint32_t reverse_hit;

    for (reverse_hit = 0U; reverse_hit < 2U; reverse_hit++)
    {
        line_follow_t f;
        line_targets_t t;
        int64_t travel = 0;
        int32_t heading = 0;
        uint32_t ms = 0U;
        int pass = 0;

        line_follow_init(&f);
        (void)follow(&f, true, false, 0, 0, ms, &t);
        (void)follow(&f, true, true, 0, 0, ms, &t);
        while (LINE_FOLLOW_SEARCHING != f.status)
        {
            ms += STEP_MS;
            travel += 1000;
            (void)follow(&f, false, false, heading, travel, ms, &t);
        }
        run_to_retry(&f, &t, &heading, &travel, &ms, 1U);
        if (0U != reverse_hit)
        {
            while ((t.left_um_s >= 0) || (t.right_um_s >= 0))
            {
                ms += STEP_MS;
                (void)follow(&f, false, false, heading, travel, ms, &t);
                plant(&t, &heading, &travel);
                assert(++pass < RETRY_PASS_MAX);
            }
            for (pass = 0; pass < 5; pass++)
            {
                ms += STEP_MS;
                (void)follow(&f, false, false, heading, travel, ms, &t);
                plant(&t, &heading, &travel);
            }
            assert((t.left_um_s < 0) && (t.right_um_s < 0));
        }
        ms += STEP_MS;
        assert(LINE_FOLLOW_FOLLOWING == follow(&f, false, true, heading, travel, ms, &t));
        assert(t.left_um_s > 0);
        assert(t.right_um_s > 0);
        ms += STEP_MS;
        assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, heading, travel, ms, &t));
    }
}

/* A search turn that finds the line gives the next loss all its retries. A
 * reverse that finds only the line the car came on does not. */
static void test_retry_count_clears_on_a_search_hit(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0, ms, &t);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    while (LINE_FOLLOW_SEARCHING != f.status)
    {
        ms += STEP_MS;
        travel += 1000;
        (void)follow(&f, false, false, heading, travel, ms, &t);
    }
    run_to_retry(&f, &t, &heading, &travel, &ms, 1U);
    while ((t.left_um_s >= 0) || (t.right_um_s >= 0))
    {
        ms += STEP_MS;
        (void)follow(&f, false, false, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
    }
    for (pass = 0; pass < 100; pass++)
    {
        ms += STEP_MS;
        (void)follow(&f, false, false, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
    }
    /* Found while reversing. */
    ms += STEP_MS;
    (void)follow(&f, true, false, heading, travel, ms, &t);
    assert(1U == f.retry);
    /* Lost again: the second search is not a new loss. Found by a search turn. */
    while (LINE_FOLLOW_SEARCHING != f.status)
    {
        ms += STEP_MS;
        travel += 1000;
        (void)follow(&f, false, false, heading, travel, ms, &t);
    }
    for (pass = 0; (pass < 10) && (1U == f.retry); pass++)
    {
        ms += STEP_MS;
        (void)follow(&f, false, false, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
    }
    ms += STEP_MS;
    (void)follow(&f, true, false, heading, travel, ms, &t);
    assert(0U == f.retry);
}

/* Drive on the line at 1 mm a pass, both sensors on it, for mm millimetres. */
static void drive_on_line(line_follow_t * p_f, line_targets_t * p_t, int32_t heading,
                          int64_t * p_travel, uint32_t * p_ms, int32_t mm)
{
    int32_t i;

    for (i = 0; i < mm; i++)
    {
        *p_ms += STEP_MS;
        *p_travel += UM_PER_MM;
        (void)follow(p_f, true, true, heading, *p_travel, *p_ms, p_t);
    }
}

/* Lose the line after some travel; returns the travel where the search began.
 * One retry follows, and a hit comes after the car reversed for reverse_mm. */
static int64_t lose_retry_and_hit(line_follow_t * p_f, line_targets_t * p_t,
                                  int32_t * p_heading, int64_t * p_travel,
                                  uint32_t * p_ms, int32_t reverse_mm)
{
    int64_t loss_um;
    int pass = 0;

    while (LINE_FOLLOW_SEARCHING != p_f->status)
    {
        *p_ms += STEP_MS;
        *p_travel += UM_PER_MM;
        (void)follow(p_f, false, false, *p_heading, *p_travel, *p_ms, p_t);
    }
    loss_um = *p_travel;
    while ((0U == p_f->retry) || (p_t->left_um_s >= 0) || (p_t->right_um_s >= 0))
    {
        *p_ms += STEP_MS;
        (void)follow(p_f, false, false, *p_heading, *p_travel, *p_ms, p_t);
        plant(p_t, p_heading, p_travel);
        assert(++pass < RETRY_PASS_MAX);
    }
    while ((loss_um - *p_travel) < ((int64_t)reverse_mm * UM_PER_MM))
    {
        *p_ms += STEP_MS;
        (void)follow(p_f, false, false, *p_heading, *p_travel, *p_ms, p_t);
        plant(p_t, p_heading, p_travel);
        assert(++pass < RETRY_PASS_MAX);
    }
    *p_ms += STEP_MS;
    assert(LINE_FOLLOW_FOLLOWING ==
           follow(p_f, true, true, *p_heading, *p_travel, *p_ms, p_t));
    return loss_um;
}

/* The car follows on past the point where it lost the line: the next loss is a
 * new one. */
static void test_retry_clears_past_the_loss_point(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    int64_t loss_um;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    loss_um = lose_retry_and_hit(&f, &t, &heading, &travel, &ms, 60);
    assert(1U == f.retry);
    while ((travel - loss_um) < (15 * UM_PER_MM))
    {
        ms += STEP_MS;
        travel += UM_PER_MM;
        (void)follow(&f, true, true, heading, travel, ms, &t);
        if (travel <= loss_um)
        {
            assert(1U == f.retry);
        }
    }
    assert(0U == f.retry);
}

/* Four humps in one run: each loss is recovered by a reverse and followed on
 * past its loss point, so the fourth has all its retries. */
static void test_each_loss_in_a_run_has_all_retries(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    uint32_t loss;
    uint32_t reverses = 0U;
    uint8_t last_retry = 0U;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    for (loss = 0U; loss < 4U; loss++)
    {
        int64_t loss_um;

        drive_on_line(&f, &t, heading, &travel, &ms, 20);
        assert(0U == f.retry);
        loss_um = lose_retry_and_hit(&f, &t, &heading, &travel, &ms, 60);
        assert(1U == f.retry);
        while ((travel - loss_um) < (15 * UM_PER_MM))
        {
            drive_on_line(&f, &t, heading, &travel, &ms, 1);
        }
        assert(0U == f.retry);
    }
    /* The fourth loss is not recovered: all retries are used before the stop. */
    drive_on_line(&f, &t, heading, &travel, &ms, 20);
    for (pass = 0; (pass < RETRY_PASS_MAX) && (LINE_FOLLOW_STOPPED != f.status); pass++)
    {
        ms += STEP_MS;
        (void)follow(&f, false, false, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
        if (f.retry != last_retry)
        {
            reverses++;
            last_retry = f.retry;
        }
    }
    assert(LINE_FOLLOW_STOPPED == f.status);
    assert(LINE_RETRY_COUNT == reverses);
}

/* A dead end: the reverse finds the line, but it ends before the loss point.
 * The retries are not cleared and the car stops after the last. */
static void test_dead_end_stops_after_all_retries(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    uint32_t reverses = 0U;
    uint8_t last_retry = 0U;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    drive_on_line(&f, &t, heading, &travel, &ms, 20);
    (void)lose_retry_and_hit(&f, &t, &heading, &travel, &ms, 60);
    last_retry = f.retry;
    reverses = 1U;
    for (pass = 0; (pass < RETRY_PASS_MAX) && (LINE_FOLLOW_STOPPED != f.status); pass++)
    {
        bool const b_on = pass < 10;

        ms += STEP_MS;
        (void)follow(&f, b_on, b_on, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
        if (b_on)
        {
            travel += UM_PER_MM;
        }
        assert(f.retry >= last_retry);
        if (f.retry != last_retry)
        {
            reverses++;
            last_retry = f.retry;
        }
    }
    assert(LINE_FOLLOW_STOPPED == f.status);
    assert(LINE_RETRY_COUNT == f.retry);
    assert(LINE_RETRY_COUNT == reverses);
}

/* A reverse that does not move (the car is against something) ends as a failed
 * retry after LINE_REVERSE_STALL_MS: the search turns again, and the retries
 * run out in a stop. */
static void test_blocked_reverse_ends_as_a_failed_retry(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    uint32_t reverses = 0U;
    uint32_t reverse_passes = 0U;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0, ms, &t);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    for (pass = 0; (pass < RETRY_PASS_MAX) && (LINE_FOLLOW_STOPPED != f.status);
         pass++)
    {
        bool b_back;

        ms += STEP_MS;
        (void)follow(&f, false, false, heading, travel, ms, &t);
        b_back = (t.left_um_s < 0) && (t.right_um_s < 0);
        if (b_back)
        {
            reverse_passes++;
            reverses += (1U == reverse_passes) ? 1U : 0U;
            /* The wheels do not turn: no travel, no heading. */
            assert(reverse_passes <= ((LINE_REVERSE_STALL_MS / STEP_MS) + 2U));
        }
        else
        {
            reverse_passes = 0U;
            plant(&t, &heading, &travel);
        }
    }
    assert(LINE_FOLLOW_STOPPED == f.status);
    assert(LINE_RETRY_COUNT == reverses);
    assert(LINE_RETRY_COUNT == f.retry);
}

/* One wheel is jammed in the reverse and the other creeps back at
 * JAM_CREEP_UM_S whatever its target: the car pivots on the jammed wheel and
 * the mean travel grows. That is no progress of the jammed wheel: the reverse
 * ends as a failed retry after LINE_REVERSE_STALL_MS all the same. The wheels
 * are kept in um, as the car reads them. */
#define JAM_CREEP_UM_S (-20000)

static void test_one_wheel_jammed_reverse_ends_as_a_failed_retry(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t left_um = 0;
    int64_t right_um = 0;
    uint32_t ms = 0U;
    uint32_t reverses = 0U;
    uint32_t reverse_passes = 0U;
    int pass;

    line_follow_init(&f);
    (void)follow(&f, false, true, 0, 0, ms, &t);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    for (pass = 0; (pass < RETRY_PASS_MAX) && (LINE_FOLLOW_STOPPED != f.status);
         pass++)
    {
        bool b_back;

        ms += STEP_MS;
        (void)follow(&f, false, false, line_heading_deci(left_um, right_um),
                     (left_um + right_um) / 2, ms, &t);
        /* A steering correction can bring one reverse target to zero. */
        b_back = (t.left_um_s <= 0) && (t.right_um_s <= 0) &&
                 ((t.left_um_s < 0) || (t.right_um_s < 0));
        if (b_back)
        {
            reverse_passes++;
            reverses += (1U == reverse_passes) ? 1U : 0U;
            assert(reverse_passes <= ((LINE_REVERSE_STALL_MS / STEP_MS) + 2U));
        }
        else
        {
            reverse_passes = 0U;
            left_um += ((int64_t)t.left_um_s * STEP_MS) / 1000;
        }
        right_um += ((int64_t)(b_back ? JAM_CREEP_UM_S : t.right_um_s) *
                     STEP_MS) / 1000;
    }
    assert(LINE_FOLLOW_STOPPED == f.status);
    assert(LINE_RETRY_COUNT == reverses);
}

/* Lose the line with the search beginning at loss_um, recover by the reverse,
 * and follow on: the retry count clears within one cm past the loss point, and
 * not before it. */
static void expect_retry_clears_past(int64_t loss_um)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = loss_um - ((int64_t)LINE_GAP_MM * UM_PER_MM);
    int32_t heading = 0;
    uint32_t ms = 0U;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, travel, ms, &t);
    assert(loss_um == lose_retry_and_hit(&f, &t, &heading, &travel, &ms, 60));
    assert(1U == f.retry);
    while ((0U != f.retry) && ((travel - loss_um) < (50 * UM_PER_MM)))
    {
        ms += STEP_MS;
        travel += UM_PER_MM;
        (void)follow(&f, true, true, heading, travel, ms, &t);
        if (travel <= loss_um)
        {
            assert(1U == f.retry);
        }
    }
    assert(0U == f.retry);
    assert(travel > loss_um);
    assert((travel - loss_um) <= (11 * UM_PER_MM));
}

/* The loss point is a cm count modulo 256: it works across the wrap, and for
 * negative travel (the car reversed past where the run began). */
static void test_retry_clears_across_the_cm_wrap_and_below_zero(void)
{
    expect_retry_clears_past(-5 * (int64_t)UM_PER_MM);
    expect_retry_clears_past(-1234 * (int64_t)UM_PER_MM);
    expect_retry_clears_past(2535 * (int64_t)UM_PER_MM);
    expect_retry_clears_past(2565 * (int64_t)UM_PER_MM);
    expect_retry_clears_past(5125 * (int64_t)UM_PER_MM);
}

/* A hit during the reverse restarts from the reverse speed: the wheels do not
 * step from backward at LINE_SEARCH_UM_S to forward at cruise. */
static void test_hit_during_reverse_starts_from_the_reverse_speed(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    int32_t heading = 0;
    uint32_t ms = 0U;
    int pass = 0;
    int64_t const rise =
        ((int64_t)LINE_ACCEL_MM_S2 * UM_PER_MM * STEP_MS) / 1000;

    line_follow_init(&f);
    (void)follow(&f, true, false, 0, 0, ms, &t);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    while (LINE_FOLLOW_SEARCHING != f.status)
    {
        ms += STEP_MS;
        travel += 1000;
        (void)follow(&f, false, false, heading, travel, ms, &t);
    }
    run_to_retry(&f, &t, &heading, &travel, &ms, 1U);
    while ((t.left_um_s >= 0) || (t.right_um_s >= 0))
    {
        ms += STEP_MS;
        (void)follow(&f, false, false, heading, travel, ms, &t);
        plant(&t, &heading, &travel);
        assert(++pass < RETRY_PASS_MAX);
    }
    ms += STEP_MS;
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, true, heading, travel, ms, &t));
    assert(t.left_um_s > 0);
    assert(t.left_um_s <= (LINE_SEARCH_UM_S + rise));
    assert(t.right_um_s <= (LINE_SEARCH_UM_S + rise));
}

/* Blind travel at 2 mm a pass; returns the travel where SEARCHING begins. */
static int64_t blind_to_search_mm(bool b_hump)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel = 0;
    uint32_t ms = 0U;

    line_follow_init(&f);
    line_follow_set_hump(&f, b_hump);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    while ((LINE_FOLLOW_SEARCHING != f.status) && (travel < (1000 * (int64_t)UM_PER_MM)))
    {
        ms += STEP_MS;
        travel += 2 * UM_PER_MM;
        (void)follow(&f, false, false, 0, travel, ms, &t);
    }

    return travel / UM_PER_MM;
}

/* On a hump the sensors lift off the floor: the blind limit is longer. */
static void test_hump_bridges_longer(void)
{
    line_follow_t f;
    line_targets_t t;

    assert((blind_to_search_mm(false) >= LINE_GAP_MM) &&
           (blind_to_search_mm(false) < (LINE_GAP_MM + 4)));
    assert((blind_to_search_mm(true) >= LINE_HUMP_GAP_MM) &&
           (blind_to_search_mm(true) < (LINE_HUMP_GAP_MM + 4)));

    /* 300 mm blind: a hump bridges it, flat ground searches. */
    line_follow_init(&f);
    line_follow_set_hump(&f, true);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 0, 300 * (int64_t)UM_PER_MM, 10U, &t));
    line_follow_set_hump(&f, false);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 300 * (int64_t)UM_PER_MM, 20U, &t));
}

static void gap_limit(bool on_hump, int64_t limit_um)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    line_follow_set_hump(&f, on_hump);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    assert(LINE_FOLLOW_BRIDGING == follow(&f, false, false, 0, limit_um - 1, 10U, &t));
    assert(t.left_um_s > 0 && t.right_um_s > 0);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, limit_um, 20U, &t));
}

static void test_flat_gap_limit(void)
{
    gap_limit(false, 150000);
}

static void test_hump_gap_limit(void)
{
    gap_limit(true, 400000);
}

static void test_stuck_hump_flag_still_searches(void)
{
    line_follow_t f;
    line_targets_t t;
    int64_t travel_um;
    uint32_t now_ms = 0U;
    line_follow_status_t status;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    for (travel_um = 1000; travel_um <= 400000; travel_um += 1000)
    {
        line_follow_set_hump(&f, true);
        now_ms += 10U;
        status = follow(&f, false, false, 0, travel_um, now_ms, &t);
        if (LINE_FOLLOW_SEARCHING == status)
        {
            break;
        }
    }
    assert(travel_um <= 400000);
    assert(t.left_um_s == -t.right_um_s && t.left_um_s != 0);
}

/* The speed climbs at LINE_ACCEL_MM_S2 however short the pass is. */
static uint32_t ramp_ms(uint32_t dt_ms)
{
    line_follow_t f;
    line_targets_t t;
    uint32_t ms = 0U;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, ms, &t);
    f.speed_q    = (uint16_t)(LINE_CORNER_UM_S / LINE_SPEED_STEP_UM_S);
    t.left_um_s  = 0;
    while ((t.left_um_s < CRUISE) && (ms < 1000U))
    {
        ms += dt_ms;
        (void)follow(&f, true, true, 0, 0, ms, &t);
    }

    return ms;
}

static void test_ramp_at_small_dt(void)
{
    uint32_t const ref_ms = ramp_ms(10U);

    assert(ref_ms < 1000U);
    assert(ramp_ms(1U) <= (ref_ms + 10U));
    assert(ramp_ms(2U) <= (ref_ms + 10U));
}

/* A turn or straight ends with line_follow_set_side(); the blind travel of
 * the follower counts from there, not from the last hit before the turn. */
static void test_blind_travel_counts_from_the_takeover(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    (void)follow(&f, true, true, 0, 0, 0U, &t);
    line_follow_set_side(&f, LINE_SIDE_LEFT);
    assert(LINE_FOLLOW_BRIDGING ==
           follow(&f, false, false, 0, 300 * (int64_t)UM_PER_MM, 10U, &t));
}

/* A hit during the search: the car follows from where it points, at the
 * corner speed, not from the end of the search turn. */
static void test_search_hit_follows_from_the_heading(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    line_follow_start_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 0, 0U, &t));
    expect_targets(&t, -LINE_SEARCH_UM_S, LINE_SEARCH_UM_S); /* no side: left first */
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 300, 0, 10U, &t));
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, true, 300, 0, 20U, &t));
    assert(300 == f.ref_deci);
    expect_targets(&t, LINE_CORNER_UM_S + (LINE_ACCEL_MM_S2 * 10),
                   LINE_CORNER_UM_S + (LINE_ACCEL_MM_S2 * 10));
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
    assert(f.ref_deci > 0 && f.ref_deci < 40); /* 20 ms of slew, not 4 billion */
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

static void test_local_egress_search(void)
{
    line_follow_t f;
    line_targets_t t;

    /* Directional encoder progress remains valid through a slow 12 s
     * sweep. Left/right encoder noise with no net approach cannot renew it. */
    line_follow_init(&f);
    line_follow_start_local_search(&f);
    int32_t heading = 0;
    int32_t min_heading = 0;
    int32_t max_heading = 0;
    unsigned polls = 0U;
    for (; polls <= 60U; polls++)
    {
        line_follow_status_t const status = follow(&f, false, false, heading, 1230,
                                                    UINT32_MAX - 200U + polls * 200U, &t);
        if (status == LINE_FOLLOW_STOPPED)
        {
            break;
        }
        assert(t.left_um_s == -t.right_um_s && t.right_um_s != 0);
        heading += t.right_um_s > 0 ? 20 : -20;
        min_heading = heading < min_heading ? heading : min_heading;
        max_heading = heading > max_heading ? heading : max_heading;
    }
    assert(polls == 60U && heading == 0 && min_heading == -300 && max_heading == 300);
    expect_targets(&t, 0, 0);

    /* A discontinuous heading across the return origin is a stop, with no
     * entitlement to continue on the remaining obstacle contour. */
    line_follow_init(&f);
    line_follow_start_local_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 0, 0U, &t));
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 300, 0, 100U, &t));
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, -300, 0, 200U, &t));
    assert(LINE_FOLLOW_STOPPED == follow(&f, false, false, 250, 0, 300U, &t));
    assert(!line_follow_local_missed(&f));
    expect_targets(&t, 0, 0);
    line_follow_start_local_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 900, 0, 400U, &t));
    assert(LINE_FOLLOW_STOPPED == follow(&f, true, false, 1400, 0, 500U, &t));
    assert(!line_follow_local_missed(&f));
    expect_targets(&t, 0, 0);

    line_follow_init(&f);
    line_follow_start_local_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 90, 1230, 37U, &t));
    for (uint32_t ms = 100U; ms < LINE_LOCAL_SEARCH_STALL_MS; ms += 100U)
    {
        int32_t const jitter = (ms % 200U == 0U) ? 20 : -20;
        assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 90 + jitter, 1230, 37U + ms, &t));
    }
    assert(LINE_FOLLOW_STOPPED == follow(&f, true, false, 90, 1230,
                                        37U + LINE_LOCAL_SEARCH_STALL_MS, &t));
    expect_targets(&t, 0, 0);
    assert(line_follow_local_search(&f) && !line_follow_local_missed(&f));
    line_follow_start_local_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 90, 1230, 45U, &t));
    line_follow_stop_local_search(&f);
    assert(!line_follow_local_missed(&f));
    assert(LINE_FOLLOW_STOPPED == follow(&f, false, true, 90, 1230, 55U, &t));
    expect_targets(&t, 0, 0);
    line_follow_start_local_search(&f);
    assert(LINE_FOLLOW_FOLLOWING == follow(&f, true, false, 90, 1230, 65U, &t));
    assert(!line_follow_local_search(&f));
    assert(f.progress.stripe.begin_mm[0] == 0U && f.progress.stripe.begin_mm[1] == 0U);
}

/* After an aborted turn the car looks for the line on the spot and does not
 * stop. */
static void test_start_search(void)
{
    line_follow_t f;
    line_targets_t t;

    line_follow_init(&f);
    line_follow_start_search(&f);
    assert(LINE_FOLLOW_SEARCHING == follow(&f, false, false, 0, 0, 0U, &t));
    assert((t.left_um_s != 0) && (t.left_um_s == -t.right_um_s));
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

/* NONE is the line task's "no turn waiting" value: a turn started with it is
 * idle, so the first update stops both wheels and never pivots. */
static void test_none_turn_is_idle(void)
{
    line_turn_state_t tn;
    line_targets_t    t = {1, 1};

    line_turn_start(&tn, LINE_TURN_KIND_NONE, 0, 0);
    assert(LINE_SIDE_NONE == line_turn_side(&tn));
    assert(LINE_TURN_ABORTED == turn_step(&tn, 0, 0, true, false, false, &t));
    expect_targets(&t, 0, 0);
}

/* The turn kind is p1 of the TURN_START, TURN_DONE and TURN_ABORT events. */
static void test_turn_kind_wire_values(void)
{
    assert(0 == (int)LINE_TURN_KIND_NONE);
    assert(1 == (int)LINE_TURN_KIND_LEFT);
    assert(2 == (int)LINE_TURN_KIND_RIGHT);
    assert(3 == (int)LINE_TURN_KIND_UTURN);
    assert(4 == (int)LINE_TURN_KIND_STRAIGHT);
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
    assert(100000 == LINE_CORNER_UM_S);
    assert(80 == LINE_ACCEL_MM_S2);
    assert(LINE_CRUISE_UM_S <= 300000); /* MOTOR_MAX_TARGET_UM_S */
    assert(107 == CHASSIS_WHEEL_TRACK_MM);
    assert(25 == LINE_SENSOR_SPACING_MM);
    assert(8 == LINE_KH);
    assert(5355 == LINE_HIT_DECI_PER_M);
    assert(400 == LINE_RATE_RELAX);
    assert(20 == LINE_SLOW_DECI);
    assert(120 == LINE_CAP_DECI);
    assert(150 == LINE_GAP_MM);
    assert(90000 == LINE_SEARCH_UM_S);
    assert(1350 == LINE_SEARCH_SWEEP_DECI);
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
    assert(LINE_TURN_TURNING == turn_step(&tn, l, r, true, false, true, &t));
    right_turn_disp(LINE_TURN_HALF_DECI, &l, &r);
    assert(LINE_TURN_DONE == turn_step(&tn, l, r, true, false, false, &t));
    expect_targets(&t, 0, 0);
    assert(LINE_SIDE_RIGHT == line_turn_side(&tn));

    /* A U-turn aborts at 180 + margin, not at 90 + margin. */
    line_turn_start(&tn, LINE_TURN_KIND_UTURN, 1000, 2000);
    right_turn_disp(LINE_TURN_HALF_DECI - 20, &l, &r);
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

/* A transverse branch crosses each installed detector on that side. Only the
 * left side has a second optical channel, the barcode reader. */
static bool approach_stripe(line_route_t * route, line_turn_kind_t kind, bool left,
                            bool barcode, int width_mm, int departure_yaw)
{
    for (int mm = 0; mm <= 180; ++mm)
    {
        bool const stripe = mm <= width_mm;
        int64_t const travel = mm * UM_PER_MM;
        int64_t const half = mm > 60 ? pivot_half_um(departure_yaw) : 0;
        if (line_route_observe(route, kind, left && stripe, !left && stripe,
                               barcode && stripe, travel - half, travel + half))
        {
            assert(mm >= LINE_SENSOR_LEAD_MM);
            return true;
        }
    }
    return false;
}

static void test_junction_approach(void)
{
    line_route_t route = {0};
    assert(approach_stripe(&route, LINE_TURN_KIND_RIGHT, false, false, 15, 0));
    route = (line_route_t){0};
    assert(!approach_stripe(&route, LINE_TURN_KIND_RIGHT, false, false, 2, 0));
    route = (line_route_t){0};
    /* An 8 mm graze of the through tape occurs as the car unwinds a bend. */
    assert(!approach_stripe(&route, LINE_TURN_KIND_RIGHT, false, false, 8, 0));
    route = (line_route_t){0};
    assert(!approach_stripe(&route, LINE_TURN_KIND_RIGHT, false, false, 12, 0));
    route = (line_route_t){0};
    assert(!approach_stripe(&route, LINE_TURN_KIND_RIGHT, false, false, 40, 0));
    route = (line_route_t){0};
    assert(!approach_stripe(&route, LINE_TURN_KIND_RIGHT, false, false, 15, 100));
    route = (line_route_t){0};
    /* Even a candidate on the other side can be the only available exit. */
    assert(approach_stripe(&route, LINE_TURN_KIND_RIGHT, true, true, 15, 0));
    route = (line_route_t){0};
    assert(!approach_stripe(&route, LINE_TURN_KIND_LEFT, true, false, 15, 0));
}

static void test_transverse_tape_recovery(void)
{
    for (int side = 0; side < 2; ++side)
    {
        line_follow_t follow_state;
        line_targets_t targets;
        line_follow_init(&follow_state);
        bool const left = side == 0;
        for (int mm = 0; mm <= LINE_SENSOR_LEAD_MM + LINE_WIDTH_MM / 2 + 1; ++mm)
        {
            bool const tape = mm <= LINE_WIDTH_MM;
            bool const overlap = mm >= 7 && mm <= 11;
            (void)line_follow_update(&follow_state, tape && (left || overlap), tape && (!left || overlap), 0,
                                     mm * UM_PER_MM, (uint32_t)mm * STEP_MS, &targets);
            if (mm < LINE_SENSOR_LEAD_MM)
            {
                assert(targets.left_um_s >= 0 && targets.right_um_s >= 0);
            }
        }
        assert(follow_state.status == LINE_FOLLOW_SEARCHING);
        assert(left ? targets.left_um_s < 0 : targets.right_um_s < 0);
    }
}

static void test_marker_bridge_separates_tape_stripes(void)
{
    line_follow_t state;
    line_targets_t targets;
    line_follow_init(&state);
    for (int mm = 0; mm <= 165; ++mm)
    {
        bool const tape = mm < 8 || (mm >= 22 && mm < 37);
        if (!tape && mm < 50)
        {
            (void)line_follow_marker_sample(&state, false, false, false, 0U, 0, mm * UM_PER_MM, (uint32_t)mm * STEP_MS, &targets);
        }
        else
        {
            (void)line_follow_update(&state, tape, false, 0, mm * UM_PER_MM,
                                     (uint32_t)mm * STEP_MS, &targets);
        }
        if (mm < 22 + LINE_SENSOR_LEAD_MM)
        {
            assert(state.status != LINE_FOLLOW_SEARCHING);
        }
    }
    assert(state.status == LINE_FOLLOW_SEARCHING && targets.left_um_s < 0);
}

static void test_corner_stripe_crosses_both_detectors(void)
{
    line_follow_t state;
    line_targets_t targets;
    line_follow_init(&state);
    for (int mm = 0; mm <= 144; ++mm)
    {
        bool const left = mm < 12;
        bool const right = mm >= 3 && mm < 18;
        (void)line_follow_update(&state, left, right, 0, mm * UM_PER_MM,
                                 (uint32_t)mm * STEP_MS, &targets);
    }
    assert(state.status == LINE_FOLLOW_SEARCHING && targets.right_um_s < 0);
}

static void test_wide_left_crossing_needs_corroboration(void)
{
    for (int marker = 0; marker <= 1; ++marker)
    {
        line_follow_t state;
        line_targets_t targets;
        line_follow_init(&state);
        if (marker)
        {
            (void)line_follow_marker_sample(&state, false, false, false, 0U, 0, 0, 0U, &targets);
        }
        for (int mm = 1; mm < 145; ++mm)
        {
            (void)line_follow_update(&state, mm < 27, false, 0, mm * UM_PER_MM,
                                     (uint32_t)mm * STEP_MS, &targets);
        }
        assert(state.status == (marker ? LINE_FOLLOW_SEARCHING : LINE_FOLLOW_BRIDGING));
    }
}

static void test_deferred_route_keeps_corner_survey(void)
{
    line_follow_t state;
    line_targets_t targets;
    line_follow_init(&state);
    line_follow_set_route_candidate(&state, true);
    for (int mm = 0; mm <= LINE_SENSOR_LEAD_MM + LINE_WIDTH_MM; ++mm)
    {
        (void)line_follow_update(&state, mm < LINE_WIDTH_MM, false, 0, mm * UM_PER_MM,
                                 (uint32_t)mm * STEP_MS, &targets);
    }
    assert(state.status == LINE_FOLLOW_BRIDGING && targets.left_um_s > 0 && targets.right_um_s > 0);
    line_follow_set_route_candidate(&state, false);
    (void)line_follow_update(&state, false, false, 0, (LINE_SENSOR_LEAD_MM + LINE_WIDTH_MM + 1) * UM_PER_MM,
                             1500U, &targets);
    assert(state.status == LINE_FOLLOW_SEARCHING && targets.left_um_s < 0);
}

static void test_transverse_stripe_with_overlapping_left_readers(void)
{
    line_route_t route = {0};
    for (int mm = 0; mm <= 180; ++mm)
    {
        int64_t const travel = mm * UM_PER_MM;
        /* The left detector grazes approach tape before it crosses the T.
         * The right detector records the transverse 15 mm stripe alone. */
        if (line_route_observe(&route, LINE_TURN_KIND_RIGHT, mm < 29,
                               mm >= 12 && mm < 27, mm < 17, travel, travel))
        {
            assert(mm >= 148 && mm <= 151);
            return;
        }
    }
    assert(false);
}

/* A front sensor crosses each edge of the 15 mm tape as the car pivots.
 * The tape can lie between the two fixed detectors when centered. */
static bool route_edge(int angle)
{
    return abs(angle) >= 30 && abs(angle) <= 90;
}

static line_turn_status_t probe_at(line_route_t * route, int yaw, bool tape,
                                   line_targets_t * targets)
{
    int64_t const pivot = pivot_half_um(abs(yaw)) * (yaw < 0 ? -1 : 1);
    return line_route_probe(route, -pivot, pivot, tape, targets);
}

static void test_junction_sector_limits(void)
{
    int const edges[] = {599, 600, 1199, 1200};
    for (int sign = -1; sign <= 1; sign += 2)
    {
        for (size_t index = 0; index < sizeof(edges) / sizeof(edges[0]); ++index)
        {
            int const alt = edges[index];
            line_route_t route = {0};
            line_targets_t targets;
            bool const left = sign > 0;
            assert(approach_stripe(&route, LINE_TURN_KIND_STRAIGHT, left, left, 15, 0));
            assert(probe_at(&route, -150 * sign, false, &targets) == LINE_TURN_TURNING);
            assert(probe_at(&route, 150 * sign, true, &targets) == LINE_TURN_TURNING);
            assert(probe_at(&route, 151 * sign, false, &targets) == LINE_TURN_TURNING);
            line_turn_status_t const status = probe_at(&route, alt * sign, true, &targets);
            bool const inside = alt == 600 || alt == 1199;
            assert(status == (inside ? LINE_TURN_BRANCH_SELECTED : LINE_TURN_TURNING));
        }
    }
}

static void check_junction_sweep(line_turn_kind_t kind, bool left, bool through,
                                 bool branch, bool opposite, bool solid, bool lose_through)
{
    line_route_t route = {0};
    line_targets_t targets;
    int const sign = left ? 1 : -1;
    int yaw = 0;
    unsigned selected = 0U;
    line_turn_status_t status = LINE_TURN_TURNING;
    assert(approach_stripe(&route, kind, left, left, 15, 0));
    for (unsigned step = 0; step < 600; ++step)
    {
        bool const through_tape = through && (!lose_through || selected == 0U) && route_edge(yaw);
        bool const tape = solid || through_tape || (branch && route_edge(yaw - sign * 900)) ||
                          (opposite && route_edge(yaw + sign * 900));
        status = probe_at(&route, yaw, tape, &targets);
        if (status == LINE_TURN_BRANCH_SELECTED)
        {
            assert(selected == 0U && abs(yaw) >= 600);
            ++selected;
        }
        else if (status != LINE_TURN_TURNING)
        {
            break;
        }
        assert(targets.left_um_s == -targets.right_um_s && targets.left_um_s != 0);
        yaw += targets.right_um_s > 0 ? 10 : -10;
    }
    expect_targets(&targets, 0, 0);
    bool const straight = kind == LINE_TURN_KIND_STRAIGHT;
    bool const requested_exit = straight ? through : branch;
    bool const another_exit = straight ? (branch || opposite) : (through || opposite);
    if (solid || (!through && !branch && !opposite))
    {
        assert(selected == 0U && status == LINE_TURN_NO_BRANCH && abs(yaw) <= 10);
    }
    else if (!requested_exit || !another_exit)
    {
        assert(selected == 0U && status == LINE_TURN_FALLBACK_DONE);
        if (through)
        {
            assert(abs(yaw) < 30);
        }
        else
        {
            assert((branch && abs(yaw - sign * 900) < 30) || (opposite && abs(yaw + sign * 900) < 30));
        }
    }
    else if (lose_through)
    {
        assert(selected == 1U && status == LINE_TURN_ABORTED);
    }
    else
    {
        int const expected = kind == LINE_TURN_KIND_STRAIGHT ? 0 : sign * 900;
        assert(selected == 1U && status == LINE_TURN_DONE && abs(yaw - expected) < 30);
    }
}

static void test_junction_sweeps(void)
{
    for (int side = 0; side < 2; ++side)
    {
        bool const left = side == 0;
        line_turn_kind_t const kind = left ? LINE_TURN_KIND_LEFT : LINE_TURN_KIND_RIGHT;
        check_junction_sweep(kind, left, true, true, false, false, false);
        check_junction_sweep(kind, left, false, true, false, false, false);
        check_junction_sweep(kind, left, true, false, false, false, false);
        check_junction_sweep(kind, left, true, true, false, true, false);
        check_junction_sweep(kind, left, false, true, true, false, false);
        check_junction_sweep(kind, left, false, false, true, false, false);
        check_junction_sweep(kind, left, false, false, false, false, false);
        check_junction_sweep(LINE_TURN_KIND_STRAIGHT, left, true, true, false, false, false);
        check_junction_sweep(LINE_TURN_KIND_STRAIGHT, left, true, true, false, false, true);
        check_junction_sweep(LINE_TURN_KIND_STRAIGHT, left, true, false, true, false, false);
        check_junction_sweep(LINE_TURN_KIND_STRAIGHT, left, false, true, true, false, false);
    }
}

int main(int argc, char * argv[])
{
    static const struct
    {
        char const * name;
        void (*test)(void);
    } selectable[] = {
        {"corner-wheel-demands", test_corner_wheel_demands},
        {"junction-approach", test_junction_approach},
        {"transverse-tape-recovery", test_transverse_tape_recovery},
        {"marker-separates-stripes", test_marker_bridge_separates_tape_stripes},
        {"cross-detector-overlap", test_corner_stripe_crosses_both_detectors},
        {"wide-corroborated-corner", test_wide_left_crossing_needs_corroboration},
        {"deferred-corner-survey", test_deferred_route_keeps_corner_survey},
        {"junction-overlap", test_transverse_stripe_with_overlapping_left_readers},
        {"junction-sweeps", test_junction_sweeps},
        {"junction-sector-limits", test_junction_sector_limits},
        {"flat-gap-limit", test_flat_gap_limit},
        {"hump-gap-limit", test_hump_gap_limit},
        {"stuck-hump-flag", test_stuck_hump_flag_still_searches},
        {"cleared-hump-flag", test_hump_bridges_longer},
        {"reverse-retry", test_failed_search_turns_back_reverses_and_searches_again},
        {"search-hit", test_retry_count_clears_on_a_search_hit},
        {"local-egress-search", test_local_egress_search},
        {"dead-end", test_dead_end_stops_after_all_retries},
        {"blocked-reverse", test_blocked_reverse_ends_as_a_failed_retry},
        {"one-wheel-jammed", test_one_wheel_jammed_reverse_ends_as_a_failed_retry},
        {"hit-while-reversing", test_hit_during_reverse_starts_from_the_reverse_speed},
    };
    size_t i;

    if (2 == argc)
    {
        for (i = 0U; i < (sizeof(selectable) / sizeof(selectable[0])); ++i)
        {
            if (0 == strcmp(argv[1], selectable[i].name))
            {
                selectable[i].test();
                return 0;
            }
        }
    }
    bool const direct = (argc == 2) && (strcmp(argv[1], "unit") == 0);
    if ((1 != argc) && !direct)
    {
        (void)fprintf(stderr, "usage: %s [case]\n", argv[0]);
        return 2;
    }

    test_local_egress_search();
    test_junction_approach();
    test_transverse_tape_recovery();
    test_marker_bridge_separates_tape_stripes();
    test_corner_stripe_crosses_both_detectors();
    test_wide_left_crossing_needs_corroboration();
    test_deferred_route_keeps_corner_survey();
    test_transverse_stripe_with_overlapping_left_readers();
    test_junction_sweeps();
    test_junction_sector_limits();
    test_corner_wheel_demands();
    test_init();
    test_one_sensor_steers_toward_the_line();
    test_cap_limits_the_correction();
    test_speed_follows_heading_error();
    test_blind_relaxes_toward_heading();
    test_blind_heading_reanchors();
    test_bumper_cap();
    test_targets_cap();
    test_gap_then_search_on_the_spot();
    test_failed_search_turns_back_reverses_and_searches_again();
    test_hit_during_retry_resumes_following();
    test_retry_count_clears_on_a_search_hit();
    test_retry_clears_past_the_loss_point();
    test_each_loss_in_a_run_has_all_retries();
    test_dead_end_stops_after_all_retries();
    test_blocked_reverse_ends_as_a_failed_retry();
    test_one_wheel_jammed_reverse_ends_as_a_failed_retry();
    test_retry_clears_across_the_cm_wrap_and_below_zero();
    test_hit_during_reverse_starts_from_the_reverse_speed();
    test_hump_bridges_longer();
    test_flat_gap_limit();
    test_hump_gap_limit();
    test_stuck_hump_flag_still_searches();
    test_ramp_at_small_dt();
    test_blind_travel_counts_from_the_takeover();
    test_search_hit_follows_from_the_heading();
    test_targets_never_exceed_cruise();
    test_time_wraps();
    test_heading_helper();
    test_start_search();
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
    test_none_turn_is_idle();
    test_turn_kind_wire_values();
    test_straight();
    test_constants();
    (void)puts("line_control: passed");
    return 0;
}
