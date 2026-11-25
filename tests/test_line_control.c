/** @file test_line_control.c
 * @brief Host tests for the line follower and one-shot turns.
 *
 * Includes a small differential-drive simulator (107 mm track, a straight
 * line, two sensors 25 mm apart ahead of the axle) for the 30 degree entry.
 */

#include <assert.h>
#include <math.h>
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
#define UM_PER_MM       1000
#define CRUISE          LINE_CRUISE_UM_S
#define TRACK_UM        ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
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

/* Neither sensor: the car holds its heading for LINE_GAP_MM at cruise. Then
 * it does not drive on: it turns on the spot toward the side that saw the line
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
        expect_targets(&t, CRUISE, CRUISE);
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
    double cross_x_mm;   /* a second line across the path at this x, 0 = none */
    bool   b_tee;        /* the cross line runs only to the left (a T) */
    struct sim_seg const * p_segs; /* a track of segments, NULL = the cases above */
} sim_case_t;

/* One piece of a segment track: a straight (radius 0) or an arc. turn_deg is
 * the arc angle, positive to the left. */
typedef struct sim_seg
{
    double len_mm;
    double radius;
    double turn_deg;
} sim_seg_t;

/* The segment track as points every SEG_STEP_MM, built once per run. */
#define SEG_STEP_MM  2.0
#define SEG_MAX_PTS  6000
static double g_seg_x[SEG_MAX_PTS];
static double g_seg_y[SEG_MAX_PTS];
static int    g_seg_n;

/* Lay out the segments from (-200, 0), heading along +x. A zero entry ends
 * the list. */
static void seg_build(sim_seg_t const * p_segs)
{
    double x = -200.0, y = 0.0, psi = 0.0;
    int i;

    g_seg_n = 0;
    for (i = 0; (p_segs[i].len_mm > 0.0) || (p_segs[i].turn_deg != 0.0); i++)
    {
        double const len = (p_segs[i].radius > 0.0)
                               ? (fabs(p_segs[i].turn_deg) * PI_RAD / 180.0) * p_segs[i].radius
                               : p_segs[i].len_mm;
        double const dpsi = (p_segs[i].radius > 0.0)
                                ? (p_segs[i].turn_deg * PI_RAD / 180.0) * (SEG_STEP_MM / len)
                                : 0.0;
        double done = 0.0;

        while ((done < len) && (g_seg_n < SEG_MAX_PTS))
        {
            g_seg_x[g_seg_n] = x;
            g_seg_y[g_seg_n] = y;
            g_seg_n++;
            x += SEG_STEP_MM * cos(psi + (dpsi / 2.0));
            y += SEG_STEP_MM * sin(psi + (dpsi / 2.0));
            psi += dpsi;
            done += SEG_STEP_MM;
        }
    }
}

static double seg_dist(double px, double py)
{
    double best = 1e9;
    int i;

    for (i = 0; i < g_seg_n; i++)
    {
        double const d = hypot(px - g_seg_x[i], py - g_seg_y[i]);

        if (d < best)
        {
            best = d;
        }
    }
    return best;
}

typedef struct
{
    double edges_hz;     /* sensor state changes per second after the settle */
    double heading_deg;  /* largest |heading| on a straight line             */
    double lost_pct;     /* time the sensor midpoint is off the line         */
    double max_off_mm;
    bool   b_stopped;    /* both targets zero at some pass                   */
    double gap_heading_deg;
    double mean_mm_s;    /* mean axle speed after the settle           */
    double sensor_x_max; /* largest x of the sensor midpoint                 */
    double end_off_mm;   /* sensor midpoint to the path at the end           */
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

    if (NULL != p_c->p_segs)
    {
        return seg_dist(px, py);
    }
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

    if (NULL != p_c->p_segs)
    {
        return 0.0; /* no gaps on a segment track */
    }
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

    if ((p_c->cross_x_mm > 0.0) && (fabs(sx - p_c->cross_x_mm) <= (LINE_WIDTH_MM / 2.0)) &&
        (!p_c->b_tee || (sy > 0.0)))
    {
        b_on = true;
    }
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
    sim_result_t res = {0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, -1e9, 0.0};
    double moved_mm = 0.0;
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
    /* A segment track is measured from the start: its bends come early. */
    double const settle = (NULL != p_c->p_segs) ? 0.5 : (p_c->seconds / 4.0);

    if (NULL != p_c->p_segs)
    {
        seg_build(p_c->p_segs);
    }
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
        psi += ((vr - vl) / CHASSIS_WHEEL_TRACK_MM) * SIM_DT_S;
        disp_l_um += vl * 1000.0 * SIM_DT_S;
        disp_r_um += vr * 1000.0 * SIM_DT_S;
        t += SIM_DT_S;
        if (t > settle)
        {
            moved_mm += ((vl + vr) / 2.0) * SIM_DT_S;
            double const mx = x + (LINE_SENSOR_LEAD_MM * cos(psi));
            double const my = y + (LINE_SENSOR_LEAD_MM * sin(psi));
            double const mid = fabs(path_dist(p_c, mx, my));
            double const pos = path_pos(p_c, mx, my);
            bool const b_in_gap = (p_c->gap_len_mm > 0.0) &&
                                  (pos > (p_c->gap_from_mm - 1.0)) &&
                                  (pos < (p_c->gap_from_mm + p_c->gap_len_mm + 60.0));

            samples++;
            res.end_off_mm = mid;
            if (mx > res.sensor_x_max)
            {
                res.sensor_x_max = mx;
            }
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
    res.mean_mm_s = moved_mm / (p_c->seconds - settle);
    res.lost_pct = (100.0 * (double)off) / (double)samples;
    return res;
}

/* The old law hunts on a straight line; the cascade hunts much less. This test
 * fails on the old law. */
static void test_sim_straight_hunting(void)
{
    sim_case_t c = {LAW_LEGACY, 0.0, 0.0, 0.0, 12.0, false, 0.0, false, NULL};
    sim_result_t legacy;
    sim_result_t cascade;

    legacy = sim_run(&c);
    c.law = LAW_CASCADE;
    cascade = sim_run(&c);
    (void)printf("straight: legacy %.1f edges/s (%.1f deg), cascade %.1f edges/s (%.1f deg, %.2f%% lost, max off %.1f mm, %.0f mm/s)\n",
                 legacy.edges_hz, legacy.heading_deg, cascade.edges_hz,
                 cascade.heading_deg, cascade.lost_pct, cascade.max_off_mm, cascade.mean_mm_s);
    assert(legacy.edges_hz > 6.0);
    /* Hunting per metre: 8 edges/s at the 163364 um/s cruise. */
    assert(cascade.edges_hz < (8.0 * (double)LINE_CRUISE_UM_S / 163364.0));
    assert(cascade.edges_hz < (legacy.edges_hz * 0.65));
    assert(cascade.heading_deg < 14.0);
    assert(!cascade.b_stopped);
    assert(cascade.lost_pct < 1.0);
    assert(cascade.max_off_mm < 25.0);
    /* The simulated sensors are points, so a centred car gets no hit and
     * searches on the spot after LINE_GAP_MM; the mean stays well under
     * cruise, but above 43% of it (130 mm/s at a 300 mm/s cruise). */
    assert(cascade.mean_mm_s > (0.43 * (double)LINE_CRUISE_UM_S / 1000.0));
}

static void test_sim_curves_keep_the_line(void)
{
    sim_case_t c = {LAW_CASCADE, 400.0, 0.0, 0.0, 12.0, false, 0.0, false, NULL};
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

/* Bends in a row (user request): a U of two 90 degree corners, then a corner
 * the other way; a 180 degree U in one arc, then a corner; an S. All arcs are
 * 150 mm, the track corner radius. */
static void test_sim_bends_in_a_row(void)
{
    static sim_seg_t const u_two[] = {
        {800.0, 0.0, 0.0}, {0.0, 150.0, 90.0}, {100.0, 0.0, 0.0}, {0.0, 150.0, 90.0},
        {300.0, 0.0, 0.0}, {0.0, 150.0, -90.0}, {4000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    static sim_seg_t const u_one[] = {
        {800.0, 0.0, 0.0}, {0.0, 150.0, 180.0}, {300.0, 0.0, 0.0}, {0.0, 150.0, -90.0},
        {4000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    static sim_seg_t const s_bend[] = {
        {800.0, 0.0, 0.0}, {0.0, 150.0, 90.0}, {0.0, 150.0, -90.0}, {4000.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};
    sim_seg_t const * const tracks[] = {u_two, u_one, s_bend};
    char const * const names[] = {"U of two corners, then right", "U in one arc, then right",
                                  "S bend"};
    unsigned i;

    for (i = 0U; i < 3U; i++)
    {
        sim_case_t c = {LAW_CASCADE, 0.0, 0.0, 0.0, 20.0, false, 0.0, false, tracks[i]};
        sim_result_t res = sim_run(&c);

        (void)printf("%s: %.2f%% lost, max off %.1f mm, %.0f mm/s\n", names[i], res.lost_pct,
                     res.max_off_mm, res.mean_mm_s);
        assert(res.lost_pct < 1.0);
        assert(res.max_off_mm < 25.0);
        assert(!res.b_stopped);
    }
}

/* A corner too sharp to follow, with a second line across the path 300 mm
 * past it (user report: after a miss the car took another line). The car
 * stops going forward, turns on the spot toward the side that saw the line
 * last, and finds its own line again. */
static void test_sim_lost_line_is_found_on_the_spot(void)
{
    static sim_seg_t const sharp[] = {
        {800.0, 0.0, 0.0}, {0.0, 10.0, 90.0}, {3000.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    /* The corner is at x = 600; the other line runs across at x = 900. */
    sim_case_t c = {LAW_CASCADE, 0.0, 0.0, 0.0, 12.0, false, 900.0, false, sharp};
    sim_result_t res = sim_run(&c);

    (void)printf("sharp corner, line across 300 mm past it: sensors reach x %.0f mm, end %.1f mm off\n",
                 res.sensor_x_max, res.end_off_mm);
    assert(res.sensor_x_max < (900.0 - LINE_WIDTH_MM));
    assert(res.end_off_mm < 25.0);
}

/* Junctions: a + crossing and a T branch to the left. With no barcode command
 * the car goes straight through both. */
static void test_sim_junctions_go_straight(void)
{
    sim_case_t c = {LAW_CASCADE, 0.0, 0.0, 0.0, 14.0, false, 900.0, false, NULL};
    sim_result_t res;

    res = sim_run(&c);
    (void)printf("junction +: %.2f%% lost, max off %.1f mm, max heading %.1f deg\n",
                 res.lost_pct, res.max_off_mm, res.heading_deg);
    assert(res.lost_pct < 1.0);
    assert(res.max_off_mm < 25.0);
    c.b_tee = true;
    res = sim_run(&c);
    (void)printf("junction T: %.2f%% lost, max off %.1f mm, max heading %.1f deg\n",
                 res.lost_pct, res.max_off_mm, res.heading_deg);
    assert(res.lost_pct < 1.0);
    assert(res.max_off_mm < 25.0);
}

/* A 60 mm cut in the line (the run-on-car check): the car crosses it. */
static void test_sim_crosses_a_gap(void)
{
    sim_case_t c = {LAW_CASCADE, 0.0, 800.0, 60.0, 14.0, false, 0.0, false, NULL};
    sim_result_t res;

    res = sim_run(&c);
    (void)printf("gap 60 mm: heading %.1f deg across, max off %.1f mm\n",
                 res.gap_heading_deg, res.max_off_mm);
    assert(!res.b_stopped);
    assert(res.lost_pct < 1.0);
    assert(res.gap_heading_deg < 12.0);
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
    assert(125000 == LINE_CRUISE_UM_S);
    assert(100000 == LINE_CORNER_UM_S);
    assert(400 == LINE_ACCEL_MM_S2);
    assert(LINE_CRUISE_UM_S <= 300000); /* MOTOR_MAX_TARGET_UM_S */
    assert(107 == CHASSIS_WHEEL_TRACK_MM);
    assert(25 == LINE_SENSOR_SPACING_MM);
    assert(8 == LINE_KH);
    assert(4000 == LINE_HIT_DECI_PER_M);
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

int main(int argc, char * argv[])
{
    static const struct
    {
        char const * name;
        void (*test)(void);
    } selectable[] = {
        {"flat-gap-limit", test_flat_gap_limit},
        {"hump-gap-limit", test_hump_gap_limit},
        {"stuck-hump-flag", test_stuck_hump_flag_still_searches},
        {"cleared-hump-flag", test_hump_bridges_longer},
        {"reverse-retry", test_failed_search_turns_back_reverses_and_searches_again},
        {"search-hit", test_retry_count_clears_on_a_search_hit},
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
    if (1 != argc)
    {
        (void)fprintf(stderr, "usage: %s [case]\n", argv[0]);
        return 2;
    }

    test_init();
    test_one_sensor_steers_toward_the_line();
    test_cap_limits_the_correction();
    test_speed_follows_heading_error();
    test_blind_relaxes_toward_heading();
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
    test_sim_straight_hunting();
    test_sim_curves_keep_the_line();
    test_sim_bends_in_a_row();
    test_sim_crosses_a_gap();
    test_sim_lost_line_is_found_on_the_spot();
    test_sim_junctions_go_straight();
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
    (void)puts("line_control: all tests passed");
    return 0;
}
