/** @file test_b5_plan_track.c
 * @brief Known-answer tests for the Buddy 5 chicken-head rotation scan
 *        geometry (b5_plan_track_point, _blocked, _near).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "b5_plan.h"

/* Allowed error of a computed position, mm. */
#define TOL_MM (1)

static unsigned int failures;

static void check (bool condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

static bool near_mm (int32_t got, int32_t want)
{
    int32_t const diff = got - want;

    return ((diff <= TOL_MM) && (diff >= -TOL_MM));
}

static void check_point (int32_t psi, int32_t gaze, uint32_t range,
                         int32_t want_fwd, int32_t want_side,
                         const char * p_name)
{
    int32_t fwd  = 0;
    int32_t side = 0;

    b5_plan_track_point(psi, gaze, range, &fwd, &side);

    if (!(near_mm(fwd, want_fwd) && near_mm(side, want_side)))
    {
        (void)fprintf(stderr, "  got forward %ld side %ld\n", (long)fwd,
                      (long)side);
    }

    check(near_mm(fwd, want_fwd) && near_mm(side, want_side), p_name);
}

int main (void)
{
    check_point(0, 0, 150U, 234, 0, "square on: 69 + 15 + 150 ahead");
    check_point(-400, 0, 150U, 218, -44, "car turned 40 deg right");
    check_point(-600, -100, 150U, 197, -88, "turned 60 right, gaze 10 right");

    check(b5_plan_track_blocked(218, -44, 300U), "point in the lane");
    check(!b5_plan_track_blocked(197, -95, 300U), "point beside the lane");
    check(!b5_plan_track_blocked(350, 0, 300U), "point past the lookahead");

    /* Asymmetric lane: 138 mm on the left (IR protrusion), 93 on the right. */
    check(b5_plan_track_blocked(200, 130, 300U), "left point in the wider lane");
    check(!b5_plan_track_blocked(200, -100, 300U), "right point beside the lane");
    check(b5_plan_track_gap_mm(100) == 38, "left gap 138 - 100");
    check(b5_plan_track_gap_mm(-80) == 13, "right gap 93 - 80");

    /* Swing circle 172 mm (front of the left protrusion). */
    check(b5_plan_track_near(130, 100), "inside the swing circle");
    check(!b5_plan_track_near(140, 110), "outside the swing circle");

    /* Bypass legs: first leg keeps the axle 192 mm from the face. */
    check(b5_plan_ahead_mm(205U) == 82, "first leg cut short for B 205");
    check(b5_plan_ahead_mm(400U) == 200, "first leg B / 2 for B 400");
    check(b5_plan_ahead_mm(100U) == -23, "back up 23 mm for B 100");
    check(b5_plan_out_mm(50U) == 242U, "E = A + 192");
    check(b5_plan_level_mm(205U, 82) == 235U, "level 215 + 20 short");

    if (0U == failures)
    {
        (void)printf("test_b5_plan_track: all passed\n");
    }

    return ((0U == failures) ? 0 : 1);
}
