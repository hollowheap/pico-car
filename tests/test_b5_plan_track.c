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

    check(b5_plan_track_near(100, 100), "inside the swing circle");
    check(!b5_plan_track_near(120, 100), "outside the swing circle");

    if (0U == failures)
    {
        (void)printf("test_b5_plan_track: all passed\n");
    }

    return ((0U == failures) ? 0 : 1);
}
