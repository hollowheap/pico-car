/** @file test_route_demo.c
 * @brief Distance/revolution endpoints, independent stops and terminal faults.
 */
#include <assert.h>
#include <stddef.h>
#include "route_demo.h"

static route_demo_t route;
static motion_sample_t sample;
static uint32_t now;
static const motion_scale_t scales[2] = {
    {1000U, 10U, -1, false}, {1000U, 10U, 1, false}
};

static void initialise (void)
{
    now = UINT32_MAX - 10000U;
    sample = (motion_sample_t){0};
    sample.elapsed_us = 20000U;
    sample.wheel[0].speed_valid = true;
    sample.wheel[1].speed_valid = true;
    sample.wheel[0].odometry_valid = true;
    sample.wheel[1].odometry_valid = true;
    assert(route_demo_start(&route, scales, now));
}

static void tick (int64_t left, int64_t right)
{
    now += 20000U;
    sample.timestamp_us = now;
    sample.wheel[0].net_edges = left;
    sample.wheel[1].net_edges = right;
    route_demo_update(&route, &sample, now, false);
}

static void targets (int32_t left, int32_t right)
{
    int32_t actual_left;
    int32_t actual_right;

    route_demo_targets(&route, &actual_left, &actual_right);
    assert(actual_left == left && actual_right == right);
}

static void assert_aborted (route_demo_error_t error)
{
    assert(route.phase == ROUTE_DEMO_ABORTED && route.error == error);
    targets(0, 0);
    tick(999999, 999999);
    assert(route.phase == ROUTE_DEMO_ABORTED && route.error == error);
}

int main (void)
{
    unsigned int i;
    motion_scale_t bad[2] = {{0U, 10U, -1, false}, {1000U, 10U, 1, false}};

    initialise();
    assert(route.goals[0][0] == 5000 && route.goals[2][1] == 1000);
    assert(route.goals[1][0] == 30 && route.goals[1][1] == 0);
    targets(50000, 50000);
    tick(4999, 4000);
    targets(50000, 50000);
    tick(5000, 4999);
    targets(0, 50000);
    tick(4998, 4999); /* Rollback cannot restart an already completed wheel. */
    targets(0, 50000);
    tick(5004, 5001);
    assert(route.phase == ROUTE_DEMO_PAUSE_FIRST);
    targets(0, 0);
    for (i = 0U; i < 19U; i++)
    {
        tick(5006, 5003);
        assert(route.phase == ROUTE_DEMO_PAUSE_FIRST);
        targets(0, 0);
    }
    tick(5006, 5003);
    assert(route.phase == ROUTE_DEMO_RIGHT);
    targets(50000, 0);
    tick(5035, 5003);
    assert(route.phase == ROUTE_DEMO_RIGHT);
    tick(5036, 5003);
    assert(route.phase == ROUTE_DEMO_PAUSE_RIGHT);
    targets(0, 0);
    for (i = 0U; i < 20U; i++)
    {
        tick(5037, 5004);
    }
    assert(route.phase == ROUTE_DEMO_LAST);
    targets(50000, 50000);
    tick(6037, 6003);
    targets(0, 50000);
    tick(6037, 6004);
    assert(route.phase == ROUTE_DEMO_DONE);
    targets(0, 0);
    tick(0, 0);
    assert(route.phase == ROUTE_DEMO_DONE); /* No automatic restart. */

    initialise();
    now += 1U;
    route_demo_update(&route, NULL, now, true);
    assert_aborted(ROUTE_DEMO_STOP);
    initialise();
    sample.wheel[1].odometry_valid = false;
    tick(1, 1);
    assert_aborted(ROUTE_DEMO_QUALITY);
    initialise();
    sample.wheel[0].speed_valid = false;
    tick(1, 1);
    assert_aborted(ROUTE_DEMO_QUALITY);
    initialise();
    sample.wheel[0].invalid_delta = 1U;
    tick(1, 1);
    assert_aborted(ROUTE_DEMO_QUALITY);
    initialise();
    tick(5000, 5000);
    sample.wheel[1].odometry_valid = false;
    tick(5000, 5000); /* A pause cannot conceal distance-quality failure. */
    assert_aborted(ROUTE_DEMO_QUALITY);
    initialise();
    sample.timestamp_us = now;
    route_demo_update(&route, &sample, now, false);
    assert_aborted(ROUTE_DEMO_QUALITY); /* Duplicate baseline isn't feedback. */
    initialise();
    now += ROUTE_DEMO_MAX_GAP_US + 1U;
    route_demo_update(&route, &sample, now, false);
    assert_aborted(ROUTE_DEMO_TIMING);
    initialise();
    now += 80000U;
    sample.timestamp_us = now - 60001U;
    route_demo_update(&route, &sample, now, false);
    assert_aborted(ROUTE_DEMO_QUALITY);
    initialise();
    sample.elapsed_us = ROUTE_DEMO_MAX_GAP_US + 1U;
    tick(1, 1);
    assert_aborted(ROUTE_DEMO_QUALITY);
    initialise();
    for (i = 0U; i < 1250U; i++)
    {
        tick(0, 0);
    }
    assert_aborted(ROUTE_DEMO_TIMEOUT);
    initialise();
    now += 20000U;
    sample.timestamp_us = now;
    sample.wheel[0].net_edges = 5000;
    sample.wheel[1].net_edges = 5000;
    route_demo_update(&route, &sample, now, true);
    assert_aborted(ROUTE_DEMO_STOP); /* Stop wins over endpoint completion. */
    initialise();
    route.baseline[0] = INT64_MIN;
    tick(1, 1);
    assert_aborted(ROUTE_DEMO_RANGE);
    assert(!route_demo_start(NULL, scales, 0U));
    assert(!route_demo_start(&route, NULL, 0U));
    assert(!route_demo_start(&route, bad, 0U));
    route_demo_update(NULL, NULL, 0U, true);
    route_demo_targets(NULL, NULL, NULL);
    {
        const motion_scale_t car[2] = {{210487U, 2541U, -1, false}, {210487U, 2530U, 1, false}};
        assert(route_demo_start(&route, car, 0U));
        assert(route.goals[1][0] == 7623);
        assert((uint64_t)route.goals[0][0] * 210487U >= UINT64_C(500000) * 2541U);
        assert((uint64_t)(route.goals[0][0] - 1) * 210487U < UINT64_C(500000) * 2541U);
        assert((uint64_t)route.goals[2][1] * 210487U >= UINT64_C(100000) * 2530U);
    }
    return 0;
}
