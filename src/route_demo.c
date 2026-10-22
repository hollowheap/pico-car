/** @file route_demo.c
 * @brief Bounded50cm / right-left-wheel3revs /10cm route policy.
 */
#include <stddef.h>
#include "route_demo.h"

static void abort_route (route_demo_t * p_route, route_demo_error_t error)
{
    p_route->phase = ROUTE_DEMO_ABORTED;
    p_route->error = error;
}

static unsigned int leg (route_demo_phase_t phase)
{
    return phase == ROUTE_DEMO_RIGHT ? 1U : (phase == ROUTE_DEMO_LAST ? 2U : 0U);
}

bool route_demo_start (route_demo_t * p_route,
    const motion_scale_t scales[ENCODER_WHEELS], uint32_t baseline_us)
{
    route_demo_t route = {0};
    unsigned int wheel;

    if (p_route == NULL || scales == NULL)
    {
        return false;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        uint64_t revolution = scales[wheel].edges_per_revolution;
        uint64_t travel = scales[wheel].wheel_travel_um;

        if (travel == 0U || revolution == 0U)
        {
            return false;
        }
        /* Ceiling keeps a fractional edge from terminating the leg early. */
        route.goals[0][wheel] = (int64_t)((UINT64_C(500000) * revolution + travel - 1U) / travel);
        route.goals[1][wheel] = wheel == 0U ? (int64_t)(UINT64_C(3) * revolution) : 0;
        route.goals[2][wheel] = (int64_t)((UINT64_C(100000) * revolution + travel - 1U) / travel);
    }
    route.started_us = baseline_us;
    route.phase_us = baseline_us;
    route.previous_us = baseline_us;
    route.last_sample_us = baseline_us;
    route.initialised = true;
    *p_route = route;
    return true;
}

void route_demo_update (route_demo_t * p_route, const motion_sample_t * p_sample,
                        uint32_t now_us, bool stop_requested)
{
    unsigned int wheel;
    uint32_t elapsed;

    if (p_route == NULL || !p_route->initialised ||
        p_route->phase == ROUTE_DEMO_DONE || p_route->phase == ROUTE_DEMO_ABORTED)
    {
        return;
    }
    if (stop_requested)
    {
        abort_route(p_route, ROUTE_DEMO_STOP);
        return;
    }
    if ((uint32_t)(now_us - p_route->previous_us) > ROUTE_DEMO_MAX_GAP_US)
    {
        abort_route(p_route, ROUTE_DEMO_TIMING);
        return;
    }
    p_route->previous_us = now_us;
    if ((uint32_t)(now_us - p_route->started_us) >= ROUTE_DEMO_TOTAL_US)
    {
        abort_route(p_route, ROUTE_DEMO_TIMEOUT);
        return;
    }
    if (p_sample == NULL || p_sample->elapsed_us == 0U ||
        p_sample->elapsed_us > ROUTE_DEMO_MAX_GAP_US ||
        !motion_sample_fresh(p_sample, now_us, ROUTE_DEMO_MAX_AGE_US) ||
        p_sample->timestamp_us == p_route->last_sample_us)
    {
        abort_route(p_route, ROUTE_DEMO_QUALITY);
        return;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if (!p_sample->wheel[wheel].speed_valid || !p_sample->wheel[wheel].odometry_valid ||
            p_sample->wheel[wheel].invalid_delta != 0U)
        {
            abort_route(p_route, ROUTE_DEMO_QUALITY);
            return;
        }
    }
    p_route->last_sample_us = p_sample->timestamp_us;
    elapsed = now_us - p_route->phase_us;
    if (p_route->phase == ROUTE_DEMO_PAUSE_FIRST || p_route->phase == ROUTE_DEMO_PAUSE_RIGHT)
    {
        if (elapsed >= ROUTE_DEMO_PAUSE_US)
        {
            p_route->phase = p_route->phase == ROUTE_DEMO_PAUSE_FIRST ?
                ROUTE_DEMO_RIGHT : ROUTE_DEMO_LAST;
            p_route->phase_us = now_us;
            for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
            {
                p_route->baseline[wheel] = p_sample->wheel[wheel].net_edges;
                p_route->progress[wheel] = 0;
                p_route->reached[wheel] = p_route->goals[leg(p_route->phase)][wheel] == 0;
            }
        }
        return;
    }
    if (elapsed >= (p_route->phase == ROUTE_DEMO_FIRST ? UINT32_C(25000000) :
                    (p_route->phase == ROUTE_DEMO_RIGHT ? UINT32_C(30000000) : UINT32_C(10000000))))
    {
        abort_route(p_route, ROUTE_DEMO_TIMEOUT);
        return;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t value = p_sample->wheel[wheel].net_edges;
        int64_t base = p_route->baseline[wheel];

        if ((base > 0 && value < INT64_MIN + base) ||
            (base < 0 && value > INT64_MAX + base))
        {
            abort_route(p_route, ROUTE_DEMO_RANGE);
            return;
        }
        p_route->progress[wheel] = value - base;
        p_route->reached[wheel] = p_route->reached[wheel] ||
            p_route->progress[wheel] >= p_route->goals[leg(p_route->phase)][wheel];
    }
    if (p_route->reached[0] && p_route->reached[1])
    {
        p_route->phase = p_route->phase == ROUTE_DEMO_FIRST ? ROUTE_DEMO_PAUSE_FIRST :
            (p_route->phase == ROUTE_DEMO_RIGHT ? ROUTE_DEMO_PAUSE_RIGHT : ROUTE_DEMO_DONE);
        p_route->phase_us = now_us;
    }
}

void route_demo_targets (const route_demo_t * p_route,
                         int32_t * p_left, int32_t * p_right)
{
    bool driving = p_route != NULL && p_route->initialised &&
        (p_route->phase == ROUTE_DEMO_FIRST || p_route->phase == ROUTE_DEMO_RIGHT ||
         p_route->phase == ROUTE_DEMO_LAST);

    if (p_left != NULL)
    {
        *p_left = driving && !p_route->reached[0] ? ROUTE_DEMO_SPEED_UM_S : 0;
    }
    if (p_right != NULL)
    {
        *p_right = driving && p_route->phase != ROUTE_DEMO_RIGHT &&
            !p_route->reached[1] ? ROUTE_DEMO_SPEED_UM_S : 0;
    }
}
