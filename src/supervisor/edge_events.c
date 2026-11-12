/*
 *	edge_events.c
 *	See edge_events.h.
 */

#include "edge_events.h"

#include "run_state.h"
#include "telemetry/wire_schema.h"

void edge_init (edge_watch_t * p_watch)
{
    p_watch->obstacle_state  = (uint8_t) TELEMETRY_OBSTACLE_STATE_STOPPED;
    p_watch->b_plan_reported = false;
    p_watch->b_hump_active   = false;
    p_watch->b_running       = false;
    p_watch->b_started       = false;
}

static uint8_t put (edge_event_t * p_out, uint8_t count, uint8_t type, int16_t p1, uint16_t p2)
{
    p_out[count].type = type;
    p_out[count].p1   = p1;
    p_out[count].p2   = p2;

    return (uint8_t) (count + 1U);
}

uint8_t edge_obstacle (edge_watch_t * p_watch, const edge_obstacle_in_t * p_in,
                       edge_event_t p_out[EDGE_EVENTS_MAX])
{
    uint8_t const prev  = p_watch->obstacle_state;
    uint8_t const state = p_in->state;
    uint8_t       count = 0U;

    if (state != prev)
    {
        if ((TELEMETRY_OBSTACLE_STATE_WAIT_STOP == state) && (TELEMETRY_OBSTACLE_STATE_MONITOR == prev))
        {
            p_watch->b_plan_reported = false;
            count = put (p_out, count, (uint8_t) TELEMETRY_EVT_OBSTACLE_SEEN, (int16_t) p_in->distance_mm, 0U);
        }
        if ((TELEMETRY_OBSTACLE_STATE_STOPPED == state) && (prev != TELEMETRY_OBSTACLE_STATE_STOPPED))
        {
            count = put (p_out, count, (uint8_t) TELEMETRY_EVT_OBSTACLE_STOP, (int16_t) p_in->cause,
                         (uint16_t) p_in->echo_class);
        }
        if ((TELEMETRY_OBSTACLE_STATE_BYPASS == state) && (TELEMETRY_OBSTACLE_STATE_BYPASS != prev) &&
            (TELEMETRY_OBSTACLE_STATE_REACQUIRE != prev))
        {
            count = put (p_out, count, (uint8_t) TELEMETRY_EVT_BYPASS_START, (int16_t) p_in->plan_side,
                         (uint16_t) p_in->plan_start_deg);
        }
        if ((TELEMETRY_OBSTACLE_STATE_MONITOR == state) &&
            ((TELEMETRY_OBSTACLE_STATE_REACQUIRE == prev) || (TELEMETRY_OBSTACLE_STATE_BYPASS == prev)))
        {
            count = put (p_out, count, (uint8_t) TELEMETRY_EVT_BYPASS_END, 0, 0U);
        }
    }
    /* The scan result is reported once per scan, when the plan exists. */
    if (((TELEMETRY_OBSTACLE_STATE_PLAN == state) || (TELEMETRY_OBSTACLE_STATE_BYPASS == state)) &&
        !p_watch->b_plan_reported && (count < EDGE_EVENTS_MAX))
    {
        p_watch->b_plan_reported = true;
        count = put (p_out, count, (uint8_t) TELEMETRY_EVT_SCAN_DONE, (int16_t) p_in->plan_status,
                     p_in->plan_width_mm);
    }
    if (TELEMETRY_OBSTACLE_STATE_MONITOR == state)
    {
        p_watch->b_plan_reported = false;
    }
    p_watch->obstacle_state = state;

    return count;
}

uint8_t edge_hump (edge_watch_t * p_watch, bool b_active, uint16_t peak_mm, bool b_height_valid,
                   edge_event_t p_out[EDGE_EVENTS_MAX])
{
    uint8_t count = 0U;

    if (b_active && !p_watch->b_hump_active)
    {
        count = put (p_out, count, (uint8_t) TELEMETRY_EVT_HUMP_START, 0, 0U);
    }
    else if (!b_active && p_watch->b_hump_active)
    {
        count = put (p_out, count, (uint8_t) TELEMETRY_EVT_HUMP_END, (int16_t) peak_mm,
                     b_height_valid ? 1U : 0U);
    }
    else
    {
        /* No change. */
    }
    p_watch->b_hump_active = b_active;

    return count;
}

uint8_t edge_run (edge_watch_t * p_watch, uint8_t run_state, edge_event_t p_out[EDGE_EVENTS_MAX])
{
    bool const b_running = (RUN_STATE_RUNNING == run_state);
    uint8_t    count     = 0U;

    if (b_running && !p_watch->b_running)
    {
        count = put (p_out, count, (uint8_t) TELEMETRY_EVT_RUN_START, 0, 0U);
    }
    else if (!b_running && p_watch->b_running)
    {
        count = put (p_out, count, (uint8_t) TELEMETRY_EVT_RUN_STOP, (int16_t) run_state, 0U);
    }
    else
    {
        /* No change. */
    }
    p_watch->b_running = b_running;

    return count;
}
