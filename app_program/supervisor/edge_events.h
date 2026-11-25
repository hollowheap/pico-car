/*
 *	edge_events.h
 *	Maps run and obstacle state changes to telemetry events. Pure C.
 *
 *	run_start / run_stop    the run state enters or leaves RUNNING
 *	obstacle_seen           the obstacle task starts to stop the car
 *	scan_done               the scan finished and a plan exists
 *	bypass_start            the car starts to drive round the obstacle
 *	bypass_end              the car is back on the line
 *	obstacle_stop           the obstacle task stopped the car with a cause
 */

#ifndef SUPERVISOR_EDGE_EVENTS_H
#define SUPERVISOR_EDGE_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/* The most events one call can return. */
#define EDGE_EVENTS_MAX (3U)

typedef struct
{
    int16_t  p1;
    uint16_t p2;
    uint8_t  type;  /* TELEMETRY_EVT_*              */
} edge_event_t;

STATIC_ASSERT(sizeof(edge_event_t) == 6U, "edge_event_t size");

typedef struct
{
    uint8_t obstacle_state;
    bool    b_plan_reported : 1;
    bool    b_running : 1;
    bool    b_started : 1;   /* the first call only records the state */
} edge_watch_t;

STATIC_ASSERT(sizeof(edge_watch_t) == 2U, "edge_watch_t size");

typedef struct
{
    uint16_t distance_mm;
    uint16_t plan_width_mm;
    uint8_t  state;          /* obstacle state of the wire schema   */
    uint8_t  cause;
    uint8_t  echo_class;
    uint8_t  plan_status;
    uint8_t  plan_side;
    uint8_t  plan_start_deg;
} edge_obstacle_in_t;

STATIC_ASSERT(sizeof(edge_obstacle_in_t) == 10U, "edge_obstacle_in_t size");

void edge_init (edge_watch_t * p_watch);

/* Returns how many events were written to p_out. */
uint8_t edge_obstacle (edge_watch_t * p_watch, const edge_obstacle_in_t * p_in,
                       edge_event_t p_out[EDGE_EVENTS_MAX]);

/* run_state is RUN_STATE_* of run_state.h; the stop event carries the new state. */
uint8_t edge_run (edge_watch_t * p_watch, uint8_t run_state, edge_event_t p_out[EDGE_EVENTS_MAX]);

#endif /* SUPERVISOR_EDGE_EVENTS_H */
