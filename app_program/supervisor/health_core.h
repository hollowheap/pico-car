/*
 *	health_core.h
 *	The health checks of the supervisor. Pure C: no kernel call, so a
 *	host test drives them.
 *
 *	liveness   Each task kicks a bit every pass. A task that misses its bit
 *	           for LIVENESS_MISS_LIMIT checks is stalled.
 *	progress   A car that is told to drive but does not move is stalled.
 *	stuck      A line sensor that never changes while the car drives a long
 *	           way is stuck.
 */

#ifndef SUPERVISOR_HEALTH_CORE_H
#define SUPERVISOR_HEALTH_CORE_H

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"
#include "supervisor/health_drive.h"
#include "supervisor/health_ids.h"

/* A task that misses this many checks in a row is stalled. A check runs every
   100 ms, so 5 is 500 ms. [Default] */
#define LIVENESS_MISS_LIMIT (5U)

/* A required task that has not kicked once after this many checks is stalled
   too (5 s). [Default] */
#define LIVENESS_BOOT_CHECKS (50U)

/* A stall in these tasks revokes the motion permit. A stalled physics
   task can recover under its attitude hold before the board watchdog fires. */
#define LIVENESS_CRITICAL_MASK \
    ((1U << LIVE_MOTION) | (1U << LIVE_LINE) | (1U << LIVE_OBSTACLE))

/* The 16-bit liveness masks hold all LIVE_TASKS bits, and the critical tasks
   are among them. */
STATIC_ASSERT(LIVE_TASKS <= (sizeof(uint16_t) * CHAR_BIT), live_ids_fit_the_mask);
STATIC_ASSERT(LIVENESS_CRITICAL_MASK < (1UL << LIVE_TASKS), critical_ids_are_live_tasks);

typedef struct
{
    uint8_t  miss[LIVE_TASKS];
    uint16_t required; /* tasks that must kick                                */
    uint16_t armed;    /* tasks that kicked at least once                     */
    uint16_t stalled;  /* tasks that are stalled now                          */
    uint8_t  checks;   /* checks since start, saturated at LIVENESS_BOOT_CHECKS */
} liveness_t;

void liveness_init (liveness_t * p_live, uint16_t required_mask);

/* One check. taken is the set of bits kicked since the last check. p_new gets
   the tasks that became stalled in this check, p_back the tasks that
   recovered. */
void liveness_check (liveness_t * p_live, uint16_t taken, uint16_t * p_new,
                     uint16_t * p_back);

bool liveness_critical_stalled (const liveness_t * p_live);

/* True when motion_status_t reports a timing fault: the drive lease lapsed
   (EXPIRED) or the control loop missed its period (GAP). The controller
   clears them in place after its brake interval; a STOPPED car keeps them.
   state is MOTION_STATE_*, fault the wire code. */
bool health_motion_timing_fault (uint8_t state, uint8_t fault);

/* A timing fault that outlasts this window did not clear in place. It must
   outlast MOTION_FAULT_CLEAR_US (500 ms). [Default] */
#define MOTION_TIMING_RECOVERY_MS (1000U)
/* Timing faults that clear in place since boot before the next one is
   terminal. [Default] */
#define MOTION_TIMING_FAULTS_MAX (3U)

typedef struct
{
    uint32_t fault_ms;
    uint8_t  faults;
    bool     b_fault;
    bool     b_terminal;
} timing_watch_t;

void timing_watch_init (timing_watch_t * p_watch);

/* One supervisor pass. Returns true, and keeps returning true, once a timing
   fault is terminal: the supervisor then stops the permit and the watchdog
   feed, and the car reboots. */
bool timing_watch_update (timing_watch_t * p_watch, bool b_fault, uint32_t dt_ms);

/* Progress: the car is told to move but the wheels do not turn. */
#define PROGRESS_STALL_MS (3000U)
/* Travel below this (in mm, both wheels added) is no progress. */
#define PROGRESS_MIN_MM (2U)

typedef struct
{
    uint32_t last_mm;
    uint32_t still_ms;
    bool     b_tripped;
} progress_t;

void progress_init (progress_t * p_prog);

/* One check. b_expected is true when the car must be moving: running, no hold,
   speed targets not zero. travel_mm is the sum of both wheels' absolute travel.
   Returns true once, when the check trips. */
bool progress_update (progress_t * p_prog, bool b_expected, uint32_t travel_mm,
                      uint32_t dt_ms);

/* Stuck sensor: the line sensors did not change over this much travel. */
#define STUCK_TRAVEL_MM (1500U)

typedef struct
{
    uint32_t edges;
    uint32_t anchor_mm;
    bool     b_tripped;
} stuck_t;

void stuck_init (stuck_t * p_stuck);

/* One check. edges counts sensor changes since boot. travel_mm is the mean wheel
   travel. Returns true once, when the sensors did not change for
   STUCK_TRAVEL_MM of driving. b_active is false while the car is held. */
bool stuck_update (stuck_t * p_stuck, bool b_active, uint32_t edges, uint32_t travel_mm);

/* Physics publishes the applied request together with the calibrated sample.
 * Zero is the boot reference. Timeouts report a delayed reference. */
#define CALHOLD_MAX_PASSES (250U)  /* 5 s at the supervisor period */

typedef struct
{
    uint32_t reference_request;
    uint16_t passes;
    bool     b_waiting;
    bool     b_timed_out;
    bool     b_timeout_seen;
} calhold_t;

void calhold_start (calhold_t * p_hold, uint32_t reference_request);
bool calhold_update (calhold_t * p_hold, uint32_t applied_request, bool b_calibrated);

/* True on the first pass of an attempt that sees its timeout. Call it once per
 * pass, after calhold_update. */
bool calhold_timeout_edge (calhold_t * p_hold);

#endif /* SUPERVISOR_HEALTH_CORE_H */
