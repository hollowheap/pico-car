/*
 *	health_core.h
 *	The health checks of the supervisor (D160). Pure C: no kernel call, so a
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

#include <stdbool.h>
#include <stdint.h>

#include "supervisor/health_ids.h"

/* A task that misses this many checks in a row is stalled. A check runs every
   100 ms, so 5 is 500 ms. [Default] */
#define LIVENESS_MISS_LIMIT (5U)

/* A required task that has not kicked once after this many checks is stalled
   too (5 s). [Default] */
#define LIVENESS_BOOT_CHECKS (50U)

/* The tasks whose stall stops the car (and, with the watchdog on, resets the
   board). The others only raise the fault. */
#define LIVENESS_CRITICAL_MASK \
    ((1U << LIVE_MOTION) | (1U << LIVE_LINE) | (1U << LIVE_OBSTACLE))

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

/* Drive check (D161). It reads the wheel targets, speeds and duty.
 *
 *   crossed   The wheel told to go faster does not turn faster. The encoder
 *             plugs are swapped, or a wheel turns the wrong way.
 *   free      Both wheels turn on target with much less duty than they needed
 *             on the ground. The wheels run free: the car is in the air.
 */

/* Only wheels told to go at least this fast count (um/s). */
#define DRIVE_TARGET_MIN_UM_S (100000)
/* A wheel is on target when its speed is within this percent of the target. */
#define DRIVE_TRACK_PERCENT (10)
/* The two targets must differ this much for the crossed test (um/s). */
#define DRIVE_TARGET_GAP_UM_S (60000)
/* The two speeds must differ this much to count as a vote (um/s). */
#define DRIVE_SPEED_GAP_UM_S (20000)
/* The crossed test judges a window of this length, then starts again. */
#define DRIVE_WINDOW_MS (2000U)
/* Disagreeing votes needed in a window, and the ratio against agreeing votes. */
#define DRIVE_CROSS_MS (800U)
#define DRIVE_CROSS_RATIO (3U)
/* Grounded time needed to learn the duty per speed of each wheel. */
#define DRIVE_BASE_MS (2000U)
/* The baseline moves 1/16 of the way to each new sample. */
#define DRIVE_BASE_DIVISOR (16)
/* A wheel is free when its duty per speed is under 1/DRIVE_FREE_DIVISOR of the baseline. */
#define DRIVE_FREE_DIVISOR (2U)
/* Checks in a row before the cue sets (300 ms at 100 ms) and before it clears. */
#define DRIVE_FREE_CHECKS (3U)
#define DRIVE_CLEAR_CHECKS (2U)

/* Both wheels, left then right. */
#define DRIVE_WHEELS (2U)

typedef struct
{
    int16_t duty_pm[DRIVE_WHEELS];     /* signed duty, permille                */
    int32_t speed_um_s[DRIVE_WHEELS];  /* signed, positive car-forward         */
    int32_t target_um_s[DRIVE_WHEELS]; /* what the follower asked for          */
    bool    b_judge;        /* running, no hold, feedback fresh, car on the floor */
} drive_in_t;

typedef struct
{
    uint16_t agree_ms;
    uint16_t disagree_ms;
    uint16_t window_ms;
    uint32_t base[DRIVE_WHEELS]; /* duty permille per m/s of a grounded wheel; 0 = none */
    uint32_t base_ms;
    uint8_t  free_run;
    uint8_t  busy_run;
    bool     b_crossed;
    bool     b_free;
} drive_t;

void drive_init (drive_t * p_drive);

/* One check every dt_ms. Returns true once, when b_crossed turns on. */
bool drive_update (drive_t * p_drive, const drive_in_t * p_in, uint32_t dt_ms);

/* Encoder probe (D162). The bench drives one wheel alone and reads how far each
 * encoder moved, in mm with the car-forward sign. The verdict says whether the
 * encoder of the driven wheel answers. */
typedef enum
{
    DRIVE_PROBE_OK = 0,   /* the driven wheel moved forward, the other did not   */
    DRIVE_PROBE_DEAD,     /* neither encoder moved                                */
    DRIVE_PROBE_REVERSED, /* the driven encoder counted backward                  */
    DRIVE_PROBE_CROSSED,  /* the other encoder moved instead                      */
    DRIVE_PROBE_UNCLEAR   /* both moved, or too little to tell                    */
} drive_probe_t;

/* Travel of the driven encoder that counts as moving (mm). */
#define DRIVE_PROBE_MOVED_MM (15)
/* A wheel moved under 1/DRIVE_PROBE_DEAD_DIV of that is not moving. The other
   encoder counts as still when it moved under 1/DRIVE_PROBE_RATIO of the driven one. */
#define DRIVE_PROBE_DEAD_DIV (3)
#define DRIVE_PROBE_RATIO    (4)

drive_probe_t drive_probe_verdict (int32_t driven_mm, int32_t other_mm);

#endif /* SUPERVISOR_HEALTH_CORE_H */
