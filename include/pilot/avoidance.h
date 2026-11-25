/** @file avoidance.h
 * @brief Per-car state for obstacle avoidance and its stationary survey.
 */
#ifndef PILOT_AVOIDANCE_H
#define PILOT_AVOIDANCE_H

#include "pilot/hug.h"
#include "pilot/look.h"

/** Phase of the stuck check. A stall while following calls for a survey.
 *  A stall during an active hug can back off and retry. A back-off
 *  after a try in which one wheel alone showed no travel ends in ENCODER when
 *  that wheel shows none in the back-off either. */
typedef enum
{
    PILOT_PHASE_TRY = 0,
    PILOT_PHASE_RETRY,
    PILOT_PHASE_BACKOFF,
    PILOT_PHASE_BACKOFF_ONE_WHEEL
} pilot_phase_t;

STATIC_ASSERT(sizeof(pilot_phase_t) <= 4U, "pilot_phase_t size");

typedef struct
{
    hug_t         hug;
    uint32_t      course_scan_ms;
    uint32_t      course_left_um; /* low wheel totals at the stationary survey origin */
    uint32_t      course_right_um;
    look_t        look;          /* sonar rays at the current survey pose */
    uint16_t      exit_width_mm; /* measured before giving up the active hug frame */
    uint16_t      exit_breadth_mm;
    uint8_t       recovery_phase; /* pilot_phase_t */
    uint8_t       course_scan_step;
    uint8_t       course_scan_seq;
    uint8_t       course_clear_count;
    uint8_t       course_scan_attempts;
    int8_t        course_side; /* provisional surveyed side: left +1, right -1, unknown 0 */
    bool          b_course_anchor : 1;
    bool          b_course_hold : 1;
    bool          b_course_stuck : 1; /* stall recovery needs operator clearance */
} avoidance_t;

STATIC_ASSERT(sizeof(avoidance_t) == 420U, "avoidance_t size");

#endif /* PILOT_AVOIDANCE_H */
