/** @file line_control.h
 * @brief Pure line-following and one-shot turn logic (decisions D92, D93).
 *
 * No kernel, no hardware and no floating point. The caller owns the clock,
 * reads the sensors and the wheel odometry, and sends the returned wheel
 * targets to the motion service.
 *
 * Units: wheel targets in um/s (positive = car forward), travel in um,
 * time in ms, headings in deci-degrees (1/10 degree, counter-clockwise
 * positive).
 *
 * The constants marked "tune on the car" are starting values, not measured
 * ones.
 */

#ifndef TRACK_LINE_CONTROL_H
#define TRACK_LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/**
 * Cruise speed of the follower on a straight, and the cap on every follower
 * target. It must not exceed MOTOR_MAX_TARGET_UM_S (300000).
 */
#define LINE_CRUISE_UM_S (300000)

/**
 * Corner speed. The speed falls in a straight line from cruise at no heading
 * error to LINE_CORNER_UM_S at LINE_SLOW_DECI of error and more, so a
 * correction slows the car at once. The speed then climbs back at most at
 * LINE_ACCEL_MM_S2, so the car is still slow when the next correction of a
 * corner comes (on the car, 120 mm/s and an instant climb missed the turn).
 */
#define LINE_CORNER_UM_S (100000)
#define LINE_SLOW_DECI   (20)
#define LINE_ACCEL_MM_S2 (400)

/**
 * Sonar bumper. With a ranged echo closer than LINE_BUMPER_FAR_MM the speed
 * cap falls in a straight line, to LINE_BUMPER_MIN_UM_S at LINE_BUMPER_NEAR_MM
 * and closer. NEAR is a copy of OBSTACLE_DETECT_MM, where the obstacle task
 * takes over, so the car arrives there slow enough to stop.
 */
#define LINE_BUMPER_FAR_MM   (600)
#define LINE_BUMPER_NEAR_MM  (250)
#define LINE_BUMPER_MIN_UM_S (120000)

/**
 * Wheel track in mm. Copy of OBSTACLE_WHEEL_TRACK_MM in
 * src/navigation/obstacle/obstacle_config.h (measured on the car). Keep the
 * two in step; this module must not include the obstacle header.
 */
#define LINE_WHEEL_TRACK_MM (107)

/**
 * Geometry of the sensors, measured on the car (2026-11-12): 25 mm between the
 * sensor midpoints, 130 mm from the axle to the sensors. The sensors are now
 * fixed at that spacing. The line is solid black and 15 mm wide, so a centred
 * car sees it with neither sensor. The track corners are 90 degree arcs of
 * 150 mm radius. The follower code does not read these; the host simulation
 * does.
 */
#define LINE_SENSOR_SPACING_MM (25)
#define LINE_SENSOR_LEAD_MM    (130)
#define LINE_WIDTH_MM          (15)

/**
 * Heading cascade (D181). The sensors move a heading target; the encoder
 * heading follows it. All values are starting values: tune on the car.
 *
 * LINE_KH            Yaw rate per heading error, in 1/s.
 * LINE_HIT_DECI_PER_M Target slew per metre of travel while one sensor sees
 *                    the line (deci-degrees per metre). Per metre, not per
 *                    second: at 163 mm/s a fixed 60 degrees per second could
 *                    not follow a 150 mm corner, and a faster car needs a
 *                    faster target. A 150 mm arc turns 382 degrees per
 *                    metre, so the slew must be faster than that, or a long
 *                    arc such as a U-turn outruns the target.
 * LINE_RATE_RELAX    Slew of the target back to the heading once the line is
 *                    between the sensors again (deci-degrees per second). The
 *                    car then stops turning where it points along the line,
 *                    in place of holding the correction and swinging across.
 * LINE_CAP_DECI      The target stays within this of the heading.
 * LINE_GAP_MM        Travel without a sensor hit that is still a crossing, a
 *                    gap, or the line between the sensors. The car holds its
 *                    heading for it. It is short: the search turns on the
 *                    spot, and the sensors, LINE_SENSOR_LEAD_MM ahead of the
 *                    axle, reach only a line that is that close.
 * LINE_SEARCH_UM_S   Wheel speed of the search. The car does not drive on: a
 *                    car that drove on blind found another line and took it.
 *                    It turns on the spot toward the side that saw the line
 *                    last, then back the other way, until a sensor sees the
 *                    line.
 * LINE_SEARCH_SWEEP_DECI How far the search turns each way from the heading
 *                    where it started. At the end of the second turn the car
 *                    stops (status STOPPED).
 */
#define LINE_KH             (8)
#define LINE_HIT_DECI_PER_M (4000)
#define LINE_RATE_RELAX     (400)
#define LINE_CAP_DECI       (120)
#define LINE_GAP_MM         (150)
#define LINE_SEARCH_UM_S    (90000)
#define LINE_SEARCH_SWEEP_DECI (1350)
/** A pass this much later than the last one counts as this long (ms). */
#define LINE_DT_MAX_MS      (50U)

/** Pivot speed of a one-shot turn before the capture window. */
#define LINE_TURN_FAST_UM_S (180000)
/** Pivot speed inside the capture window. Tune on the car. */
#define LINE_TURN_SLOW_UM_S (80000)
/**
 * Capture window in deci-degrees before the target. Line sensors count only
 * once the turn is this close to the target. Tune on the car.
 */
#define LINE_TURN_CAPTURE_DECI (300)
/**
 * Abort margin in deci-degrees past the target. Tune on the car.
 */
#define LINE_TURN_ABORT_DECI (450)

/** Distance a STRAIGHT command drives blind across a junction (mm). Tune on the car. */
#define LINE_STRAIGHT_MM (250)

/** Turn targets in deci-degrees. */
#define LINE_TURN_QUARTER_DECI (900)
#define LINE_TURN_HALF_DECI    (1800)

/** Side of the line that last saw it, or the direction of a turn. */
typedef enum
{
    LINE_SIDE_NONE = 0,
    LINE_SIDE_LEFT,
    LINE_SIDE_RIGHT
} line_side_t;

STATIC_ASSERT (sizeof(line_side_t) <= 4U, "line_side_t size");

typedef enum
{
    /** At least one sensor sees the line, or no side is recorded yet. */
    LINE_FOLLOW_FOLLOWING = 0,
    /** Neither sensor sees the line for less than LINE_GAP_MM: the car holds
     *  its heading. This is a centred car or a gap in the line. */
    LINE_FOLLOW_BRIDGING,
    /** The gap is over: the line is lost. The car turns on the spot, toward
     *  the side that saw the line last, then the other way, until a sensor
     *  sees the line. */
    LINE_FOLLOW_SEARCHING,
    /** Both search turns found nothing: the targets are zero. A sensor
     *  hit, line_follow_init(), line_follow_set_side() or
     *  line_follow_start_search() ends the stop. */
    LINE_FOLLOW_STOPPED
} line_follow_status_t;

STATIC_ASSERT (sizeof(line_follow_status_t) <= 4U,
               "line_follow_status_t size");

/* The values are the turn event's p1 on the wire (TURN_START, TURN_DONE,
 * TURN_ABORT); do not renumber. */
typedef enum
{
    LINE_TURN_KIND_NONE = 0, /* no turn; line_turn_start() treats it as idle */
    LINE_TURN_KIND_LEFT = 1, /* 90 degrees counter-clockwise */
    LINE_TURN_KIND_RIGHT,    /* 90 degrees clockwise */
    LINE_TURN_KIND_UTURN,    /* 180 degrees, pivots like RIGHT */
    LINE_TURN_KIND_STRAIGHT  /* drive LINE_STRAIGHT_MM on the start heading, sensors ignored */
} line_turn_kind_t;

STATIC_ASSERT (sizeof(line_turn_kind_t) <= 4U, "line_turn_kind_t size");

typedef enum
{
    LINE_TURN_TURNING = 0,
    LINE_TURN_DONE,
    /** Overshoot, stale feedback, or a step without a started turn. */
    LINE_TURN_ABORTED
} line_turn_status_t;

STATIC_ASSERT (sizeof(line_turn_status_t) <= 4U, "line_turn_status_t size");

/** Wheel targets in um/s. */
typedef struct
{
    int32_t left_um_s;
    int32_t right_um_s;
} line_targets_t;

STATIC_ASSERT(sizeof(line_targets_t) == 8U, "line_targets_t size");

/**
 * Follower state. Set up with line_follow_init(). side and status hold
 * line_side_t and line_follow_status_t values; they are bytes so the size
 * does not depend on the compiler's enum size.
 */
typedef struct
{
    int64_t  anchor_um;     /* travel at the last sensor hit                   */
    int32_t  ref_deci;      /* heading target                                  */
    uint32_t last_ms;
    uint8_t  side;
    uint8_t  status;
    uint8_t  started;       /* 0 until the first pass after init or a turn     */
    uint8_t  sweep;         /* search turn: 0 none yet, 1 first, 2 back        */
    uint16_t speed_mm_s;    /* base speed of the last pass (in the padding)    */
} line_follow_t;

STATIC_ASSERT(sizeof(line_follow_t) == 24U, "line_follow_t size");

/**
 * Speed cap of the sonar bumper.
 *
 * @param b_ranged True when the latest echo is a ranged reading.
 * @param dist_mm  Distance of that reading in mm.
 * @return The cap in um/s, from LINE_BUMPER_MIN_UM_S to LINE_CRUISE_UM_S.
 */
int32_t line_bumper_cap_um_s(bool b_ranged, uint32_t dist_mm);

/**
 * Scale both wheel targets so the faster one is at most cap_um_s. The ratio of
 * the two stays, so the car keeps its curve.
 */
void line_targets_cap(line_targets_t * p_targets, int32_t cap_um_s);

/**
 * Turn state. Set up with line_turn_start(). side holds a line_side_t and
 * kind a line_turn_kind_t; phase is internal to line_control.c and is not
 * for the caller to read or write.
 */
typedef struct
{
    int64_t  start_left_um;
    int64_t  start_right_um;
    int32_t  target_deci;
    uint8_t  side;
    uint8_t  kind;
    uint8_t  phase;
} line_turn_state_t;

STATIC_ASSERT(sizeof(line_turn_state_t) == 24U, "line_turn_state_t size");

/** Forget the recorded side and start in FOLLOWING. */
void line_follow_init(line_follow_t * p_follow);

/**
 * Force the recorded side (after a turn) and clear a search. The heading
 * target restarts from the heading at the next pass.
 */
void line_follow_set_side(line_follow_t * p_follow, line_side_t side);

/**
 * Start the search at once (status SEARCHING). The caller uses this after an
 * aborted turn. The search turns count from the heading at the next pass.
 */
void line_follow_start_search(line_follow_t * p_follow);

/**
 * Heading of the car since boot in deci-degrees, counter-clockwise positive:
 * (right - left displacement) * 573 / track.
 */
int32_t line_heading_deci(int64_t left_um, int64_t right_um);

/**
 * One follower pass.
 *
 * @param[in,out] p_follow   Follower state.
 * @param[in]     b_left     Left sensor is over the line.
 * @param[in]     b_right    Right sensor is over the line.
 * @param[in]     heading_deci Encoder heading from line_heading_deci().
 * @param[in]     travel_um  Mean of both wheel displacements.
 * @param[in]     now_ms     Free-running millisecond clock (wraps at 2^32).
 * @param[out]    p_targets  Wheel targets, never above LINE_CRUISE_UM_S. Both
 *                           are zero only for STOPPED.
 *
 * @return FOLLOWING, BRIDGING, SEARCHING or STOPPED.
 */
line_follow_status_t line_follow_update(line_follow_t * p_follow, bool b_left,
                                        bool b_right, int32_t heading_deci,
                                        int64_t travel_um, uint32_t now_ms,
                                        line_targets_t * p_targets);

/**
 * Begin a turn from the current wheel displacements.
 *
 * @param[out] p_turn         Turn state.
 * @param[in]  kind           LEFT, RIGHT or UTURN.
 * @param[in]  left_um        Left wheel displacement, positive forward.
 * @param[in]  right_um       Right wheel displacement, positive forward.
 */
void line_turn_start(line_turn_state_t * p_turn, line_turn_kind_t kind,
                     int64_t left_um, int64_t right_um);

/**
 * One turn pass.
 *
 * @param[in,out] p_turn     Turn state.
 * @param[in]     left_um    Left wheel displacement, positive forward.
 * @param[in]     right_um   Right wheel displacement, positive forward.
 * @param[in]     b_fresh    Feedback snapshot is fresh.
 * @param[in]     b_left     Left sensor is over the line.
 * @param[in]     b_right    Right sensor is over the line.
 * @param[out]    p_targets  Wheel targets; zero on DONE and ABORTED.
 *
 * @return TURNING, DONE or ABORTED. DONE and ABORTED are sticky.
 */
line_turn_status_t line_turn_update(line_turn_state_t * p_turn, int64_t left_um,
                                    int64_t right_um, bool b_fresh, bool b_left,
                                    bool b_right, line_targets_t * p_targets);

/**
 * Heading change since the turn started, in deci-degrees, counter-clockwise
 * positive: (right - left displacement) * 573 / track.
 */
int32_t line_turn_heading_deci(line_turn_state_t const * p_turn,
                               int64_t left_um, int64_t right_um);

/** Direction of the turn, for line_follow_set_side() once DONE. */
line_side_t line_turn_side(line_turn_state_t const * p_turn);

#endif /* TRACK_LINE_CONTROL_H */

/*** end of file ***/
