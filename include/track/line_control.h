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

/** Cruise speed of the follower, and the cap on every follower target. */
#define LINE_CRUISE_UM_S (163364)

/**
 * Wheel track in mm. Copy of OBSTACLE_WHEEL_TRACK_MM in
 * src/navigation/obstacle/obstacle_config.h (measured on the car). Keep the
 * two in step; this module must not include the obstacle header.
 */
#define LINE_WHEEL_TRACK_MM (107)

/** Search limit by mean wheel travel. Tune on the car. */
#define LINE_SEARCH_MAX_MM (150)
/** Search limit by time. Tune on the car. */
#define LINE_SEARCH_MAX_MS (1500U)

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

typedef enum
{
    /** At least one sensor sees the line, or no side is recorded yet. */
    LINE_FOLLOW_FOLLOWING = 0,
    /** Neither sensor sees the line; correcting toward the recorded side. */
    LINE_FOLLOW_SEARCHING,
    /** Search limit reached; targets are zero until a sensor sees the line. */
    LINE_FOLLOW_LOST
} line_follow_status_t;

typedef enum
{
    LINE_TURN_KIND_LEFT = 1, /* 90 degrees counter-clockwise */
    LINE_TURN_KIND_RIGHT,    /* 90 degrees clockwise */
    LINE_TURN_KIND_UTURN     /* 180 degrees, pivots like RIGHT */
} line_turn_kind_t;

typedef enum
{
    LINE_TURN_TURNING = 0,
    LINE_TURN_DONE,
    /** Overshoot, stale feedback, or a step without a started turn. */
    LINE_TURN_ABORTED
} line_turn_status_t;

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
    int64_t  search_start_um;
    uint32_t search_start_ms;
    uint8_t  side;
    uint8_t  status;
} line_follow_t;

STATIC_ASSERT(sizeof(line_follow_t) == 16U, "line_follow_t size");

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
 * Force the recorded side (after a turn) and clear a search or LOST state.
 */
void line_follow_set_side(line_follow_t * p_follow, line_side_t side);

/**
 * Stop the follower (targets 0, status LOST) until a sensor sees the line.
 * The recorded side is kept. The caller uses this after an aborted turn.
 */
void line_follow_hold_lost(line_follow_t * p_follow);

/**
 * One follower pass.
 *
 * @param[in,out] p_follow   Follower state.
 * @param[in]     b_left     Left sensor is over the line.
 * @param[in]     b_right    Right sensor is over the line.
 * @param[in]     travel_um  Mean of both wheel displacements.
 * @param[in]     now_ms     Free-running millisecond clock (wraps at 2^32).
 * @param[out]    p_targets  Wheel targets, never above LINE_CRUISE_UM_S.
 *
 * @return FOLLOWING, SEARCHING or LOST.
 */
line_follow_status_t line_follow_update(line_follow_t * p_follow, bool b_left,
                                        bool b_right, int64_t travel_um,
                                        uint32_t now_ms,
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
