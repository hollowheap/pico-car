/** @file line_control.h
 * @brief Pure line-following and one-shot turn logic.
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

#include "platform/chassis.h"
#include "platform/static_assert.h"
#include "platform/units.h"

/**
 * Corner speed. The speed falls in a straight line from cruise at no heading
 * error to LINE_CORNER_UM_S at LINE_SLOW_DECI of error and more, so a
 * correction slows the car at once. The speed then climbs back at most at
 * LINE_ACCEL_MM_S2, so the car is still slow when the next correction of a
 * corner comes (on the car, 120 mm/s and an instant climb missed the turn).
 */
#define LINE_CORNER_UM_S (100000)
#define LINE_SLOW_DECI   (20)
/* From 100 to 163 mm/s in about one wheel track of straight travel:
 * (v_cruise^2 - v_corner^2) / (2 * 107 mm) gives about 78 mm/s^2. */
#define LINE_ACCEL_MM_S2 (80)

/**
 * Cruise speed of the follower on a straight, and the cap on every follower
 * target. Buddy 5's straight-wheel request in um/s.
 */
#define LINE_CRUISE_UM_S (163364)

/**
 * The follower keeps its speed in steps of this size (um/s) in a 16-bit field.
 * It is far below the smallest rise of one pass (LINE_ACCEL_MM_S2 over 1 ms is
 * 80 um/s), so a short pass still raises the speed.
 */
#define LINE_SPEED_STEP_UM_S (4)
STATIC_ASSERT((LINE_CRUISE_UM_S / LINE_SPEED_STEP_UM_S) <= 65535,
              "follower speed fits 16 bits");

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
 * Geometry of the sensors, measured on the car (2026-11-12): 25 mm between the
 * sensor midpoints, 130 mm from the axle to the sensors. The sensors are now
 * fixed at that spacing. The line is solid black and 15 mm wide, so a centred
 * car sees it with neither sensor. The route probe and square-corner
 * search use the measured tape width and the sensor lead.
 */
#define LINE_SENSOR_SPACING_MM (25)
#define LINE_SENSOR_COUNT      (2U)
#define LINE_SENSOR_LEAD_MM    (130)
#define LINE_WIDTH_MM          (15)

/**
 * Heading cascade. The sensors move a heading target; the encoder
 * heading follows it. All values are starting values: tune on the car.
 *
 * LINE_KH            Yaw rate per heading error, in 1/s.
 * LINE_HIT_DECI_PER_M Target slew per metre of requested outer-wheel travel.
 *                    Buddy 5 stops the inner wheel on a hit. That geometry
 *                    turns through one radian per wheel track of outer travel.
 *                    Slewing toward that curvature preserves a gradual entry.
 * LINE_RATE_RELAX    Slew of the target back to the heading once the line is
 *                    between the sensors again (deci-degrees per second).
 *                    The car settles the correction, then briefly holds that
 *                    heading before relaxed steering resumes in the gap.
 * LINE_CAP_DECI      The target stays within this of the heading.
 * LINE_GAP_MM        Travel without a sensor hit that is still a crossing, a
 *                    gap, or the line between the sensors. On flat ground
 *                    the car makes a shallow forward steering probe near the
 *                    end of this distance to pick up the edge of centered
 *                    tape. If a detector first crossed a transverse
 *                    stripe, clear readings to its projected axle position
 *                    start the search there. The normal limit remains for
 *                    an unqualified gap. The search turns on the spot.
 * LINE_SEARCH_UM_S   Wheel speed of the search. The car does not drive on: a
 *                    car that drove on blind found another line and took it.
 *                    It turns on the spot toward the side that saw the line
 *                    last, then back the other way, until a sensor sees the
 *                    line.
 * LINE_SEARCH_SWEEP_DECI How far the search turns each way from the heading
 *                    where it started.
 * LINE_RETRY_COUNT   Retries after a failed search (a guess, not measured on
 *                    the car). A retry turns the car back to the heading it
 *                    had when the search began, reverses LINE_REVERSE_MM
 *                    straight and searches again. After the last retry fails
 *                    the car stops (status STOPPED).
 *                    The retry count stays until a search turn finds the line
 *                    or a hit comes past the travel where the search began
 *                    (to 1 cm), so a reverse that only finds the line the
 *                    car came on does not give the car new retries.
 * LINE_REVERSE_MM    Length of that reverse, about LINE_GAP_MM so that the
 *                    sensors come back over the ground they crossed blind (a
 *                    guess, not measured on the car). It runs at
 *                    LINE_SEARCH_UM_S.
 * LINE_REVERSE_STALL_MS, LINE_REVERSE_STEP_MM
 *                    A reverse in which the slower wheel does not back up
 *                    LINE_REVERSE_STEP_MM within LINE_REVERSE_STALL_MS (so a
 *                    jam of one wheel counts) is blocked: it ends and the
 *                    search begins again, so a blocked car still reaches
 *                    STOPPED after the retries (guesses, not measured on the
 *                    car).
 * LINE_HUMP_GAP_MM   LINE_GAP_MM while the physics reports a hump. The
 *                    sensors lift off the floor at the crest, so the blind
 *                    stretch is longer (a guess, not measured on the car).
 */
#define LINE_KH             (8)
#define LINE_HIT_DECI_PER_M ((DECI_PER_RAD * UM_PER_MM) / CHASSIS_WHEEL_TRACK_MM)
#define LINE_RATE_RELAX     (400)
#define LINE_CAP_DECI       (120)
#define LINE_GAP_MM         (150)
/** Start the probe with two sensor spacings of forward travel left. */
#define LINE_PROBE_FROM_MM  (LINE_GAP_MM - 2 * LINE_SENSOR_SPACING_MM)
#define LINE_SEARCH_UM_S    (90000)
#define LINE_SEARCH_SWEEP_DECI (1350)
/* Local search after obstacle egress. At 30 degrees the outer detector
 * center reaches 76 mm sideways from the axle path (130 mm lead, 25 mm
 * spacing). The ideal centered footprint reaches 83 mm including tape
 * half-width. Actual reach and braking lag need a car measurement. */
#define LINE_LOCAL_SEARCH_DECI (300)
/* Allow one normal stall window to make a measurable angular advance. */
#define LINE_LOCAL_SEARCH_STALL_MS LINE_REVERSE_STALL_MS
#define LINE_RETRY_COUNT    (3U)
#define LINE_REVERSE_MM     (150)
#define LINE_REVERSE_STALL_MS (1500U)
#define LINE_REVERSE_STEP_MM  (10)
#define LINE_HUMP_GAP_MM    (400)
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

/** Distance a manual STRAIGHT command drives on its initial heading (mm). Tune on the car. */
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
    /** Neither sensor sees the line within the bridge distance. The car
     * advances and may probe the tape while both wheels move forward. */
    LINE_FOLLOW_BRIDGING,
    /** The gap is over: the line is lost. The car turns on the spot, toward
     *  the side that saw the line last, then the other way, until a sensor
     *  sees the line. After a failed search it turns back to the heading it
     *  had when the search began, reverses
     *  LINE_REVERSE_MM and searches again, LINE_RETRY_COUNT times. */
    LINE_FOLLOW_SEARCHING,
    /** The last retry found nothing: the targets are zero. A sensor
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
    LINE_TURN_KIND_STRAIGHT  /* manual crossing or barcode through-route selection */
} line_turn_kind_t;

STATIC_ASSERT (sizeof(line_turn_kind_t) <= 4U, "line_turn_kind_t size");

typedef enum
{
    LINE_TURN_TURNING = 0,
    LINE_TURN_DONE,
    /** Overshoot, stale feedback, or a step without a started turn. */
    LINE_TURN_ABORTED,
    LINE_TURN_NO_BRANCH,
    /** Both routes confirmed. The supplied targets begin branch alignment. */
    LINE_TURN_BRANCH_SELECTED,
    /** Centered on available tape while the requested instruction stays pending. */
    LINE_TURN_FALLBACK_DONE
} line_turn_status_t;

STATIC_ASSERT (sizeof(line_turn_status_t) <= 4U, "line_turn_status_t size");

/** Wheel targets in um/s. */
typedef struct
{
    int32_t left_um_s;
    int32_t right_um_s;
} line_targets_t;

STATIC_ASSERT(sizeof(line_targets_t) == 8U, "line_targets_t size");

/* Wheel travel at the candidate and the approach heading define the local
 * frame for scanning both outgoing tape segments. Zero initializes a route. */
typedef struct
{
    int64_t start_um;
    int32_t heading_deci;
    uint16_t left_begin_mm;
    uint16_t left_at_mm;
    uint16_t side_at_mm;
    uint16_t right_begin_mm;
    uint8_t phase;
    uint8_t kind;
    bool    left_on_tape : 1;
    bool    left_recent : 1;
    bool    left_stripe_anchored : 1;
    bool    side_recent : 1;
    bool    was_crossing : 1;
    bool    probe_eligible : 1;
    bool    through_seen : 1;
    bool    clear_seen : 1;
    bool    right_on_tape : 1;
    bool    right_branch : 1;
    bool    opposite_seen : 1;
    bool    align_increasing : 1;
    bool    retained : 1;
    bool    extended : 1;
} line_route_t;

STATIC_ASSERT(sizeof(line_route_t) == 24U, "line_route_t size");

/* True when the axle reaches a candidate on a plausible approach. The fixed
 * barcode reader is left of the line. A right-hand candidate uses the right
 * line detector's transverse tape width. STRAIGHT can probe either side. */
bool line_route_observe(line_route_t * p_route, line_turn_kind_t kind, bool b_left, bool b_right,
                        bool b_side_tape, int64_t left_um, int64_t right_um);

/** The route probe has an optical crossing to survey at the projected axle. */
bool line_route_has_candidate(line_route_t const * p_route);

/** A scan has finished and the car is centering on an observed fallback exit. */
bool line_route_returning(line_route_t const * p_route);

/* Probe the outgoing tape directions around the axle. BRANCH_SELECTED commits
 * the requested exit and DONE finishes alignment on its tape. FALLBACK_DONE
 * hands the car to the follower on an available exit with the command pending.
 * NO_BRANCH restores the approach heading when the sweep lacks usable evidence. */
line_turn_status_t line_route_probe(line_route_t * p_route, int64_t left_um, int64_t right_um,
                                    bool b_tape, line_targets_t * p_targets);

/**
 * Follower state. Set up with line_follow_init(). side and status hold
 * line_side_t and line_follow_status_t values; they are bytes so the size
 * does not depend on the compiler's enum size.
 */
#define LINE_FOLLOW_STARTED (1U)
#define LINE_FOLLOW_HUMP    (2U)

/* Following and search reverse never run together. */
typedef union
{
    struct
    {
        uint32_t progress_ms;
        int32_t heading_deci;
    } local;
    struct
    {
        int32_t mark_um;
        uint16_t stall_ms;
    } reverse;
    struct
    {
        uint16_t begin_mm[LINE_SENSOR_COUNT];
        uint16_t anchor_mm;
        uint8_t detector_bits; /* current levels and corroborated stripe bits */
        uint8_t marker_mm;     /* position of the last side-reader bridge */
    } stripe;
} line_follow_progress_t;

STATIC_ASSERT(sizeof(line_follow_progress_t) == 8U, "line follow progress size");

typedef struct
{
    int64_t  anchor_um;     /* travel at the last sensor hit                   */
    int32_t  ref_deci;      /* heading target                                  */
    uint32_t last_ms;
    uint8_t  side;
    uint8_t  status;
    uint8_t  flags;         /* LINE_FOLLOW_STARTED: set after the first pass since init or a turn;
                             * LINE_FOLLOW_HUMP: the physics reports a hump (longer blind limit) */
    uint8_t  sweep;         /* search: 0 none, 1 toward, 2 back, 3 turn back, 4 reverse */
    uint8_t  retry;         /* retries begun since the last search hit         */
    uint8_t  loss_cm;       /* travel of the loss, in cm modulo 256: retry clears past it */
    uint16_t speed_q;       /* base speed of the last pass, in LINE_SPEED_STEP_UM_S */
    line_follow_progress_t progress;
} line_follow_t;

STATIC_ASSERT(sizeof(line_follow_t) == 32U, "line_follow_t size");

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
 * Line-lost bookkeeping of the line task, kept here so it runs on the host.
 * b_lost is the line-lost state to report; b_stopped is set after the first
 * STOPPED of a search.
 */
typedef struct
{
    uint8_t b_lost;
    uint8_t b_stopped;
} line_watch_t;

STATIC_ASSERT(sizeof(line_watch_t) == 2U, "line_watch_t size");

/** Returned by line_watch_step(): the follower has just stopped for the first time. */
#define LINE_WATCH_STOP  (0x01U)
/** Returned by line_watch_step(): the lost state ended with the hug, so clear it
 *  without a line-found event. */
#define LINE_WATCH_QUIET (0x02U)

/**
 * One pass of the line-lost bookkeeping, after each pilot step. Only a
 * follower that has the wheels counts: SEARCHING or STOPPED is lost. While the
 * hug has the wheels the line is off the sensors and the follower is started
 * afresh when the hug ends, so a lost state then ends quietly.
 *
 * @param[in,out] p_watch  State.
 * @param[in]     b_follow The follower set the command.
 * @param[in]     b_hug    The hug set the command.
 * @param[in]     status   Status of the follower.
 *
 * @return LINE_WATCH_* bits; p_watch->b_lost is the state to report.
 */
uint8_t line_watch_step(line_watch_t * p_watch, bool b_follow, bool b_hug,
                        line_follow_status_t status);

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
 * Count blind travel from the next pass, unless a search is under way. The
 * caller uses this after driving straight without line_follow_update(), so that
 * the straight does not count as blind travel.
 */
void line_follow_reanchor(line_follow_t * p_follow);

/** Hold speed over matching front readings during a marker. Returns false
 * when a directional reading or the normal blind-travel probe takes over. */
bool line_follow_marker_sample(line_follow_t * p_follow, bool b_left, bool b_right, bool b_progress,
                               uint8_t prefix_mm, int32_t heading_deci, int64_t travel_um, uint32_t now_ms, line_targets_t * p_targets);

/**
 * Start the search at once (status SEARCHING). The caller uses this after an
 * aborted turn. The search turns count from the heading at the next pass.
 */
void line_follow_start_search(line_follow_t * p_follow);

/** Search both sides of the entry heading at a qualified egress position.
 * A tape hit resumes steering. An empty search returns to that heading
 * and stops. A stalled sweep or explicit interruption stops at the current pose. */
void line_follow_start_local_search(line_follow_t * p_follow);

/** True during a local search and after its stop, until a fresh start or hit. */
bool line_follow_local_search(line_follow_t const * p_follow);

/** True only after a complete empty sweep returned to its original heading.
 * The caller can consume this result with line_follow_stop_local_search(). */
bool line_follow_local_missed(line_follow_t const * p_follow);

/** Interrupt or acknowledge a local search and retain its stopped state. */
void line_follow_stop_local_search(line_follow_t * p_follow);

/**
 * Tell the follower whether the physics reports a hump. Call it before each
 * line_follow_update(): while set, the blind limit is LINE_HUMP_GAP_MM.
 */
void line_follow_set_hump(line_follow_t * p_follow, bool b_hump);

/** Reserve an observed candidate for the route probe. A lost candidate gives
 * the ordinary follower its stored transverse tape position back. */
void line_follow_set_route_candidate(line_follow_t * p_follow, bool b_candidate);

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
