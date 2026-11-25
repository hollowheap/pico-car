/** @file line_control.c
 * @brief Pure line-following and one-shot turn logic (decisions D92, D93).
 *
 * Integer math only. The follower is a heading cascade (D181): a sensor hit
 * slews a heading target toward the line, the encoder heading follows the
 * target, and no wheel stops. With no sensor hit the car holds its heading
 * for LINE_GAP_MM, then turns on the spot toward the side that saw the line
 * last and back the other way, until a sensor sees the line or both turns are
 * done and the car stops. A turn measures its
 * heading from the wheel displacements and ends when a sensor sees the line
 * inside the capture window.
 */

#include "track/line_control.h"

#define UM_PER_MM            (1000)
#define DECI_PER_RAD         (573)
#define HEADING_CLAMP_DECI   (100000)
#define HEADING_ABS_MAX      (1000000000)

/* Turn phases (line_turn_state_t.phase). IDLE is zero so that a zeroed state is
 * safe. */
#define TURN_PHASE_IDLE      (0U)
#define TURN_PHASE_FAST      (1U)
#define TURN_PHASE_SLOW      (2U)
#define TURN_PHASE_DONE      (3U)
#define TURN_PHASE_ABORTED   (4U)

#define TRACK_UM             ((int64_t)LINE_WHEEL_TRACK_MM * UM_PER_MM)
#define GAP_UM               ((int64_t)LINE_GAP_MM * UM_PER_MM)
#define MS_PER_S             (1000)
#define UM_PER_M             (1000000)

/* line_follow_t.sweep: the search turn in progress. */
#define SWEEP_NONE           (0U)
#define SWEEP_TOWARD         (1U)
#define SWEEP_BACK           (2U)
#define SWEEP_SPAN           (2 * LINE_SEARCH_SWEEP_DECI)

static void set_targets(line_targets_t * p_targets, int32_t left_um_s,
                        int32_t right_um_s)
{
    p_targets->left_um_s  = left_um_s;
    p_targets->right_um_s = right_um_s;
}

static int64_t abs_i64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (result < low)
    {
        result = low;
    }
    if (result > high)
    {
        result = high;
    }

    return result;
}

int32_t line_heading_deci(int64_t left_um, int64_t right_um)
{
    int64_t const heading = ((right_um - left_um) * DECI_PER_RAD) / TRACK_UM;

    int64_t const limited = (heading > HEADING_ABS_MAX) ? HEADING_ABS_MAX
                            : ((heading < -HEADING_ABS_MAX) ? -HEADING_ABS_MAX : heading);

    return (int32_t)limited;
}

void line_follow_init(line_follow_t * p_follow)
{
    p_follow->anchor_um   = 0;
    p_follow->ref_deci    = 0;
    p_follow->last_ms     = 0U;
    p_follow->side        = (uint8_t)LINE_SIDE_NONE;
    p_follow->status      = (uint8_t)LINE_FOLLOW_FOLLOWING;
    p_follow->started     = 0U;
    p_follow->sweep       = SWEEP_NONE;
    p_follow->speed_mm_s  = 0U;
}

void line_follow_set_side(line_follow_t * p_follow, line_side_t side)
{
    p_follow->side    = (uint8_t)side;
    p_follow->status  = (uint8_t)LINE_FOLLOW_FOLLOWING;
    p_follow->started = 0U;
    p_follow->sweep   = SWEEP_NONE;
}

void line_follow_start_search(line_follow_t * p_follow)
{
    p_follow->status  = (uint8_t)LINE_FOLLOW_SEARCHING;
    p_follow->started = 0U;
    p_follow->sweep   = SWEEP_NONE;
}

/* Slew the heading target by a rate (deci-degrees per second) over dt_ms, and
 * keep it within LINE_CAP_DECI of the heading. */
static void slew_ref(line_follow_t * p_follow, int32_t rate, uint32_t dt_ms,
                     int32_t heading_deci, int32_t cap_deci)
{
    int32_t const step = (int32_t)(((int64_t)rate * (int64_t)dt_ms) / MS_PER_S);

    p_follow->ref_deci = clamp_i32(p_follow->ref_deci + step,
                                   heading_deci - cap_deci,
                                   heading_deci + cap_deci);
}

/* Move the heading target toward the heading by rate * dt, without passing it. */
static void relax_ref(line_follow_t * p_follow, uint32_t dt_ms, int32_t heading_deci)
{
    int32_t const step = (int32_t)(((int64_t)LINE_RATE_RELAX * (int64_t)dt_ms) / MS_PER_S);
    int32_t const gap  = heading_deci - p_follow->ref_deci;

    p_follow->ref_deci += clamp_i32(gap, -step, step);
}

/* Base speed for a heading error: cruise at no error, the corner speed at the
 * cap, a straight line between. */
static int32_t corner_speed(int32_t ref_deci, int32_t heading_deci)
{
    int64_t const err = abs_i64((int64_t)ref_deci - (int64_t)heading_deci);
    int64_t const lim = (err > LINE_SLOW_DECI) ? LINE_SLOW_DECI : err;

    return (int32_t)(LINE_CRUISE_UM_S -
                     (((int64_t)(LINE_CRUISE_UM_S - LINE_CORNER_UM_S) * lim) / LINE_SLOW_DECI));
}

int32_t line_bumper_cap_um_s(bool b_ranged, uint32_t dist_mm)
{
    int32_t cap = LINE_CRUISE_UM_S;

    if (b_ranged && (dist_mm < (uint32_t)LINE_BUMPER_FAR_MM))
    {
        int64_t const into = (dist_mm <= (uint32_t)LINE_BUMPER_NEAR_MM)
                                 ? 0
                                 : ((int64_t)dist_mm - LINE_BUMPER_NEAR_MM);

        cap = (int32_t)(LINE_BUMPER_MIN_UM_S +
                        (((int64_t)(LINE_CRUISE_UM_S - LINE_BUMPER_MIN_UM_S) * into) /
                         (LINE_BUMPER_FAR_MM - LINE_BUMPER_NEAR_MM)));
    }

    return cap;
}

void line_targets_cap(line_targets_t * p_targets, int32_t cap_um_s)
{
    int64_t const left  = abs_i64(p_targets->left_um_s);
    int64_t const right = abs_i64(p_targets->right_um_s);
    int64_t const top   = (left > right) ? left : right;

    if (top > cap_um_s)
    {
        p_targets->left_um_s  = (int32_t)(((int64_t)p_targets->left_um_s * cap_um_s) / top);
        p_targets->right_um_s = (int32_t)(((int64_t)p_targets->right_um_s * cap_um_s) / top);
    }
}

/* Wheel targets that turn the car toward a heading target. The outer wheel
 * keeps the base speed and the inner wheel slows, so no target exceeds base. */
static void steer_to(int32_t ref_deci, int32_t heading_deci, int32_t base_um_s,
                     line_targets_t * p_targets)
{
    int64_t const err   = (int64_t)ref_deci - (int64_t)heading_deci;
    int64_t const diff  = ((int64_t)LINE_KH * err * TRACK_UM) / DECI_PER_RAD;
    int32_t const slow  = (int32_t)((abs_i64(diff) > base_um_s) ? base_um_s
                                                                : abs_i64(diff));

    if (diff > 0)
    {
        set_targets(p_targets, base_um_s - slow, base_um_s);
    }
    else
    {
        set_targets(p_targets, base_um_s, base_um_s - slow);
    }
}

static void steer(line_follow_t const * p_follow, int32_t heading_deci,
                  int32_t base_um_s, line_targets_t * p_targets)
{
    steer_to(p_follow->ref_deci, heading_deci, base_um_s, p_targets);
}

/* Start the pass: the first call after init or a turn takes the heading as the
 * target. Returns the time since the last pass, limited to LINE_DT_MAX_MS. */
static uint32_t begin_pass(line_follow_t * p_follow, int32_t heading_deci,
                           int64_t travel_um, uint32_t now_ms)
{
    uint32_t dt_ms = 0U;

    if (0U == p_follow->started)
    {
        p_follow->started     = 1U;
        p_follow->speed_mm_s  = (uint16_t)(LINE_CRUISE_UM_S / UM_PER_MM);
        p_follow->ref_deci    = heading_deci;
        p_follow->anchor_um   = travel_um;
        p_follow->last_ms     = now_ms;
    }
    else
    {
        dt_ms = (uint32_t)(now_ms - p_follow->last_ms);
        p_follow->last_ms = now_ms;
        if (dt_ms > LINE_DT_MAX_MS)
        {
            dt_ms = LINE_DT_MAX_MS;
        }
    }

    return dt_ms;
}

/* The search turn. The heading target is the end of the current turn: first
 * LINE_SEARCH_SWEEP_DECI toward the side that saw the line last (left if none
 * did), then as far the other way. At the end of the second turn the car
 * stops. */
static void search_turn(line_follow_t * p_follow, int32_t heading_deci)
{
    int32_t const sign = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -1 : 1;
    int32_t const past = sign * (heading_deci - p_follow->ref_deci);

    if (SWEEP_NONE == p_follow->sweep)
    {
        p_follow->sweep    = SWEEP_TOWARD;
        p_follow->ref_deci = heading_deci + (sign * LINE_SEARCH_SWEEP_DECI);
    }
    else if ((SWEEP_TOWARD == p_follow->sweep) && (past >= 0))
    {
        p_follow->sweep    = SWEEP_BACK;
        p_follow->ref_deci -= sign * SWEEP_SPAN;
    }
    else if ((SWEEP_BACK == p_follow->sweep) && (past <= 0))
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_STOPPED;
    }
    else
    {
        /* Still turning. */
    }
}

/* Neither sensor sees the line. A short stretch is a crossing, a gap or the
 * line between the sensors: the car holds its heading. After LINE_GAP_MM the
 * line is lost and the car searches on the spot. */
static void follow_blind(line_follow_t * p_follow, int32_t heading_deci,
                         int64_t travel_um, uint32_t dt_ms)
{
    int64_t const blind_um = abs_i64(travel_um - p_follow->anchor_um);

    if ((uint8_t)LINE_FOLLOW_FOLLOWING == p_follow->status)
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_BRIDGING;
    }
    if (((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status) && (blind_um >= GAP_UM))
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_SEARCHING;
    }
    if ((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status)
    {
        /* Stop turning, and keep the target within reach of the heading. */
        relax_ref(p_follow, dt_ms, heading_deci);
        slew_ref(p_follow, 0, dt_ms, heading_deci, LINE_CAP_DECI);
    }
    else if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        search_turn(p_follow, heading_deci);
    }
    else
    {
        /* STOPPED: wait for a sensor hit. */
    }
}

line_follow_status_t line_follow_update(line_follow_t * p_follow, bool b_left,
                                        bool b_right, int32_t heading_deci,
                                        int64_t travel_um, uint32_t now_ms,
                                        line_targets_t * p_targets)
{
    uint32_t const dt_ms = begin_pass(p_follow, heading_deci, travel_um, now_ms);
    int32_t base_um_s = 0;

    if ((b_left || b_right) && (SWEEP_NONE != p_follow->sweep))
    {
        /* Found by the search: follow from the heading, not from the end of
         * the search turn. The speed drops with the angle the search turned,
         * as for a heading error: a centred car finds the line at once and
         * keeps most of its speed, a corner restarts slow. */
        int32_t const sign   = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -1 : 1;
        int32_t const origin = (SWEEP_TOWARD == p_follow->sweep)
                                   ? (p_follow->ref_deci - (sign * LINE_SEARCH_SWEEP_DECI))
                                   : (p_follow->ref_deci + (sign * LINE_SEARCH_SWEEP_DECI));
        int32_t const speed  = corner_speed(origin, heading_deci) / UM_PER_MM;

        if (speed < (int32_t)p_follow->speed_mm_s)
        {
            p_follow->speed_mm_s = (uint16_t)speed;
        }
        p_follow->sweep    = SWEEP_NONE;
        p_follow->ref_deci = heading_deci;
    }
    if (b_left && b_right)
    {
        p_follow->ref_deci  = heading_deci;
        p_follow->anchor_um = travel_um;
        p_follow->status    = (uint8_t)LINE_FOLLOW_FOLLOWING;
    }
    else if (b_left || b_right)
    {
        p_follow->side      = (uint8_t)(b_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
        p_follow->anchor_um = travel_um;
        p_follow->status    = (uint8_t)LINE_FOLLOW_FOLLOWING;
        /* The target turns a fixed angle per mm travelled, so a faster car
         * turns its target faster and still holds the tightest corner. */
        int32_t const rate = (int32_t)(((int64_t)LINE_HIT_DECI_PER_M *
                                        corner_speed(p_follow->ref_deci, heading_deci)) /
                                       UM_PER_M);

        slew_ref(p_follow, b_left ? rate : -rate, dt_ms, heading_deci, LINE_CAP_DECI);
    }
    else
    {
        follow_blind(p_follow, heading_deci, travel_um, dt_ms);
    }

    if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        /* Keep the speed from before the search for the restart. */
        base_um_s = (int32_t)p_follow->speed_mm_s * UM_PER_MM;
    }
    else
    {
        /* Slow down at once, speed up at LINE_ACCEL_MM_S2. */
        int32_t const want = corner_speed(p_follow->ref_deci, heading_deci);
        int32_t const rise = (int32_t)(((int64_t)LINE_ACCEL_MM_S2 * UM_PER_MM * dt_ms) / MS_PER_S);
        int32_t const last = (int32_t)p_follow->speed_mm_s * UM_PER_MM;

        base_um_s = (want < (last + rise)) ? want : (last + rise);
    }
    p_follow->speed_mm_s = (uint16_t)(base_um_s / UM_PER_MM);
    if ((uint8_t)LINE_FOLLOW_STOPPED == p_follow->status)
    {
        set_targets(p_targets, 0, 0);
    }
    else if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        /* On the spot: counter-clockwise when the target is to the left. */
        int32_t const spin = (p_follow->ref_deci > heading_deci) ? LINE_SEARCH_UM_S
                                                                 : -LINE_SEARCH_UM_S;

        set_targets(p_targets, -spin, spin);
    }
    else
    {
        steer(p_follow, heading_deci, base_um_s, p_targets);
    }

    return (line_follow_status_t)p_follow->status;
}

void line_turn_start(line_turn_state_t * p_turn, line_turn_kind_t kind,
                     int64_t left_um, int64_t right_um)
{
    p_turn->start_left_um  = left_um;
    p_turn->start_right_um = right_um;
    p_turn->kind           = (uint8_t)kind;
    p_turn->phase          = TURN_PHASE_FAST;

    switch (kind)
    {
        case LINE_TURN_KIND_LEFT:
            p_turn->target_deci = LINE_TURN_QUARTER_DECI;
            p_turn->side        = (uint8_t)LINE_SIDE_LEFT;
            break;
        case LINE_TURN_KIND_RIGHT:
            p_turn->target_deci = LINE_TURN_QUARTER_DECI;
            p_turn->side        = (uint8_t)LINE_SIDE_RIGHT;
            break;
        case LINE_TURN_KIND_UTURN:
            p_turn->target_deci = LINE_TURN_HALF_DECI;
            p_turn->side        = (uint8_t)LINE_SIDE_RIGHT;
            break;
        case LINE_TURN_KIND_STRAIGHT:
            p_turn->target_deci = 0;
            p_turn->side        = (uint8_t)LINE_SIDE_NONE;
            break;
        default:
            p_turn->target_deci = 0;
            p_turn->side        = (uint8_t)LINE_SIDE_NONE;
            p_turn->phase       = TURN_PHASE_IDLE;
            break;
    }
}

int32_t line_turn_heading_deci(line_turn_state_t const * p_turn,
                               int64_t left_um, int64_t right_um)
{
    int64_t const diff_um = (right_um - p_turn->start_right_um) -
                            (left_um - p_turn->start_left_um);
    int64_t heading = (diff_um * DECI_PER_RAD) / TRACK_UM;

    if (heading > HEADING_CLAMP_DECI)
    {
        heading = HEADING_CLAMP_DECI;
    }
    if (heading < -HEADING_CLAMP_DECI)
    {
        heading = -HEADING_CLAMP_DECI;
    }

    return (int32_t)heading;
}

line_side_t line_turn_side(line_turn_state_t const * p_turn)
{
    return (line_side_t)p_turn->side;
}

static line_turn_status_t turn_stop(line_turn_state_t * p_turn, uint8_t phase,
                                    line_targets_t * p_targets)
{
    p_turn->phase = phase;
    set_targets(p_targets, 0, 0);

    return (TURN_PHASE_DONE == phase) ? LINE_TURN_DONE : LINE_TURN_ABORTED;
}

/* Drive straight on the start heading for LINE_STRAIGHT_MM. The line sensors
 * do not count: the car crosses a junction. The wheels steer back to the
 * start heading with the same law as the follower. */
static line_turn_status_t straight_update(line_turn_state_t * p_turn, int64_t left_um,
                                          int64_t right_um, bool b_fresh,
                                          line_targets_t * p_targets)
{
    int32_t const heading = line_turn_heading_deci(p_turn, left_um, right_um);
    int64_t const travel_um = ((left_um - p_turn->start_left_um) +
                               (right_um - p_turn->start_right_um)) / 2;
    line_turn_status_t status = LINE_TURN_TURNING;

    if ((TURN_PHASE_FAST != p_turn->phase) && (TURN_PHASE_SLOW != p_turn->phase))
    {
        status = turn_stop(p_turn, (TURN_PHASE_DONE == p_turn->phase) ? TURN_PHASE_DONE
                                                                      : TURN_PHASE_ABORTED,
                           p_targets);
    }
    else if (!b_fresh || (abs_i64(heading) > LINE_TURN_ABORT_DECI))
    {
        status = turn_stop(p_turn, TURN_PHASE_ABORTED, p_targets);
    }
    else if (travel_um >= ((int64_t)LINE_STRAIGHT_MM * UM_PER_MM))
    {
        status = turn_stop(p_turn, TURN_PHASE_DONE, p_targets);
    }
    else
    {
        steer_to(0, heading, LINE_CRUISE_UM_S, p_targets);
    }

    return status;
}

line_turn_status_t line_turn_update(line_turn_state_t * p_turn, int64_t left_um,
                                    int64_t right_um, bool b_fresh, bool b_left,
                                    bool b_right, line_targets_t * p_targets)
{
    int32_t heading = 0;
    int32_t progress = 0;
    int32_t speed = LINE_TURN_FAST_UM_S;
    line_turn_status_t status = LINE_TURN_TURNING;

    if ((uint8_t)LINE_TURN_KIND_STRAIGHT == p_turn->kind)
    {
        return straight_update(p_turn, left_um, right_um, b_fresh, p_targets);
    }

    if ((TURN_PHASE_FAST != p_turn->phase) &&
        (TURN_PHASE_SLOW != p_turn->phase))
    {
        status = turn_stop(p_turn, (TURN_PHASE_DONE == p_turn->phase)
                                       ? TURN_PHASE_DONE
                                       : TURN_PHASE_ABORTED,
                           p_targets);
    }
    else if (!b_fresh)
    {
        status = turn_stop(p_turn, TURN_PHASE_ABORTED, p_targets);
    }
    else
    {
        heading  = line_turn_heading_deci(p_turn, left_um, right_um);
        progress = (LINE_SIDE_LEFT == (line_side_t)p_turn->side) ? heading
                                                                  : -heading;
        if (((heading < 0) ? -heading : heading) >
            (p_turn->target_deci + LINE_TURN_ABORT_DECI))
        {
            status = turn_stop(p_turn, TURN_PHASE_ABORTED, p_targets);
        }
        else
        {
            if (progress >= (p_turn->target_deci - LINE_TURN_CAPTURE_DECI))
            {
                p_turn->phase = TURN_PHASE_SLOW;
            }
            if ((TURN_PHASE_SLOW == p_turn->phase) && (b_left || b_right))
            {
                status = turn_stop(p_turn, TURN_PHASE_DONE, p_targets);
            }
        }
    }

    if (LINE_TURN_TURNING == status)
    {
        if (TURN_PHASE_SLOW == p_turn->phase)
        {
            speed = LINE_TURN_SLOW_UM_S;
        }
        if (LINE_SIDE_LEFT == (line_side_t)p_turn->side)
        {
            set_targets(p_targets, -speed, speed);
        }
        else
        {
            set_targets(p_targets, speed, -speed);
        }
    }

    return status;
}
