/** @file line_control.c
 * @brief Pure line-following and one-shot turn logic.
 *
 * Integer math only. The follower is a heading cascade: a sensor hit slews a
 * heading target toward the line, the encoder heading follows the target, and
 * no wheel stops. With no sensor hit the car holds its heading for
 * LINE_GAP_MM, then turns on the spot toward the side that saw the line last
 * and back the other way, and retries from a reverse (see search_turn()),
 * until a sensor sees the line or the last retry fails and the car stops. A
 * turn measures its heading from the wheel displacements and ends when a
 * sensor sees the line inside the capture window.
 */

#include "track/line_control.h"

#include "platform/units.h"

#define HEADING_CLAMP_DECI   (100000)
#define HEADING_ABS_MAX      (1000000000)

/* Turn phases (line_turn_state_t.phase). IDLE is zero so that a zeroed state is
 * safe. */
#define TURN_PHASE_IDLE      (0U)
#define TURN_PHASE_FAST      (1U)
#define TURN_PHASE_SLOW      (2U)
#define TURN_PHASE_DONE      (3U)
#define TURN_PHASE_ABORTED   (4U)

#define TRACK_UM             ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
#define GAP_UM               ((int64_t)LINE_GAP_MM * UM_PER_MM)
#define HUMP_GAP_UM          ((int64_t)LINE_HUMP_GAP_MM * UM_PER_MM)
#define REVERSE_UM           ((int64_t)LINE_REVERSE_MM * UM_PER_MM)
#define MS_PER_S             (1000)
#define UM_PER_M             (1000000)
#define UM_PER_CM            (10000)
#define CM_MASK              (0xFFU)
#define CM_HALF              (0x80U)
#define REVERSE_STEP_UM      ((int64_t)LINE_REVERSE_STEP_MM * UM_PER_MM)

/* line_follow_t.sweep: the search turn in progress. */
#define SWEEP_NONE           (0U)
#define SWEEP_TOWARD         (1U)
#define SWEEP_BACK           (2U)
#define SWEEP_TURN_BACK      (3U)
#define SWEEP_REVERSE        (4U)
#define SWEEP_SPAN           (2 * LINE_SEARCH_SWEEP_DECI)

static void set_targets(line_targets_t * p_targets, int32_t left_um_s,
                        int32_t right_um_s)
{
    p_targets->left_um_s  = left_um_s;
    p_targets->right_um_s = right_um_s;
}

/* Travel in cm modulo 256, rounded down. The loss point stays within 128 cm of
 * the car while a retry count is held, so the wrapped difference tells before
 * from beyond. */
static uint8_t travel_cm(int64_t travel_um)
{
    int64_t cm = travel_um / UM_PER_CM;

    if ((travel_um % UM_PER_CM) < 0)
    {
        cm--;
    }

    return (uint8_t)((uint64_t)cm & CM_MASK);
}

/* True when the car is 1 to 127 cm beyond the loss point. */
static bool past_loss(uint8_t loss_cm, int64_t travel_um)
{
    uint8_t const ahead = (uint8_t)(travel_cm(travel_um) - loss_cm);

    return (0U != ahead) && (ahead < CM_HALF);
}

static int64_t abs_i64(int64_t value)
{
    return (value < 0) ? -value : value;
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
    p_follow->flags       = 0U;
    p_follow->sweep       = SWEEP_NONE;
    p_follow->speed_q     = 0U;
    p_follow->retry       = 0U;
    p_follow->loss_cm     = 0U;
    p_follow->rev_mark_um  = 0;
    p_follow->rev_stall_ms = 0U;
}

void line_follow_set_hump(line_follow_t * p_follow, bool b_hump)
{
    if (b_hump)
    {
        p_follow->flags |= LINE_FOLLOW_HUMP;
    }
    else
    {
        p_follow->flags &= (uint8_t)~LINE_FOLLOW_HUMP;
    }
}

void line_follow_set_side(line_follow_t * p_follow, line_side_t side)
{
    p_follow->side    = (uint8_t)side;
    p_follow->status  = (uint8_t)LINE_FOLLOW_FOLLOWING;
    p_follow->flags  &= (uint8_t)~LINE_FOLLOW_STARTED;
    p_follow->sweep   = SWEEP_NONE;
    p_follow->retry   = 0U;
}

void line_follow_reanchor(line_follow_t * p_follow)
{
    /* A search in progress keeps its turn and its reverse start. */
    if ((uint8_t)LINE_FOLLOW_SEARCHING != p_follow->status)
    {
        p_follow->flags &= (uint8_t)~LINE_FOLLOW_STARTED;
    }
}

void line_follow_start_search(line_follow_t * p_follow)
{
    p_follow->status  = (uint8_t)LINE_FOLLOW_SEARCHING;
    p_follow->flags  &= (uint8_t)~LINE_FOLLOW_STARTED;
    p_follow->sweep   = SWEEP_NONE;
    p_follow->retry   = 0U;
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

    if (0U == (p_follow->flags & LINE_FOLLOW_STARTED))
    {
        p_follow->flags      |= LINE_FOLLOW_STARTED;
        p_follow->speed_q     = (uint16_t)(LINE_CRUISE_UM_S / LINE_SPEED_STEP_UM_S);
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

/* A reverse in which the slower wheel does not back up LINE_REVERSE_STEP_MM
 * within LINE_REVERSE_STALL_MS is blocked. The slower wheel is the mean travel
 * less half the wheel difference, which the heading's departure from the
 * reverse heading gives: with one wheel jammed the other turns the car round
 * it and the mean grows, but the slower wheel does not. The departure is taken
 * from the target heading, not the heading the reverse began at, so the
 * turn-back overshoot (one pass, under half a step) shifts the estimate by at
 * most one step: one extra stall window at worst. The time counts passes
 * of the follower only, each limited to LINE_DT_MAX_MS, so a pause of the
 * caller does not count. */
static bool reverse_blocked(line_follow_t * p_follow, int64_t back_um,
                            int32_t heading_deci, uint32_t dt_ms)
{
    int64_t const off_deci = (int64_t)heading_deci - p_follow->ref_deci;
    int64_t const yaw_um = (abs_i64(off_deci) * TRACK_UM) / (2 * DECI_PER_RAD);
    int64_t const slow_um = back_um - yaw_um;
    bool b_blocked = false;

    if ((slow_um - p_follow->rev_mark_um) >= REVERSE_STEP_UM)
    {
        p_follow->rev_mark_um  = (int32_t)slow_um;
        p_follow->rev_stall_ms = 0U;
    }
    else
    {
        uint32_t const stall_ms = p_follow->rev_stall_ms + dt_ms;

        p_follow->rev_stall_ms = (uint16_t)stall_ms;
        b_blocked = (stall_ms >= LINE_REVERSE_STALL_MS);
    }

    return b_blocked;
}

/* The search. The heading target is the end of the current turn: first
 * LINE_SEARCH_SWEEP_DECI toward the side that saw the line last (left if none
 * did), then as far the other way. A failed search starts a retry: the car
 * turns back to the heading it had when the search began, reverses
 * LINE_REVERSE_MM straight and searches again. A reverse that is blocked ends
 * early and the search begins again at once. After LINE_RETRY_COUNT retries
 * the car stops. */
static void search_turn(line_follow_t * p_follow, int32_t heading_deci,
                        int64_t travel_um, uint32_t dt_ms)
{
    int32_t const sign = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -1 : 1;
    int32_t const past = sign * (heading_deci - p_follow->ref_deci);

    if (SWEEP_REVERSE == p_follow->sweep)
    {
        int64_t const back_um = p_follow->anchor_um - travel_um;

        if ((back_um >= REVERSE_UM) ||
            reverse_blocked(p_follow, back_um, heading_deci, dt_ms))
        {
            p_follow->sweep = SWEEP_NONE;
        }
    }
    if (SWEEP_NONE == p_follow->sweep)
    {
        if (0U == p_follow->retry)
        {
            p_follow->loss_cm = travel_cm(travel_um);
        }
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
        if (p_follow->retry < LINE_RETRY_COUNT)
        {
            /* The end of this turn is a sweep short of the heading at the
             * start of the search. */
            p_follow->retry++;
            p_follow->sweep     = SWEEP_TURN_BACK;
            p_follow->ref_deci += sign * LINE_SEARCH_SWEEP_DECI;
        }
        else
        {
            p_follow->status = (uint8_t)LINE_FOLLOW_STOPPED;
        }
    }
    else if ((SWEEP_TURN_BACK == p_follow->sweep) && (past >= 0))
    {
        /* The anchor is not used while the car searches: it marks where the
         * reverse began. */
        p_follow->sweep        = SWEEP_REVERSE;
        p_follow->anchor_um    = travel_um;
        p_follow->rev_mark_um  = 0;
        p_follow->rev_stall_ms = 0U;
    }
    else
    {
        /* Still turning or reversing. */
    }
}

/* Neither sensor sees the line. A short stretch is a crossing, a gap or the
 * line between the sensors: the car holds its heading. After LINE_GAP_MM (on a
 * hump LINE_HUMP_GAP_MM, the sensors lift off the floor at the crest) the line
 * is lost and the car searches on the spot. */
static void follow_blind(line_follow_t * p_follow, int32_t heading_deci,
                         int64_t travel_um, uint32_t dt_ms)
{
    int64_t const blind_um = abs_i64(travel_um - p_follow->anchor_um);
    int64_t const gap_um   = (0U != (p_follow->flags & LINE_FOLLOW_HUMP)) ? HUMP_GAP_UM : GAP_UM;

    if ((uint8_t)LINE_FOLLOW_FOLLOWING == p_follow->status)
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_BRIDGING;
    }
    if (((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status) && (blind_um >= gap_um))
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
        search_turn(p_follow, heading_deci, travel_um, dt_ms);
    }
    else
    {
        /* STOPPED: wait for a sensor hit. */
    }
}

/* A sensor sees the line during the search or a retry: follow from the
 * heading, not from the end of the search turn. The speed drops with the angle
 * the search turned, as for a heading error: a centred car finds the line at
 * once and keeps most of its speed, a corner restarts slow. A hit during the
 * reverse starts from the reverse speed, not from the speed before the loss. */
static void search_found(line_follow_t * p_follow, int32_t heading_deci)
{
    int32_t const sign = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -1 : 1;
    int32_t origin = p_follow->ref_deci;
    int32_t speed = 0;

    if (SWEEP_TOWARD == p_follow->sweep)
    {
        origin -= sign * LINE_SEARCH_SWEEP_DECI;
    }
    else if (SWEEP_BACK == p_follow->sweep)
    {
        origin += sign * LINE_SEARCH_SWEEP_DECI;
    }
    else
    {
        /* A retry turns back to the start heading, which is the target. */
    }
    if (p_follow->sweep < SWEEP_TURN_BACK)
    {
        /* A search turn found the line: the next loss has all its retries.
         * A reverse can only find the line the car came on. */
        p_follow->retry = 0U;
    }
    speed = corner_speed(origin, heading_deci);
    if ((SWEEP_REVERSE == p_follow->sweep) && (speed > LINE_SEARCH_UM_S))
    {
        speed = LINE_SEARCH_UM_S;
    }
    if (speed < (int32_t)(p_follow->speed_q * LINE_SPEED_STEP_UM_S))
    {
        p_follow->speed_q = (uint16_t)(speed / LINE_SPEED_STEP_UM_S);
    }
    p_follow->sweep    = SWEEP_NONE;
    p_follow->ref_deci = heading_deci;
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
        search_found(p_follow, heading_deci);
    }
    if ((b_left || b_right) && (0U != p_follow->retry) &&
        past_loss(p_follow->loss_cm, travel_um))
    {
        /* Following the line past the point where it was lost. */
        p_follow->retry = 0U;
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
        base_um_s = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;
    }
    else
    {
        /* Slow down at once, speed up at LINE_ACCEL_MM_S2. */
        int32_t const want = corner_speed(p_follow->ref_deci, heading_deci);
        int32_t const rise = (int32_t)(((int64_t)LINE_ACCEL_MM_S2 * UM_PER_MM * dt_ms) / MS_PER_S);
        int32_t const last = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;

        base_um_s = (want < (last + rise)) ? want : (last + rise);
    }
    p_follow->speed_q = (uint16_t)(base_um_s / LINE_SPEED_STEP_UM_S);
    if ((uint8_t)LINE_FOLLOW_STOPPED == p_follow->status)
    {
        set_targets(p_targets, 0, 0);
    }
    else if (((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status) &&
             (SWEEP_REVERSE == p_follow->sweep))
    {
        /* Straight back on the start heading, at the search speed. The wheel
         * difference that turns the car is the same in reverse. */
        steer_to(heading_deci, p_follow->ref_deci, LINE_SEARCH_UM_S, p_targets);
        set_targets(p_targets, -p_targets->left_um_s, -p_targets->right_um_s);
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

uint8_t line_watch_step(line_watch_t * p_watch, bool b_follow, bool b_hug,
                        line_follow_status_t status)
{
    uint8_t bits = 0U;

    if (b_hug)
    {
        if (0U != p_watch->b_lost)
        {
            bits = LINE_WATCH_QUIET;
        }
        p_watch->b_lost    = 0U;
        p_watch->b_stopped = 0U;
    }
    else if (b_follow)
    {
        bool const b_stopped = (LINE_FOLLOW_STOPPED == status);

        if (b_stopped && (0U == p_watch->b_stopped))
        {
            bits = LINE_WATCH_STOP;
        }
        p_watch->b_lost    = ((LINE_FOLLOW_SEARCHING == status) || b_stopped) ? 1U : 0U;
        p_watch->b_stopped = b_stopped ? 1U : 0U;
    }
    else
    {
        /* Safety has the wheels: the follower does not run, nothing changes. */
    }

    return bits;
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
