/** @file follow.c
 * @brief Continuous line following, speed limits and tape stripe reporting.
 *
 * A sensor hit slews a heading target toward the line. Encoder heading closes
 * the loop. A sustained correction can stop the inner wheel.
 */

#include "internal.h"

#define HEADING_ABS_MAX      (1000000000)
#define MS_PER_S             (1000)
#define UM_PER_M             (1000000)
#define FRONT_DETECTOR_MASK  (3U)
#define CORROBORATED_SHIFT   (2U)

int32_t line_heading_deci(int64_t left_um, int64_t right_um)
{
    int64_t const heading = wheel_yaw_deci(right_um - left_um);

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
    p_follow->progress = (line_follow_progress_t){0};
    p_follow->trail_count = 0U;
    p_follow->stripe_report = 0U;
}

void line_follow_set_hump(line_follow_t * p_follow, bool b_hump)
{
    if (b_hump)
    {
        p_follow->flags |= LINE_FOLLOW_HUMP;
        p_follow->flags &= (uint8_t)~(FOLLOW_STRIPE_RIGHT | FOLLOW_STRIPE_READY | FOLLOW_MARKER_RECENT);
    }
    else
    {
        p_follow->flags &= (uint8_t)~LINE_FOLLOW_HUMP;
    }
}

void line_follow_set_side(line_follow_t * p_follow, line_side_t side)
{
    p_follow->side    = (uint8_t)side;
    p_follow->progress = (line_follow_progress_t){0};
    p_follow->status  = (uint8_t)LINE_FOLLOW_FOLLOWING;
    p_follow->flags  &= (uint8_t)~(LINE_FOLLOW_STARTED | FOLLOW_STRIPE_RIGHT | FOLLOW_STRIPE_READY |
                                   FOLLOW_MARKER_RECENT | FOLLOW_LOCAL_SEARCH);
    p_follow->sweep   = SWEEP_NONE;
    p_follow->retry   = 0U;
    /* A turn on the spot changes the heading without travel. */
    p_follow->trail_count = 0U;
}

void line_follow_set_route_candidate(line_follow_t * p_follow, bool b_candidate)
{
    if (b_candidate)
    {
        p_follow->flags |= FOLLOW_ROUTE_CANDIDATE;
    }
    else
    {
        p_follow->flags &= (uint8_t)~FOLLOW_ROUTE_CANDIDATE;
    }
}

void line_follow_reanchor(line_follow_t * p_follow)
{
    /* A search in progress keeps its turn and its reverse start. */
    if ((uint8_t)LINE_FOLLOW_SEARCHING != p_follow->status)
    {
        p_follow->flags &= (uint8_t)~LINE_FOLLOW_STARTED;
    }
}

/* Slew the heading target by a rate (deci-degrees per second) over dt_ms, and
 * keep it within LINE_CAP_DECI of the heading. */
void line_follow_slew_ref(line_follow_t * p_follow, int32_t rate, uint32_t dt_ms,
                     int32_t heading_deci, int32_t cap_deci)
{
    int32_t const step = (int32_t)(((int64_t)rate * (int64_t)dt_ms) / MS_PER_S);

    p_follow->ref_deci = clamp_i32(p_follow->ref_deci + step,
                                   heading_deci - cap_deci,
                                   heading_deci + cap_deci);
}

/* Move the heading target toward the heading by rate * dt, without passing it. */
void line_follow_relax_ref(line_follow_t * p_follow, uint32_t dt_ms, int32_t heading_deci)
{
    int32_t const step = (int32_t)(((int64_t)LINE_RATE_RELAX * (int64_t)dt_ms) / MS_PER_S);
    int32_t const gap  = heading_deci - p_follow->ref_deci;

    p_follow->ref_deci += clamp_i32(gap, -step, step);
}

/* Base speed for a heading error: cruise at no error, the corner speed at the
 * cap, a straight line between. */
int32_t line_follow_corner_speed(int32_t ref_deci, int32_t heading_deci)
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
void line_follow_steer_to(int32_t ref_deci, int32_t heading_deci, int32_t base_um_s,
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

/* Start the pass: the first call after init or a turn takes the heading as the
 * target. Returns the time since the last pass, limited to LINE_DT_MAX_MS. */
static uint32_t begin_pass(line_follow_t * p_follow, int32_t heading_deci,
                           int64_t travel_um, uint32_t now_ms)
{
    uint32_t dt_ms = 0U;

    if (0U == (p_follow->flags & LINE_FOLLOW_STARTED))
    {
        p_follow->flags      &= (uint8_t)~BLIND_SETTLED;
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

/* Keep both detector entry positions: an outside wheel can be on the
 * approach tape while the inside detector starts crossing the outgoing ray. */
static void follow_stripe(line_follow_t * p_follow, bool b_left, bool b_right, int64_t travel_um)
{
    if ((p_follow->status == (uint8_t)LINE_FOLLOW_SEARCHING) ||
        (p_follow->status == (uint8_t)LINE_FOLLOW_STOPPED))
    {
        return;
    }
    uint8_t const on = (uint8_t)((b_left ? 1U : 0U) | (b_right ? 2U : 0U));
    uint8_t const changed = on ^ (p_follow->progress.stripe.detector_bits & FRONT_DETECTOR_MASK);
    uint8_t corroborated = ((uint32_t)p_follow->progress.stripe.detector_bits >> CORROBORATED_SHIFT) & FRONT_DETECTOR_MASK;
    uint8_t sustained = p_follow->progress.stripe.detector_bits & FOLLOW_SUSTAINED_EDGE;
    if ((p_follow->flags & LINE_FOLLOW_HUMP) != 0U)
    {
        p_follow->progress.stripe.detector_bits = 0U;
        p_follow->flags &= (uint8_t)~FOLLOW_MARKER_RECENT;
        return;
    }
    if (on != 0U)
    {
        uint8_t const side = (uint8_t)(b_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
        if ((b_left == b_right) || (p_follow->side != side) ||
            (abs_i64(travel_um - p_follow->anchor_um) > LINE_SENSOR_SPACING_MM * UM_PER_MM))
        {
            sustained = 0U;
        }
        p_follow->flags &= (uint8_t)~FOLLOW_STRIPE_READY;
    }
    uint16_t const mm = (uint16_t)(travel_um / UM_PER_MM);
    if ((uint8_t)((uint8_t)mm - p_follow->progress.stripe.marker_mm) > LINE_SENSOR_SPACING_MM)
    {
        p_follow->flags &= (uint8_t)~FOLLOW_MARKER_RECENT;
    }
    for (unsigned i = 0U; i < LINE_SENSOR_COUNT; ++i)
    {
        uint8_t const mask = (uint8_t)(1U << i);
        if ((changed & mask) == 0U)
        {
            continue;
        }
        if ((on & mask) != 0U)
        {
            p_follow->progress.stripe.begin_mm[i] = mm;
            corroborated &= (uint8_t)~mask;
        }
        else
        {
            uint16_t const width = (uint16_t)(mm - p_follow->progress.stripe.begin_mm[i]);
            uint16_t const limit = (corroborated & mask) != 0U ?
                                   ROUTE_MAX_CROSS_MM : ROUTE_STRIPE_MAX_MM;
            if ((width >= SINGLE_STRIPE_MIN_MM) && (width <= limit))
            {
                p_follow->stripe_report |= (uint8_t)(((corroborated & mask) != 0U ? LINE_STRIPE_CORROBORATED : 0U) |
                                                     (i == 1U ? LINE_STRIPE_RIGHT : LINE_STRIPE_LEFT));
                if (((p_follow->flags & FOLLOW_STRIPE_READY) == 0U) ||
                    (p_follow->side == (uint8_t)LINE_SIDE_RIGHT))
                {
                    p_follow->progress.stripe.anchor_mm = p_follow->progress.stripe.begin_mm[i];
                    p_follow->flags &= (uint8_t)~FOLLOW_STRIPE_RIGHT;
                    p_follow->flags |= (uint8_t)(FOLLOW_STRIPE_READY | (i == 1U ? FOLLOW_STRIPE_RIGHT : 0U));
                }
            }
        }
    }
    if (b_left && b_right)
    {
        corroborated = on;
    }
    else if (b_left && ((p_follow->flags & FOLLOW_MARKER_RECENT) != 0U))
    {
        /* The left detector may graze the incoming line before crossing the
         * side ray. A nearby reading on the left barcode channel corroborates
         * that optical interval even if the right detector stays clear. */
        corroborated |= 1U;
    }
    uint16_t const contact_mm = (uint16_t)(mm - p_follow->progress.stripe.begin_mm[b_left ? 0U : 1U]);
    if ((b_left != b_right) && (contact_mm >= LINE_SENSOR_SPACING_MM) && (contact_mm <= INT16_MAX))
    {
        sustained = FOLLOW_SUSTAINED_EDGE;
    }
    p_follow->progress.stripe.detector_bits = (uint8_t)((uint32_t)on | sustained | ((uint32_t)corroborated << CORROBORATED_SHIFT));
}

uint8_t line_follow_take_stripe(line_follow_t * p_follow)
{
    bool const b_tracking = (p_follow->status == (uint8_t)LINE_FOLLOW_FOLLOWING) ||
                            (p_follow->status == (uint8_t)LINE_FOLLOW_BRIDGING);
    /* Both detectors of one stripe report together once the tape has passed. */
    if (b_tracking && ((p_follow->progress.stripe.detector_bits & FRONT_DETECTOR_MASK) != 0U))
    {
        return 0U;
    }
    uint8_t const report = p_follow->stripe_report;
    p_follow->stripe_report = 0U;
    return report;
}

bool line_follow_marker_sample(line_follow_t * p_follow, bool b_left, bool b_right, bool b_progress,
                               uint8_t prefix_mm, int32_t heading_deci, int64_t travel_um, uint32_t now_ms, line_targets_t * p_targets)
{
    if ((b_left != b_right) || ((p_follow->status != (uint8_t)LINE_FOLLOW_FOLLOWING) &&
        (p_follow->status != (uint8_t)LINE_FOLLOW_BRIDGING)))
    {
        return false;
    }
    if (!b_left && !b_progress && (0U != (p_follow->flags & LINE_FOLLOW_STARTED)) &&
        (((p_follow->progress.stripe.detector_bits & FOLLOW_SUSTAINED_EDGE) != 0U) ||
         (abs_i64(travel_um - p_follow->anchor_um) >= (LINE_PROBE_FROM_MM + prefix_mm) * UM_PER_MM)))
    {
        return false;
    }
    (void)begin_pass(p_follow, heading_deci, travel_um, now_ms);
    if (b_left || b_progress)
    {
        p_follow->anchor_um = travel_um;
    }
    p_follow->ref_deci = heading_deci;
    p_follow->flags &= (uint8_t)~BLIND_SETTLED;
    p_follow->progress.stripe.marker_mm = (uint8_t)(travel_um / UM_PER_MM);
    p_follow->flags |= FOLLOW_MARKER_RECENT;
    follow_stripe(p_follow, b_left, b_right, travel_um);
    int32_t const speed = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;
    set_targets(p_targets, speed, speed);
    return true;
}

static void follow_detector(line_follow_t * p_follow, bool b_left, int32_t heading_deci,
                            uint32_t dt_ms, bool b_started)
{
    /* A short gap between bends can leave the outer wheel below cruise. */
    int32_t const last = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;
    int32_t const cap = line_follow_corner_speed(p_follow->ref_deci, heading_deci);
    int32_t const rate = (int32_t)(((int64_t)LINE_HIT_DECI_PER_M *
                                    ((last < cap) ? last : cap)) /
                                   UM_PER_M);

    /* Start a new correction from the measured heading when the old
     * target points away from the tape. */
    if ((b_left && p_follow->ref_deci < heading_deci) || (!b_left && p_follow->ref_deci > heading_deci))
    {
        p_follow->ref_deci = heading_deci;
    }
    line_follow_slew_ref(p_follow, b_left ? rate : -rate, dt_ms, heading_deci, LINE_CAP_DECI);
    if (b_started && (p_follow->ref_deci == heading_deci))
    {
        /* Two sensor samples can share one 10 ms kernel tick. */
        p_follow->ref_deci += b_left ? 1 : -1;
    }
}

line_follow_status_t line_follow_update_floor(line_follow_t * p_follow, bool b_left, bool b_right,
                                              int32_t heading_deci, int64_t travel_um, uint32_t now_ms,
                                              bool b_continuous_floor, line_targets_t * p_targets)
{
    if (line_recovery_local_stopped(p_follow, heading_deci, now_ms))
    {
        set_targets(p_targets, 0, 0);
        return LINE_FOLLOW_STOPPED;
    }
    bool const b_started = (p_follow->flags & LINE_FOLLOW_STARTED) != 0U;
    uint32_t const dt_ms = begin_pass(p_follow, heading_deci, travel_um, now_ms);
    bool const b_hit = b_left || b_right;
    int32_t base_um_s = 0;
    int32_t probe_deci = 0;
    follow_stripe(p_follow, b_left, b_right, travel_um);

    if (b_hit)
    {
        p_follow->flags &= (uint8_t)~BLIND_SETTLED;
        if ((SWEEP_NONE != p_follow->sweep) || line_follow_local_search(p_follow))
        {
            line_recovery_found(p_follow, heading_deci);
        }
        p_follow->flags &= (uint8_t)~FOLLOW_LOCAL_SEARCH;
    }
    if (b_hit && (0U != p_follow->retry) &&
        line_recovery_past_loss(p_follow->loss_cm, travel_um))
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
    else if (b_hit)
    {
        p_follow->side      = (uint8_t)(b_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
        p_follow->anchor_um = travel_um;
        p_follow->status    = (uint8_t)LINE_FOLLOW_FOLLOWING;
        follow_detector(p_follow, b_left, heading_deci, dt_ms, b_started);
    }
    else
    {
        probe_deci = line_recovery_blind(p_follow, heading_deci, travel_um, dt_ms, b_continuous_floor);
    }

    if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        /* Keep the speed from before the search for the restart. */
        base_um_s = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;
    }
    else
    {
        /* Slow down at once, speed up at LINE_ACCEL_MM_S2. */
        int32_t const want = line_follow_corner_speed(p_follow->ref_deci, heading_deci);
        int32_t const rise = (int32_t)(((int64_t)LINE_ACCEL_MM_S2 * UM_PER_MM * dt_ms) / MS_PER_S);
        int32_t const last = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;

        base_um_s = (want < (last + rise)) ? want : (last + rise);
    }
    p_follow->speed_q = (uint16_t)(base_um_s / LINE_SPEED_STEP_UM_S);
    if (((uint8_t)LINE_FOLLOW_FOLLOWING == p_follow->status) ||
        ((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status))
    {
        line_recovery_trail_record(p_follow, heading_deci, travel_um);
    }
    if ((uint8_t)LINE_FOLLOW_STOPPED == p_follow->status)
    {
        set_targets(p_targets, 0, 0);
    }
    else if (((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status) &&
             (SWEEP_REVERSE == p_follow->sweep))
    {
        /* Back along the recorded path, at the search speed. The wheel
         * difference that turns the car is the same in reverse. */
        line_follow_steer_to(heading_deci, p_follow->ref_deci, LINE_SEARCH_UM_S, p_targets);
        set_targets(p_targets, -p_targets->left_um_s, -p_targets->right_um_s);
    }
    else if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        /* On the spot: counter-clockwise when the target is to the left. */
        int32_t const spin = line_recovery_spin(p_follow, heading_deci);

        set_targets(p_targets, -spin, spin);
    }
    else
    {
        line_follow_steer_to(p_follow->ref_deci + probe_deci, heading_deci, base_um_s, p_targets);
    }

    return (line_follow_status_t)p_follow->status;
}

line_follow_status_t line_follow_update(line_follow_t * p_follow, bool b_left, bool b_right,
                                        int32_t heading_deci, int64_t travel_um, uint32_t now_ms,
                                        line_targets_t * p_targets)
{
    return line_follow_update_floor(p_follow, b_left, b_right, heading_deci, travel_um, now_ms, false, p_targets);
}
