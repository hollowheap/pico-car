/** @file recovery.c
 * @brief Line-loss bridging, optical search and recovery along recorded travel.
 */

#include "internal.h"

/* Other line_follow_t.sweep phases remain local to recovery. */
#define SWEEP_TOWARD         (1U)
#define SWEEP_BACK           (2U)
#define SWEEP_TURN_BACK      (3U)
#define SWEEP_LOCAL_MISSED   (5U)
#define SWEEP_DIRECTIONS     (2)

#define GAP_UM               ((int64_t)LINE_GAP_MM * UM_PER_MM)
#define HUMP_GAP_UM          ((int64_t)LINE_HUMP_GAP_MM * UM_PER_MM)
#define REVERSE_UM           ((int64_t)LINE_REVERSE_MM * UM_PER_MM)

#define UM_PER_CM            (10000)
#define CM_MASK              (0xFFU)
#define CM_HALF              (0x80U)
#define REVERSE_STEP_UM      ((int64_t)LINE_REVERSE_STEP_MM * UM_PER_MM)
#define TRAIL_STEP_UM        ((int64_t)LINE_TRAIL_STEP_MM * UM_PER_MM)
/* Trial from the closed-loop straight replay. Leave most of the 150 mm
 * bridge for relaxed steering, so centered tape can come into view again. */
#define BLIND_HOLD_UM        (60000)

/* Heading change needed to move one sensor across the full centered blind
 * strip. The existing loss distance still limits forward travel. */
#define PROBE_ANGLE_DECI     (((LINE_SENSOR_SPACING_MM - LINE_WIDTH_MM) * DECI_PER_RAD) / LINE_SENSOR_LEAD_MM)

/* The simulated wheels need 45 mm/s to start, and conversion through the
 * commanded yaw rate rounds down. Recheck the 50 mm/s floor on the car. */
#define LOCAL_SPIN_MIN_UM_S  (50000)

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
bool line_recovery_past_loss(uint8_t loss_cm, int64_t travel_um)
{
    uint8_t const ahead = (uint8_t)(travel_cm(travel_um) - loss_cm);

    return (0U != ahead) && (ahead < CM_HALF);
}

void line_follow_start_search(line_follow_t * p_follow)
{
    p_follow->status  = (uint8_t)LINE_FOLLOW_SEARCHING;
    p_follow->flags  &= (uint8_t)~(LINE_FOLLOW_STARTED | FOLLOW_STRIPE_RIGHT | FOLLOW_STRIPE_READY |
                                   FOLLOW_MARKER_RECENT | FOLLOW_LOCAL_SEARCH);
    p_follow->sweep   = SWEEP_NONE;
    p_follow->retry   = 0U;
    p_follow->trail_count = 0U;
}

bool line_follow_local_search(line_follow_t const * p_follow)
{
    return (p_follow->flags & FOLLOW_LOCAL_SEARCH) != 0U;
}

void line_follow_start_local_search(line_follow_t * p_follow)
{
    line_follow_start_search(p_follow);
    p_follow->flags |= FOLLOW_LOCAL_SEARCH;
}

bool line_follow_local_missed(line_follow_t const * p_follow)
{
    return line_follow_local_search(p_follow) && (p_follow->sweep == SWEEP_LOCAL_MISSED);
}

void line_follow_stop_local_search(line_follow_t * p_follow)
{
    if (line_follow_local_search(p_follow))
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_STOPPED;
        p_follow->sweep = SWEEP_NONE;
    }
}

static int32_t trail_step(int64_t travel_um)
{
    int64_t step = travel_um / TRAIL_STEP_UM;

    if ((travel_um < 0) && ((step * TRAIL_STEP_UM) != travel_um))
    {
        step--;
    }
    return (int32_t)step;
}

static uint32_t trail_slot(int32_t step)
{
    int32_t const slots = (int32_t)LINE_TRAIL_SLOTS;

    return (uint32_t)(((step % slots) + slots) % slots);
}

/* Record the heading where the car drives forward into a step. Travel back
 * over recorded steps forgets the steps ahead of the car. */
void line_recovery_trail_record(line_follow_t * p_follow, int32_t heading_deci, int64_t travel_um)
{
    int32_t const step = trail_step(travel_um);
    int32_t const oldest = p_follow->trail_step - (int32_t)p_follow->trail_count;

    if ((0U != p_follow->trail_count) && (step == p_follow->trail_step))
    {
        return;
    }
    if ((0U == p_follow->trail_count) || (step > (p_follow->trail_step + 1)) || (step <= oldest))
    {
        p_follow->trail_count = 1U;
    }
    else if (step == (p_follow->trail_step + 1))
    {
        if (p_follow->trail_count < LINE_TRAIL_SLOTS)
        {
            p_follow->trail_count++;
        }
    }
    else
    {
        p_follow->trail_count = (uint8_t)(step - oldest);
    }
    p_follow->trail_step = step;
    p_follow->trail_deci[trail_slot(step)] = heading_deci;
}

static bool trail_known(line_follow_t const * p_follow, int32_t step)
{
    return (0U != p_follow->trail_count) && (step <= p_follow->trail_step) &&
           (step > (p_follow->trail_step - (int32_t)p_follow->trail_count));
}

/* The recorded heading at a travel, interpolated between step entries. */
static bool trail_heading(line_follow_t const * p_follow, int64_t travel_um, int32_t * p_heading_deci)
{
    int32_t const step = trail_step(travel_um);
    bool const b_known = trail_known(p_follow, step);

    if (b_known)
    {
        int32_t const here = p_follow->trail_deci[trail_slot(step)];
        int32_t heading = here;

        if (trail_known(p_follow, step + 1))
        {
            int64_t const into_um = travel_um - ((int64_t)step * TRAIL_STEP_UM);
            int64_t const change = (int64_t)p_follow->trail_deci[trail_slot(step + 1)] - here;

            heading += (int32_t)((change * into_um) / TRAIL_STEP_UM);
        }
        *p_heading_deci = heading;
    }
    return b_known;
}

static int32_t search_extent(line_follow_t const * p_follow)
{
    return line_follow_local_search(p_follow) ? LINE_LOCAL_SEARCH_DECI : LINE_SEARCH_SWEEP_DECI;
}

static int32_t search_origin(line_follow_t const * p_follow)
{
    int32_t const sign = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -1 : 1;
    int32_t origin = p_follow->ref_deci;

    if (SWEEP_TOWARD == p_follow->sweep)
    {
        origin -= sign * search_extent(p_follow);
    }
    else if (SWEEP_BACK == p_follow->sweep)
    {
        origin += sign * search_extent(p_follow);
    }
    else
    {
        /* A retry turns back to the start heading, which is the target. */
    }
    return origin;
}

static bool reverse_blocked(line_follow_t * p_follow, int64_t back_um,
                            int32_t heading_deci, uint32_t dt_ms)
{
    int64_t const off_deci = (int64_t)heading_deci - p_follow->ref_deci;
    int64_t const yaw_um = (abs_i64(off_deci) * TRACK_UM) / (2 * DECI_PER_RAD);
    int64_t const slow_um = back_um - yaw_um;
    bool b_blocked = false;

    if ((slow_um - p_follow->progress.reverse.mark_um) >= REVERSE_STEP_UM)
    {
        p_follow->progress.reverse.mark_um  = (int32_t)slow_um;
        p_follow->progress.reverse.stall_ms = 0U;
    }
    else
    {
        uint32_t const stall_ms = p_follow->progress.reverse.stall_ms + dt_ms;

        p_follow->progress.reverse.stall_ms = (uint16_t)stall_ms;
        b_blocked = (stall_ms >= LINE_REVERSE_STALL_MS);
    }

    return b_blocked;
}

/* The search. The heading target is the end of the current turn: first
 * LINE_SEARCH_SWEEP_DECI toward the side that saw the line last (left if none
 * did), then as far the other way. A failed search starts a retry: the car
 * turns back to the heading it had when the search began, reverses
 * LINE_REVERSE_MM along the recorded path and searches again. A reverse that is blocked ends
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

        /* Without a record the target stays the heading at the search start. */
        (void)trail_heading(p_follow, travel_um, &p_follow->ref_deci);
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
        p_follow->ref_deci = heading_deci + (sign * search_extent(p_follow));
    }
    else if ((SWEEP_TOWARD == p_follow->sweep) && (past >= 0))
    {
        p_follow->sweep    = SWEEP_BACK;
        p_follow->ref_deci -= sign * SWEEP_DIRECTIONS * search_extent(p_follow);
    }
    else if ((SWEEP_BACK == p_follow->sweep) && (past <= 0))
    {
        if (p_follow->retry < LINE_RETRY_COUNT)
        {
            /* The end of this turn is a sweep short of the heading at the
             * start of the search. */
            if (!line_follow_local_search(p_follow))
            {
                p_follow->retry++;
            }
            p_follow->sweep     = SWEEP_TURN_BACK;
            p_follow->ref_deci += sign * search_extent(p_follow);
        }
        else
        {
            p_follow->status = (uint8_t)LINE_FOLLOW_STOPPED;
        }
    }
    else if ((SWEEP_TURN_BACK == p_follow->sweep) && (past >= 0))
    {
        if (line_follow_local_search(p_follow))
        {
            line_follow_stop_local_search(p_follow);
            if (past <= PROBE_ANGLE_DECI)
            {
                p_follow->sweep = SWEEP_LOCAL_MISSED;
            }
            return;
        }
        /* The anchor is not used while the car searches: it marks where the
         * reverse began. */
        p_follow->sweep        = SWEEP_REVERSE;
        p_follow->anchor_um    = travel_um;
        p_follow->progress.reverse.mark_um  = 0;
        p_follow->progress.reverse.stall_ms = 0U;
    }
    else
    {
        /* Still turning or reversing. */
    }
}

/* Neither sensor sees the line. On a settled flat approach, turn the front
 * sensors toward the last sighting while advancing. Hold the reference so
 * the probe has a bounded angle even if neither sensor then sees tape.
 * Search at the axle projection of a transverse stripe. A bounded course
 * also searches at the normal gap distance. Ignore backwards travel when the millimeter
 * counter wraps. */
int32_t line_recovery_blind(line_follow_t * p_follow, int32_t heading_deci,
                            int64_t travel_um, uint32_t dt_ms, bool b_continuous_floor)
{
    int64_t const blind_um = abs_i64(travel_um - p_follow->anchor_um);
    int64_t const gap_um   = (0U != (p_follow->flags & LINE_FOLLOW_HUMP)) ? HUMP_GAP_UM : GAP_UM;
    int32_t probe_deci = 0;
    bool b_at_corner = false;
    bool b_corner_near = false;
    if ((p_follow->flags & (FOLLOW_STRIPE_READY | FOLLOW_ROUTE_CANDIDATE)) == FOLLOW_STRIPE_READY)
    {
        uint16_t const since = (uint16_t)((uint16_t)(travel_um / UM_PER_MM) - p_follow->progress.stripe.anchor_mm);
        b_at_corner = (since >= LINE_SENSOR_LEAD_MM + LINE_WIDTH_MM / ROUTE_MIDPOINT_DIVISOR) &&
                      (since <= INT16_MAX);
        b_corner_near = (since >= LINE_SENSOR_LEAD_MM + LINE_WIDTH_MM / ROUTE_MIDPOINT_DIVISOR -
                                  (LINE_GAP_MM - LINE_PROBE_FROM_MM)) &&
                        (since <= INT16_MAX);
    }

    if ((uint8_t)LINE_FOLLOW_FOLLOWING == p_follow->status)
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_BRIDGING;
    }
    bool const b_bend = b_continuous_floor &&
        ((p_follow->flags & (LINE_FOLLOW_HUMP | FOLLOW_ROUTE_CANDIDATE | FOLLOW_STRIPE_READY)) == 0U) &&
        ((p_follow->progress.stripe.detector_bits & FOLLOW_SUSTAINED_EDGE) != 0U);
    if (((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status) &&
        (b_at_corner || (b_bend && blind_um >= LINE_SENSOR_LEAD_MM * UM_PER_MM) ||
         (!b_continuous_floor && blind_um >= gap_um)))
    {
        if (b_at_corner)
        {
            p_follow->side = (uint8_t)((p_follow->flags & FOLLOW_STRIPE_RIGHT) ? LINE_SIDE_RIGHT : LINE_SIDE_LEFT);
        }
        p_follow->flags &= (uint8_t)~(FOLLOW_STRIPE_READY | FOLLOW_MARKER_RECENT);
        p_follow->status = (uint8_t)LINE_FOLLOW_SEARCHING;
    }
    if ((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status)
    {
        if (b_bend && (blind_um <= LINE_SENSOR_SPACING_MM * UM_PER_MM))
        {
            /* The tape can pass between the detectors mid-bend. Carry the
             * last steering direction for at most one detector spacing. */
            int32_t const side = (p_follow->side == (uint8_t)LINE_SIDE_LEFT) ? 1 : -1;
            int32_t const error = clamp_i32(side * (p_follow->ref_deci - heading_deci), LINE_SLOW_DECI, LINE_CAP_DECI);
            p_follow->ref_deci = heading_deci + side * error;
            return 0;
        }
        if (b_continuous_floor && ((p_follow->side == (uint8_t)LINE_SIDE_NONE) || blind_um >= gap_um))
        {
            return 0;
        }
        /* Settle the previous correction first. Keep that heading through
         * the early gap so steady wheel mismatch cannot pull the target away.
         * A stripe searches at its axle projection, before the gap ends, so
         * the probe also starts the same distance before that point. */
        if (((blind_um >= LINE_PROBE_FROM_MM * UM_PER_MM) || b_corner_near) &&
            ((p_follow->flags & (BLIND_SETTLED | LINE_FOLLOW_HUMP)) == BLIND_SETTLED))
        {
            probe_deci = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -PROBE_ANGLE_DECI : PROBE_ANGLE_DECI;
        }
        else if ((0U == (p_follow->flags & BLIND_SETTLED)) || (blind_um > BLIND_HOLD_UM))
        {
            line_follow_relax_ref(p_follow, dt_ms, heading_deci);
            if (p_follow->ref_deci == heading_deci)
            {
                p_follow->flags |= BLIND_SETTLED;
            }
        }
        line_follow_slew_ref(p_follow, 0, dt_ms, heading_deci, LINE_CAP_DECI);
    }
    else if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        uint8_t const previous = p_follow->sweep;
        search_turn(p_follow, heading_deci, travel_um, dt_ms);
        if (b_continuous_floor && !line_follow_local_search(p_follow) && (p_follow->sweep == SWEEP_REVERSE))
        {
            /* The optical sweep has returned to the approach heading. */
            p_follow->sweep = SWEEP_NONE;
            p_follow->retry = 0U;
            p_follow->progress = (line_follow_progress_t){0};
            p_follow->side = (uint8_t)LINE_SIDE_NONE;
            p_follow->status = (uint8_t)LINE_FOLLOW_BRIDGING;
        }
        if (line_follow_local_search(p_follow) && (previous != p_follow->sweep))
        {
            /* Reversing the sweep starts a fresh progress window. */
            p_follow->progress.local.heading_deci = heading_deci;
            p_follow->progress.local.progress_ms = p_follow->last_ms;
        }
    }
    else
    {
        /* STOPPED: wait for a sensor hit. */
    }
    return probe_deci;
}

/* A sensor sees the line during the search or a retry: follow from the
 * heading, not from the end of the search turn. The speed drops with the angle
 * the search turned, as for a heading error: a centred car finds the line at
 * once and keeps most of its speed, a corner restarts slow. A hit during the
 * reverse starts from the reverse speed, not from the speed before the loss. */
void line_recovery_found(line_follow_t * p_follow, int32_t heading_deci)
{
    int32_t const origin = search_origin(p_follow);
    int32_t speed = 0;

    if (p_follow->sweep < SWEEP_TURN_BACK)
    {
        /* A search turn found the line: the next loss has all its retries.
         * A reverse can only find the line the car came on. */
        p_follow->retry = 0U;
    }
    speed = line_follow_corner_speed(origin, heading_deci);
    if ((SWEEP_REVERSE == p_follow->sweep) && (speed > LINE_SEARCH_UM_S))
    {
        speed = LINE_SEARCH_UM_S;
    }
    if (speed < (int32_t)(p_follow->speed_q * LINE_SPEED_STEP_UM_S))
    {
        p_follow->speed_q = (uint16_t)(speed / LINE_SPEED_STEP_UM_S);
    }
    p_follow->sweep    = SWEEP_NONE;
    p_follow->progress = (line_follow_progress_t){0};
    p_follow->ref_deci = heading_deci;
}

/* A heading improvement sufficient to move a detector across the centered
 * blind strip renews the stall window. Jitter and motion away from the
 * active endpoint leave the original progress clock running. */
bool line_recovery_local_stopped(line_follow_t * p_follow, int32_t heading_deci, uint32_t now_ms)
{
    if (!line_follow_local_search(p_follow))
    {
        return false;
    }
    if (p_follow->status == (uint8_t)LINE_FOLLOW_STOPPED)
    {
        return true;
    }
    if ((p_follow->flags & LINE_FOLLOW_STARTED) != 0U)
    {
        int64_t const previous = abs_i64((int64_t)p_follow->ref_deci - p_follow->progress.local.heading_deci);
        int64_t const remaining = abs_i64((int64_t)p_follow->ref_deci - heading_deci);

        int64_t const from_entry = abs_i64((int64_t)heading_deci - search_origin(p_follow));

        if ((from_entry > (LINE_LOCAL_SEARCH_DECI + PROBE_ANGLE_DECI)) ||
            ((now_ms - p_follow->progress.local.progress_ms) >= LINE_LOCAL_SEARCH_STALL_MS))
        {
            line_follow_stop_local_search(p_follow);
            return true;
        }
        if ((previous - remaining) >= PROBE_ANGLE_DECI)
        {
            p_follow->progress.local.heading_deci = heading_deci;
            p_follow->progress.local.progress_ms = now_ms;
        }
    }
    return false;
}

/* Slow near each local endpoint to limit coast beyond the surveyed angle.
 * The trial minimum keeps the motor turning at low error. */
int32_t line_recovery_spin(line_follow_t const * p_follow, int32_t heading_deci)
{
    int32_t speed = LINE_SEARCH_UM_S;

    if (line_follow_local_search(p_follow))
    {
        int64_t const err = abs_i64((int64_t)p_follow->ref_deci - heading_deci);
        int64_t const target = (err * LINE_KH * TRACK_UM) / (SWEEP_DIRECTIONS * DECI_PER_RAD);

        speed = target < LINE_SEARCH_UM_S ? (int32_t)target : LINE_SEARCH_UM_S;
        if (speed < LOCAL_SPIN_MIN_UM_S)
        {
            speed = LOCAL_SPIN_MIN_UM_S;
        }
    }
    return (p_follow->ref_deci > heading_deci) ? speed : -speed;
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
