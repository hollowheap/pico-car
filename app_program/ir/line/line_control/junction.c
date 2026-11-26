/** @file junction.c
 * @brief Commanded junction selection and encoder-guided turns.
 *
 * A measured stripe projects the crossing to the drive axle. A turn pivots
 * about the axle and uses tape observations to align with the exit.
 */

#include "internal.h"

#define HEADING_CLAMP_DECI   (100000)

/* Turn phases (line_turn_state_t.phase). IDLE is zero so that a zeroed state is
 * safe. */
#define TURN_PHASE_IDLE      (0U)
#define TURN_PHASE_FAST      (1U)
#define TURN_PHASE_SLOW      (2U)
#define TURN_PHASE_DONE      (3U)
#define TURN_PHASE_ABORTED   (4U)

#define ROUTE_FULL_STRIPE_MIN_MM (LINE_WIDTH_MM - 1)
#define ROUTE_STRAIGHT_DECI      (150)
#define ROUTE_BRANCH_MIN_DECI    (600)
#define ROUTE_BRANCH_MAX_DECI    (1200)
#define ROUTE_PROBE_SPEED_UM_S   LINE_SEARCH_UM_S

typedef enum
{
    ROUTE_WAIT_EDGE = 0,
    ROUTE_ON_EDGE,
    ROUTE_APPROACH,
    ROUTE_PIVOT_START,
    ROUTE_PROBE_OUT,
    ROUTE_REVISIT_THROUGH,
    ROUTE_PROBE_OPPOSITE,
    ROUTE_RECHECK_BRANCH,
    ROUTE_ALIGN_BRANCH,
    ROUTE_ALIGN_OPPOSITE,
    ROUTE_ALIGN_THROUGH,
    ROUTE_RESTORE
} route_phase_t;

static bool route_paired_stripe_width(uint16_t width_mm)
{
    /* Steering can keep the left detector on a transverse stripe longer.
     * The offset barcode reader also has to see tape for this measurement. */
    return (width_mm >= LINE_WIDTH_MM / ROUTE_MIDPOINT_DIVISOR) && (width_mm <= ROUTE_MAX_CROSS_MM);
}

/* Associate the two left optical hits over a short travelled interval. The
 * readers need not be mounted at exactly the same fore/aft station. */
static bool route_left_crossing(line_route_t * p_route, uint16_t mm, bool b_left, bool b_side_tape)
{
    if (b_left)
    {
        if (!p_route->left_on_tape)
        {
            p_route->left_begin_mm = mm;
        }
        p_route->left_at_mm = mm;
    }
    if (b_side_tape)
    {
        p_route->side_at_mm = mm;
    }
    p_route->left_on_tape = b_left;
    p_route->left_recent = b_left || (p_route->left_recent &&
                          ((uint16_t)(mm - p_route->left_at_mm) <= LINE_SENSOR_SPACING_MM));
    p_route->side_recent = b_side_tape || (p_route->side_recent &&
                          ((uint16_t)(mm - p_route->side_at_mm) <= LINE_SENSOR_SPACING_MM));
    return p_route->left_recent && p_route->side_recent;
}

/* Measure the right detector's completed pulse. On its own it must cross
 * a full tape width within whole-mm rounding. The other detectors can
 * corroborate a shorter crossing that could otherwise be a grazing hit. */
static uint16_t route_right_stripe(line_route_t * p_route, uint16_t mm, bool b_right)
{
    if (b_right && !p_route->right_on_tape)
    {
        p_route->right_begin_mm = mm;
    }
    uint16_t const width = (uint16_t)(mm - p_route->right_begin_mm);
    bool const b_ended = !b_right && p_route->right_on_tape;
    p_route->right_on_tape = b_right;
    uint16_t const left_width = (uint16_t)(p_route->left_at_mm - p_route->left_begin_mm);
    bool const b_left_stripe = p_route->left_recent && (left_width >= ROUTE_FULL_STRIPE_MIN_MM) &&
                               (left_width <= ROUTE_STRIPE_MAX_MM);
    bool const b_supported = (width >= ROUTE_FULL_STRIPE_MIN_MM) || p_route->side_recent || b_left_stripe;
    return (b_ended && b_supported && width >= SINGLE_STRIPE_MIN_MM && width <= ROUTE_STRIPE_MAX_MM) ? width : 0U;
}

/* Use the front detector's stripe midpoint. The barcode reader only
 * corroborates the stripe because its fore/aft station can differ. */
static void route_anchor_left(line_route_t * p_route, int64_t travel_um, int32_t heading_deci,
                              uint16_t since_mm, uint16_t width_mm)
{
    p_route->start_um = travel_um - ((int64_t)since_mm + width_mm / ROUTE_MIDPOINT_DIVISOR) * UM_PER_MM;
    p_route->heading_deci = heading_deci;
    p_route->phase = ROUTE_APPROACH;
    p_route->full_width_anchor = width_mm >= ROUTE_FULL_STRIPE_MIN_MM;
    p_route->right_anchor = false;
}

/* A turn needs evidence of two routes. A hit alone is also consistent with
 * a bend. Let the follower steer during the approach to the candidate. */
bool line_route_observe(line_route_t * p_route, line_turn_kind_t kind, bool b_left, bool b_right,
                        bool b_side_tape, int64_t left_um, int64_t right_um)
{
    int64_t const travel = (left_um + right_um) / 2;
    int32_t const heading = line_heading_deci(left_um, right_um);
    int64_t const progress = travel - p_route->start_um;
    int32_t const yaw = heading - p_route->heading_deci;
    uint16_t const mm = (uint16_t)(travel / UM_PER_MM);
    bool const b_crossing = route_left_crossing(p_route, mm, b_left, b_side_tape);
    uint16_t const right_width = route_right_stripe(p_route, mm, b_right);
    uint16_t const left_width = (uint16_t)(p_route->left_at_mm - p_route->left_begin_mm);
    uint16_t const since_left = (uint16_t)(mm - p_route->left_at_mm);

    /* Keep a full-width anchor when only one reader revisits the tape.
     * A simultaneous hit can establish a better anchor on the left. */
    bool const b_refine = (ROUTE_APPROACH == p_route->phase) &&
                          (!p_route->full_width_anchor || (b_left && b_side_tape));
    if (((ROUTE_WAIT_EDGE == p_route->phase) || b_refine) && b_crossing && !p_route->was_crossing && (0U == right_width))
    {
        p_route->start_um = travel;
        p_route->heading_deci = heading;
        p_route->phase = ROUTE_ON_EDGE;
        p_route->full_width_anchor = false;
        p_route->right_branch = LINE_TURN_KIND_RIGHT == kind;
        p_route->kind = (uint8_t)kind;
        if (!b_left && (left_width >= ROUTE_FULL_STRIPE_MIN_MM) && route_paired_stripe_width(left_width))
        {
            route_anchor_left(p_route, travel, heading, since_left, left_width);
        }
    }
    else if (((ROUTE_WAIT_EDGE == p_route->phase) || (ROUTE_ON_EDGE == p_route->phase)) && (0U != right_width))
    {
        p_route->start_um = travel - (int64_t)right_width * UM_PER_MM / ROUTE_MIDPOINT_DIVISOR;
        p_route->heading_deci = heading;
        p_route->phase = ROUTE_APPROACH;
        p_route->full_width_anchor = right_width >= ROUTE_FULL_STRIPE_MIN_MM;
        p_route->right_anchor = true;
        /* The left detector can stay on the approach tape into a T. The
         * right detector's clean crossing still fixes the axle position. */
        p_route->right_branch = (LINE_TURN_KIND_RIGHT == kind) ||
                                (((LINE_TURN_KIND_STRAIGHT == kind) || (LINE_TURN_KIND_UTURN == kind)) && !b_crossing);
        p_route->kind = (uint8_t)kind;
    }
    /* Retire the projected axle position if steering moves it out of the crossing. */
    else if ((progress < 0) || ((ROUTE_APPROACH == p_route->phase) && (abs_i64(yaw) > LINE_ROUTE_YAW_LIMIT_DECI)))
    {
        p_route->phase = ROUTE_WAIT_EDGE;
    }
    else if (ROUTE_ON_EDGE == p_route->phase)
    {
        if (progress > (ROUTE_MAX_CROSS_MM + LINE_SENSOR_SPACING_MM) * UM_PER_MM)
        {
            p_route->phase = ROUTE_WAIT_EDGE;
        }
        else if (!b_crossing && !b_left)
        {
            if (route_paired_stripe_width(left_width) && (since_left <= ROUTE_MAX_CROSS_MM + LINE_SENSOR_SPACING_MM))
            {
                route_anchor_left(p_route, travel, heading, since_left, left_width);
            }
            else
            {
                p_route->phase = ROUTE_WAIT_EDGE;
            }
        }
    }
    else if (ROUTE_APPROACH == p_route->phase)
    {
        if (progress >= LINE_SENSOR_LEAD_MM * UM_PER_MM)
        {
            p_route->phase = ROUTE_PIVOT_START;
            p_route->through_seen = false;
            p_route->clear_seen = false;
            p_route->opposite_seen = false;
            p_route->extended = false;
        }
    }
    p_route->was_crossing = b_crossing;
    return ROUTE_PIVOT_START == p_route->phase;
}

bool line_route_has_candidate(line_route_t const * p_route)
{
    return p_route->phase != ROUTE_WAIT_EDGE;
}

line_side_t line_route_approach_side(line_route_t const * p_route)
{
    if (ROUTE_APPROACH != p_route->phase)
    {
        return LINE_SIDE_NONE;
    }
    return p_route->right_anchor ? LINE_SIDE_RIGHT : LINE_SIDE_LEFT;
}

bool line_route_returning(line_route_t const * p_route)
{
    return p_route->retained && ((p_route->phase == ROUTE_ALIGN_THROUGH) ||
                                 (p_route->phase == ROUTE_ALIGN_OPPOSITE) ||
                                 (p_route->phase == ROUTE_ALIGN_BRANCH));
}

static bool route_side_sector(int32_t yaw)
{
    return (yaw >= ROUTE_BRANCH_MIN_DECI) && (yaw < ROUTE_BRANCH_MAX_DECI);
}

/* The leading detector clears the chosen tape edge as the tape enters the gap
 * between the detectors. A return from another sector needs a fresh hit. */
static void route_begin_alignment(line_route_t * p_route, route_phase_t phase, bool b_increasing,
                                  bool b_on_tape, bool b_retain)
{
    p_route->phase = (uint8_t)phase;
    p_route->align_increasing = b_increasing;
    p_route->clear_seen = b_on_tape;
    p_route->retained = b_retain;
}

static line_turn_status_t route_align(line_route_t * p_route, int32_t yaw, bool b_tape,
                                      int32_t * p_speed)
{
    int32_t const center = (ROUTE_ALIGN_THROUGH == p_route->phase) ? 0 :
                           (ROUTE_ALIGN_BRANCH == p_route->phase) ? LINE_TURN_QUARTER_DECI :
                           -LINE_TURN_QUARTER_DECI;
    int32_t const window = (ROUTE_ALIGN_THROUGH == p_route->phase) ? ROUTE_STRAIGHT_DECI :
                           ROUTE_BRANCH_MAX_DECI - LINE_TURN_QUARTER_DECI;
    int32_t const offset = p_route->align_increasing ? yaw - center : center - yaw;
    if (offset > window)
    {
        return LINE_TURN_ABORTED;
    }
    if (offset >= -window)
    {
        if (!b_tape && p_route->clear_seen)
        {
            return p_route->retained ? LINE_TURN_FALLBACK_DONE : LINE_TURN_DONE;
        }
        p_route->clear_seen = b_tape;
    }
    int32_t const speed = (p_route->extended && (offset < -window)) ?
                          LINE_TURN_FAST_UM_S : ROUTE_PROBE_SPEED_UM_S;
    *p_speed = p_route->align_increasing ? speed : -speed;
    return LINE_TURN_TURNING;
}

/* Check each side from a sampled tape-free sector. On an approach without a
 * through route, a requested side is a choice only if the opposite side also
 * carries tape. A single exit remains a bend and retains the instruction. */
static bool route_probe_out(line_route_t * p_route, int32_t yaw, bool b_tape)
{
    bool const b_straight = p_route->kind == (uint8_t)LINE_TURN_KIND_STRAIGHT;
    if (!b_tape && (yaw > ROUTE_STRAIGHT_DECI) && (yaw < ROUTE_BRANCH_MIN_DECI))
    {
        p_route->clear_seen = true;
    }
    if ((ROUTE_PROBE_OUT == p_route->phase) && !p_route->through_seen && (yaw > ROUTE_STRAIGHT_DECI))
    {
        p_route->phase = ROUTE_REVISIT_THROUGH;
        p_route->clear_seen = false;
        p_route->extended = true;
    }
    else if (p_route->clear_seen && b_tape && route_side_sector(yaw))
    {
        bool const b_select = b_straight ? p_route->through_seen :
                              (p_route->through_seen || p_route->opposite_seen);
        bool const b_use_through = b_straight && p_route->through_seen;
        route_begin_alignment(p_route, b_use_through ? ROUTE_ALIGN_THROUGH : ROUTE_ALIGN_BRANCH,
                              !b_use_through, !b_use_through, !b_select);
        return b_select;
    }
    else if (yaw >= ROUTE_BRANCH_MAX_DECI)
    {
        if (!p_route->clear_seen)
        {
            p_route->phase = ROUTE_RESTORE;
        }
        else if (b_straight && p_route->through_seen)
        {
            p_route->phase = ROUTE_PROBE_OPPOSITE;
            p_route->clear_seen = false;
            p_route->extended = true;
        }
        else if (p_route->through_seen || p_route->opposite_seen)
        {
            route_begin_alignment(p_route, p_route->through_seen ? ROUTE_ALIGN_THROUGH : ROUTE_ALIGN_OPPOSITE,
                                  false, false, true);
        }
        else
        {
            p_route->phase = ROUTE_RESTORE;
        }
    }
    return false;
}

static bool route_probe_opposite(line_route_t * p_route, int32_t yaw, bool b_tape)
{
    if (!b_tape && (-yaw > ROUTE_STRAIGHT_DECI) && (-yaw < ROUTE_BRANCH_MIN_DECI))
    {
        p_route->clear_seen = true;
    }
    if (b_tape && p_route->clear_seen && route_side_sector(-yaw))
    {
        p_route->opposite_seen = true;
        if ((p_route->kind == (uint8_t)LINE_TURN_KIND_UTURN) && p_route->through_seen)
        {
            return true;
        }
        if (p_route->kind == (uint8_t)LINE_TURN_KIND_STRAIGHT)
        {
            bool const b_through = p_route->through_seen;
            route_begin_alignment(p_route, b_through ? ROUTE_ALIGN_THROUGH : ROUTE_ALIGN_OPPOSITE,
                                  b_through, !b_through, !b_through);
            return b_through;
        }
    }
    if (p_route->opposite_seen || (-yaw >= ROUTE_BRANCH_MAX_DECI))
    {
        if (p_route->through_seen)
        {
            route_begin_alignment(p_route, ROUTE_ALIGN_THROUGH, true, false, true);
        }
        else
        {
            p_route->phase = ROUTE_RECHECK_BRANCH;
            p_route->clear_seen = false;
        }
    }
    return false;
}

/* Measure headings toward the requested side (or the observed side for S).
 * The extra sweep uses the normal fast pivot while locating sectors and
 * drops to capture speed to center on the chosen tape. */
line_turn_status_t line_route_probe(line_route_t * p_route, int64_t left_um, int64_t right_um,
                                    bool b_tape, line_targets_t * p_targets)
{
    int32_t const delta = line_heading_deci(left_um, right_um) - p_route->heading_deci;
    int32_t const yaw = p_route->right_branch ? -delta : delta;
    bool b_selected = false;
    int32_t speed = 0;
    set_targets(p_targets, 0, 0);
    /* Begin toward the chosen side as soon as the axle is at the stripe. */
    if (ROUTE_PIVOT_START == p_route->phase)
    {
        p_route->phase = ROUTE_PROBE_OUT;
    }
    /* An axle already angled toward the branch can start past the through
     * tape. Sample the full forward sector on the return before deciding
     * whether a second side exit is needed to establish a junction. */
    if (((ROUTE_PROBE_OUT == p_route->phase) || (ROUTE_REVISIT_THROUGH == p_route->phase)) &&
        (abs_i64(yaw) <= ROUTE_STRAIGHT_DECI))
    {
        p_route->through_seen |= b_tape;
    }
    if ((ROUTE_REVISIT_THROUGH == p_route->phase) && (yaw < -ROUTE_STRAIGHT_DECI))
    {
        p_route->phase = p_route->through_seen ? ROUTE_RECHECK_BRANCH : ROUTE_PROBE_OPPOSITE;
    }
    if ((ROUTE_PROBE_OUT == p_route->phase) || (ROUTE_RECHECK_BRANCH == p_route->phase))
    {
        b_selected = route_probe_out(p_route, yaw, b_tape);
    }
    else if (ROUTE_PROBE_OPPOSITE == p_route->phase)
    {
        b_selected = route_probe_opposite(p_route, yaw, b_tape);
    }

    bool const b_align = (ROUTE_ALIGN_BRANCH == p_route->phase) || (ROUTE_ALIGN_OPPOSITE == p_route->phase) ||
                         (ROUTE_ALIGN_THROUGH == p_route->phase);
    if (b_align)
    {
        line_turn_status_t const status = route_align(p_route, yaw, b_tape, &speed);
        if (LINE_TURN_TURNING != status)
        {
            return status;
        }
    }
    else
    {
        if ((ROUTE_RESTORE == p_route->phase) && (yaw <= 0))
        {
            return LINE_TURN_NO_BRANCH;
        }
        bool const b_increasing = (ROUTE_PROBE_OUT == p_route->phase) || (ROUTE_RECHECK_BRANCH == p_route->phase);
        speed = p_route->extended ? LINE_TURN_FAST_UM_S : ROUTE_PROBE_SPEED_UM_S;
        speed = b_increasing ? speed : -speed;
    }
    p_targets->right_um_s = p_route->right_branch ? -speed : speed;
    p_targets->left_um_s = -p_targets->right_um_s;
    return b_selected ? LINE_TURN_BRANCH_SELECTED : LINE_TURN_TURNING;
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
    int64_t heading = wheel_yaw_deci(diff_um);

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
        line_follow_steer_to(0, heading, LINE_CRUISE_UM_S, p_targets);
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
            bool const complete = ((uint8_t)LINE_TURN_KIND_UTURN == p_turn->kind) ?
                (progress >= p_turn->target_deci) : (b_left || b_right);
            if ((TURN_PHASE_SLOW == p_turn->phase) && complete)
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
