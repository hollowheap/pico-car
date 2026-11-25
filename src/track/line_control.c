/** @file line_control.c
 * @brief Pure line-following and one-shot turn logic.
 *
 * Integer math only. The follower is a heading cascade: a sensor hit slews a
 * heading target toward the line and encoder heading closes the loop. A
 * sustained correction can stop the inner wheel. With no sensor hit the car
 * bridges the gap, probes for centered tape, then turns toward the last hit
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
#define TRAIL_STEP_UM        ((int64_t)LINE_TRAIL_STEP_MM * UM_PER_MM)
/* Trial from the closed-loop straight replay. Leave most of the 150 mm
 * bridge for relaxed steering, so centered tape can come into view again. */
#define BLIND_HOLD_UM        (60000)
#define BLIND_SETTLED        (4U)
#define FOLLOW_STRIPE_RIGHT  (8U)
#define FOLLOW_STRIPE_READY  (16U)
#define FOLLOW_MARKER_RECENT (32U)
#define FRONT_DETECTOR_MASK  (3U)
#define FOLLOW_SUSTAINED_EDGE (16U) /* retained in stripe.detector_bits during a gap */
#define CORROBORATED_SHIFT   (2U)
#define FOLLOW_ROUTE_CANDIDATE (64U)
#define FOLLOW_LOCAL_SEARCH    (128U)
/* Heading change needed to move one sensor across the full centered blind
 * strip. The existing loss distance still limits forward travel. */
#define PROBE_ANGLE_DECI     (((LINE_SENSOR_SPACING_MM - LINE_WIDTH_MM) * DECI_PER_RAD) / LINE_SENSOR_LEAD_MM)

/* line_follow_t.sweep: the search turn in progress. */
#define SWEEP_NONE           (0U)
#define SWEEP_TOWARD         (1U)
#define SWEEP_BACK           (2U)
#define SWEEP_TURN_BACK      (3U)
#define SWEEP_REVERSE        (4U)
#define SWEEP_LOCAL_MISSED   (5U)
#define SWEEP_DIRECTIONS     (2)
/* The simulated wheels need 45 mm/s to start, and conversion through the
 * commanded yaw rate rounds down. Recheck the 50 mm/s floor on the car. */
#define LOCAL_SPIN_MIN_UM_S  (50000)

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
    int64_t const heading = wheel_yaw_deci(right_um - left_um);

    int64_t const limited = (heading > HEADING_ABS_MAX) ? HEADING_ABS_MAX
                            : ((heading < -HEADING_ABS_MAX) ? -HEADING_ABS_MAX : heading);

    return (int32_t)limited;
}

/* The transverse stripe anchors the axle for a sweep of two outgoing sectors. */
#define ROUTE_MIDPOINT_DIVISOR   (2)
#define ROUTE_MAX_CROSS_MM       (2 * LINE_WIDTH_MM)
/* A 60-degree transverse crossing projects 15 mm of tape to about 17 mm.
 * Allow another couple of millimetres for the sampled entry and exit. */
#define ROUTE_STRIPE_MAX_MM      (LINE_WIDTH_MM + LINE_WIDTH_MM / 3)
#define ROUTE_FULL_STRIPE_MIN_MM (LINE_WIDTH_MM - 1)
/* A lone detector can graze the through tape while unwinding a bend.
 * Require a near-full-width traversal. The 5 mm allowance covers steering,
 * debounce and quantized encoder travel in the simulated placements. */
#define SINGLE_STRIPE_MIN_MM     (LINE_WIDTH_MM - LINE_WIDTH_MM / 3)
#define ROUTE_STRAIGHT_DECI      (150)
#define ROUTE_BRANCH_MIN_DECI    (600)
#define ROUTE_BRANCH_MAX_DECI    (1200)
#define ROUTE_PROBE_SPEED_UM_S   LINE_SEARCH_UM_S

typedef enum
{
    ROUTE_WAIT_EDGE = 0,
    ROUTE_ON_EDGE,
    ROUTE_APPROACH,
    ROUTE_PROBE_START,
    ROUTE_PROBE_OUT,
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
    bool const b_supported = (width >= ROUTE_FULL_STRIPE_MIN_MM) || p_route->left_recent || p_route->side_recent;
    return (b_ended && b_supported && width >= SINGLE_STRIPE_MIN_MM && width <= ROUTE_STRIPE_MAX_MM) ? width : 0U;
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

    /* A second left hit can still be near the previous barcode-reader pulse.
     * Refining a full-width paired anchor needs tape under that reader again. */
    bool const b_refine = (ROUTE_APPROACH == p_route->phase) && (!p_route->left_stripe_anchored || b_side_tape);
    if (((ROUTE_WAIT_EDGE == p_route->phase) || b_refine) && b_crossing && !p_route->was_crossing && (0U == right_width))
    {
        p_route->start_um = travel;
        p_route->heading_deci = heading;
        p_route->phase = ROUTE_ON_EDGE;
        p_route->left_stripe_anchored = false;
        p_route->probe_eligible = false;
        p_route->right_branch = LINE_TURN_KIND_RIGHT == kind;
        p_route->kind = (uint8_t)kind;
    }
    else if (((ROUTE_WAIT_EDGE == p_route->phase) || (ROUTE_ON_EDGE == p_route->phase)) && (0U != right_width))
    {
        p_route->start_um = travel - (int64_t)right_width * UM_PER_MM / ROUTE_MIDPOINT_DIVISOR;
        p_route->heading_deci = heading;
        p_route->phase = ROUTE_APPROACH;
        p_route->left_stripe_anchored = false;
        p_route->probe_eligible = true;
        /* The left detector can stay on the approach tape into a T. The
         * right detector's clean crossing still fixes the axle position. */
        p_route->right_branch = (LINE_TURN_KIND_RIGHT == kind) ||
                                ((LINE_TURN_KIND_STRAIGHT == kind) && !b_crossing);
        p_route->kind = (uint8_t)kind;
    }
    /* After measuring the stripe, retire its projected axle position if the
     * subsequent steering moves that position by more than a tape width. */
    else if ((progress < 0) || ((ROUTE_APPROACH == p_route->phase) &&
             (abs_i64(yaw) * LINE_SENSOR_LEAD_MM > LINE_WIDTH_MM * DECI_PER_RAD)))
    {
        p_route->phase = ROUTE_WAIT_EDGE;
    }
    else if (ROUTE_ON_EDGE == p_route->phase)
    {
        if (progress > (ROUTE_MAX_CROSS_MM + LINE_SENSOR_SPACING_MM) * UM_PER_MM)
        {
            p_route->phase = ROUTE_WAIT_EDGE;
        }
        else if (!b_crossing)
        {
            p_route->phase = ROUTE_APPROACH;
            uint16_t const width = (uint16_t)(p_route->left_at_mm - p_route->left_begin_mm);
            uint16_t const since = (uint16_t)(mm - p_route->left_at_mm);
            if (!b_left && route_paired_stripe_width(width) &&
                (since <= ROUTE_MAX_CROSS_MM + LINE_SENSOR_SPACING_MM))
            {
                /* Anchor a transverse stripe at the known line-detector lead.
                 * GP26 can see it earlier or later. Beyond it a centered car
                 * can see clear with both line detectors until the sweep. */
                p_route->start_um = travel - ((int64_t)since + width / ROUTE_MIDPOINT_DIVISOR) * UM_PER_MM;
                p_route->heading_deci = heading;
                p_route->left_stripe_anchored = width >= ROUTE_FULL_STRIPE_MIN_MM;
                p_route->probe_eligible = true;
            }
            else
            {
                p_route->start_um += (progress - LINE_SENSOR_SPACING_MM * UM_PER_MM) / ROUTE_MIDPOINT_DIVISOR;
            }
        }
    }
    else if (ROUTE_APPROACH == p_route->phase)
    {
        /* A broad crossing on a bend can leave the approach heading bare.
         * A later tape sighting can qualify this otherwise ambiguous crossing
         * before the axle reaches the candidate. */
        if ((b_left || b_right) && (progress > LINE_WIDTH_MM * UM_PER_MM))
        {
            p_route->probe_eligible = true;
        }
        if (progress >= LINE_SENSOR_LEAD_MM * UM_PER_MM)
        {
            p_route->phase = p_route->probe_eligible ? ROUTE_PROBE_START : ROUTE_WAIT_EDGE;
            p_route->through_seen = false;
            p_route->clear_seen = false;
            p_route->opposite_seen = false;
            p_route->extended = false;
        }
    }
    p_route->was_crossing = b_crossing;
    return ROUTE_PROBE_START == p_route->phase;
}

bool line_route_has_candidate(line_route_t const * p_route)
{
    return p_route->phase != ROUTE_WAIT_EDGE;
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
    if (ROUTE_PROBE_OUT == p_route->phase && abs_i64(yaw) <= ROUTE_STRAIGHT_DECI)
    {
        p_route->through_seen |= b_tape;
    }
    if (!b_tape && (yaw > ROUTE_STRAIGHT_DECI) && (yaw < ROUTE_BRANCH_MIN_DECI))
    {
        p_route->clear_seen = true;
    }
    if ((ROUTE_PROBE_OUT == p_route->phase) && !p_route->through_seen && (yaw > ROUTE_STRAIGHT_DECI))
    {
        p_route->phase = ROUTE_PROBE_OPPOSITE;
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
    if ((ROUTE_PROBE_START == p_route->phase) && (yaw <= -ROUTE_STRAIGHT_DECI))
    {
        p_route->phase = ROUTE_PROBE_OUT;
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
static void trail_record(line_follow_t * p_follow, int32_t heading_deci, int64_t travel_um)
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
static int32_t follow_blind(line_follow_t * p_follow, int32_t heading_deci,
                            int64_t travel_um, uint32_t dt_ms, bool b_continuous_floor)
{
    int64_t const blind_um = abs_i64(travel_um - p_follow->anchor_um);
    int64_t const gap_um   = (0U != (p_follow->flags & LINE_FOLLOW_HUMP)) ? HUMP_GAP_UM : GAP_UM;
    int32_t probe_deci = 0;
    bool b_at_corner = false;
    if ((p_follow->flags & (FOLLOW_STRIPE_READY | FOLLOW_ROUTE_CANDIDATE)) == FOLLOW_STRIPE_READY)
    {
        uint16_t const since = (uint16_t)((uint16_t)(travel_um / UM_PER_MM) - p_follow->progress.stripe.anchor_mm);
        b_at_corner = (since >= LINE_SENSOR_LEAD_MM + LINE_WIDTH_MM / ROUTE_MIDPOINT_DIVISOR) &&
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
         * the early gap so steady wheel mismatch cannot pull the target away. */
        if ((blind_um >= LINE_PROBE_FROM_MM * UM_PER_MM) &&
            ((p_follow->flags & (BLIND_SETTLED | LINE_FOLLOW_HUMP)) == BLIND_SETTLED))
        {
            probe_deci = ((uint8_t)LINE_SIDE_RIGHT == p_follow->side) ? -PROBE_ANGLE_DECI : PROBE_ANGLE_DECI;
        }
        else if ((0U == (p_follow->flags & BLIND_SETTLED)) || (blind_um > BLIND_HOLD_UM))
        {
            relax_ref(p_follow, dt_ms, heading_deci);
            if (p_follow->ref_deci == heading_deci)
            {
                p_follow->flags |= BLIND_SETTLED;
            }
        }
        slew_ref(p_follow, 0, dt_ms, heading_deci, LINE_CAP_DECI);
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
static void search_found(line_follow_t * p_follow, int32_t heading_deci)
{
    int32_t const origin = search_origin(p_follow);
    int32_t speed = 0;

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
    p_follow->progress = (line_follow_progress_t){0};
    p_follow->ref_deci = heading_deci;
}

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

/* A heading improvement sufficient to move a detector across the centered
 * blind strip renews the stall window. Jitter and motion away from the
 * active endpoint leave the original progress clock running. */
static bool local_search_stopped(line_follow_t * p_follow, int32_t heading_deci, uint32_t now_ms)
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
static int32_t search_spin(line_follow_t const * p_follow, int32_t heading_deci)
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

static void follow_detector(line_follow_t * p_follow, bool b_left, int32_t heading_deci,
                            uint32_t dt_ms, bool b_started)
{
    /* A short gap between bends can leave the outer wheel below cruise. */
    int32_t const last = (int32_t)p_follow->speed_q * LINE_SPEED_STEP_UM_S;
    int32_t const cap = corner_speed(p_follow->ref_deci, heading_deci);
    int32_t const rate = (int32_t)(((int64_t)LINE_HIT_DECI_PER_M *
                                    ((last < cap) ? last : cap)) /
                                   UM_PER_M);

    /* Start a new correction from the measured heading when the old
     * target points away from the tape. */
    if ((b_left && p_follow->ref_deci < heading_deci) || (!b_left && p_follow->ref_deci > heading_deci))
    {
        p_follow->ref_deci = heading_deci;
    }
    slew_ref(p_follow, b_left ? rate : -rate, dt_ms, heading_deci, LINE_CAP_DECI);
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
    if (local_search_stopped(p_follow, heading_deci, now_ms))
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
            search_found(p_follow, heading_deci);
        }
        p_follow->flags &= (uint8_t)~FOLLOW_LOCAL_SEARCH;
    }
    if (b_hit && (0U != p_follow->retry) &&
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
    else if (b_hit)
    {
        p_follow->side      = (uint8_t)(b_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
        p_follow->anchor_um = travel_um;
        p_follow->status    = (uint8_t)LINE_FOLLOW_FOLLOWING;
        follow_detector(p_follow, b_left, heading_deci, dt_ms, b_started);
    }
    else
    {
        probe_deci = follow_blind(p_follow, heading_deci, travel_um, dt_ms, b_continuous_floor);
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
    if (((uint8_t)LINE_FOLLOW_FOLLOWING == p_follow->status) ||
        ((uint8_t)LINE_FOLLOW_BRIDGING == p_follow->status))
    {
        trail_record(p_follow, heading_deci, travel_um);
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
        steer_to(heading_deci, p_follow->ref_deci, LINE_SEARCH_UM_S, p_targets);
        set_targets(p_targets, -p_targets->left_um_s, -p_targets->right_um_s);
    }
    else if ((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status)
    {
        /* On the spot: counter-clockwise when the target is to the left. */
        int32_t const spin = search_spin(p_follow, heading_deci);

        set_targets(p_targets, -spin, spin);
    }
    else
    {
        steer_to(p_follow->ref_deci + probe_deci, heading_deci, base_um_s, p_targets);
    }

    return (line_follow_status_t)p_follow->status;
}

line_follow_status_t line_follow_update(line_follow_t * p_follow, bool b_left, bool b_right,
                                        int32_t heading_deci, int64_t travel_um, uint32_t now_ms,
                                        line_targets_t * p_targets)
{
    return line_follow_update_floor(p_follow, b_left, b_right, heading_deci, travel_um, now_ms, false, p_targets);
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
