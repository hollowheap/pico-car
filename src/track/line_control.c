/** @file line_control.c
 * @brief Pure line-following and one-shot turn logic (decisions D92, D93).
 *
 * Integer math only. The follower keeps the inner-wheel-stop correction and
 * adds a memory of the last side that saw the line. A turn measures its
 * heading from the wheel displacements and ends when a sensor sees the line
 * inside the capture window.
 */

#include "track/line_control.h"

#define UM_PER_MM            (1000)
#define DECI_PER_RAD         (573)
#define HEADING_CLAMP_DECI   (100000)

/* Turn phases (line_turn_state_t.phase). IDLE is zero so that a zeroed state is
 * safe. */
#define TURN_PHASE_IDLE      (0U)
#define TURN_PHASE_FAST      (1U)
#define TURN_PHASE_SLOW      (2U)
#define TURN_PHASE_DONE      (3U)
#define TURN_PHASE_ABORTED   (4U)

#define SEARCH_MAX_UM        ((int64_t)LINE_SEARCH_MAX_MM * UM_PER_MM)
#define TRACK_UM             ((int64_t)LINE_WHEEL_TRACK_MM * UM_PER_MM)

static void set_targets(line_targets_t * p_targets, int32_t left_um_s,
                        int32_t right_um_s)
{
    p_targets->left_um_s  = left_um_s;
    p_targets->right_um_s = right_um_s;
}

/* Inner-wheel-stop correction toward a side. */
static void correct_toward(line_side_t side, line_targets_t * p_targets)
{
    if (LINE_SIDE_LEFT == side)
    {
        set_targets(p_targets, 0, LINE_CRUISE_UM_S);
    }
    else
    {
        set_targets(p_targets, LINE_CRUISE_UM_S, 0);
    }
}

static int64_t abs_i64(int64_t value)
{
    return (value < 0) ? -value : value;
}

void line_follow_init(line_follow_t * p_follow)
{
    p_follow->search_start_um = 0;
    p_follow->search_start_ms = 0U;
    p_follow->side            = (uint8_t)LINE_SIDE_NONE;
    p_follow->status          = (uint8_t)LINE_FOLLOW_FOLLOWING;
}

void line_follow_set_side(line_follow_t * p_follow, line_side_t side)
{
    p_follow->side   = (uint8_t)side;
    p_follow->status = (uint8_t)LINE_FOLLOW_FOLLOWING;
}

void line_follow_hold_lost(line_follow_t * p_follow)
{
    p_follow->status = (uint8_t)LINE_FOLLOW_LOST;
}

/* At least one sensor is over the line: correct, remember the side. */
static line_follow_status_t follow_on_line(line_follow_t * p_follow,
                                           bool b_left, bool b_right,
                                           line_targets_t * p_targets)
{
    if (b_left && b_right)
    {
        set_targets(p_targets, LINE_CRUISE_UM_S, LINE_CRUISE_UM_S);
    }
    else
    {
        p_follow->side = (uint8_t)(b_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
        correct_toward((line_side_t)p_follow->side, p_targets);
    }
    p_follow->status = (uint8_t)LINE_FOLLOW_FOLLOWING;

    return LINE_FOLLOW_FOLLOWING;
}

/* Neither sensor sees the line and a side is recorded. */
static line_follow_status_t follow_search(line_follow_t * p_follow,
                                          int64_t travel_um, uint32_t now_ms,
                                          line_targets_t * p_targets)
{
    if ((uint8_t)LINE_FOLLOW_FOLLOWING == p_follow->status)
    {
        p_follow->search_start_um = travel_um;
        p_follow->search_start_ms = now_ms;
        p_follow->status          = (uint8_t)LINE_FOLLOW_SEARCHING;
    }

    if (((uint8_t)LINE_FOLLOW_SEARCHING == p_follow->status) &&
        ((abs_i64(travel_um - p_follow->search_start_um) >= SEARCH_MAX_UM) ||
         ((uint32_t)(now_ms - p_follow->search_start_ms) >=
          LINE_SEARCH_MAX_MS)))
    {
        p_follow->status = (uint8_t)LINE_FOLLOW_LOST;
    }

    if ((uint8_t)LINE_FOLLOW_LOST == p_follow->status)
    {
        set_targets(p_targets, 0, 0);
    }
    else
    {
        correct_toward((line_side_t)p_follow->side, p_targets);
    }

    return (line_follow_status_t)p_follow->status;
}

line_follow_status_t line_follow_update(line_follow_t * p_follow, bool b_left,
                                        bool b_right, int64_t travel_um,
                                        uint32_t now_ms,
                                        line_targets_t * p_targets)
{
    line_follow_status_t status = LINE_FOLLOW_FOLLOWING;

    if (b_left || b_right)
    {
        status = follow_on_line(p_follow, b_left, b_right, p_targets);
    }
    else if (((uint8_t)LINE_SIDE_NONE == p_follow->side) &&
             ((uint8_t)LINE_FOLLOW_LOST != p_follow->status))
    {
        set_targets(p_targets, LINE_CRUISE_UM_S, LINE_CRUISE_UM_S);
        p_follow->status = (uint8_t)LINE_FOLLOW_FOLLOWING;
    }
    else
    {
        status = follow_search(p_follow, travel_um, now_ms, p_targets);
    }

    return status;
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

line_turn_status_t line_turn_update(line_turn_state_t * p_turn, int64_t left_um,
                                    int64_t right_um, bool b_fresh, bool b_left,
                                    bool b_right, line_targets_t * p_targets)
{
    int32_t heading = 0;
    int32_t progress = 0;
    int32_t speed = LINE_TURN_FAST_UM_S;
    line_turn_status_t status = LINE_TURN_TURNING;

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
