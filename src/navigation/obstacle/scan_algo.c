/** @file scan_algo.c
 *
 * @brief Echo classification, ping agreement and the ±30° servo scan with
 *        body-pivot windows. Pure C99 and integer only.
 *
 * Scan order (D96, S40):
 *   1. Coarse centre window: 10 degree steps over +-30 degrees (slots 12,
 *      14, ... 24, from right to left).
 *   2. Region: the closest VALID reading within 600 mm inside the path
 *      (+-30 degrees), grown over coarse neighbours within closest + 250 mm.
 *   3. Fine: walk the left edge outward in 5 degree steps, then the right
 *      edge. If an edge reaches the end of the centre servo window (+-30 deg)
 *      without a miss, pivot the body toward that side by
 *      OBSTACLE_SCAN_PIVOT_DECI (+-60 deg) once and continue walking in the
 *      shifted servo window (+-30 deg around the pivoted body, covering up
 *      to +-90 deg). All readings merge into the 37-slot profile in the
 *      original frame.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "scan_algo.h"

#define PHASE_COARSE     (0U)
#define PHASE_FINE_LEFT  (1U)
#define PHASE_FINE_RIGHT (2U)
#define PHASE_DONE       (3U)

/* Coarse stride in slots: 100 deci / 50 deci = 2 slots. */
#define COARSE_STRIDE ((uint8_t)(OBSTACLE_COARSE_STEP_DECI / OBSTACLE_SLOT_STEP_DECI))

/* Centre window slot bounds (-30 deg = slot 12, +30 deg = slot 24). */
#define CENTRE_FIRST_SLOT                                                        \
    ((uint8_t)((-OBSTACLE_SERVO_LIMIT_DECI - OBSTACLE_SLOT_FIRST_DECI) /         \
               OBSTACLE_SLOT_STEP_DECI))
#define CENTRE_LAST_SLOT                                                         \
    ((uint8_t)((OBSTACLE_SERVO_LIMIT_DECI - OBSTACLE_SLOT_FIRST_DECI) /          \
               OBSTACLE_SLOT_STEP_DECI))

/* Highest slot number (36). */
#define LAST_SLOT (OBSTACLE_SLOT_COUNT - 1U)

/* Outcome of walking one edge. */
#define WALK_MEASURE   (0U) /* p_ctx->slot must be measured next */
#define WALK_BOUNDED   (1U) /* a real miss was found             */
#define WALK_UNBOUNDED (2U) /* the walk ran past the window      */

static bool    find_closest (scan_algo_ctx_t * p_ctx, int32_t half_deci,
                             uint16_t * p_best_mm);
static void    finish_coarse (scan_algo_ctx_t * p_ctx);
static bool    slot_in_servo_window (int16_t body_deci, int32_t slot);
static uint8_t walk_edge (scan_algo_ctx_t * p_ctx, uint8_t side, int32_t first,
                          uint8_t * p_miss);
static void    run_fine (scan_algo_ctx_t * p_ctx, uint8_t side, int32_t first);

/*!
 * @brief Convert an echo width to a distance (integer maths, 343 m/s).
 *
 * @param[in] width_us Echo pulse width in microseconds.
 *
 * @return Distance in mm, saturated to the 12-bit field.
 */
uint16_t scan_algo_dist_mm (uint32_t width_us)
{
    uint32_t const width =
        (width_us > OBSTACLE_VALID_MAX_US) ? OBSTACLE_VALID_MAX_US : width_us;
    uint32_t dist = (width * OBSTACLE_SOUND_NUM) / OBSTACLE_SOUND_DEN;

    if (dist > OBSTACLE_DIST_MAX)
    {
        dist = OBSTACLE_DIST_MAX;
    }

    /* Cast is safe: dist is at most 4095 here. */
    return ((uint16_t)dist);
}

/*!
 * @brief Classify one ping. A missing echo is never OPEN.
 *
 * @param[in] mbx         Mailbox word written by the echo ISR.
 * @param[in] b_timed_out True if the 40 ms wait ended without DONE.
 * @param[in] b_echo_high ECHO level read after the wait.
 * @param[in] b_stuck     True if ECHO was already high before the trigger.
 *
 * @return One of OBSTACLE_CLASS_*.
 */
uint8_t scan_algo_classify (uint32_t mbx, bool b_timed_out, bool b_echo_high,
                            bool b_stuck)
{
    uint8_t        cls    = OBSTACLE_CLASS_UNKNOWN;
    uint32_t const width  = mbx & OBSTACLE_MBX_WIDTH_MASK;
    bool const     b_rise = (0U != (mbx & OBSTACLE_MBX_RISE));
    bool const     b_done = (0U != (mbx & OBSTACLE_MBX_DONE));

    if (b_stuck)
    {
        cls = OBSTACLE_CLASS_STUCK;
    }
    else if (b_done && b_rise)
    {
        if (width < OBSTACLE_VALID_MIN_US)
        {
            /* Too short: a glitch or an object closer than 2 cm. */
            cls = OBSTACLE_CLASS_UNKNOWN;
        }
        else if (width <= OBSTACLE_VALID_MAX_US)
        {
            cls = OBSTACLE_CLASS_VALID;
        }
        else
        {
            cls = OBSTACLE_CLASS_OPEN;
        }
    }
    else if (b_timed_out && b_rise && b_echo_high)
    {
        /* Still high after 40 ms: nothing within 4 m. */
        cls = OBSTACLE_CLASS_OPEN;
    }
    else
    {
        /* No rise, a fall without a rise, or a rise whose fall was missed
         * (ECHO is low now). None of these can be trusted. */
        cls = OBSTACLE_CLASS_UNKNOWN;
    }

    return (cls);
}

/*!
 * @brief Bearing angle of a slot in the scan frame.
 *
 * @param[in] slot Slot number, 0..36.
 *
 * @return Angle in deci-degrees, positive = left. Slot 18 is 0.
 */
int16_t scan_algo_slot_deci (uint8_t slot)
{
    int32_t const deci =
        OBSTACLE_SLOT_FIRST_DECI + ((int32_t)slot * OBSTACLE_SLOT_STEP_DECI);

    /* Cast is safe: slots 0..36 give -900..+900. */
    return ((int16_t)deci);
}

/*!
 * @brief Body heading offset required for the current scan target, relative
 *        to the heading when the scan began.
 *
 * @param[in] p_ctx Scan engine state.
 *
 * @return Body pivot angle in deci-degrees (0, +600, or -600).
 */
int16_t scan_algo_body_deci (scan_algo_ctx_t const * p_ctx)
{
    return ((PHASE_DONE == p_ctx->phase) ? (int16_t)0 : p_ctx->body_deci);
}

/*!
 * @brief Servo angle for the current scan target, clamped to +-30 degrees.
 *
 * @param[in] p_ctx Scan engine state.
 *
 * @return Servo angle in deci-degrees (-300..+300).
 */
int16_t scan_algo_servo_deci (scan_algo_ctx_t const * p_ctx)
{
    int32_t deci = 0;

    if (PHASE_DONE != p_ctx->phase)
    {
        deci = (int32_t)scan_algo_slot_deci(p_ctx->slot) - (int32_t)p_ctx->body_deci;
        if (deci > OBSTACLE_SERVO_LIMIT_DECI)
        {
            deci = OBSTACLE_SERVO_LIMIT_DECI;
        }
        else if (deci < -OBSTACLE_SERVO_LIMIT_DECI)
        {
            deci = -OBSTACLE_SERVO_LIMIT_DECI;
        }
        else
        {
            /* Already within +-30 degrees. */
        }
    }

    return ((int16_t)deci);
}

/*!
 * @brief Empty the agreement buffer before a new slot.
 *
 * @param[out] p_agree Agreement buffer.
 */
void scan_algo_agree_reset (scan_algo_agree_t * p_agree)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < OBSTACLE_MAX_PINGS; idx++)
    {
        p_agree->sample[idx] = 0U;
    }

    p_agree->count = 0U;
}

/*!
 * @brief Add one ping to the agreement buffer.
 *
 * Two pings agree when they have the same class and, for VALID, are within
 * 15 mm of each other. The closer distance is kept. After four pings
 * without agreement the slot is UNKNOWN.
 *
 * @param[in,out] p_agree Agreement buffer for the current slot.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance (only used for VALID).
 * @param[out]    p_slot  Packed slot value when the function returns true.
 *
 * @return True when the slot is settled.
 */
bool scan_algo_agree_add (scan_algo_agree_t * p_agree, uint8_t cls,
                          uint16_t dist_mm, uint16_t * p_slot)
{
    bool           b_settled = false;
    uint8_t        idx       = 0U;
    uint16_t const dist      = (OBSTACLE_CLASS_VALID == cls) ? dist_mm : 0U;

    for (idx = 0U; (idx < p_agree->count) && (!b_settled); idx++)
    {
        uint16_t const old_val  = p_agree->sample[idx];
        uint16_t const old_dist = obstacle_pack_slot_dist(old_val);
        uint16_t const near_mm  = (old_dist < dist) ? old_dist : dist;
        uint16_t const far_mm   = (old_dist < dist) ? dist : old_dist;
        uint32_t const diff_mm  = (uint32_t)far_mm - (uint32_t)near_mm;

        if ((obstacle_pack_slot_class(old_val) == cls) &&
            ((OBSTACLE_CLASS_VALID != cls) || (diff_mm <= OBSTACLE_AGREE_MM)))
        {
            *p_slot   = obstacle_pack_slot(near_mm, cls);
            b_settled = true;
        }
    }

    if ((!b_settled) && (p_agree->count < OBSTACLE_MAX_PINGS))
    {
        p_agree->sample[p_agree->count] = obstacle_pack_slot(dist, cls);
        p_agree->count++;
    }

    if ((!b_settled) && (p_agree->count >= OBSTACLE_MAX_PINGS))
    {
        *p_slot   = obstacle_pack_slot(0U, OBSTACLE_CLASS_UNKNOWN);
        b_settled = true;
    }

    return (b_settled);
}

/*!
 * @brief Tell whether a measured slot belongs to the obstacle region.
 *
 * @param[in] slot_val   Packed slot.
 * @param[in] closest_mm Closest distance in the region.
 *
 * @return True for a VALID reading within closest + 250 mm.
 */
bool scan_algo_is_hit (uint16_t slot_val, uint16_t closest_mm)
{
    uint32_t const limit_mm = (uint32_t)closest_mm + OBSTACLE_REGION_EXTRA_MM;

    return ((obstacle_pack_slot_is_measured(slot_val)) &&
            (OBSTACLE_CLASS_VALID == obstacle_pack_slot_class(slot_val)) &&
            ((uint32_t)obstacle_pack_slot_dist(slot_val) <= limit_mm));
}

/*!
 * @brief Tell whether a measured slot proves that nothing is there.
 *
 * @param[in] slot_val   Packed slot.
 * @param[in] ref_mm     Reference range in mm (the last hit on the edge).
 *
 * @return True for OPEN, or for VALID beyond ref + 250 mm.
 */
bool scan_algo_is_miss (uint16_t slot_val, uint16_t ref_mm)
{
    uint8_t const  cls      = obstacle_pack_slot_class(slot_val);
    uint32_t const limit_mm = (uint32_t)ref_mm + OBSTACLE_REGION_EXTRA_MM;
    bool           b_miss   = false;

    if (!obstacle_pack_slot_is_measured(slot_val))
    {
        b_miss = false;
    }
    else if (OBSTACLE_CLASS_OPEN == cls)
    {
        b_miss = true;
    }
    else if (OBSTACLE_CLASS_VALID == cls)
    {
        b_miss = ((uint32_t)obstacle_pack_slot_dist(slot_val) > limit_mm);
    }
    else
    {
        b_miss = false;
    }

    return (b_miss);
}

/*!
 * @brief Start a new scan in the centre servo window (-30..+30 deg).
 *
 * @param[out] p_ctx Scan engine state.
 */
void scan_algo_start (scan_algo_ctx_t * p_ctx)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < OBSTACLE_SLOT_COUNT; idx++)
    {
        p_ctx->profile[idx] = 0U;
    }

    scan_algo_agree_reset(&p_ctx->agree);
    p_ctx->phase        = PHASE_COARSE;
    p_ctx->slot         = CENTRE_FIRST_SLOT;
    p_ctx->body_deci    = 0;
    p_ctx->closest_slot = OBSTACLE_SLOT_CENTRE;
    p_ctx->closest_mm   = 0U;
    p_ctx->region_lo    = CENTRE_FIRST_SLOT;
    p_ctx->region_hi    = CENTRE_FIRST_SLOT;

    for (idx = 0U; idx < SCAN_ALGO_SIDES; idx++)
    {
        p_ctx->edge_state[idx] = SCAN_ALGO_EDGE_NONE;
        p_ctx->edge_hit[idx]   = 0U;
        p_ctx->edge_miss[idx]  = 0U;
        p_ctx->b_pivoted[idx]  = false;
    }
}

/*!
 * @brief Slot that must be measured next.
 *
 * @param[in] p_ctx Scan engine state.
 *
 * @return Slot number, or SCAN_ALGO_NO_SLOT when the scan is finished.
 */
uint8_t scan_algo_target (scan_algo_ctx_t const * p_ctx)
{
    return ((PHASE_DONE == p_ctx->phase) ? SCAN_ALGO_NO_SLOT : p_ctx->slot);
}

/*!
 * @brief Tell whether the finished scan found a region.
 *
 * @param[in] p_ctx Scan engine state.
 *
 * @return True if a region exists.
 */
bool scan_algo_has_region (scan_algo_ctx_t const * p_ctx)
{
    return ((SCAN_ALGO_EDGE_NONE != p_ctx->edge_state[SCAN_ALGO_LEFT]) ||
            (SCAN_ALGO_EDGE_NONE != p_ctx->edge_state[SCAN_ALGO_RIGHT]));
}

/*!
 * @brief Feed one ping taken at scan_algo_target().
 *
 * @param[in,out] p_ctx   Scan engine state.
 * @param[in]     cls     Ping class.
 * @param[in]     dist_mm Ping distance.
 *
 * @return True when the whole scan is finished.
 */
bool scan_algo_feed (scan_algo_ctx_t * p_ctx, uint8_t cls, uint16_t dist_mm)
{
    uint16_t slot_val = 0U;

    if ((PHASE_DONE != p_ctx->phase) &&
        (scan_algo_agree_add(&p_ctx->agree, cls, dist_mm, &slot_val)))
    {
        p_ctx->profile[p_ctx->slot] = slot_val;
        scan_algo_agree_reset(&p_ctx->agree);

        switch (p_ctx->phase)
        {
            case PHASE_COARSE:
                if ((p_ctx->slot + COARSE_STRIDE) <= CENTRE_LAST_SLOT)
                {
                    p_ctx->slot = (uint8_t)(p_ctx->slot + COARSE_STRIDE);
                }
                else
                {
                    finish_coarse(p_ctx);
                }
            break;

            case PHASE_FINE_LEFT:
                run_fine(p_ctx, SCAN_ALGO_LEFT, (int32_t)p_ctx->slot);
            break;

            case PHASE_FINE_RIGHT:
                run_fine(p_ctx, SCAN_ALGO_RIGHT, (int32_t)p_ctx->slot);
            break;

            default:
                p_ctx->phase     = PHASE_DONE;
                p_ctx->body_deci = 0;
            break;
        }
    }

    return (PHASE_DONE == p_ctx->phase);
}

/*!
 * @brief Find the closest VALID coarse reading within 600 mm, looking only
 *        at slots no further than half_deci from straight ahead.
 *
 * @param[in,out] p_ctx     Scan engine state; closest_slot is set on success.
 * @param[in]     half_deci Largest angle to look at, in deci-degrees.
 * @param[out]    p_best_mm Closest distance on success.
 *
 * @return True if a reading was found.
 */
static bool find_closest (scan_algo_ctx_t * p_ctx, int32_t half_deci,
                          uint16_t * p_best_mm)
{
    uint32_t slot    = 0U;
    bool     b_found = false;

    for (slot = CENTRE_FIRST_SLOT; slot <= CENTRE_LAST_SLOT; slot += COARSE_STRIDE)
    {
        int32_t const  deci     = scan_algo_slot_deci((uint8_t)slot);
        uint16_t const slot_val = p_ctx->profile[slot];
        uint16_t const dist     = obstacle_pack_slot_dist(slot_val);

        if ((deci >= -half_deci) && (deci <= half_deci) &&
            (obstacle_pack_slot_is_measured(slot_val)) &&
            (OBSTACLE_CLASS_VALID == obstacle_pack_slot_class(slot_val)) &&
            (dist <= OBSTACLE_REGION_MAX_MM) && ((!b_found) || (dist < *p_best_mm)))
        {
            *p_best_mm          = dist;
            p_ctx->closest_slot = (uint8_t)slot;
            b_found             = true;
        }
    }

    return (b_found);
}

/*!
 * @brief After the last coarse slot: find the region and start the fine
 *        scan.
 *
 * @param[in,out] p_ctx Scan engine state.
 */
static void finish_coarse (scan_algo_ctx_t * p_ctx)
{
    uint16_t best_mm = 0U;
    bool     b_found = find_closest(p_ctx, OBSTACLE_PATH_HALF_DECI, &best_mm);

    if (!b_found)
    {
        p_ctx->phase     = PHASE_DONE;
        p_ctx->body_deci = 0;
    }
    else
    {
        p_ctx->closest_mm = best_mm;
        p_ctx->region_lo  = p_ctx->closest_slot;
        p_ctx->region_hi  = p_ctx->closest_slot;

        while (
            ((p_ctx->region_hi + COARSE_STRIDE) <= CENTRE_LAST_SLOT) &&
            (scan_algo_is_hit(
                p_ctx->profile[p_ctx->region_hi + COARSE_STRIDE], best_mm)))
        {
            p_ctx->region_hi = (uint8_t)(p_ctx->region_hi + COARSE_STRIDE);
        }

        while (
            (p_ctx->region_lo >= (CENTRE_FIRST_SLOT + COARSE_STRIDE)) &&
            (scan_algo_is_hit(
                p_ctx->profile[p_ctx->region_lo - COARSE_STRIDE], best_mm)))
        {
            p_ctx->region_lo = (uint8_t)(p_ctx->region_lo - COARSE_STRIDE);
        }

        p_ctx->edge_hit[SCAN_ALGO_LEFT] = p_ctx->region_hi;
        /* Check the 5-degree slot inside the last coarse interval first if
         * region_hi stopped before CENTRE_LAST_SLOT. */
        run_fine(p_ctx, SCAN_ALGO_LEFT, (int32_t)p_ctx->region_hi + 1);
    }
}

/*!
 * @brief Tell whether a slot's bearing is reachable within +-30 degrees of
 *        the current body heading.
 *
 * @param[in] body_deci Body heading offset in deci-degrees.
 * @param[in] slot      Slot index.
 *
 * @return True if |slot_deci - body_deci| <= OBSTACLE_SERVO_LIMIT_DECI.
 */
static bool slot_in_servo_window (int16_t body_deci, int32_t slot)
{
    bool b_in = false;

    if ((slot >= 0) && (slot <= (int32_t)LAST_SLOT))
    {
        int32_t const diff =
            (int32_t)scan_algo_slot_deci((uint8_t)slot) - (int32_t)body_deci;

        b_in = (diff >= -OBSTACLE_SERVO_LIMIT_DECI) &&
               (diff <= OBSTACLE_SERVO_LIMIT_DECI);
    }

    return (b_in);
}

/*!
 * @brief Walk one edge outward, starting at slot first. If the walk reaches
 *        the end of the centre servo window without a real miss, pivot the
 *        body once toward that side so the servo window shifts to 30..90 deg.
 *
 * @param[in,out] p_ctx  Scan engine state.
 * @param[in]     side   SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 * @param[in]     first  First slot to look at.
 * @param[out]    p_miss First slot beyond the edge (BOUNDED/UNBOUNDED).
 *
 * @return WALK_MEASURE, WALK_BOUNDED or WALK_UNBOUNDED.
 */
static uint8_t walk_edge (scan_algo_ctx_t * p_ctx, uint8_t side, int32_t first,
                          uint8_t * p_miss)
{
    int32_t const  step    = (SCAN_ALGO_LEFT == side) ? 1 : -1;
    int32_t        slot    = first;
    uint8_t        result  = WALK_MEASURE;
    bool           b_more  = true;
    uint16_t const hit_val = p_ctx->profile[p_ctx->edge_hit[side]];
    uint16_t       ref_mm  = obstacle_pack_slot_dist(hit_val);

    while (b_more)
    {
        bool const b_inside = (slot >= 0) && (slot <= (int32_t)LAST_SLOT);

        if (!b_inside)
        {
            *p_miss = (SCAN_ALGO_LEFT == side) ? (uint8_t)LAST_SLOT : 0U;
            result  = WALK_UNBOUNDED;
            b_more  = false;
        }
        else if (!slot_in_servo_window(p_ctx->body_deci, slot))
        {
            if (!p_ctx->b_pivoted[side])
            {
                p_ctx->b_pivoted[side] = true;
                p_ctx->body_deci       = (SCAN_ALGO_LEFT == side)
                                             ? (int16_t)OBSTACLE_SCAN_PIVOT_DECI
                                             : (int16_t)(-OBSTACLE_SCAN_PIVOT_DECI);
            }
            else
            {
                *p_miss = (SCAN_ALGO_LEFT == side) ? (uint8_t)LAST_SLOT : 0U;
                result  = WALK_UNBOUNDED;
                b_more  = false;
            }
        }
        else
        {
            uint16_t const slot_val = p_ctx->profile[slot];

            if (!obstacle_pack_slot_is_measured(slot_val))
            {
                p_ctx->slot = (uint8_t)slot;
                result      = WALK_MEASURE;
                b_more      = false;
            }
            else if (scan_algo_is_miss(slot_val, ref_mm))
            {
                *p_miss = (uint8_t)slot;
                result  = WALK_BOUNDED;
                b_more  = false;
            }
            else if (scan_algo_is_hit(slot_val, p_ctx->closest_mm))
            {
                p_ctx->edge_hit[side] = (uint8_t)slot;
                ref_mm                = obstacle_pack_slot_dist(slot_val);
                slot += step;
            }
            else
            {
                /* UNKNOWN or STUCK: assume the obstacle goes on. */
                slot += step;
            }
        }
    }

    return (result);
}

/*!
 * @brief Carry the fine scan on from slot first: walk the left edge, then
 *        the right edge, until a slot needs measuring or both are closed.
 *
 * @param[in,out] p_ctx Scan engine state.
 * @param[in]     side  Edge being walked (SCAN_ALGO_LEFT or _RIGHT).
 * @param[in]     first First slot to look at.
 */
static void run_fine (scan_algo_ctx_t * p_ctx, uint8_t side, int32_t first)
{
    uint8_t cur_side  = side;
    int32_t cur_first = first;
    bool    b_more    = true;

    while (b_more)
    {
        uint8_t       miss   = 0U;
        uint8_t const result = walk_edge(p_ctx, cur_side, cur_first, &miss);

        if (WALK_MEASURE == result)
        {
            p_ctx->phase = (SCAN_ALGO_LEFT == cur_side) ? PHASE_FINE_LEFT
                                                        : PHASE_FINE_RIGHT;
            b_more       = false;
        }
        else
        {
            p_ctx->edge_state[cur_side] = (WALK_BOUNDED == result)
                                              ? SCAN_ALGO_EDGE_BOUNDED
                                              : SCAN_ALGO_EDGE_UNBOUNDED;
            p_ctx->edge_miss[cur_side]  = miss;

            if (SCAN_ALGO_LEFT == cur_side)
            {
                cur_side                         = SCAN_ALGO_RIGHT;
                p_ctx->body_deci                 = 0;
                p_ctx->edge_hit[SCAN_ALGO_RIGHT] = p_ctx->region_lo;
                cur_first                        = (int32_t)p_ctx->region_lo - 1;
            }
            else
            {
                p_ctx->phase     = PHASE_DONE;
                p_ctx->body_deci = 0;
                b_more           = false;
            }
        }
    }
}

/*** end of file ***/
