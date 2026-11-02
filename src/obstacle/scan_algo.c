/** @file scan_algo.c
 *
 * @brief Buddy 5 echo classification, ping agreement and the coarse to
 *        fine scan engine. Pure C99 and integer only.
 *
 * Scan order (Spec 3a-3d):
 *   1. Coarse: slots 0, 3, ... 24, from right to left.
 *   2. Region: the closest VALID reading within 600 mm, grown over coarse
 *      neighbours that are VALID and within closest + 250 mm. The closest
 *      reading is looked for inside the car's path (+-30 degrees) first,
 *      so clutter off to one side cannot take the place of the obstacle
 *      that stopped the car.
 *   3. Fine, left edge first because the servo is already on the left:
 *      the slots are walked outward until the first real miss (OPEN, or
 *      VALID beyond closest + 250 mm). An UNKNOWN or STUCK slot proves
 *      nothing, so the walk assumes the obstacle goes on and steps past
 *      it. An edge that reaches +-60 degrees without a miss is
 *      UNBOUNDED, so the planner treats that side as unsafe.
 */

#include <stdbool.h>
#include <stdint.h>

#include "b5_cfg.h"
#include "b5_pack.h"
#include "scan_algo.h"

#define PHASE_COARSE     (0U)
#define PHASE_FINE_LEFT  (1U)
#define PHASE_FINE_RIGHT (2U)
#define PHASE_DONE       (3U)

/* Highest slot number (24); fits easily in uint8_t. */
#define LAST_SLOT (B5_SLOT_COUNT - 1U)

/* Outcome of walking one edge. */
#define WALK_MEASURE   (0U) /* p_ctx->slot must be measured next */
#define WALK_BOUNDED   (1U) /* a real miss was found             */
#define WALK_UNBOUNDED (2U) /* the walk ran past +-60 degrees     */

/* Largest angle of the sweep in deci-degrees (600). */
#define SWEEP_HALF_DECI (-B5_SLOT_FIRST_DECI)

static bool    find_closest (scan_algo_ctx_t * p_ctx, int32_t half_deci,
                             uint16_t * p_best_mm);
static void    finish_coarse (scan_algo_ctx_t * p_ctx);
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
        (width_us > B5_VALID_MAX_US) ? B5_VALID_MAX_US : width_us;
    uint32_t dist = (width * B5_SOUND_NUM) / B5_SOUND_DEN;

    if (dist > B5_DIST_MAX)
    {
        dist = B5_DIST_MAX;
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
 * @return One of B5_CLASS_*.
 */
uint8_t scan_algo_classify (uint32_t mbx, bool b_timed_out, bool b_echo_high,
                            bool b_stuck)
{
    uint8_t        cls    = B5_CLASS_UNKNOWN;
    uint32_t const width  = mbx & B5_MBX_WIDTH_MASK;
    bool const     b_rise = (0U != (mbx & B5_MBX_RISE));
    bool const     b_done = (0U != (mbx & B5_MBX_DONE));

    if (b_stuck)
    {
        cls = B5_CLASS_STUCK;
    }
    else if (b_done && b_rise)
    {
        if (width < B5_VALID_MIN_US)
        {
            /* Too short: a glitch or an object closer than 2 cm. */
            cls = B5_CLASS_UNKNOWN;
        }
        else if (width <= B5_VALID_MAX_US)
        {
            cls = B5_CLASS_VALID;
        }
        else
        {
            cls = B5_CLASS_OPEN;
        }
    }
    else if (b_timed_out && b_rise && b_echo_high)
    {
        /* Still high after 40 ms: nothing within 4 m. */
        cls = B5_CLASS_OPEN;
    }
    else
    {
        /* No rise, a fall without a rise, or a rise whose fall was missed
         * (ECHO is low now). None of these can be trusted. */
        cls = B5_CLASS_UNKNOWN;
    }

    return (cls);
}

/*!
 * @brief Servo angle of a slot.
 *
 * @param[in] slot Slot number, 0..24.
 *
 * @return Angle in deci-degrees, positive = left. Slot 12 is 0.
 */
int16_t scan_algo_slot_deci (uint8_t slot)
{
    int32_t const deci =
        B5_SLOT_FIRST_DECI + ((int32_t)slot * B5_SLOT_STEP_DECI);

    /* Cast is safe: slots 0..24 give -600..+600. */
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

    for (idx = 0U; idx < B5_MAX_PINGS; idx++)
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
    uint16_t const dist      = (B5_CLASS_VALID == cls) ? dist_mm : 0U;

    for (idx = 0U; (idx < p_agree->count) && (!b_settled); idx++)
    {
        uint16_t const old_val  = p_agree->sample[idx];
        uint16_t const old_dist = b5_pack_slot_dist(old_val);
        uint16_t const near_mm  = (old_dist < dist) ? old_dist : dist;
        uint16_t const far_mm   = (old_dist < dist) ? dist : old_dist;
        uint32_t const diff_mm  = (uint32_t)far_mm - (uint32_t)near_mm;

        if ((b5_pack_slot_class(old_val) == cls) &&
            ((B5_CLASS_VALID != cls) || (diff_mm <= B5_AGREE_MM)))
        {
            *p_slot   = b5_pack_slot(near_mm, cls);
            b_settled = true;
        }
    }

    if ((!b_settled) && (p_agree->count < B5_MAX_PINGS))
    {
        p_agree->sample[p_agree->count] = b5_pack_slot(dist, cls);
        p_agree->count++;
    }

    if ((!b_settled) && (p_agree->count >= B5_MAX_PINGS))
    {
        *p_slot   = b5_pack_slot(0U, B5_CLASS_UNKNOWN);
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
    uint32_t const limit_mm = (uint32_t)closest_mm + B5_REGION_EXTRA_MM;

    return ((b5_pack_slot_is_measured(slot_val)) &&
            (B5_CLASS_VALID == b5_pack_slot_class(slot_val)) &&
            ((uint32_t)b5_pack_slot_dist(slot_val) <= limit_mm));
}

/*!
 * @brief Tell whether a measured slot proves that nothing is there.
 *
 * @param[in] slot_val   Packed slot.
 * @param[in] closest_mm Closest distance in the region.
 *
 * @return True for OPEN, or for VALID beyond closest + 250 mm.
 */
bool scan_algo_is_miss (uint16_t slot_val, uint16_t closest_mm)
{
    uint8_t const  cls      = b5_pack_slot_class(slot_val);
    uint32_t const limit_mm = (uint32_t)closest_mm + B5_REGION_EXTRA_MM;
    bool           b_miss   = false;

    if (!b5_pack_slot_is_measured(slot_val))
    {
        b_miss = false;
    }
    else if (B5_CLASS_OPEN == cls)
    {
        b_miss = true;
    }
    else if (B5_CLASS_VALID == cls)
    {
        b_miss = ((uint32_t)b5_pack_slot_dist(slot_val) > limit_mm);
    }
    else
    {
        b_miss = false;
    }

    return (b_miss);
}

/*!
 * @brief Start a new scan.
 *
 * @param[out] p_ctx Scan engine state.
 */
void scan_algo_start (scan_algo_ctx_t * p_ctx)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < B5_SLOT_COUNT; idx++)
    {
        p_ctx->profile[idx] = 0U;
    }

    scan_algo_agree_reset(&p_ctx->agree);
    p_ctx->phase        = PHASE_COARSE;
    p_ctx->slot         = 0U;
    p_ctx->closest_slot = B5_SLOT_CENTRE;
    p_ctx->closest_mm   = 0U;
    p_ctx->region_lo    = 0U;
    p_ctx->region_hi    = 0U;

    for (idx = 0U; idx < SCAN_ALGO_SIDES; idx++)
    {
        p_ctx->edge_state[idx] = SCAN_ALGO_EDGE_NONE;
        p_ctx->edge_hit[idx]   = 0U;
        p_ctx->edge_miss[idx]  = 0U;
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
                if ((p_ctx->slot + B5_COARSE_STRIDE) <= LAST_SLOT)
                {
                    /* Cast is safe: the result is at most 24. */
                    p_ctx->slot = (uint8_t)(p_ctx->slot + B5_COARSE_STRIDE);
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
                p_ctx->phase = PHASE_DONE;
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

    for (slot = 0U; slot <= LAST_SLOT; slot += B5_COARSE_STRIDE)
    {
        /* Cast is safe: slot is at most 24. */
        int32_t const  deci     = scan_algo_slot_deci((uint8_t)slot);
        uint16_t const slot_val = p_ctx->profile[slot];
        uint16_t const dist     = b5_pack_slot_dist(slot_val);

        if ((deci >= -half_deci) && (deci <= half_deci) &&
            (b5_pack_slot_is_measured(slot_val)) &&
            (B5_CLASS_VALID == b5_pack_slot_class(slot_val)) &&
            (dist <= B5_REGION_MAX_MM) && ((!b_found) || (dist < *p_best_mm)))
        {
            *p_best_mm = dist;

            /* Cast is safe: slot is at most 24. */
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
    bool     b_found = find_closest(p_ctx, B5_PATH_HALF_DECI, &best_mm);

    if (!b_found)
    {
        /* Nothing in the car's path: fall back to the whole sweep. */
        b_found = find_closest(p_ctx, SWEEP_HALF_DECI, &best_mm);
    }

    if (!b_found)
    {
        /* Nothing within 600 mm. */
        p_ctx->phase = PHASE_DONE;
    }
    else
    {
        p_ctx->closest_mm = best_mm;
        p_ctx->region_lo  = p_ctx->closest_slot;
        p_ctx->region_hi  = p_ctx->closest_slot;

        while (
            ((p_ctx->region_hi + B5_COARSE_STRIDE) <= LAST_SLOT) &&
            (scan_algo_is_hit(
                p_ctx->profile[p_ctx->region_hi + B5_COARSE_STRIDE], best_mm)))
        {
            /* Cast is safe: the loop test keeps the result <= 24. */
            p_ctx->region_hi = (uint8_t)(p_ctx->region_hi + B5_COARSE_STRIDE);
        }

        while (
            (p_ctx->region_lo >= B5_COARSE_STRIDE) &&
            (scan_algo_is_hit(
                p_ctx->profile[p_ctx->region_lo - B5_COARSE_STRIDE], best_mm)))
        {
            /* Cast is safe: the loop test keeps the result >= 0. */
            p_ctx->region_lo = (uint8_t)(p_ctx->region_lo - B5_COARSE_STRIDE);
        }

        p_ctx->edge_hit[SCAN_ALGO_LEFT] = p_ctx->region_hi;
        run_fine(p_ctx, SCAN_ALGO_LEFT, (int32_t)p_ctx->region_hi + 1);
    }
}

/*!
 * @brief Walk one edge outward, starting at slot first, over the slots
 *        that are already measured.
 *
 * A hit moves the edge out. An UNKNOWN or STUCK slot proves nothing, so
 * the walk assumes the obstacle goes on and steps past it without moving
 * the edge (edge_hit always names a VALID reading, whose range the planner
 * uses). The walk stops at the first slot not measured yet, at the first
 * real miss, or past +-60 degrees.
 *
 * @param[in,out] p_ctx  Scan engine state.
 * @param[in]     side   SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 * @param[in]     first  First slot to look at; may be -1 or 25.
 * @param[out]    p_miss First slot beyond the edge (BOUNDED/UNBOUNDED).
 *
 * @return WALK_MEASURE, WALK_BOUNDED or WALK_UNBOUNDED.
 */
static uint8_t walk_edge (scan_algo_ctx_t * p_ctx, uint8_t side, int32_t first,
                          uint8_t * p_miss)
{
    int32_t const step   = (SCAN_ALGO_LEFT == side) ? 1 : -1;
    int32_t       slot   = first;
    uint8_t       result = WALK_MEASURE;
    bool          b_more = true;

    while (b_more)
    {
        bool const b_inside = (slot >= 0) && (slot <= (int32_t)LAST_SLOT);

        /* An outside slot reads as empty; it is never used below. */
        uint16_t const slot_val = b_inside ? p_ctx->profile[slot] : 0U;

        if (!b_inside)
        {
            /* The obstacle reaches the end of the sweep. */
            *p_miss = (SCAN_ALGO_LEFT == side) ? (uint8_t)LAST_SLOT : 0U;
            result  = WALK_UNBOUNDED;
            b_more  = false;
        }
        else if (!b5_pack_slot_is_measured(slot_val))
        {
            /* Cast is safe: slot is 0..24 here. */
            p_ctx->slot = (uint8_t)slot;
            result      = WALK_MEASURE;
            b_more      = false;
        }
        else if (scan_algo_is_miss(slot_val, p_ctx->closest_mm))
        {
            /* Cast is safe: slot is 0..24 here. */
            *p_miss = (uint8_t)slot;
            result  = WALK_BOUNDED;
            b_more  = false;
        }
        else if (scan_algo_is_hit(slot_val, p_ctx->closest_mm))
        {
            /* Cast is safe: slot is 0..24 here. */
            p_ctx->edge_hit[side] = (uint8_t)slot;
            slot += step;
        }
        else
        {
            /* UNKNOWN or STUCK: assume the obstacle goes on. */
            slot += step;
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
                p_ctx->edge_hit[SCAN_ALGO_RIGHT] = p_ctx->region_lo;
                cur_first = (int32_t)p_ctx->region_lo - 1;
            }
            else
            {
                p_ctx->phase = PHASE_DONE;
                b_more       = false;
            }
        }
    }
}

/*** end of file ***/

