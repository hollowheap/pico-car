/** @file scan_algo.c
 *
 * @brief Buddy 5 echo classification, ping agreement and the coarse to
 *        fine scan engine. Pure C99 and integer only.
 *
 * Scan order (Spec 3a-3d):
 *   1. Coarse: slots 0, 3, ... 24, from right to left.
 *   2. Region: the closest VALID reading within 600 mm, grown over coarse
 *      neighbours that are VALID and within closest + 250 mm.
 *   3. Fine, left edge first because the servo is already on the left:
 *      the in-between slots are measured outward until the first miss.
 *      An edge at +-60 degrees, or one whose next reading is not a real
 *      miss, is UNBOUNDED, so the planner treats that side as unsafe.
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

static void begin_right (scan_algo_ctx_t * p_ctx);
static void begin_left (scan_algo_ctx_t * p_ctx);
static void finish_coarse (scan_algo_ctx_t * p_ctx);
static void close_edge (scan_algo_ctx_t * p_ctx, uint8_t side, uint8_t state,
                        uint8_t miss_slot);
static void step_fine_hit (scan_algo_ctx_t * p_ctx, uint8_t side);
static void step_fine (scan_algo_ctx_t * p_ctx, uint8_t side);

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
                step_fine(p_ctx, SCAN_ALGO_LEFT);
            break;

            case PHASE_FINE_RIGHT:
                step_fine(p_ctx, SCAN_ALGO_RIGHT);
            break;

            default:
                p_ctx->phase = PHASE_DONE;
            break;
        }
    }

    return (PHASE_DONE == p_ctx->phase);
}

/*!
 * @brief Begin the fine scan of the right edge, or skip it at -60 degrees.
 *
 * @param[in,out] p_ctx Scan engine state.
 */
static void begin_right (scan_algo_ctx_t * p_ctx)
{
    p_ctx->edge_hit[SCAN_ALGO_RIGHT] = p_ctx->region_lo;

    if (0U == p_ctx->region_lo)
    {
        p_ctx->edge_state[SCAN_ALGO_RIGHT] = SCAN_ALGO_EDGE_UNBOUNDED;
        p_ctx->phase                       = PHASE_DONE;
    }
    else
    {
        p_ctx->phase = PHASE_FINE_RIGHT;

        /* Cast is safe: region_lo is at least 1 here. */
        p_ctx->slot = (uint8_t)(p_ctx->region_lo - 1U);
    }
}

/*!
 * @brief Begin the fine scan of the left edge, or skip it at +60 degrees.
 *
 * @param[in,out] p_ctx Scan engine state.
 */
static void begin_left (scan_algo_ctx_t * p_ctx)
{
    p_ctx->edge_hit[SCAN_ALGO_LEFT] = p_ctx->region_hi;

    if (p_ctx->region_hi >= LAST_SLOT)
    {
        p_ctx->edge_state[SCAN_ALGO_LEFT] = SCAN_ALGO_EDGE_UNBOUNDED;
        begin_right(p_ctx);
    }
    else
    {
        p_ctx->phase = PHASE_FINE_LEFT;

        /* Cast is safe: region_hi is at most 23 here. */
        p_ctx->slot = (uint8_t)(p_ctx->region_hi + 1U);
    }
}

/*!
 * @brief After the last coarse slot: find the region and start the fine
 *        scan.
 *
 * @param[in,out] p_ctx Scan engine state.
 */
static void finish_coarse (scan_algo_ctx_t * p_ctx)
{
    uint32_t slot    = 0U;
    bool     b_found = false;
    uint16_t best_mm = 0U;

    for (slot = 0U; slot <= LAST_SLOT; slot += B5_COARSE_STRIDE)
    {
        uint16_t const slot_val = p_ctx->profile[slot];
        uint16_t const dist     = b5_pack_slot_dist(slot_val);

        if ((b5_pack_slot_is_measured(slot_val)) &&
            (B5_CLASS_VALID == b5_pack_slot_class(slot_val)) &&
            (dist <= B5_REGION_MAX_MM) && ((!b_found) || (dist < best_mm)))
        {
            best_mm = dist;

            /* Cast is safe: slot is at most 24. */
            p_ctx->closest_slot = (uint8_t)slot;
            b_found             = true;
        }
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

        begin_left(p_ctx);
    }
}

/*!
 * @brief Close one edge and move on to the next one.
 *
 * @param[in,out] p_ctx     Scan engine state.
 * @param[in]     side      SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 * @param[in]     state     SCAN_ALGO_EDGE_BOUNDED or _UNBOUNDED.
 * @param[in]     miss_slot First slot beyond the edge.
 */
static void close_edge (scan_algo_ctx_t * p_ctx, uint8_t side, uint8_t state,
                        uint8_t miss_slot)
{
    p_ctx->edge_state[side] = state;
    p_ctx->edge_miss[side]  = miss_slot;

    if (SCAN_ALGO_LEFT == side)
    {
        begin_right(p_ctx);
    }
    else
    {
        p_ctx->phase = PHASE_DONE;
    }
}

/*!
 * @brief The fine slot just measured still hits: go one slot further out,
 *        or judge the coarse slot when both in-between slots hit.
 *
 * @param[in,out] p_ctx Scan engine state.
 * @param[in]     side  SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 */
static void step_fine_hit (scan_algo_ctx_t * p_ctx, uint8_t side)
{
    /* Casts are safe: fine slots stay inside 1..23, so the neighbours and
     * the next coarse slot stay inside 0..24. */
    uint8_t const coarse = (SCAN_ALGO_LEFT == side)
                               ? (uint8_t)(p_ctx->region_hi + B5_COARSE_STRIDE)
                               : (uint8_t)(p_ctx->region_lo - B5_COARSE_STRIDE);
    uint8_t const next = (SCAN_ALGO_LEFT == side) ? (uint8_t)(p_ctx->slot + 1U)
                                                  : (uint8_t)(p_ctx->slot - 1U);

    p_ctx->edge_hit[side] = p_ctx->slot;

    if (next != coarse)
    {
        p_ctx->slot = next;
    }
    else if (scan_algo_is_miss(p_ctx->profile[coarse], p_ctx->closest_mm))
    {
        close_edge(p_ctx, side, SCAN_ALGO_EDGE_BOUNDED, coarse);
    }
    else
    {
        /* The coarse slot was UNKNOWN or STUCK: the edge is not proven. */
        close_edge(p_ctx, side, SCAN_ALGO_EDGE_UNBOUNDED, coarse);
    }
}

/*!
 * @brief Decide what comes after a fine slot.
 *
 * @param[in,out] p_ctx Scan engine state.
 * @param[in]     side  SCAN_ALGO_LEFT or SCAN_ALGO_RIGHT.
 */
static void step_fine (scan_algo_ctx_t * p_ctx, uint8_t side)
{
    uint16_t const slot_val = p_ctx->profile[p_ctx->slot];

    if (scan_algo_is_hit(slot_val, p_ctx->closest_mm))
    {
        step_fine_hit(p_ctx, side);
    }
    else if (scan_algo_is_miss(slot_val, p_ctx->closest_mm))
    {
        close_edge(p_ctx, side, SCAN_ALGO_EDGE_BOUNDED, p_ctx->slot);
    }
    else
    {
        close_edge(p_ctx, side, SCAN_ALGO_EDGE_UNBOUNDED, p_ctx->slot);
    }
}

/*** end of file ***/

