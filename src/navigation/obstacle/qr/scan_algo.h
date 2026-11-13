/** @file scan_algo.h
 *
 * @brief Buddy 5 echo classification, ping agreement and the coarse to
 *        fine scan engine. Pure C99, integer only and free of kernel
 *        calls.
 */

#ifndef SCAN_ALGO_H
#define SCAN_ALGO_H

#include <stdbool.h>
#include <stdint.h>

#include "b5_cfg.h"

/* Returned by scan_algo_target() when no slot is left to measure. */
#define SCAN_ALGO_NO_SLOT (0xFFU)

/* Edge result for one side of the region. NONE means there is no region,
 * BOUNDED means the last hit and the first miss are known, and UNBOUNDED
 * means the obstacle reached +-60 degrees without a real miss. */
#define SCAN_ALGO_EDGE_NONE      (0U)
#define SCAN_ALGO_EDGE_BOUNDED   (1U)
#define SCAN_ALGO_EDGE_UNBOUNDED (2U)

/* Edge index: the left edge is at the higher slot numbers. */
#define SCAN_ALGO_LEFT  (0U)
#define SCAN_ALGO_RIGHT (1U)
#define SCAN_ALGO_SIDES (2U)

/* Pings collected for one slot until two agree. Each sample is packed
 * like a slot (distance 12 bits, class 2 bits). */
typedef struct
{
    uint16_t sample[B5_MAX_PINGS];
    uint8_t  count;
} scan_algo_agree_t;

/* Scan engine state. The profile is 25 packed slots, 50 bytes. */
typedef struct
{
    uint16_t          profile[B5_SLOT_COUNT];
    scan_algo_agree_t agree;
    uint8_t           phase;
    uint8_t           slot;
    uint8_t           closest_slot;
    uint16_t          closest_mm;
    uint8_t           region_lo;
    uint8_t           region_hi;
    uint8_t           edge_state[SCAN_ALGO_SIDES];
    uint8_t           edge_hit[SCAN_ALGO_SIDES];
    uint8_t           edge_miss[SCAN_ALGO_SIDES];
} scan_algo_ctx_t;

uint16_t scan_algo_dist_mm (uint32_t width_us);
uint8_t  scan_algo_classify (uint32_t mbx, bool b_timed_out, bool b_echo_high,
                             bool b_stuck);
int16_t  scan_algo_slot_deci (uint8_t slot);

void scan_algo_agree_reset (scan_algo_agree_t * p_agree);
bool scan_algo_agree_add (scan_algo_agree_t * p_agree, uint8_t cls,
                          uint16_t dist_mm, uint16_t * p_slot);

bool scan_algo_is_hit (uint16_t slot_val, uint16_t closest_mm);
bool scan_algo_is_miss (uint16_t slot_val, uint16_t closest_mm);

void    scan_algo_start (scan_algo_ctx_t * p_ctx);
uint8_t scan_algo_target (scan_algo_ctx_t const * p_ctx);
bool    scan_algo_feed (scan_algo_ctx_t * p_ctx, uint8_t cls, uint16_t dist_mm);
bool    scan_algo_has_region (scan_algo_ctx_t const * p_ctx);

#endif /* SCAN_ALGO_H */

/*** end of file ***/
