/** @file scan.h
 *
 * @brief Buddy 5 echo classification, ping agreement and the coarse to
 *        fine scan engine. Pure C99, integer only and free of kernel
 *        calls.
 */

#ifndef EXTRAS_SCAN_H
#define EXTRAS_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

/* Returned by extras_scan_target() when no slot is left to measure. */
#define EXTRAS_SCAN_NO_SLOT (0xFFU)

/* Edge result for one side of the region. NONE means there is no region,
 * BOUNDED means the last hit and the first miss are known, and UNBOUNDED
 * means the obstacle reached +-60 degrees without a real miss. */
#define EXTRAS_SCAN_EDGE_NONE      (0U)
#define EXTRAS_SCAN_EDGE_BOUNDED   (1U)
#define EXTRAS_SCAN_EDGE_UNBOUNDED (2U)

/* Edge index: the left edge is at the higher slot numbers. */
#define EXTRAS_SCAN_LEFT  (0U)
#define EXTRAS_SCAN_RIGHT (1U)
#define EXTRAS_SCAN_SIDES (2U)

/* Pings collected for one slot until two agree. Each sample is packed
 * like a slot (distance 12 bits, class 2 bits). */
typedef struct
{
    uint16_t sample[EXTRAS_MAX_PINGS];
    uint8_t  count;
} extras_scan_agree_t;

/* Scan engine state. The profile is 25 packed slots, 50 bytes. */
typedef struct
{
    uint16_t          profile[EXTRAS_SLOT_COUNT];
    extras_scan_agree_t agree;
    uint8_t           phase;
    uint8_t           slot;
    uint8_t           closest_slot;
    uint16_t          closest_mm;
    uint8_t           region_lo;
    uint8_t           region_hi;
    uint8_t           edge_state[EXTRAS_SCAN_SIDES];
    uint8_t           edge_hit[EXTRAS_SCAN_SIDES];
    uint8_t           edge_miss[EXTRAS_SCAN_SIDES];
    uint8_t           face_slot;
    bool              b_flat;
} extras_scan_ctx_t;

uint16_t extras_scan_dist_mm (uint32_t width_us);
uint8_t  extras_scan_classify (uint32_t mbx, bool b_timed_out, bool b_echo_high,
                             bool b_stuck);
int16_t  extras_scan_slot_deci (uint8_t slot);

void extras_scan_agree_reset (extras_scan_agree_t * p_agree);
bool extras_scan_agree_add (extras_scan_agree_t * p_agree, uint8_t cls,
                          uint16_t dist_mm, uint16_t * p_slot);

bool extras_scan_is_hit (uint16_t slot_val, uint16_t closest_mm);
bool extras_scan_is_miss (uint16_t slot_val, uint16_t closest_mm);
bool extras_scan_is_side_echo (extras_scan_ctx_t const * p_ctx, uint8_t slot);

void    extras_scan_start (extras_scan_ctx_t * p_ctx);
uint8_t extras_scan_target (extras_scan_ctx_t const * p_ctx);
bool    extras_scan_feed (extras_scan_ctx_t * p_ctx, uint8_t cls, uint16_t dist_mm);
bool    extras_scan_has_region (extras_scan_ctx_t const * p_ctx);

#endif /* EXTRAS_SCAN_H */

/*** end of file ***/
