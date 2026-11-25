/** @file scan_algo.h
 *
 * @brief Echo classification, ping agreement and the ±30° servo scan with
 *        body-pivot windows. Pure C99, integer only and free of kernel
 *        calls.
 */

#ifndef SCAN_ALGO_H
#define SCAN_ALGO_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_config.h"

/* Returned by scan_algo_target() when no slot is left to measure. */
#define SCAN_ALGO_NO_SLOT (0xFFU)

/* Edge result for one side of the region. NONE means there is no region,
 * BOUNDED means the last hit and the first miss are known inside the
 * trusted incidence angle, and UNBOUNDED means the obstacle reached the
 * sweep limit or the trusted incidence limit without a real miss. */
#define SCAN_ALGO_EDGE_NONE      (0U)
#define SCAN_ALGO_EDGE_BOUNDED   (1U)
#define SCAN_ALGO_EDGE_UNBOUNDED (2U)

/* Edge index: the left edge is at the higher slot numbers. */
#define SCAN_ALGO_LEFT  (0U)
#define SCAN_ALGO_RIGHT (1U)
#define SCAN_ALGO_SIDES (2U)

/* Pings collected for one slot until two agree. Each sample is packed
 * like a slot (distance 12 bits, class 3 bits). */
typedef struct
{
    uint16_t sample[OBSTACLE_MAX_PINGS];
    uint8_t  count;
} scan_algo_agree_t;

/* Scan engine state. The profile is 37 packed slots (-90..+90 deg in 5 deg
 * steps), 74 bytes. */
typedef struct
{
    uint16_t          profile[OBSTACLE_SLOT_COUNT];
    scan_algo_agree_t agree;
    uint8_t           phase;
    uint8_t           slot;
    int16_t           body_deci;
    uint8_t           closest_slot;
    uint16_t          closest_mm;
    uint8_t           region_lo;
    uint8_t           region_hi;
    uint8_t           edge_state[SCAN_ALGO_SIDES];
    uint8_t           edge_hit[SCAN_ALGO_SIDES];
    uint8_t           edge_miss[SCAN_ALGO_SIDES];
    bool              b_pivoted[SCAN_ALGO_SIDES];
    uint8_t           noisy_hist;    /* P1: one bit per ping, newest in bit 0 */
    uint8_t           slots_done;    /* P5: slots settled in this scan        */
    uint8_t           slots_retried; /* P5: slots that needed 3 or 4 pings    */
    bool              b_scan_noisy;  /* P5: set when a scan finishes noisy    */
} scan_algo_ctx_t;

uint16_t scan_algo_dist_mm (uint32_t width_us);
uint8_t  scan_algo_classify (uint32_t echo_result, bool b_timed_out, bool b_echo_high,
                              bool b_stuck);
int16_t  scan_algo_slot_deci (uint8_t slot);
int16_t  scan_algo_body_deci (scan_algo_ctx_t const * p_ctx);
int16_t  scan_algo_servo_deci (scan_algo_ctx_t const * p_ctx);

void scan_algo_agree_reset (scan_algo_agree_t * p_agree);
bool scan_algo_agree_add (scan_algo_agree_t * p_agree, uint8_t cls,
                          uint16_t dist_mm, uint16_t * p_slot);

bool scan_algo_is_hit (uint16_t slot_val, uint16_t closest_mm);
bool scan_algo_is_miss (uint16_t slot_val, uint16_t ref_mm);

void    scan_algo_start (scan_algo_ctx_t * p_ctx);
uint8_t scan_algo_target (scan_algo_ctx_t const * p_ctx);
bool    scan_algo_feed (scan_algo_ctx_t * p_ctx, uint8_t cls, uint16_t dist_mm);
bool    scan_algo_has_region (scan_algo_ctx_t const * p_ctx);
bool    scan_algo_echo_noisy (scan_algo_ctx_t const * p_ctx);

/* Servo travel check (D162). A square wall straight ahead returns d(angle) =
 * d0 / cos(angle). The ratio of the readings at the +30 and -30 degree slots
 * to the reading straight ahead gives the angle the servo really reached. */
typedef enum
{
    SERVO_CHECK_OK = 0,       /* both sides within 3 degrees of 30                 */
    SERVO_CHECK_NO_DATA,      /* a slot is missing, not VALID, or the wall is far  */
    SERVO_CHECK_FLAT,         /* both ratios near 1: the servo hardly moves        */
    SERVO_CHECK_SCALE,        /* it moves, but not 30 degrees: tune US_PER_10DEG   */
    SERVO_CHECK_ASYMMETRIC    /* the two sides differ: the wall is skewed or the
                                 centre is off (OBSTACLE_SERVO_CENTRE_US)          */
} servo_check_verdict_t;

typedef struct
{
    uint8_t  verdict;     /* servo_check_verdict_t                              */
    int16_t  left_deci;   /* angle reached at the +30 slot, deci-degrees        */
    int16_t  right_deci;  /* angle reached at the -30 slot, deci-degrees        */
    uint16_t centre_mm;   /* reading straight ahead                             */
} servo_check_t;

/* Evaluate a finished profile of OBSTACLE_SLOT_COUNT packed slots. */
void scan_algo_servo_check (uint16_t const * p_profile, servo_check_t * p_out);

#endif /* SCAN_ALGO_H */

/*** end of file ***/
