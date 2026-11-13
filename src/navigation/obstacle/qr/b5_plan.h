/** @file b5_plan.h
 *
 * @brief Buddy 5 planner: beam-corrected obstacle width, side choice,
 *        starting angle, the escape rule and the geometry of the revised
 *        bypass (b5_nav.c). Pure C99, integer only, no libm.
 */

#ifndef B5_PLAN_H
#define B5_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#include "scan_algo.h"

/* need_mm value for a side that cannot be used. */
#define B5_PLAN_SIDE_BLOCKED (0xFFFFU)

/* Side scan reading with no VALID echo. */
#define B5_PLAN_NO_ECHO (0xFFFFU)

/* clear_deci of an edge that is not bounded: one 5 degree slot past the
 * 60 degree servo limit (checked in b5_plan.c), so A is always the cap. */
#define B5_PLAN_CLEAR_UNBOUNDED (650)

/* Car and course numbers the planner needs. */
typedef struct
{
    uint16_t half_width_mm;
    uint16_t side_margin_mm;
    uint16_t free_floor_mm;
    uint16_t depth_mm;
    uint16_t rear_mm;
    int16_t  beam_half_deci;
    int16_t  path_half_deci;
} b5_plan_cfg_t;

/* Scan result. width_mm 1023 means "1023 mm or wider"; start_deg is how
 * far to turn to aim just past the chosen edge; need_mm is the sideways
 * offset each side would need; clear_deci is each side's raw cleared
 * angle (the first missed slot past the edge, positive = outwards; past
 * the servo limit when the edge is not bounded). */
typedef struct
{
    uint8_t  status;
    uint8_t  side;
    uint16_t width_mm;
    uint8_t  start_deg;
    uint16_t closest_mm;
    uint16_t need_mm[SCAN_ALGO_SIDES];
    int16_t  clear_deci[SCAN_ALGO_SIDES];
} b5_plan_result_t;

/* One side scan of the revised bypass (b5_plan_side_scan()). */
typedef struct
{
    bool     b_lane_clear;
    bool     b_cleared;
    int16_t  clear_deci;
    uint16_t closest_mm;
    uint32_t move_mm;
} b5_plan_side_t;

void b5_plan_cfg_default (b5_plan_cfg_t * p_cfg);
void b5_plan_build (scan_algo_ctx_t const * p_scan, b5_plan_cfg_t const * p_cfg,
                    b5_plan_result_t * p_result);

bool    b5_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                                 uint16_t closest_mm);
uint8_t b5_plan_steer_away (uint8_t side);
uint8_t b5_plan_steer_toward (uint8_t side);

uint32_t b5_plan_backup_mm (uint32_t closest_mm);
uint32_t b5_plan_room_mm (uint32_t closest_mm, int32_t clear_deci);
uint32_t b5_plan_out_mm (uint32_t room_mm);
uint32_t b5_plan_level_mm (uint32_t closest_mm);
void     b5_plan_side_scan (uint16_t const * p_mm, int32_t fwd_sign,
                            uint32_t lane_mm, b5_plan_side_t * p_side);
int32_t  b5_plan_point_side_mm (int32_t side_mm, int32_t away_deci,
                                uint32_t ahead_mm);

int32_t  b5_plan_sin_q14 (int32_t deci);
int32_t  b5_plan_cos_q14 (int32_t deci);
int32_t  b5_plan_tan_q14 (int32_t deci);
int32_t  b5_plan_atan_deci (uint32_t num, uint32_t den);
uint32_t b5_plan_isqrt (uint32_t value);

#endif /* B5_PLAN_H */

/*** end of file ***/
