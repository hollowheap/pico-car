/** @file planner.h
 *
 * @brief Buddy 5 planner: beam-corrected obstacle width, side choice,
 *        starting angle, the escape rule and the geometry of the revised
 *        bypass (extras_nav.c). Pure C99, integer only, no libm.
 */

#ifndef EXTRAS_PLAN_H
#define EXTRAS_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#include "scan.h"

/* need_mm value for a side that cannot be used. */
#define EXTRAS_PLAN_SIDE_BLOCKED (0xFFFFU)

/* Side scan reading with no VALID echo. */
#define EXTRAS_PLAN_NO_ECHO (0xFFFFU)

/* clear_deci of an edge that is not bounded: one 5 degree slot past the
 * 60 degree servo limit (checked in extras_plan.c), so A is always the cap. */
#define EXTRAS_PLAN_CLEAR_UNBOUNDED (520)

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
} extras_plan_cfg_t;

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
    uint16_t need_mm[EXTRAS_SCAN_SIDES];
    int16_t  clear_deci[EXTRAS_SCAN_SIDES];
} extras_plan_result_t;

/* One side scan of the revised bypass (extras_plan_side_scan()). */
typedef struct
{
    bool     b_lane_clear;
    bool     b_cleared;
    int16_t  clear_deci;
    uint16_t closest_mm;
    uint32_t move_mm;
} extras_plan_side_t;

void extras_plan_cfg_default (extras_plan_cfg_t * p_cfg);
void extras_plan_build (extras_scan_ctx_t const * p_scan, extras_plan_cfg_t const * p_cfg,
                    extras_plan_result_t * p_result);

bool    extras_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                                 uint16_t closest_mm);
uint8_t extras_plan_steer_away (uint8_t side);
uint8_t extras_plan_steer_toward (uint8_t side);

int32_t  extras_plan_ahead_mm (uint32_t closest_mm);
uint32_t extras_plan_room_mm (uint32_t closest_mm, int32_t clear_deci);
uint32_t extras_plan_out_mm (uint32_t room_mm);
uint32_t extras_plan_level_mm (uint32_t closest_mm, int32_t ahead_mm);
void     extras_plan_side_scan (uint16_t const * p_mm, int32_t fwd_sign,
                            uint32_t line_mm, int32_t pan_deci,
                            extras_plan_side_t * p_side);
int32_t  extras_plan_point_side_mm (int32_t side_mm, int32_t away_deci,
                                uint32_t ahead_mm);

/* Chicken-head rotation scan (see extras_nav.c). */
void    extras_plan_track_point (int32_t psi_deci, int32_t gaze_deci,
                             uint32_t range_mm, int32_t * p_fwd_mm,
                             int32_t * p_side_mm);
bool    extras_plan_track_blocked (int32_t fwd_mm, int32_t side_mm,
                               uint32_t lookahead_mm);
bool    extras_plan_track_near (int32_t fwd_mm, int32_t side_mm);
int32_t extras_plan_track_gap_mm (int32_t side_mm);

int32_t  extras_plan_sin_q14 (int32_t deci);
int32_t  extras_plan_cos_q14 (int32_t deci);
int32_t  extras_plan_tan_q14 (int32_t deci);
int32_t  extras_plan_atan_deci (uint32_t num, uint32_t den);
uint32_t extras_plan_isqrt (uint32_t value);

#endif /* EXTRAS_PLAN_H */

/*** end of file ***/
