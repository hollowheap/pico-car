/** @file planner.h
 *
 * @brief Buddy 5 planner: beam-corrected obstacle width, side choice,
 *        starting angle and the per-ping steering rules for the
 *        feel-your-way-round bypass. Pure C99, integer only, no libm.
 */

#ifndef OBSTACLE_PLANNER_H
#define OBSTACLE_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "scan_algo.h"

/* need_mm value for a side that cannot be used. */
#define OBSTACLE_PLAN_SIDE_BLOCKED (0xFFFFU)

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
} obstacle_plan_cfg_t;

/* Scan result. width_mm 1023 means "1023 mm or wider"; start_deg is how
 * far to turn to aim just past the chosen edge; need_mm is the sideways
 * offset each side would need. */
typedef struct
{
    uint8_t  status;
    uint8_t  side;
    uint16_t width_mm;
    uint8_t  start_deg;
    uint16_t closest_mm;
    uint16_t need_mm[SCAN_ALGO_SIDES];
} obstacle_plan_result_t;

/* HUG direction from obstacle_plan_hug_ctrl(). */
#define OBSTACLE_HUG_STRAIGHT (0U)
#define OBSTACLE_HUG_AWAY     (1U)
#define OBSTACLE_HUG_TOWARD   (2U)

/* Proportional hug command: direction and inner wheel speed as permille of
 * the outer wheel speed. */
typedef struct
{
    uint8_t  dir;
    uint16_t inside_permille;
} obstacle_hug_cmd_t;

void obstacle_plan_cfg_default (obstacle_plan_cfg_t * p_cfg);
void obstacle_plan_build (scan_algo_ctx_t const * p_scan, obstacle_plan_cfg_t const * p_cfg,
                    obstacle_plan_result_t * p_result);

bool    obstacle_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                                 uint16_t closest_mm);
uint8_t obstacle_plan_hug_steer (uint8_t side, uint8_t cls, uint16_t dist_mm,
                           uint8_t last_steer);
void    obstacle_plan_hug_ctrl (uint8_t cls, uint16_t reading_mm,
                                obstacle_hug_cmd_t * p_cmd);
uint32_t obstacle_plan_hug_limit_mm (obstacle_plan_result_t const * p_result,
                                     obstacle_plan_cfg_t const * p_cfg);
uint8_t obstacle_plan_steer_away (uint8_t side);
uint8_t obstacle_plan_steer_toward (uint8_t side);

int32_t  obstacle_plan_sin_q14 (int32_t deci);
int32_t  obstacle_plan_cos_q14 (int32_t deci);
int32_t  obstacle_plan_tan_q14 (int32_t deci);
int32_t  obstacle_plan_atan_deci (uint32_t num, uint32_t den);
uint32_t obstacle_plan_isqrt (uint32_t value);

#endif /* OBSTACLE_PLANNER_H */

/*** end of file ***/

