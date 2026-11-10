/** @file planner.h
 *
 * @brief Obstacle planner and stop-and-look bypass driver (D96, S40).
 *        Pure C99, integer only, no libm and no kernel calls.
 */

#ifndef OBSTACLE_PLANNER_H
#define OBSTACLE_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "geometry.h"
#include "scan_algo.h"

/* need_mm value for a side that cannot be used. */
#define OBSTACLE_PLAN_SIDE_BLOCKED (0xFFFFU)

/* Width reported when at least one edge is unbounded. */
#define OBSTACLE_WIDTH_MAX (1023U)

/* Car and course numbers the planner needs. */
typedef struct
{
    uint16_t half_width_mm;
    uint16_t side_margin_mm;
    uint16_t box_gap_mm;
    uint16_t free_floor_mm;
    uint16_t depth_mm;
    uint16_t rear_mm;
    int16_t  beam_half_deci;
    int16_t  path_half_deci;
    int16_t  edge_trust_deci;
} obstacle_plan_cfg_t;

/* Scan result. width_mm 1023 means "1023 mm or wider"; start_deg is the
 * out-leg pivot angle in degrees; need_mm is the sideways axle offset each
 * side needs. b_wall is true when neither edge is trusted inside
 * +-OBSTACLE_EDGE_TRUST_DECI: the car pivots left to +90 deg and follows the
 * wall on its right at 300 mm. */
typedef struct
{
    uint8_t  status;
    uint8_t  side;
    uint16_t width_mm;
    uint8_t  start_deg;
    uint16_t closest_mm;
    uint16_t need_mm[SCAN_ALGO_SIDES];
    bool     b_wall;
} obstacle_plan_result_t;

/* Bypass phases inside BYPASS / REACQUIRE. */
#define OBSTACLE_PHASE_TURN         (0U)
#define OBSTACLE_PHASE_OUT          (1U)
#define OBSTACLE_PHASE_ALIGN        (2U)
#define OBSTACLE_PHASE_ALONG        (3U)
#define OBSTACLE_PHASE_LOOK_STOP    (4U)
#define OBSTACLE_PHASE_LOOK_IN      (5U)
#define OBSTACLE_PHASE_LOOK_PING    (6U)
#define OBSTACLE_PHASE_LOOK_OUT     (7U)
#define OBSTACLE_PHASE_EXTRA        (8U)
#define OBSTACLE_PHASE_RETURN_PIVOT (9U)
#define OBSTACLE_PHASE_RETURN_DRIVE (10U)

void    obstacle_plan_cfg_default (obstacle_plan_cfg_t * p_cfg);
void    obstacle_plan_build (scan_algo_ctx_t const * p_scan, obstacle_plan_cfg_t const * p_cfg,
                             obstacle_plan_result_t * p_result);
bool    obstacle_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                                       uint16_t closest_mm);
uint32_t obstacle_plan_hug_limit_mm (obstacle_plan_result_t const * p_result,
                                     obstacle_plan_cfg_t const * p_cfg);
uint8_t  obstacle_plan_steer_away (uint8_t side);

void obstacle_plan_wheel_pivot (int32_t cur_head_deci, int32_t target_head_deci,
                                int32_t * p_left_um_s, int32_t * p_right_um_s,
                                uint8_t * p_steer, bool * p_b_done);
void obstacle_plan_wheel_straight (int32_t cur_head_deci, int32_t target_head_deci,
                                   int32_t speed_mm_s, int32_t * p_left_um_s,
                                   int32_t * p_right_um_s, uint8_t * p_steer);
int32_t obstacle_plan_side_lean_deci (int32_t gap_mm, int32_t prev_gap_mm,
                                      bool b_have_prev, int32_t target_gap_mm);
int32_t obstacle_plan_side_trim_deci (int32_t gap_mm, int32_t prev_gap_mm,
                                      int32_t prev_lean_deci);

int32_t  obstacle_plan_sin_q14 (int32_t deci);
int32_t  obstacle_plan_cos_q14 (int32_t deci);
int32_t  obstacle_plan_tan_q14 (int32_t deci);
int32_t  obstacle_plan_atan_deci (uint32_t num, uint32_t den);
uint32_t obstacle_plan_isqrt (uint32_t value);

#endif /* OBSTACLE_PLANNER_H */

/*** end of file ***/
