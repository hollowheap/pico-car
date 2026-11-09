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
 * offset each side would need. b_wall is true when the plan came from the
 * left-wall fallback (neither edge bounded): the car then follows the wall
 * in parallel instead of going out and back. */
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

/* HUG direction from obstacle_plan_hug_ctrl(). */
#define OBSTACLE_HUG_STRAIGHT (0U)
#define OBSTACLE_HUG_AWAY     (1U)
#define OBSTACLE_HUG_TOWARD   (2U)

/* Bypass phases inside REACQUIRE. The values appear in the flight log. */
#define OBSTACLE_PHASE_TURN  (0U)
#define OBSTACLE_PHASE_OUT   (1U)
#define OBSTACLE_PHASE_HUG   (2U)
#define OBSTACLE_PHASE_ALIGN (3U)

/* Proportional hug command: direction and inner wheel speed as permille of
 * the outer wheel speed. */
typedef struct
{
    uint8_t  dir;
    uint16_t inside_permille;
} obstacle_hug_cmd_t;

/* Inputs of the hug law. reading_mm is the filtered side reading (median of
 * three), latest_mm the newest side reading; both are OBSTACLE_SIDE_FAR_MM
 * for a ping that was not VALID. cls is the class of the newest side ping.
 * head_deci is the heading from the original line, positive away from the
 * obstacle (left for a left pass). */
typedef struct
{
    uint8_t  cls;
    uint16_t reading_mm;
    uint16_t latest_mm;
    int32_t  head_deci;
    bool     b_wall;
} obstacle_hug_in_t;

/* Everything the bypass driver needs for one pass (obstacle_plan_drive and
 * obstacle_plan_pivot). need_mm and lateral_mm are for a box plan. */
typedef struct
{
    bool     b_wall;
    uint8_t  side;
    uint8_t  phase;
    uint8_t  start_deg;
    int32_t  head_deci;
    int32_t  lateral_mm;
    int32_t  need_mm;
    uint8_t  side_cls;
    uint16_t median_mm;
    uint16_t latest_mm;
} obstacle_drive_in_t;

/* What the bypass driver asks for: the phase afterwards, the steering
 * (OBSTACLE_STEER_*, HOLD stands still), the hug direction for the log,
 * the wheel speeds in um/s (inside = the wheel next to the obstacle), and
 * b_turned once the planned pivot angle is reached. */
typedef struct
{
    uint8_t phase;
    uint8_t steer;
    uint8_t dir;
    int32_t inside_um_s;
    int32_t outside_um_s;
    bool    b_turned;
} obstacle_drive_out_t;

/* The newest side readings of the hug (forward pings are not kept), and the
 * wall-mode heading trim (obstacle_plan_wall_head): trim_cdeci is in
 * hundredths of a deci-degree. */
#define OBSTACLE_SIDE_FILTER_LEN (3U)

typedef struct
{
    uint16_t mm[OBSTACLE_SIDE_FILTER_LEN];
    uint8_t  cls;
    uint8_t  last_seq;
    uint8_t  count;
    uint8_t  next;
    bool     b_fresh;
    int32_t  trim_cdeci;
} obstacle_side_filter_t;

void obstacle_plan_cfg_default (obstacle_plan_cfg_t * p_cfg);
void obstacle_plan_build (scan_algo_ctx_t const * p_scan, obstacle_plan_cfg_t const * p_cfg,
                    obstacle_plan_result_t * p_result);

bool    obstacle_plan_is_escape_clear (uint8_t cls, uint16_t dist_mm,
                                 uint16_t closest_mm);
void    obstacle_plan_hug_ctrl (obstacle_hug_in_t const * p_in,
                                obstacle_hug_cmd_t * p_cmd);
void    obstacle_plan_drive (obstacle_drive_in_t const * p_in,
                             obstacle_drive_out_t * p_out);
void    obstacle_plan_pivot (obstacle_drive_in_t const * p_in,
                             obstacle_drive_out_t * p_out);
bool    obstacle_plan_is_look_ahead (uint8_t seq);
void    obstacle_plan_side_reset (obstacle_side_filter_t * p_filt, uint8_t seq);
void    obstacle_plan_side_feed (obstacle_side_filter_t * p_filt, uint8_t seq,
                                 uint8_t cls, uint16_t dist_mm);
uint16_t obstacle_plan_side_median (obstacle_side_filter_t const * p_filt);
uint16_t obstacle_plan_side_latest (obstacle_side_filter_t const * p_filt);
uint8_t  obstacle_plan_side_class (obstacle_side_filter_t const * p_filt);
int32_t  obstacle_plan_wall_head (obstacle_side_filter_t * p_filt, int32_t head_deci,
                                  bool b_track);
uint32_t obstacle_plan_hug_limit_mm (obstacle_plan_result_t const * p_result,
                                     obstacle_plan_cfg_t const * p_cfg);
uint8_t obstacle_plan_steer_away (uint8_t side);

int32_t  obstacle_plan_sin_q14 (int32_t deci);
int32_t  obstacle_plan_cos_q14 (int32_t deci);
int32_t  obstacle_plan_tan_q14 (int32_t deci);
int32_t  obstacle_plan_atan_deci (uint32_t num, uint32_t den);
uint32_t obstacle_plan_isqrt (uint32_t value);

#endif /* OBSTACLE_PLANNER_H */

/*** end of file ***/

