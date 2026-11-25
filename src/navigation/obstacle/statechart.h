/** @file statechart.h
 *
 * @brief Buddy 5 statechart (design review section 6.2). Pure C99 with no
 *        kernel calls: the task feeds it commands, pings and time, and
 *        carries out what it asks for (events, steering, emergency stop).
 *        This split keeps every decision in plain, kernel-free C.
 */

#ifndef OBSTACLE_STATECHART_H
#define OBSTACLE_STATECHART_H

#include <stdbool.h>
#include <stdint.h>

#include "planner.h"
#include "scan_algo.h"

/* At most two events come out of one step. */
#define OBSTACLE_STATECHART_OUT_MAX_EVT (2U)

/* What the statechart asks the task to do after one step. */
typedef struct
{
    uint32_t evt[OBSTACLE_STATECHART_OUT_MAX_EVT];
    uint8_t  evt_count;
    bool     b_steer_valid;
    uint32_t steer_word;
    bool     b_call_stop;
    bool     b_profile_changed;
} obstacle_statechart_out_t;

/* Statechart state. seq and steer_seq use 6 bits; counters is the packed
 * word described by OBSTACLE_CNT_* in obstacle_pack.h. */
typedef struct
{
    uint8_t          state;
    uint8_t          cause;
    uint8_t          faults;
    uint8_t          req_id;
    uint8_t          last_class;
    uint16_t         last_dist;
    uint8_t          seq;
    uint8_t          steer;
    uint8_t          steer_seq;
    uint16_t         counters;
    bool             b_reset_ok;
    uint32_t         since_ms;
    uint32_t         bypass_mm;
    uint32_t         scan_progress_ms;
    uint32_t         hug_limit_mm;
    scan_algo_ctx_t  scan;
    obstacle_plan_result_t plan;
    obstacle_plan_cfg_t    cfg;
} obstacle_statechart_t;

void obstacle_statechart_init (obstacle_statechart_t * p_fsm, obstacle_plan_cfg_t const * p_cfg,
                  uint32_t now_ms);
void obstacle_statechart_init_failed (obstacle_statechart_t * p_fsm);
void obstacle_statechart_out_clear (obstacle_statechart_out_t * p_out);

void obstacle_statechart_command (obstacle_statechart_t * p_fsm, uint8_t cmd, uint32_t now_ms,
                     uint32_t odo_mm, obstacle_statechart_out_t * p_out);
void obstacle_statechart_ping (obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                  uint32_t now_ms, obstacle_statechart_out_t * p_out);
void obstacle_statechart_tick (obstacle_statechart_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                  obstacle_statechart_out_t * p_out);

int16_t  obstacle_statechart_servo_deci (obstacle_statechart_t const * p_fsm);
uint32_t obstacle_statechart_status (obstacle_statechart_t const * p_fsm);

#endif /* OBSTACLE_STATECHART_H */

/*** end of file ***/

