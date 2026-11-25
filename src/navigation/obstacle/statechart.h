/** @file statechart.h
 *
 * @brief Obstacle statechart (S40, D96). Pure C99 with no kernel calls and
 *        no libm: the task feeds it commands, pings and time, and carries
 *        out what it asks for (events, steering, emergency stop).
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

/* Task-private ping counters (D97). A run counts the pings in a row that
 * match, and a history keeps one bit per ping with the newest in bit 0. */
typedef struct
{
    uint8_t no_echo;   /* NO_ECHO pings in a row                              */
    uint8_t stuck;     /* STUCK pings in a row                                */
    uint8_t glitch;    /* GLITCH pings in a row                               */
    uint8_t fail;      /* NO_ECHO, GLITCH or STUCK pings in a row             */
    uint8_t hit_hist;  /* last OBSTACLE_HIT_WINDOW pings: a hit to detect     */
    uint8_t coll_hist; /* last OBSTACLE_HIT_WINDOW pings: a collision range   */
    uint8_t clear;     /* clear pings in a row while the bypass goes round    */
} obstacle_counters_t;

/* Statechart state. seq uses 5 bits and steer_seq 6 bits. */
typedef struct
{
    uint8_t                state;
    uint8_t                cause;
    uint16_t               faults;
    uint8_t                req_id;
    uint8_t                last_class;
    uint16_t               last_dist;
    uint8_t                seq;
    uint8_t                steer;
    uint8_t                steer_seq;
    obstacle_counters_t    counts;
    bool                   b_reset_ok;
    bool                   b_turned;
    uint32_t               since_ms;
    uint32_t               bypass_mm;
    uint32_t               scan_progress_ms;
    uint32_t               hug_limit_mm;
    uint32_t               odo_mm;       /* wheel travel at the latest ping (P2)        */
    uint32_t               anchor_odo_mm; /* travel at the last VALID reading (P2)       */
    uint16_t               anchor_dist;  /* the last VALID reading under 600 mm (P2)    */
    bool                   b_anchor;
    bool                   b_inconsistent; /* P2 */
    uint8_t                boot_pings;   /* P3: pings seen since boot, saturates        */
    uint8_t                boot_hits;
    bool                   b_beam_blocked; /* P3 */
    scan_algo_ctx_t        scan;
    obstacle_plan_result_t plan;
    obstacle_plan_cfg_t    cfg;
} obstacle_statechart_t;

void obstacle_statechart_init (obstacle_statechart_t * p_fsm,
                               obstacle_plan_cfg_t const * p_cfg,
                               uint32_t now_ms);
void obstacle_statechart_init_failed (obstacle_statechart_t * p_fsm);
void obstacle_statechart_out_clear (obstacle_statechart_out_t * p_out);

void obstacle_statechart_command (obstacle_statechart_t * p_fsm, uint8_t cmd,
                                  uint32_t now_ms, uint32_t odo_mm,
                                  obstacle_statechart_out_t * p_out);
void obstacle_statechart_ping (obstacle_statechart_t * p_fsm, uint8_t cls,
                               uint16_t dist_mm, uint32_t now_ms,
                               obstacle_statechart_out_t * p_out);
/* Tell the statechart the wheel travel before a ping, for the P2 check. */
void obstacle_statechart_set_odometer (obstacle_statechart_t * p_fsm, uint32_t odo_mm);
void obstacle_statechart_tick (obstacle_statechart_t * p_fsm, uint32_t now_ms,
                               uint32_t odo_mm,
                               obstacle_statechart_out_t * p_out);

int16_t  obstacle_statechart_servo_deci (obstacle_statechart_t const * p_fsm);
int16_t  obstacle_statechart_body_deci (obstacle_statechart_t const * p_fsm);
uint32_t obstacle_statechart_status (obstacle_statechart_t const * p_fsm);
uint8_t  obstacle_statechart_flags (obstacle_statechart_t const * p_fsm);

#endif /* OBSTACLE_STATECHART_H */

/*** end of file ***/
