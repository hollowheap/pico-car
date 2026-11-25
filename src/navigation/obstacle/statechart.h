/** @file statechart.h
 *
 * @brief Obstacle statechart. Pure C99 with no kernel calls and
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

/* Task-private ping counters. A run counts the pings in a row that
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
    uint8_t                reason; /* FOLLOW_REASON_* for BYPASS_GAVE_UP */
    uint8_t                seq;
    uint16_t               faults;
    uint8_t                req_id;
    uint8_t                last_class;
    uint16_t               last_dist;
    uint16_t               echo_faults; /* Current qualified echo faults. */
    uint32_t               ping;      /* last face ping: obstacle_pack_ping() */
    int16_t                look_deci; /* servo angle of the next ping        */
    uint8_t                steer;
    uint8_t                steer_seq;
    obstacle_counters_t    counts;
    bool                   b_reset_ok;
    bool                   b_turned;
    uint32_t               since_ms;
    uint32_t               bypass_mm;
    uint32_t               scan_progress_ms;
    uint32_t               hug_limit_mm;
    uint32_t               odo_mm;       /* net forward travel at the latest ping (P2)  */
    uint32_t               anchor_odo_mm; /* net travel at the last VALID reading (P2)   */
    uint16_t               anchor_dist;  /* the last VALID reading under 600 mm (P2)    */
    bool                   b_anchor;
    bool                   b_odo_fresh;  /* odo_mm comes from fresh feedback (P2)  */
    bool                   b_inconsistent; /* P2 */
    uint8_t                boot_pings;   /* P3: pings seen since boot, saturates        */
    uint8_t                boot_hits;
    bool                   b_beam_blocked; /* P3 */
    int16_t                escape_deci;  /* BYPASS turn target, from the line heading */
    uint32_t               escape_ms;    /* time of the last escape step             */
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
void obstacle_statechart_set_odometer (obstacle_statechart_t * p_fsm, uint32_t odo_mm,
                                       bool b_fresh);
/* Tell the statechart the servo angle of the next ping. */
void obstacle_statechart_set_look (obstacle_statechart_t * p_fsm, int16_t look_deci);
void obstacle_statechart_tick (obstacle_statechart_t * p_fsm, uint32_t now_ms,
                               uint32_t odo_mm,
                               obstacle_statechart_out_t * p_out);

int16_t  obstacle_statechart_servo_deci (obstacle_statechart_t const * p_fsm);
int16_t  obstacle_statechart_body_deci (obstacle_statechart_t const * p_fsm);

/* A ping taken while the body turns: only the health and collision checks use
 * it. It changes no reading in the status or the face ping word, so the face
 * follow never takes it for a ping at its look angle. */
void obstacle_statechart_ping_guard (obstacle_statechart_t * p_fsm, uint8_t cls,
                                     uint16_t dist_mm, uint32_t now_ms,
                                     obstacle_statechart_out_t * p_out);
/* A ping that feeds only the echo health checks (stuck echo, dead sonar,
 * noise). It never reaches the collision latch or any state action. It
 * updates the reading in the status; only a ping straight ahead (b_ahead)
 * goes into the forward ping word, because a requested ping has its own
 * reply word. An echo fault stops the car once and ends when it no longer
 * holds; the operator stop stays latched. */
void obstacle_statechart_ping_health (obstacle_statechart_t * p_fsm, uint8_t cls,
                                      uint16_t dist_mm, bool b_ahead, uint32_t now_ms,
                                      obstacle_statechart_out_t * p_out);
uint32_t obstacle_statechart_status (obstacle_statechart_t const * p_fsm);
uint8_t  obstacle_statechart_flags (obstacle_statechart_t const * p_fsm);

#endif /* OBSTACLE_STATECHART_H */

/*** end of file ***/
