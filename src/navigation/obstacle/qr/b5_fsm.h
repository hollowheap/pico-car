/** @file b5_fsm.h
 *
 * @brief Buddy 5 statechart (design review section 6.2). Pure C99 with no
 *        kernel calls: the task feeds it commands, pings and time, and
 *        carries out what it asks for (events, steering, emergency stop).
 *        This split keeps every decision in plain, kernel-free C.
 */

#ifndef B5_FSM_H
#define B5_FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "b5_plan.h"
#include "scan_algo.h"

/* At most two events come out of one step. */
#define B5_FSM_OUT_MAX_EVT (2U)

/* What the statechart asks the task to do after one step. */
typedef struct
{
    uint32_t evt[B5_FSM_OUT_MAX_EVT];
    uint8_t  evt_count;
    bool     b_steer_valid;
    uint32_t steer_word;
    bool     b_call_stop;
    bool     b_profile_changed;
} b5_fsm_out_t;

/* Statechart state. seq and steer_seq use 6 bits; counters is the packed
 * word described by B5_CNT_* in b5_pack.h. */
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
    scan_algo_ctx_t  scan;
    b5_plan_result_t plan;
    b5_plan_cfg_t    cfg;
} b5_fsm_t;

void b5_fsm_init (b5_fsm_t * p_fsm, b5_plan_cfg_t const * p_cfg,
                  uint32_t now_ms);
void b5_fsm_init_failed (b5_fsm_t * p_fsm);
void b5_fsm_out_clear (b5_fsm_out_t * p_out);

void b5_fsm_command (b5_fsm_t * p_fsm, uint8_t cmd, uint32_t now_ms,
                     uint32_t odo_mm, b5_fsm_out_t * p_out);
void b5_fsm_ping (b5_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                  uint32_t now_ms, b5_fsm_out_t * p_out);
void b5_fsm_tick (b5_fsm_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                  b5_fsm_out_t * p_out);

int16_t  b5_fsm_servo_deci (b5_fsm_t const * p_fsm, int16_t look_deci);
uint32_t b5_fsm_status (b5_fsm_t const * p_fsm);

#endif /* B5_FSM_H */

/*** end of file ***/
