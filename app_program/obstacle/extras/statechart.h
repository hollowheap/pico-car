/** @file statechart.h
 *
 * @brief Buddy 5 statechart (design review section 6.2). Pure C99 with no
 *        kernel calls: the task feeds it commands, pings and time, and
 *        carries out what it asks for (events, steering, emergency stop).
 *        This split keeps every decision in plain, kernel-free C.
 */

#ifndef EXTRAS_FSM_H
#define EXTRAS_FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "planner.h"
#include "scan.h"

/* At most two events come out of one step. */
#define EXTRAS_FSM_OUT_MAX_EVT (2U)

/* What the statechart asks the task to do after one step. */
typedef struct
{
    uint32_t evt[EXTRAS_FSM_OUT_MAX_EVT];
    uint8_t  evt_count;
    bool     b_steer_valid;
    uint32_t steer_word;
    bool     b_call_stop;
    bool     b_profile_changed;
} extras_fsm_out_t;

/* Statechart state. seq and steer_seq use 6 bits; counters is the packed
 * word described by EXTRAS_CNT_* in extras_pack.h. */
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
    bool             b_guard_off;
    uint32_t         since_ms;
    uint32_t         bypass_mm;
    extras_scan_ctx_t  scan;
    extras_plan_result_t plan;
    extras_plan_cfg_t    cfg;
} extras_fsm_t;

void extras_fsm_init (extras_fsm_t * p_fsm, extras_plan_cfg_t const * p_cfg,
                  uint32_t now_ms);
void extras_fsm_init_failed (extras_fsm_t * p_fsm);
void extras_fsm_out_clear (extras_fsm_out_t * p_out);

void extras_fsm_command (extras_fsm_t * p_fsm, uint8_t cmd, uint32_t now_ms,
                     uint32_t odo_mm, extras_fsm_out_t * p_out);
void extras_fsm_ping (extras_fsm_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                  uint32_t now_ms, extras_fsm_out_t * p_out);
void extras_fsm_tick (extras_fsm_t * p_fsm, uint32_t now_ms, uint32_t odo_mm,
                  extras_fsm_out_t * p_out);

int16_t  extras_fsm_servo_deci (extras_fsm_t const * p_fsm, int16_t look_deci);
uint32_t extras_fsm_status (extras_fsm_t const * p_fsm);
void     extras_fsm_set_guard_off (extras_fsm_t * p_fsm, bool b_off);

#endif /* EXTRAS_FSM_H */

/*** end of file ***/
