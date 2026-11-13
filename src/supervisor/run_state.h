/*
 *	run_state.h
 *	Run state of the vehicle and the rules for each command (D120). Pure C:
 *	no kernel call, so a host test drives it.
 *
 *	The visible state is derived from the stored run flags. A
 *	command or a tick changes flags and the supervisor applies the result.
 *	The supervisor decides what is allowed. The line task does what is
 *	allowed.
 */

#ifndef SUPERVISOR_RUN_STATE_H
#define SUPERVISOR_RUN_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "supervisor/command.h"

typedef enum
{
    RUN_STATE_BOOT = 0,
    RUN_STATE_RUNNING,
    RUN_STATE_STOPPED, /* an operator or dashboard STOP holds the car       */
    RUN_STATE_HELD,    /* a barcode stop or a lift holds the car            */
    RUN_STATE_LATCHED  /* emergency stop or overturn: needs RESET or reboot */
} run_state_t;

/* Why a command was refused. */
#define RUN_REJECT_NONE        (0U)
#define RUN_REJECT_NOT_RUNNING (1U)
#define RUN_REJECT_BUSY        (2U)
#define RUN_REJECT_LATCHED     (3U)
#define RUN_REJECT_LIFTED      (4U)
#define RUN_REJECT_SOURCE      (5U)
#define RUN_REJECT_OPCODE      (6U)

/* The turn the line task must run, or none. */
#define RUN_TURN_NONE (0xFFU)

typedef struct
{
    bool b_booted;
    bool b_wait_button; /* wait for a physical START after boot */
    bool b_operator; /* STOP from the dashboard or a button           */
    bool b_barcode;  /* STOP from a barcode                            */
    bool b_lift;     /* lifted or overturned: the lift hold is on      */
    bool b_overturn; /* overturned: needs RESET after the car is upright */
    bool b_estop;    /* the motion emergency latch                     */
    bool b_motion_fault; /* controller fault remains latched until reboot */
} run_ctx_t;

/* Facts the supervisor reads before each step. */
typedef struct
{
    bool b_lifted;
    bool b_overturned;
    bool b_estop;
    bool b_motion_fault;
    bool b_follow; /* the line task follows the line, the obstacle task monitors, no turn is pending */
    bool b_obstacle_stopped; /* a new physical START can retry a stopped scan */
} run_inputs_t;

/* What the supervisor must do after a step. hold_mask is the full set of
   holds that must be on (MOTION_HOLD_*); the supervisor sets the changes. */
typedef struct
{
    bool     b_accepted;
    bool     b_rejected; /* a command was refused: reason says why */
    uint8_t  reason;
    uint8_t  turn;
    uint32_t hold_mask;
    bool     b_cancel_turn;
    bool     b_obstacle_stop;
    bool     b_obstacle_reset;
    bool     b_obstacle_start;
    bool     b_motion_estop;
    bool     b_physics_recal; /* the IMU must calibrate again after an overturn */
} run_result_t;

void        run_state_init (run_ctx_t * p_ctx, bool b_button_start);
run_state_t run_state_of (const run_ctx_t * p_ctx);
uint32_t    run_state_hold_mask (const run_ctx_t * p_ctx);

/* Apply one command. */
void run_state_command (run_ctx_t * p_ctx, vehicle_cmd_t cmd,
                        const run_inputs_t * p_in, run_result_t * p_out);

/* Apply the facts that change the state by themselves: boot done, the
   emergency latch, a lift, an overturn. */
void run_state_tick (run_ctx_t * p_ctx, const run_inputs_t * p_in,
                     run_result_t * p_out);

#endif /* SUPERVISOR_RUN_STATE_H */
