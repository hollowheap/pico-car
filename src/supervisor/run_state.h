/*
 *	run_state.h
 *	Run state of the vehicle and the rules for each command. Pure C:
 *	no kernel call, so a host test drives it.
 *
 *	The state is a few flags, and the visible state is derived from them. A
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
    RUN_STATE_STOPPED, /* boot, or an operator STOP, holds the car            */
    RUN_STATE_HELD,    /* a barcode stop or a lift holds the car            */
    RUN_STATE_LATCHED  /* emergency stop, or overturn awaiting recovery */
} run_state_t;

/* Why a command was refused. */
#define RUN_REJECT_NONE        (0U)
#define RUN_REJECT_NOT_RUNNING (1U)
#define RUN_REJECT_BUSY        (2U)
#define RUN_REJECT_LATCHED     (3U)
#define RUN_REJECT_LIFTED      (4U)
#define RUN_REJECT_SOURCE      (5U)
#define RUN_REJECT_OPCODE      (6U)
#define RUN_REJECT_ON_HUMP     (7U) /* a calibration on a ramp would be wrong */
#define RUN_REJECT_STALLED     (8U) /* the motion permit lapsed */
#define RUN_REJECT_NO_ATTITUDE (9U) /* the lean is not known yet */
#define RUN_REJECT_HARDWARE_FAULT (10U)

/* The tick runs once per supervisor pass (SUPERVISOR_PASS_MS). */
#define RUN_PASS_MS (20U)

/* Obstacle posts the full obstacle command queue refused. They are owed and
   posted again on the next pass, RESET before START. */
#define RUN_OWE_RESET (1U)
#define RUN_OWE_START (2U)

/* The turn the line task must run, or none. */
#define RUN_TURN_NONE (0xFFU)

typedef struct
{
    bool b_booted;
    bool b_operator; /* boot, or STOP from the dashboard or a button   */
    bool b_barcode;  /* STOP from a barcode                            */
    bool b_lift;     /* lifted or overturned: the lift hold is on      */
    bool b_overturn; /* overturn awaiting operator recovery */
    bool b_righted_overturn; /* previous attitude latch awaiting physics recalibration */
    bool b_continuous_floor; /* floor selected by the last accepted START */
    bool b_estop;    /* the motion emergency latch                     */
    uint8_t start_count; /* accepted STARTs; wraps */
    uint8_t reset_count; /* accepted RESETs; wraps */
} run_ctx_t;

/* Facts the supervisor reads before each step. */
typedef struct
{
    bool b_lifted;
    bool b_overturned;
    bool b_estop;
    bool b_follow; /* the line task follows the line, the obstacle task monitors, no turn is pending */
    bool b_on_hump; /* the hump detector is in a hump: the car is not level */
    bool b_upright; /* calibrated, level and settled in the IMU recovery window */
    bool b_permit_lapsed; /* the motion permit lapsed: a stall or a long gap */
    bool b_attitude_unknown; /* the lean is not known: no START or RECAL */
    bool b_collecting_rest;  /* fresh qualifying calibration or level/still sample */
    uint8_t rest_interruptions;
    bool b_connection_fault; /* a current IMU or sonar fault: no START */
    bool b_calibration_timed_out; /* a calibration hold still waits for a reference */
} run_inputs_t;

/* The physics and motion facts the supervisor maps to run_inputs_t. */
typedef struct
{
    bool    b_ready;
    bool    b_calibrated;
    bool    b_attitude_unknown;
    bool    b_hump_active;
    uint8_t hump_state;     /* TELEMETRY_HUMP_STATE_* */
    uint8_t attitude_state; /* TELEMETRY_ATTITUDE_STATE_* */
    bool    b_level_still;
} run_physics_t;

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
    bool     b_physics_recal; /* the IMU must calibrate again: after an overturn, on START, on RECAL */
    bool     b_start;         /* START accepted: ends a latched sonar stop */
    bool     b_sticky_clear;  /* START or RESET: acknowledge recovered faults */
} run_result_t;

void        run_state_init (run_ctx_t * p_ctx);
run_state_t run_state_of (const run_ctx_t * p_ctx);
uint32_t    run_state_hold_mask (const run_ctx_t * p_ctx);

/* Apply one command. */
void run_state_command (run_ctx_t * p_ctx, vehicle_cmd_t cmd,
                        const run_inputs_t * p_in, run_result_t * p_out);

/* Set the inputs that come from physics. The attitude fields are read only
   while the lean is known. */
void run_state_read_physics (run_inputs_t * p_in, const run_physics_t * p_phys);

/* True when the motion permit lapsed since the last renew: the supervisor
   skipped it, or the gap since it is over the lease. */
bool run_permit_lapsed (bool b_skipped, uint32_t gap_ms, uint32_t lease_ms);

/* True when a pass does not renew the motion permit: a critical task stalled,
   or the motion service refused the renewal. */
bool run_permit_skipped (bool b_critical, bool b_renew_failed);

/* Apply the facts that change the state by themselves: boot done, the
   emergency latch, a lift, an overturn, a stalled task. */
void run_state_tick (run_ctx_t * p_ctx, const run_inputs_t * p_in,
                     run_result_t * p_out);

/* Post the RESET and START a step asks for, after those still owed (RUN_OWE_*
   bits). p_refused posts one OBSTACLE_CMD_* and returns true when the full
   queue refused it; that post and the ones after it stay owed. A stop drops
   what is owed, so a START that never arrived cannot run after it; the caller
   posts the STOP, which the obstacle task never refuses. Returns what is
   still owed. */
uint8_t run_obstacle_post (uint8_t owed, const run_result_t * p_res,
                           bool (*p_refused) (uint8_t cmd));

#endif /* SUPERVISOR_RUN_STATE_H */
