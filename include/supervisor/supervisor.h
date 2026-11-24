/*
 *	supervisor.h
 *	Supervisor public interface.
 *
 *	One task owns the run state and watches the other tasks. It has two roles.
 *	Mission control drains the command ring and decides which command is
 *	allowed. The car starts only on an operator START, except that a pilot
 *	stop for a short sonar or stale-feedback fault resumes by itself (see
 *	PILOT_SONAR_RESUME_MS). Health collects the liveness bits, checks
 *	that the car moves, renews the run permit and feeds the hardware
 *	watchdog. It never drives a wheel: the line task drives, and does
 *	what the supervisor allows.
 */

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "platform/static_assert.h"

#include "supervisor/command.h"
#include "supervisor/health_ids.h"

/* supervisor_poll_turn returns this when no turn is pending. */
#define SUPERVISOR_NO_TURN (0xFFU)

/* Event parameter p1 of a task_stall event for a check that is not a task. */
#define SUPERVISOR_STALL_PROGRESS (0x80U) /* the car is told to move and does not */
#define SUPERVISOR_STALL_SENSOR   (0x81U) /* the line sensors never change         */
#define SUPERVISOR_STALL_ENCODERS (0x82U) /* the wheels answer crosswise */

/* The pass period of the task. The composition root wakes the task with a
   cyclic handler of this period. */
#define SUPERVISOR_PASS_MS (20U)

/** Watchdog reboots of one armed run that start it again by themselves; the
 *  next one waits for START. A guess until measured on the car. */
#define SUPERVISOR_WATCHDOG_RESUMES (2U)

typedef struct
{
    /* Number of line sensor changes since boot. May be NULL. */
    uint32_t (*p_line_edges) (void);
    /* Whether telemetry has had a broker session since boot. NULL disables offline start. */
    bool (*p_telemetry_seen) (void);
    /* Read the physical START button in task context. NULL disables it. */
    bool (*p_start_pressed) (void);
    /* Tasks that must kick their liveness bit (bits of LIVE_*). */
    uint16_t required_mask;
} supervisor_config_t;

/* Three function pointers and a mask, padded to a word. */
STATIC_ASSERT (sizeof(supervisor_config_t) == (3U * sizeof(void *)) + 4U,
               "supervisor_config_t size");

/* Prepare the supervisor. Call once from the startup task, before the task
   runs. */
ER supervisor_init (const supervisor_config_t * p_config);

/* Task body. Never returns. The composition root creates the task and the
   cyclic handler that wakes it. */
void supervisor_run (void);

/* Post a command from any task. Never blocks. One non-barcode E-STOP can wait
   separately from the ring and is handled first. Returns E_OK when stored or
   E_TMOUT if the command was dropped. */
ER vehicle_command_post (uint8_t opcode, uint8_t source);

/* The same with the ids of a telemetry command frame, so its cmd_accept or
   cmd_reject event names the command. vehicle_command_post passes 0 for both. */
ER vehicle_command_post_ids (uint8_t opcode, uint8_t source, uint16_t cmd_id,
                             uint16_t run_id);

/* Commands the inbox dropped since boot. The count wraps. */
uint16_t supervisor_cmd_drops (void);

/* Poll an admitted maneuver and its VEHICLE_SRC_*. Barcode L/R/S remain
   pending until selected or canceled. A one-shot maneuver is claimed here. */
uint8_t supervisor_poll_turn (uint8_t * p_source);
/* A consuming check removes only the matching barcode junction choice. */
bool supervisor_route_pending (uint8_t expected_opcode, bool b_consume);

/* Accepted RESET commands since boot, wrapping at 8 bits. The line task
   compares it with the last value it saw and clears the pilot's latches when it
   differs. A RESET of a running car also sets the operator hold (as a STOP
   does) before the count moves. A RESET after a pickup keeps the fault and
   the stop active until START. */
uint8_t supervisor_reset_count (void);

/* Accepted START commands since boot, wrapping at 8 bits. The line task
   clears recoverable pilot latches and rechecks a blocked course when it changes. */
uint8_t supervisor_start_count (void);
/* Copy the start generation and its active floor selection under one lock. */
uint8_t supervisor_start_snapshot (bool * p_continuous_floor);
/* Observe the accepted START generation and whether the run is armed. */
uint8_t supervisor_armed_start (bool * p_armed);
/* Stop the still-armed generation if the startup motion check fails. */
void supervisor_start_check_failed (uint8_t generation);
/* Completed in-run attitude recoveries. The accompanying flag says the fresh
 * forward observation cleared the path. A near return needs a new survey. */
uint8_t supervisor_recovery_snapshot (bool * p_clear);

/* A wheel told to go forward runs well under its target with much more duty
   than its grounded baseline needs: the car pushes against something. */
bool supervisor_wheels_loaded (void);

/* The line task tells the supervisor while a one-shot turn runs. */
void supervisor_set_turning (bool b_turning);

/* The line task could not run a turn the supervisor allowed (the obstacle task
   took the motors first). Sends the cmd_reject event. */
void supervisor_turn_refused (uint8_t opcode);

/* Tasks that are alive: they kicked at least once and are not stalled. One bit per
   LIVE_* number. */
uint16_t supervisor_alive_mask (void);

/* Passes since boot, wraps at 16 bits. A frozen value shows a hung supervisor. */
uint16_t supervisor_pass_count (void);

#endif /* SUPERVISOR_SUPERVISOR_H */
