/*
 *	supervisor.h
 *	Supervisor public interface (D120, D160).
 *
 *	One task owns the run state and watches the other tasks. It has two roles.
 *	Mission control drains the command ring and decides which command is
 *	allowed. Health collects the liveness bits, checks that the car moves,
 *	renews the run permit and feeds the hardware watchdog. It never drives a
 *	wheel: the line task drives, and does what the supervisor allows.
 */

#ifndef SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "platform/static_assert.h"

#include "supervisor/command.h"
#include "supervisor/health_ids.h"

/* supervisor_take_turn returns this when no turn is pending. */
#define SUPERVISOR_NO_TURN (0xFFU)

/* Event parameter p1 of a task_stall event for a check that is not a task. */
#define SUPERVISOR_STALL_PROGRESS (0x80U) /* the car is told to move and does not */
#define SUPERVISOR_STALL_SENSOR   (0x81U) /* the line sensors never change         */
#define SUPERVISOR_STALL_ENCODERS (0x82U) /* the wheels answer crosswise (D161)    */

/* The pass period of the task. The composition root wakes the task with a
   cyclic handler of this period. */
#define SUPERVISOR_PASS_MS (20U)

typedef struct
{
    /* Called about once a second from the supervisor task, for work that needs
       data only the composition root has (stack marks). May be NULL. */
    void (*p_background) (void);
    /* Number of line sensor changes since boot. May be NULL. */
    uint32_t (*p_line_edges) (void);
    /* Tasks that must kick their liveness bit (bits of LIVE_*). */
    uint16_t required_mask;
} supervisor_config_t;

/* Two function pointers and a mask, padded to a word. */
STATIC_ASSERT (sizeof(supervisor_config_t) == (2U * sizeof(void *)) + 4U,
               "supervisor_config_t size");

/* Prepare the supervisor. Call once from the startup task, before the task
   runs. */
ER supervisor_init (const supervisor_config_t * p_config);

/* Task body. Never returns. The composition root creates the task and the
   cyclic handler that wakes it. */
void supervisor_run (void);

/* Post a command from any task. Never blocks. Returns E_OK, or E_TMOUT when the
   ring is full and the command was dropped. */
ER vehicle_command_post (uint8_t opcode, uint8_t source);

/* The line task takes a turn the supervisor allowed: VEHICLE_CMD_LEFT, _RIGHT or
   _UTURN, _STRAIGHT, or SUPERVISOR_NO_TURN. A turn is returned once. *p_source gets the
   VEHICLE_SRC_* of the command that asked for it. */
uint8_t supervisor_take_turn (uint8_t * p_source);

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
