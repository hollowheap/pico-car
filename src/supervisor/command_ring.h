/*
 *	command_ring.h
 *	Inbox for vehicle commands (D120). Pure C: no kernel call, no lock.
 *
 *	Any task posts a command with its source. The supervisor is the only
 *	reader. The caller (supervisor.c) wraps push and pop in a spinlock. The
 *	ring only stores; it never decides.
 */

#ifndef SUPERVISOR_COMMAND_RING_H
#define SUPERVISOR_COMMAND_RING_H

#include <stdbool.h>
#include <stdint.h>

#include "supervisor/command.h"

#define COMMAND_RING_LEN (8U)

typedef struct
{
    vehicle_cmd_t item[COMMAND_RING_LEN];
    uint8_t       head;
    uint8_t       count;
    uint16_t      dropped;
} command_ring_t;

void command_ring_init (command_ring_t * p_ring);

/* Stores one command. A full ring drops the oldest command that is not a
   stop, so a stop is never lost. When every stored command is a stop, the new
   command is dropped. Each drop is counted. Returns false if the new command
   was not stored. */
bool command_ring_push (command_ring_t * p_ring, uint8_t opcode, uint8_t source);

/* Takes the oldest command. Returns false when the ring is empty. */
bool command_ring_pop (command_ring_t * p_ring, vehicle_cmd_t * p_cmd);

#endif /* SUPERVISOR_COMMAND_RING_H */
