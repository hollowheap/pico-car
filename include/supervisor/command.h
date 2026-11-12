/*
 *	command.h
 *	Vehicle command opcodes and sources (D120).
 *
 *	The opcode values are the wire opcodes of protocol/telemetry.yaml, so a
 *	command from the dashboard needs no translation.
 */

#ifndef SUPERVISOR_COMMAND_H
#define SUPERVISOR_COMMAND_H

#include <stdint.h>

#include "platform/static_assert.h"

#define VEHICLE_CMD_STOP  (0U)
#define VEHICLE_CMD_START (1U)
#define VEHICLE_CMD_RESET (2U)
#define VEHICLE_CMD_LEFT  (3U)
#define VEHICLE_CMD_RIGHT (4U)
#define VEHICLE_CMD_UTURN (5U)
#define VEHICLE_CMD_ESTOP (6U)
#define VEHICLE_CMD_STRAIGHT (7U) /* cross a junction without turning */

/* Who asked. */
#define VEHICLE_SRC_TELEMETRY (1U)
#define VEHICLE_SRC_BARCODE   (2U)
#define VEHICLE_SRC_BUTTON    (3U)

typedef struct
{
    uint8_t opcode;
    uint8_t source;
} vehicle_cmd_t;

STATIC_ASSERT (sizeof(vehicle_cmd_t) == 2U, "vehicle_cmd_t size");

#endif /* SUPERVISOR_COMMAND_H */
