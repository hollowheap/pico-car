/*
 *	command.h
 *	Vehicle command opcodes and sources.
 *
 *	The opcode values are the wire opcodes of protocol/telemetry.yaml, so a
 *	command from the dashboard needs no translation.
 */

#ifndef SUPERVISOR_COMMAND_H
#define SUPERVISOR_COMMAND_H

#include <stdbool.h>
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
#define VEHICLE_CMD_RECAL    (8U) /* calibrate the IMU again at rest */

#define VEHICLE_CMD_START_CONTINUOUS_FLOOR (9U)
/* 10U is reserved; never reuse for a new command. */

static inline bool vehicle_is_start (uint8_t opcode)
{
    return (VEHICLE_CMD_START == opcode) || (VEHICLE_CMD_START_CONTINUOUS_FLOOR == opcode);
}

static inline bool vehicle_is_junction_choice (uint8_t opcode)
{
    return (VEHICLE_CMD_LEFT == opcode) || (VEHICLE_CMD_RIGHT == opcode) ||
           (VEHICLE_CMD_STRAIGHT == opcode) || (VEHICLE_CMD_UTURN == opcode);
}

/* Who asked. */
#define VEHICLE_SRC_TELEMETRY (1U)
#define VEHICLE_SRC_BARCODE   (2U)
#define VEHICLE_SRC_BUTTON    (3U)

/* The ids name a telemetry command in its events; both are 0 for a barcode
   or the button. */
typedef struct
{
    uint16_t cmd_id; /* the cmd_id of the command frame */
    uint16_t run_id; /* the run ID it carried */
    uint8_t  opcode;
    uint8_t  source;
} vehicle_cmd_t;

STATIC_ASSERT (sizeof(vehicle_cmd_t) == 6U, "vehicle_cmd_t size");

#endif /* SUPERVISOR_COMMAND_H */
