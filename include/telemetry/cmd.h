#ifndef TELEMETRY_CMD_H
#define TELEMETRY_CMD_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

#define TELEMETRY_CMD_WIRE_SIZE   8u
#define TELEMETRY_CMD_MAX_AGE_MS  2000u

typedef enum
{
    TELEMETRY_CMD_STOP  = 0u,
    TELEMETRY_CMD_START = 1u,
    TELEMETRY_CMD_RESET = 2u,
    TELEMETRY_CMD_LEFT  = 3u,
    TELEMETRY_CMD_RIGHT = 4u,
    TELEMETRY_CMD_UTURN = 5u,
    TELEMETRY_CMD_ESTOP = 6u,
    TELEMETRY_CMD_STRAIGHT = 7u
} telemetry_cmd_opcode_t;

STATIC_ASSERT (sizeof(telemetry_cmd_opcode_t) <= 4U,
               "telemetry_cmd_opcode_t size");

typedef enum
{
    TELEMETRY_CMD_OK = 0,
    TELEMETRY_CMD_BAD_ARG,
    TELEMETRY_CMD_BAD_LENGTH,
    TELEMETRY_CMD_BAD_OPCODE,
    TELEMETRY_CMD_BAD_RESERVED,
    TELEMETRY_CMD_STALE,
    TELEMETRY_CMD_DUPLICATE,
    TELEMETRY_CMD_RUN_MISMATCH
} telemetry_cmd_result_t;

STATIC_ASSERT (sizeof(telemetry_cmd_result_t) <= 4U,
               "telemetry_cmd_result_t size");

typedef struct
{
    uint16_t cmd_id;
    uint16_t run_id;
    uint16_t sent_ms;
    uint8_t  opcode;
} telemetry_cmd_t;

STATIC_ASSERT (sizeof(telemetry_cmd_t) == 8U, "telemetry_cmd_t size");

typedef struct
{
    uint32_t last_cmd_ms;
    uint16_t last_cmd_id;
    uint16_t expected_run_id;
    bool     has_last;
} telemetry_cmd_state_t;

STATIC_ASSERT (sizeof(telemetry_cmd_state_t) == 12U,
               "telemetry_cmd_state_t size");

typedef void (*telemetry_cmd_dispatch_fn_t)(const telemetry_cmd_t * cmd,
                                             void * context);

void telemetry_cmd_state_init (telemetry_cmd_state_t * state,
                               uint16_t expected_run_id);

/* Wire layout: cmd_id u16, run_id u16, sent_ms u16, opcode u8, reserved u8. */
telemetry_cmd_result_t telemetry_cmd_parse (const uint8_t * src, uint16_t len,
                                            telemetry_cmd_t * cmd);

telemetry_cmd_result_t telemetry_cmd_accept (telemetry_cmd_state_t * state,
                                             const telemetry_cmd_t * cmd,
                                             uint32_t now_ms,
                                             telemetry_cmd_dispatch_fn_t
                                                 dispatch,
                                             void * context);

#endif /* TELEMETRY_CMD_H */
