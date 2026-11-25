#ifndef TELEMETRY_CMD_H
#define TELEMETRY_CMD_H

#include <stdbool.h>
#include <stdint.h>

#define TELEMETRY_CMD_WIRE_SIZE   8u
#define TELEMETRY_CMD_MAX_AGE_MS  2000u

typedef enum
{
    TELEMETRY_CMD_STOP  = 0u,
    TELEMETRY_CMD_START = 1u,
    TELEMETRY_CMD_RESET = 2u
} telemetry_cmd_opcode_t;

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

typedef struct
{
    uint16_t cmd_id;
    uint16_t run_id;
    uint16_t sent_ms;
    uint8_t  opcode;
} telemetry_cmd_t;

typedef struct
{
    uint16_t last_cmd_id;
    uint16_t expected_run_id;
    bool     has_last;
} telemetry_cmd_state_t;

typedef void (*telemetry_cmd_dispatch_fn)(const telemetry_cmd_t * cmd,
                                           void * context);

void telemetry_cmd_state_init (telemetry_cmd_state_t * state,
                               uint16_t expected_run_id);

/* Wire layout: cmd_id u16, run_id u16, sent_ms u16, opcode u8, reserved u8. */
telemetry_cmd_result_t telemetry_cmd_parse (const uint8_t * src, uint16_t len,
                                            telemetry_cmd_t * cmd);

telemetry_cmd_result_t telemetry_cmd_accept (telemetry_cmd_state_t * state,
                                             const telemetry_cmd_t * cmd,
                                             uint16_t now_ms,
                                             telemetry_cmd_dispatch_fn dispatch,
                                             void * context);

#endif /* TELEMETRY_CMD_H */
