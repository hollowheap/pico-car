#ifndef TELEMETRY_CMD_H
#define TELEMETRY_CMD_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

#define TELEMETRY_CMD_WIRE_SIZE   9u

/* The oldest heartbeat uptime a command may echo, in seconds before the car
   uptime. Both count whole seconds, so a command is always accepted within
   3 s of the heartbeat it echoes and always refused 4 s or more after it. */
#define TELEMETRY_CMD_MAX_AGE_S   3u

typedef enum
{
    TELEMETRY_CMD_STOP  = 0u,
    TELEMETRY_CMD_START = 1u,
    TELEMETRY_CMD_RESET = 2u,
    TELEMETRY_CMD_LEFT  = 3u,
    TELEMETRY_CMD_RIGHT = 4u,
    TELEMETRY_CMD_UTURN = 5u,
    TELEMETRY_CMD_ESTOP = 6u,
    TELEMETRY_CMD_STRAIGHT = 7u,
    TELEMETRY_CMD_RECALIBRATE = 8u,
    TELEMETRY_CMD_START_CONTINUOUS_FLOOR = 9u,
    TELEMETRY_CMD_START_STATIONARY = 10u
} telemetry_cmd_opcode_t;

STATIC_ASSERT (sizeof(telemetry_cmd_opcode_t) <= 4U,
               "telemetry_cmd_opcode_t size");

typedef enum
{
    TELEMETRY_CMD_OK = 0,
    TELEMETRY_CMD_BAD_ARG,
    TELEMETRY_CMD_BAD_LENGTH,
    TELEMETRY_CMD_BAD_OPCODE,
    TELEMETRY_CMD_OLD_ID,
    TELEMETRY_CMD_RUN_MISMATCH,
    TELEMETRY_CMD_BAD_AGE
} telemetry_cmd_result_t;

STATIC_ASSERT (sizeof(telemetry_cmd_result_t) <= 4U,
               "telemetry_cmd_result_t size");

typedef struct
{
    uint32_t uptime_s; /* the heartbeat uptime the host echoes */
    uint16_t cmd_id;
    uint16_t run_id;
    uint8_t  opcode;
} telemetry_cmd_t;

STATIC_ASSERT (sizeof(telemetry_cmd_t) == 12U, "telemetry_cmd_t size");

/* Refused commands per cause. Each count wraps; the diag frame carries it. */
typedef struct
{
    uint8_t length; /* wrong length, or no payload */
    uint8_t opcode; /* unknown opcode */
    uint8_t run;    /* another run ID */
    uint8_t old_id; /* cmd_id not newer than the last accepted one */
    uint8_t age;    /* uptime echo too old, or ahead of the car */
} telemetry_cmd_rejects_t;

STATIC_ASSERT (sizeof(telemetry_cmd_rejects_t) == 5U,
               "telemetry_cmd_rejects_t size");

typedef struct
{
    uint16_t                last_cmd_id;
    uint16_t                expected_run_id;
    bool                    has_last;
    telemetry_cmd_rejects_t rejects;
} telemetry_cmd_state_t;

STATIC_ASSERT (sizeof(telemetry_cmd_state_t) == 10U,
               "telemetry_cmd_state_t size");

typedef void (*telemetry_cmd_dispatch_fn_t)(const telemetry_cmd_t * cmd,
                                             void * context);

/* expected_run_id is the run ID the heartbeat announces; 0 is replaced by
   1, so a host that never read a heartbeat (run ID 0) is always refused. */
void telemetry_cmd_state_init (telemetry_cmd_state_t * state,
                               uint16_t expected_run_id);

/* Wire layout, big-endian: cmd_id u16, run_id u16, opcode u8, uptime_s u32.
   Counts nothing: telemetry_cmd_receive counts its refusals. */
telemetry_cmd_result_t telemetry_cmd_parse (const uint8_t * src, uint16_t len,
                                            telemetry_cmd_t * cmd);

/* Accept a command of this run when its cmd_id is newer than the last
   accepted one, at most 32767 ahead in the 16-bit sequence, and its echoed
   uptime is 0 to TELEMETRY_CMD_MAX_AGE_S seconds behind now_s, the car
   uptime in the heartbeat's units. The first command after init may carry
   any cmd_id; a new boot brings a new run ID and a fresh state, so a host
   that restarts takes the next id after the diag cmd_last_id. A refused
   command is counted and leaves the last id as it was. */
telemetry_cmd_result_t telemetry_cmd_accept (telemetry_cmd_state_t * state,
                                             const telemetry_cmd_t * cmd,
                                             uint32_t now_s,
                                             telemetry_cmd_dispatch_fn_t
                                                 dispatch,
                                             void * context);

/* Parse one frame and accept it; a frame that does not parse is counted. */
telemetry_cmd_result_t telemetry_cmd_receive (telemetry_cmd_state_t * state,
                                              const uint8_t * src,
                                              uint16_t len, uint32_t now_s,
                                              telemetry_cmd_dispatch_fn_t
                                                  dispatch,
                                              void * context);

#endif /* TELEMETRY_CMD_H */
