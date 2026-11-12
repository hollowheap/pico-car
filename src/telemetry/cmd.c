#include "telemetry/cmd.h"

#define CMD_BYTE_BITS     8U
#define CMD_OFFSET_CMD_ID 0U
#define CMD_OFFSET_RUN_ID 2U
#define CMD_OFFSET_SENT   4U
#define CMD_OFFSET_OPCODE 6U
#define CMD_OFFSET_RSVD   7U

static uint16_t
read_u16 (const uint8_t * src)
{
    return (uint16_t) (((uint32_t) src[0] << CMD_BYTE_BITS) |
                       (uint32_t) src[1]);
}

void
telemetry_cmd_state_init (telemetry_cmd_state_t * state,
                          uint16_t expected_run_id)
{
    if (state == 0)
    {
        return;
    }
    state->last_cmd_id     = 0u;
    state->expected_run_id = expected_run_id;
    state->has_last        = false;
}

telemetry_cmd_result_t
telemetry_cmd_parse (const uint8_t * src, uint16_t len, telemetry_cmd_t * cmd)
{
    if ((src == 0) || (cmd == 0))
    {
        return TELEMETRY_CMD_BAD_ARG;
    }
    if (len != TELEMETRY_CMD_WIRE_SIZE)
    {
        return TELEMETRY_CMD_BAD_LENGTH;
    }
    if (src[CMD_OFFSET_RSVD] != 0u)
    {
        return TELEMETRY_CMD_BAD_RESERVED;
    }

    cmd->cmd_id   = read_u16(&src[CMD_OFFSET_CMD_ID]);
    cmd->run_id   = read_u16(&src[CMD_OFFSET_RUN_ID]);
    cmd->sent_ms  = read_u16(&src[CMD_OFFSET_SENT]);
    cmd->opcode   = src[CMD_OFFSET_OPCODE];

    if (cmd->opcode > (uint8_t) TELEMETRY_CMD_STRAIGHT)
    {
        return TELEMETRY_CMD_BAD_OPCODE;
    }

    return TELEMETRY_CMD_OK;
}

telemetry_cmd_result_t
telemetry_cmd_accept (telemetry_cmd_state_t * state, const telemetry_cmd_t * cmd,
                      uint16_t now_ms,
                      telemetry_cmd_dispatch_fn_t dispatch, void * context)
{
    uint16_t age   = 0u;
    int16_t  delta = 0;

    if ((state == 0) || (cmd == 0))
    {
        return TELEMETRY_CMD_BAD_ARG;
    }
    if ((state->expected_run_id != 0u) &&
        (cmd->run_id != state->expected_run_id))
    {
        return TELEMETRY_CMD_RUN_MISMATCH;
    }

    age = (uint16_t) (now_ms - cmd->sent_ms);
    if (age > TELEMETRY_CMD_MAX_AGE_MS)
    {
        return TELEMETRY_CMD_STALE;
    }

    if (state->has_last)
    {
        delta = (int16_t) (cmd->cmd_id - state->last_cmd_id);
        if (delta <= 0)
        {
            return TELEMETRY_CMD_DUPLICATE;
        }
    }

    state->last_cmd_id = cmd->cmd_id;
    state->has_last    = true;
    if (dispatch != 0)
    {
        dispatch(cmd, context);
    }

    return TELEMETRY_CMD_OK;
}
