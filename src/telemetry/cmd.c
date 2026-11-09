#include "telemetry/cmd.h"

static uint16_t
read_u16 (const uint8_t * src)
{
    return (uint16_t) (((uint16_t) src[0] << 8U) | (uint16_t) src[1]);
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
    if (src[7] != 0u)
    {
        return TELEMETRY_CMD_BAD_RESERVED;
    }

    cmd->cmd_id   = read_u16(&src[0]);
    cmd->run_id   = read_u16(&src[2]);
    cmd->sent_ms  = read_u16(&src[4]);
    cmd->opcode   = src[6];

    if (cmd->opcode > (uint8_t) TELEMETRY_CMD_ESTOP)
    {
        return TELEMETRY_CMD_BAD_OPCODE;
    }

    return TELEMETRY_CMD_OK;
}

telemetry_cmd_result_t
telemetry_cmd_accept (telemetry_cmd_state_t * state, const telemetry_cmd_t * cmd,
                      uint16_t now_ms, telemetry_cmd_dispatch_fn dispatch,
                      void * context)
{
    uint16_t age;
    int16_t  delta;

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
