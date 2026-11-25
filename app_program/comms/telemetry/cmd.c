#include "comms/telemetry/cmd.h"

#define CMD_BYTE_BITS     8U
#define CMD_U16_BITS      16U
#define CMD_U16_BYTES     2U
#define CMD_OFFSET_CMD_ID 0U
#define CMD_OFFSET_RUN_ID 2U
#define CMD_OFFSET_OPCODE 4U
#define CMD_OFFSET_UPTIME 5U
/* A cmd_id this far ahead of the last one or more reads as behind it. */
#define CMD_ID_HALF       0x8000U

static uint16_t
read_u16 (const uint8_t * src)
{
    return (uint16_t) (((uint32_t) src[0] << CMD_BYTE_BITS) |
                       (uint32_t) src[1]);
}

static uint32_t
read_u32 (const uint8_t * src)
{
    return ((uint32_t) read_u16(&src[0]) << CMD_U16_BITS) |
           (uint32_t) read_u16(&src[CMD_U16_BYTES]);
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
    state->expected_run_id = (expected_run_id != 0u) ? expected_run_id : 1u;
    state->has_last        = false;
    state->rejects.length  = 0u;
    state->rejects.opcode  = 0u;
    state->rejects.run     = 0u;
    state->rejects.old_id  = 0u;
    state->rejects.age     = 0u;
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
    cmd->cmd_id   = read_u16(&src[CMD_OFFSET_CMD_ID]);
    cmd->run_id   = read_u16(&src[CMD_OFFSET_RUN_ID]);
    cmd->opcode   = src[CMD_OFFSET_OPCODE];
    cmd->uptime_s = read_u32(&src[CMD_OFFSET_UPTIME]);

    if (cmd->opcode > (uint8_t) TELEMETRY_CMD_START_CONTINUOUS_FLOOR)
    {
        return TELEMETRY_CMD_BAD_OPCODE;
    }

    return TELEMETRY_CMD_OK;
}

telemetry_cmd_result_t
telemetry_cmd_accept (telemetry_cmd_state_t * state, const telemetry_cmd_t * cmd,
                      uint32_t now_s, telemetry_cmd_dispatch_fn_t dispatch,
                      void * context)
{
    uint16_t ahead = 0u;

    if ((state == 0) || (cmd == 0))
    {
        return TELEMETRY_CMD_BAD_ARG;
    }
    if (cmd->run_id != state->expected_run_id)
    {
        state->rejects.run++;
        return TELEMETRY_CMD_RUN_MISMATCH;
    }
    /* Same as (int16_t) (cmd_id - last) > 0, without a signed conversion. */
    ahead = (uint16_t) (cmd->cmd_id - state->last_cmd_id);
    if (state->has_last && ((ahead == 0u) || (ahead >= CMD_ID_HALF)))
    {
        state->rejects.old_id++;
        return TELEMETRY_CMD_OLD_ID;
    }
    if ((cmd->opcode != TELEMETRY_CMD_ESTOP) &&
        ((uint32_t)(now_s - cmd->uptime_s) > TELEMETRY_CMD_MAX_AGE_S))
    {
        state->rejects.age++;
        return TELEMETRY_CMD_BAD_AGE;
    }

    state->last_cmd_id = cmd->cmd_id;
    state->has_last    = true;
    if (dispatch != 0)
    {
        dispatch(cmd, context);
    }

    return TELEMETRY_CMD_OK;
}

telemetry_cmd_result_t
telemetry_cmd_receive (telemetry_cmd_state_t * state, const uint8_t * src,
                       uint16_t len, uint32_t now_s,
                       telemetry_cmd_dispatch_fn_t dispatch, void * context)
{
    telemetry_cmd_t        cmd;
    telemetry_cmd_result_t result = TELEMETRY_CMD_BAD_ARG;

    if (state == 0)
    {
        return TELEMETRY_CMD_BAD_ARG;
    }
    result = telemetry_cmd_parse(src, len, &cmd);
    if (result == TELEMETRY_CMD_BAD_OPCODE)
    {
        state->rejects.opcode++;
    }
    else if (result != TELEMETRY_CMD_OK)
    {
        state->rejects.length++;
    }
    else
    {
        result = telemetry_cmd_accept(state, &cmd, now_s, dispatch, context);
    }

    return result;
}
