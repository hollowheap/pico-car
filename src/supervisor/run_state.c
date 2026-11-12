/*
 *	run_state.c
 *	Run state of the vehicle. See run_state.h.
 */

#include "run_state.h"
#include "motion/motion.h"

static void clear_result (run_result_t * p_out, const run_ctx_t * p_ctx)
{
    p_out->b_accepted       = false;
    p_out->b_rejected       = false;
    p_out->reason           = RUN_REJECT_NONE;
    p_out->turn             = RUN_TURN_NONE;
    p_out->hold_mask        = run_state_hold_mask (p_ctx);
    p_out->b_cancel_turn    = false;
    p_out->b_obstacle_stop  = false;
    p_out->b_obstacle_reset = false;
    p_out->b_obstacle_start = false;
    p_out->b_motion_estop   = false;
    p_out->b_physics_recal  = false;
}

static void reject (run_result_t * p_out, uint8_t reason)
{
    p_out->b_rejected = true;
    p_out->reason     = reason;
}

void run_state_init (run_ctx_t * p_ctx)
{
    p_ctx->b_booted   = false;
    p_ctx->b_operator = false;
    p_ctx->b_barcode  = false;
    p_ctx->b_lift     = false;
    p_ctx->b_overturn = false;
    p_ctx->b_estop    = false;
}

run_state_t run_state_of (const run_ctx_t * p_ctx)
{
    run_state_t state = RUN_STATE_RUNNING;

    if (!p_ctx->b_booted)
    {
        state = RUN_STATE_BOOT;
    }
    else if (p_ctx->b_estop || p_ctx->b_overturn)
    {
        state = RUN_STATE_LATCHED;
    }
    else if (p_ctx->b_barcode || p_ctx->b_lift)
    {
        state = RUN_STATE_HELD;
    }
    else if (p_ctx->b_operator)
    {
        state = RUN_STATE_STOPPED;
    }
    else
    {
        state = RUN_STATE_RUNNING;
    }

    return state;
}

uint32_t run_state_hold_mask (const run_ctx_t * p_ctx)
{
    uint32_t mask = 0U;

    if (p_ctx->b_operator)
    {
        mask |= MOTION_HOLD_OPERATOR;
    }
    if (p_ctx->b_barcode)
    {
        mask |= MOTION_HOLD_BARCODE;
    }
    if (p_ctx->b_lift)
    {
        mask |= MOTION_HOLD_LIFT;
    }

    return mask;
}

static void command_stop (run_ctx_t * p_ctx, vehicle_cmd_t cmd, run_result_t * p_out)
{
    if (VEHICLE_SRC_BARCODE == cmd.source)
    {
        p_ctx->b_barcode = true;
    }
    else
    {
        p_ctx->b_operator = true;
    }
    p_out->b_accepted      = true;
    p_out->b_cancel_turn   = true;
    p_out->b_obstacle_stop = true;
}

static void command_start (run_ctx_t * p_ctx, const run_inputs_t * p_in, run_result_t * p_out)
{
    if (p_ctx->b_estop || p_ctx->b_overturn)
    {
        reject (p_out, RUN_REJECT_LATCHED);
    }
    else if (p_in->b_lifted)
    {
        reject (p_out, RUN_REJECT_LIFTED);
    }
    else
    {
        p_ctx->b_operator = false;
        p_ctx->b_barcode  = false;
        p_ctx->b_lift     = false;

        p_out->b_accepted       = true;
        p_out->b_obstacle_reset = true;
        p_out->b_obstacle_start = true;
    }
}

static void command_reset (run_ctx_t * p_ctx, const run_inputs_t * p_in, run_result_t * p_out)
{
    if (p_ctx->b_estop)
    {
        reject (p_out, RUN_REJECT_LATCHED);
    }
    else if (p_ctx->b_overturn && p_in->b_overturned)
    {
        reject (p_out, RUN_REJECT_LIFTED);
    }
    else
    {
        /* RESET clears the latched faults. It never releases a hold: START does. */
        p_out->b_physics_recal  = p_ctx->b_overturn;
        p_ctx->b_overturn       = false;
        p_out->b_accepted       = true;
        p_out->b_obstacle_reset = true;
    }
}

static void command_turn (run_ctx_t * p_ctx, vehicle_cmd_t cmd, const run_inputs_t * p_in,
                          run_result_t * p_out)
{
    if (RUN_STATE_RUNNING != run_state_of (p_ctx))
    {
        reject (p_out, RUN_REJECT_NOT_RUNNING);
    }
    else if (!p_in->b_follow)
    {
        reject (p_out, RUN_REJECT_BUSY);
    }
    else
    {
        p_out->b_accepted = true;
        p_out->turn       = cmd.opcode;
    }
}

void run_state_command (run_ctx_t * p_ctx, vehicle_cmd_t cmd, const run_inputs_t * p_in,
                        run_result_t * p_out)
{
    clear_result (p_out, p_ctx);

    if ((VEHICLE_SRC_BARCODE == cmd.source) && (VEHICLE_CMD_START == cmd.opcode))
    {
        reject (p_out, RUN_REJECT_SOURCE);
    }
    else if ((VEHICLE_SRC_BARCODE == cmd.source) &&
             ((VEHICLE_CMD_RESET == cmd.opcode) || (VEHICLE_CMD_ESTOP == cmd.opcode)))
    {
        reject (p_out, RUN_REJECT_SOURCE);
    }
    else
    {
        switch (cmd.opcode)
        {
            case VEHICLE_CMD_ESTOP:
                p_ctx->b_estop         = true;
                p_out->b_accepted      = true;
                p_out->b_cancel_turn   = true;
                p_out->b_obstacle_stop = true;
                p_out->b_motion_estop  = true;
            break;

            case VEHICLE_CMD_STOP:
                command_stop (p_ctx, cmd, p_out);
            break;

            case VEHICLE_CMD_START:
                command_start (p_ctx, p_in, p_out);
            break;

            case VEHICLE_CMD_RESET:
                command_reset (p_ctx, p_in, p_out);
            break;

            case VEHICLE_CMD_LEFT:
            case VEHICLE_CMD_RIGHT:
            case VEHICLE_CMD_UTURN:
            case VEHICLE_CMD_STRAIGHT:
                command_turn (p_ctx, cmd, p_in, p_out);
            break;

            default:
                reject (p_out, RUN_REJECT_OPCODE);
            break;
        }
    }

    p_out->hold_mask = run_state_hold_mask (p_ctx);
}

void run_state_tick (run_ctx_t * p_ctx, const run_inputs_t * p_in, run_result_t * p_out)
{
    bool const b_was_overturn = p_ctx->b_overturn;

    clear_result (p_out, p_ctx);

    p_ctx->b_booted = true;
    if (p_in->b_estop)
    {
        p_ctx->b_estop = true;
    }
    if (p_in->b_overturned)
    {
        p_ctx->b_overturn = true;
        p_ctx->b_lift     = true;
    }
    else if (p_in->b_lifted)
    {
        p_ctx->b_lift = true;
    }
    else
    {
        /* Level again: the lift hold stays until START. */
    }

    if (p_ctx->b_overturn && !b_was_overturn)
    {
        p_out->b_obstacle_stop = true;
        p_out->b_cancel_turn   = true;
    }
    p_out->hold_mask = run_state_hold_mask (p_ctx);
}
