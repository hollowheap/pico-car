/*
 *	run_state.c
 *	Run state of the vehicle. See run_state.h.
 */

#include "run_state.h"
#include "motion/motion.h"
#include "navigation/obstacle_cmd.h"
#include "telemetry/wire_schema.h"

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
    p_out->b_sticky_clear   = false;
    p_out->b_start          = false;
}

static void reject (run_result_t * p_out, uint8_t reason)
{
    p_out->b_rejected = true;
    p_out->reason     = reason;
}

void run_state_init (run_ctx_t * p_ctx)
{
    p_ctx->b_booted   = false;
    /* A booted car, after a watchdog reset too, waits for an operator START. */
    p_ctx->b_operator = true;
    p_ctx->b_barcode  = false;
    p_ctx->b_lift     = false;
    p_ctx->b_overturn = false;
    p_ctx->b_righted_overturn = false;
    p_ctx->b_estop    = false;
    p_ctx->b_continuous_floor = false;
    p_ctx->start_count = 0U;
    p_ctx->reset_count = 0U;
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
    bool const b_recover_pose = p_ctx->b_lift || p_ctx->b_overturn || p_in->b_overturned;

    if (p_ctx->b_estop || p_in->b_estop || (p_ctx->b_overturn && !p_in->b_upright))
    {
        reject (p_out, RUN_REJECT_LATCHED);
    }
    else if (p_in->b_lifted || (p_in->b_overturned && !p_in->b_upright))
    {
        reject (p_out, RUN_REJECT_LIFTED);
    }
    else if (p_in->b_connection_fault)
    {
        reject (p_out, RUN_REJECT_HARDWARE_FAULT);
    }
    else if (p_in->b_attitude_unknown)
    {
        reject (p_out, RUN_REJECT_NO_ATTITUDE);
        if (p_in->b_calibration_timed_out && !p_in->b_permit_lapsed)
        {
            p_out->b_physics_recal = true;
            /* A refused START must leave the car held when calibration ends. */
            if (!p_ctx->b_operator)
            {
                p_ctx->b_operator = true;
                p_out->b_cancel_turn = true;
                p_out->b_obstacle_stop = true;
            }
        }
    }
    else if (p_in->b_permit_lapsed)
    {
        reject (p_out, RUN_REJECT_STALLED);
    }
    else if (!p_in->b_upright && (p_ctx->b_lift || !p_in->b_on_hump))
    {
        reject (p_out, RUN_REJECT_LIFTED);
    }
    else
    {
        p_ctx->b_righted_overturn = p_in->b_overturned;
        p_ctx->b_overturn = false;
        p_ctx->b_operator = false;
        p_ctx->b_barcode  = false;
        p_ctx->b_lift     = false;
        p_ctx->start_count++;
        p_out->b_accepted       = true;
        p_out->b_start          = true;
        p_out->b_sticky_clear   = true;
        p_out->b_obstacle_reset = true;
        p_out->b_obstacle_start = true;
        /* A level car calibrates again before it moves; on a hump the old
           reference is better than a tilted one. */
        p_out->b_physics_recal = b_recover_pose || !p_in->b_on_hump;
    }
}

static void command_recal (const run_ctx_t * p_ctx, const run_inputs_t * p_in, run_result_t * p_out)
{
    if (p_ctx->b_estop)
    {
        reject (p_out, RUN_REJECT_LATCHED);
    }
    else if (p_in->b_lifted || p_in->b_overturned)
    {
        reject (p_out, RUN_REJECT_LIFTED);
    }
    else if (p_in->b_attitude_unknown)
    {
        reject (p_out, RUN_REJECT_NO_ATTITUDE);
    }
    else if (p_in->b_on_hump)
    {
        reject (p_out, RUN_REJECT_ON_HUMP);
    }
    else
    {
        p_out->b_accepted      = true;
        p_out->b_physics_recal = true;
    }
}

static void command_reset (run_ctx_t * p_ctx, const run_inputs_t * p_in, run_result_t * p_out)
{
    if (p_ctx->b_estop)
    {
        reject (p_out, RUN_REJECT_LATCHED);
    }
    else if ((p_ctx->b_overturn || p_in->b_overturned) && !p_in->b_upright)
    {
        reject (p_out, RUN_REJECT_LIFTED);
    }
    else
    {
        /* RESET clears the latched faults. It never releases a hold: START
           does. A running car is held like a STOP, so that a cleared latch
           does not set it going again; START then releases it. */
        if (RUN_STATE_RUNNING == run_state_of (p_ctx))
        {
            p_ctx->b_operator      = true;
            p_out->b_cancel_turn   = true;
            p_out->b_obstacle_stop = true;
        }
        /* The attitude latch clears only with a new run. */
        p_out->b_physics_recal  = p_ctx->b_overturn || p_in->b_overturned;
        p_ctx->b_righted_overturn = p_in->b_overturned;
        p_ctx->b_overturn       = false;
        p_ctx->reset_count++;
        p_out->b_accepted       = true;
        p_out->b_obstacle_reset = true;
        p_out->b_sticky_clear   = true;
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

    if (vehicle_is_start (cmd.opcode) &&
        ((VEHICLE_SRC_BARCODE == cmd.source) ||
         ((VEHICLE_CMD_START != cmd.opcode) && (VEHICLE_SRC_TELEMETRY != cmd.source))))
    {
        reject (p_out, RUN_REJECT_SOURCE);
    }
    else if ((VEHICLE_SRC_BARCODE == cmd.source) &&
             ((VEHICLE_CMD_RESET == cmd.opcode) || (VEHICLE_CMD_ESTOP == cmd.opcode) ||
              (VEHICLE_CMD_RECAL == cmd.opcode)))
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

            case VEHICLE_CMD_START_STATIONARY:
            case VEHICLE_CMD_START_CONTINUOUS_FLOOR:
            case VEHICLE_CMD_START:
                if ((RUN_STATE_RUNNING == run_state_of (p_ctx)) &&
                    ((VEHICLE_CMD_START != cmd.opcode) || !p_ctx->b_continuous_floor))
                {
                    reject (p_out, RUN_REJECT_BUSY);
                }
                else
                {
                    command_start (p_ctx, p_in, p_out);
                    if (p_out->b_start)
                    {
                        p_ctx->b_continuous_floor = (VEHICLE_CMD_START_STATIONARY != cmd.opcode);
                    }
                }
            break;

            case VEHICLE_CMD_RESET:
                command_reset (p_ctx, p_in, p_out);
            break;

            case VEHICLE_CMD_RECAL:
                command_recal (p_ctx, p_in, p_out);
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
        p_out->b_cancel_turn = !p_ctx->b_estop;
        p_ctx->b_estop = true;
    }
    /* A righted, acknowledged attitude can still read OVERTURNED while physics
       handles its new-run request. Any new unsafe pose immediately holds again. */
    if (!p_in->b_attitude_unknown && (!p_in->b_overturned || !p_in->b_upright))
    {
        p_ctx->b_righted_overturn = false;
    }
    bool const b_overturn = p_in->b_overturned && !p_ctx->b_righted_overturn;
    if (b_overturn)
    {
        p_ctx->b_overturn = true;
    }
    if (b_overturn || p_in->b_lifted)
    {
        /* Only START ends the hold, however long the car stands level. */
        p_out->b_obstacle_stop = !p_ctx->b_lift;
        p_out->b_cancel_turn = p_out->b_cancel_turn || !p_ctx->b_lift;
        p_ctx->b_lift = true;
    }
    if (p_in->b_permit_lapsed && !p_ctx->b_operator)
    {
        /* The car stops like a STOP and waits for START, so it does not drive
           on when the stalled task runs again. */
        p_ctx->b_operator      = true;
        p_out->b_cancel_turn   = true;
        p_out->b_obstacle_stop = true;
    }

    if (p_ctx->b_overturn && !b_was_overturn)
    {
        p_out->b_obstacle_stop = true;
        p_out->b_cancel_turn   = true;
    }
    p_out->hold_mask = run_state_hold_mask (p_ctx);
}

void run_state_read_physics (run_inputs_t * p_in, const run_physics_t * p_phys)
{
    bool const    b_known = p_phys->b_ready && !p_phys->b_attitude_unknown;
    uint8_t const state   = p_phys->attitude_state;
    bool const    b_hump  = p_phys->b_hump_active ||
                            (TELEMETRY_HUMP_STATE_FLAT != p_phys->hump_state);

    p_in->b_attitude_unknown = !b_known;
    p_in->b_lifted           = b_known && (TELEMETRY_ATTITUDE_STATE_LIFTED == state);
    p_in->b_overturned       = b_known &&
                               (TELEMETRY_ATTITUDE_STATE_OVERTURNED == state);
    p_in->b_upright          = b_known && p_phys->b_calibrated && p_phys->b_level_still;
    p_in->b_on_hump          = p_phys->b_ready && b_hump;
}

bool run_permit_lapsed (bool b_skipped, uint32_t gap_ms, uint32_t lease_ms)
{
    return b_skipped || (gap_ms > lease_ms);
}

bool run_permit_skipped (bool b_critical, bool b_renew_failed)
{
    return b_critical || b_renew_failed;
}

uint8_t run_obstacle_post (uint8_t owed, const run_result_t * p_res, bool (*p_refused) (uint8_t cmd))
{
    uint8_t want = p_res->b_obstacle_stop ? 0U : owed;

    if (p_res->b_obstacle_reset)
    {
        want = (uint8_t) (want | RUN_OWE_RESET);
    }
    if (p_res->b_obstacle_start)
    {
        want = (uint8_t) (want | RUN_OWE_START);
    }
    if ((0U != (want & RUN_OWE_RESET)) && !p_refused (OBSTACLE_CMD_RESET))
    {
        want = (uint8_t) (want & ~RUN_OWE_RESET);
    }
    /* START only after its RESET has gone. */
    if ((0U == (want & RUN_OWE_RESET)) && (0U != (want & RUN_OWE_START)) &&
        !p_refused (OBSTACLE_CMD_START))
    {
        want = (uint8_t) (want & ~RUN_OWE_START);
    }

    return want;
}
