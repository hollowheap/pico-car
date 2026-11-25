/*
 *	test_supervisor.c
 *	Host tests for the command ring, the run state and the health checks.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "command_ring.h"
#include "supervisor/edge_events.h"
#include "health_core.h"
#include "obstacle/obstacle_cmd.h"
#include "platform/liveness.h"
#include "comms/telemetry/wire_schema.h"
#include "run_state.h"
#include "motion/motion_state.h"

static unsigned g_fail;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            (void)printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_fail++;                                                        \
        }                                                                    \
    } while (0)

static vehicle_cmd_t cmd_of (uint8_t opcode, uint8_t source)
{
    vehicle_cmd_t cmd;

    cmd.cmd_id = 0U;
    cmd.run_id = 0U;
    cmd.opcode = opcode;
    cmd.source = source;
    return cmd;
}

static void test_ring_order_and_source (void)
{
    command_ring_t ring;
    vehicle_cmd_t  cmd;

    command_ring_init (&ring);
    CHECK (!command_ring_pop (&ring, &cmd));
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE)));
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY)));
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BUTTON)));

    CHECK (command_ring_pop (&ring, &cmd));
    CHECK ((VEHICLE_CMD_LEFT == cmd.opcode) && (VEHICLE_SRC_BARCODE == cmd.source));
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK ((VEHICLE_CMD_STOP == cmd.opcode) && (VEHICLE_SRC_TELEMETRY == cmd.source));
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK ((VEHICLE_CMD_START == cmd.opcode) && (VEHICLE_SRC_BUTTON == cmd.source));
    CHECK (!command_ring_pop (&ring, &cmd));
}

static void test_ring_full_keeps_stops (void)
{
    command_ring_t ring;
    vehicle_cmd_t  cmd;
    uint8_t        idx = 0U;

    command_ring_init (&ring);
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY)));
    for (idx = 1U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY)));
    }
    CHECK (COMMAND_RING_LEN == ring.count);

    /* Full: a new command removes the oldest command that is not a stop. */
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_RIGHT, VEHICLE_SRC_TELEMETRY)));
    CHECK (1U == ring.dropped);
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK (VEHICLE_CMD_STOP == cmd.opcode);
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK (VEHICLE_CMD_LEFT == cmd.opcode);

    /* A ring of only stops drops the new command, never a stop. */
    command_ring_init (&ring);
    for (idx = 0U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY)));
    }
    CHECK (!command_ring_push (&ring, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY)));
    CHECK (1U == ring.dropped);
    CHECK (COMMAND_RING_LEN == ring.count);

    /* A stop pushed into a full ring of non-stops is kept. */
    command_ring_init (&ring);
    for (idx = 0U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_UTURN, VEHICLE_SRC_TELEMETRY)));
    }
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY)));
    for (idx = 0U; idx < (COMMAND_RING_LEN - 1U); idx++)
    {
        CHECK (command_ring_pop (&ring, &cmd));
    }
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK (VEHICLE_CMD_STOP == cmd.opcode);
}

/* The command and run ids of a remote command reach the reader unchanged,
   also when a full ring moves the stored commands. */
static void test_ring_keeps_ids (void)
{
    command_ring_t ring;
    vehicle_cmd_t  cmd = cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY);
    uint8_t        idx = 0U;

    CHECK (sizeof (vehicle_cmd_t) == 6U);
    CHECK (sizeof (ring.item) == (COMMAND_RING_LEN * 6U));

    command_ring_init (&ring);
    cmd.run_id = 0xBEEFU;
    for (idx = 0U; idx < COMMAND_RING_LEN; idx++)
    {
        cmd.cmd_id = (uint16_t) (0xFFFCU + idx); /* wraps after 0xFFFF */
        CHECK (command_ring_push (&ring, cmd));
    }
    /* Full: the oldest command (id 0xFFFC) goes, the others move up. */
    cmd.opcode = VEHICLE_CMD_STOP;
    cmd.cmd_id = 0x0004U;
    CHECK (command_ring_push (&ring, cmd));

    for (idx = 1U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_pop (&ring, &cmd));
        CHECK ((uint16_t) (0xFFFCU + idx) == cmd.cmd_id);
        CHECK ((0xBEEFU == cmd.run_id) && (VEHICLE_SRC_TELEMETRY == cmd.source));
        CHECK (VEHICLE_CMD_LEFT == cmd.opcode);
    }
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK ((0x0004U == cmd.cmd_id) && (0xBEEFU == cmd.run_id));
    CHECK (VEHICLE_CMD_STOP == cmd.opcode);
    CHECK (!command_ring_pop (&ring, &cmd));

    /* A local command carries no ids. */
    CHECK (command_ring_push (&ring, cmd_of (VEHICLE_CMD_RIGHT, VEHICLE_SRC_BARCODE)));
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK ((0U == cmd.cmd_id) && (0U == cmd.run_id));
}

static run_inputs_t follow_inputs (void)
{
    run_inputs_t in = { .b_follow = true, .b_upright = true };

    return in;
}

/* A booted car that an operator START has released. */
static void start_running (run_ctx_t * p_ctx, const run_inputs_t * p_in)
{
    run_result_t out;

    run_state_init (p_ctx);
    run_state_tick (p_ctx, p_in, &out);
    run_state_command (p_ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       p_in, &out);
}

static void test_boot_and_stop_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx);
    CHECK (RUN_STATE_BOOT == run_state_of (&ctx));
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_start);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_stop && out.b_cancel_turn);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_reset && !out.b_obstacle_start);
    /* The operator has seen the faults: RESET clears the sticky word. */
    CHECK (out.b_sticky_clear);
    /* RESET never releases a hold. */
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_reset && out.b_obstacle_start);
    CHECK (out.b_start && out.b_sticky_clear);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
}

/* Nothing but an operator START lets a car drive after boot: not time, not a
 * calm tick, not a RESET, not a barcode. */
static void test_boot_holds_until_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in   = follow_inputs ();
    uint32_t     pass = 0U;

    run_state_init (&ctx);
    for (pass = 0U; pass < 100U; pass++)
    {
        run_state_tick (&ctx, &in, &out);
        CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
        CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);
    }
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_STATE_STOPPED == run_state_of (&ctx)));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (RUN_STATE_RUNNING == run_state_of (&ctx)));
}

/* A watchdog reset reboots through the same init: the car that was running
 * holds again until START. */
static void test_reboot_holds_until_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);
}

static void test_stop_wins_in_a_batch (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (VEHICLE_CMD_LEFT == out.turn);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_cancel_turn);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_NOT_RUNNING == out.reason));
    CHECK (RUN_TURN_NONE == out.turn);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
}

static void test_turn_rules (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);

    /* Before START no turn runs, from a barcode or from the dashboard. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_NOT_RUNNING == out.reason));
    CHECK (RUN_TURN_NONE == out.turn);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RIGHT, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_TURN_NONE == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_TURN_NONE == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_TURN_NONE == out.turn));

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RIGHT, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_RIGHT == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_UTURN == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STRAIGHT, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_STRAIGHT == out.turn));
    run_state_command (&ctx,
                       cmd_of (VEHICLE_CMD_STRAIGHT, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_STRAIGHT == out.turn));

    /* Busy: a bypass, a scan or a turn in progress. */
    in.b_follow = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_BUSY == out.reason));
    CHECK (RUN_TURN_NONE == out.turn);
}

static void test_barcode_stop (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_stop);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    CHECK (MOTION_HOLD_BARCODE == out.hold_mask);

    /* A barcode cannot start, reset or emergency-stop the car. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    CHECK (!out.b_start);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    CHECK (!out.b_motion_estop);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));

    /* The dashboard START releases the barcode hold. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
}

/* RESET while the car runs clears the faults and holds the wheels until START:
 * a cleared latch does not set the car going again by itself. */
static void test_reset_while_running_holds_until_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_sticky_clear && out.b_obstacle_reset);
    CHECK (out.b_cancel_turn && out.b_obstacle_stop);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);
    run_state_tick (&ctx, &in, &out);
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);

    /* RESET also cancels any pending route while already held. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (MOTION_HOLD_OPERATOR == out.hold_mask));
    CHECK (out.b_cancel_turn && out.b_obstacle_stop);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_start);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);

    /* The emergency stop stays latched: RESET is refused and sets no hold. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && !out.b_sticky_clear && !out.b_cancel_turn);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
}

/* START in the pass that sees the overturn is refused, as RECAL is. RESET is
 * refused only when overturned and not upright. The car stays held. */
static void test_start_refused_when_overturned (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);

    in.b_overturned = true;
    in.b_upright = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));
    CHECK (!out.b_obstacle_start && !out.b_physics_recal);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);
}

static void test_emergency_stop_latches (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_motion_estop && out.b_obstacle_stop);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    CHECK (!out.b_sticky_clear);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));

    /* The motion latch seen on a tick latches the state too (the STOP pin). */
    run_state_init (&ctx);
    in.b_estop = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && !out.b_sticky_clear && !out.b_accepted);
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
}

/* Long enough that a hold ending by itself would show with the car level. */
#define LONG_LEVEL_PASSES (500U)

static void test_lift_and_overturn (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);

    /* A pickup immediately stops the run and holds both wheels. */
    in.b_lifted = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK ((MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT) == out.hold_mask);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));
    CHECK (!out.b_start);

    /* Set down: START ends the hold at once. */
    in.b_lifted = false;
    /* A lingering hump state cannot skip a level set-down calibration. */
    in.b_on_hump = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal && (0U == out.hold_mask));
    in.b_on_hump = false;
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));

    /* With no START the hold never ends, however long the car stands level,
       and a later lift changes nothing. */
    {
        uint32_t pass = 0U;

        in.b_lifted = true;
        run_state_tick (&ctx, &in, &out);
        in.b_lifted = false;
        for (pass = 0U; pass < LONG_LEVEL_PASSES; pass++)
        {
            run_state_tick (&ctx, &in, &out);
            CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
            CHECK ((MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT) == out.hold_mask);
            CHECK (!out.b_physics_recal);
        }
        run_state_command (&ctx,
                           cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                           &in, &out);
        CHECK (out.b_accepted && out.b_physics_recal);
        CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
        CHECK (0U == out.hold_mask);
    }

    /* A righted car needs one START. The old attitude latch can outlive the
       accepted command until physics handles the new calibration request. */
    in.b_overturned = true;
    in.b_upright = false;
    run_state_tick (&ctx, &in, &out);
    in.b_upright = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    uint8_t const starts = ctx.start_count;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal && out.b_sticky_clear);
    CHECK ((starts + 1U == ctx.start_count) && (0U == ctx.reset_count));
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
    in.b_attitude_unknown = true;
    in.b_overturned = false;
    in.b_upright = false;
    run_state_tick (&ctx, &in, &out);
    in.b_attitude_unknown = false;
    in.b_overturned = true;
    in.b_upright = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    in.b_upright = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    in = follow_inputs ();
    start_running (&ctx, &in);

    /* Overturned but upright before any latch: RESET is accepted and asks for
       a new run, which clears the attitude latch. */
    in.b_overturned = true;
    in.b_upright    = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal);
    in.b_overturned = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (RUN_STATE_RUNNING == run_state_of (&ctx)));

    /* Overturned and upside down: latched, the obstacle task is stopped once,
       START refused. */
    in.b_overturned = true;
    in.b_lifted     = true;
    in.b_upright    = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    CHECK (out.b_obstacle_stop);
    run_state_tick (&ctx, &in, &out);
    CHECK (!out.b_obstacle_stop);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));

    /* RESET while still upside down is refused and clears nothing. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));
    CHECK (!out.b_sticky_clear && !out.b_physics_recal);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));

    /* Righted, the attitude latch still reads OVERTURNED until physics starts
       a new run. RESET clears the latch and asks for that run; the car then
       waits for START. */
    in.b_upright = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_reset && out.b_sticky_clear);
    /* The IMU calibrates again after an overturn. */
    CHECK (out.b_physics_recal);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    CHECK (MOTION_HOLD_LIFT == out.hold_mask);

    /* The stale latch does not latch the car again, and START waits until the
       attitude reads OK. */
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));
    in.b_overturned = false;
    in.b_lifted     = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

/* A RESET on a car held by a lift or an overturn clears the latch, but the car
 * stays held until START. */
static void test_reset_in_a_lift_waits_for_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();
    uint32_t     pass = 0U;

    /* Overturned, set upright, RESET, then standing level for a long time. */
    start_running (&ctx, &in);
    in.b_overturned = true;
    in.b_lifted     = true;
    in.b_upright    = false;
    run_state_tick (&ctx, &in, &out);
    in.b_overturned = false;
    in.b_lifted     = false;
    in.b_upright    = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted);
    for (pass = 0U; pass < (2U * LONG_LEVEL_PASSES); pass++)
    {
        run_state_tick (&ctx, &in, &out);
        CHECK (RUN_STATE_HELD == run_state_of (&ctx));
        CHECK (0U != out.hold_mask);
    }
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (RUN_STATE_RUNNING == run_state_of (&ctx)));
    CHECK (0U == out.hold_mask);

    /* Lifted while running, set down, RESET, then level: held until START. */
    in.b_lifted = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    in.b_lifted = false;
    run_state_tick (&ctx, &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted);
    for (pass = 0U; pass < (2U * LONG_LEVEL_PASSES); pass++)
    {
        run_state_tick (&ctx, &in, &out);
        CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
        CHECK ((MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT) == out.hold_mask);
    }
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (RUN_STATE_RUNNING == run_state_of (&ctx)));

    /* A plain lift with no RESET holds until START as well. */
    in.b_lifted = true;
    run_state_tick (&ctx, &in, &out);
    in.b_lifted = false;
    for (pass = 0U; pass < LONG_LEVEL_PASSES; pass++)
    {
        run_state_tick (&ctx, &in, &out);
    }
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (RUN_STATE_RUNNING == run_state_of (&ctx)));
}

/* A critical task stalls: motion brakes while the permit is lapsed,
   cancels any active turn, and resumes in place when the permit renews. */
static void test_stall_holds_until_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    in.b_permit_lapsed = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
    CHECK (out.b_cancel_turn);

    in.b_permit_lapsed = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
}

/* Nothing holds a running car by time: run_state has no link input, so a long
   stretch of ticks with the same facts keeps the car running. */
static void test_running_car_is_not_held_by_time (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in     = follow_inputs ();
    uint32_t     pass   = 0U;
    bool         b_held = false;

    start_running (&ctx, &in);
    for (pass = 0U; pass < (600000U / RUN_PASS_MS); pass++)
    {
        run_state_tick (&ctx, &in, &out);
        b_held = b_held || (RUN_STATE_RUNNING != run_state_of (&ctx)) ||
                 (0U != out.hold_mask) || out.b_obstacle_stop ||
                 out.b_cancel_turn;
    }
    CHECK (!b_held);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

static void test_remote_stop_after_silent_run (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in   = follow_inputs ();
    uint32_t     pass = 0U;

    start_running (&ctx, &in);
    for (pass = 0U; pass < (70000U / RUN_PASS_MS); pass++)
    {
        run_state_tick (&ctx, &in, &out);
    }
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && (MOTION_HOLD_OPERATOR == out.hold_mask));
}

static void test_single_estop_sample_survives_release (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    in.b_estop = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    in.b_estop = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
}

static void test_lift_reading_holds_on_first_tick (void)
{
    run_ctx_t     ctx;
    run_result_t  out;
    run_inputs_t  in   = follow_inputs ();
    run_physics_t phys = {
        .b_ready = true,
        .b_calibrated = true,
        .attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED,
    };

    start_running (&ctx, &in);
    run_state_read_physics (&in, &phys);
    run_state_tick (&ctx, &in, &out);
    CHECK ((MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT) == out.hold_mask);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
}

/* RESET reads the overturn now, not only the latch: a car that turned over
 * since the last tick is refused with no tick in between. */
static void test_reset_refused_upside_down_without_tick (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    in.b_overturned = true;
    in.b_lifted     = true;
    in.b_upright    = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));
    CHECK (!out.b_sticky_clear && !out.b_physics_recal);
}

/* The run stays armed while the supervisor owns the temporary calibration
 * hold. START and RECAL require a known attitude. */
static void test_attitude_unknown (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    in.b_attitude_unknown = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_NO_ATTITUDE == out.reason));
    CHECK (!out.b_start && (RUN_STATE_STOPPED == run_state_of (&ctx)));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_NO_ATTITUDE == out.reason));
    CHECK (!out.b_physics_recal);

    in.b_attitude_unknown = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_start);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

/* The lift hold does not end because the lean is not known. */
static void test_attitude_unknown_keeps_lift_hold (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    in.b_lifted = true;
    run_state_tick (&ctx, &in, &out);
    in.b_lifted           = false;
    in.b_attitude_unknown = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK ((MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT) == out.hold_mask);
}

/* A stall that lapses the motion permit: START waits until the permit is
 * back. */
static void test_start_refused_while_permit_lapsed (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);
    in.b_permit_lapsed = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_STALLED == out.reason));
    CHECK (!out.b_start && !out.b_obstacle_start);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));

    in.b_permit_lapsed = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_start);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

/* The permit lapses on a skipped renew, or when the gap since the last renew,
 * in whole ms rounded up as the motion service counts it, is over the lease. */
static void test_permit_lapse_rule (void)
{
    CHECK (!run_permit_lapsed (false, 20U, 200U));
    CHECK (!run_permit_lapsed (false, 200U, 200U));
    CHECK (run_permit_lapsed (false, 201U, 200U));
    CHECK (run_permit_lapsed (true, 0U, 200U));
}

/* A renew is skipped for a critical stall, and also when the motion service
 * refuses it. */
static void test_permit_skipped_rule (void)
{
    CHECK (!run_permit_skipped (false, false));
    CHECK (run_permit_skipped (true, false));
    CHECK (run_permit_skipped (false, true));
    CHECK (run_permit_skipped (true, true));
}

/* The start and reset counts move on an accepted command only. */
static void test_start_reset_counts (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (0U == ctx.start_count);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (1U == ctx.start_count);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK ((1U == ctx.reset_count) && (1U == ctx.start_count));

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK ((1U == ctx.reset_count) && (1U == ctx.start_count));
}

static run_physics_t level_physics (void)
{
    run_physics_t p;

    p.b_ready            = true;
    p.b_calibrated       = true;
    p.b_attitude_unknown = false;
    p.b_hump_active      = false;
    p.hump_state         = TELEMETRY_HUMP_STATE_FLAT;
    p.attitude_state     = TELEMETRY_ATTITUDE_STATE_OK;
    p.b_level_still      = true;

    return p;
}

/* The physics facts become the inputs. The attitude is read only while known,
 * and recovery requires a reference and sustained level, still samples. */
static void test_physics_inputs (void)
{
    run_inputs_t in = follow_inputs ();
    run_physics_t p = level_physics ();

    run_state_read_physics (&in, &p);
    CHECK (!in.b_attitude_unknown && !in.b_lifted && !in.b_overturned);
    CHECK (in.b_upright && !in.b_on_hump);

    p.b_level_still = false;
    run_state_read_physics (&in, &p);
    CHECK (!in.b_upright);
    p.b_level_still = true;
    p.b_calibrated = false;
    run_state_read_physics (&in, &p);
    CHECK (!in.b_upright);
    p.b_calibrated = true;

    p.attitude_state = TELEMETRY_ATTITUDE_STATE_LIFTED;
    run_state_read_physics (&in, &p);
    CHECK (in.b_lifted && !in.b_overturned);
    p.attitude_state = TELEMETRY_ATTITUDE_STATE_OVERTURNED;
    run_state_read_physics (&in, &p);
    CHECK (!in.b_lifted && in.b_overturned);
    p.attitude_state = TELEMETRY_ATTITUDE_STATE_OK;

    p.b_hump_active = true;
    run_state_read_physics (&in, &p);
    CHECK (in.b_on_hump);
    p.b_hump_active = false;
    p.hump_state    = TELEMETRY_HUMP_STATE_CLIMB;
    run_state_read_physics (&in, &p);
    CHECK (in.b_on_hump);
    p.hump_state = TELEMETRY_HUMP_STATE_FLAT;

    p.b_attitude_unknown = true;
    p.attitude_state     = TELEMETRY_ATTITUDE_STATE_LIFTED;
    run_state_read_physics (&in, &p);
    CHECK (in.b_attitude_unknown && !in.b_lifted && !in.b_upright);
    p.attitude_state = TELEMETRY_ATTITUDE_STATE_OVERTURNED;
    run_state_read_physics (&in, &p);
    CHECK (!in.b_overturned);
    p.b_attitude_unknown = false;
    p.attitude_state     = TELEMETRY_ATTITUDE_STATE_OK;

    p.b_ready = false;
    run_state_read_physics (&in, &p);
    CHECK (in.b_attitude_unknown && !in.b_lifted && !in.b_overturned);
    CHECK (!in.b_upright && !in.b_on_hump);
}

/* A fake obstacle command queue: it refuses a post while it has no room, and
   records the posts it took. */
#define FAKE_QUEUE_LOG (8U)

static uint8_t g_queue_room;
static uint8_t g_posted[FAKE_QUEUE_LOG];
static uint8_t g_posted_count;

static void fake_queue (uint8_t room)
{
    g_queue_room   = room;
    g_posted_count = 0U;
}

static bool fake_refused (uint8_t cmd)
{
    bool const b_refused = (0U == g_queue_room) || (g_posted_count >= FAKE_QUEUE_LOG);

    if (!b_refused)
    {
        g_queue_room--;
        g_posted[g_posted_count] = cmd;
        g_posted_count++;
    }

    return b_refused;
}

static void test_obstacle_post_retry (void)
{
    run_ctx_t    ctx;
    run_result_t idle;
    run_result_t start;
    run_result_t stop;
    run_inputs_t in   = follow_inputs ();
    uint8_t      owed = 0U;

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &idle);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &start);

    /* A full queue refuses the RESET: the START waits behind it. */
    fake_queue (0U);
    owed = run_obstacle_post (0U, &start, fake_refused);
    CHECK ((RUN_OWE_RESET | RUN_OWE_START) == owed);
    CHECK (0U == g_posted_count);
    owed = run_obstacle_post (owed, &idle, fake_refused);
    CHECK ((RUN_OWE_RESET | RUN_OWE_START) == owed);

    /* Room on a later pass: RESET, then START. */
    fake_queue (4U);
    owed = run_obstacle_post (owed, &idle, fake_refused);
    CHECK (0U == owed);
    CHECK ((2U == g_posted_count) && (OBSTACLE_CMD_RESET == g_posted[0]) &&
           (OBSTACLE_CMD_START == g_posted[1]));

    /* Room for the RESET only: the START alone is owed, and sent next pass. */
    fake_queue (1U);
    owed = run_obstacle_post (0U, &start, fake_refused);
    CHECK (RUN_OWE_START == owed);
    fake_queue (4U);
    owed = run_obstacle_post (owed, &idle, fake_refused);
    CHECK ((0U == owed) && (1U == g_posted_count) &&
           (OBSTACLE_CMD_START == g_posted[0]));

    /* Nothing owed and nothing asked: nothing is posted. */
    fake_queue (4U);
    CHECK (0U == run_obstacle_post (0U, &idle, fake_refused));
    CHECK (0U == g_posted_count);

    /* A stop drops what is owed: a START that never arrived must not run
       after it. */
    fake_queue (0U);
    owed = run_obstacle_post (0U, &start, fake_refused);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &stop);
    fake_queue (4U);
    CHECK (0U == run_obstacle_post (owed, &stop, fake_refused));
    CHECK (0U == g_posted_count);
}

static void test_unknown_opcode (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    start_running (&ctx, &in);
    run_state_command (&ctx, cmd_of (99U, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_OPCODE == out.reason));
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

static void test_liveness (void)
{
    liveness_t live;
    uint16_t   fresh = 0U;
    uint16_t   back  = 0U;
    uint8_t    idx   = 0U;
    uint16_t   const motion = (uint16_t) (1U << LIVE_MOTION);
    uint16_t   const phys   = (uint16_t) (1U << LIVE_PHYSICS);

    liveness_init (&live, (uint16_t) (motion | phys));

    /* Both tasks run: nothing stalls. */
    for (idx = 0U; idx < 20U; idx++)
    {
        liveness_check (&live, (uint16_t) (motion | phys), &fresh, &back);
    }
    CHECK (0U == live.stalled);

    /* Physics stops kicking: stalled after the limit, not before. */
    for (idx = 1U; idx < LIVENESS_MISS_LIMIT; idx++)
    {
        liveness_check (&live, motion, &fresh, &back);
        CHECK (0U == fresh);
    }
    liveness_check (&live, motion, &fresh, &back);
    CHECK (phys == fresh);
    CHECK (phys == live.stalled);
    CHECK (!liveness_critical_stalled (&live)); /* physics is not critical */
    liveness_check (&live, motion, &fresh, &back);
    CHECK (0U == fresh); /* reported once */

    /* It recovers. */
    liveness_check (&live, (uint16_t) (motion | phys), &fresh, &back);
    CHECK (phys == back);
    CHECK (0U == live.stalled);

    /* A critical task stalls. */
    for (idx = 0U; idx < LIVENESS_MISS_LIMIT; idx++)
    {
        liveness_check (&live, phys, &fresh, &back);
    }
    CHECK (motion == live.stalled);
    CHECK (liveness_critical_stalled (&live));
}

static void test_liveness_boot_allowance (void)
{
    liveness_t live;
    uint16_t   fresh = 0U;
    uint16_t   back  = 0U;
    uint8_t    idx   = 0U;
    uint16_t   const line = (uint16_t) (1U << LIVE_LINE);

    liveness_init (&live, line);

    /* A task that has not started is not stalled during the boot allowance. */
    for (idx = 0U; idx < (LIVENESS_BOOT_CHECKS - 1U); idx++)
    {
        liveness_check (&live, 0U, &fresh, &back);
    }
    CHECK (0U == live.stalled);

    /* After the allowance a required task that never kicked counts misses. */
    for (idx = 0U; idx < LIVENESS_MISS_LIMIT; idx++)
    {
        liveness_check (&live, 0U, &fresh, &back);
    }
    CHECK (line == live.stalled);

    /* A task that is not required is never stalled. */
    liveness_init (&live, 0U);
    for (idx = 0U; idx < 100U; idx++)
    {
        liveness_check (&live, 0U, &fresh, &back);
    }
    CHECK (0U == live.stalled);
}

/* Only a LIVE_* id reaches the word; a larger id is dropped, never shifted. */
static void test_liveness_kick_range (void)
{
    (void) liveness_take ();
    liveness_kick (LIVE_BARCODE);
    CHECK ((uint16_t) (1U << LIVE_BARCODE) == liveness_take ());
    CHECK (0U == liveness_take ());

    liveness_kick (LIVE_TASKS);
    liveness_kick (31U);
    liveness_kick (32U);
    liveness_kick (UINT8_MAX);
    CHECK (0U == liveness_take ());
}

/* The supervisor checks every 100 ms; a timed wait can run one 10 ms tick long. */
#define LIVE_CHECK_MS (100U)
#define KERNEL_TICK_MS (10U)

/* One task that kicks every period_ms, the kick landing just before a check
   when both fall on the same tick. True if it is ever stalled. */
static bool stalls_with_kicks_every (uint32_t period_ms)
{
    liveness_t     live;
    uint16_t const bar     = (uint16_t) (1U << LIVE_BARCODE);
    uint16_t       taken   = 0U;
    uint16_t       fresh   = 0U;
    uint16_t       back    = 0U;
    uint32_t       t       = 0U;
    bool           b_stall = false;

    liveness_init (&live, bar);
    for (t = 0U; t <= 10000U; t += KERNEL_TICK_MS)
    {
        if (0U == (t % period_ms))
        {
            taken = bar;
        }
        if (0U == (t % LIVE_CHECK_MS))
        {
            liveness_check (&live, taken, &fresh, &back);
            taken   = 0U;
            b_stall = b_stall || (0U != live.stalled);
        }
    }

    return b_stall;
}

/* A task that waits LIVENESS_WAIT_MS and kicks after each wait never stalls;
   one 500 ms wait per kick misses LIVENESS_MISS_LIMIT checks. */
static void test_liveness_wait_fits_budget (void)
{
    CHECK (!stalls_with_kicks_every (LIVENESS_WAIT_MS + KERNEL_TICK_MS));
    CHECK (stalls_with_kicks_every (500U + KERNEL_TICK_MS));
}

static void test_progress (void)
{
    progress_t prog;
    uint32_t   t = 0U;
    uint32_t   mm = 0U;
    bool       b_trip = false;

    progress_init (&prog);

    /* Moving: no trip. */
    for (t = 0U; t < 5000U; t += 100U)
    {
        mm += 5U;
        CHECK (!progress_update (&prog, true, mm, 100U));
    }

    /* Told to move but standing still: trips once after 3 s. */
    for (t = 0U; t < PROGRESS_STALL_MS; t += 100U)
    {
        CHECK (!b_trip);
        b_trip = progress_update (&prog, true, mm, 100U);
    }
    CHECK (b_trip);
    CHECK (!progress_update (&prog, true, mm, 100U));

    /* Not expected to move (held): no trip, and the count restarts. */
    for (t = 0U; t < 10000U; t += 100U)
    {
        CHECK (!progress_update (&prog, false, mm, 100U));
    }
    CHECK (0U == prog.still_ms);

    /* Slow motion of 3 mm per second is progress. */
    for (t = 0U; t < 10000U; t += 100U)
    {
        if (0U == ((t / 100U) % 10U))
        {
            mm += 3U;
        }
        CHECK (!progress_update (&prog, true, mm, 100U));
    }
}

static void test_stuck_sensor (void)
{
    stuck_t  stuck;
    uint32_t mm = 0U;

    stuck_init (&stuck);
    /* Edges keep coming: no trip however far the car drives. */
    for (mm = 0U; mm < 6000U; mm += 100U)
    {
        CHECK (!stuck_update (&stuck, true, mm / 200U, mm));
    }
    /* Edges stop: trips once after STUCK_TRAVEL_MM of driving. */
    {
        uint32_t const edges = 6000U / 200U;
        uint32_t const start = mm;
        bool           b_new = false;

        for (; mm < (start + STUCK_TRAVEL_MM - 100U); mm += 100U)
        {
            CHECK (!stuck_update (&stuck, true, edges, mm));
        }
        b_new = stuck_update (&stuck, true, edges, start + STUCK_TRAVEL_MM);
        CHECK (b_new);
        CHECK (!stuck_update (&stuck, true, edges, start + STUCK_TRAVEL_MM + 100U));
        /* A held car restarts the count. */
        CHECK (!stuck_update (&stuck, false, edges, start + STUCK_TRAVEL_MM + 200U));
        CHECK (!stuck_update (&stuck, true, edges, start + STUCK_TRAVEL_MM + 300U));
    }
}

/* One 100 ms check with both wheels on the stated values. */
static bool drive_step (drive_t * p_drive, int32_t tl, int32_t tr, int32_t sl, int32_t sr,
                        int16_t dl, int16_t dr)
{
    drive_in_t in;

    in.target_um_s[0] = tl;
    in.target_um_s[1] = tr;
    in.speed_um_s[0]  = sl;
    in.speed_um_s[1]  = sr;
    in.duty_pm[0]     = dl;
    in.duty_pm[1]     = dr;
    in.b_judge        = true;
    return health_drive_update (p_drive, &in, 100U);
}

static void test_drive_crossed (void)
{
    drive_t  drive;
    unsigned idx   = 0U;
    unsigned fired = 0U;

    /* A good car: the faster-commanded wheel turns faster, with some lag. */
    health_drive_init (&drive);
    for (idx = 0U; idx < 100U; idx++)
    {
        fired += drive_step (&drive, 163000, 0, 150000, 30000, 400, 0) ? 1U : 0U;
        fired += drive_step (&drive, 0, 163000, 20000, 150000, 0, 400) ? 1U : 0U;
    }
    CHECK (0U == fired);
    CHECK (!drive.b_crossed);

    /* Swapped plugs: the left command moves the right encoder. */
    health_drive_init (&drive);
    fired = 0U;
    for (idx = 0U; idx < 40U; idx++)
    {
        fired += drive_step (&drive, 163000, 0, 0, 163000, 400, 0) ? 1U : 0U;
    }
    CHECK (1U == fired);
    CHECK (drive.b_crossed);
    /* Fixed: the window after the repair agrees and clears the flag. */
    for (idx = 0U; idx < 40U; idx++)
    {
        (void) drive_step (&drive, 163000, 0, 163000, 0, 400, 0);
    }
    CHECK (!drive.b_crossed);

    /* One encoder counts backwards: both wheels go forward, the left reads negative. */
    health_drive_init (&drive);
    fired = 0U;
    for (idx = 0U; idx < 40U; idx++)
    {
        fired += drive_step (&drive, 163000, 163000, -160000, 160000, 400, 400) ? 1U : 0U;
    }
    CHECK (1U == fired);
}

/* A wheel held well under its forward target with much more duty than the
   grounded baseline asks pushes against something. */
/* A crawl target above DRIVE_LOAD_TARGET_MIN_UM_S, a wheel at a quarter of
 * it, and duties against a baseline learnt at BASE_UM_S with BASE_DUTY. The
 * push duty is over DRIVE_LOAD_DUTY_TIMES the baseline need at the crawl. */
#define LOAD_TARGET_UM_S (80000)
#define LOAD_SLOW_UM_S   (LOAD_TARGET_UM_S / 4)
#define LOAD_BASE_UM_S   (200000)
#define LOAD_BASE_DUTY   (400)
#define LOAD_BASE_PASSES (30U)
#define LOAD_FREE_PASSES (5U)
#define LOAD_FREE_DUTY   (180)
#define LOAD_RISE_DUTY   (200)
#define LOAD_PUSH_DUTY   (500)

static void test_drive_loaded (void)
{
    drive_t  drive;
    unsigned idx = 0U;

    health_drive_init (&drive);
    /* No baseline yet: a slow wheel with high duty is no contact. */
    for (idx = 0U; idx < DRIVE_LOAD_CHECKS; idx++)
    {
        (void) drive_step (&drive, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_SLOW_UM_S, LOAD_SLOW_UM_S,
                           LOAD_PUSH_DUTY, LOAD_PUSH_DUTY);
    }
    CHECK (!drive.b_loaded);
    for (idx = 0U; idx < LOAD_BASE_PASSES; idx++)
    {
        (void) drive_step (&drive, LOAD_BASE_UM_S, LOAD_BASE_UM_S, LOAD_BASE_UM_S, LOAD_BASE_UM_S,
                           LOAD_BASE_DUTY, LOAD_BASE_DUTY);
    }
    /* On target at the crawl: no contact. */
    for (idx = 0U; idx < LOAD_FREE_PASSES; idx++)
    {
        (void) drive_step (&drive, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S,
                           LOAD_FREE_DUTY, LOAD_FREE_DUTY);
    }
    CHECK (!drive.b_loaded);
    /* Slow but with low duty (still speeding up): no contact. */
    for (idx = 0U; idx < DRIVE_LOAD_CHECKS; idx++)
    {
        (void) drive_step (&drive, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_SLOW_UM_S, LOAD_SLOW_UM_S,
                           LOAD_RISE_DUTY, LOAD_RISE_DUTY);
    }
    CHECK (!drive.b_loaded);
    /* One wheel pushes for DRIVE_LOAD_CHECKS checks. */
    (void) drive_step (&drive, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_SLOW_UM_S, LOAD_TARGET_UM_S,
                       LOAD_PUSH_DUTY, LOAD_FREE_DUTY);
    CHECK (!drive.b_loaded);
    (void) drive_step (&drive, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_SLOW_UM_S, LOAD_TARGET_UM_S,
                       LOAD_PUSH_DUTY, LOAD_FREE_DUTY);
    CHECK (drive.b_loaded);
    /* Free again. */
    (void) drive_step (&drive, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S, LOAD_TARGET_UM_S,
                       LOAD_FREE_DUTY, LOAD_FREE_DUTY);
    CHECK (!drive.b_loaded);
    /* Reverse targets are not judged. */
    for (idx = 0U; idx < DRIVE_LOAD_CHECKS; idx++)
    {
        (void) drive_step (&drive, -LOAD_TARGET_UM_S, -LOAD_TARGET_UM_S, -LOAD_SLOW_UM_S, -LOAD_SLOW_UM_S,
                           -LOAD_PUSH_DUTY, -LOAD_PUSH_DUTY);
    }
    CHECK (!drive.b_loaded);
}

/* The vote and window counts stop at UINT16_MAX instead of wrapping. */
static void test_drive_counts_saturate (void)
{
    drive_t    drive;
    drive_in_t in;

    /* An agreeing vote near the top. */
    health_drive_init (&drive);
    drive.agree_ms = (uint16_t) (UINT16_MAX - 50U);
    (void) drive_step (&drive, 163000, 0, 150000, 30000, 400, 0);
    CHECK (UINT16_MAX == drive.agree_ms);

    /* Swapped plugs: the pair disagrees. */
    health_drive_init (&drive);
    drive.disagree_ms = (uint16_t) (UINT16_MAX - 50U);
    (void) drive_step (&drive, 163000, 0, 0, 163000, 400, 0);
    CHECK (UINT16_MAX == drive.disagree_ms);

    /* A wheel turns against its command. */
    health_drive_init (&drive);
    drive.disagree_ms = (uint16_t) (UINT16_MAX - 50U);
    (void) drive_step (&drive, 163000, 163000, -160000, 160000, 400, 400);
    CHECK (UINT16_MAX == drive.disagree_ms);

    /* One step longer than the counts can hold still closes and judges the window. */
    health_drive_init (&drive);
    in.target_um_s[0] = 163000;
    in.target_um_s[1] = 0;
    in.speed_um_s[0]  = 0;
    in.speed_um_s[1]  = 163000;
    in.duty_pm[0]     = 400;
    in.duty_pm[1]     = 0;
    in.b_judge        = true;
    CHECK (health_drive_update (&drive, &in, (uint32_t) UINT16_MAX + 101U));
    CHECK (drive.b_crossed);
}

static void test_drive_free_wheels (void)
{
    drive_t  drive;
    unsigned idx = 0U;

    health_drive_init (&drive);
    /* Before the baseline is learned, a light wheel proves nothing. */
    for (idx = 0U; idx < 5U; idx++)
    {
        (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 100);
    }
    CHECK (!drive.b_free);

    /* Learn on the ground: 400 permille at 160 mm/s. */
    health_drive_init (&drive);
    for (idx = 0U; idx < 40U; idx++)
    {
        (void) drive_step (&drive, 163000, 163000, 160000, 160000, 400, 400);
    }
    CHECK (!drive.b_free);

    /* In the air: on target with a quarter of the duty. Sets after 300 ms. */
    (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 100);
    (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 100);
    CHECK (!drive.b_free);
    (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 100);
    CHECK (drive.b_free);

    /* One wheel loaded again: the cue clears after two checks. */
    (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 400);
    CHECK (drive.b_free);
    (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 400);
    CHECK (!drive.b_free);

    /* A wheel that lags the target is not free; it is loaded. */
    health_drive_init (&drive);
    for (idx = 0U; idx < 40U; idx++)
    {
        (void) drive_step (&drive, 163000, 163000, 160000, 160000, 400, 400);
    }
    for (idx = 0U; idx < 10U; idx++)
    {
        (void) drive_step (&drive, 163000, 163000, 100000, 100000, 100, 100);
    }
    CHECK (!drive.b_free);
}

static void test_drive_probe (void)
{
    CHECK (DRIVE_PROBE_OK == health_drive_probe_verdict (160, 1));
    CHECK (DRIVE_PROBE_DEAD == health_drive_probe_verdict (1, 2));
    CHECK (DRIVE_PROBE_REVERSED == health_drive_probe_verdict (-150, 0));
    CHECK (DRIVE_PROBE_CROSSED == health_drive_probe_verdict (2, 150));
    CHECK (DRIVE_PROBE_UNCLEAR == health_drive_probe_verdict (120, 80));
    CHECK (DRIVE_PROBE_UNCLEAR == health_drive_probe_verdict (10, 0));
}

static edge_obstacle_in_t obs (uint8_t state)
{
    edge_obstacle_in_t in = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

    in.state = state;
    return in;
}

static void test_edge_obstacle_cycle (void)
{
    edge_watch_t watch;
    edge_event_t ev[EDGE_EVENTS_MAX];
    edge_obstacle_in_t in;

    edge_init (&watch);
    in = obs (TELEMETRY_OBSTACLE_STATE_MONITOR);
    CHECK (0U == edge_obstacle (&watch, &in, ev));

    in = obs (TELEMETRY_OBSTACLE_STATE_WAIT_STOP);
    in.distance_mm = 245U;
    CHECK (1U == edge_obstacle (&watch, &in, ev));
    CHECK ((TELEMETRY_EVT_OBSTACLE_SEEN == ev[0].type) && (245 == ev[0].p1));

    in = obs (TELEMETRY_OBSTACLE_STATE_SCANNING);
    CHECK (0U == edge_obstacle (&watch, &in, ev));

    in = obs (TELEMETRY_OBSTACLE_STATE_PLAN);
    in.plan_status = 0U;
    in.plan_width_mm = 190U;
    CHECK (1U == edge_obstacle (&watch, &in, ev));
    CHECK ((TELEMETRY_EVT_SCAN_DONE == ev[0].type) && (190U == ev[0].p2));
    /* Staying in PLAN reports nothing more. */
    CHECK (0U == edge_obstacle (&watch, &in, ev));

    in = obs (TELEMETRY_OBSTACLE_STATE_BYPASS);
    in.plan_side = 1U;
    in.plan_start_deg = 40U;
    CHECK (1U == edge_obstacle (&watch, &in, ev));
    CHECK ((TELEMETRY_EVT_BYPASS_START == ev[0].type) && (1 == ev[0].p1) && (40U == ev[0].p2));

    in = obs (TELEMETRY_OBSTACLE_STATE_REACQUIRE);
    CHECK (0U == edge_obstacle (&watch, &in, ev));
    in = obs (TELEMETRY_OBSTACLE_STATE_MONITOR);
    CHECK (1U == edge_obstacle (&watch, &in, ev));
    CHECK (TELEMETRY_EVT_BYPASS_END == ev[0].type);
}

static void test_edge_obstacle_stop_and_clear (void)
{
    edge_watch_t watch;
    edge_event_t ev[EDGE_EVENTS_MAX];
    edge_obstacle_in_t in;

    edge_init (&watch);
    in = obs (TELEMETRY_OBSTACLE_STATE_MONITOR);
    (void) edge_obstacle (&watch, &in, ev);
    /* If polling skipped PLAN, the bypass snapshot still reports the plan. */
    in = obs (TELEMETRY_OBSTACLE_STATE_WAIT_STOP);
    (void) edge_obstacle (&watch, &in, ev);
    in = obs (TELEMETRY_OBSTACLE_STATE_BYPASS);
    CHECK (2U == edge_obstacle (&watch, &in, ev));
    CHECK ((TELEMETRY_EVT_BYPASS_START == ev[0].type) && (TELEMETRY_EVT_SCAN_DONE == ev[1].type));

    in = obs (TELEMETRY_OBSTACLE_STATE_STOPPED);
    in.cause = 7U;
    in.echo_class = 2U;
    CHECK (1U == edge_obstacle (&watch, &in, ev));
    CHECK ((TELEMETRY_EVT_OBSTACLE_STOP == ev[0].type) && (7 == ev[0].p1) && (2U == ev[0].p2));
    CHECK (0U == edge_obstacle (&watch, &in, ev));
}

static void test_edge_run (void)
{
    edge_watch_t watch;
    edge_watch_t fresh;
    edge_event_t ev[EDGE_EVENTS_MAX];

    edge_init (&watch);
    edge_init (&fresh);
    CHECK (0U == edge_run (&fresh, RUN_STATE_STOPPED, ev));
    CHECK (0U == edge_run (&watch, RUN_STATE_BOOT, ev));
    CHECK (1U == edge_run (&watch, RUN_STATE_RUNNING, ev));
    CHECK (TELEMETRY_EVT_RUN_START == ev[0].type);
    CHECK (0U == edge_run (&watch, RUN_STATE_RUNNING, ev));
    CHECK (1U == edge_run (&watch, RUN_STATE_STOPPED, ev));
    CHECK ((TELEMETRY_EVT_RUN_STOP == ev[0].type) && (RUN_STATE_STOPPED == ev[0].p1));
}

static void test_recalibrate (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx);
    run_state_tick (&ctx, &in, &out);

    /* START on level ground calibrates again; on a hump it keeps the old reference. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    in.b_on_hump = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && !out.b_physics_recal);

    /* RECAL: refused on a hump, while lifted, from a barcode, after an e-stop. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_ON_HUMP == out.reason));
    in.b_on_hump = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_BARCODE),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    in.b_lifted = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    in.b_lifted = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_accepted && out.b_physics_recal);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK ((MOTION_HOLD_OPERATOR | MOTION_HOLD_LIFT) == out.hold_mask);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RECAL, VEHICLE_SRC_TELEMETRY),
                       &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
}

static void test_calhold (void)
{
    calhold_t hold;
    unsigned  idx = 0U;

    /* Boot: held until the first calibration. */
    calhold_start (&hold, 0U);
    CHECK (calhold_update (&hold, 0U, false));
    CHECK (!calhold_update (&hold, 0U, true));
    CHECK (!hold.b_timed_out);

    /* An unrelated calibration or a delayed task cannot satisfy this START. */
    calhold_start (&hold, 17U);
    CHECK (calhold_update (&hold, 16U, false));
    for (idx = 0U; idx <= CALHOLD_MAX_PASSES; idx++)
    {
        CHECK (calhold_update (&hold, 16U, true));
    }
    CHECK (hold.b_timed_out);
    CHECK (calhold_update (&hold, 17U, false));
    CHECK (!calhold_update (&hold, 17U, true));

    /* No calibration at all: the timeout is reported and the hold stays on
       until a calibration comes. A new request starts a fresh attempt. */
    calhold_start (&hold, 0U);
    for (idx = 0U; idx < CALHOLD_MAX_PASSES; idx++)
    {
        CHECK (calhold_update (&hold, 0U, false));
    }
    CHECK (hold.b_timed_out);
    for (idx = 0U; idx < (2U * CALHOLD_MAX_PASSES); idx++)
    {
        CHECK (calhold_update (&hold, 0U, false));
    }
    CHECK (!calhold_update (&hold, 0U, true));

    calhold_start (&hold, 1U);
    CHECK (!hold.b_timed_out);
    CHECK (calhold_update (&hold, 0U, false));
}

/* The hold covers a calibration the supervisor asked for. A reference physics
   drops later in a run (a lasting IMU read failure) never holds the car. */
static void test_calhold_ignores_a_lost_reference (void)
{
    calhold_t hold;
    unsigned  idx = 0U;

    calhold_start (&hold, 0U);
    CHECK (!calhold_update (&hold, 0U, true));
    for (idx = 0U; idx < (2U * CALHOLD_MAX_PASSES); idx++)
    {
        CHECK (!calhold_update (&hold, 0U, false));
    }
    CHECK (!hold.b_timed_out);
}

/* The timeout is reported once per attempt, on the pass it is first seen. */
static void test_calhold_timeout_edge (void)
{
    calhold_t hold;
    unsigned  idx     = 0U;
    unsigned  attempt = 0U;

    for (attempt = 0U; attempt < 2U; attempt++)
    {
        calhold_start (&hold, 0U);
        CHECK (!calhold_timeout_edge (&hold));
        for (idx = 0U; idx < (CALHOLD_MAX_PASSES - 1U); idx++)
        {
            CHECK (calhold_update (&hold, 0U, false));
            CHECK (!calhold_timeout_edge (&hold));
        }
        CHECK (calhold_update (&hold, 0U, false));
        CHECK (calhold_timeout_edge (&hold));
        CHECK (calhold_update (&hold, 0U, false));
        CHECK (!calhold_timeout_edge (&hold));
        CHECK (!calhold_timeout_edge (&hold));
    }
}

/* A stop by request can carry a timing fault already raised by the controller. */
static void test_motion_timing_fault (void)
{
    CHECK (health_motion_timing_fault (MOTION_STATE_FAULT, TELEMETRY_MOTION_FAULT_EXPIRED));
    CHECK (health_motion_timing_fault (MOTION_STATE_FAULT, TELEMETRY_MOTION_FAULT_GAP));
    CHECK (!health_motion_timing_fault (MOTION_STATE_FAULT, TELEMETRY_MOTION_FAULT_FEEDBACK));
    CHECK (!health_motion_timing_fault (MOTION_STATE_FAULT, TELEMETRY_MOTION_FAULT_NO_EDGES));
    CHECK (!health_motion_timing_fault (MOTION_STATE_FAULT, TELEMETRY_MOTION_FAULT_COMMAND));
    CHECK (!health_motion_timing_fault (MOTION_STATE_FAULT, TELEMETRY_MOTION_FAULT_OUTPUT));
    CHECK (health_motion_timing_fault (MOTION_STATE_STOPPED, TELEMETRY_MOTION_FAULT_EXPIRED));
    CHECK (health_motion_timing_fault (MOTION_STATE_STOPPED, TELEMETRY_MOTION_FAULT_GAP));
    CHECK (!health_motion_timing_fault (MOTION_STATE_STOPPED, TELEMETRY_MOTION_FAULT_NONE));
    CHECK (!health_motion_timing_fault (MOTION_STATE_STOPPED, TELEMETRY_MOTION_FAULT_OUTPUT));
    CHECK (!health_motion_timing_fault (MOTION_STATE_RUNNING, TELEMETRY_MOTION_FAULT_GAP));
}

#define TIMING_PASS_MS (20U)

/* A timing fault that clears inside its window costs no reboot, until too
   many have come since boot. One that stays past the window reboots. */
static void test_motion_timing_watch (void)
{
    timing_watch_t watch;
    uint32_t       elapsed = 0U;

    timing_watch_init (&watch);
    for (unsigned episode = 0U; episode < MOTION_TIMING_FAULTS_MAX; episode++)
    {
        for (elapsed = 0U; elapsed < MOTION_TIMING_RECOVERY_MS; elapsed += TIMING_PASS_MS)
        {
            CHECK (!timing_watch_update (&watch, true, TIMING_PASS_MS));
        }
        CHECK (!timing_watch_update (&watch, false, TIMING_PASS_MS));
    }
    CHECK (timing_watch_update (&watch, true, TIMING_PASS_MS));
    CHECK (timing_watch_update (&watch, false, TIMING_PASS_MS));

    timing_watch_init (&watch);
    for (elapsed = 0U; elapsed < MOTION_TIMING_RECOVERY_MS; elapsed += TIMING_PASS_MS)
    {
        CHECK (!timing_watch_update (&watch, true, TIMING_PASS_MS));
    }
    CHECK (timing_watch_update (&watch, true, TIMING_PASS_MS));
    CHECK (timing_watch_update (&watch, false, TIMING_PASS_MS));
}

typedef struct
{
    const char * name;
    void (*run) (void);
} test_case_t;

#define CASE(name) { #name, test_ ## name }

static const test_case_t cases[] = {
    CASE(ring_order_and_source),
    CASE(ring_full_keeps_stops),
    CASE(ring_keeps_ids),
    CASE(boot_and_stop_start),
    CASE(reset_while_running_holds_until_start),
    CASE(boot_holds_until_start),
    CASE(reboot_holds_until_start),
    CASE(stop_wins_in_a_batch),
    CASE(turn_rules),
    CASE(barcode_stop),
    CASE(start_refused_when_overturned),
    CASE(emergency_stop_latches),
    CASE(lift_and_overturn),
    CASE(reset_in_a_lift_waits_for_start),
    CASE(stall_holds_until_start),
    CASE(reset_refused_upside_down_without_tick),
    CASE(attitude_unknown),
    CASE(attitude_unknown_keeps_lift_hold),
    CASE(start_refused_while_permit_lapsed),
    CASE(permit_lapse_rule),
    CASE(permit_skipped_rule),
    CASE(start_reset_counts),
    CASE(physics_inputs),
    CASE(running_car_is_not_held_by_time),
    CASE(obstacle_post_retry),
    CASE(unknown_opcode),
    CASE(liveness),
    CASE(motion_timing_fault),
    CASE(motion_timing_watch),
    CASE(liveness_boot_allowance),
    CASE(liveness_kick_range),
    CASE(liveness_wait_fits_budget),
    CASE(progress),
    CASE(stuck_sensor),
    CASE(drive_crossed),
    CASE(drive_loaded),
    CASE(drive_counts_saturate),
    CASE(drive_free_wheels),
    CASE(drive_probe),
    CASE(recalibrate),
    CASE(calhold),
    CASE(calhold_ignores_a_lost_reference),
    CASE(calhold_timeout_edge),
    CASE(edge_obstacle_cycle),
    CASE(edge_obstacle_stop_and_clear),
    CASE(edge_run),
    CASE(remote_stop_after_silent_run),
    CASE(single_estop_sample_survives_release),
    CASE(lift_reading_holds_on_first_tick),
};

int main (int argc, char ** argv)
{
    size_t idx = 0U;

    if (2 == argc)
    {
        for (idx = 0U; idx < (sizeof (cases) / sizeof (cases[0])); idx++)
        {
            if (0 == strcmp (argv[1], cases[idx].name))
            {
                cases[idx].run ();
                return (g_fail == 0) ? 0 : 1;
            }
        }

        (void)fprintf (stderr, "Unknown case: %s\n", argv[1]);
        return 2;
    }
    if (1 != argc)
    {
        (void)fprintf (stderr, "Usage: %s [case]\n", argv[0]);
        return 2;
    }
    for (idx = 0U; idx < (sizeof (cases) / sizeof (cases[0])); idx++)
    {
        cases[idx].run ();
    }

    if (g_fail != 0U)
    {
        (void)printf ("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf ("test_supervisor: all checks passed\n");
    return 0;
}
