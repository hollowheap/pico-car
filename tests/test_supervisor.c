/*
 *	test_supervisor.c
 *	Host tests for the command ring, the run state and the health checks.
 */

#include <stdint.h>
#include <stdio.h>

#include "command_ring.h"
#include "edge_events.h"
#include "health_core.h"
#include "telemetry/wire_schema.h"
#include "run_state.h"
#include "motion/motion.h"

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

static void test_ring_order_and_source (void)
{
    command_ring_t ring;
    vehicle_cmd_t  cmd;

    command_ring_init (&ring);
    CHECK (!command_ring_pop (&ring, &cmd));
    CHECK (command_ring_push (&ring, VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE));
    CHECK (command_ring_push (&ring, VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY));
    CHECK (command_ring_push (&ring, VEHICLE_CMD_START, VEHICLE_SRC_BUTTON));

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
    CHECK (command_ring_push (&ring, VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY));
    for (idx = 1U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_push (&ring, VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY));
    }
    CHECK (COMMAND_RING_LEN == ring.count);

    /* Full: a new command removes the oldest command that is not a stop. */
    CHECK (command_ring_push (&ring, VEHICLE_CMD_RIGHT, VEHICLE_SRC_TELEMETRY));
    CHECK (1U == ring.dropped);
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK (VEHICLE_CMD_STOP == cmd.opcode);
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK (VEHICLE_CMD_LEFT == cmd.opcode);

    /* A ring of only stops drops the new command, never a stop. */
    command_ring_init (&ring);
    for (idx = 0U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_push (&ring, VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY));
    }
    CHECK (!command_ring_push (&ring, VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY));
    CHECK (1U == ring.dropped);
    CHECK (COMMAND_RING_LEN == ring.count);

    /* A stop pushed into a full ring of non-stops is kept. */
    command_ring_init (&ring);
    for (idx = 0U; idx < COMMAND_RING_LEN; idx++)
    {
        CHECK (command_ring_push (&ring, VEHICLE_CMD_UTURN, VEHICLE_SRC_TELEMETRY));
    }
    CHECK (command_ring_push (&ring, VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY));
    for (idx = 0U; idx < (COMMAND_RING_LEN - 1U); idx++)
    {
        CHECK (command_ring_pop (&ring, &cmd));
    }
    CHECK (command_ring_pop (&ring, &cmd));
    CHECK (VEHICLE_CMD_STOP == cmd.opcode);
}

static run_inputs_t follow_inputs (void)
{
    run_inputs_t in = { .b_follow = true };

    return in;
}

static vehicle_cmd_t cmd_of (uint8_t opcode, uint8_t source)
{
    vehicle_cmd_t cmd;

    cmd.opcode = opcode;
    cmd.source = source;
    return cmd;
}

static void test_boot_and_stop_start (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    CHECK (RUN_STATE_BOOT == run_state_of (&ctx));
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_stop && out.b_cancel_turn);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_reset && !out.b_obstacle_start);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx)); /* RESET never releases a hold */
    CHECK (MOTION_HOLD_OPERATOR == out.hold_mask);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_reset && out.b_obstacle_start);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
}

static void test_stop_wins_in_a_batch (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (VEHICLE_CMD_LEFT == out.turn);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_cancel_turn);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_NOT_RUNNING == out.reason));
    CHECK (RUN_TURN_NONE == out.turn);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
}

static void test_turn_rules (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RIGHT, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_RIGHT == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_UTURN, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_UTURN == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STRAIGHT, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_STRAIGHT == out.turn));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STRAIGHT, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && (VEHICLE_CMD_STRAIGHT == out.turn));

    /* Busy: a bypass, a scan or a turn in progress. */
    in.b_follow = false;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_LEFT, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_BUSY == out.reason));
    CHECK (RUN_TURN_NONE == out.turn);
}

static void test_barcode_stop (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_stop);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    CHECK (MOTION_HOLD_BARCODE == out.hold_mask);

    /* A barcode cannot start, reset or emergency-stop the car. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_BARCODE), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_SOURCE == out.reason));
    CHECK (!out.b_motion_estop);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));

    /* The dashboard START releases the barcode hold. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
    CHECK (0U == out.hold_mask);
}

static void test_emergency_stop_latches (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_ESTOP, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && out.b_motion_estop && out.b_obstacle_stop);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));

    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));

    /* The motion latch seen on a tick latches the state too (the STOP pin). */
    run_state_init (&ctx, false);
    in.b_estop = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
}

static void test_motion_fault_stops_mission (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));

    /* A command arriving just before the next supervisor tick is also refused. */
    in.b_motion_fault = true;
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RIGHT, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));

    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    CHECK (out.b_obstacle_stop && out.b_cancel_turn);
    CHECK (!out.b_motion_estop);

    in.b_motion_fault = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BUTTON), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_STOP, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_stop);
}

static void test_lift_and_overturn (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);

    /* Lifted: the lift hold goes on at once. */
    in.b_lifted = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    CHECK (MOTION_HOLD_LIFT == out.hold_mask);
    CHECK (out.b_obstacle_stop && out.b_cancel_turn);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LIFTED == out.reason));

    /* Set down: the hold stays until START. */
    in.b_lifted = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && (0U == out.hold_mask));
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));

    /* Overturned: latched, the obstacle task is stopped once, START refused. */
    in.b_overturned = true;
    in.b_lifted     = true;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    CHECK (out.b_obstacle_stop);
    run_state_tick (&ctx, &in, &out);
    CHECK (!out.b_obstacle_stop);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));

    /* RESET while still on its side is refused. Upright, it clears the latch
       but keeps the hold; START then runs. */
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_rejected);
    in.b_overturned = false;
    in.b_lifted     = false;
    run_state_tick (&ctx, &in, &out);
    CHECK (RUN_STATE_LATCHED == run_state_of (&ctx));
    CHECK (MOTION_HOLD_LIFT == out.hold_mask);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_BUTTON), &in, &out);
    CHECK (out.b_rejected && (RUN_REJECT_LATCHED == out.reason));
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_RESET, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted && out.b_obstacle_reset);
    CHECK (out.b_physics_recal); /* the IMU calibrates again after an overturn */
    CHECK (RUN_STATE_HELD == run_state_of (&ctx));
    CHECK (MOTION_HOLD_LIFT == out.hold_mask);
    run_state_command (&ctx, cmd_of (VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY), &in, &out);
    CHECK (out.b_accepted);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

static void test_unknown_opcode (void)
{
    run_ctx_t    ctx;
    run_result_t out;
    run_inputs_t in = follow_inputs ();

    run_state_init (&ctx, false);
    run_state_tick (&ctx, &in, &out);
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
    return drive_update (p_drive, &in, 100U);
}

static void test_drive_crossed (void)
{
    drive_t  drive;
    unsigned idx   = 0U;
    unsigned fired = 0U;

    /* A good car: the faster-commanded wheel turns faster, with some lag. */
    drive_init (&drive);
    for (idx = 0U; idx < 100U; idx++)
    {
        fired += drive_step (&drive, 163000, 0, 150000, 30000, 400, 0) ? 1U : 0U;
        fired += drive_step (&drive, 0, 163000, 20000, 150000, 0, 400) ? 1U : 0U;
    }
    CHECK (0U == fired);
    CHECK (!drive.b_crossed);

    /* Swapped plugs: the left command moves the right encoder. */
    drive_init (&drive);
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
    drive_init (&drive);
    fired = 0U;
    for (idx = 0U; idx < 40U; idx++)
    {
        fired += drive_step (&drive, 163000, 163000, -160000, 160000, 400, 400) ? 1U : 0U;
    }
    CHECK (1U == fired);
}

static void test_drive_free_wheels (void)
{
    drive_t  drive;
    unsigned idx = 0U;

    drive_init (&drive);
    /* Before the baseline is learned, a light wheel proves nothing. */
    for (idx = 0U; idx < 5U; idx++)
    {
        (void) drive_step (&drive, 163000, 163000, 160000, 160000, 100, 100);
    }
    CHECK (!drive.b_free);

    /* Learn on the ground: 400 permille at 160 mm/s. */
    drive_init (&drive);
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
    drive_init (&drive);
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
    CHECK (DRIVE_PROBE_OK == drive_probe_verdict (160, 1));
    CHECK (DRIVE_PROBE_DEAD == drive_probe_verdict (1, 2));
    CHECK (DRIVE_PROBE_REVERSED == drive_probe_verdict (-150, 0));
    CHECK (DRIVE_PROBE_CROSSED == drive_probe_verdict (2, 150));
    CHECK (DRIVE_PROBE_UNCLEAR == drive_probe_verdict (120, 80));
    CHECK (DRIVE_PROBE_UNCLEAR == drive_probe_verdict (10, 0));
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
    /* A clear path skips PLAN: the plan is reported when the bypass starts. */
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

static void test_edge_hump_and_run (void)
{
    edge_watch_t watch;
    edge_event_t ev[EDGE_EVENTS_MAX];

    edge_init (&watch);
    CHECK (0U == edge_hump (&watch, false, 0U, false, ev));
    CHECK (1U == edge_hump (&watch, true, 0U, false, ev));
    CHECK (TELEMETRY_EVT_HUMP_START == ev[0].type);
    CHECK (0U == edge_hump (&watch, true, 0U, false, ev));
    CHECK (1U == edge_hump (&watch, false, 52U, true, ev));
    CHECK ((TELEMETRY_EVT_HUMP_END == ev[0].type) && (52 == ev[0].p1) && (1U == ev[0].p2));

    CHECK (0U == edge_run (&watch, RUN_STATE_BOOT, ev));
    CHECK (1U == edge_run (&watch, RUN_STATE_RUNNING, ev));
    CHECK (TELEMETRY_EVT_RUN_START == ev[0].type);
    CHECK (0U == edge_run (&watch, RUN_STATE_RUNNING, ev));
    CHECK (1U == edge_run (&watch, RUN_STATE_STOPPED, ev));
    CHECK ((TELEMETRY_EVT_RUN_STOP == ev[0].type) && (RUN_STATE_STOPPED == ev[0].p1));
}

static void test_button_start_holds_until_local_press (void)
{
    run_ctx_t ctx;
    run_inputs_t in = {0};
    run_result_t result;
    vehicle_cmd_t cmd = {VEHICLE_CMD_START, VEHICLE_SRC_TELEMETRY};

    run_state_init (&ctx, true);
    CHECK (MOTION_HOLD_OPERATOR == run_state_hold_mask (&ctx));
    run_state_tick (&ctx, &in, &result);
    CHECK (RUN_STATE_STOPPED == run_state_of (&ctx));
    CHECK (!result.b_obstacle_start && !result.b_obstacle_reset);

    run_state_command (&ctx, cmd, &in, &result);
    CHECK (result.b_rejected && ctx.b_wait_button);
    CHECK (MOTION_HOLD_OPERATOR == result.hold_mask);

    cmd.source = VEHICLE_SRC_BUTTON;
    in.b_lifted = true;
    run_state_command (&ctx, cmd, &in, &result);
    CHECK (result.b_rejected && ctx.b_wait_button);

    in.b_lifted = false;
    run_state_command (&ctx, cmd, &in, &result);
    CHECK (result.b_accepted && !ctx.b_wait_button);
    CHECK (result.b_obstacle_start && result.b_obstacle_reset);
    CHECK (0U == result.hold_mask);
    CHECK (RUN_STATE_RUNNING == run_state_of (&ctx));
}

int main (void)
{
    test_ring_order_and_source ();
    test_ring_full_keeps_stops ();
    test_boot_and_stop_start ();
    test_button_start_holds_until_local_press ();
    test_stop_wins_in_a_batch ();
    test_turn_rules ();
    test_barcode_stop ();
    test_emergency_stop_latches ();
    test_motion_fault_stops_mission ();
    test_lift_and_overturn ();
    test_unknown_opcode ();
    test_liveness ();
    test_liveness_boot_allowance ();
    test_progress ();
    test_stuck_sensor ();
    test_drive_crossed ();
    test_drive_free_wheels ();
    test_drive_probe ();
    test_edge_obstacle_cycle ();
    test_edge_obstacle_stop_and_clear ();
    test_edge_hump_and_run ();

    if (g_fail != 0U)
    {
        (void)printf ("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf ("test_supervisor: all checks passed\n");
    return 0;
}
