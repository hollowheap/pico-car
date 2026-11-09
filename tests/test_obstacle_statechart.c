/** @file test_statechart.c
 *
 * @brief Host tests for the Buddy 5 obstacle statechart.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "obstacle_config.h"
#include "statechart.h"
#include "obstacle_pack.h"
#include "planner.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static uint8_t cmd(uint32_t type, uint32_t id)
{
    return obstacle_pack_cmd(type, id);
}

static void start_monitor(obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out)
{
    obstacle_statechart_out_clear(p_out);
    obstacle_statechart_command(p_fsm, cmd(OBSTACLE_CMD_RESET, 0U), 0U, 0U, p_out);
    obstacle_statechart_command(p_fsm, cmd(OBSTACLE_CMD_START, 0U), 1U, 0U, p_out);
}

static void test_init_and_status(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t cfg;
    uint32_t     word;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_BOOT);
    CHECK(fsm.faults == 0U);
    CHECK(!fsm.b_reset_ok);
    CHECK(obstacle_statechart_servo_deci(&fsm) == 0);

    word = obstacle_statechart_status(&fsm);
    CHECK(obstacle_pack_get(word, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK) ==
          OBSTACLE_STATE_STOPPED);
    CHECK(obstacle_pack_get(word, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK) ==
          OBSTACLE_CAUSE_BOOT);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_RESET, 0U), 10U, 0U, &out);
    CHECK(fsm.b_reset_ok);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_START, 0U), 20U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);

    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_STOP, 0U), 30U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_EXTERNAL);
    CHECK(out.evt_count == 1U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK) ==
          OBSTACLE_EVT_FAULT);
    CHECK(!out.b_call_stop);
}

static void test_obstacle_to_scan(void)
{
    obstacle_statechart_t      fsm;
    obstacle_statechart_out_t  out;
    obstacle_plan_cfg_t cfg;
    uint32_t      guard = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_WAIT_STOP);
    CHECK(out.evt_count == 1U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK) ==
          OBSTACLE_EVT_OBSTACLE);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK) == 200U);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 3U), 30U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_SCANNING);
    CHECK(out.b_profile_changed);
    CHECK(obstacle_statechart_servo_deci(&fsm) == -600);

    while ((fsm.state == OBSTACLE_STATE_SCANNING) && (guard < 100U))
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 40U, &out);
        if (fsm.state == OBSTACLE_STATE_SCANNING)
        {
            obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 50U, &out);
        }
        guard++;
    }
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    CHECK(fsm.plan.status == OBSTACLE_RESULT_CLEAR);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_ACCEPT, 3U), 60U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
}

static void test_no_echo_fault(void)
{
    obstacle_statechart_t      fsm;
    obstacle_statechart_out_t  out;
    obstacle_plan_cfg_t cfg;
    uint8_t       i;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);

    obstacle_statechart_out_clear(&out);
    for (i = 0U; i < OBSTACLE_NO_ECHO_COUNT; i++)
    {
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_UNKNOWN, 0U, (uint32_t)i, &out);
    }
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK((fsm.faults & OBSTACLE_FAULT_NO_ECHO) != 0U);
    CHECK(out.b_call_stop);
    CHECK(out.evt_count > 0U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK) ==
          OBSTACLE_CAUSE_NO_ECHO);
}

static void test_handover_timeout(void)
{
    obstacle_statechart_t      fsm;
    obstacle_statechart_out_t  out;
    obstacle_plan_cfg_t cfg;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_WAIT_STOP);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, 20U + OBSTACLE_WAIT_STOP_MS + 1U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_HANDOVER);
    CHECK((fsm.faults & OBSTACLE_FAULT_HANDOVER) != 0U);
    CHECK(out.b_call_stop);
}

static void test_scan_stall(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_WAIT_STOP);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 1U), 30U, 0U,
                                &out);

    /* A scan that keeps settling slots is not cut off by total time. */
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 9000U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 9060U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_SCANNING);
    obstacle_statechart_tick(&fsm, 11000U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_SCANNING);

    /* A scan that settles nothing for OBSTACLE_SCAN_STALL_MS is hung. */
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, 9060U + OBSTACLE_SCAN_STALL_MS + 1U, 0U,
                             &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_SCAN_TIMEOUT);
    CHECK((fsm.faults & OBSTACLE_FAULT_SCAN_TIMEOUT) != 0U);
    CHECK(out.b_call_stop);
}

static void test_wall_reaches_bypass(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;
    uint32_t                  guard = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_WAIT_STOP);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 5U), 30U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_SCANNING);

    /* A wall: every slot reads 300 mm, so no far edge exists. */
    while ((fsm.state == OBSTACLE_STATE_SCANNING) && (guard < 100U))
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 300U, 40U, &out);
        if (fsm.state == OBSTACLE_STATE_SCANNING)
        {
            obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 300U, 50U, &out);
        }
        guard++;
    }
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    CHECK(fsm.cause != OBSTACLE_CAUSE_NO_SAFE_PATH);
    CHECK(fsm.plan.status == OBSTACLE_RESULT_OK);
    CHECK(fsm.plan.side == OBSTACLE_SIDE_LEFT);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_ACCEPT, 5U), 60U, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    CHECK(fsm.hug_limit_mm == obstacle_plan_hug_limit_mm(&fsm.plan, &cfg));
    CHECK(fsm.hug_limit_mm == 3841U);
    CHECK(fsm.steer == obstacle_plan_steer_away(OBSTACLE_SIDE_LEFT));
    CHECK(!out.b_call_stop);
}


/* ---- D95: moving forward guard, pivot exit, wall budget ---------------- */

#define TEST_ID (5U)

/* Run a scan from MONITOR to PLAN: every slot reads 300 mm for a wall (no
 * far edge), or only slots 9..15 do for a bounded box. */
static void scan_to_plan(obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out,
                         bool b_wall)
{
    uint32_t guard = 0U;

    start_monitor(p_fsm, p_out);
    obstacle_statechart_out_clear(p_out);
    obstacle_statechart_ping(p_fsm, OBSTACLE_CLASS_VALID, 200U, 10U, p_out);
    obstacle_statechart_ping(p_fsm, OBSTACLE_CLASS_VALID, 200U, 20U, p_out);
    obstacle_statechart_command(p_fsm, cmd(OBSTACLE_CMD_SCAN_REQ, TEST_ID), 30U, 0U, p_out);

    while ((p_fsm->state == OBSTACLE_STATE_SCANNING) && (guard < 200U))
    {
        uint8_t const slot = scan_algo_target(&p_fsm->scan);
        bool const    b_hit = b_wall || ((slot >= 9U) && (slot <= 15U));

        uint8_t const cls   = b_hit ? (uint8_t)OBSTACLE_CLASS_VALID : (uint8_t)OBSTACLE_CLASS_OPEN;
        uint16_t const dist = b_hit ? (uint16_t)300U : (uint16_t)0U;

        obstacle_statechart_out_clear(p_out);
        obstacle_statechart_ping(p_fsm, cls, dist, 40U, p_out);
        if (p_fsm->state == OBSTACLE_STATE_SCANNING)
        {
            obstacle_statechart_ping(p_fsm, cls, dist, 50U, p_out);
        }
        guard++;
    }
}

/* To BYPASS with the given plan. */
static void to_bypass(obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out,
                      bool b_wall)
{
    obstacle_plan_cfg_t cfg;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(p_fsm, &cfg, 0U);
    scan_to_plan(p_fsm, p_out, b_wall);
    CHECK(p_fsm->state == OBSTACLE_STATE_PLAN);
    obstacle_statechart_out_clear(p_out);
    obstacle_statechart_command(p_fsm, cmd(OBSTACLE_CMD_ACCEPT, TEST_ID), 60U, 1000U, p_out);
    CHECK(p_fsm->state == OBSTACLE_STATE_BYPASS);
}

static void turned(obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out,
                   uint32_t now_ms)
{
    obstacle_statechart_command(p_fsm, cmd(OBSTACLE_CMD_TURNED, TEST_ID), now_ms, 1000U, p_out);
}

/* BYPASS -> REACQUIRE: turned, then two clear forward pings. */
static void to_reacquire(obstacle_statechart_t * p_fsm, obstacle_statechart_out_t * p_out,
                         bool b_wall)
{
    to_bypass(p_fsm, p_out, b_wall);
    turned(p_fsm, p_out, 100U);
    obstacle_statechart_ping(p_fsm, OBSTACLE_CLASS_OPEN, 0U, 200U, p_out);
    obstacle_statechart_ping(p_fsm, OBSTACLE_CLASS_OPEN, 0U, 260U, p_out);
    CHECK(p_fsm->state == OBSTACLE_STATE_REACQUIRE);
}

/* Wall flag travels from the plan to the statechart and the SCAN_RESULT event. */
static void test_wall_flag(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    CHECK(!fsm.plan.b_wall);

    scan_to_plan(&fsm, &out, true);
    CHECK(fsm.plan.b_wall);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_WALL_SH, OBSTACLE_EV_WALL_MK) == 1U);

    obstacle_statechart_init(&fsm, &cfg, 0U);
    scan_to_plan(&fsm, &out, false);
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    CHECK(fsm.plan.status == OBSTACLE_RESULT_OK);
    CHECK(!fsm.plan.b_wall);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_WALL_SH, OBSTACLE_EV_WALL_MK) == 0U);
}

/* Servo target per state: BYPASS is always ahead; REACQUIRE aims at the side
 * angle on three pings and ahead on every fourth (seq 3, 7, 11 ...). */
static void test_servo_schedule(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_bypass(&fsm, &out, false);
    for (ping = 0U; ping < 12U; ping++)
    {
        CHECK(obstacle_statechart_servo_deci(&fsm) == 0);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 290U, 100U + ping, &out);
        CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    }

    to_reacquire(&fsm, &out, false);
    CHECK(fsm.plan.side == OBSTACLE_SIDE_LEFT);
    for (ping = 0U; ping < 24U; ping++)
    {
        int16_t const servo    = obstacle_statechart_servo_deci(&fsm);
        uint32_t      seq_next = ((uint32_t)fsm.seq + 1U) & OBSTACLE_ST_SEQ_MK;

        if (3U == (seq_next % 4U))
        {
            CHECK(servo == 0);
        }
        else
        {
            CHECK(servo == -OBSTACLE_HUG_SERVO_DECI);
        }
        /* A far, harmless reading on either ray. */
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + (ping * 10U), &out);
        CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    }

    /* Exactly one look ahead in every four pings. */
    {
        uint32_t ahead = 0U;

        for (ping = 0U; ping < 16U; ping++)
        {
            if (0 == obstacle_statechart_servo_deci(&fsm))
            {
                ahead++;
            }
            obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 600U + (ping * 10U), &out);
        }
        CHECK(ahead == 4U);
    }
}

/* Passing on the right puts the obstacle on the car's left: side ray +60. */
static void test_servo_schedule_right(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;
    uint32_t                  side = 0U;

    to_reacquire(&fsm, &out, false);
    fsm.plan.side = OBSTACLE_SIDE_RIGHT;
    for (ping = 0U; ping < 8U; ping++)
    {
        int16_t const servo = obstacle_statechart_servo_deci(&fsm);

        if (0 != servo)
        {
            CHECK(servo == OBSTACLE_HUG_SERVO_DECI);
            side++;
        }
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + (ping * 10U), &out);
    }
    CHECK(side == 6U);
}

/* Forward reading under OBSTACLE_FWD_GUARD_MM on a look-ahead ping stops the
 * car and starts a stationary scan the same way a first detection does. */
static void test_forward_guard_stops(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_reacquire(&fsm, &out, false);

    /* Run side pings until the next ping is a look-ahead ping. */
    while (0 != obstacle_statechart_servo_deci(&fsm))
    {
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + ping, &out);
        ping++;
    }

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_FWD_GUARD_MM - 1U, 500U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_WAIT_STOP);
    CHECK(fsm.cause != OBSTACLE_CAUSE_COLLISION);
    CHECK(out.evt_count == 1U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK) ==
          OBSTACLE_EVT_OBSTACLE);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK) ==
          (OBSTACLE_FWD_GUARD_MM - 1U));
    /* The car holds still; nothing steers it towards the reading. */
    CHECK(out.b_steer_valid);
    CHECK(obstacle_pack_get(out.steer_word, OBSTACLE_EV_STEER_SH, OBSTACLE_EV_STEER_MK) ==
          OBSTACLE_STEER_HOLD);
    CHECK(!out.b_call_stop);
    CHECK(obstacle_statechart_servo_deci(&fsm) == 0);

    /* The same handshake as a first detection: SCAN_REQ starts a scan. */
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 9U), 600U, 2000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_SCANNING);
    CHECK(out.b_profile_changed);
    CHECK(obstacle_statechart_servo_deci(&fsm) == -600);
}

static void test_forward_guard_limits(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;
    uint16_t const            dist[] = {OBSTACLE_FWD_GUARD_MM, OBSTACLE_FWD_GUARD_MM + 1U, 2000U};
    uint32_t                  idx = 0U;

    /* 300 mm and more on a look-ahead ping: no stop. */
    for (idx = 0U; idx < 3U; idx++)
    {
        to_reacquire(&fsm, &out, false);
        while (0 != obstacle_statechart_servo_deci(&fsm))
        {
            obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + ping, &out);
            ping++;
        }
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, dist[idx], 500U, &out);
        CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
        CHECK(out.evt_count == 0U);
    }

    /* OPEN, UNKNOWN or STUCK on a look-ahead ping never count as close. */
    to_reacquire(&fsm, &out, false);
    while (0 != obstacle_statechart_servo_deci(&fsm))
    {
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + ping, &out);
        ping++;
    }
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 500U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    CHECK(out.evt_count == 0U);
}

/* A close reading on a side ping does not feed the forward guard. */
static void test_side_ping_not_guarded(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_reacquire(&fsm, &out, false);

    for (ping = 0U; ping < 16U; ping++)
    {
        bool const b_ahead = (0 == obstacle_statechart_servo_deci(&fsm));

        obstacle_statechart_out_clear(&out);
        if (b_ahead)
        {
            obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + (ping * 10U), &out);
        }
        else
        {
            /* 150..299 mm on the side ray is the normal hug distance. */
            obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID,
                                     (uint16_t)(150U + ((ping * 17U) % 150U)),
                                     300U + (ping * 10U), &out);
        }
        CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
        CHECK(out.evt_count == 0U);
    }
}

/* BYPASS keeps the servo ahead for the pivot: its readings are the pivot's
 * own beam, so a close forward reading there never rescans. */
static void test_bypass_no_guard(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_bypass(&fsm, &out, false);
    for (ping = 0U; ping < 8U; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 100U + (ping * 10U), &out);
        CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    }
}

/* The pivot ends only when the beam is clear AND the planned angle is reached. */
static void test_pivot_needs_turned(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_bypass(&fsm, &out, false);

    /* A clear beam before the planned angle: the pivot goes on. */
    for (ping = 0U; ping < 10U; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 100U + (ping * 10U), &out);
        CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
        CHECK(out.b_steer_valid);
        CHECK(obstacle_pack_get(out.steer_word, OBSTACLE_EV_STEER_SH, OBSTACLE_EV_STEER_MK) ==
              OBSTACLE_STEER_LEFT);
    }

    /* A TURNED for another request is ignored. */
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_TURNED, TEST_ID + 1U), 300U, 1000U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 310U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);

    /* TURNED with the beam clear: REACQUIRE, driving straight for the servo. */
    turned(&fsm, &out, 320U);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 330U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    CHECK(obstacle_pack_get(out.steer_word, OBSTACLE_EV_STEER_SH, OBSTACLE_EV_STEER_MK) ==
          OBSTACLE_STEER_STRAIGHT);
}

static void test_pivot_turned_but_blocked(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_bypass(&fsm, &out, false);
    turned(&fsm, &out, 100U);

    /* Turned, but the beam still sees the obstacle: keep pivoting. */
    for (ping = 0U; ping < 6U; ping++)
    {
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 280U, 110U + (ping * 10U), &out);
        CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    }

    /* One clear ping is not enough, and a close one resets the count. */
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 200U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 280U, 210U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 220U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 230U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
}

/* TURNED does not leak into the next bypass. */
static void test_turned_resets(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, false);
    CHECK(fsm.b_turned);

    /* Rescan from REACQUIRE, accept again: the new pivot starts un-turned. */
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_LINE_FOUND, TEST_ID), 520U, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);

    scan_to_plan(&fsm, &out, false);
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_ACCEPT, TEST_ID), 460U, 2000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    CHECK(!fsm.b_turned);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 100U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 160U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
}

/* LINE_FOUND still ends REACQUIRE (same opcode as TURNED). */
static void test_line_found_unchanged(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, false);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_LINE_FOUND, TEST_ID + 1U), 400U, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_LINE_FOUND, TEST_ID), 520U, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
}

/* A TURNED repeat that reaches the statechart just after BYPASS ended is the
 * same opcode as LINE_FOUND but is not the line: it is ignored in the first
 * OBSTACLE_LINE_GRACE_MS of REACQUIRE (entered at 260 ms by to_reacquire). */
static void test_stale_turned_is_not_line(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, false);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_LINE_FOUND, TEST_ID), 261U, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_LINE_FOUND, TEST_ID),
                                260U + OBSTACLE_LINE_GRACE_MS - 1U, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_LINE_FOUND, TEST_ID),
                                260U + OBSTACLE_LINE_GRACE_MS, 1000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
}

/* A box plan ends on the distance budget; a wall plan does not. Both end on
 * OBSTACLE_HUG_MAX_MS. */
static void test_hug_budget(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  start_ms = 0U;

    /* Box. */
    to_reacquire(&fsm, &out, false);
    start_ms = fsm.since_ms;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, start_ms + 1000U, 1000U + fsm.hug_limit_mm, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_tick(&fsm, start_ms + 1000U, 1000U + fsm.hug_limit_mm + 1U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_LINE_TIMEOUT);

    /* Wall: far beyond the budget and still going. */
    to_reacquire(&fsm, &out, true);
    start_ms = fsm.since_ms;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, start_ms + 1000U, 1000U + OBSTACLE_HUG_MAX_MM + 5000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_tick(&fsm, start_ms + 30000U, 1000U + 20000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_tick(&fsm, start_ms + OBSTACLE_HUG_MAX_MS, 1000U + 50000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_tick(&fsm, start_ms + OBSTACLE_HUG_MAX_MS + 1U, 1000U + 50000U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_LINE_TIMEOUT);
    CHECK((fsm.faults & OBSTACLE_FAULT_LINE_TIMEOUT) != 0U);
}

/* The wall plan reaches REACQUIRE through the same pivot rule. */
static void test_wall_reacquire(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, true);
    CHECK(fsm.plan.b_wall);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
}

/* Forward and side readings steer nothing in the statechart any more: the
 * only STEER words in REACQUIRE are the entry STRAIGHT and a stop's HOLD. */
static void test_reacquire_no_steer(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_reacquire(&fsm, &out, false);
    for (ping = 0U; ping < 12U; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, ((ping % 2U) == 0U) ? OBSTACLE_CLASS_OPEN : OBSTACLE_CLASS_VALID,
                                 400U, 500U + (ping * 10U), &out);
        CHECK(!out.b_steer_valid);
    }
}

int main(void)
{
    test_init_and_status();
    test_obstacle_to_scan();
    test_no_echo_fault();
    test_handover_timeout();
    test_scan_stall();
    test_wall_reaches_bypass();
    test_wall_flag();
    test_servo_schedule();
    test_servo_schedule_right();
    test_forward_guard_stops();
    test_forward_guard_limits();
    test_side_ping_not_guarded();
    test_bypass_no_guard();
    test_pivot_needs_turned();
    test_pivot_turned_but_blocked();
    test_turned_resets();
    test_line_found_unchanged();
    test_stale_turned_is_not_line();
    test_hug_budget();
    test_wall_reacquire();
    test_reacquire_no_steer();

    if (g_failures == 0)
    {
        printf("obstacle_statechart: all checks passed\n");
        return 0;
    }
    printf("obstacle_statechart: %d check(s) failed\n", g_failures);
    return 1;
}
