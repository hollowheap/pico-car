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

int main(void)
{
    test_init_and_status();
    test_obstacle_to_scan();
    test_no_echo_fault();
    test_handover_timeout();
    test_scan_stall();

    if (g_failures == 0)
    {
        printf("obstacle_statechart: all checks passed\n");
        return 0;
    }
    printf("obstacle_statechart: %d check(s) failed\n", g_failures);
    return 1;
}
