/** @file test_statechart.c
 *
 * @brief Host tests for the Buddy 5 obstacle statechart.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 3U), 5U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
    CHECK(!out.b_profile_changed);
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
    CHECK(obstacle_statechart_servo_deci(&fsm) == -OBSTACLE_SERVO_LIMIT_DECI);

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
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_NO_ECHO, 0U, (uint32_t)i, &out);
    }
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK((fsm.faults & OBSTACLE_FAULT_NO_ECHO) != 0U);
    CHECK(out.b_call_stop);
    CHECK(out.evt_count > 0U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK) ==
          OBSTACLE_CAUSE_NO_ECHO);
}

/* Send one ping and return the state, for the pattern tests below. */
static uint8_t ping_state(obstacle_statechart_t * p_fsm, uint8_t cls, uint16_t dist_mm,
                          uint32_t * p_now)
{
    obstacle_statechart_out_t out;

    obstacle_statechart_out_clear(&out);
    *p_now += 1U;
    obstacle_statechart_ping(p_fsm, cls, dist_mm, *p_now, &out);
    return p_fsm->state;
}

static void new_monitor(obstacle_statechart_t * p_fsm)
{
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(p_fsm, &cfg, 0U);
    start_monitor(p_fsm, &out);
}

/* 2 hits among the last 3 pings report an obstacle. A hit, a miss and a
 * hit used to restart the count at the miss and report nothing. */
static void test_detect_two_of_three(void)
{
    obstacle_statechart_t fsm;
    uint32_t              now = 10U;

    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 200U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 900U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 200U, &now) == OBSTACLE_STATE_WAIT_STOP);

    /* A failed ping in the gap also keeps the hit in the window. */
    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 200U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_GLITCH, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 200U, &now) == OBSTACLE_STATE_WAIT_STOP);

    /* Two misses push the first hit out of the window of 3. */
    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 200U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_OPEN, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_OPEN, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 200U, &now) == OBSTACLE_STATE_MONITOR);

    /* A reading beyond the detect distance is no hit. */
    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_DETECT_MM + 1U, &now) ==
          OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_DETECT_MM + 1U, &now) ==
          OBSTACLE_STATE_MONITOR);
}

/* TOO_CLOSE is an obstacle at the minimum range, not a fault. */
static void test_too_close_is_a_hit(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  now = 10U;

    new_monitor(&fsm);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_TOO_CLOSE, 0U, now, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 900U, now + 1U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_TOO_CLOSE, 0U, now + 2U, &out);
    /* 2 TOO_CLOSE pings among the last 3 are a collision. */
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_COLLISION);
    CHECK((fsm.faults & OBSTACLE_FAULT_COLLISION) != 0U);
    CHECK(out.b_call_stop);
    /* The status word shows the class and the minimum range. */
    CHECK(obstacle_pack_get(obstacle_statechart_status(&fsm), OBSTACLE_ST_CLASS_SH,
                            OBSTACLE_ST_CLASS_MK) == OBSTACLE_CLASS_TOO_CLOSE);
    CHECK(obstacle_pack_get(obstacle_statechart_status(&fsm), OBSTACLE_ST_DIST_SH,
                            OBSTACLE_ST_DIST_MK) == OBSTACLE_MIN_RANGE_MM);
}

/* The collision guard uses the same 2 of 3 rule on VALID readings. */
static void test_collision_two_of_three(void)
{
    obstacle_statechart_t fsm;
    uint32_t              now = 10U;

    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, &now) !=
          OBSTACLE_STATE_STOPPED);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, 900U, &now) != OBSTACLE_STATE_STOPPED);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, &now) ==
          OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_COLLISION);

    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM + 1U, &now) !=
          OBSTACLE_STATE_STOPPED);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM + 1U, &now) !=
          OBSTACLE_STATE_STOPPED);
}

/* each failing class has its own limit and cause. */
static void test_echo_fault_causes(void)
{
    obstacle_statechart_t fsm;
    uint32_t              now = 10U;
    uint8_t               i   = 0U;

    new_monitor(&fsm);
    for (i = 0U; i < OBSTACLE_STUCK_COUNT; i++)
    {
        (void)ping_state(&fsm, OBSTACLE_CLASS_STUCK, 0U, &now);
    }
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_ECHO_STUCK);
    CHECK((fsm.faults & OBSTACLE_FAULT_ECHO_STUCK) != 0U);

    new_monitor(&fsm);
    for (i = 0U; i < OBSTACLE_GLITCH_COUNT; i++)
    {
        CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
        (void)ping_state(&fsm, OBSTACLE_CLASS_GLITCH, 0U, &now);
    }
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_ECHO_NOISE);
    CHECK((fsm.faults & OBSTACLE_FAULT_ECHO_NOISE) != 0U);

    /* A mix of failures that never reaches a single limit still adds up. */
    new_monitor(&fsm);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_NO_ECHO, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_GLITCH, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_NO_ECHO, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_GLITCH, 0U, &now) == OBSTACLE_STATE_MONITOR);
    CHECK(ping_state(&fsm, OBSTACLE_CLASS_STUCK, 0U, &now) == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_ECHO_NOISE);

    /* Any good ping ends the run. */
    new_monitor(&fsm);
    for (i = 0U; i < 10U; i++)
    {
        (void)ping_state(&fsm, OBSTACLE_CLASS_NO_ECHO, 0U, &now);
        (void)ping_state(&fsm, OBSTACLE_CLASS_NO_ECHO, 0U, &now);
        (void)ping_state(&fsm, OBSTACLE_CLASS_OPEN, 0U, &now);
    }
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
}

/* The retry policy: only the first start and a transient cause restart by
 * themselves. */
static void test_retry_policy(void)
{
    uint32_t cause = 0U;

    CHECK(obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_BOOT));
    CHECK(obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_SCAN_TIMEOUT));
    CHECK(obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_HANDOVER));
    CHECK(obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_PLAN_TIMEOUT));
    CHECK(obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_PLAN_REJECTED));

    /* A bypass that gave up must not drive back at the same obstacle. */
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_BYPASS_LIMIT));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_LINE_TIMEOUT));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_BYPASS_GAVE_UP));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_COLLISION));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_NO_SAFE_PATH));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_NO_ECHO));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_ECHO_STUCK));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_ECHO_NOISE));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_EXTERNAL));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_INIT_FAIL));
    CHECK(!obstacle_pack_cause_may_auto_restart(OBSTACLE_CAUSE_NONE));

    /* Every cause the statechart can latch is covered by the list above. */
    for (cause = 0U; cause < OBSTACLE_CAUSE_COUNT; cause++)
    {
        (void)obstacle_pack_cause_may_auto_restart(cause);
    }
}

/* The status word keeps the 3-bit class and the 5-bit sequence apart. */
static void test_status_word_fields(void)
{
    obstacle_statechart_t fsm;
    uint32_t              now = 10U;
    uint32_t              word = 0U;
    uint8_t               cls  = 0U;

    new_monitor(&fsm);
    for (cls = 0U; cls < OBSTACLE_CLASS_COUNT; cls++)
    {
        if (OBSTACLE_STATE_STOPPED == fsm.state)
        {
            new_monitor(&fsm);
        }
        (void)ping_state(&fsm, (uint8_t)((cls % 2U == 0U) ? cls : OBSTACLE_CLASS_VALID), 900U, &now);
        word = obstacle_statechart_status(&fsm);
        CHECK(obstacle_pack_get(word, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK) ==
              (fsm.seq & OBSTACLE_ST_SEQ_MK));
    }
    fsm.last_class = OBSTACLE_CLASS_STUCK;
    fsm.seq        = 31U;
    fsm.req_id     = 31U;
    word           = obstacle_statechart_status(&fsm);
    CHECK(obstacle_pack_get(word, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK) ==
          OBSTACLE_CLASS_STUCK);
    CHECK(obstacle_pack_get(word, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK) == 31U);
    CHECK(obstacle_pack_get(word, OBSTACLE_ST_ID_SH, OBSTACLE_ST_ID_MK) == 31U);
}

/* P3: of the first pings after boot, mostly hits means the beam is blocked. */
static void test_beam_blocked_at_boot (void)
{
    obstacle_statechart_t fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t   cfg;
    uint32_t              idx = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    obstacle_statechart_out_clear(&out);
    for (idx = 0U; idx < OBSTACLE_BEAM_CHECK_PINGS; idx++)
    {
        CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_BEAM_BLOCKED) == 0U);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 120U, idx, &out);
    }
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_BEAM_BLOCKED) != 0U);

    /* A clear beam raises nothing, and later hits do not set it. */
    obstacle_statechart_init(&fsm, &cfg, 0U);
    for (idx = 0U; idx < OBSTACLE_BEAM_CHECK_PINGS; idx++)
    {
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, idx, &out);
    }
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 120U, 100U, &out);
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_BEAM_BLOCKED) == 0U);
}

/* P2: after 100 mm of travel the range must have changed by the travel. */
static void test_range_follows_travel (void)
{
    obstacle_statechart_t fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t   cfg;
    uint32_t              now = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    obstacle_statechart_out_clear(&out);

    obstacle_statechart_set_odometer(&fsm, 1000U, true);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 500U, ++now, &out);

    /* 60 mm of travel: too short to judge. */
    obstacle_statechart_set_odometer(&fsm, 1060U, true);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 100U, ++now, &out);
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_ECHO_INCONSISTENT) == 0U);

    /* 120 mm travelled and the range fell 110 mm (500 -> 390): consistent. */
    obstacle_statechart_set_odometer(&fsm, 1120U, true);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 390U, ++now, &out);
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_ECHO_INCONSISTENT) == 0U);

    /* 150 mm travelled and the range did not move: a wrong target or a phantom. */
    obstacle_statechart_set_odometer(&fsm, 1270U, true);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 385U, ++now, &out);
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_ECHO_INCONSISTENT) != 0U);

    /* A consistent pair clears it again. */
    obstacle_statechart_set_odometer(&fsm, 1400U, true);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 255U, ++now, &out);
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_ECHO_INCONSISTENT) == 0U);

    /* No fresh travel: the old anchor is not compared with a later one. */
    obstacle_statechart_set_odometer(&fsm, 1400U, false);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 250U, ++now, &out);
    obstacle_statechart_set_odometer(&fsm, 1600U, true);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 250U, ++now, &out);
    CHECK((obstacle_statechart_flags(&fsm) & OBSTACLE_FLAG_ECHO_INCONSISTENT) == 0U);
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
    obstacle_statechart_tick(&fsm, 520U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_WAIT_STOP);
    obstacle_statechart_tick(&fsm, 521U, 0U, &out);
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

/* A body pivot that stalls stops the pings mid edge walk. The stall timeout
 * then plans with the slots it has, so the car still drives a leg. */
static void test_pivot_stall_still_plans(void)
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
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 5U), 30U, 0U, &out);

    /* A wall, fed until the scan asks for the first body pivot. */
    while ((0 == obstacle_statechart_body_deci(&fsm)) && (guard < 100U))
    {
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 300U, 40U, &out);
        guard++;
    }
    CHECK(fsm.state == OBSTACLE_STATE_SCANNING);
    CHECK(obstacle_statechart_body_deci(&fsm) > 0);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, 40U + OBSTACLE_SCAN_STALL_MS + 1U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    CHECK(fsm.cause == OBSTACLE_CAUSE_NONE);
    CHECK(fsm.plan.status == OBSTACLE_RESULT_OK);
    CHECK(fsm.plan.side == OBSTACLE_SIDE_LEFT);
    CHECK(!out.b_call_stop);
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
    CHECK(fsm.hug_limit_mm == OBSTACLE_HUG_MAX_MM); /* 3441 + the 2500 mm along-leg cap, capped */
    CHECK(fsm.steer == obstacle_plan_steer_away(OBSTACLE_SIDE_LEFT));
    CHECK(!out.b_call_stop);
}


/* ---- moving forward guard, pivot exit, wall budget ---------------- */

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
        CHECK(obstacle_statechart_servo_deci(&fsm) == 0);
        /* A far, harmless reading ahead. */
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + (ping * 10U), &out);
        CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    }
}

/* Passing on either side keeps the moving sonar forward (0 deg). */
static void test_servo_schedule_right(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_reacquire(&fsm, &out, false);
    fsm.plan.side = OBSTACLE_SIDE_RIGHT;
    for (ping = 0U; ping < 8U; ping++)
    {
        CHECK(obstacle_statechart_servo_deci(&fsm) == 0);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + (ping * 10U), &out);
    }
}

/* In REACQUIRE a close reading never starts a rescan: the face follow turns
 * away from it. Only the collision window still stops the car. */
static void test_reacquire_close_reading_no_rescan(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, false);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_FWD_GUARD_MM - 1U, 500U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    CHECK(out.evt_count == 0U);

    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, 560U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, 620U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_COLLISION);
}

static void test_forward_guard_limits(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint16_t const            dist[] = {OBSTACLE_FWD_GUARD_MM, OBSTACLE_FWD_GUARD_MM + 1U, 2000U};
    uint32_t                  idx = 0U;

    /* 300 mm and more forward: no stop. */
    for (idx = 0U; idx < 3U; idx++)
    {
        to_reacquire(&fsm, &out, false);
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, dist[idx], 500U, &out);
        CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
        CHECK(out.evt_count == 0U);
    }

    /* OPEN, NO_ECHO, GLITCH or STUCK forward never count as close. */
    to_reacquire(&fsm, &out, false);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 500U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    CHECK(out.evt_count == 0U);
}

/* Far forward readings do not trigger the guard. */
static void test_side_ping_not_guarded(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_reacquire(&fsm, &out, false);

    for (ping = 0U; ping < 16U; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 1000U, 300U + (ping * 10U), &out);
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

    /* TURNED with the beam clear: the clear pings count again from the
     * turned heading, then REACQUIRE, driving straight for the servo. */
    turned(&fsm, &out, 320U);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 330U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 340U, &out);
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

/* Turned, and the beam still sees the obstacle: the turn target moves on
 * away from the obstacle, one step at a time up to the limit, so the car
 * keeps turning instead of standing still. A clear beam then ends BYPASS. */
static void test_blocked_escape_turns_further(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    int32_t                   start = 0;
    uint32_t                  now   = 100U;
    uint32_t                  ping  = 0U;

    to_bypass(&fsm, &out, false);
    CHECK(fsm.plan.side == OBSTACLE_SIDE_LEFT);
    start = obstacle_statechart_body_deci(&fsm);
    CHECK(start == ((int32_t)fsm.plan.start_deg * OBSTACLE_DECI_PER_DEG));
    turned(&fsm, &out, now);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 280U, now + 10U, &out);
    CHECK(obstacle_statechart_body_deci(&fsm) == start);

    now += OBSTACLE_ESCAPE_STEP_MS;
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 280U, now, &out);
    CHECK(obstacle_statechart_body_deci(&fsm) == (start + OBSTACLE_ESCAPE_STEP_DECI));

    /* The next step waits until the car has reached this one. */
    now += OBSTACLE_ESCAPE_STEP_MS;
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 280U, now, &out);
    CHECK(obstacle_statechart_body_deci(&fsm) == (start + OBSTACLE_ESCAPE_STEP_DECI));
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, now + 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, now + 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);

    /* Each reached step restarts the time budget, so a slow escape that
     * keeps turning is not stopped by the limit. */
    for (ping = 0U; (ping < 20U) && (obstacle_statechart_body_deci(&fsm) <
                                     OBSTACLE_ESCAPE_TURN_MAX_DECI); ping++)
    {
        now += OBSTACLE_ESCAPE_MAX_MS - OBSTACLE_ESCAPE_STEP_MS;
        turned(&fsm, &out, now);
        obstacle_statechart_tick(&fsm, now, 0U, &out);
        CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
        now += OBSTACLE_ESCAPE_STEP_MS;
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 280U, now, &out);
    }
    CHECK((now - 100U) > OBSTACLE_ESCAPE_MAX_MS);
    CHECK(obstacle_statechart_body_deci(&fsm) == OBSTACLE_ESCAPE_TURN_MAX_DECI);

    turned(&fsm, &out, now);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, now + 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, now + 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
}

/* No reached step for the whole budget: the limit still ends BYPASS. */
static void test_escape_without_progress_hits_limit(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_bypass(&fsm, &out, false);
    obstacle_statechart_tick(&fsm, 6060U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_BYPASS);
    obstacle_statechart_tick(&fsm, 6061U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_BYPASS_LIMIT);
}

static void test_plan_timeout_boundary(void)
{
    obstacle_statechart_t fsm;
    obstacle_statechart_out_t out;
    unsigned ping = 0U;

    new_monitor(&fsm);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 5U), 30U, 0U, &out);
    while (OBSTACLE_STATE_SCANNING == fsm.state && ping < 100U)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 40U, &out);
        ++ping;
    }
    CHECK(OBSTACLE_STATE_PLAN == fsm.state);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, 240U, 0U, &out);
    CHECK(OBSTACLE_STATE_PLAN == fsm.state);
    obstacle_statechart_tick(&fsm, 241U, 0U, &out);
    CHECK(OBSTACLE_STATE_STOPPED == fsm.state);
    CHECK(OBSTACLE_CAUSE_PLAN_TIMEOUT == fsm.cause);
    CHECK(1U == out.evt_count);
    CHECK(OBSTACLE_CAUSE_PLAN_TIMEOUT == obstacle_pack_get(out.evt[0], OBSTACLE_EV_CAUSE_SH,
                                                           OBSTACLE_EV_CAUSE_MK));
}

/* Select explicitly while the implementation and the recorded timing differ. */
static void test_recorded_ping_budget(void)
{
    CHECK(60U == OBSTACLE_PING_PERIOD_MS);
    CHECK(40U == OBSTACLE_ECHO_WAIT_MS);
    CHECK(OBSTACLE_ECHO_WAIT_MS < OBSTACLE_PING_PERIOD_MS);
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

/* The hug travel only grows, forward and backward, and wraps at 2^32: a
 * short distance across the wrap is short, not ~4e9 mm. */
static void test_hug_odometer_wraps(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, false);
    fsm.bypass_mm = UINT32_MAX - 10U;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&fsm, fsm.since_ms + 100U, 20U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    CHECK(fsm.cause != OBSTACLE_CAUSE_LINE_TIMEOUT);
}

/* REJECT during the bypass gives the bypass up; the id field is the
 * reason, which the FAULT event carries in its reason bits. */
static void test_reject_in_reacquire_gives_up(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;

    to_reacquire(&fsm, &out, false);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_REJECT, 2U), 400U, 1500U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_BYPASS_GAVE_UP);
    CHECK(fsm.reason == 2U);
    CHECK((fsm.faults & OBSTACLE_FAULT_BYPASS_LIMIT) != 0U);
    CHECK(out.evt_count == 1U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK) ==
          OBSTACLE_CAUSE_BYPASS_GAVE_UP);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_REASON_SH, OBSTACLE_EV_REASON_MK) == 2U);
    CHECK(obstacle_pack_get(out.evt[0], OBSTACLE_EV_FAULTS_SH, OBSTACLE_EV_FAULTS_MK) ==
          (uint32_t)fsm.faults);

    /* Any other stop clears the reason. */
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_RESET, 0U), 500U, 1500U, &out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_START, 0U), 510U, 1500U, &out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_STOP, 0U), 520U, 1500U, &out);
    CHECK(fsm.cause == OBSTACLE_CAUSE_EXTERNAL);
    CHECK(fsm.reason == 0U);

    /* RESET after a give-up clears the reason too. */
    to_reacquire(&fsm, &out, false);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_REJECT, 3U), 400U, 1500U, &out);
    CHECK(fsm.reason == 3U);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_RESET, 0U), 500U, 1500U, &out);
    CHECK(fsm.reason == 0U);
}

/* A plan with no safe path, rejected by the navigation, stops with
 * NO_SAFE_PATH and its fault bit; it does not restart on the same obstacle. */
static void test_no_safe_path_plan_stops(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;
    uint32_t                  guard = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, TEST_ID), 30U, 0U, &out);

    /* Open on both sides, unresolved straight ahead. */
    while ((fsm.state == OBSTACLE_STATE_SCANNING) && (guard < 200U))
    {
        bool const    b_centre = (scan_algo_target(&fsm.scan) == OBSTACLE_SLOT_CENTRE);
        uint8_t const cls = b_centre ? (uint8_t)OBSTACLE_CLASS_GLITCH : (uint8_t)OBSTACLE_CLASS_OPEN;

        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, cls, 0U, 40U, &out);
        guard++;
    }
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    CHECK(fsm.plan.status == OBSTACLE_RESULT_NO_SAFE_PATH);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_REJECT, TEST_ID), 60U, 0U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_NO_SAFE_PATH);
    CHECK((fsm.faults & OBSTACLE_FAULT_NO_SAFE_PATH) != 0U);
}

/* pings during a spin in REACQUIRE do not count for the
 * collision guard; the same readings while driving do. */
static void test_spin_pings_skip_collision(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t                  ping = 0U;

    to_reacquire(&fsm, &out, false);
    for (ping = 0U; ping < 4U; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping_guard(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, 400U + (ping * 60U), &out);
    }
    CHECK(fsm.state == OBSTACLE_STATE_REACQUIRE);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, 700U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, OBSTACLE_COLLISION_MM, 760U, &out);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_COLLISION);
}

/* The health-only entry: close readings never latch a collision, but a stuck
 * echo still stops the statechart. */
static void test_ping_health_skips_collision(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;
    uint32_t                  ping = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
    for (ping = 0U; ping < 8U; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping_health(&fsm, OBSTACLE_CLASS_VALID, 300U, true, 400U + (ping * 60U), &out);
    }
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
    for (ping = 0U; ping < OBSTACLE_STUCK_COUNT; ping++)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping_health(&fsm, OBSTACLE_CLASS_STUCK, 300U, true, 900U + (ping * 60U), &out);
    }
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_ECHO_STUCK);
}

/* The health entry records the ping in the published ping word. */
static void test_ping_health_records_word(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;
    uint32_t                  before = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);

    before = fsm.ping;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping_health(&fsm, OBSTACLE_CLASS_VALID, 700U, true, 100U, &out);
    CHECK(obstacle_pack_get(fsm.ping, OBSTACLE_PING_SEQ_SH, OBSTACLE_PING_SEQ_MK) ==
          ((obstacle_pack_get(before, OBSTACLE_PING_SEQ_SH, OBSTACLE_PING_SEQ_MK) + 1U) &
           OBSTACLE_PING_SEQ_MK));
    CHECK(obstacle_pack_get(fsm.ping, OBSTACLE_PING_CLASS_SH, OBSTACLE_PING_CLASS_MK) ==
          OBSTACLE_CLASS_VALID);
    CHECK(obstacle_pack_get(fsm.ping, OBSTACLE_PING_DIST_SH, OBSTACLE_PING_DIST_MK) == 700U);
    CHECK(fsm.last_dist == 700U);
    CHECK(fsm.last_class == OBSTACLE_CLASS_VALID);

    /* A requested ping has its own reply word: it updates the reading but
     * must not pass for a ping straight ahead. */
    before = fsm.ping;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping_health(&fsm, OBSTACLE_CLASS_VALID, 680U, false, 130U, &out);
    CHECK(fsm.ping == before);
    CHECK(fsm.last_dist == 680U);

    /* A guard ping changes no reading and leaves the face ping word alone. */
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping_guard(&fsm, OBSTACLE_CLASS_VALID, 650U, 160U, &out);
    CHECK(fsm.ping == before);
    CHECK(fsm.last_dist == 680U);
}

static void test_guard_pings_leave_scan_target(void)
{
    obstacle_statechart_t fsm;
    obstacle_statechart_out_t out;
    int16_t target;

    new_monitor(&fsm);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 20U, &out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_SCAN_REQ, 4U), 30U, 0U, &out);
    CHECK(OBSTACLE_STATE_SCANNING == fsm.state);
    target = obstacle_statechart_servo_deci(&fsm);

    obstacle_statechart_ping_guard(&fsm, OBSTACLE_CLASS_VALID, 300U, 90U, &out);
    obstacle_statechart_ping_guard(&fsm, OBSTACLE_CLASS_VALID, 300U, 150U, &out);
    CHECK(target == obstacle_statechart_servo_deci(&fsm));

    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 300U, 210U, &out);
    obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 300U, 270U, &out);
    CHECK(target != obstacle_statechart_servo_deci(&fsm));
}

static void test_guard_pings_keep_echo_health(void)
{
    obstacle_statechart_t fsm;
    obstacle_statechart_out_t out;
    unsigned ping;

    new_monitor(&fsm);
    obstacle_statechart_out_clear(&out);
    for (ping = 0U; ping < OBSTACLE_STUCK_COUNT; ++ping)
    {
        obstacle_statechart_ping_guard(&fsm, OBSTACLE_CLASS_STUCK, 0U,
                                       60U + (ping * 60U), &out);
    }
    CHECK(OBSTACLE_STATE_STOPPED == fsm.state);
    CHECK(OBSTACLE_CAUSE_ECHO_STUCK == fsm.cause);
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

/* Feed health pings and return the FAULT events they raised. */
static uint32_t health_pings(obstacle_statechart_t * p_fsm, uint8_t cls, uint32_t count,
                             uint32_t * p_now)
{
    obstacle_statechart_out_t out;
    uint32_t                  faults = 0U;
    uint32_t                  idx    = 0U;
    uint32_t                  evt    = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        obstacle_statechart_out_clear(&out);
        *p_now += 1U;
        obstacle_statechart_ping_health(p_fsm, cls, 0U, false, *p_now, &out);
        for (evt = 0U; evt < out.evt_count; evt++)
        {
            if (obstacle_pack_get(out.evt[evt], OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK) ==
                OBSTACLE_EVT_FAULT)
            {
                faults++;
            }
        }
    }

    return faults;
}

/* A stuck echo, a dead sonar and echo noise stop the car only while they
 * hold: one FAULT event on the way in, and a good ping lets it go on. */
static void test_health_fault_ends_when_cause_clears(void)
{
    static uint8_t const classes[3]  = {OBSTACLE_CLASS_NO_ECHO, OBSTACLE_CLASS_STUCK,
                                        OBSTACLE_CLASS_GLITCH};
    static uint16_t const bits[3]    = {OBSTACLE_FAULT_NO_ECHO, OBSTACLE_FAULT_ECHO_STUCK,
                                        OBSTACLE_FAULT_ECHO_NOISE};
    static uint32_t const counts[3]  = {OBSTACLE_NO_ECHO_COUNT, OBSTACLE_STUCK_COUNT,
                                        OBSTACLE_GLITCH_COUNT};
    uint32_t              idx = 0U;

    for (idx = 0U; idx < 3U; idx++)
    {
        obstacle_statechart_t     fsm;
        obstacle_statechart_out_t out;
        obstacle_plan_cfg_t       cfg;
        uint32_t                  now = 10U;

        obstacle_plan_cfg_default(&cfg);
        obstacle_statechart_init(&fsm, &cfg, 0U);
        start_monitor(&fsm, &out);

        CHECK(health_pings(&fsm, classes[idx], counts[idx], &now) == 1U);
        CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
        CHECK((fsm.faults & bits[idx]) != 0U);

        /* Still failing: no new event, the fault holds. */
        CHECK(health_pings(&fsm, classes[idx], 20U, &now) == 0U);
        CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
        CHECK((fsm.faults & bits[idx]) != 0U);

        /* One good ping ends it. */
        CHECK(health_pings(&fsm, OBSTACLE_CLASS_VALID, 1U, &now) == 0U);
        CHECK((fsm.faults & bits[idx]) == 0U);
        CHECK(fsm.state == OBSTACLE_STATE_MONITOR);
        CHECK(fsm.cause == OBSTACLE_CAUSE_NONE);
    }
}

/* Only the operator stop latches: echo faults coming and going do not undo it. */
static void test_estop_survives_health_recovery(void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_plan_cfg_t       cfg;
    uint32_t                  now = 10U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_STOP, 0U), now, 0U, &out);

    CHECK(health_pings(&fsm, OBSTACLE_CLASS_NO_ECHO, 10U, &now) == 0U);
    CHECK(health_pings(&fsm, OBSTACLE_CLASS_VALID, 10U, &now) == 0U);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(fsm.cause == OBSTACLE_CAUSE_EXTERNAL);

    /* The operator stop also holds when it comes during an echo stop. */
    obstacle_statechart_init(&fsm, &cfg, 0U);
    start_monitor(&fsm, &out);
    CHECK(health_pings(&fsm, OBSTACLE_CLASS_NO_ECHO, OBSTACLE_NO_ECHO_COUNT, &now) == 1U);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, cmd(OBSTACLE_CMD_STOP, 0U), now, 0U, &out);
    CHECK(health_pings(&fsm, OBSTACLE_CLASS_VALID, 10U, &now) == 0U);
    CHECK(fsm.state == OBSTACLE_STATE_STOPPED);
}

int main(int argc, char * argv[])
{
    static const struct
    {
        char const * name;
        void (*test)(void);
    } selectable[] = {
        {"coarse-stall", test_scan_stall},
        {"fine-pivot-stall", test_pivot_stall_still_plans},
        {"two-of-three", test_detect_two_of_three},
        {"too-close", test_too_close_is_a_hit},
        {"collision-window", test_collision_two_of_three},
        {"retry-policy", test_retry_policy},
        {"spin-pings", test_spin_pings_skip_collision},
        {"health-pings", test_ping_health_records_word},
        {"guard-keeps-scan-target", test_guard_pings_leave_scan_target},
        {"guard-keeps-health", test_guard_pings_keep_echo_health},
        {"rescue-escape-turn", test_blocked_escape_turns_further},
        {"no-escape-progress", test_escape_without_progress_hits_limit},
        {"bypass-budgets", test_hug_budget},
        {"bypass-odometer-wrap", test_hug_odometer_wraps},
        {"publish-gave-up-reason", test_reject_in_reacquire_gives_up},
        {"handover-timeout", test_handover_timeout},
        {"plan-timeout", test_plan_timeout_boundary},
        {"recorded-ping-budget", test_recorded_ping_budget},
        {"scan-handshake", test_obstacle_to_scan},
        {"echo-fault-causes", test_echo_fault_causes},
        {"no-echo-fault", test_no_echo_fault},
    };
    size_t i;

    if (2 == argc)
    {
        for (i = 0U; i < sizeof(selectable) / sizeof(selectable[0]); ++i)
        {
            if (0 == strcmp(argv[1], selectable[i].name))
            {
                selectable[i].test();
                return (0 == g_failures) ? 0 : 1;
            }
        }
    }
    if (1 != argc)
    {
        (void)fprintf(stderr, "usage: %s [case]\n", argv[0]);
        return 2;
    }

    test_init_and_status();
    test_obstacle_to_scan();
    test_no_echo_fault();
    test_detect_two_of_three();
    test_too_close_is_a_hit();
    test_collision_two_of_three();
    test_echo_fault_causes();
    test_retry_policy();
    test_status_word_fields();
    test_beam_blocked_at_boot();
    test_range_follows_travel();
    test_handover_timeout();
    test_scan_stall();
    test_blocked_escape_turns_further();
    test_escape_without_progress_hits_limit();
    test_plan_timeout_boundary();
    test_pivot_stall_still_plans();
    test_wall_reaches_bypass();
    test_wall_flag();
    test_servo_schedule();
    test_servo_schedule_right();
    test_reacquire_close_reading_no_rescan();
    test_forward_guard_limits();
    test_side_ping_not_guarded();
    test_bypass_no_guard();
    test_pivot_needs_turned();
    test_pivot_turned_but_blocked();
    test_turned_resets();
    test_line_found_unchanged();
    test_stale_turned_is_not_line();
    test_hug_budget();
    test_hug_odometer_wraps();
    test_reject_in_reacquire_gives_up();
    test_no_safe_path_plan_stops();
    test_spin_pings_skip_collision();
    test_ping_health_skips_collision();
    test_ping_health_records_word();
    test_guard_pings_leave_scan_target();
    test_guard_pings_keep_echo_health();
    test_wall_reacquire();
    test_reacquire_no_steer();
    test_health_fault_ends_when_cause_clears();
    test_estop_survives_health_recovery();

    if (g_failures == 0)
    {
        printf("obstacle_statechart: all checks passed\n");
        return 0;
    }
    printf("obstacle_statechart: %d check(s) failed\n", g_failures);
    return 1;
}
