/** @file test_obstacle_comms.c
 *
 * @brief Host tests for the obstacle published copies: the status scalars
 *        read back in one call as the last step published them, and a
 *        command lost to a full queue counted with the dropped events.
 *
 * comms.c and obstacle.c build unchanged against the kernel shim; the event
 * flag, message buffer, fault register and sonar hardware are faked here.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tk/tkernel.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "statechart.h"
#include "comms.h"
#include "scan_hw.h"
#include "scan_task.h"
#include "navigation/obstacle.h"
#include "platform/faults.h"

static int g_failures = 0;
static uint32_t g_level_faults;
static unsigned g_transition_count;
static unsigned g_completed_count;
static unsigned g_scan_handoffs;
static uint8_t g_last_state;
static bool g_b_observing_legacy;
static obstacle_plan_summary_t g_finished_plan;
static uint16_t g_finished_centre;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            (void)fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,   \
                          __LINE__, #cond);                                \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

/* ---- Fakes for the kernel and the neighbours of comms.c / obstacle.c ---- */

INT g_shim_prc = 1;

static UINT g_flag_ptn = 0U;

ID tk_cre_flg (const T_CFLG * pk_cflg)
{
    g_flag_ptn = pk_cflg->iflgptn;
    return 1;
}

ER tk_set_flg (ID flgid, UINT setptn)
{
    (void)flgid;
    g_flag_ptn |= setptn;
    return E_OK;
}

ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout)
{
    ER result = E_TMOUT;

    (void)flgid;
    (void)tmout;
    if (0U != (g_flag_ptn & waiptn))
    {
        *p_flgptn = g_flag_ptn;
        if (0U != (wfmode & TWF_BITCLR))
        {
            g_flag_ptn &= ~waiptn;
        }
        result = E_OK;
    }
    return result;
}

ER tk_snd_mbf (ID mbfid, const void * msg, INT msgsz, TMO tmout)
{
    (void)mbfid;
    (void)tmout;
    if (g_b_observing_legacy && msgsz == (INT)sizeof(uint32_t))
    {
        uint32_t const word = *(const uint32_t *)msg;
        if (OBSTACLE_EVT_SCAN_RESULT == obstacle_pack_get(word, OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK))
        {
            g_scan_handoffs++;
            CHECK(g_completed_count == g_scan_handoffs);
            CHECK(OBSTACLE_STATE_PLAN == g_last_state);
        }
    }
    return E_OK;
}

void system_fault_assign (fault_owner_t owner, uint32_t mask, uint32_t active)
{
    CHECK(FAULT_OWNER_OBSTACLE == owner);
    g_level_faults = (g_level_faults & ~mask) | (active & mask);
}

ER scan_hw_init (ID h_echo_flag)
{
    (void)h_echo_flag;
    return E_OK;
}

void scan_task_obstacle_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
}

/* ---- Tests ---- */

/* A step with a distinct value in every scalar of the group. */
static void make_step (obstacle_statechart_t * p_fsm)
{
    obstacle_plan_cfg_t cfg;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(p_fsm, &cfg, 0U);
    p_fsm->state          = OBSTACLE_STATE_BYPASS;
    p_fsm->cause          = 2U;
    p_fsm->last_class     = OBSTACLE_CLASS_VALID;
    p_fsm->last_dist      = 345U;
    p_fsm->req_id         = 9U;
    p_fsm->seq            = 17U;
    p_fsm->escape_deci    = -123;
    p_fsm->plan.b_wall    = true;
    p_fsm->b_beam_blocked = true;
    p_fsm->faults         = (uint16_t)(OBSTACLE_FAULT_NO_ECHO | OBSTACLE_FAULT_ECHO_STUCK);
    p_fsm->reason         = 5U;
}

static void test_scalars_read_as_published (void)
{
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_comms_scalars_t  pub;
    obstacle_status_t         status;
    uint16_t const            faults = (uint16_t)(OBSTACLE_FAULT_NO_ECHO | OBSTACLE_FAULT_ECHO_STUCK);

    make_step(&fsm);
    obstacle_statechart_out_clear(&out);
    obstacle_comms_publish(&out, &fsm);

    (void)memset(&pub, 0xA5, sizeof(pub));
    obstacle_comms_copy_scalars(&pub);
    CHECK(pub.status == obstacle_statechart_status(&fsm));
    CHECK(pub.faults == (uint16_t)(faults | (5U << OBSTACLE_STOP_REASON_SH)));
    CHECK(pub.flags == (uint8_t)(OBSTACLE_FLAG_WALL | OBSTACLE_FLAG_BEAM_BLOCKED));
    CHECK(pub.aim == ((uint32_t)(uint16_t)(int16_t)-123 << 16U));
    CHECK(pub.drops == 0U);

    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(status.state == OBSTACLE_STATE_BYPASS);
    CHECK(status.cause == 2U);
    CHECK(status.echo_class == OBSTACLE_CLASS_VALID);
    CHECK(status.distance_mm == 345U);
    CHECK(status.req_id == 9U);
    CHECK(status.seq == 17U);
    CHECK(status.faults == faults);
    CHECK(status.stop_reason == 5U);
    CHECK(status.flags == (uint8_t)(OBSTACLE_FLAG_WALL | OBSTACLE_FLAG_BEAM_BLOCKED));
    CHECK(status.servo_deci == 0);
    CHECK(status.body_deci == -123);
    CHECK(status.dropped_events == 0U);
}

static void test_full_command_queue_counts_drop (void)
{
    obstacle_status_t status;
    uint8_t           cmd = 0U;
    uint32_t          idx = 0U;

    for (idx = 0U; idx < OBSTACLE_CMD_RING_LEN; idx++)
    {
        CHECK(E_OK == obstacle_post_cmd(obstacle_pack_cmd(OBSTACLE_CMD_START, idx)));
    }
    CHECK(E_LIMIT == obstacle_post_cmd(obstacle_pack_cmd(OBSTACLE_CMD_RESET, 1U)));

    CHECK(obstacle_dropped_events() == 1U);
    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(status.dropped_events == 1U);

    /* STOP never enters the queue, so it is never dropped. */
    CHECK(E_OK == obstacle_post_cmd(obstacle_pack_cmd(OBSTACLE_CMD_STOP, 0U)));
    CHECK(obstacle_dropped_events() == 1U);

    /* Room again: the next command is queued and nothing more is counted. */
    CHECK(obstacle_comms_pop_cmd(&cmd));
    CHECK(E_OK == obstacle_post_cmd(obstacle_pack_cmd(OBSTACLE_CMD_RESET, 2U)));
    CHECK(obstacle_dropped_events() == 1U);
}

static void publish_ping (obstacle_statechart_t * p_fsm, bool pilot, uint8_t cls, uint32_t now)
{
    obstacle_statechart_out_t out;
    uint16_t const            dist = (OBSTACLE_CLASS_VALID == cls) ? 900U : 0U;

    obstacle_statechart_out_clear(&out);
    if (pilot)
    {
        obstacle_statechart_ping_health(p_fsm, cls, dist, true, now, &out);
    }
    else
    {
        obstacle_statechart_ping(p_fsm, cls, dist, now, &out);
    }
    obstacle_comms_publish(&out, p_fsm);
}

static void test_parked_echo_faults (bool pilot)
{
    obstacle_plan_cfg_t       cfg;
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    uint32_t const            mask = FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT;
    uint32_t                  ping = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    for (ping = 0U; ping < OBSTACLE_NO_ECHO_COUNT; ping++)
    {
        publish_ping(&fsm, pilot, OBSTACLE_CLASS_NO_ECHO, ping);
    }
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & mask));

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_RESET, 0U), ping, 0U, &out);
    obstacle_comms_publish(&out, &fsm);
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & mask));

    publish_ping(&fsm, pilot, OBSTACLE_CLASS_STUCK, ping++);
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & mask));
    for (uint32_t stuck = 1U; stuck < OBSTACLE_STUCK_COUNT; stuck++)
    {
        publish_ping(&fsm, pilot, OBSTACLE_CLASS_STUCK, ping++);
    }
    /* Both diagnoses are still unresolved. Publishing STUCK must not emit
     * a false NO_ECHO recovery to the dashboard. */
    CHECK(mask == (g_level_faults & mask));
    publish_ping(&fsm, pilot, OBSTACLE_CLASS_VALID, ping);
    CHECK(0U == (g_level_faults & mask));
    CHECK(OBSTACLE_STATE_STOPPED == fsm.state);
}

static void test_pilot_echo_stop_needs_usable_ping (void)
{
    obstacle_plan_cfg_t       cfg;
    obstacle_statechart_t     fsm;
    obstacle_statechart_out_t out;
    obstacle_status_t         status;
    uint32_t const            health_mask = FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT;
    uint32_t                  ping = 0U;

    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_RESET, 0U), 0U, 0U, &out);
    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_START, 0U), 0U, 0U, &out);
    for (ping = 0U; ping < OBSTACLE_NO_ECHO_COUNT; ping++)
    {
        publish_ping(&fsm, true, OBSTACLE_CLASS_NO_ECHO, ping);
    }
    CHECK(OBSTACLE_FAULT_NO_ECHO == obstacle_get_faults());

    publish_ping(&fsm, true, OBSTACLE_CLASS_GLITCH, ping++);
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & health_mask));
    CHECK(OBSTACLE_FAULT_NO_ECHO == obstacle_get_faults());
    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(OBSTACLE_STATE_STOPPED == status.state);

    publish_ping(&fsm, true, OBSTACLE_CLASS_VALID, ping);
    CHECK(0U == (g_level_faults & health_mask));
    CHECK(0U == obstacle_get_faults());
    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(OBSTACLE_STATE_MONITOR == status.state);
}

static void observe_legacy_transition (const obstacle_legacy_transition_t * p_transition)
{
    obstacle_comms_scalars_t current;

    obstacle_comms_copy_scalars(&current);
    CHECK(p_transition->state == obstacle_pack_get(current.status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK));
    g_transition_count++;
    g_last_state = p_transition->state;
    if (p_transition->p_completed_profile != NULL)
    {
        g_completed_count++;
        g_finished_plan = p_transition->plan;
        g_finished_centre = p_transition->p_completed_profile[OBSTACLE_SLOT_CENTRE];
    }
}

static void test_legacy_plan_before_navigation (void)
{
    obstacle_links_t const links = {.h_nav_mbf = 2};
    obstacle_plan_cfg_t cfg;
    obstacle_statechart_t fsm;
    obstacle_statechart_out_t out;
    uint8_t const request = 3U;

    CHECK(E_OK == obstacle_init(&links));
    g_b_observing_legacy = true;
    obstacle_bind_legacy_transition(observe_legacy_transition);
    obstacle_plan_cfg_default(&cfg);
    obstacle_statechart_init(&fsm, &cfg, 0U);
    obstacle_statechart_out_clear(&out);
    out.b_profile_changed = true;
    obstacle_comms_publish(&out, &fsm);
    CHECK(g_transition_count == 0U);

    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_RESET, 0U), 1U, 0U, &out);
    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_START, 0U), 2U, 0U, &out);
    obstacle_comms_publish(&out, &fsm);
    for (uint32_t hit = 0U; hit < OBSTACLE_DETECT_HITS; ++hit)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_VALID, 200U, 10U + hit, &out);
        obstacle_comms_publish(&out, &fsm);
    }
    CHECK(OBSTACLE_STATE_WAIT_STOP == g_last_state);

    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_SCAN_REQ, request), 30U, 0U, &out);
    obstacle_comms_publish(&out, &fsm);
    CHECK(g_completed_count == 0U);
    for (uint32_t i = 0U; i < 100U && fsm.state == OBSTACLE_STATE_SCANNING; ++i)
    {
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping(&fsm, OBSTACLE_CLASS_OPEN, 0U, 40U + i, &out);
        obstacle_comms_publish(&out, &fsm);
    }
    CHECK(fsm.state == OBSTACLE_STATE_PLAN);
    CHECK(g_completed_count == 1U && g_scan_handoffs == 1U);
    CHECK(g_finished_plan.req_id == request && g_finished_plan.status == OBSTACLE_RESULT_CLEAR);
    CHECK(g_finished_centre == obstacle_pack_slot(0U, OBSTACLE_CLASS_OPEN));

    /* Acceptance moves a clear plan directly to MONITOR before a poll. */
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&fsm, obstacle_pack_cmd(OBSTACLE_CMD_ACCEPT, request), 141U, 0U, &out);
    obstacle_comms_publish(&out, &fsm);
    obstacle_comms_publish(&out, &fsm);
    CHECK(fsm.state == OBSTACLE_STATE_MONITOR && g_last_state == OBSTACLE_STATE_MONITOR);
    CHECK(g_transition_count == 5U && g_completed_count == 1U);
}

int main (int argc, char ** argv)
{
    if (argc > 1 && strcmp(argv[1], "legacy-plan-source") == 0)
    {
        test_legacy_plan_before_navigation();
    }
    else
    {
        CHECK(E_OK == obstacle_init(NULL));
        test_scalars_read_as_published();
        test_full_command_queue_counts_drop();
        test_parked_echo_faults(true);
        test_parked_echo_faults(false);
        test_pilot_echo_stop_needs_usable_ping();
    }

    if (0 != g_failures)
    {
        (void)fprintf(stderr, "test_obstacle_comms: %d failure(s)\n", g_failures);
        return 1;
    }
    (void)printf("test_obstacle_comms: all passed\n");
    return 0;
}
