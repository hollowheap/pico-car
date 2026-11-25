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
    (void)msg;
    (void)msgsz;
    (void)tmout;
    return E_OK;
}

void system_fault_assign (fault_owner_t owner, uint32_t mask, uint32_t active)
{
    (void)owner;
    (void)mask;
    (void)active;
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

int main (void)
{
    CHECK(E_OK == obstacle_init(NULL));

    test_scalars_read_as_published();
    test_full_command_queue_counts_drop();

    if (0 != g_failures)
    {
        (void)fprintf(stderr, "test_obstacle_comms: %d failure(s)\n", g_failures);
        return 1;
    }
    (void)printf("test_obstacle_comms: all passed\n");
    return 0;
}
