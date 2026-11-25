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
#include "sonar_health.h"
#include "comms.h"
#include "scan_hw.h"
#include "scan_task.h"
#include "navigation/obstacle.h"
#include "platform/faults.h"

static int g_failures = 0;
static uint32_t g_level_faults;

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

ER scan_hw_init (ID h_flag)
{
    (void)h_flag;
    return E_OK;
}

void scan_task_obstacle_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
}

void system_fault_assign (fault_owner_t owner, uint32_t owned_mask, uint32_t active_mask)
{
    (void)owner;
    g_level_faults = (g_level_faults & ~owned_mask) | (active_mask & owned_mask);
}

/* ---- Tests ---- */

static void make_step (sonar_health_t * p_health)
{
    sonar_health_init(p_health);
    p_health->state      = OBSTACLE_STATE_MONITOR;
    p_health->cause      = OBSTACLE_CAUSE_NO_ECHO;
    p_health->last_class = OBSTACLE_CLASS_VALID;
    p_health->last_dist  = 345U;
    p_health->seq        = 5U;
    p_health->look_deci  = -123;
    p_health->faults     = (uint16_t)(OBSTACLE_FAULT_NO_ECHO | OBSTACLE_FAULT_ECHO_STUCK);
}

static void test_scalars_read_as_published (void)
{
    sonar_health_t           health;
    obstacle_comms_scalars_t pub;
    obstacle_status_t        status;
    uint16_t const           faults = (uint16_t)(OBSTACLE_FAULT_NO_ECHO | OBSTACLE_FAULT_ECHO_STUCK);

    make_step(&health);
    obstacle_comms_publish(&health);

    (void)memset(&pub, 0xA5, sizeof(pub));
    obstacle_comms_copy_scalars(&pub);
    CHECK(pub.status == sonar_health_status(&health));
    CHECK(pub.faults == faults);
    CHECK(pub.flags == 0U);
    CHECK(pub.aim == (uint32_t)(uint16_t)(int16_t)-123);
    CHECK(pub.drops == 0U);

    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(status.state == OBSTACLE_STATE_MONITOR);
    CHECK(status.cause == OBSTACLE_CAUSE_NO_ECHO);
    CHECK(status.echo_class == OBSTACLE_CLASS_VALID);
    CHECK(status.distance_mm == 345U);
    CHECK(status.seq == 5U);
    CHECK(status.faults == faults);
    CHECK(status.flags == 0U);
    CHECK(status.servo_deci == -123);
    CHECK(status.body_deci == 0);
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

static void publish_ping (sonar_health_t * p_health, uint8_t cls)
{
    uint16_t const dist = (OBSTACLE_CLASS_VALID == cls) ? 900U : 0U;

    sonar_health_ping(p_health, cls, dist, true, 0);
    obstacle_comms_publish(p_health);
}

static void test_parked_echo_faults (void)
{
    sonar_health_t health;
    uint32_t const mask = FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT;
    uint32_t       ping = 0U;

    sonar_health_init(&health);
    for (ping = 0U; ping < OBSTACLE_NO_ECHO_COUNT; ping++)
    {
        publish_ping(&health, OBSTACLE_CLASS_NO_ECHO);
    }
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & mask));

    sonar_health_command(&health, obstacle_pack_cmd(OBSTACLE_CMD_RESET, 0U));
    obstacle_comms_publish(&health);
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & mask));

    publish_ping(&health, OBSTACLE_CLASS_STUCK);
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & mask));
    for (uint32_t stuck = 1U; stuck < OBSTACLE_STUCK_COUNT; stuck++)
    {
        publish_ping(&health, OBSTACLE_CLASS_STUCK);
    }
    CHECK(mask == (g_level_faults & mask));
    publish_ping(&health, OBSTACLE_CLASS_VALID);
    CHECK(0U == (g_level_faults & mask));
    CHECK(OBSTACLE_STATE_STOPPED == health.state);
}

static void test_pilot_echo_stop_needs_usable_ping (void)
{
    sonar_health_t    health;
    obstacle_status_t status;
    uint32_t const    health_mask = FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT;
    uint32_t          ping        = 0U;

    sonar_health_init(&health);
    sonar_health_command(&health, obstacle_pack_cmd(OBSTACLE_CMD_RESET, 0U));
    sonar_health_command(&health, obstacle_pack_cmd(OBSTACLE_CMD_START, 0U));
    for (ping = 0U; ping < OBSTACLE_NO_ECHO_COUNT; ping++)
    {
        publish_ping(&health, OBSTACLE_CLASS_NO_ECHO);
    }
    CHECK(OBSTACLE_FAULT_NO_ECHO == obstacle_get_faults());

    publish_ping(&health, OBSTACLE_CLASS_GLITCH);
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (g_level_faults & health_mask));
    CHECK(OBSTACLE_FAULT_NO_ECHO == obstacle_get_faults());
    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(OBSTACLE_STATE_STOPPED == status.state);

    publish_ping(&health, OBSTACLE_CLASS_VALID);
    CHECK(0U == (g_level_faults & health_mask));
    CHECK(0U == obstacle_get_faults());
    CHECK(E_OK == obstacle_snapshot(&status));
    CHECK(OBSTACLE_STATE_MONITOR == status.state);
}

int main (void)
{
    CHECK(E_OK == obstacle_setup(NULL));
    test_scalars_read_as_published();
    test_full_command_queue_counts_drop();
    test_parked_echo_faults();
    test_pilot_echo_stop_needs_usable_ping();

    if (0 != g_failures)
    {
        (void)fprintf(stderr, "test_obstacle_comms: %d failure(s)\n", g_failures);
        return 1;
    }
    (void)printf("test_obstacle_comms: all passed\n");
    return 0;
}
