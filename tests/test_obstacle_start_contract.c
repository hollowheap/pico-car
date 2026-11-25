/* Drive the real navigation adapter with a synthetic clock and obstacle status. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <tm/tmonitor.h>

#include "comms.h"
#include "flight_log.h"
#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "obstacle_pack.h"

#define WAIT_MS (5000U)
#define CHECK(condition)                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (!(condition))                                                                   \
        {                                                                                   \
            (void)fprintf(stderr, "%s:%d: %s failed\n", __FILE__, __LINE__, #condition);        \
            abort();                                                                        \
        }                                                                                   \
    } while (0)

static uint32_t g_now;
static uint32_t g_status;
static uint32_t g_fault_event;
static uint32_t g_aim;
static unsigned g_commands;
static unsigned g_failures;
static bool g_obstacle_hold;
static bool g_scan_pivot;

ER tk_get_otm (SYSTIM * p_time)
{
    *p_time = (SYSTIM){0, g_now};
    return E_OK;
}

ER obstacle_init (obstacle_links_t const * p_links)
{
    CHECK(p_links->p_get_distance_mm != NULL);
    CHECK(p_links->p_stop != NULL);
    return E_OK;
}

ER obstacle_post_cmd (uint8_t cmd)
{
    uint32_t const type = obstacle_pack_get(cmd, OBSTACLE_CMD_TYPE_SH, OBSTACLE_CMD_TYPE_MK);

    CHECK(type == OBSTACLE_CMD_RESET || type == OBSTACLE_CMD_START);
    g_commands++;
    return E_OK;
}

uint32_t obstacle_get_status (void)
{
    return g_status;
}

uint32_t obstacle_get_steer (void)
{
    return 0U;
}

uint16_t obstacle_get_faults (void)
{
    return 0U;
}

ER obstacle_copy_profile (uint16_t * p_profile)
{
    (void)p_profile;
    return E_OBJ;
}

ER obstacle_copy_plan (obstacle_plan_summary_t * p_plan)
{
    (void)p_plan;
    return E_OBJ;
}

ER obstacle_comms_get_event (uint32_t * p_evt, TMO tmout)
{
    CHECK(tmout == TMO_POL);
    if (g_fault_event == 0U)
    {
        return E_TMOUT;
    }
    *p_evt = g_fault_event;
    g_fault_event = 0U;
    return E_OK;
}

uint32_t obstacle_comms_aim (void)
{
    return g_aim;
}

uint32_t obstacle_comms_ping (void)
{
    return 0U;
}

int16_t obstacle_comms_look (void)
{
    return 0;
}

void obstacle_comms_set_look (int16_t deci)
{
    (void)deci;
}

void obstacle_comms_set_pivoting (bool b_pivoting)
{
    CHECK(!b_pivoting || (0U != g_aim));
}

int motion_set_hold (uint32_t reason, bool held)
{
    CHECK(reason == MOTION_HOLD_OBSTACLE);
    g_obstacle_hold = held;
    return E_OK;
}

int motion_set_speed (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    (void)ttl_us;
    CHECK(left_um_s == 0 && right_um_s == 0);
    return E_OK;
}

int motion_set_scan_pivot (int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    CHECK(0U != ttl_us);
    CHECK((left_um_s < 0) && (right_um_s == -left_um_s));
    g_scan_pivot = true;
    return E_OK;
}

int motion_status (motion_status_t * p_status)
{
    *p_status = (motion_status_t){.ready = true, .hold_mask = g_obstacle_hold ? MOTION_HOLD_OBSTACLE : 0U};
    return E_OK;
}

int motion_feedback_snapshot (motion_feedback_t * p_feedback)
{
    *p_feedback = (motion_feedback_t){.fresh = true};
    return E_OK;
}

void flight_log_add (flight_log_rec_t const * p_rec)
{
    (void)p_rec;
}

uint32_t flight_log_profile (uint8_t req_id, uint16_t const * p_profile)
{
    (void)req_id;
    (void)p_profile;
    return 0U;
}

void flight_log_service (void)
{
}

void tm_printf (const UB * p_format, ...)
{
    (void)p_format;
}

static void select_status (uint32_t state, uint32_t cause)
{
    g_status = obstacle_pack_put(0U, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK, state);
    g_status = obstacle_pack_put(g_status, OBSTACLE_ST_CAUSE_SH, OBSTACLE_ST_CAUSE_MK, cause);
    g_status = obstacle_pack_put(g_status, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK, OBSTACLE_CLASS_OPEN);
}

static void check_no_command (const char * p_stage, uint32_t cause)
{
    if (g_commands != 0U)
    {
        (void)fprintf(stderr, "%s for cause %u posted %u RESET/START commands\n", p_stage, cause, g_commands);
        g_failures++;
        g_commands = 0U;
    }
}

int main (void)
{
    select_status(OBSTACLE_STATE_STOPPED, OBSTACLE_CAUSE_BOOT);
    CHECK(obstacle_line_init() == E_OK);
    check_no_command("Navigation init", OBSTACLE_CAUSE_BOOT);

    g_now += WAIT_MS;
    CHECK(obstacle_line_step(true, true));
    CHECK(g_obstacle_hold);
    check_no_command("Parked at boot", OBSTACLE_CAUSE_BOOT);

    /* The accepted supervisor START puts the obstacle statechart in MONITOR. */
    select_status(OBSTACLE_STATE_MONITOR, OBSTACLE_CAUSE_NONE);
    (void)obstacle_line_step(true, true);
    g_now += WAIT_MS;
    CHECK(!obstacle_line_step(true, true));
    CHECK(!g_obstacle_hold);

    select_status(OBSTACLE_STATE_SCANNING, OBSTACLE_CAUSE_NONE);
    g_aim = 600U << 16U;
    CHECK(obstacle_line_step(true, true));
    CHECK(g_scan_pivot);
    CHECK(!g_obstacle_hold);
    g_aim = 0U;

    for (uint32_t cause = 0U; cause < OBSTACLE_CAUSE_COUNT; cause++)
    {
        select_status(OBSTACLE_STATE_STOPPED, cause);
        g_fault_event = obstacle_pack_put(0U, OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK, OBSTACLE_EVT_FAULT);
        g_fault_event = obstacle_pack_put(g_fault_event, OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK, cause);

        CHECK(obstacle_line_step(true, true));
        g_now += WAIT_MS;
        CHECK(obstacle_line_step(true, true));
        CHECK(g_obstacle_hold);
        check_no_command("Stopped obstacle navigator", cause);
    }
    return (g_failures == 0U) ? EXIT_SUCCESS : EXIT_FAILURE;
}
