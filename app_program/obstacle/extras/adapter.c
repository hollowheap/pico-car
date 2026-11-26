/** @file adapter.c
 * @brief Share the extras scanner's state with the supervisor and telemetry.
 */

#include "comms.h"
#include "obstacle/obstacle.h"
#include "obstacle/obstacle_pack.h"
#include "platform/drv_timer.h"

STATIC_ASSERT(OBSTACLE_CMD_STOP == EXTRAS_CMD_STOP &&
              OBSTACLE_CMD_RESET == EXTRAS_CMD_RESET &&
              OBSTACLE_CMD_START == EXTRAS_CMD_START, "obstacle command IDs");

ER obstacle_post_cmd (uint8_t cmd)
{
    return obstacle_extras_post_cmd(cmd);
}

ER obstacle_snapshot (obstacle_status_t * p_status)
{
    if (NULL == p_status)
    {
        return E_PAR;
    }
    if (!obstacle_extras_ready())
    {
        return E_OBJ;
    }

    uint32_t const status = obstacle_extras_get_status();
    uint32_t const ping = extras_comms_ping();

    *p_status = (obstacle_status_t){
        .distance_mm = obstacle_pack_ping_mm(ping),
        .faults = obstacle_extras_get_faults(),
        .servo_deci = (0U != ping) ? (int16_t)obstacle_pack_ping_servo(ping) : 0,
        .state = (uint8_t)extras_pack_get(status, EXTRAS_ST_STATE_SH, EXTRAS_ST_STATE_MK),
        .cause = (uint8_t)extras_pack_get(status, EXTRAS_ST_CAUSE_SH, EXTRAS_ST_CAUSE_MK),
        .echo_class = (0U != ping) ? (uint8_t)obstacle_pack_get(ping, OBSTACLE_PING_CLASS_SH,
                                                              OBSTACLE_PING_CLASS_MK)
                                  : OBSTACLE_CLASS_NO_ECHO,
        .req_id = (uint8_t)extras_pack_get(status, EXTRAS_ST_ID_SH, EXTRAS_ST_ID_MK),
        .seq = obstacle_pack_ping_seq(ping),
        .dropped_events = obstacle_extras_dropped_events(),
        .ready = true,
    };
    return E_OK;
}

ER obstacle_copy_plan (obstacle_plan_summary_t * p_plan)
{
    if (NULL == p_plan)
    {
        return E_PAR;
    }
    return obstacle_extras_ready() ? extras_comms_copy_plan(p_plan) : E_OBJ;
}

void obstacle_echo_counts (uint32_t * p_counts)
{
    if ((NULL != p_counts) && obstacle_extras_ready())
    {
        extras_comms_echo_counts(p_counts);
    }
}

uint32_t obstacle_ping_read (void)
{
    return extras_comms_ping();
}

uint32_t obstacle_ping_recent (void)
{
    return obstacle_extras_ready() ? extras_comms_ping_recent(now_us()) : 0U;
}

ER obstacle_forward_snapshot (obstacle_forward_sample_t * p_sample)
{
    if (NULL == p_sample)
    {
        return E_PAR;
    }
    return obstacle_extras_ready() ? extras_comms_forward_snapshot(p_sample) : E_OBJ;
}

void obstacle_bind_sonar_aim (obstacle_sonar_aim_fn_t p_observe)
{
    extras_comms_bind_aim(p_observe);
}

ER obstacle_ping_request (int16_t servo_deci, uint8_t seq)
{
    /* The extras statechart and its navigator own the scan bearings. */
    (void)servo_deci;
    (void)seq;
    return E_NOSPT;
}
