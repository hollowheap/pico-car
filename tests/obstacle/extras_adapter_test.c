#include <assert.h>

#include "obstacle/extras/comms.h"
#include "obstacle/obstacle_pack.h"

static uint32_t g_test_clock_us;

uint32_t shim_reg_read (uint32_t address)
{
    assert(TIMER_TIMERAWL == address);
    return g_test_clock_us;
}

static void test_stop_order (void)
{
    uint8_t cmd = 0U;

    assert(E_OK == obstacle_post_cmd(OBSTACLE_CMD_RESET));
    assert(E_OK == obstacle_post_cmd(OBSTACLE_CMD_START));
    assert(E_OK == obstacle_post_cmd(OBSTACLE_CMD_STOP));
    assert(!extras_comms_pop_cmd(&cmd));

    UINT pattern = 0U;
    assert(E_OK == tk_wai_flg(extras_comms_flag(), EXTRAS_FLG_STOP, TWF_ORW | TWF_BITCLR,
                               &pattern, TMO_POL));
    assert(0U != (pattern & EXTRAS_FLG_STOP));
    assert(E_OK == obstacle_post_cmd(OBSTACLE_CMD_RESET));
    assert(E_OK == obstacle_post_cmd(OBSTACLE_CMD_START));
    assert(extras_comms_pop_cmd(&cmd) && (EXTRAS_CMD_RESET == cmd));
    assert(extras_comms_pop_cmd(&cmd) && (EXTRAS_CMD_START == cmd));
}

static void test_scanner_observations (void)
{
    obstacle_status_t status = {0};
    obstacle_forward_sample_t forward = {0};
    uint32_t counts[OBSTACLE_ECHO_CLASSES] = {0U};

    assert(0U == obstacle_ping_recent());
    extras_comms_publish_ping(17U, -480, EXTRAS_CLASS_STUCK, 0U, 1000U);
    assert(E_OBJ == obstacle_forward_snapshot(&forward));
    assert(E_OK == obstacle_snapshot(&status));
    assert(status.ready && (17U == status.seq) && (-480 == status.servo_deci));
    assert(OBSTACLE_CLASS_STUCK == status.echo_class);

    uint32_t const trigger = UINT32_MAX - 1000U;
    extras_comms_publish_ping(18U, 0, EXTRAS_CLASS_VALID, 650U, trigger);
    uint32_t const ping = obstacle_ping_read();
    assert(!obstacle_pack_ping_is_reply(ping));
    assert((18U == obstacle_pack_ping_seq(ping)) && (0 == obstacle_pack_ping_servo(ping)));
    assert(650U == obstacle_pack_ping_mm(ping));
    assert(E_OK == obstacle_forward_snapshot(&forward));
    assert((18U == forward.seq) && (650U == forward.distance_mm) && (trigger == forward.trigger_us));
    assert(OBSTACLE_CLASS_VALID == forward.echo_class);
    g_test_clock_us = trigger + OBSTACLE_FORWARD_MAX_AGE_US - 1U;
    assert(ping == obstacle_ping_recent());
    g_test_clock_us++;
    assert(0U == obstacle_ping_recent());
    assert(ping == obstacle_ping_read());
    obstacle_echo_counts(counts);
    assert((1U == counts[OBSTACLE_ECHO_IDX_STUCK]) && (1U == counts[OBSTACLE_ECHO_IDX_VALID]));

    extras_fsm_t fsm = {.state = EXTRAS_STATE_SCANNING};
    extras_fsm_out_t const out = {0};
    extras_comms_publish(&out, &fsm);
    assert(E_OBJ == obstacle_forward_snapshot(&forward));
    extras_comms_publish_ping(19U, 0, EXTRAS_CLASS_VALID, 123U, 3000U);
    fsm.state = EXTRAS_STATE_MONITOR;
    extras_comms_publish(&out, &fsm);
    assert(E_OBJ == obstacle_forward_snapshot(&forward));
    extras_comms_publish_ping(20U, 0, EXTRAS_CLASS_STUCK, 0U, 4000U);
    assert(E_OK == obstacle_forward_snapshot(&forward));
    assert((20U == forward.seq) && (4000U == forward.trigger_us));
    assert(OBSTACLE_CLASS_STUCK == forward.echo_class);
}

static void test_completed_plan (void)
{
    obstacle_plan_summary_t plan = {0};
    obstacle_status_t status = {0};
    extras_fsm_t const fsm = {
        .state = EXTRAS_STATE_PLAN,
        .req_id = 3U,
        .plan = {
            .status = EXTRAS_RESULT_OK,
            .side = EXTRAS_SIDE_RIGHT,
            .closest_mm = 110U,
            .width_mm = 225U,
            .need_mm = {175U, 150U},
        },
    };
    extras_fsm_out_t const out = {.b_profile_changed = true};

    assert(E_OBJ == obstacle_copy_plan(&plan));
    extras_comms_publish(&out, &fsm);
    assert((E_OK == obstacle_snapshot(&status)) && (OBSTACLE_STATE_PLAN == status.state));
    assert(E_OK == obstacle_copy_plan(&plan));
    assert((3U == plan.req_id) && (225U == plan.width_mm) && (150U == plan.need_mm));
}

int main (void)
{
    obstacle_forward_sample_t forward = {0};
    assert(E_PAR == obstacle_forward_snapshot(NULL));
    assert(E_OBJ == obstacle_forward_snapshot(&forward));
    assert(0U == obstacle_ping_recent());
    assert(E_OK == obstacle_extras_setup(NULL));
    test_stop_order();
    test_scanner_observations();
    test_completed_plan();
    return 0;
}
