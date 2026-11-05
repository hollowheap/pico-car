/** @file test_planner.c
 *
 * @brief Host tests for Buddy 5 planner maths and steering rules.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "scan_algo.h"

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

static void test_trig_and_roots(void)
{
    CHECK(obstacle_plan_sin_q14(0) == 0);
    CHECK(obstacle_plan_sin_q14(900) == 16384);
    CHECK(obstacle_plan_sin_q14(-900) == -16384);
    CHECK(obstacle_plan_cos_q14(0) == 16384);
    CHECK(obstacle_plan_cos_q14(900) == 0);
    CHECK(obstacle_plan_tan_q14(0) == 0);
    CHECK(obstacle_plan_tan_q14(450) == 16384);

    CHECK(obstacle_plan_atan_deci(0U, 10U) == 0);
    CHECK((obstacle_plan_atan_deci(10U, 10U) >= 450) &&
          (obstacle_plan_atan_deci(10U, 10U) <= 460));
    CHECK(obstacle_plan_atan_deci(10U, 0U) == 600);

    CHECK(obstacle_plan_isqrt(0U) == 0U);
    CHECK(obstacle_plan_isqrt(1U) == 1U);
    CHECK(obstacle_plan_isqrt(4U) == 2U);
    CHECK(obstacle_plan_isqrt(15U) == 3U);
    CHECK(obstacle_plan_isqrt(16U) == 4U);
    CHECK(obstacle_plan_isqrt(100U) == 10U);
}

static void test_config_and_steering(void)
{
    obstacle_plan_cfg_t cfg;

    obstacle_plan_cfg_default(&cfg);
    CHECK(cfg.half_width_mm == OBSTACLE_CAR_HALF_W_MM);
    CHECK(cfg.side_margin_mm == OBSTACLE_SIDE_MARGIN_MM);
    CHECK(cfg.free_floor_mm == OBSTACLE_FREE_FLOOR_MM);
    CHECK(cfg.depth_mm == OBSTACLE_DEPTH_ASSUMED_MM);
    CHECK(cfg.rear_mm == OBSTACLE_SERVO_TO_REAR_MM);
    CHECK(cfg.beam_half_deci == OBSTACLE_BEAM_HALF_DECI);
    CHECK(cfg.path_half_deci == OBSTACLE_PATH_HALF_DECI);

    CHECK(obstacle_plan_steer_away(OBSTACLE_SIDE_LEFT) == OBSTACLE_STEER_LEFT);
    CHECK(obstacle_plan_steer_away(OBSTACLE_SIDE_RIGHT) == OBSTACLE_STEER_RIGHT);
    CHECK(obstacle_plan_steer_toward(OBSTACLE_SIDE_LEFT) == OBSTACLE_STEER_RIGHT);
    CHECK(obstacle_plan_steer_toward(OBSTACLE_SIDE_RIGHT) == OBSTACLE_STEER_LEFT);

    CHECK(obstacle_plan_hug_steer(OBSTACLE_SIDE_LEFT, OBSTACLE_CLASS_VALID, 200U,
                            OBSTACLE_STEER_STRAIGHT) == OBSTACLE_STEER_LEFT);
    CHECK(obstacle_plan_hug_steer(OBSTACLE_SIDE_LEFT, OBSTACLE_CLASS_VALID, 300U,
                            OBSTACLE_STEER_LEFT) == OBSTACLE_STEER_STRAIGHT);
    CHECK(obstacle_plan_hug_steer(OBSTACLE_SIDE_LEFT, OBSTACLE_CLASS_VALID, 400U,
                            OBSTACLE_STEER_STRAIGHT) == OBSTACLE_STEER_RIGHT);
    CHECK(obstacle_plan_hug_steer(OBSTACLE_SIDE_LEFT, OBSTACLE_CLASS_OPEN, 0U,
                            OBSTACLE_STEER_STRAIGHT) == OBSTACLE_STEER_RIGHT);
    CHECK(obstacle_plan_hug_steer(OBSTACLE_SIDE_LEFT, OBSTACLE_CLASS_UNKNOWN, 0U,
                            OBSTACLE_STEER_HOLD) == OBSTACLE_STEER_HOLD);
}

static void test_escape_clear(void)
{
    CHECK(obstacle_plan_is_escape_clear(OBSTACLE_CLASS_OPEN, 0U, 200U));
    CHECK(obstacle_plan_is_escape_clear(OBSTACLE_CLASS_VALID, 500U, 200U));
    CHECK(!obstacle_plan_is_escape_clear(OBSTACLE_CLASS_VALID, 300U, 200U));
    CHECK(!obstacle_plan_is_escape_clear(OBSTACLE_CLASS_UNKNOWN, 0U, 200U));
    CHECK(!obstacle_plan_is_escape_clear(OBSTACLE_CLASS_STUCK, 0U, 200U));
}

static void fill_scan(scan_algo_ctx_t * p_ctx, uint8_t cls)
{
    uint8_t slot = 0U;

    for (slot = 0U; slot < OBSTACLE_SLOT_COUNT; slot++)
    {
        p_ctx->profile[slot] = obstacle_pack_slot(0U, cls);
    }
    p_ctx->closest_mm = 0U;
}

static void test_build_clear_and_unsafe(void)
{
    scan_algo_ctx_t ctx;
    obstacle_plan_cfg_t   cfg;
    obstacle_plan_result_t result;

    obstacle_plan_cfg_default(&cfg);
    scan_algo_start(&ctx);
    fill_scan(&ctx, OBSTACLE_CLASS_OPEN);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_CLEAR);

    scan_algo_start(&ctx);
    fill_scan(&ctx, OBSTACLE_CLASS_OPEN);
    ctx.profile[OBSTACLE_SLOT_CENTRE] = obstacle_pack_slot(0U, OBSTACLE_CLASS_UNKNOWN);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_NO_SAFE_PATH);
}

static void test_hug_ctrl(void)
{
    obstacle_hug_cmd_t cmd;

    obstacle_plan_hug_ctrl(OBSTACLE_CLASS_VALID, OBSTACLE_HUG_TARGET_MM, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);

    obstacle_plan_hug_ctrl(OBSTACLE_CLASS_VALID,
                           (uint16_t)(OBSTACLE_HUG_TARGET_MM -
                                      OBSTACLE_HUG_DEADBAND_MM - 30U),
                           &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille == 700U);

    obstacle_plan_hug_ctrl(OBSTACLE_CLASS_VALID,
                           (uint16_t)(OBSTACLE_HUG_TARGET_MM -
                                      OBSTACLE_HUG_DEADBAND_MM - 80U),
                           &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille == 200U);

    obstacle_plan_hug_ctrl(OBSTACLE_CLASS_VALID,
                           (uint16_t)(OBSTACLE_HUG_TARGET_MM +
                                      OBSTACLE_HUG_DEADBAND_MM + 30U),
                           &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    CHECK(cmd.inside_permille == 700U);

    obstacle_plan_hug_ctrl(OBSTACLE_CLASS_OPEN, 0U, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    CHECK(cmd.inside_permille == 200U);

    obstacle_plan_hug_ctrl(OBSTACLE_CLASS_UNKNOWN, 100U, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);
}

static void test_hug_limit(void)
{
    obstacle_plan_cfg_t    cfg;
    obstacle_plan_result_t result = {0};

    obstacle_plan_cfg_default(&cfg);

    result.closest_mm = 250U;
    result.width_mm   = 300U;
    result.side       = OBSTACLE_SIDE_LEFT;
    result.need_mm[OBSTACLE_SIDE_LEFT]  = 100U;
    result.need_mm[OBSTACLE_SIDE_RIGHT] = 100U;
    CHECK(obstacle_plan_hug_limit_mm(&result, &cfg) == OBSTACLE_HUG_MIN_MM);

    result.width_mm = 900U;
    result.need_mm[OBSTACLE_SIDE_LEFT] = 400U;
    CHECK(obstacle_plan_hug_limit_mm(&result, &cfg) == 3455U);

    result.width_mm = OBSTACLE_WIDTH_MAX;
    result.need_mm[OBSTACLE_SIDE_LEFT] = OBSTACLE_PLAN_SIDE_BLOCKED;
    CHECK(obstacle_plan_hug_limit_mm(&result, &cfg) == 3246U);
}

int main(void)
{
    test_trig_and_roots();
    test_config_and_steering();
    test_escape_clear();
    test_build_clear_and_unsafe();
    test_hug_ctrl();
    test_hug_limit();

    if (g_failures == 0)
    {
        printf("obstacle_plan: all checks passed\n");
        return 0;
    }
    printf("obstacle_plan: %d check(s) failed\n", g_failures);
    return 1;
}
