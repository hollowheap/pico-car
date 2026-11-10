/** @file test_obstacle_plan.c
 *
 * @brief Host tests for obstacle planner maths, geometry and steering rules (D96, S40).
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

/* Scan description: one class and range per slot, fed through the real
 * scan engine so that the edge fields are what the firmware produces. */
typedef struct
{
    uint8_t  cls[OBSTACLE_SLOT_COUNT];
    uint16_t dist[OBSTACLE_SLOT_COUNT];
} slot_map_t;

static void map_fill(slot_map_t * p_map, uint8_t first, uint8_t last,
                     uint8_t cls, uint16_t dist_mm)
{
    uint8_t slot = 0U;

    for (slot = first; slot <= last; slot++)
    {
        p_map->cls[slot]  = cls;
        p_map->dist[slot] = dist_mm;
    }
}

static void map_open(slot_map_t * p_map)
{
    map_fill(p_map, 0U, (uint8_t)(OBSTACLE_SLOT_COUNT - 1U), OBSTACLE_CLASS_OPEN, 0U);
}

static void run_scan(scan_algo_ctx_t * p_ctx, slot_map_t const * p_map)
{
    uint32_t guard = 0U;

    scan_algo_start(p_ctx);
    while ((scan_algo_target(p_ctx) != SCAN_ALGO_NO_SLOT) && (guard < 400U))
    {
        uint8_t const slot = scan_algo_target(p_ctx);

        if (!scan_algo_feed(p_ctx, p_map->cls[slot], p_map->dist[slot]))
        {
            (void)scan_algo_feed(p_ctx, p_map->cls[slot], p_map->dist[slot]);
        }
        guard++;
    }
    CHECK(scan_algo_target(p_ctx) == SCAN_ALGO_NO_SLOT);
}

/* Fallback sideways offset for a wall: the largest need plan_side can
 * accept, free floor minus half the car width. */
#define WALL_NEED_MM (OBSTACLE_FREE_FLOOR_MM - OBSTACLE_CAR_HALF_W_MM)

static void test_build_bounded_unchanged(void)
{
    scan_algo_ctx_t        ctx;
    obstacle_plan_cfg_t    cfg;
    obstacle_plan_result_t result;
    slot_map_t             map;

    obstacle_plan_cfg_default(&cfg);
    map_open(&map);
    map_fill(&map, 15U, 21U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);

    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_LEFT);
    CHECK(result.width_mm == 82U);
    CHECK(result.start_deg == 45U);
    CHECK(result.closest_mm == 300U);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == 245U);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == 245U);
    CHECK(!result.b_wall);

    /* A wider obstacle on the right: the left side needs less. */
    map_open(&map);
    map_fill(&map, 11U, 19U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_LEFT);
    CHECK(!result.b_wall);

    /* Mirror image: the right side needs less. */
    map_open(&map);
    map_fill(&map, 17U, 25U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_RIGHT);
    CHECK(!result.b_wall);
}

static void test_build_wall(void)
{
    scan_algo_ctx_t        ctx;
    obstacle_plan_cfg_t    cfg;
    obstacle_plan_result_t result;
    slot_map_t             map;

    obstacle_plan_cfg_default(&cfg);

    /* A wall: every slot is a near hit, so neither edge is bounded. The
     * plan passes on the left with the fallback offset and never reports a
     * measured width. */
    map_fill(&map, 0U, (uint8_t)(OBSTACLE_SLOT_COUNT - 1U), OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_UNBOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_LEFT);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == WALL_NEED_MM);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(result.width_mm == OBSTACLE_WIDTH_MAX);
    CHECK(result.closest_mm == 300U);
    CHECK(result.start_deg == 90U);
    CHECK(result.b_wall);

    /* Left end of the sweep cannot be read and the right edge is bounded:
     * the bounded right edge takes the pass. */
    map_open(&map);
    map_fill(&map, 14U, 35U, OBSTACLE_CLASS_VALID, 300U);
    map_fill(&map, 36U, 36U, OBSTACLE_CLASS_UNKNOWN, 0U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_BOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_RIGHT);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] != OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(result.width_mm == OBSTACLE_WIDTH_MAX);
    CHECK(!result.b_wall);

    /* Left blocked and the right edge also unbounded: no safe path. */
    map_fill(&map, 0U, 35U, OBSTACLE_CLASS_VALID, 300U);
    map_fill(&map, 36U, 36U, OBSTACLE_CLASS_UNKNOWN, 0U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_UNBOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_NO_SAFE_PATH);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(!result.b_wall);

    /* A STUCK reading after the last hit blocks the left corridor too. */
    map_fill(&map, 0U, 35U, OBSTACLE_CLASS_VALID, 300U);
    map_fill(&map, 36U, 36U, OBSTACLE_CLASS_STUCK, 0U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_UNBOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_NO_SAFE_PATH);
}

/* A scan with no region (CLEAR) is not a wall plan. */
static void test_wall_flag_clear(void)
{
    scan_algo_ctx_t        ctx;
    obstacle_plan_cfg_t    cfg;
    obstacle_plan_result_t result;
    slot_map_t             map;

    obstacle_plan_cfg_default(&cfg);
    map_open(&map);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_CLEAR);
    CHECK(!result.b_wall);
}

static void test_wheel_pivot(void)
{
    int32_t left_um_s  = 0;
    int32_t right_um_s = 0;
    uint8_t steer      = 0U;
    bool    b_done     = false;

    /* On target heading (within tolerance): stops and sets HOLD. */
    obstacle_plan_wheel_pivot(0, 10, &left_um_s, &right_um_s, &steer, &b_done);
    CHECK(b_done);
    CHECK(left_um_s == 0);
    CHECK(right_um_s == 0);
    CHECK(steer == OBSTACLE_STEER_HOLD);

    /* Turn left: right forward, left backward. */
    obstacle_plan_wheel_pivot(0, 600, &left_um_s, &right_um_s, &steer, &b_done);
    CHECK(!b_done);
    CHECK(steer == OBSTACLE_STEER_LEFT);
    CHECK(left_um_s < 0);
    CHECK(right_um_s > 0);
    CHECK(right_um_s == -left_um_s);
    CHECK(right_um_s == ((int32_t)OBSTACLE_PIVOT_SPEED_MM_S * 1000));

    /* Turn right: left forward, right backward. */
    obstacle_plan_wheel_pivot(0, -600, &left_um_s, &right_um_s, &steer, &b_done);
    CHECK(!b_done);
    CHECK(steer == OBSTACLE_STEER_RIGHT);
    CHECK(left_um_s > 0);
    CHECK(right_um_s < 0);
    CHECK(left_um_s == -right_um_s);

    /* Speed ramp down inside ramp zone: at 100 deci-deg error vs 450 ramp zone */
    obstacle_plan_wheel_pivot(0, 100, &left_um_s, &right_um_s, &steer, &b_done);
    CHECK(!b_done);
    CHECK(right_um_s < ((int32_t)OBSTACLE_PIVOT_SPEED_MM_S * 1000));
    CHECK(right_um_s >= ((int32_t)OBSTACLE_PIVOT_MIN_MM_S * 1000));
}

static void test_wheel_straight(void)
{
    int32_t left_um_s  = 0;
    int32_t right_um_s = 0;
    uint8_t steer      = 0U;

    /* On heading: straight with equal wheel speeds. */
    obstacle_plan_wheel_straight(0, 0, 140, &left_um_s, &right_um_s, &steer);
    CHECK(steer == OBSTACLE_STEER_STRAIGHT);
    CHECK(left_um_s == 140000);
    CHECK(right_um_s == 140000);

    /* Heading error positive (car points right of target, needs left turn). */
    obstacle_plan_wheel_straight(0, 100, 140, &left_um_s, &right_um_s, &steer);
    CHECK(steer == OBSTACLE_STEER_LEFT);
    CHECK(left_um_s < 140000);
    CHECK(right_um_s > 140000);

    /* Large error capped at max differential (40 mm/s = 40000 um/s). */
    obstacle_plan_wheel_straight(0, 500, 140, &left_um_s, &right_um_s, &steer);
    CHECK(steer == OBSTACLE_STEER_LEFT);
    CHECK(left_um_s == (140000 - 40000));
    CHECK(right_um_s == (140000 + 40000));
}

static void test_side_lean_and_trim(void)
{
    int32_t lean = 0;
    int32_t trim = 0;

    /* Exactly on target gap: zero lean. */
    lean = obstacle_plan_side_lean_deci(190, 190, false, 190);
    CHECK(lean == 0);

    /* Too close (gap 100 mm < target 190 mm): positive lean away from obstacle. */
    lean = obstacle_plan_side_lean_deci(100, 100, false, 190);
    CHECK(lean > 0);
    CHECK(lean <= OBSTACLE_LEAN_MAX_DECI);

    /* Too far (gap 300 mm > target 190 mm): negative lean towards obstacle. */
    lean = obstacle_plan_side_lean_deci(300, 300, false, 190);
    CHECK(lean < 0);
    CHECK(lean >= -OBSTACLE_LEAN_MAX_DECI);

    /* Rate of change damping: gap grew by 20 mm compared to previous check. */
    lean = obstacle_plan_side_lean_deci(100, 80, true, 190);
    CHECK(lean > 0);

    /* Trim calculation: change in gap relative to prev lean. */
    trim = obstacle_plan_side_trim_deci(190, 190, 0);
    CHECK(trim == 0);
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
    CHECK(obstacle_plan_hug_limit_mm(&result, &cfg) >= OBSTACLE_HUG_MIN_MM);
    CHECK(obstacle_plan_hug_limit_mm(&result, &cfg) <= OBSTACLE_HUG_MAX_MM);
}

int main(void)
{
    test_trig_and_roots();
    test_config_and_steering();
    test_escape_clear();
    test_build_clear_and_unsafe();
    test_build_bounded_unchanged();
    test_build_wall();
    test_wall_flag_clear();
    test_wheel_pivot();
    test_wheel_straight();
    test_side_lean_and_trim();
    test_hug_limit();

    if (g_failures == 0)
    {
        printf("obstacle_plan: all checks passed\n");
        return 0;
    }
    printf("obstacle_plan: %d check(s) failed\n", g_failures);
    return 1;
}
