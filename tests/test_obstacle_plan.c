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
    map_fill(&map, 9U, 15U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);

    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_LEFT);
    CHECK(result.width_mm == 82U);
    CHECK(result.start_deg == 29U);
    CHECK(result.closest_mm == 300U);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == 168U);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == 168U);
    CHECK(!result.b_wall);

    /* A wider obstacle on the right: the left side needs less. */
    map_open(&map);
    map_fill(&map, 5U, 13U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_LEFT);
    CHECK(result.width_mm == 134U);
    CHECK(result.start_deg == 21U);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == 115U);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == 289U);
    CHECK(!result.b_wall);

    /* Mirror image: the right side needs less. */
    map_open(&map);
    map_fill(&map, 11U, 19U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_OK);
    CHECK(result.side == OBSTACLE_SIDE_RIGHT);
    CHECK(result.width_mm == 134U);
    CHECK(result.start_deg == 21U);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == 289U);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == 115U);
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
    CHECK(result.start_deg == 56U);
    CHECK(result.b_wall);
    CHECK(obstacle_plan_hug_limit_mm(&result, &cfg) == 3841U);

    /* Left end of the sweep cannot be read and the right edge is bounded:
     * the bounded right edge takes the pass (unchanged behaviour). */
    map_open(&map);
    map_fill(&map, 8U, 23U, OBSTACLE_CLASS_VALID, 300U);
    map_fill(&map, 24U, 24U, OBSTACLE_CLASS_UNKNOWN, 0U);
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
    map_fill(&map, 0U, 23U, OBSTACLE_CLASS_VALID, 300U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_UNBOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_NO_SAFE_PATH);
    CHECK(result.need_mm[OBSTACLE_SIDE_LEFT] == OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(result.need_mm[OBSTACLE_SIDE_RIGHT] == OBSTACLE_PLAN_SIDE_BLOCKED);
    CHECK(!result.b_wall);

    /* A STUCK reading after the last hit blocks the left corridor too. */
    map_fill(&map, 0U, 23U, OBSTACLE_CLASS_VALID, 300U);
    map_fill(&map, 24U, 24U, OBSTACLE_CLASS_STUCK, 0U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_UNBOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_NO_SAFE_PATH);

    /* One bounded side that is too tight keeps NO_SAFE_PATH: the fallback
     * is for a wall with no bounded edge at all. */
    map_open(&map);
    map_fill(&map, 3U, 24U, OBSTACLE_CLASS_VALID, 590U);
    run_scan(&ctx, &map);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_UNBOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_BOUNDED);
    obstacle_plan_build(&ctx, &cfg, &result);
    CHECK(result.status == OBSTACLE_RESULT_NO_SAFE_PATH);
}

/* Box-mode inputs: the latest reading equals the filtered one, and a ping that
 * is not VALID reads as OBSTACLE_SIDE_FAR_MM, as the side filter delivers it. */
static void hug_box(uint8_t cls, uint16_t mm, int32_t head_deci,
                    obstacle_hug_cmd_t * p_cmd)
{
    obstacle_hug_in_t in;

    in.cls        = cls;
    in.reading_mm = (OBSTACLE_CLASS_VALID == cls) ? mm : (uint16_t)OBSTACLE_SIDE_FAR_MM;
    in.latest_mm  = in.reading_mm;
    in.head_deci  = head_deci;
    in.b_wall     = false;
    obstacle_plan_hug_ctrl(&in, p_cmd);
}

static void hug_wall(uint8_t cls, uint16_t mm, int32_t head_deci,
                     obstacle_hug_cmd_t * p_cmd)
{
    obstacle_hug_in_t in;

    in.cls        = cls;
    in.reading_mm = (OBSTACLE_CLASS_VALID == cls) ? mm : (uint16_t)OBSTACLE_SIDE_FAR_MM;
    in.latest_mm  = in.reading_mm;
    in.head_deci  = head_deci;
    in.b_wall     = true;
    obstacle_plan_hug_ctrl(&in, p_cmd);
}

/* Today's proportional numbers still hold in box mode at heading 0. */
static void test_hug_ctrl(void)
{
    obstacle_hug_cmd_t cmd;

    hug_box(OBSTACLE_CLASS_VALID, OBSTACLE_HUG_TARGET_MM, 0, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);

    hug_box(OBSTACLE_CLASS_VALID,
            (uint16_t)(OBSTACLE_HUG_TARGET_MM - OBSTACLE_HUG_DEADBAND_MM - 30U), 0, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille == 700U);

    hug_box(OBSTACLE_CLASS_VALID,
            (uint16_t)(OBSTACLE_HUG_TARGET_MM - OBSTACLE_HUG_DEADBAND_MM - 80U), 0, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille == 200U);

    hug_box(OBSTACLE_CLASS_VALID,
            (uint16_t)(OBSTACLE_HUG_TARGET_MM + OBSTACLE_HUG_DEADBAND_MM + 30U), 0, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    CHECK(cmd.inside_permille == 700U);

    hug_box(OBSTACLE_CLASS_OPEN, 0U, 0, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    CHECK(cmd.inside_permille == 200U);

    hug_box(OBSTACLE_CLASS_UNKNOWN, 100U, 0, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);
}

/* D95: the box-mode law caps TOWARD as well as AWAY. */
static void test_hug_caps_box(void)
{
    obstacle_hug_cmd_t cmd;

    /* Too close: bends away until the heading reaches the AWAY cap. */
    hug_box(OBSTACLE_CLASS_VALID, 200U, OBSTACLE_HUG_AWAY_MAX_DECI - 10, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    hug_box(OBSTACLE_CLASS_VALID, 200U, OBSTACLE_HUG_AWAY_MAX_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);
    hug_box(OBSTACLE_CLASS_VALID, 200U, OBSTACLE_HUG_AWAY_MAX_DECI + 200, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);

    /* Far or OPEN: bends towards until the heading reaches the TOWARD cap. */
    hug_box(OBSTACLE_CLASS_OPEN, 0U, -OBSTACLE_HUG_TOWARD_MAX_DECI + 10, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    hug_box(OBSTACLE_CLASS_OPEN, 0U, -OBSTACLE_HUG_TOWARD_MAX_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    hug_box(OBSTACLE_CLASS_VALID, 900U, -OBSTACLE_HUG_TOWARD_MAX_DECI - 300, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);

    /* A turn the other way is never held back by the opposite cap. */
    hug_box(OBSTACLE_CLASS_OPEN, 0U, OBSTACLE_HUG_AWAY_MAX_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    hug_box(OBSTACLE_CLASS_VALID, 200U, -OBSTACLE_HUG_TOWARD_MAX_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
}

/* One close ping wins over the filtered value (as before), but the AWAY cap
 * still applies to it. */
static void test_hug_near_override(void)
{
    obstacle_hug_in_t  in;
    obstacle_hug_cmd_t cmd;

    in.cls        = OBSTACLE_CLASS_VALID;
    in.reading_mm = OBSTACLE_HUG_TARGET_MM;
    in.latest_mm  = OBSTACLE_HUG_NEAR_MM - 1U;
    in.head_deci  = 0;
    in.b_wall     = false;
    obstacle_plan_hug_ctrl(&in, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille == (1000U - OBSTACLE_HUG_TURN_PERMILLE));

    in.head_deci = OBSTACLE_HUG_AWAY_MAX_DECI;
    obstacle_plan_hug_ctrl(&in, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
}

/* Wall-mode inputs: the ray at -60 deg reads the wall at a slant, so the
 * distance to the wall is reading * cos(ray angle off the normal less the
 * beam half angle). At heading 90 the ray is 30 deg off the normal, the beam
 * brings the nearest echo to 20.5 deg, cos = 0.937. */
static void test_hug_wall(void)
{
    obstacle_hug_cmd_t cmd;
    int32_t            head = 0;

    /* 320 mm on the ray is 300 mm from the wall: parallel, straight on. */
    hug_wall(OBSTACLE_CLASS_VALID, 320U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    CHECK(cmd.inside_permille == 1000U);

    /* Too near the wall (234 mm): wants 99.2 deg, so it turns away (left)
     * in proportion to the heading error of 9.2 deg. */
    hug_wall(OBSTACLE_CLASS_VALID, 250U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille < 1000U);
    CHECK(cmd.inside_permille > 200U);

    /* Same distance error, bigger lean: the turn is sharper, never beyond
     * the full turn. */
    hug_wall(OBSTACLE_CLASS_VALID, 100U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    CHECK(cmd.inside_permille >= 200U);

    /* A little too far (337 mm, a real echo): leans in by 3.4 deg. */
    hug_wall(OBSTACLE_CLASS_VALID, 360U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);

    /* Very far (beyond 400 mm), OPEN or blind: parallel, never towards. */
    hug_wall(OBSTACLE_CLASS_VALID, 600U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    hug_wall(OBSTACLE_CLASS_OPEN, 0U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    hug_wall(OBSTACLE_CLASS_UNKNOWN, 0U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    hug_wall(OBSTACLE_CLASS_STUCK, 0U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);

    /* Far or OPEN with the nose turned away (heading above 90): straighten
     * by turning back towards 90, which is the only turn "towards". */
    hug_wall(OBSTACLE_CLASS_OPEN, 0U, OBSTACLE_WALL_PIVOT_DECI + 200, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
    hug_wall(OBSTACLE_CLASS_VALID, 900U, OBSTACLE_WALL_PIVOT_DECI + 200, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);

    /* Far or OPEN with the nose turned to the wall (heading below 90):
     * straighten by turning away. */
    hug_wall(OBSTACLE_CLASS_OPEN, 0U, OBSTACLE_WALL_PIVOT_DECI - 200, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);
    hug_wall(OBSTACLE_CLASS_VALID, 900U, OBSTACLE_WALL_PIVOT_DECI - 200, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_AWAY);

    /* OPEN, UNKNOWN and STUCK never turn the car towards the wall at or
     * below 90 deg (plus the deadband), anywhere in the window. */
    for (head = OBSTACLE_WALL_HEAD_MIN_DECI;
         head <= (OBSTACLE_WALL_PIVOT_DECI + OBSTACLE_WALL_HEAD_DEAD_DECI); head += 10)
    {
        hug_wall(OBSTACLE_CLASS_OPEN, 0U, head, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_TOWARD);
        hug_wall(OBSTACLE_CLASS_UNKNOWN, 0U, head, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_TOWARD);
        hug_wall(OBSTACLE_CLASS_STUCK, 0U, head, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_TOWARD);
    }
}

/* Leaning in on a real, slightly-far echo is limited: never below the lean
 * (90 deg less 16 deg) and never from a reading beyond 400 mm. */
static void test_hug_wall_lean(void)
{
    obstacle_hug_cmd_t cmd;
    int32_t const      lean_floor = OBSTACLE_WALL_PIVOT_DECI -
                               (int32_t)(OBSTACLE_HUG_SPAN_MM * OBSTACLE_WALL_LEAN_DECI_PER_MM);
    int32_t            head = 0;
    uint16_t           mm   = 0U;

    for (head = OBSTACLE_WALL_HEAD_MIN_DECI; head <= OBSTACLE_WALL_HEAD_MAX_DECI; head += 10)
    {
        for (mm = 50U; mm <= 4095U; mm = (uint16_t)(mm + 5U))
        {
            hug_wall(OBSTACLE_CLASS_VALID, mm, head, &cmd);

            if (OBSTACLE_HUG_TOWARD == cmd.dir)
            {
                /* Only while above the lowest wanted heading + deadband. */
                CHECK(head > (lean_floor + OBSTACLE_WALL_HEAD_DEAD_DECI));
            }
            if (OBSTACLE_HUG_AWAY == cmd.dir)
            {
                CHECK(head < (OBSTACLE_WALL_PIVOT_DECI + (int32_t)(OBSTACLE_HUG_SPAN_MM *
                                 OBSTACLE_WALL_LEAN_DECI_PER_MM) - OBSTACLE_WALL_HEAD_DEAD_DECI));
            }
        }
    }

    /* At heading 90 with the wall 450 mm away (a ray that may be seeing
     * something else) the car stays parallel. */
    hug_wall(OBSTACLE_CLASS_VALID, 480U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
    hug_wall(OBSTACLE_CLASS_VALID, 4000U, OBSTACLE_WALL_PIVOT_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_STRAIGHT);
}

/* The heading window +60..+120 holds in wall mode whatever the reading. */
static void test_hug_wall_window(void)
{
    obstacle_hug_cmd_t cmd;
    uint16_t           mm = 0U;

    for (mm = 50U; mm <= 4095U; mm = (uint16_t)(mm + 45U))
    {
        hug_wall(OBSTACLE_CLASS_VALID, mm, OBSTACLE_WALL_HEAD_MAX_DECI, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_AWAY);
        hug_wall(OBSTACLE_CLASS_VALID, mm, OBSTACLE_WALL_HEAD_MAX_DECI + 150, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_AWAY);
        hug_wall(OBSTACLE_CLASS_VALID, mm, OBSTACLE_WALL_HEAD_MIN_DECI, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_TOWARD);
        hug_wall(OBSTACLE_CLASS_VALID, mm, OBSTACLE_WALL_HEAD_MIN_DECI - 150, &cmd);
        CHECK(cmd.dir != OBSTACLE_HUG_TOWARD);
    }

    /* Too close with the heading at the top of the window: the wanted
     * heading is 106 deg, so the car turns back towards it, not further. */
    hug_wall(OBSTACLE_CLASS_VALID, 150U, OBSTACLE_WALL_HEAD_MAX_DECI, &cmd);
    CHECK(cmd.dir == OBSTACLE_HUG_TOWARD);
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

/* Every fourth ping looks ahead: seq 3, 7, 11 ... and the 6-bit counter
 * wraps on a multiple of four. */
static void test_look_ahead_schedule(void)
{
    uint32_t seq   = 0U;
    uint32_t count = 0U;

    for (seq = 0U; seq < 64U; seq++)
    {
        if (obstacle_plan_is_look_ahead((uint8_t)seq))
        {
            count++;
            CHECK((seq % 4U) == 3U);
        }
    }
    CHECK(count == 16U);
    CHECK(!obstacle_plan_is_look_ahead(0U));
    CHECK(obstacle_plan_is_look_ahead(3U));
    CHECK(obstacle_plan_is_look_ahead(63U));
}

/* Side filter: pings route by sequence; forward pings never enter it. */
static void test_side_filter(void)
{
    obstacle_side_filter_t filt;

    obstacle_plan_side_reset(&filt, 0U);
    CHECK(obstacle_plan_side_median(&filt) == OBSTACLE_SIDE_FAR_MM);
    CHECK(obstacle_plan_side_latest(&filt) == OBSTACLE_SIDE_FAR_MM);

    obstacle_plan_side_feed(&filt, 1U, OBSTACLE_CLASS_VALID, 300U);
    obstacle_plan_side_feed(&filt, 2U, OBSTACLE_CLASS_VALID, 310U);
    CHECK(obstacle_plan_side_median(&filt) == OBSTACLE_SIDE_FAR_MM);
    obstacle_plan_side_feed(&filt, 3U, OBSTACLE_CLASS_VALID, 100U);
    /* seq 3 is a look-ahead ping: ignored. */
    CHECK(obstacle_plan_side_median(&filt) == OBSTACLE_SIDE_FAR_MM);
    CHECK(obstacle_plan_side_latest(&filt) == 310U);

    obstacle_plan_side_feed(&filt, 4U, OBSTACLE_CLASS_VALID, 320U);
    CHECK(obstacle_plan_side_median(&filt) == 310U);
    CHECK(obstacle_plan_side_latest(&filt) == 320U);
    CHECK(obstacle_plan_side_class(&filt) == OBSTACLE_CLASS_VALID);

    /* The same seq twice counts once. */
    obstacle_plan_side_feed(&filt, 4U, OBSTACLE_CLASS_VALID, 999U);
    CHECK(obstacle_plan_side_latest(&filt) == 320U);

    /* A forward OPEN does not change the class of the side ping. */
    obstacle_plan_side_feed(&filt, 7U, OBSTACLE_CLASS_OPEN, 0U);
    CHECK(obstacle_plan_side_class(&filt) == OBSTACLE_CLASS_VALID);
    CHECK(obstacle_plan_side_latest(&filt) == 320U);

    /* A side OPEN counts as far. */
    obstacle_plan_side_feed(&filt, 8U, OBSTACLE_CLASS_OPEN, 0U);
    CHECK(obstacle_plan_side_class(&filt) == OBSTACLE_CLASS_OPEN);
    CHECK(obstacle_plan_side_latest(&filt) == OBSTACLE_SIDE_FAR_MM);

    /* Reset forgets everything. */
    obstacle_plan_side_reset(&filt, 9U);
    CHECK(obstacle_plan_side_median(&filt) == OBSTACLE_SIDE_FAR_MM);
    CHECK(obstacle_plan_side_class(&filt) == OBSTACLE_CLASS_UNKNOWN);
}

/* Wall mode heading trim: the wall distance corrects the dead-reckoned
 * heading, one step per new side ping, inside the lean range only. */
static void test_wall_trim(void)
{
    obstacle_side_filter_t filt;
    int32_t const          head = OBSTACLE_WALL_PIVOT_DECI;
    int32_t                trimmed = 0;
    int32_t                before = 0;
    uint8_t                seq = 0U;
    uint32_t               i = 0U;

    obstacle_plan_side_reset(&filt, 0U);

    /* Not tracking: the trim stays 0 whatever the readings. */
    obstacle_plan_side_feed(&filt, 4U, OBSTACLE_CLASS_VALID, 400U);
    CHECK(obstacle_plan_wall_head(&filt, head, false) == head);

    /* Too far (reading 400 mm on the slanted ray is about 375 mm from the
     * wall): the trim rises. The same ping is not counted twice. */
    obstacle_plan_side_feed(&filt, 5U, OBSTACLE_CLASS_VALID, 400U);
    trimmed = obstacle_plan_wall_head(&filt, head, true);
    CHECK(trimmed > head);
    CHECK(obstacle_plan_wall_head(&filt, head, true) == trimmed);

    /* Too close: it falls, by more for a closer wall. */
    before = trimmed;
    obstacle_plan_side_feed(&filt, 6U, OBSTACLE_CLASS_VALID, 250U);
    trimmed = obstacle_plan_wall_head(&filt, head, true);
    CHECK(trimmed < before);

    /* At the target (300 mm from the wall) the trim does not move. */
    before = trimmed;
    obstacle_plan_side_feed(&filt, 8U, OBSTACLE_CLASS_VALID, 320U);
    trimmed = obstacle_plan_wall_head(&filt, head, true);
    CHECK((trimmed - before) <= 1);
    CHECK((trimmed - before) >= -1);

    /* Look-ahead, OPEN, a reading beyond the lean range and a blind class
     * leave the trim alone. */
    before = trimmed;
    obstacle_plan_side_feed(&filt, 11U, OBSTACLE_CLASS_VALID, 400U);
    CHECK(obstacle_plan_wall_head(&filt, head, true) == before);
    obstacle_plan_side_feed(&filt, 12U, OBSTACLE_CLASS_OPEN, 0U);
    CHECK(obstacle_plan_wall_head(&filt, head, true) == before);
    obstacle_plan_side_feed(&filt, 13U, OBSTACLE_CLASS_VALID, 1000U);
    CHECK(obstacle_plan_wall_head(&filt, head, true) == before);
    obstacle_plan_side_feed(&filt, 14U, OBSTACLE_CLASS_UNKNOWN, 0U);
    CHECK(obstacle_plan_wall_head(&filt, head, true) == before);

    /* A persistent far reading cannot wind the trim past the limit. */
    seq = 16U;
    for (i = 0U; i < 400U; i++)
    {
        seq = (uint8_t)((seq % 3U == 2U) ? (seq + 2U) : (seq + 1U));
        obstacle_plan_side_feed(&filt, (uint8_t)(seq & OBSTACLE_ST_SEQ_MK), OBSTACLE_CLASS_VALID,
                                400U);
        trimmed = obstacle_plan_wall_head(&filt, head, true);
    }
    CHECK(trimmed <= (head + OBSTACLE_WALL_TRIM_MAX_DECI));
    /* (The ray gets too slanted to trust before the limit, which stops it too.) */
    CHECK(trimmed > (head + 100));

    /* Entering REACQUIRE again starts from zero. */
    obstacle_plan_side_reset(&filt, 1U);
    CHECK(obstacle_plan_wall_head(&filt, head, true) == head);
}

static obstacle_drive_in_t drive_in(bool b_wall, uint8_t phase)
{
    obstacle_drive_in_t in;

    in.b_wall     = b_wall;
    in.side       = OBSTACLE_SIDE_LEFT;
    in.phase      = phase;
    in.start_deg  = 29U;
    in.head_deci  = 0;
    in.lateral_mm = 0;
    in.need_mm    = 168;
    in.side_cls   = OBSTACLE_CLASS_VALID;
    in.median_mm  = OBSTACLE_HUG_TARGET_MM;
    in.latest_mm  = OBSTACLE_HUG_TARGET_MM;
    return in;
}

/* The escape pivot: AWAY from the obstacle on the inside wheel, "turned"
 * once the planned angle is reached, held at the cap. */
static void test_pivot(void)
{
    obstacle_drive_in_t  in  = drive_in(false, OBSTACLE_PHASE_TURN);
    obstacle_drive_out_t out;

    in.head_deci = 100;
    obstacle_plan_pivot(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);
    CHECK(out.inside_um_s == OBSTACLE_NAV_ESCAPE_INSIDE_UM_S);
    CHECK(out.outside_um_s == (int32_t)OBSTACLE_NAV_SPEED_UM_S);
    CHECK(!out.b_turned);

    in.head_deci = 289;
    obstacle_plan_pivot(&in, &out);
    CHECK(!out.b_turned);
    in.head_deci = 290;
    obstacle_plan_pivot(&in, &out);
    CHECK(out.b_turned);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);

    in.side = OBSTACLE_SIDE_RIGHT;
    obstacle_plan_pivot(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_RIGHT);
    in.side = OBSTACLE_SIDE_LEFT;

    /* Box: no pivot beyond OBSTACLE_PIVOT_MAX_DECI. */
    in.head_deci = OBSTACLE_PIVOT_MAX_DECI;
    obstacle_plan_pivot(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_HOLD);

    /* Wall: the planned angle is 90 deg whatever start_deg says. */
    in = drive_in(true, OBSTACLE_PHASE_TURN);
    in.start_deg = 56U;
    in.head_deci = 560;
    obstacle_plan_pivot(&in, &out);
    CHECK(!out.b_turned);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);
    in.head_deci = OBSTACLE_WALL_PIVOT_DECI - 1;
    obstacle_plan_pivot(&in, &out);
    CHECK(!out.b_turned);
    in.head_deci = OBSTACLE_WALL_PIVOT_DECI;
    obstacle_plan_pivot(&in, &out);
    CHECK(out.b_turned);
    in.head_deci = OBSTACLE_WALL_HEAD_MAX_DECI;
    obstacle_plan_pivot(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_HOLD);
}

/* Box phases: TURN, OUT, then ALIGN (new) before HUG. */
static void test_drive_box_phases(void)
{
    obstacle_drive_in_t  in = drive_in(false, OBSTACLE_PHASE_TURN);
    obstacle_drive_out_t out;

    /* Still short of the start angle: keep pivoting. */
    in.head_deci = 250;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_TURN);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);
    CHECK(out.inside_um_s == OBSTACLE_NAV_ESCAPE_INSIDE_UM_S);

    /* Start angle reached: OUT, straight on at that heading. */
    in.head_deci = 295;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_OUT);
    CHECK(out.steer == OBSTACLE_STEER_STRAIGHT);

    /* Offset reached: ALIGN, turn back towards the obstacle side. */
    in.phase      = OBSTACLE_PHASE_OUT;
    in.lateral_mm = 167;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_OUT);
    in.lateral_mm = 168;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_ALIGN);
    CHECK(out.steer == OBSTACLE_STEER_RIGHT);
    CHECK(out.inside_um_s == (int32_t)(OBSTACLE_NAV_HUG_OUTSIDE_UM_S / 5U));
    CHECK(out.outside_um_s == (int32_t)OBSTACLE_NAV_HUG_OUTSIDE_UM_S);

    /* Still turned out: stay in ALIGN. Parallel (within the tolerance): HUG. */
    in.phase     = OBSTACLE_PHASE_ALIGN;
    in.head_deci = OBSTACLE_ALIGN_DONE_DECI + 1;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_ALIGN);
    CHECK(out.steer == OBSTACLE_STEER_RIGHT);
    in.head_deci = OBSTACLE_ALIGN_DONE_DECI;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_HUG);

    /* HUG on the reading, with the cap. */
    in.phase     = OBSTACLE_PHASE_HUG;
    in.head_deci = 0;
    in.median_mm = 200U;
    in.latest_mm = 200U;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_HUG);
    CHECK(out.dir == OBSTACLE_HUG_AWAY);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);
    CHECK(out.inside_um_s == (int32_t)(OBSTACLE_NAV_HUG_OUTSIDE_UM_S / 5U));
    in.head_deci = OBSTACLE_HUG_AWAY_MAX_DECI;
    obstacle_plan_drive(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_STRAIGHT);
    CHECK(out.inside_um_s == (int32_t)OBSTACLE_NAV_SPEED_UM_S);

    /* TOWARD is capped at the other end. */
    in.side_cls  = OBSTACLE_CLASS_OPEN;
    in.median_mm = OBSTACLE_SIDE_FAR_MM;
    in.latest_mm = OBSTACLE_SIDE_FAR_MM;
    in.head_deci = 0;
    obstacle_plan_drive(&in, &out);
    CHECK(out.dir == OBSTACLE_HUG_TOWARD);
    CHECK(out.steer == OBSTACLE_STEER_RIGHT);
    in.head_deci = -OBSTACLE_HUG_TOWARD_MAX_DECI;
    obstacle_plan_drive(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_STRAIGHT);

    /* Mirror: passing on the right swaps the wheels. */
    in.side      = OBSTACLE_SIDE_RIGHT;
    in.head_deci = 0;
    obstacle_plan_drive(&in, &out);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);
}

/* Several phase changes can happen in one call (as the old loop did). */
static void test_drive_phase_chain(void)
{
    obstacle_drive_in_t  in = drive_in(false, OBSTACLE_PHASE_TURN);
    obstacle_drive_out_t out;

    in.start_deg  = 0U;
    in.head_deci  = 0;
    in.need_mm    = 0;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_HUG);
}

/* Wall phases: TURN to 90 deg, then straight into HUG (no OUT, no ALIGN). */
static void test_drive_wall_phases(void)
{
    obstacle_drive_in_t  in = drive_in(true, OBSTACLE_PHASE_TURN);
    obstacle_drive_out_t out;

    in.head_deci = OBSTACLE_WALL_PIVOT_DECI - 10;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_TURN);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);

    in.head_deci = OBSTACLE_WALL_PIVOT_DECI;
    obstacle_plan_drive(&in, &out);
    CHECK(out.phase == OBSTACLE_PHASE_HUG);
    CHECK(out.steer == OBSTACLE_STEER_STRAIGHT);

    /* Nose to the wall in the window: turn away. */
    in.phase     = OBSTACLE_PHASE_HUG;
    in.side_cls  = OBSTACLE_CLASS_OPEN;
    in.median_mm = OBSTACLE_SIDE_FAR_MM;
    in.latest_mm = OBSTACLE_SIDE_FAR_MM;
    in.head_deci = OBSTACLE_WALL_PIVOT_DECI - 200;
    obstacle_plan_drive(&in, &out);
    CHECK(out.dir == OBSTACLE_HUG_AWAY);
    CHECK(out.steer == OBSTACLE_STEER_LEFT);

    /* Nose away from the wall: turn back (right). */
    in.head_deci = OBSTACLE_WALL_PIVOT_DECI + 200;
    obstacle_plan_drive(&in, &out);
    CHECK(out.dir == OBSTACLE_HUG_TOWARD);
    CHECK(out.steer == OBSTACLE_STEER_RIGHT);
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
    test_build_bounded_unchanged();
    test_build_wall();
    test_wall_flag_clear();
    test_look_ahead_schedule();
    test_side_filter();
    test_wall_trim();
    test_hug_ctrl();
    test_hug_caps_box();
    test_hug_near_override();
    test_hug_wall();
    test_hug_wall_lean();
    test_hug_wall_window();
    test_pivot();
    test_drive_box_phases();
    test_drive_phase_chain();
    test_drive_wall_phases();
    test_hug_limit();

    if (g_failures == 0)
    {
        printf("obstacle_plan: all checks passed\n");
        return 0;
    }
    printf("obstacle_plan: %d check(s) failed\n", g_failures);
    return 1;
}
