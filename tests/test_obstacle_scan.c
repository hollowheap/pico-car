/** @file test_obstacle_scan.c
 *
 * @brief Host tests for Buddy 5 echo classification and scan engine.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
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

static void test_distance_and_angle(void)
{
    CHECK(scan_algo_dist_mm(0U) == 0U);
    CHECK(scan_algo_dist_mm(116U) == 19U);
    CHECK(scan_algo_dist_mm(23500U) == 4030U);
    CHECK(scan_algo_dist_mm(40000U) == 4030U);

    CHECK(scan_algo_slot_deci(0U) == -600);
    CHECK(scan_algo_slot_deci(12U) == 0);
    CHECK(scan_algo_slot_deci(24U) == 600);
}

static void test_classify(void)
{
    CHECK(scan_algo_classify(0U, false, false, true) == OBSTACLE_CLASS_STUCK);
    CHECK(scan_algo_classify(OBSTACLE_MBX_DONE | OBSTACLE_MBX_RISE | 1000U, false, false,
                             false) == OBSTACLE_CLASS_VALID);
    CHECK(scan_algo_classify(OBSTACLE_MBX_DONE | OBSTACLE_MBX_RISE | 100U, false, false,
                             false) == OBSTACLE_CLASS_UNKNOWN);
    CHECK(scan_algo_classify(OBSTACLE_MBX_DONE | OBSTACLE_MBX_RISE | 25000U, false, false,
                             false) == OBSTACLE_CLASS_OPEN);
    CHECK(scan_algo_classify(OBSTACLE_MBX_RISE, true, true, false) == OBSTACLE_CLASS_OPEN);
    CHECK(scan_algo_classify(OBSTACLE_MBX_RISE, true, false, false) ==
          OBSTACLE_CLASS_UNKNOWN);
    CHECK(scan_algo_classify(OBSTACLE_MBX_DONE, false, false, false) ==
          OBSTACLE_CLASS_UNKNOWN);
}

static void test_agreement(void)
{
    scan_algo_agree_t agree;
    uint16_t          slot = 0U;

    scan_algo_agree_reset(&agree);
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 100U, &slot));
    CHECK(scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 110U, &slot));
    CHECK(obstacle_pack_slot_class(slot) == OBSTACLE_CLASS_VALID);
    CHECK(obstacle_pack_slot_dist(slot) == 100U);

    scan_algo_agree_reset(&agree);
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 100U, &slot));
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 200U, &slot));
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 300U, &slot));
    CHECK(scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 400U, &slot));
    CHECK(obstacle_pack_slot_class(slot) == OBSTACLE_CLASS_UNKNOWN);

    scan_algo_agree_reset(&agree);
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_OPEN, 0U, &slot));
    CHECK(scan_algo_agree_add(&agree, OBSTACLE_CLASS_OPEN, 0U, &slot));
    CHECK(obstacle_pack_slot_class(slot) == OBSTACLE_CLASS_OPEN);
}

static void test_hit_and_miss(void)
{
    const uint16_t valid_near = obstacle_pack_slot(100U, OBSTACLE_CLASS_VALID);
    const uint16_t valid_far = obstacle_pack_slot(600U, OBSTACLE_CLASS_VALID);
    const uint16_t open = obstacle_pack_slot(0U, OBSTACLE_CLASS_OPEN);
    const uint16_t unknown = obstacle_pack_slot(0U, OBSTACLE_CLASS_UNKNOWN);

    CHECK(scan_algo_is_hit(valid_near, 200U));
    CHECK(!scan_algo_is_hit(valid_far, 200U));
    CHECK(!scan_algo_is_hit(open, 200U));
    CHECK(!scan_algo_is_hit(0U, 200U));

    CHECK(scan_algo_is_miss(open, 200U));
    CHECK(scan_algo_is_miss(valid_far, 200U));
    CHECK(!scan_algo_is_miss(valid_near, 200U));
    CHECK(!scan_algo_is_miss(unknown, 200U));
    CHECK(!scan_algo_is_miss(0U, 200U));

    /* The reference is the last hit on the edge, not the global closest:
     * 600 mm is a miss when the last hit was 300 mm and no miss when it
     * was 400 mm. */
    CHECK(scan_algo_is_miss(obstacle_pack_slot(600U, OBSTACLE_CLASS_VALID),
                            300U));
    CHECK(!scan_algo_is_miss(obstacle_pack_slot(600U, OBSTACLE_CLASS_VALID),
                             400U));
}

static bool feed_pair(scan_algo_ctx_t * p_ctx, uint8_t cls, uint16_t dist_mm)
{
    bool done = scan_algo_feed(p_ctx, cls, dist_mm);

    if (!done)
    {
        done = scan_algo_feed(p_ctx, cls, dist_mm);
    }
    return done;
}

static void test_empty_scan(void)
{
    scan_algo_ctx_t ctx;
    uint32_t        guard = 0U;

    scan_algo_start(&ctx);
    CHECK(scan_algo_target(&ctx) == 0U);
    while ((scan_algo_target(&ctx) != SCAN_ALGO_NO_SLOT) && (guard < 100U))
    {
        (void)feed_pair(&ctx, OBSTACLE_CLASS_OPEN, 0U);
        guard++;
    }
    CHECK(scan_algo_target(&ctx) == SCAN_ALGO_NO_SLOT);
    CHECK(!scan_algo_has_region(&ctx));
}

static void test_region_scan(void)
{
    scan_algo_ctx_t ctx;
    uint32_t        guard = 0U;

    scan_algo_start(&ctx);
    while ((scan_algo_target(&ctx) != SCAN_ALGO_NO_SLOT) && (guard < 100U))
    {
        const uint8_t slot = scan_algo_target(&ctx);
        const uint8_t cls = (slot == OBSTACLE_SLOT_CENTRE)
                                ? (uint8_t)OBSTACLE_CLASS_VALID
                                : (uint8_t)OBSTACLE_CLASS_OPEN;
        const uint16_t dist = (slot == OBSTACLE_SLOT_CENTRE) ? (uint16_t)200U
                                                       : (uint16_t)0U;

        (void)feed_pair(&ctx, cls, dist);
        guard++;
    }
    CHECK(scan_algo_target(&ctx) == SCAN_ALGO_NO_SLOT);
    CHECK(scan_algo_has_region(&ctx));
    CHECK(ctx.closest_mm == 200U);
    CHECK(ctx.edge_state[SCAN_ALGO_LEFT] == SCAN_ALGO_EDGE_BOUNDED);
    CHECK(ctx.edge_state[SCAN_ALGO_RIGHT] == SCAN_ALGO_EDGE_BOUNDED);
}

static void test_receding_edge(void)
{
    scan_algo_ctx_t ctx;
    uint32_t        guard = 0U;

    scan_algo_start(&ctx);

    /* Coarse pass: the obstacle at the centre and a receding surface at
     * slot 15. Only slot 12 is inside the region. */
    while ((scan_algo_target(&ctx) != 13U) && (guard < 100U))
    {
        const uint8_t  slot = scan_algo_target(&ctx);
        const uint8_t  cls  = ((slot == OBSTACLE_SLOT_CENTRE) || (15U == slot))
                                  ? (uint8_t)OBSTACLE_CLASS_VALID
                                  : (uint8_t)OBSTACLE_CLASS_OPEN;
        const uint16_t dist = (slot == OBSTACLE_SLOT_CENTRE)
                                  ? (uint16_t)200U
                                  : ((15U == slot) ? (uint16_t)560U
                                                   : (uint16_t)0U);

        (void)feed_pair(&ctx, cls, dist);
        guard++;
    }
    CHECK(scan_algo_target(&ctx) == 13U);

    /* A surface that recedes by 120 mm per 5-degree slot must not be
     * mistaken for an edge. Slot 15 (560 mm) was measured in the coarse
     * pass: it is beyond closest + 250 (450) but only 120 mm beyond the
     * last hit, so the walk steps over it instead of closing the edge. */
    (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 320U);
    CHECK(scan_algo_target(&ctx) == 14U);
    (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 440U);
    CHECK(scan_algo_target(&ctx) == 16U);
    (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 680U);
    CHECK(scan_algo_target(&ctx) == 17U);
}

int main(void)
{
    test_distance_and_angle();
    test_classify();
    test_agreement();
    test_hit_and_miss();
    test_empty_scan();
    test_region_scan();
    test_receding_edge();

    if (g_failures == 0)
    {
        printf("obstacle_scan: all checks passed\n");
        return 0;
    }
    printf("obstacle_scan: %d check(s) failed\n", g_failures);
    return 1;
}
