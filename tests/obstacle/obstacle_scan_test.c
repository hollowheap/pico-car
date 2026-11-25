/** @file obstacle_scan_test.c
 *
 * @brief Host tests for Buddy 5 echo classification and scan engine.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

    CHECK(scan_algo_slot_deci(0U) == -900);
    CHECK(scan_algo_slot_deci(18U) == 0);
    CHECK(scan_algo_slot_deci(36U) == 900);
}

static void test_classify(void)
{
    CHECK(scan_algo_classify(0U, false, false, true) == OBSTACLE_CLASS_STUCK);
    CHECK(scan_algo_classify(OBSTACLE_ECHO_DONE | OBSTACLE_ECHO_RISE | 1000U, false, false,
                             false) == OBSTACLE_CLASS_VALID);
    CHECK(scan_algo_classify(OBSTACLE_ECHO_DONE | OBSTACLE_ECHO_RISE | 25000U, false, false,
                             false) == OBSTACLE_CLASS_OPEN);
    /* The rise may have arrived just before the deadline. Without its
     * falling edge even a fresh ping cannot establish forward clearance. */
    CHECK(scan_algo_classify(OBSTACLE_ECHO_RISE, true, true, false) == OBSTACLE_CLASS_NO_ECHO);
}

/* every outcome has its own class. */
static void test_classify_each_class(void)
{
    uint32_t const done_rise = OBSTACLE_ECHO_DONE | OBSTACLE_ECHO_RISE;

    /* A clean echo between the glitch limit and the VALID minimum is an object
     * inside the blind zone. */
    CHECK(scan_algo_classify(done_rise | 100U, false, false, false) == OBSTACLE_CLASS_TOO_CLOSE);
    CHECK(scan_algo_classify(done_rise | OBSTACLE_GLITCH_MAX_US, false, false, false) ==
          OBSTACLE_CLASS_TOO_CLOSE);
    CHECK(scan_algo_classify(done_rise | (OBSTACLE_VALID_MIN_US - 1U), false, false, false) ==
          OBSTACLE_CLASS_TOO_CLOSE);
    CHECK(scan_algo_classify(done_rise | OBSTACLE_VALID_MIN_US, false, false, false) ==
          OBSTACLE_CLASS_VALID);
    /* Too short to be a reflection. */
    CHECK(scan_algo_classify(done_rise | (OBSTACLE_GLITCH_MAX_US - 1U), false, false, false) ==
          OBSTACLE_CLASS_GLITCH);
    CHECK(scan_algo_classify(done_rise | 1U, false, false, false) == OBSTACLE_CLASS_GLITCH);
    /* One edge missing: a rise whose fall was lost, a fall with no rise, ECHO
     * high with no rise seen. */
    CHECK(scan_algo_classify(OBSTACLE_ECHO_RISE, true, false, false) == OBSTACLE_CLASS_GLITCH);
    CHECK(scan_algo_classify(OBSTACLE_ECHO_DONE, false, false, false) == OBSTACLE_CLASS_GLITCH);
    CHECK(scan_algo_classify(0U, true, true, false) == OBSTACLE_CLASS_GLITCH);
    /* Nothing on ECHO within the wait. */
    CHECK(scan_algo_classify(0U, true, false, false) == OBSTACLE_CLASS_NO_ECHO);
    /* A wake that left the result empty carries no echo either. */
    CHECK(scan_algo_classify(0U, false, false, false) == OBSTACLE_CLASS_NO_ECHO);
    /* ECHO high before the trigger wins over everything else. */
    CHECK(scan_algo_classify(done_rise | 1000U, false, false, true) == OBSTACLE_CLASS_STUCK);
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
    /* Four contradicting pings leave the slot unresolved. */
    CHECK(obstacle_pack_slot_class(slot) == OBSTACLE_CLASS_GLITCH);
    CHECK(obstacle_pack_class_is_unresolved(obstacle_pack_slot_class(slot)));

    /* Two TOO_CLOSE pings agree and store a hit at the minimum range. */
    scan_algo_agree_reset(&agree);
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_TOO_CLOSE, 0U, &slot));
    CHECK(scan_algo_agree_add(&agree, OBSTACLE_CLASS_TOO_CLOSE, 0U, &slot));
    CHECK(obstacle_pack_slot_class(slot) == OBSTACLE_CLASS_TOO_CLOSE);
    CHECK(obstacle_pack_slot_dist(slot) == OBSTACLE_MIN_RANGE_MM);

    /* A NO_ECHO ping next to a VALID ping does not agree. */
    scan_algo_agree_reset(&agree);
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_NO_ECHO, 0U, &slot));
    CHECK(!scan_algo_agree_add(&agree, OBSTACLE_CLASS_VALID, 300U, &slot));
    CHECK(scan_algo_agree_add(&agree, OBSTACLE_CLASS_NO_ECHO, 0U, &slot));
    CHECK(obstacle_pack_slot_class(slot) == OBSTACLE_CLASS_NO_ECHO);

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
    const uint16_t unknown = obstacle_pack_slot(0U, OBSTACLE_CLASS_GLITCH);
    const uint16_t too_close = obstacle_pack_slot(OBSTACLE_MIN_RANGE_MM, OBSTACLE_CLASS_TOO_CLOSE);

    CHECK(scan_algo_is_hit(valid_near, 200U));
    CHECK(!scan_algo_is_hit(valid_far, 200U));
    CHECK(!scan_algo_is_hit(open, 200U));
    CHECK(!scan_algo_is_hit(0U, 200U));
    CHECK(!scan_algo_is_hit(unknown, 200U));
    /* TOO_CLOSE is a hit at the minimum range, and never a miss. */
    CHECK(scan_algo_is_hit(too_close, 200U));
    CHECK(!scan_algo_is_miss(too_close, 200U));

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

/* A profile of a square wall d0 away, seen by a servo that reaches
 * scale_percent of the commanded angle. */
static void wall_profile(uint16_t * p_profile, double d0, double scale_percent)
{
    uint32_t slot = 0U;

    for (slot = 0U; slot < OBSTACLE_SLOT_COUNT; slot++)
    {
        double const cmd_deg = ((double)OBSTACLE_SLOT_FIRST_DECI +
                                ((double)slot * (double)OBSTACLE_SLOT_STEP_DECI)) / 10.0;
        double const real_rad = (cmd_deg * scale_percent / 100.0) * 3.14159265358979 / 180.0;

        p_profile[slot] = obstacle_pack_slot((uint32_t)(d0 / cos(real_rad)), OBSTACLE_CLASS_VALID);
        p_profile[slot] = (uint16_t)(p_profile[slot] | (1U << OBSTACLE_SLOT_MEAS_SH));
    }
}

static void test_servo_check(void)
{
    uint16_t profile[OBSTACLE_SLOT_COUNT];
    servo_check_t check;

    wall_profile(profile, 300.0, 100.0);
    scan_algo_servo_check(profile, &check);
    CHECK(SERVO_CHECK_OK == check.verdict);
    /* Both ends read the servo limit, within 6 degrees. */
    CHECK(check.left_deci > (OBSTACLE_SERVO_LIMIT_DECI - 60) &&
          check.left_deci < (OBSTACLE_SERVO_LIMIT_DECI + 60));
    CHECK(check.right_deci > (OBSTACLE_SERVO_LIMIT_DECI - 60) &&
          check.right_deci < (OBSTACLE_SERVO_LIMIT_DECI + 60));

    wall_profile(profile, 300.0, 1.0);
    scan_algo_servo_check(profile, &check);
    CHECK(SERVO_CHECK_FLAT == check.verdict);

    wall_profile(profile, 300.0, 60.0);
    scan_algo_servo_check(profile, &check);
    CHECK(SERVO_CHECK_SCALE == check.verdict);
    CHECK(check.left_deci > ((OBSTACLE_SERVO_LIMIT_DECI * 6) / 10 - 60) &&
          check.left_deci < ((OBSTACLE_SERVO_LIMIT_DECI * 6) / 10 + 60));

    wall_profile(profile, 300.0, 100.0);
    profile[OBSTACLE_SLOT_CENTRE - (OBSTACLE_SERVO_LIMIT_DECI / OBSTACLE_SLOT_STEP_DECI)] =
        (uint16_t)(profile[OBSTACLE_SLOT_CENTRE - (OBSTACLE_SERVO_LIMIT_DECI / OBSTACLE_SLOT_STEP_DECI)] + 30U);
    scan_algo_servo_check(profile, &check);
    CHECK(SERVO_CHECK_ASYMMETRIC == check.verdict);

    wall_profile(profile, 300.0, 100.0);
    profile[OBSTACLE_SLOT_CENTRE] = 0U;
    scan_algo_servo_check(profile, &check);
    CHECK(SERVO_CHECK_NO_DATA == check.verdict);
}

static void test_empty_scan(void)
{
    scan_algo_ctx_t ctx;
    uint32_t        guard = 0U;

    scan_algo_start(&ctx);
    /* The right end of the servo window. */
    CHECK(scan_algo_target(&ctx) == (OBSTACLE_SLOT_CENTRE - (OBSTACLE_SERVO_LIMIT_DECI / OBSTACLE_SLOT_STEP_DECI)));
    while ((scan_algo_target(&ctx) != SCAN_ALGO_NO_SLOT) && (guard < 100U))
    {
        (void)feed_pair(&ctx, OBSTACLE_CLASS_OPEN, 0U);
        guard++;
    }
    CHECK(scan_algo_target(&ctx) == SCAN_ALGO_NO_SLOT);
    CHECK(!scan_algo_has_region(&ctx));
}

/* P5: a clean scan is not noisy. A scan where every slot needs a third ping
 * is. P1: VALID pings of one slot that jump by more than 40 mm are noisy. */
static void test_noise_flags(void)
{
    scan_algo_ctx_t ctx;
    uint32_t        guard = 0U;

    scan_algo_start(&ctx);
    while ((scan_algo_target(&ctx) != SCAN_ALGO_NO_SLOT) && (guard < 100U))
    {
        (void)feed_pair(&ctx, OBSTACLE_CLASS_OPEN, 0U);
        guard++;
    }
    CHECK(!ctx.b_scan_noisy);
    CHECK(!scan_algo_echo_noisy(&ctx));

    /* First two pings disagree, the third agrees with the second. */
    scan_algo_start(&ctx);
    guard = 0U;
    while ((scan_algo_target(&ctx) != SCAN_ALGO_NO_SLOT) && (guard < 100U))
    {
        (void)scan_algo_feed(&ctx, OBSTACLE_CLASS_VALID, 400U);
        (void)scan_algo_feed(&ctx, OBSTACLE_CLASS_VALID, 400U + OBSTACLE_NOISE_MM + 10U);
        (void)scan_algo_feed(&ctx, OBSTACLE_CLASS_VALID, 400U + OBSTACLE_NOISE_MM + 10U);
        guard++;
    }
    CHECK(ctx.slots_retried == ctx.slots_done);
    CHECK(ctx.b_scan_noisy);
    CHECK(scan_algo_echo_noisy(&ctx));

    /* A new scan clears the flags. */
    scan_algo_start(&ctx);
    CHECK(!ctx.b_scan_noisy);
    CHECK(!scan_algo_echo_noisy(&ctx));
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

    /* The obstacle starts at the centre and recedes by 60 mm per slot on
     * the left. Continue the surface after the first body pivot. */
    while ((scan_algo_target(&ctx) != SCAN_ALGO_NO_SLOT) && (0 == ctx.body_deci) && (guard < 100U))
    {
        const uint8_t  slot = scan_algo_target(&ctx);
        const bool     b_hit = (slot >= OBSTACLE_SLOT_CENTRE) &&
                               (slot <= (OBSTACLE_SLOT_CENTRE + (OBSTACLE_SERVO_LIMIT_DECI / OBSTACLE_SLOT_STEP_DECI)));
        const uint16_t dist  = b_hit ? (uint16_t)(200U + (60U * (uint16_t)(slot - OBSTACLE_SLOT_CENTRE)))
                                     : (uint16_t)0U;

        (void)feed_pair(&ctx, b_hit ? (uint8_t)OBSTACLE_CLASS_VALID : (uint8_t)OBSTACLE_CLASS_OPEN, dist);
        guard++;
    }
    CHECK(ctx.closest_mm == 200U);
    CHECK(ctx.edge_hit[SCAN_ALGO_LEFT] == 22U);
    CHECK(ctx.body_deci == OBSTACLE_SCAN_PIVOT_DECI);
    CHECK(scan_algo_target(&ctx) == (OBSTACLE_SLOT_CENTRE + (OBSTACLE_SERVO_LIMIT_DECI / OBSTACLE_SLOT_STEP_DECI) + 1U));

    /* The next readings are beyond the region but close enough to the last
     * hit that the walk must keep looking for the edge. */
    (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 500U);
    CHECK(scan_algo_target(&ctx) == 24U);
    (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 560U);
    CHECK(scan_algo_target(&ctx) == 25U);
    CHECK(ctx.edge_hit[SCAN_ALGO_LEFT] == 22U);
}

static void test_partial_scan_keeps_unmeasured_edges_open(void)
{
    scan_algo_ctx_t ctx;
    unsigned seq;

    scan_algo_start(&ctx);
    for (seq = 0U; seq < 100U && 0 == scan_algo_body_deci(&ctx); ++seq)
    {
        (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 300U);
    }
    CHECK(scan_algo_body_deci(&ctx) > 0);
    CHECK(scan_algo_finish_partial(&ctx));
    CHECK(scan_algo_has_region(&ctx));
    CHECK(SCAN_ALGO_NO_SLOT == scan_algo_target(&ctx));
    CHECK(SCAN_ALGO_EDGE_UNBOUNDED == ctx.edge_state[SCAN_ALGO_LEFT]);
    CHECK(SCAN_ALGO_EDGE_UNBOUNDED == ctx.edge_state[SCAN_ALGO_RIGHT]);
}

static void test_wide_scan_stays_within_servo_window(void)
{
    scan_algo_ctx_t ctx;
    bool b_within_window = true;
    unsigned sample;

    scan_algo_start(&ctx);
    for (sample = 0U; sample < OBSTACLE_SLOT_COUNT &&
                      SCAN_ALGO_NO_SLOT != scan_algo_target(&ctx); ++sample)
    {
        uint8_t const slot  = scan_algo_target(&ctx);
        int16_t const servo = scan_algo_servo_deci(&ctx);

        b_within_window = b_within_window && (servo >= -200) && (servo <= 200);
        CHECK(scan_algo_body_deci(&ctx) + servo == scan_algo_slot_deci(slot));
        (void)feed_pair(&ctx, OBSTACLE_CLASS_VALID, 300U);
    }
    CHECK(b_within_window);
    CHECK(SCAN_ALGO_NO_SLOT == scan_algo_target(&ctx));
    CHECK(SCAN_ALGO_EDGE_UNBOUNDED == ctx.edge_state[SCAN_ALGO_LEFT]);
    CHECK(SCAN_ALGO_EDGE_UNBOUNDED == ctx.edge_state[SCAN_ALGO_RIGHT]);
    for (sample = 0U; sample < OBSTACLE_SLOT_COUNT; ++sample)
    {
        CHECK(obstacle_pack_slot_is_measured(ctx.profile[sample]));
    }
}

static void test_profile_records_measured_bearings(void)
{
    uint16_t slots[OBSTACLE_SLOT_COUNT] = {0};

    CHECK(obstacle_profile_record_ray(slots, -197, 130U, OBSTACLE_CLASS_VALID) == 1U);
    CHECK(obstacle_profile_record_ray(slots, 6, 110U, OBSTACLE_CLASS_VALID) == 5U);
    CHECK(obstacle_profile_record_ray(slots, 201, 0U, OBSTACLE_CLASS_OPEN) == 9U);
    CHECK(slots[14] == obstacle_pack_slot(130U, OBSTACLE_CLASS_VALID));
    CHECK(slots[18] == obstacle_pack_slot(110U, OBSTACLE_CLASS_VALID));
    CHECK(slots[22] == obstacle_pack_slot(0U, OBSTACLE_CLASS_OPEN));
    /* Equidistant between slots, or outside the profile field of view. */
    CHECK(obstacle_profile_record_ray(slots, 75, 100U, OBSTACLE_CLASS_VALID) == 0U);
    CHECK(obstacle_profile_record_ray(slots, 950, 100U, OBSTACLE_CLASS_VALID) == 0U);
    CHECK(slots[19] == 0U && slots[20] == 0U && slots[36] == 0U);
}

int main(int argc, char * argv[])
{
    if (2 == argc)
    {
        if (0 == strcmp(argv[1], "unmeasured-edges"))
        {
            test_partial_scan_keeps_unmeasured_edges_open();
        }
        else if (0 == strcmp(argv[1], "echo-classification"))
        {
            test_classify();
            test_classify_each_class();
        }
        else
        {
            (void)fprintf(stderr, "unknown case %s\n", argv[1]);
            return 2;
        }
        return (0 == g_failures) ? 0 : 1;
    }
    if (1 != argc)
    {
        (void)fprintf(stderr, "usage: %s [case]\n", argv[0]);
        return 2;
    }

    test_profile_records_measured_bearings();
    test_distance_and_angle();
    test_classify();
    test_classify_each_class();
    test_agreement();
    test_hit_and_miss();
    test_empty_scan();
    test_servo_check();
    test_noise_flags();
    test_region_scan();
    test_receding_edge();
    test_partial_scan_keeps_unmeasured_edges_open();
    test_wide_scan_stays_within_servo_window();

    if (g_failures == 0)
    {
        printf("obstacle_scan: all checks passed\n");
        return 0;
    }
    printf("obstacle_scan: %d check(s) failed\n", g_failures);
    return 1;
}
