/** @file test_barcode_width.c
 *
 * @brief Unit tests for barcode pulse interval to distance conversion.
 */

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motion/motion.h"
#include "track/barcode.h"

static void test_nominal_conversion (void)
{
    motion_feedback_t feedback = {
        .left_speed_um_s       = 120000,
        .right_speed_um_s      = 120000,
        .left_displacement_um  = 0,
        .right_displacement_um = 0,
        .timestamp_us          = 1000000U,
        .age_us                = 10000U,
        .speed_valid           = {true, true},
        .odometry_valid        = {true, true},
        .fresh                 = true,
    };
    uint16_t width_um = 0U;
    bool     ok       = false;

    /* 0.12 m/s: 8333 us -> 999 um */
    ok = barcode_width_um(8333U, &feedback, &width_um);
    assert(ok);
    assert(999U == width_um);

    /* 0.12 m/s: 25000 us -> 3000 um */
    ok = barcode_width_um(25000U, &feedback, &width_um);
    assert(ok);
    assert(3000U == width_um);

    /* 0.18 m/s: 5555 us -> 999 um */
    feedback.left_speed_um_s  = 180000;
    feedback.right_speed_um_s = 180000;
    ok = barcode_width_um(5555U, &feedback, &width_um);
    assert(ok);
    assert(999U == width_um);

    /* 0.18 m/s: 16666 us -> 2999 um */
    ok = barcode_width_um(16666U, &feedback, &width_um);
    assert(ok);
    assert(2999U == width_um);

    /* Zero interval yields 0 um */
    ok = barcode_width_um(0U, &feedback, &width_um);
    assert(ok);
    assert(0U == width_um);
}

static void test_differential_speed (void)
{
    motion_feedback_t feedback = {
        .left_speed_um_s       = 140000,
        .right_speed_um_s      = 160000,
        .left_displacement_um  = 0,
        .right_displacement_um = 0,
        .timestamp_us          = 1000000U,
        .age_us                = 10000U,
        .speed_valid           = {true, true},
        .odometry_valid        = {true, true},
        .fresh                 = true,
    };
    uint16_t width_um = 0U;
    bool     ok       = false;

    /* Mean speed = (140000 + 160000) / 2 = 150000 um/s. 10000 us -> 1500 um */
    ok = barcode_width_um(10000U, &feedback, &width_um);
    assert(ok);
    assert(1500U == width_um);

    /* Single wheel at 0, other at 200000 -> mean 100000 um/s. 10000 us -> 1000 um */
    feedback.left_speed_um_s  = 0;
    feedback.right_speed_um_s = 200000;
    ok = barcode_width_um(10000U, &feedback, &width_um);
    assert(ok);
    assert(1000U == width_um);
}

static void test_saturation (void)
{
    motion_feedback_t feedback = {
        .left_speed_um_s       = 200000,
        .right_speed_um_s      = 200000,
        .left_displacement_um  = 0,
        .right_displacement_um = 0,
        .timestamp_us          = 1000000U,
        .age_us                = 10000U,
        .speed_valid           = {true, true},
        .odometry_valid        = {true, true},
        .fresh                 = true,
    };
    uint16_t width_um = 0U;
    bool     ok       = false;

    /* Exact boundary: 655350 us at 100000 um/s -> 65535 um (UINT16_MAX) */
    feedback.left_speed_um_s  = 100000;
    feedback.right_speed_um_s = 100000;
    ok = barcode_width_um(655350U, &feedback, &width_um);
    assert(ok);
    assert(UINT16_MAX == width_um);

    /* Boundary + 1: 655360 us at 100000 um/s -> 65536 um clamped to UINT16_MAX */
    ok = barcode_width_um(655360U, &feedback, &width_um);
    assert(ok);
    assert(UINT16_MAX == width_um);

    /* 1,000,000 us * 200,000 um/s / 1e6 = 200,000 um > UINT16_MAX -> saturated */
    feedback.left_speed_um_s  = 200000;
    feedback.right_speed_um_s = 200000;
    ok = barcode_width_um(1000000U, &feedback, &width_um);
    assert(ok);
    assert(UINT16_MAX == width_um);

    /* Large interval without wrap-around: 3,000,000,000 us */
    ok = barcode_width_um(3000000000U, &feedback, &width_um);
    assert(ok);
    assert(UINT16_MAX == width_um);
}

static void test_rejection_conditions (void)
{
    motion_feedback_t feedback = {
        .left_speed_um_s       = 150000,
        .right_speed_um_s      = 150000,
        .left_displacement_um  = 0,
        .right_displacement_um = 0,
        .timestamp_us          = 1000000U,
        .age_us                = 10000U,
        .speed_valid           = {true, true},
        .odometry_valid        = {true, true},
        .fresh                 = true,
    };
    uint16_t width_um = 0U;

    /* Stale snapshot */
    feedback.fresh = false;
    assert(!barcode_width_um(10000U, &feedback, &width_um));
    feedback.fresh = true;

    /* Zero speed */
    feedback.left_speed_um_s  = 0;
    feedback.right_speed_um_s = 0;
    assert(!barcode_width_um(10000U, &feedback, &width_um));

    /* Opposing speeds cancelling to zero */
    feedback.left_speed_um_s  = -100000;
    feedback.right_speed_um_s = 100000;
    assert(!barcode_width_um(10000U, &feedback, &width_um));

    /* Reversing speed: both negative */
    feedback.left_speed_um_s  = -150000;
    feedback.right_speed_um_s = -150000;
    assert(!barcode_width_um(10000U, &feedback, &width_um));

    /* Reversing speed: one wheel negative */
    feedback.left_speed_um_s  = -10000;
    feedback.right_speed_um_s = 150000;
    assert(!barcode_width_um(10000U, &feedback, &width_um));

    feedback.left_speed_um_s  = 150000;
    feedback.right_speed_um_s = -10000;
    assert(!barcode_width_um(10000U, &feedback, &width_um));

    /* Reset speeds to valid positive values */
    feedback.left_speed_um_s  = 150000;
    feedback.right_speed_um_s = 150000;

    /* Invalid speed flags */
    feedback.speed_valid[0] = false;
    assert(!barcode_width_um(10000U, &feedback, &width_um));
    feedback.speed_valid[0] = true;

    feedback.speed_valid[1] = false;
    assert(!barcode_width_um(10000U, &feedback, &width_um));
    feedback.speed_valid[1] = true;

    /* NULL pointers */
    assert(!barcode_width_um(10000U, NULL, &width_um));
    assert(!barcode_width_um(10000U, &feedback, NULL));
}

/* A black element longer than any Code 39 wide bar is a block on the
 * track, not a bar; a space is never a block. */
static void test_block_is_not_a_bar (void)
{
    assert(!barcode_element_is_block(false, 16383U));   /* the widest narrow bar */
    assert(!barcode_element_is_block(false, (uint16_t)BARCODE_BLOCK_UM));
    assert(barcode_element_is_block(false, (uint16_t)(BARCODE_BLOCK_UM + 1U)));
    assert(barcode_element_is_block(false, UINT16_MAX)); /* saturated width */
    assert(!barcode_element_is_block(true, UINT16_MAX)); /* a long space */
}

/* The block event: the length comes from the odometer at the two edges
 * of the black stretch, so a block longer than the width field or the 500 ms
 * gap is still measured. */
static void test_block_event_length (void)
{
    barcode_block_t block;

    barcode_block_init(&block);
    /* A white edge with no black edge before it reports nothing. */
    assert(0U == barcode_block_edge(&block, false, 100U));
    /* A bar of 50 mm is not a block; 51 mm is. */
    assert(0U == barcode_block_edge(&block, true, 1000U));
    assert(0U == barcode_block_edge(&block, false, 1050U));
    assert(0U == barcode_block_edge(&block, true, 2000U));
    assert(51U == barcode_block_edge(&block, false, 2051U));
    /* A long block keeps its true length. */
    assert(0U == barcode_block_edge(&block, true, 3000U));
    assert(300U == barcode_block_edge(&block, false, 3300U));
    /* The length saturates at the int16 event parameter. */
    assert(0U == barcode_block_edge(&block, true, 10000U));
    assert(INT16_MAX == barcode_block_edge(&block, false, 110000U));
    /* The car backed up over the block: no length, no event. */
    assert(0U == barcode_block_edge(&block, true, 200000U));
    assert(0U == barcode_block_edge(&block, false, 199900U));
}

/* At most one block event per BARCODE_BLOCK_EVENT_MM of travel, so a striped
 * floor cannot flood the event queue. */
static void test_block_event_rate (void)
{
    barcode_block_t block;

    barcode_block_init(&block);
    /* The first block reports even at odometer 0. */
    assert(0U == barcode_block_edge(&block, true, 0U));
    assert(60U == barcode_block_edge(&block, false, 60U));
    /* A second block that ends inside the spacing is silent. */
    assert(0U == barcode_block_edge(&block, true, 80U));
    assert(0U == barcode_block_edge(&block, false, 60U + BARCODE_BLOCK_EVENT_MM - 1U));
    /* The next one past the spacing reports. */
    assert(0U == barcode_block_edge(&block, true, 400U));
    assert(60U == barcode_block_edge(&block, false, 460U));
    /* The car backed up behind the last event: a block there is silent. */
    assert(0U == barcode_block_edge(&block, true, 300U));
    assert(0U == barcode_block_edge(&block, false, 360U));
    /* A repeated black edge moves the start of the stretch. */
    assert(0U == barcode_block_edge(&block, true, 700U));
    assert(0U == barcode_block_edge(&block, true, 760U));
    assert(0U == barcode_block_edge(&block, false, 800U));
    assert(8U == sizeof(barcode_block_t));
}

int main (void)
{
    test_block_is_not_a_bar();
    test_block_event_length();
    test_block_event_rate();
    test_nominal_conversion();
    test_differential_speed();
    test_saturation();
    test_rejection_conditions();

    return 0;
}

/*** end of file ***/
