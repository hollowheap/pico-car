/** @file test_obstacle_nav_core.c
 *
 * @brief Host tests of the pure navigation glue (nav_core.c).
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nav_core.h"
#include "obstacle_config.h"
#include "obstacle_pack.h"

#define UM_PER_MM (1000)

/* A status word with a ping class, a distance and a sequence. */
static uint32_t status_word(uint8_t cls, uint16_t dist_mm, uint8_t seq)
{
    uint32_t word = 0U;

    word = obstacle_pack_put(word, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK,
                             OBSTACLE_STATE_REACQUIRE);
    word = obstacle_pack_put(word, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK, cls);
    word = obstacle_pack_put(word, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK, dist_mm);
    word = obstacle_pack_put(word, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK, seq);

    return word;
}

static void test_pose_straight_and_reset(void)
{
    nav_core_t core;
    int64_t    wheel[NAV_CORE_WHEELS] = {5000000, 7000000};
    follow_in_t in;

    (void)memset(&core, 0, sizeof(core));
    core.b_ir_cleared = true;
    nav_core_reset_pose(&core, wheel);
    assert(0 == nav_core_heading_deci(&core, wheel));
    assert(!core.b_ir_cleared);

    /* 300 mm straight on: y grows, x stays. */
    wheel[NAV_CORE_LEFT]  += 300 * UM_PER_MM;
    wheel[NAV_CORE_RIGHT] += 300 * UM_PER_MM;
    nav_core_update_pose(&core, wheel);
    nav_core_follow_begin(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 3U), wheel, &in);
    assert(0 == in.x_mm);
    assert(300 == in.y_mm);
    assert(0 == in.head_deci);
    assert(300U == in.travel_mm);
    assert(!in.b_ping && !in.b_line);

    /* Back 100 mm: the path counts both ways. */
    wheel[NAV_CORE_LEFT]  -= 100 * UM_PER_MM;
    wheel[NAV_CORE_RIGHT] -= 100 * UM_PER_MM;
    nav_core_update_pose(&core, wheel);
    nav_core_follow_begin(&core, 0U, wheel, &in);
    assert(200 == in.y_mm);
    assert(400U == in.travel_mm);
}

static void test_pose_left_turn(void)
{
    nav_core_t core;
    int64_t    wheel[NAV_CORE_WHEELS] = {0, 0};
    /* Wheel travel of a pivot by 90 degrees: track * (pi / 2) / 2. */
    int64_t const quarter_um = ((int64_t)OBSTACLE_WHEEL_TRACK_MM * UM_PER_MM * 900) /
                               (2 * OBSTACLE_NAV_DECI_PER_RAD);
    follow_in_t in;
    int32_t     head = 0;

    (void)memset(&core, 0, sizeof(core));
    nav_core_reset_pose(&core, wheel);
    wheel[NAV_CORE_LEFT]  = -quarter_um;
    wheel[NAV_CORE_RIGHT] = quarter_um;
    nav_core_update_pose(&core, wheel);
    head = nav_core_heading_deci(&core, wheel);
    assert((head >= 899) && (head <= 900));

    /* 200 mm at a left heading of 90 degrees: x grows to the left. */
    wheel[NAV_CORE_LEFT]  += 200 * UM_PER_MM;
    wheel[NAV_CORE_RIGHT] += 200 * UM_PER_MM;
    nav_core_update_pose(&core, wheel);
    nav_core_follow_begin(&core, 0U, wheel, &in);
    assert((in.x_mm >= 199) && (in.x_mm <= 200));
    assert((in.y_mm >= -1) && (in.y_mm <= 1));
}

static void test_servo_wait(void)
{
    nav_core_t core;

    (void)memset(&core, 0, sizeof(core));
    assert(!nav_core_servo_moving(&core, status_word(0U, 0U, 4U), 0U));

    /* A new ping ends the wait. */
    nav_core_servo_wait(&core, status_word(0U, 0U, 4U), 1000U);
    assert(nav_core_servo_moving(&core, status_word(0U, 0U, 4U), 1010U));
    assert(!nav_core_servo_moving(&core, status_word(0U, 0U, 5U), 1020U));
    assert(!nav_core_servo_moving(&core, status_word(0U, 0U, 5U), 1030U));

    /* With no ping the wait ends after OBSTACLE_NAV_SERVO_WAIT_MS. */
    nav_core_servo_wait(&core, status_word(0U, 0U, 5U), 2000U);
    assert(nav_core_servo_moving(&core, status_word(0U, 0U, 5U),
                                 2000U + OBSTACLE_NAV_SERVO_WAIT_MS - 1U));
    assert(!nav_core_servo_moving(&core, status_word(0U, 0U, 5U),
                                  2000U + OBSTACLE_NAV_SERVO_WAIT_MS));
}

static void test_repeat_gate(void)
{
    nav_core_t core;

    (void)memset(&core, 0, sizeof(core));
    assert(nav_core_repeat_due(&core, 1000U));
    assert(!nav_core_repeat_due(&core, 1000U + OBSTACLE_NAV_LINE_REPEAT_MS - 1U));
    assert(nav_core_repeat_due(&core, 1000U + OBSTACLE_NAV_LINE_REPEAT_MS));
    assert(!nav_core_repeat_due(&core, 1001U + OBSTACLE_NAV_LINE_REPEAT_MS));
}

static void test_follow_input(void)
{
    nav_core_t core;
    int64_t const wheel[NAV_CORE_WHEELS] = {0, 0};
    follow_in_t in;

    (void)memset(&core, 0, sizeof(core));
    nav_core_reset_pose(&core, wheel);
    nav_core_follow_begin(&core, status_word(OBSTACLE_CLASS_VALID, 250U, 7U), wheel, &in);

    /* Still on the start tape: the gate is closed, and the ping is not new. */
    assert(!nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_VALID, 250U, 7U), wheel,
                                  true, &in));
    assert(!in.b_line);
    assert(!in.b_ping);

    /* A new sequence is a new ping, with its class and distance. */
    assert(!nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_VALID, 310U, 8U), wheel,
                                  true, &in));
    assert(in.b_ping);
    assert(OBSTACLE_CLASS_VALID == in.cls);
    assert(310U == in.dist_mm);
    /* The same word again is not a new ping. */
    (void)nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_VALID, 310U, 8U), wheel, true,
                                &in);
    assert(!in.b_ping);

    /* Off the tape: the gate opens once; the next tape counts. */
    assert(nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 8U), wheel, false,
                                 &in));
    assert(!in.b_line);
    assert(!nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 8U), wheel, false,
                                  &in));
    (void)nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 8U), wheel, true, &in);
    assert(in.b_line);

    /* The sequence wraps at 5 bits: 31 to 0 is a new ping. */
    (void)nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 31U), wheel, false,
                                &in);
    (void)nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 0U), wheel, false,
                                &in);
    assert(in.b_ping);

    /* A pose reset closes the gate again. */
    nav_core_reset_pose(&core, wheel);
    (void)nav_core_follow_input(&core, status_word(OBSTACLE_CLASS_OPEN, 0U, 0U), wheel, true, &in);
    assert(!in.b_line);
}

int main(void)
{
    test_pose_straight_and_reset();
    test_pose_left_turn();
    test_servo_wait();
    test_repeat_gate();
    test_follow_input();
    printf("obstacle nav core: all tests passed\n");

    return 0;
}
