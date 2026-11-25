/** @file obstacle_ping_test.c
 *
 * @brief Host tests for the single-ping request word and the reply word:
 *        pack and unpack, the reply flag, the sequence match, a request
 *        that replaces one not yet started, and the clamped angle.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "motion/pilot/look.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            (void)fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,   \
                          __LINE__, #cond);                                \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static void test_request_round_trip(void)
{
    int32_t const angles[] = {-300, -150, -1, 0, 1, 250, 300};
    uint32_t idx = 0U;

    for (idx = 0U; idx < (sizeof(angles) / sizeof(angles[0])); idx++)
    {
        uint32_t const word = obstacle_pack_ping_req(angles[idx], 41U);

        CHECK(obstacle_pack_ping_req_is_pending(word));
        CHECK(obstacle_pack_ping_req_servo(word) == angles[idx]);
        CHECK(obstacle_pack_ping_req_seq(word) == 41U);
    }
}

static void test_request_is_never_zero(void)
{
    /* Angle 0 with sequence 0 must still read as pending. */
    CHECK(obstacle_pack_ping_req(0, 0U) != 0U);
    CHECK(obstacle_pack_ping_req_is_pending(obstacle_pack_ping_req(0, 0U)));
    CHECK(!obstacle_pack_ping_req_is_pending(0U));
}

static void test_request_sequence_is_six_bits(void)
{
    uint32_t const word = obstacle_pack_ping_req(100, 64U + 5U);

    CHECK(obstacle_pack_ping_req_seq(word) == 5U);
    CHECK(obstacle_pack_ping_req_servo(word) == 100);
}

static void test_replace_before_start(void)
{
    uint32_t slot = 0U;

    slot = obstacle_pack_ping_req(-200, 3U);
    slot = obstacle_pack_ping_req(150, 4U);

    CHECK(obstacle_pack_ping_req_is_pending(slot));
    CHECK(obstacle_pack_ping_req_servo(slot) == 150);
    CHECK(obstacle_pack_ping_req_seq(slot) == 4U);
}

static void test_clamp(void)
{
    int32_t const limit = (int32_t)OBSTACLE_SERVO_LIMIT_DECI;

    CHECK(obstacle_pack_servo_clamp(0) == 0);
    CHECK(obstacle_pack_servo_clamp(limit) == limit);
    CHECK(obstacle_pack_servo_clamp(-limit) == -limit);
    CHECK(obstacle_pack_servo_clamp(limit + 1) == limit);
    CHECK(obstacle_pack_servo_clamp(-limit - 1) == -limit);
    CHECK(obstacle_pack_servo_clamp(32000) == limit);
    CHECK(obstacle_pack_servo_clamp(-32000) == -limit);
}

static void test_clamped_request_reply_carries_used_angle(void)
{
    uint32_t const req   = obstacle_pack_ping_req(900, 12U);
    int32_t const  used  = obstacle_pack_servo_clamp(obstacle_pack_ping_req_servo(req));
    uint32_t const reply = obstacle_pack_ping_reply(obstacle_pack_ping_req_seq(req),
                                                    OBSTACLE_CLASS_VALID, 321U, used);

    CHECK(used == (int32_t)OBSTACLE_SERVO_LIMIT_DECI);
    CHECK(obstacle_pack_ping_servo(reply) == (int32_t)OBSTACLE_SERVO_LIMIT_DECI);
    CHECK(obstacle_pack_ping_answers(reply, 12U));
}

static void test_reply_flag(void)
{
    uint32_t const free_run = obstacle_pack_ping(9U, OBSTACLE_CLASS_VALID, 400U, 100);
    uint32_t const reply    = obstacle_pack_ping_reply(9U, OBSTACLE_CLASS_VALID, 400U, 100);

    CHECK(!obstacle_pack_ping_is_reply(free_run));
    CHECK(obstacle_pack_ping_is_reply(reply));
    /* The flag leaves every other field as in a free-running word. */
    CHECK((reply & ~(1U << OBSTACLE_PING_REPLY_SH)) == free_run);
    CHECK(obstacle_pack_get(reply, OBSTACLE_PING_CLASS_SH, OBSTACLE_PING_CLASS_MK) ==
          OBSTACLE_CLASS_VALID);
    CHECK(obstacle_pack_get(reply, OBSTACLE_PING_DIST_SH, OBSTACLE_PING_DIST_MK) == 400U);
    CHECK(obstacle_pack_ping_servo(reply) == 100);
}

static void test_sequence_match(void)
{
    uint32_t const reply    = obstacle_pack_ping_reply(20U, OBSTACLE_CLASS_OPEN, 0U, 0);
    uint32_t const free_run = obstacle_pack_ping(20U, OBSTACLE_CLASS_OPEN, 0U, 0);

    CHECK(obstacle_pack_ping_answers(reply, 20U));
    CHECK(!obstacle_pack_ping_answers(reply, 21U));
    /* A free-running ping with the same sequence is not an answer. */
    CHECK(!obstacle_pack_ping_answers(free_run, 20U));
    CHECK(obstacle_pack_ping_answers(reply, 20U + 64U));
}

static void test_requested_bearing(void)
{
    CHECK(obstacle_ping_reply_matches(true, 0U, 100, 0U, 100, 0U));
    CHECK(!obstacle_ping_reply_matches(false, 0U, 100, 0U, 100, 0U));
    CHECK(!obstacle_ping_reply_matches(true, 63U, 100, 0U, 100, 0U));
    CHECK(!obstacle_ping_reply_matches(true, 0U, 100, 0U, 101, 0U));

    /* The servo reports whole degrees for requests in tenths of a degree. */
    uint8_t const tolerance = OBSTACLE_PING_MAX_QUANTIZATION_DECI;
    CHECK(obstacle_ping_reply_matches(true, 1U, 100, 1U, 109, tolerance));
    CHECK(obstacle_ping_reply_matches(true, 1U, -100, 1U, -109, tolerance));
    CHECK(!obstacle_ping_reply_matches(true, 1U, 100, 1U, 110, tolerance));
    CHECK(!obstacle_ping_reply_matches(true, 1U, -100, 1U, -110, tolerance));
}

static void test_failure_classes_are_carried(void)
{
    uint32_t const stuck = obstacle_pack_ping_reply(1U, OBSTACLE_CLASS_STUCK, 0U, -50);
    uint32_t const none  = obstacle_pack_ping_reply(2U, OBSTACLE_CLASS_NO_ECHO, 0U, 50);

    CHECK(obstacle_pack_get(stuck, OBSTACLE_PING_CLASS_SH, OBSTACLE_PING_CLASS_MK) ==
          OBSTACLE_CLASS_STUCK);
    CHECK(obstacle_pack_get(none, OBSTACLE_PING_CLASS_SH, OBSTACLE_PING_CLASS_MK) ==
          OBSTACLE_CLASS_NO_ECHO);
    CHECK(obstacle_pack_ping_servo(stuck) == -50);
    CHECK(obstacle_pack_ping_servo(none) == 50);
}

static void test_decode_echo_class(void)
{
    uint32_t cls = 0U;

    /* Only VALID, OPEN and TOO_CLOSE are readings. NO_ECHO, GLITCH and STUCK
     * say nothing about the scene, so they never read as OPEN. */
    CHECK(obstacle_pack_ping_echo(obstacle_pack_ping(1U, OBSTACLE_CLASS_VALID, 300U, 0)) ==
          LOOK_ECHO_VALID);
    CHECK(obstacle_pack_ping_echo(obstacle_pack_ping(1U, OBSTACLE_CLASS_OPEN, 0U, 0)) ==
          LOOK_ECHO_OPEN);
    CHECK(obstacle_pack_ping_echo(obstacle_pack_ping(1U, OBSTACLE_CLASS_TOO_CLOSE, 0U, 0)) ==
          LOOK_ECHO_TOO_CLOSE);
    for (cls = OBSTACLE_CLASS_NO_ECHO; cls <= OBSTACLE_CLASS_STUCK; cls++)
    {
        uint8_t const echo = obstacle_pack_ping_echo(obstacle_pack_ping_reply(1U, cls, 0U, 0));

        CHECK(echo == LOOK_ECHO_NONE);
        CHECK(echo != LOOK_ECHO_OPEN);
    }
    /* A class code that no ping carries is unusable too. */
    CHECK(obstacle_pack_ping_echo(obstacle_pack_ping(1U, 7U, 0U, 0)) == LOOK_ECHO_NONE);
}

static void test_decode_range_and_seq(void)
{
    uint32_t const valid = obstacle_pack_ping_reply(37U, OBSTACLE_CLASS_VALID, 412U, -120);
    uint32_t const close = obstacle_pack_ping(5U, OBSTACLE_CLASS_TOO_CLOSE, 0U, 0);
    uint32_t const open  = obstacle_pack_ping(6U, OBSTACLE_CLASS_OPEN, 3000U, 0);

    CHECK(obstacle_pack_ping_mm(valid) == 412U);
    CHECK(obstacle_pack_ping_seq(valid) == 37U);
    CHECK(obstacle_pack_ping_is_reply(valid));
    CHECK(obstacle_pack_ping_servo(valid) == -120);
    /* Too close counts as a hit at the minimum range, as in the statechart. */
    CHECK(obstacle_pack_ping_mm(close) == (uint16_t)OBSTACLE_MIN_RANGE_MM);
    CHECK(obstacle_pack_ping_mm(close) > 0U);
    /* OPEN has no range. */
    CHECK(obstacle_pack_ping_mm(open) == 0U);
    CHECK(obstacle_pack_ping_seq(open) == 6U);
}

static void test_qualified_limits(void)
{
    for (int32_t side = -1; side <= 1; side += 2)
    {
        CHECK(obstacle_pack_servo_clamp(side * 300) == side * 300);
        CHECK(obstacle_pack_servo_clamp(side * 310) == side * 300);
        uint32_t const reply = obstacle_pack_ping_reply(63U, OBSTACLE_CLASS_VALID, 321U, side * 300);
        CHECK(obstacle_pack_ping_servo(reply) == side * 300);
        CHECK(obstacle_pack_ping_answers(reply, 63U));
    }
}

static void test_candidate_reply_angles(void)
{
    for (int32_t side = -1; side <= 1; side += 2)
    {
        int32_t const angle = side * 350;
        uint32_t const reply = obstacle_pack_ping_reply(63U, OBSTACLE_CLASS_VALID, 400U, angle);
        CHECK(obstacle_pack_ping_answers(reply, 63U));
        CHECK(obstacle_pack_ping_servo(reply) == angle);
        CHECK(obstacle_pack_ping_mm(reply) == 400U);
        CHECK(!obstacle_ping_reply_matches(true, 63U, (int16_t)obstacle_pack_ping_servo(reply),
                                           63U, -angle, OBSTACLE_PING_MAX_QUANTIZATION_DECI));
    }
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "qualified-limits") == 0)
    {
        test_qualified_limits();
        return g_failures != 0;
    }
    if (argc == 2 && strcmp(argv[1], "candidate-reply-angles") == 0)
    {
        test_candidate_reply_angles();
        return g_failures != 0;
    }
    if (argc != 1)
    {
        return 1;
    }
    test_decode_echo_class();
    test_decode_range_and_seq();
    test_request_round_trip();
    test_request_is_never_zero();
    test_request_sequence_is_six_bits();
    test_replace_before_start();
    test_clamp();
    test_clamped_request_reply_carries_used_angle();
    test_reply_flag();
    test_sequence_match();
    test_requested_bearing();
    test_failure_classes_are_carried();

    if (0 != g_failures)
    {
        (void)fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    (void)printf("test_obstacle_ping: all checks passed\n");
    return 0;
}
