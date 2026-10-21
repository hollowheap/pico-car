/** @file test_encoder_math.c
 * @brief Known-answer and failure-contract tests for encoder calculations.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "encoder_math.h"

static unsigned int failures;

static void check (bool condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

int main (void)
{
    uint32_t result = UINT32_C(123);
    uint32_t left;
    uint32_t right;

    check(encoder_math_count_delta(UINT32_C(10), UINT32_C(13)) == UINT32_C(3),
          "ordinary count difference");
    check(encoder_math_count_delta(UINT32_MAX - UINT32_C(1), UINT32_C(2)) ==
          UINT32_C(4), "count wraps across zero");
    check(encoder_math_count_delta(UINT32_C(9), UINT32_C(9)) == UINT32_C(0),
          "unchanged counter");

    check(encoder_math_distance_um(10, 210000, 20, &result) && result == 105000,
          "half revolution uses calibrated wheel travel");
    check(encoder_math_distance_um(1, 10, 3, &result) && result == 3,
          "distance rounds down once");
    check(encoder_math_distance_um(0, 210000, 20, &result) && result == 0,
          "zero edges is a valid distance");
    check(encoder_math_distance_um(UINT32_MAX, UINT32_MAX, UINT32_MAX, &result)
          && result == UINT32_MAX, "maximum representable distance");

    result = 123;
    check(!encoder_math_distance_um(UINT32_MAX, 2, 1, &result) && result == 123,
          "distance overflow preserves output");
    check(!encoder_math_distance_um(1, 0, 20, &result) && result == 123,
          "unknown wheel calibration is rejected");
    check(!encoder_math_distance_um(1, 210000, 0, &result) && result == 123,
          "zero edges per revolution is rejected");
    check(!encoder_math_distance_um(1, 210000, 20, NULL),
          "null distance output");

    check(encoder_math_speed_um_s(1, 100000, 300000, 20, &result) &&
          result == 150000, "known speed in micrometres per second");
    check(encoder_math_speed_um_s(1, 1, 1, 3, &result) && result == 333333,
          "speed retains fractional distance until final division");
    check(encoder_math_speed_um_s(0, 10000, 210000, 20, &result) && result == 0,
          "zero edges is a valid sampled speed");
    check(encoder_math_speed_um_s(UINT32_MAX, UINT32_MAX, UINT32_MAX,
                                 UINT32_MAX, &result) && result == 1000000,
          "large products do not overflow");
    check(encoder_math_speed_um_s(UINT32_MAX - 1, UINT32_MAX, UINT32_MAX,
                                 UINT32_MAX, &result) && result == 999999,
          "large fractional product rounds correctly");
    check(encoder_math_speed_um_s(UINT32_MAX, 1000000, 1, 1, &result) &&
          result == UINT32_MAX, "maximum representable speed");

    result = 123;
    check(!encoder_math_speed_um_s(UINT32_MAX, 1, UINT32_MAX, 1, &result) &&
          result == 123, "speed overflow preserves output");
    check(!encoder_math_speed_um_s(1, 0, 210000, 20, &result) && result == 123,
          "zero measurement time is rejected");
    check(!encoder_math_speed_um_s(1, 10000, 0, 20, &result) && result == 123,
          "speed requires wheel calibration");
    check(!encoder_math_speed_um_s(1, 10000, 210000, 0, &result) &&
          result == 123,
          "speed requires edges per revolution");
    check(!encoder_math_speed_um_s(1, 10000, 210000, 20, NULL),
          "null speed output");

    check(encoder_math_speed_um_s(2, 100000, 200000, 20, &left) &&
          encoder_math_speed_um_s(3, 100000, 210000, 30, &right) &&
          left == 200000 && right == 210000,
          "left and right wheels can use different calibrations");

    if (failures == 0)
    {
        (void)puts("23 encoder math cases passed");
    }
    return (failures == 0) ? 0 : 1;
}

/*** end of file ***/
