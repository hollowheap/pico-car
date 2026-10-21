/** @file encoder_math.c
 *
 * @brief Overflow-checked encoder conversions without floating point.
 */

#include <stddef.h>
#include <stdint.h>

#include "encoder_math.h"

#define MICROSECONDS_PER_SECOND UINT32_C(1000000)

/* Compute floor(remainder * multiplier / denominator) without forming the
 * potentially overflowing product. Each step extends a binary prefix of the
 * multiplier. The remainder stays below denominator, and subtraction tests
 * make both modular additions safe even when denominator is near UINT64_MAX.
 * Preconditions: denominator > 0 and remainder < denominator. */
static uint32_t fractional_product (uint64_t remainder,
                                   uint32_t multiplier,
                                   uint64_t denominator)
{
    uint64_t accumulator = UINT64_C(0);
    uint32_t quotient = UINT32_C(0);
    uint32_t mask;

    for (mask = UINT32_C(1) << 31; mask != UINT32_C(0); mask >>= 1)
    {
        quotient *= UINT32_C(2);
        if (accumulator >= (denominator - accumulator))
        {
            accumulator -= denominator - accumulator;
            quotient++;
        }
        else
        {
            accumulator += accumulator;
        }

        if ((multiplier & mask) != UINT32_C(0))
        {
            if (accumulator >= (denominator - remainder))
            {
                accumulator -= denominator - remainder;
                quotient++;
            }
            else
            {
                accumulator += remainder;
            }
        }
    }

    return quotient;
}

uint32_t encoder_math_count_delta (uint32_t previous, uint32_t current)
{
    return (uint32_t)(current - previous);
}

bool encoder_math_distance_um (uint32_t count,
                              uint32_t wheel_travel_um,
                              uint32_t edges_per_revolution,
                              uint32_t * p_distance_um)
{
    uint64_t distance;

    if ((p_distance_um == NULL) || (wheel_travel_um == UINT32_C(0)) ||
        (edges_per_revolution == UINT32_C(0)))
    {
        return false;
    }

    /* Both factors are 32-bit, so the product fits in 64 bits. */
    distance = ((uint64_t)count * wheel_travel_um) / edges_per_revolution;
    if (distance > UINT32_MAX)
    {
        return false;
    }

    *p_distance_um = (uint32_t)distance;
    return true;
}

bool encoder_math_speed_um_s (uint32_t count,
                             uint32_t elapsed_us,
                             uint32_t wheel_travel_um,
                             uint32_t edges_per_revolution,
                             uint32_t * p_speed_um_s)
{
    uint64_t numerator;
    uint64_t denominator;
    uint64_t whole;
    uint64_t speed;
    uint32_t fraction;

    if ((p_speed_um_s == NULL) || (elapsed_us == UINT32_C(0)) ||
        (wheel_travel_um == UINT32_C(0)) ||
        (edges_per_revolution == UINT32_C(0)))
    {
        return false;
    }

    numerator = (uint64_t)count * wheel_travel_um;
    denominator = (uint64_t)edges_per_revolution * elapsed_us;
    whole = numerator / denominator;
    if (whole > (UINT32_MAX / MICROSECONDS_PER_SECOND))
    {
        return false;
    }

    fraction = fractional_product(numerator % denominator,
                                  MICROSECONDS_PER_SECOND, denominator);
    speed = (whole * MICROSECONDS_PER_SECOND) + fraction;
    if (speed > UINT32_MAX)
    {
        return false;
    }

    *p_speed_um_s = (uint32_t)speed;
    return true;
}

/*** end of file ***/
