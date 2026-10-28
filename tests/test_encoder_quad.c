/** @file test_encoder_quad.c
 * @brief Exhaustive transition checks and counter failure cases.
 */
#include <stdio.h>
#include <stddef.h>
#include "encoder_quad.h"

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
    static const uint8_t positive_next[4] = {1U, 3U, 0U, 2U};
    static const uint8_t negative_next[4] = {2U, 0U, 3U, 1U};
    encoder_quad_t quad;
    unsigned int previous;
    unsigned int current;
    unsigned int i;
    bool result;

    for (previous = 0U; previous < 4U; previous++)
    {
        for (current = 0U; current < 4U; current++)
        {
            encoder_quad_init(&quad, (uint8_t)previous);
            result = encoder_quad_update(&quad, ENCODER_LEFT,
                                         (uint8_t)current, false);
            check(quad.states == current, "state resynchronises");
            check(quad.wheel[0].positive ==
                  (uint32_t)(current == positive_next[previous]),
                  "positive transitions");
            check(quad.wheel[0].negative ==
                  (uint32_t)(current == negative_next[previous]),
                  "negative transitions");
            check(quad.wheel[0].invalid ==
                  (uint32_t)((previous ^ current) == 3U),
                  "two-bit jump detected");
            check(result == ((previous ^ current) != 3U),
                  "invalid return value");
        }
    }

    encoder_quad_init(&quad, 0U);
    for (i = 0U; i < 100U; i++)
    {
        current = positive_next[quad.states & 3U];
        check(encoder_quad_update(&quad, 0U, (uint8_t)current, false),
              "positive sequence");
        current = negative_next[(quad.states >> 2U) & 3U];
        check(encoder_quad_update(&quad, 1U, (uint8_t)current, false),
              "independent negative sequence");
    }
    check(quad.wheel[0].positive == 100U && quad.wheel[1].negative == 100U,
          "left and right remain independent");
    check(quad.wheel[0].negative == 0U && quad.wheel[1].positive == 0U,
          "no cross-wheel updates");
    check(!encoder_quad_update(&quad, 0U, 1U, true) &&
          quad.wheel[0].invalid == 1U && quad.wheel[0].positive == 100U,
          "coalesced edges never counted as distance");
    check(encoder_quad_update(&quad, 0U, 3U, false) &&
          quad.wheel[0].positive == 101U, "recovery after ambiguous sample");
    quad.wheel[0].positive = UINT32_MAX;
    check(encoder_quad_update(&quad, 0U, 2U, false) &&
          quad.wheel[0].positive == 0U, "defined counter rollover");
    check(!encoder_quad_update(NULL, 0U, 0U, false), "null update");
    check(!encoder_quad_update(&quad, 2U, 0U, false), "invalid wheel");
    check(!encoder_quad_update(&quad, 0U, 4U, false), "invalid A/B value");
    encoder_quad_init(NULL, 0U);
    encoder_quad_init(&quad, UINT8_MAX);
    check(quad.states == 15U, "unused state bits cleared");
    (void)printf("quadrature: %u failures\n", failures);
    return failures == 0U ? 0 : 1;
}

/*** end of file ***/
