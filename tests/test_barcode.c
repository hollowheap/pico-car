#include <stdint.h>
#include <stdio.h>

#include "track/barcode.h"

static unsigned int failures;

static void check (int condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

static uint16_t digit_pattern (uint8_t digit)
{
    static const uint8_t bars[10][2] = {
        {2U, 3U}, {0U, 4U}, {1U, 4U}, {0U, 1U}, {2U, 4U},
        {0U, 2U}, {1U, 2U}, {3U, 4U}, {0U, 3U}, {1U, 3U},
    };
    uint16_t pattern = (uint16_t)(1U << 3U);
    pattern |= (uint16_t)(1U << (bars[digit][0] * 2U));
    pattern |= (uint16_t)(1U << (bars[digit][1] * 2U));
    return pattern;
}

static void feed_pattern (barcode_decoder_t * p_decoder, uint16_t pattern,
                          char * p_char)
{
    uint32_t now = 1000U;

    (void)barcode_decode_edge(p_decoder, now, 1000, p_char);
    for (unsigned int i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        const uint32_t width = (pattern & (1U << i)) != 0U ? 3000U : 1000U;
        now += width * 1000U;
        (void)barcode_decode_edge(p_decoder, now, 1000, p_char);
    }
}

int main (void)
{
    barcode_decoder_t decoder;
    char              decoded = 0;

    barcode_decoder_init(&decoder);
    feed_pattern(&decoder, digit_pattern(3U), &decoded);
    check(decoded == '3', "digit 3");

    barcode_decoder_init(&decoder);
    feed_pattern(&decoder, digit_pattern(7U), &decoded);
    check(decoded == '7', "digit 7");

    barcode_decoder_init(&decoder);
    feed_pattern(&decoder, 0x000U, &decoded);
    check(decoded != '0', "unknown pattern");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("barcode decoder OK\n");
    return 0;
}
