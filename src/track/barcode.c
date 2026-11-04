#include <stddef.h>

#include "track/barcode.h"

typedef struct
{
    uint8_t bar_a;
    uint8_t bar_b;
} digit_bars_t;

static const digit_bars_t DIGIT_BARS[10] = {
    {2U, 3U}, {0U, 4U}, {1U, 4U}, {0U, 1U}, {2U, 4U},
    {0U, 2U}, {1U, 2U}, {3U, 4U}, {0U, 3U}, {1U, 3U},
};

static uint16_t digit_pattern (uint8_t digit)
{
    const digit_bars_t bars = DIGIT_BARS[digit];
    uint16_t           pattern = 0U;

    pattern |= (uint16_t)(1U << (bars.bar_a * 2U));
    pattern |= (uint16_t)(1U << (bars.bar_b * 2U));
    pattern |= (uint16_t)(1U << 3U); /* wide space in the digit group */
    return pattern;
}

static uint16_t classify (const uint32_t widths[BARCODE_ELEMENTS])
{
    uint32_t sorted[BARCODE_ELEMENTS];
    uint32_t threshold;
    uint16_t pattern = 0U;
    unsigned int i;

    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        sorted[i] = widths[i];
    }
    for (i = 1U; i < BARCODE_ELEMENTS; i++)
    {
        const uint32_t value = sorted[i];
        unsigned int   j     = i;
        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }

    threshold = (sorted[5] + sorted[6]) / 2U;
    for (i = 0U; i < BARCODE_ELEMENTS; i++)
    {
        if (widths[i] > threshold)
        {
            pattern |= (uint16_t)(1U << i);
        }
    }
    return pattern;
}

void barcode_decoder_init (barcode_decoder_t * p_decoder)
{
    if (p_decoder == NULL)
    {
        return;
    }
    *p_decoder = (barcode_decoder_t){0};
    p_decoder->initialised = true;
}

void barcode_decoder_reset (barcode_decoder_t * p_decoder)
{
    if ((p_decoder == NULL) || !p_decoder->initialised)
    {
        return;
    }
    p_decoder->count     = 0U;
    p_decoder->have_last = false;
}

bool barcode_decode_edge (barcode_decoder_t * p_decoder, uint32_t now_us,
                          int32_t speed_um_s, char * p_char)
{
    int64_t width_um;
    uint16_t pattern;
    uint8_t  digit;

    if ((p_decoder == NULL) || !p_decoder->initialised || (p_char == NULL))
    {
        return false;
    }
    if (!p_decoder->have_last)
    {
        p_decoder->last_us   = now_us;
        p_decoder->have_last = true;
        return false;
    }

    width_um = ((int64_t)speed_um_s * (int64_t)(now_us - p_decoder->last_us)) /
               INT64_C(1000000);
    if (width_um < 0)
    {
        width_um = -width_um;
    }
    if (width_um > (int64_t)UINT32_MAX)
    {
        barcode_decoder_reset(p_decoder);
        return false;
    }

    p_decoder->last_us = now_us;
    p_decoder->widths_um[p_decoder->count++] = (uint32_t)width_um;
    if (p_decoder->count < BARCODE_ELEMENTS)
    {
        return false;
    }

    pattern = classify(p_decoder->widths_um);
    p_decoder->count = 0U;
    for (digit = 0U; digit < 10U; digit++)
    {
        if (pattern == digit_pattern(digit))
        {
            *p_char = (char)('0' + digit);
            return true;
        }
    }
    return false;
}
