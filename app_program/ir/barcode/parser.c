/** @file barcode_parser.c
 * @brief Parses single-character Code 39 barcodes. Only supports 'A' to 'Z'
 * characters.
 *
 * To simplify the implementation, the encoding used internally differs from
 * actual Code 39. The character encoding used internally is:
 * 'A'-'J' => 1-11
 * 'K'-'T' => 17-27
 * 'U'-'Z  => 33-38
 * *       => 43
 * else    => unsupported
 */
#include <stddef.h>

#include "ir/barcode/parser.h"

#define BARCODE_CHARACTER_COUNT   3
#define CHARACTER_ELEM_COUNT      9
#define WIDE_SPACES_PER_CHARACTER 1
#define WIDE_BARS_PER_CHARACTER   2
#define DELIM_CHARACTER           43U

/* Assumes minimum 50mm and maximum 400mm for a 3-character barcode. */
/* Assumes the first element is narrow. */
#define START_MIN_UM 638U
#define START_MAX_UM 12765U
#define PERCENT_FULL        (100U)
/* A gap must stay narrower than the narrowest printed wide space (2x).
 * The recorded reverse Z read its gaps at about 1.6x the narrow space. */
#define INTERCHAR_GAP_MAX_RATIO (2U)
#define GLITCH_MAX_PCT      (30U)
/* The narrowest wide element of a colour is more than 1.5 times its widest
 * narrow one. Code 39 prints wide elements 2 to 3 times the narrow. */
#define WIDE_SEPARATION_NUM (3U)
#define WIDE_SEPARATION_DEN (2U)
#define CHARACTER_BARS      (5U)
#define CHARACTER_SPACES    (4U)

/* Bar code table: 3 bits per entry, bar indices 0 to 4 */
#define BAR_CODE_BITS 3U
#define BAR_LAST_IDX  4U
/* Space table: index 2 => 0, 3 => 16, 0 => 32, 1 => 48 */
#define SPACE_LAST_IDX    3U
#define SPACE_IDX_OFFSET  2U
#define SPACE_IDX_MASK    0x3U
#define SPACE_VALUE_SHIFT 4U
/* Internal encoding: tens in the high nibble, ones (offset by 1) below */
#define DATA_TENS_SHIFT 4U
#define DATA_ONES_MASK  0xFU
#define DATA_ONES_BASE  1U
#define DATA_ONES_SKIP  10U
#define DATA_TENS_STEP  10U
#define LETTER_COUNT    26U

/**
 * Returns the value of a bar given its index.
 * Only valid when 0 <= idx <= 4.
 */
static uint8_t decode_bar_value (uint8_t idx, bool is_reverse)
{
    /* Packing codes together requires at most 4 bytes, which is smaller than
     * using an array, which requires at least 5 bytes. */
    static const uint32_t bits = BAR_CODE_BITS;
    static const uint32_t mask = (1U << BAR_CODE_BITS) - 1U;
    static const uint32_t codes =
        (1U << 0U) | (2U << 3U) | (4U << 6U) | (7U << 9U) | (0U << 12U);

    if (is_reverse)
    {
        idx = (uint8_t)(BAR_LAST_IDX - idx);
    }
    return (uint8_t)((codes >> ((uint32_t)idx * bits)) & mask);
}

/**
 * Returns the value of a space given its index.
 * Only valid when 0 <= idx <= 3.
 */
static uint8_t decode_space_value (uint8_t idx, bool is_reverse)
{
    /* Mapping for the following:
     * 2 =>  0
     * 3 => 16
     * 0 => 32
     * 1 => 48 */
    if (is_reverse)
    {
        idx = (uint8_t)(SPACE_LAST_IDX - idx);
    }
    return (uint8_t)((((uint32_t)idx - SPACE_IDX_OFFSET) & SPACE_IDX_MASK)
                     << SPACE_VALUE_SHIFT);
}

/** Converts the internal encoding to 0-25, representing 'A'-'Z'. */
static uint8_t decode_data (uint8_t data)
{
    const uint8_t tens = (uint8_t)(data >> DATA_TENS_SHIFT);
    uint8_t       ones =
        (uint8_t)(((uint32_t)data & DATA_ONES_MASK) - DATA_ONES_BASE);
    if (ones == DATA_ONES_SKIP)
    {
        ones = (uint8_t)(ones - 1U);
    }
    return (uint8_t)((tens * DATA_TENS_STEP) + ones);
}

/** Width classes of one colour: the `wide` widest elements are wide. */
typedef struct
{
    uint16_t wide_mask;   /* element indices of the wide elements */
    uint32_t narrow_sum;  /* summed narrow widths                 */
    bool     b_separated; /* wide and narrow widths are distinct  */
} width_rank_t;

/* Ranking within one colour cancels the reader's bar and space bias, and
 * the speed error between distant elements of the character. */
static width_rank_t rank_widths (const uint16_t * p_width, uint8_t first, uint8_t count,
                                 uint8_t wide)
{
    width_rank_t rank = {0U, 0U, false};
    uint32_t     min_wide = UINT32_MAX;
    uint32_t     max_narrow = 0U;

    for (uint8_t n = 0U; n < wide; n++)
    {
        uint8_t widest = CHARACTER_ELEM_COUNT;
        for (uint8_t k = 0U; k < count; k++)
        {
            uint8_t const i = (uint8_t)(first + (2U * k));
            if ((0U == (rank.wide_mask & (1U << i))) &&
                ((CHARACTER_ELEM_COUNT == widest) || (p_width[i] > p_width[widest])))
            {
                widest = i;
            }
        }
        rank.wide_mask = (uint16_t)(rank.wide_mask | (1U << widest));
        min_wide = (p_width[widest] < min_wide) ? p_width[widest] : min_wide;
    }
    for (uint8_t k = 0U; k < count; k++)
    {
        uint8_t const i = (uint8_t)(first + (2U * k));
        if (0U == (rank.wide_mask & (1U << i)))
        {
            rank.narrow_sum += p_width[i];
            max_narrow = (p_width[i] > max_narrow) ? p_width[i] : max_narrow;
        }
    }
    rank.b_separated = (min_wide * WIDE_SEPARATION_DEN) > (max_narrow * WIDE_SEPARATION_NUM);
    return rank;
}

static uint8_t character_value (uint16_t wide_mask, bool is_reverse)
{
    uint8_t value = 0U;

    for (uint8_t i = 0U; i < CHARACTER_ELEM_COUNT; i++)
    {
        if (0U != (wide_mask & (1U << i)))
        {
            uint8_t const idx = (uint8_t)(i >> 1U);
            value = (uint8_t)(value + ((0U != (i & 1U)) ? decode_space_value(idx, is_reverse)
                                                         : decode_bar_value(idx, is_reverse)));
        }
    }
    return value;
}

/**
 * Classify the nine elements of a complete character.
 * Returns whether the character is invalid.
 */
static bool classify_character (barcode_parser_t * state, uint8_t * p_reason)
{
    width_rank_t const bars = rank_widths(state->elem_width_um, 0U, CHARACTER_BARS,
                                          WIDE_BARS_PER_CHARACTER);
    width_rank_t const spaces = rank_widths(state->elem_width_um, 1U, CHARACTER_SPACES,
                                            WIDE_SPACES_PER_CHARACTER);
    uint16_t const wide_mask = (uint16_t)(bars.wide_mask | spaces.wide_mask);

    if (!bars.b_separated || !spaces.b_separated)
    {
        *p_reason = BARCODE_REJECT_PARITY;
        return true;
    }
    if (0U == state->char_index)
    {
        /* The delimiter is not symmetric, so it gives the direction. */
        state->is_reverse = (DELIM_CHARACTER != character_value(wide_mask, false));
    }
    uint8_t const value = character_value(wide_mask, state->is_reverse);
    if ((1U != state->char_index) && (DELIM_CHARACTER != value))
    {
        *p_reason = BARCODE_REJECT_DELIMITER;
        return true;
    }
    if (1U == state->char_index)
    {
        state->data_value = value;
    }
    state->narrow_width_um = (uint16_t)(bars.narrow_sum / (CHARACTER_BARS - WIDE_BARS_PER_CHARACTER));
    state->narrow_space_um = (uint16_t)(spaces.narrow_sum / (CHARACTER_SPACES - WIDE_SPACES_PER_CHARACTER));
    return false;
}

static bool handle_end_of_character (barcode_parser_t * state,
                                     uint16_t gap_um,
                                     uint8_t * p_reason)
{
    /* A separator is not a character element and cannot train its widths. */
    if ((0U == gap_um) || (gap_um >= (uint32_t)state->narrow_space_um * INTERCHAR_GAP_MAX_RATIO))
    {
        *p_reason = BARCODE_REJECT_ELEMENT;
        return true;
    }
    state->elem_index       = 0;
    state->char_index += 1;
    return false;
}

/** Returns whether the first element is not a valid start element. */
static bool is_start_invalid (bool is_elem_space, uint16_t elem_width_um)
{
    return (is_elem_space || (elem_width_um < START_MIN_UM) ||
            (elem_width_um > START_MAX_UM));
}

/**
 * Checks and records one element, and handles the end of a character.
 * Returns whether the element is invalid.
 */
static bool process_elem (barcode_parser_t * state, bool is_elem_space,
                          uint16_t elem_width_um, uint8_t * p_reason)
{
    const bool expect_space = (((uint32_t)state->elem_index & 1U) == 1U);
    const bool is_first_elem =
        (state->char_index == 0) && (state->elem_index == 0);

    *p_reason = BARCODE_REJECT_ELEMENT;

    if (expect_space != is_elem_space)
    {
        return true;
    }
    if (state->elem_index == CHARACTER_ELEM_COUNT)
    {
        return handle_end_of_character(state, elem_width_um, p_reason);
    }
    if (is_first_elem)
    {
        if (is_start_invalid(is_elem_space, elem_width_um))
        {
            return true;
        }
        /* Scales glitches until the start character is classified. */
        state->narrow_width_um = elem_width_um;
    }
    if (0U == elem_width_um)
    {
        return true;
    }
    state->elem_width_um[state->elem_index] = elem_width_um;
    state->elem_index = (uint8_t)(state->elem_index + 1U);
    return (state->elem_index == CHARACTER_ELEM_COUNT) && classify_character(state, p_reason);
}

uint8_t barcode_parser_input_info (barcode_parser_t * state, bool is_elem_space,
                                   uint16_t elem_width_um, barcode_info_t * p_info)
{
    uint8_t    result      = BARCODE_NO_CHAR;
    uint8_t    reason      = BARCODE_REJECT_NONE;
    bool const b_progress  = (state->char_index != 0) || (state->elem_index != 0);
    bool       b_invalid   = process_elem(state, is_elem_space, elem_width_um, &reason);
    const bool b_at_end =
        !b_invalid && (state->char_index == (BARCODE_CHARACTER_COUNT - 1)) &&
        (state->elem_index == CHARACTER_ELEM_COUNT);

    if (b_at_end)
    {
        result = decode_data(state->data_value);
        if (result >= LETTER_COUNT)
        {
            b_invalid = true;
            reason    = BARCODE_REJECT_SYMBOL;
            result    = BARCODE_NO_CHAR;
        }
    }

    if (p_info != NULL)
    {
        p_info->narrow_width_um = state->narrow_width_um;
        p_info->is_reverse      = state->is_reverse;
        p_info->in_progress     = b_progress;
        p_info->reject = (b_invalid && b_progress) ? reason : BARCODE_REJECT_NONE;
    }

    if (b_invalid || b_at_end)
    {
        *state = BARCODE_PARSER_INIT;
    }
    else
    {
        /* Not at the end of barcode yet */
    }

    return result;
}

uint8_t barcode_parser_input (barcode_parser_t * state, bool is_elem_space,
                              uint16_t elem_width_um)
{
    return barcode_parser_input_info(state, is_elem_space, elem_width_um, NULL);
}

uint16_t barcode_parser_glitch_limit_um (const barcode_parser_t * state)
{
    return (uint16_t)((uint32_t)state->narrow_width_um * GLITCH_MAX_PCT / PERCENT_FULL);
}

bool barcode_parser_is_glitch (barcode_parser_t const * state, uint16_t elem_width_um)
{
    return elem_width_um <= barcode_parser_glitch_limit_um(state);
}

/*** end of file ***/
