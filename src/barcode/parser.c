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

#include "barcode/parser.h"

#define BARCODE_CHARACTER_COUNT   3
#define CHARACTER_ELEM_COUNT      9
#define WIDE_SPACES_PER_CHARACTER 1
#define WIDE_BARS_PER_CHARACTER   2
#define DELIM_CHARACTER           43U

/* Assumes minimum 50mm and maximum 400mm for a 3-character barcode. */
/* Assumes the first element is narrow. */
#define START_MIN_UM 638U
#define START_MAX_UM 12765U
/* 16-bit fixed point. 2-bit integer, 14-bit fraction */
#define FP16_OFFSET        14U
#define NARROW_WIDTH_MASK  0x3FFFU
/* Limits based on empirical testing */
#define NARROW_MIN_SCALE 9830U  /* 0.6 */
#define NARROW_MAX_SCALE 24576U /* 1.5 */
#define WIDE_MIN_SCALE   39321U /* 2.4 */
#define WIDE_MAX_SCALE   58982U /* 3.6 */

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

typedef enum
{
    ELEM_TYPE_INVALID,
    ELEM_TYPE_NARROW,
    ELEM_TYPE_WIDE,
} elem_type_t;

static uint16_t fp16_mul (uint16_t a, uint16_t b)
{
    return (uint16_t)(((uint32_t)a * (uint32_t)b) >> FP16_OFFSET);
}

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

static elem_type_t get_elem_type (barcode_parser_t * state,
                                  uint16_t           elem_width_um)
{
    const uint16_t narrow_min =
        fp16_mul(state->narrow_width_um, NARROW_MIN_SCALE);
    const uint16_t narrow_max =
        fp16_mul(state->narrow_width_um, NARROW_MAX_SCALE);
    if ((elem_width_um >= narrow_min) && (elem_width_um <= narrow_max))
    {
        return ELEM_TYPE_NARROW;
    }

    const uint16_t wide_min = fp16_mul(state->narrow_width_um, WIDE_MIN_SCALE);
    const uint16_t wide_max = fp16_mul(state->narrow_width_um, WIDE_MAX_SCALE);
    if ((elem_width_um >= wide_min) && (elem_width_um <= wide_max))
    {
        return ELEM_TYPE_WIDE;
    }

    return ELEM_TYPE_INVALID;
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

/**
 * Add value of the this element to the current character,
 * and determine if the barcode is in reverse.
 * Returns whether there was an error.
 */
static bool handle_elem (barcode_parser_t * state, bool is_elem_space,
                         elem_type_t elem_type)
{
    const bool is_second_elem =
        (state->char_index == 0) && (state->elem_index == 1);
    switch (elem_type)
    {
        case ELEM_TYPE_NARROW:
            if (is_second_elem)
            {
                state->is_reverse = true;
            }
            break;
        case ELEM_TYPE_WIDE:
            if (is_second_elem)
            {
                state->is_reverse = false;
            }
            if (is_elem_space)
            {
                if (state->wide_space_count >= WIDE_SPACES_PER_CHARACTER)
                {
                    return true;
                }
                state->wide_space_count = (uint8_t)(state->wide_space_count + 1U);
                state->curr_value = (uint8_t)(state->curr_value +
                    decode_space_value(state->elem_index >> 1U,
                                       state->is_reverse));
            }
            else
            {
                if (state->wide_bar_count >= WIDE_BARS_PER_CHARACTER)
                {
                    return true;
                }
                state->wide_bar_count = (uint8_t)(state->wide_bar_count + 1U);
                state->curr_value = (uint8_t)(state->curr_value +
                    decode_bar_value(state->elem_index >> 1U,
                                     state->is_reverse));
            }
            break;
        case ELEM_TYPE_INVALID:
        default:
            return true;
    }
    state->elem_index = (uint8_t)(state->elem_index + 1U);
    return false;
}

static bool handle_end_of_character (barcode_parser_t * state,
                                     bool is_elem_space, elem_type_t elem_type,
                                     uint8_t * p_reason)
{
    if (state->wide_bar_count != WIDE_BARS_PER_CHARACTER ||
        state->wide_space_count != WIDE_SPACES_PER_CHARACTER)
    {
        *p_reason = BARCODE_REJECT_PARITY;
        return true;
    }
    /* The element between characters must be a narrow space */
    if (!is_elem_space || (elem_type != ELEM_TYPE_NARROW))
    {
        *p_reason = BARCODE_REJECT_ELEMENT;
        return true;
    }
    /* First character must be the delimiter */
    if ((state->char_index == 0) && (state->curr_value != DELIM_CHARACTER))
    {
        *p_reason = BARCODE_REJECT_DELIMITER;
        return true;
    }
    /* Second character contains the data */
    if (state->char_index == 1)
    {
        state->data_value = state->curr_value;
    }
    state->wide_bar_count   = 0;
    state->wide_space_count = 0;
    state->curr_value       = 0;
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
    bool        b_invalid = (expect_space != is_elem_space);
    elem_type_t elem_type = ELEM_TYPE_INVALID;

    *p_reason = BARCODE_REJECT_ELEMENT;

    if (!b_invalid && is_first_elem)
    {
        b_invalid = is_start_invalid(is_elem_space, elem_width_um);
        /* First element is assumed to be narrow */
        state->narrow_width_um =
            (uint16_t)((uint32_t)elem_width_um & NARROW_WIDTH_MASK);
    }

    if (!b_invalid)
    {
        elem_type = get_elem_type(state, elem_width_um);
        b_invalid = handle_elem(state, is_elem_space, elem_type);
    }

    if (!b_invalid && (state->elem_index > CHARACTER_ELEM_COUNT))
    {
        b_invalid = handle_end_of_character(state, is_elem_space, elem_type, p_reason);
    }

    return b_invalid;
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
        /* Last character must be the delimiter with valid parity */
        if ((state->wide_bar_count != WIDE_BARS_PER_CHARACTER) ||
            (state->wide_space_count != WIDE_SPACES_PER_CHARACTER))
        {
            b_invalid = true;
            reason    = BARCODE_REJECT_PARITY;
        }
        else if (state->curr_value != DELIM_CHARACTER)
        {
            b_invalid = true;
            reason    = BARCODE_REJECT_DELIMITER;
        }
        else
        {
            /* A closed barcode. */
        }
    }

    if (p_info != NULL)
    {
        p_info->narrow_width_um = state->narrow_width_um;
        p_info->is_reverse      = state->is_reverse;
        p_info->in_progress     = b_progress;
        p_info->reject = (b_invalid && b_progress) ? reason : BARCODE_REJECT_NONE;
    }

    if (b_invalid)
    {
        *state = BARCODE_PARSER_INIT;
    }
    else if (b_at_end)
    {
        result = decode_data(state->data_value);
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

/*** end of file ***/
