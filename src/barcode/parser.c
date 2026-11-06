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
#include "barcode/parser.h"

#define BARCODE_CHARACTER_COUNT   3
#define CHARACTER_ELEM_COUNT      9
#define WIDE_SPACES_PER_CHARACTER 1
#define WIDE_BARS_PER_CHARACTER   2
#define DELIM_CHARACTER           43U

// Assumes minimum 50mm and maximum 400mm for a 3-character barcode.
// Assumes the first element is narrow.
#define START_MIN_UM 638U
#define START_MAX_UM 12765U
// 16-bit fixed point. 2-bit integer, 14-bit fraction
#define FP16_OFFSET 14U
// Limits based on empirical testing
#define NARROW_MIN_SCALE 9830U  // 0.6
#define NARROW_MAX_SCALE 24576U // 1.5
#define WIDE_MIN_SCALE   39321U // 2.4
#define WIDE_MAX_SCALE   58982U // 3.6

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
    // Packing codes together requires at most 4 bytes, which is smaller than
    // using an array, which requires at least 5 bytes.
    static const uint8_t  bits = 3U;
    static const uint8_t  mask = (1U << bits) - 1U;
    static const uint16_t codes =
        (1U << 0) | (2U << 3) | (4U << 6) | (7U << 9) | (0U << 12);

    if (is_reverse)
    {
        idx = 4U - idx;
    }
    return (codes >> (idx * bits)) & mask;
}

/**
 * Returns the value of a space given its index.
 * Only valid when 0 <= idx <= 3.
 */
static uint8_t decode_space_value (uint8_t idx, bool is_reverse)
{
    // Mapping for the following:
    // 2 =>  0
    // 3 => 16
    // 0 => 32
    // 1 => 48
    if (is_reverse)
    {
        idx = 3U - idx;
    }
    return ((idx - 2U) & 0x3U) << 4U;
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
    const uint8_t tens = data >> 4U;
    uint8_t       ones = (data & 0xF) - 1;
    if (ones == 10)
    {
        ones -= 1;
    }
    return (uint8_t)((tens * 10U) + ones);
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
                    decode_space_value(state->elem_index >> 1,
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
                    decode_bar_value(state->elem_index >> 1,
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
                                     bool is_elem_space, elem_type_t elem_type)
{
    if (state->wide_bar_count != WIDE_BARS_PER_CHARACTER ||
        state->wide_space_count != WIDE_SPACES_PER_CHARACTER)
    {
        return true;
    }
    // The element between characters must be a narrow space
    if (!is_elem_space || (elem_type != ELEM_TYPE_NARROW))
    {
        return true;
    }
    // First character must be the delimiter
    if ((state->char_index == 0) && (state->curr_value != DELIM_CHARACTER))
    {
        return true;
    }
    // Second character contains the data
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

uint8_t barcode_parser_input (barcode_parser_t * state, bool is_elem_space,
                              uint16_t elem_width_um)
{
    const bool expect_space = (state->elem_index & 1) == 1;
    if (expect_space != is_elem_space)
    {
        goto invalid;
    }

    const bool is_first_elem =
        (state->char_index == 0) && (state->elem_index == 0);
    if (is_first_elem)
    {
        if (is_elem_space)
        {
            goto invalid;
        }
        if ((elem_width_um < START_MIN_UM) || (elem_width_um > START_MAX_UM))
        {
            goto invalid;
        }
        // First element is assumed to be narrow
        state->narrow_width_um = (uint16_t)(elem_width_um & 0x3FFFU);
    }

    const elem_type_t elem_type = get_elem_type(state, elem_width_um);
    if (handle_elem(state, is_elem_space, elem_type))
    {
        goto invalid;
    }

    if (state->elem_index > CHARACTER_ELEM_COUNT)
    {
        if (handle_end_of_character(state, is_elem_space, elem_type))
        {
            goto invalid;
        }
    }

    // End of barcode
    if ((state->char_index == (BARCODE_CHARACTER_COUNT - 1)) &&
        (state->elem_index == CHARACTER_ELEM_COUNT))
    {
        // Last character must be the delimiter
        if (state->curr_value != DELIM_CHARACTER)
        {
            goto invalid;
        }
        const uint8_t data = decode_data(state->data_value);
        *state             = BARCODE_PARSER_INIT;
        return data;
    }

    // Not at the end of barcode yet
    return BARCODE_NO_CHAR;

invalid:
    *state = BARCODE_PARSER_INIT;
    return BARCODE_NO_CHAR;
}

/*** end of file ***/
