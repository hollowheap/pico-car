/** @file test_barcode_parser.c
 * @brief Sanity check on a few barcodes.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "barcode/parser.h"

#define NARROW_UM 4255U
#define WIDE_UM   12766U

static uint8_t char_to_data (char c)
{
    return c - 'A';
}

static uint8_t put_char (barcode_parser_t * parser, uint8_t wide_space,
                         uint8_t wide_bar1, uint8_t wide_bar2)
{
    uint8_t result = 0;
    for (int i = 0; i < 9; i++)
    {
        const bool     is_space = (i % 2) == 1;
        const uint16_t width =
            (is_space && (i == wide_space))                         ? WIDE_UM
            : (!is_space && ((i == wide_bar1) || (i == wide_bar2))) ? WIDE_UM
                                                                    : NARROW_UM;
        result = barcode_parser_input(parser, is_space, width);
        if (i < 8)
        {
            assert(BARCODE_NO_CHAR == result);
        }
    }
    return result;
}

static void put_space (barcode_parser_t * parser)
{
    assert(BARCODE_NO_CHAR == barcode_parser_input(parser, true, NARROW_UM));
}

static void assert_parser_reset (const barcode_parser_t * parser)
{
    assert(parser->elem_index == 0);
    assert(parser->char_index == 0);
}

int main (void)
{
    // Forwards
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); // A
    put_space(&parser);
    assert(char_to_data('A') == put_char(&parser, 1, 4, 6)); // *

    // Forwards
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 8)); // L
    put_space(&parser);
    assert(char_to_data('L') == put_char(&parser, 1, 4, 6)); // *

    // Forwards
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 2, 4)); // Z
    put_space(&parser);
    assert(char_to_data('Z') == put_char(&parser, 1, 4, 6)); // *

    // Reversed
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 3, 8, 0)); // A
    put_space(&parser);
    assert(char_to_data('A') == put_char(&parser, 7, 2, 4)); // *

    // Reversed
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 0, 6)); // L
    put_space(&parser);
    assert(char_to_data('L') == put_char(&parser, 7, 2, 4)); // *

    // Reversed
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 4, 6)); // Z
    put_space(&parser);
    assert(char_to_data('Z') == put_char(&parser, 7, 2, 4)); // *

    // Forwards
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); // *
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, WIDE_UM));
    assert_parser_reset(&parser);

    // Forwards
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); // *
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, NARROW_UM));
    assert_parser_reset(&parser);

    // Forwards
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); // *
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, 1));
    assert_parser_reset(&parser);
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 65000));
    assert_parser_reset(&parser);
    // Reversed
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); // *
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 0, 6)); // L
    put_space(&parser);
    assert(char_to_data('L') == put_char(&parser, 7, 2, 4)); // *

    return 0;
}
