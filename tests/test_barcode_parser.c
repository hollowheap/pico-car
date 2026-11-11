/** @file test_barcode_parser.c
 * @brief Unit tests for Code 39 barcode parser: valid decodes,
 * parity validation, boundary conditions, deadband rejection,
 * and unclosed delimiter rejection (S10, T65).
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "barcode/parser.h"

#define NARROW_UM 4255U
#define WIDE_UM   12766U

static uint8_t char_to_data (char c)
{
    return (uint8_t)(c - 'A');
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

static uint8_t put_custom_char (barcode_parser_t * parser,
                                const bool is_wide[9])
{
    uint8_t result = BARCODE_NO_CHAR;
    for (int i = 0; i < 9; i++)
    {
        const bool     is_space = (i % 2) == 1;
        const uint16_t width    = is_wide[i] ? WIDE_UM : NARROW_UM;
        const uint8_t  code     = barcode_parser_input(parser, is_space, width);
        if (BARCODE_NO_CHAR != code)
        {
            result = code;
        }
        if ((0 == parser->char_index) && (0 == parser->elem_index))
        {
            /* Parser rejected an element and reset */
            break;
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

static void test_forward_and_reverse_valid (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;

    /* Forwards 'A' */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* A */
    put_space(&parser);
    assert(char_to_data('A') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'B' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 2, 8)); /* B */
    put_space(&parser);
    assert(char_to_data('B') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'J' (edge of 1-11 range with DATA_ONES_SKIP) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 4, 6)); /* J */
    put_space(&parser);
    assert(char_to_data('J') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'K' (first of K-T group) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 0, 8)); /* K */
    put_space(&parser);
    assert(char_to_data('K') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'L' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 8)); /* L */
    put_space(&parser);
    assert(char_to_data('L') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'T' (last of K-T group) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 4, 6)); /* T */
    put_space(&parser);
    assert(char_to_data('T') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'U' (first of U-Z group) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 0, 8)); /* U */
    put_space(&parser);
    assert(char_to_data('U') == put_char(&parser, 1, 4, 6)); /* * */

    /* Forwards 'Z' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 2, 4)); /* Z */
    put_space(&parser);
    assert(char_to_data('Z') == put_char(&parser, 1, 4, 6)); /* * */

    /* Reversed 'A' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 3, 8, 0)); /* A */
    put_space(&parser);
    assert(char_to_data('A') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'B' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 3, 0, 6)); /* B */
    put_space(&parser);
    assert(char_to_data('B') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'J' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 3, 2, 4)); /* J */
    put_space(&parser);
    assert(char_to_data('J') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'K' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 0, 8)); /* K */
    put_space(&parser);
    assert(char_to_data('K') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'L' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 0, 6)); /* L */
    put_space(&parser);
    assert(char_to_data('L') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'T' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 2, 4)); /* T */
    put_space(&parser);
    assert(char_to_data('T') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'U' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 0, 8)); /* U */
    put_space(&parser);
    assert(char_to_data('U') == put_char(&parser, 7, 2, 4)); /* * */

    /* Reversed 'Z' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 4, 6)); /* Z */
    put_space(&parser);
    assert(char_to_data('Z') == put_char(&parser, 7, 2, 4)); /* * */
}

static void test_invalid_interchar_elements (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;

    /* Inter-character element is wide space instead of narrow space */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, WIDE_UM));
    assert_parser_reset(&parser);

    /* Inter-character element is bar instead of space */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, NARROW_UM));
    assert_parser_reset(&parser);

    /* Element too narrow or too wide */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, 1));
    assert_parser_reset(&parser);
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 65000));
    assert_parser_reset(&parser);
}

static void test_start_boundaries_and_deadbands (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;

    /* START_MIN_UM = 638U: exactly 638 is accepted */
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 638U));
    assert(1 == parser.elem_index);

    /* Below START_MIN_UM (637U) resets parser */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 637U));
    assert_parser_reset(&parser);

    /* START_MAX_UM = 12765U: exactly 12765 is accepted */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 12765U));
    assert(1 == parser.elem_index);

    /* Above START_MAX_UM (12766U) resets parser */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 12766U));
    assert_parser_reset(&parser);

    /* First element space rejection */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, NARROW_UM));
    assert_parser_reset(&parser);

    /* Deadband width rejection: narrow width baseline = 4000 um */
    /* 0.5x = 2000 um (< 0.6x narrow scale) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 4000U));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, 2000U));
    assert_parser_reset(&parser);

    /* 2.0x = 8000 um (between 1.5x narrow and 2.4x wide scale) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 4000U));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, 8000U));
    assert_parser_reset(&parser);

    /* 4.0x = 16000 um (> 3.6x wide scale) */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, 4000U));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, 16000U));
    assert_parser_reset(&parser);
}

static void test_invalid_parity (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;

    /* Patterns: [bar0, sp0, bar1, sp1, bar2, sp2, bar3, sp3, bar4] */

    /* 1. Parity error in start delimiter: 3 wide bars (bars 1, 2, 3 wide) */
    const bool start_3_bars[9] = {
        false, true, true, false, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, start_3_bars));
    assert_parser_reset(&parser);

    /* 2. Parity error in start delimiter: 1 wide bar */
    const bool start_1_bar[9] = {
        false, true, false, false, true, false, false, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, start_1_bar));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, NARROW_UM));
    assert_parser_reset(&parser);

    /* 3. Parity error in start delimiter: 2 wide spaces */
    const bool start_2_spaces[9] = {
        false, true, false, true, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, start_2_spaces));
    assert_parser_reset(&parser);

    /* 4. Parity error in start delimiter: 0 wide spaces */
    const bool start_0_spaces[9] = {
        false, false, false, false, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, start_0_spaces));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, NARROW_UM));
    assert_parser_reset(&parser);

    /* 5. Parity error in data character: 3 wide bars */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    const bool data_3_bars[9] = {
        true, false, true, false, false, true, false, false, true
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, data_3_bars));
    assert_parser_reset(&parser);

    /* 6. Parity error in data character: 1 wide bar */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    const bool data_1_bar[9] = {
        true, false, false, false, false, true, false, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, data_1_bar));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, NARROW_UM));
    assert_parser_reset(&parser);

    /* 7. Parity error in data character: 2 wide spaces */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    const bool data_2_spaces[9] = {
        true, true, false, false, false, true, false, false, true
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, data_2_spaces));
    assert_parser_reset(&parser);

    /* 8. Parity error in data character: 0 wide spaces */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    const bool data_0_spaces[9] = {
        true, false, false, false, false, false, false, false, true
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, data_0_spaces));
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, NARROW_UM));
    assert_parser_reset(&parser);

    /* 9. Parity error in stop delimiter: 3 wide bars */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* valid A */
    put_space(&parser);
    const bool stop_3_bars[9] = {
        false, true, true, false, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, stop_3_bars));
    assert_parser_reset(&parser);

    /* 10. Parity error in stop delimiter: 1 wide bar */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* valid A */
    put_space(&parser);
    const bool stop_1_bar[9] = {
        false, true, false, false, true, false, false, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, stop_1_bar));
    assert_parser_reset(&parser);

    /* 11. Parity error in stop delimiter: 2 wide spaces */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* valid A */
    put_space(&parser);
    const bool stop_2_spaces[9] = {
        false, true, false, true, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, stop_2_spaces));
    assert_parser_reset(&parser);

    /* 12. Parity error in stop delimiter: 0 wide spaces */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* valid * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* valid A */
    put_space(&parser);
    const bool stop_0_spaces[9] = {
        false, false, false, false, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, stop_0_spaces));
    assert_parser_reset(&parser);
}

static void test_unclosed_delimiters (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;

    /* 1. Start character is a valid character but not delimiter '*'
     * (bars 1, 4; space 0) */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 2, 8));
    /* Inter-character space triggers character end check:
     * char 0 must be DELIM_CHARACTER */
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, true, NARROW_UM));
    assert_parser_reset(&parser);

    /* 2. Stop character is a data character ('L') instead of delimiter '*' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* A */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 8)); /* L instead of * */
    assert_parser_reset(&parser);

    /* 3. Stop character is reversed '*' when start was forward '*' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* forward * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* forward A */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* reversed * */
    assert_parser_reset(&parser);

    /* 4. Stop character is forward '*' when start was reversed '*' */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 7, 2, 4)); /* reversed * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 3, 8, 0)); /* reversed A */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* forward * */
    assert_parser_reset(&parser);

    /* 5. Barcode truncated before stop delimiter is complete */
    parser = BARCODE_PARSER_INIT;
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6)); /* forward * */
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8)); /* forward A */
    put_space(&parser);
    /* Feed only 5 elements of stop delimiter */
    for (int i = 0; i < 5; i++)
    {
        const bool is_space = (i % 2) == 1;
        assert(BARCODE_NO_CHAR ==
               barcode_parser_input(&parser, is_space, NARROW_UM));
    }
    assert(2 == parser.char_index);
    /* Feeding an unexpected element level (bar instead of space) resets */
    assert(BARCODE_NO_CHAR == barcode_parser_input(&parser, false, NARROW_UM));
    assert_parser_reset(&parser);
}

static void test_continuous_streaming (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;

    /* Feed *A* */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6));
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 0, 8));
    put_space(&parser);
    assert(char_to_data('A') == put_char(&parser, 1, 4, 6));
    assert_parser_reset(&parser);

    /* Immediately feed *B* without manually resetting parser */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6));
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 2, 8));
    put_space(&parser);
    assert(char_to_data('B') == put_char(&parser, 1, 4, 6));
    assert_parser_reset(&parser);

    /* Corrupted sequence rejects, then next clean barcode decodes */
    const bool corrupt[9] = {
        false, true, true, false, true, false, true, false, false
    };
    assert(BARCODE_NO_CHAR == put_custom_char(&parser, corrupt));
    assert_parser_reset(&parser);

    /* Stream clean *J* after corruption */
    assert(BARCODE_NO_CHAR == put_char(&parser, 1, 4, 6));
    put_space(&parser);
    assert(BARCODE_NO_CHAR == put_char(&parser, 5, 4, 6));
    put_space(&parser);
    assert(char_to_data('J') == put_char(&parser, 1, 4, 6));
    assert_parser_reset(&parser);
}

int main (void)
{
    test_forward_and_reverse_valid();
    test_invalid_interchar_elements();
    test_start_boundaries_and_deadbands();
    test_invalid_parity();
    test_unclosed_delimiters();
    test_continuous_streaming();

    return 0;
}

/*** end of file ***/
