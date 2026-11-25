/*
 *	test_barcode_action.c
 *	Host tests for the barcode actions (D130): the letter map, the 100 mm
 *	re-arm gate, the 250 ms suppression window, and the information the
 *	parser reports (narrow width, read direction, reject reason).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "barcode/action.h"
#include "barcode/parser.h"
#include "supervisor/command.h"

#define NARROW_UM 4255U
#define WIDE_UM   12766U

static unsigned g_fail;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            (void)printf ("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
            g_fail++;                                                        \
        }                                                                    \
    } while (0)

/* Feed one 9-element character. Returns the last parser result; *p_info is the
 * info of the last element. */
static uint8_t feed_char (barcode_parser_t * p_parser, uint8_t wide_space, uint8_t wide_bar1,
                          uint8_t wide_bar2, barcode_info_t * p_info)
{
    uint8_t result = BARCODE_NO_CHAR;
    uint8_t idx    = 0U;

    for (idx = 0U; idx < 9U; idx++)
    {
        bool const     is_space = ((idx % 2U) == 1U);
        uint16_t const width =
            (is_space && (idx == wide_space))                          ? WIDE_UM
            : (!is_space && ((idx == wide_bar1) || (idx == wide_bar2))) ? WIDE_UM
                                                                        : NARROW_UM;

        result = barcode_parser_input_info (p_parser, is_space, width, p_info);
        if (BARCODE_NO_CHAR != result)
        {
            break;
        }
    }

    return result;
}

static void feed_gap (barcode_parser_t * p_parser, barcode_info_t * p_info)
{
    (void) barcode_parser_input_info (p_parser, true, NARROW_UM, p_info);
}

static void test_letter_map (void)
{
    uint8_t code = 0U;

    CHECK (BARCODE_ACTION_STRAIGHT == barcode_action_of ('S' - 'A').action);
    CHECK (VEHICLE_CMD_STRAIGHT == barcode_action_of ('S' - 'A').opcode);
    CHECK (BARCODE_ACTION_STOP == barcode_action_of ('H' - 'A').action);
    CHECK (VEHICLE_CMD_STOP == barcode_action_of ('H' - 'A').opcode);
    CHECK (BARCODE_ACTION_LEFT == barcode_action_of ('L' - 'A').action);
    CHECK (VEHICLE_CMD_LEFT == barcode_action_of ('L' - 'A').opcode);
    CHECK (BARCODE_ACTION_RIGHT == barcode_action_of ('R' - 'A').action);
    CHECK (VEHICLE_CMD_RIGHT == barcode_action_of ('R' - 'A').opcode);
    CHECK (BARCODE_ACTION_UTURN == barcode_action_of ('U' - 'A').action);
    CHECK (VEHICLE_CMD_UTURN == barcode_action_of ('U' - 'A').opcode);

    /* Every other letter is rejected. */
    for (code = 0U; code < 26U; code++)
    {
        char const letter = (char) ('A' + (char) code);

        if ((letter != 'S') && (letter != 'H') && (letter != 'L') && (letter != 'R') && (letter != 'U'))
        {
            CHECK (BARCODE_ACTION_REJECTED == barcode_action_of (code).action);
        }
    }
    CHECK (BARCODE_ACTION_REJECTED == barcode_action_of (200U).action);
}

static void test_rearm_gate (void)
{
    barcode_gate_t gate;

    barcode_gate_init (&gate);
    CHECK (barcode_gate_pass (&gate, 500U)); /* the first one always acts */
    CHECK (!barcode_gate_pass (&gate, 500U)); /* the same marker read twice */
    CHECK (!barcode_gate_pass (&gate, 599U));
    CHECK (barcode_gate_pass (&gate, 600U)); /* 100 mm after the last one that acted */
    CHECK (!barcode_gate_pass (&gate, 650U));

    /* A pivot moves the mean travel back: the car must go forward again. */
    CHECK (!barcode_gate_pass (&gate, 580U));
    CHECK (!barcode_gate_pass (&gate, 690U));
    CHECK (barcode_gate_pass (&gate, 700U));
}

static void test_suppression_window (void)
{
    CHECK (!barcode_suppress_active (false, 1000000U, 0U));
    CHECK (barcode_suppress_active (true, 1000000U, 1000000U));
    CHECK (barcode_suppress_active (true, 1000000U + BARCODE_SUPPRESS_US - 1U, 1000000U));
    CHECK (!barcode_suppress_active (true, 1000000U + BARCODE_SUPPRESS_US, 1000000U));
    /* The timer wraps. */
    CHECK (barcode_suppress_active (true, 100000U, 0xFFFF0000U));
    CHECK (!barcode_suppress_active (true, 400000U, 0xFFFF0000U));
}

static void test_info_forward_and_reverse (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    barcode_info_t   info;

    /* Forward 'U': start *, U, stop *. */
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 1, 4, 6, &info));
    feed_gap (&parser, &info);
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 1, 0, 8, &info));
    feed_gap (&parser, &info);
    CHECK (('U' - 'A') == feed_char (&parser, 1, 4, 6, &info));
    CHECK (NARROW_UM == info.narrow_width_um);
    CHECK (!info.is_reverse);
    CHECK (BARCODE_REJECT_NONE == info.reject);

    /* Reversed 'A'. */
    parser = BARCODE_PARSER_INIT;
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 7, 2, 4, &info));
    feed_gap (&parser, &info);
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 3, 8, 0, &info));
    feed_gap (&parser, &info);
    CHECK (0U == feed_char (&parser, 7, 2, 4, &info));
    CHECK (info.is_reverse);
}

static void test_reject_reasons (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    barcode_info_t   info;

    /* A stray first edge is not a barcode: nothing to report. */
    (void) barcode_parser_input_info (&parser, true, WIDE_UM, &info);
    CHECK (BARCODE_REJECT_NONE == info.reject);
    CHECK (!info.in_progress);

    /* Bad parity inside a barcode that began: reported. */
    parser = BARCODE_PARSER_INIT;
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 1, 4, 6, &info));
    feed_gap (&parser, &info);
    /* One wide bar where two are needed: the character ends with the wrong parity,
     * which the parser sees at the gap after it. */
    {
        bool const is_wide[9] = { true, true, false, false, false, false, false, false, false };
        uint8_t    idx        = 0U;

        for (idx = 0U; idx < 9U; idx++)
        {
            (void) barcode_parser_input_info (&parser, (idx % 2U) == 1U,
                                              is_wide[idx] ? WIDE_UM : NARROW_UM, &info);
            CHECK (BARCODE_REJECT_NONE == info.reject);
        }
        feed_gap (&parser, &info);
    }
    CHECK (BARCODE_REJECT_PARITY == info.reject);
    CHECK (info.in_progress);

    /* The stop delimiter is a letter: reported as a delimiter error. */
    parser = BARCODE_PARSER_INIT;
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 1, 4, 6, &info));
    feed_gap (&parser, &info);
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 1, 0, 8, &info));
    feed_gap (&parser, &info);
    CHECK (BARCODE_NO_CHAR == feed_char (&parser, 1, 0, 8, &info));
    CHECK (BARCODE_REJECT_DELIMITER == info.reject);
}

static void test_rejects_excess_wide_elements_as_parity (void)
{
    static const struct
    {
        uint16_t widths[5];
        uint8_t count;
        uint8_t reason;
    } cases[] = {
        {{WIDE_UM, NARROW_UM, WIDE_UM, NARROW_UM, WIDE_UM}, 5U, BARCODE_REJECT_PARITY},
        {{NARROW_UM, WIDE_UM, NARROW_UM, WIDE_UM}, 4U, BARCODE_REJECT_PARITY},
        {{NARROW_UM, 2U * NARROW_UM}, 2U, BARCODE_REJECT_ELEMENT},
    };

    for (uint8_t reverse = 0U; reverse < 2U; reverse++)
    {
        uint8_t const space = (reverse != 0U) ? 7U : 1U;
        uint8_t const bar1  = (reverse != 0U) ? 2U : 4U;
        uint8_t const bar2  = (reverse != 0U) ? 4U : 6U;

        for (size_t c = 0U; c < sizeof(cases) / sizeof(cases[0]); c++)
        {
            barcode_parser_t parser = BARCODE_PARSER_INIT;
            barcode_info_t   info;

            CHECK (BARCODE_NO_CHAR == feed_char (&parser, space, bar1, bar2, &info));
            feed_gap (&parser, &info);
            for (uint8_t i = 0U; i < cases[c].count; i++)
            {
                uint8_t const reason = (i + 1U == cases[c].count) ? cases[c].reason : BARCODE_REJECT_NONE;
                CHECK (BARCODE_NO_CHAR == barcode_parser_input_info(
                    &parser, (i % 2U) != 0U, cases[c].widths[i], &info));
                CHECK (info.reject == reason);
                CHECK (info.in_progress && (info.is_reverse == (reverse != 0U)));
            }
            CHECK (parser.elem_index == 0U && parser.char_index == 0U);

            CHECK (BARCODE_NO_CHAR == feed_char (&parser, space, bar1, bar2, &info));
            feed_gap (&parser, &info);
            CHECK (BARCODE_NO_CHAR == feed_char (&parser, (reverse != 0U) ? 3U : 5U, 0U, 8U, &info));
            feed_gap (&parser, &info);
            CHECK (0U == feed_char (&parser, space, bar1, bar2, &info));
            CHECK (info.reject == BARCODE_REJECT_NONE && (info.is_reverse == (reverse != 0U)));
        }
    }
}

int main (int argc, char * argv[])
{
    if ((2 == argc) && (0 == strcmp (argv[1], "letter-map")))
    {
        test_letter_map ();
    }
    else if (1 == argc)
    {
        test_letter_map ();
        test_rearm_gate ();
        test_suppression_window ();
        test_info_forward_and_reverse ();
        test_reject_reasons ();
        test_rejects_excess_wide_elements_as_parity ();
    }
    else
    {
        (void)fprintf (stderr, "Usage: %s [letter-map]\n", argv[0]);
        return 2;
    }

    if (g_fail != 0U)
    {
        (void)printf ("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf ("test_barcode_action: all checks passed\n");
    return 0;
}
