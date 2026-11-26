/** @file parser.h
 * @brief Parses single-character Code 39 barcodes.
 * Only supports 'A' to 'Z' characters.
 * The initial narrow bar is bounded for the supported 50–400 mm markers.
 * Each character is classified once its nine elements are in: its two
 * widest bars and its widest space are wide. Bars and spaces are ranked
 * apart because the reader shrinks bars and widens spaces.
 */

#ifndef BARCODE_PARSER_H
#define BARCODE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#include "platform/static_assert.h"

#define BARCODE_NO_CHAR     0xFFU
#define BARCODE_PARSER_INIT ((barcode_parser_t){0})
#define BARCODE_CHARACTER_ELEMENTS (9U)

typedef struct
{
    /** Widths of the elements read so far in the current character. */
    uint16_t elem_width_um[BARCODE_CHARACTER_ELEMENTS];
    /** Mean narrow bar of the last character, or the first bar before it. */
    uint16_t narrow_width_um;
    /** Mean narrow space of the last character. */
    uint16_t narrow_space_um;
    /** Value of the middle data character. */
    uint8_t data_value;
    /** Bar/space index inside current character. */
    uint8_t elem_index : 4;
    /** Index of current character. */
    uint8_t char_index : 2;
    /**
     * Whether we are reading the barcode in reverse.
     * Only valid after the start character has been read.
     */
    bool is_reverse : 1;
} barcode_parser_t;

STATIC_ASSERT (sizeof(barcode_parser_t) == 24U, "barcode_parser_t size");

/** Why a barcode was rejected. */
#define BARCODE_REJECT_NONE      (0U)
#define BARCODE_REJECT_ELEMENT   (1U) /* element out of order, zero width, or a bad gap */
#define BARCODE_REJECT_PARITY    (2U) /* a character has no clear two wide bars and one wide space */
#define BARCODE_REJECT_DELIMITER (3U) /* the start or stop delimiter is wrong */
#define BARCODE_REJECT_SYMBOL    (6U) /* correctly framed symbol outside A-Z */

/** What the parser knew when it finished or rejected a barcode. */
typedef struct
{
    /** Narrow element width of the barcode, valid when a character returns. */
    uint16_t narrow_width_um;
    /** The barcode was read backward, valid when a character returns. */
    bool is_reverse;
    /** A barcode that had begun was rejected by this element. */
    uint8_t reject;
    /** The parser had read at least one element of a barcode before this one. */
    bool in_progress;
} barcode_info_t;

STATIC_ASSERT (sizeof(barcode_info_t) == 6U, "barcode_info_t size");

/** Feeds the new element into the parser.
 * If at the end of a barcode, returns a number 0-25, representing 'A'-'Z'.
 * Otherwise, returns BARCODE_NO_CHAR.
 */
uint8_t barcode_parser_input (barcode_parser_t * state, bool is_elem_space,
                              uint16_t elem_width_um);

/** Same as barcode_parser_input, and fills *p_info when it is not NULL.
 * reject is not BARCODE_REJECT_NONE only if a barcode that had begun
 * (in_progress) was rejected; a stray first edge is not reported.
 */
uint8_t barcode_parser_input_info (barcode_parser_t * state, bool is_elem_space,
                                   uint16_t elem_width_um, barcode_info_t * p_info);

/* Largest width accepted as an optical pulse. */
uint16_t barcode_parser_glitch_limit_um (const barcode_parser_t * state);

/** Whether an element is too short to be a real element of a barcode whose
 *  narrow width is held by `state`. */
bool barcode_parser_is_glitch (barcode_parser_t const * state, uint16_t elem_width_um);

#endif /* BARCODE_PARSER_H */

/*** end of file ***/
