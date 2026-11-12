/** @file parser.h
 * @brief Parses single-character Code 39 barcodes.
 * Only supports 'A' to 'Z' characters.
 * Only supports barcodes that are 50-400mm in length.2
 */

#ifndef BARCODE_PARSER_H
#define BARCODE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#include "platform/static_assert.h"

#define BARCODE_NO_CHAR     0xFFU
#define BARCODE_PARSER_INIT ((barcode_parser_t){0})

typedef struct
{
    /** Width of a narrow element. */
    uint16_t narrow_width_um : 14;
    /** Value of the middle data character. */
    uint8_t data_value;
    /** Value of the character. */
    uint8_t curr_value;
    /** Bar/space index inside current character. */
    uint8_t elem_index : 4;
    /** Index of current character. */
    uint8_t char_index : 2;
    /** Number of wide bars observed in current character. */
    uint8_t wide_bar_count : 2;
    /** Number of wide spaces observed in current character. */
    uint8_t wide_space_count : 1;
    /**
     * Whether we are reading the barcode in reverse.
     * Only valid after the start charcter has been read.
     */
    bool is_reverse : 1;
} barcode_parser_t;

STATIC_ASSERT (sizeof(barcode_parser_t) == 6U, "barcode_parser_t size");

/** Why a barcode was rejected (D130). */
#define BARCODE_REJECT_NONE      (0U)
#define BARCODE_REJECT_ELEMENT   (1U) /* element out of order, or the width fits no class */
#define BARCODE_REJECT_PARITY    (2U) /* a character has the wrong number of wide elements */
#define BARCODE_REJECT_DELIMITER (3U) /* the start or stop delimiter is wrong */

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

#endif /* BARCODE_PARSER_H */

/*** end of file ***/
