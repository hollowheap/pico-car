/** @file parser.h
 * @brief Parses single-character Code 39 barcodes.
 * Only supports 'A' to 'Z' characters.
 * Only supports barcodes that are 50-400mm in length.2
 */

#ifndef BARCODE_PARSER_H
#define BARCODE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define BARCODE_NO_CHAR     0xFFU
#define BARCODE_PARSER_INIT ((barcode_parser_t){0})

typedef struct
{
    /** Width of a narrow element. */
    uint16_t narrow_width_um : 14;
    /** Value of the middle data character. */
    uint8_t data_value : 6;
    /** Value of the character. */
    uint8_t curr_value : 6;
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

/** Feeds the new element into the parser.
 * If at the end of a barcode, returns a number 0-25, representing 'A'-'Z'.
 * Otherwise, returns BARCODE_NO_CHAR.
 */
uint8_t barcode_parser_input (barcode_parser_t * state, bool is_elem_space,
                              uint16_t elem_width_um);

#endif /* BARCODE_PARSER_H */

/*** end of file ***/
