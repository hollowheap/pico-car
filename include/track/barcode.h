#ifndef TRACK_BARCODE_H
#define TRACK_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define BARCODE_ELEMENTS 9U

typedef struct
{
    uint32_t last_us;
    uint32_t widths_um[BARCODE_ELEMENTS];
    uint8_t  count;
    bool     have_last;
    bool     initialised;
} barcode_decoder_t;

void barcode_decoder_init (barcode_decoder_t * p_decoder);
void barcode_decoder_reset (barcode_decoder_t * p_decoder);

/** Feed one edge; returns true when a character was decoded into p_char. */
bool barcode_decode_edge (barcode_decoder_t * p_decoder, uint32_t now_us,
                          int32_t speed_um_s, char * p_char);

#endif /* TRACK_BARCODE_H */
