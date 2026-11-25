/** @file quadrature.h
 * @brief Hardware-independent decoder for the quadrature state machine words.
 */
#ifndef ENCODER_QUAD_H
#define ENCODER_QUAD_H

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_LEFT   0U
#define ENCODER_RIGHT  1U
#define ENCODER_WHEELS 2U

typedef struct
{
    uint32_t positive;
    uint32_t negative;
    uint32_t invalid;
} quadrature_counts_t;

typedef struct
{
    quadrature_counts_t wheel[ENCODER_WHEELS];
    /* The last word from each wheel's state machine. */
    uint32_t last[ENCODER_WHEELS];
} quadrature_t;

/* No shared-state protection here: the capture driver supplies it. Zero the
 * counts and take words as the baseline. */
void quadrature_init (quadrature_t * p_quad, const uint32_t words[ENCODER_WHEELS]);

/* Add the steps since the last word of one wheel. A word holds the net step
 * count in bits 23..0 and a count of skipped states, counting down, in bits
 * 31..24 (see quadrature.pio). The net change goes to positive or negative;
 * steps that cancel between two words are not counted. Positive means
 * A,B = 00->01->11->10->00, not necessarily car-forward. A skip never adds
 * distance. Calls must come often enough that the net count moves less than
 * 2^23 steps and the skip count less than 256 between them. Counts roll
 * modulo 2^32. */
bool quadrature_apply (quadrature_t * p_quad, unsigned int wheel, uint32_t word);

#endif /* ENCODER_QUAD_H */

/*** end of file ***/
