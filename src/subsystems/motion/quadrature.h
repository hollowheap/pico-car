/** @file quadrature.h
 * @brief Hardware-independent x4 quadrature capture state.
 */
#ifndef QUADRATURE_H
#define QUADRATURE_H

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
    /* Two bits per wheel: A is bit 1, B is bit 0. Upper four bits zero. */
    uint8_t states;
} quadrature_t;

/* No shared-state protection here: the capture driver supplies it. */
void quadrature_init (quadrature_t * p_quad, uint8_t states);

/* Update one wheel from a sampled two-bit A/B state. An ambiguous capture
 * (e.g. multiple edges pending) is recorded as invalid and resynchronises.
 * Invalid transitions never fabricate a direction or travelled distance.
 * Positive means 00->01->11->10->00, not necessarily car-forward.
 * Counters roll modulo 2^32. */
bool quadrature_update (quadrature_t * p_quad, unsigned int wheel,
                          uint8_t state, bool ambiguous);

#endif /* QUADRATURE_H */

/*** end of file ***/
