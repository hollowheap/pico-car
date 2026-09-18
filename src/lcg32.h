/** @file lcg32.h
 *
 * @brief Linear congruent generator for pseudorandom number generation. 2^32
 * cycle length.
 */

#ifndef LCG32_H
#define LCG32_H

typedef struct
{
    uint32_t state;
} lcg32_t;

uint32_t lcg32_next (lcg32_t * p_lcg);
bool     lcg32_bool (lcg32_t * p_lcg);

#endif /* LCG32_H */

/*** end of file ***/
