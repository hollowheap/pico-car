/** @file lcg32.c
 *
 * @brief 32-bit linear congruent generator for pseudorandom number generation.
 * 2^32 cycle length.
 */

#include <stdbool.h>
#include <stdint.h>
#include "lcg32.h"

/*!
 * @brief Update the LCG and return the next 32-bit integer value.
 *
 * @param[in] lcg The LCG to update.
 *
 * @return The next pseudorandom number.
 */
uint32_t lcg32_next (lcg32_t * p_lcg)
{
    // NOTE: To get the longest possible cycle of 2^32:
    // 1. (a - 1) must be divisible by 4
    // 2. c must not be a multiple of 2
    //
    // Reference:
    // https://en.wikipedia.org/wiki/Linear_congruential_generator#m_a_power_of_2,_c_%E2%89%A0_0
    const uint32_t a = 5194973u;
    const uint32_t c = 113u;
    p_lcg->state     = (p_lcg->state * a) + c;

    return p_lcg->state;
}

/*!
 * @brief Update the LCG and return the next boolean value.
 *
 * @param[in] lcg The LCG to update.
 *
 * @return The next pseudorandom boolean.
 */
bool lcg32_bool (lcg32_t * p_lcg)
{
    // Note: modulus must not be a multiple of 2 for maximum randomness
    const uint32_t modulus = 1234567u;
    const uint32_t half    = modulus / 2u;
    const uint32_t value   = lcg32_next(p_lcg);
    return (value % modulus) < half;
}

/*** end of file ***/
