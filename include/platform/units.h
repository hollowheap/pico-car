/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	units.h
 *	Unit factors and the integer clamp that the line follower and the pilot
 *	share, one copy each.
 */

#ifndef PLATFORM_UNITS_H
#define PLATFORM_UNITS_H

#include <stdint.h>

#define UM_PER_MM    (1000)
/* Deci-degrees per radian, 1800 / pi rounded. */
#define DECI_PER_RAD (573)

static inline int32_t clamp_i32 (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (result < low)
    {
        result = low;
    }
    if (result > high)
    {
        result = high;
    }

    return result;
}

#endif /* PLATFORM_UNITS_H */
