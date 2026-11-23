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
#include "platform/chassis.h"

#define UM_PER_MM    (1000)
/* Deci-degrees per radian, 1800 / pi rounded. */
#define DECI_PER_RAD (573)

/* Signed right-minus-left wheel difference to unwrapped axle yaw.
 * Travel in um gives deci-degrees. Speed in um/s gives deci-degrees/s. */
static inline int64_t wheel_yaw_deci (int64_t diff_um)
{
    return (diff_um * DECI_PER_RAD) / (CHASSIS_WHEEL_TRACK_MM * UM_PER_MM);
}

/* Signed midpoint wheel travel in whole millimetres, saturated for telemetry. */
static inline uint32_t wheel_odometer_mm (int64_t left_um, int64_t right_um)
{
    if ((right_um > 0) && (left_um > (INT64_MAX - right_um)))
    {
        return UINT32_MAX;
    }
    if ((right_um < 0) && (left_um < (INT64_MIN - right_um)))
    {
        return 0U;
    }
    int64_t const mm = (left_um + right_um) / (2 * UM_PER_MM);

    return (mm < 0) ? 0U : (mm > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)mm;
}

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
