/** @file bridge.h
 * @brief Pure compare planning for the verified two-wheel motor polarity.
 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#define BRIDGE_PERIOD 12500U

typedef struct
{
    int16_t  duty[2]; /* Signed permille; positive is verified car-forward. */
    uint32_t compare[2];
    bool     reversal;
} bridge_plan_t;

/* Each channel pair has at most one nonzero compare. On this car both A-side
 * inputs drive forward. A nonzero sign change requires an intervening brake.
 * Invalid input leaves output unchanged. This function does not touch GPIO. */
bool bridge_plan (const int16_t previous[2], int16_t left, int16_t right,
                      bridge_plan_t * p_plan);

#endif /* BRIDGE_H */
