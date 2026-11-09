/** @file motor.h
 * @brief Pure compare planning for the verified two-wheel motor polarity.
 */
#ifndef MOTOR_PAIR_H
#define MOTOR_PAIR_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PAIR_PERIOD 12500U

/* Wheels in the pair, and the full-scale duty in permille. */
#define MOTOR_PAIR_WHEELS       2U
#define MOTOR_DUTY_MAX_PERMILLE 1000U

/* Bit offset of the channel B compare field in a PWM CC register. */
#define MOTOR_COMPARE_B_SHIFT 16U

typedef struct
{
    /* Signed permille; positive is verified car-forward. */
    int16_t  duty[MOTOR_PAIR_WHEELS];
    uint32_t compare[MOTOR_PAIR_WHEELS];
    bool     reversal;
} motor_plan_t;

/* Each channel pair has at most one nonzero compare. On this car both A-side
 * inputs drive forward. A nonzero sign change requires an intervening brake.
 * Invalid input leaves output unchanged. This function does not touch GPIO. */
bool motor_plan (const int16_t previous[MOTOR_PAIR_WHEELS], int16_t left,
                 int16_t right, motor_plan_t * p_plan);

#endif /* MOTOR_PAIR_H */
