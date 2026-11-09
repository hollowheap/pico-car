/** @file motor.c
 * @brief Map signed wheel duties to mutually exclusive bridge inputs.
 */
#include <stddef.h>
#include "motor.h"

/* Signed duty bound for the range checks. */
#define DUTY_LIMIT ((int32_t)MOTOR_DUTY_MAX_PERMILLE)

bool motor_plan (const int16_t previous[MOTOR_PAIR_WHEELS], int16_t left,
                 int16_t right, motor_plan_t * p_plan)
{
    motor_plan_t plan = {{left, right}, {0U, 0U}, false};
    unsigned int      wheel = 0U;

    if ((previous == NULL) || (p_plan == NULL))
    {
        return false;
    }
    for (wheel = 0U; wheel < MOTOR_PAIR_WHEELS; wheel++)
    {
        int32_t  duty = plan.duty[wheel];
        uint32_t compare = 0U;

        if ((duty < -DUTY_LIMIT) || (duty > DUTY_LIMIT) ||
            (previous[wheel] < -DUTY_LIMIT) || (previous[wheel] > DUTY_LIMIT))
        {
            return false;
        }
        compare = (MOTOR_PAIR_PERIOD * (uint32_t)(duty < 0 ? -duty : duty)) /
                  MOTOR_DUTY_MAX_PERMILLE;
        plan.compare[wheel] =
            duty < 0 ? compare << MOTOR_COMPARE_B_SHIFT : compare;
        if (((previous[wheel] > 0) && (duty < 0)) ||
            ((previous[wheel] < 0) && (duty > 0)))
        {
            plan.reversal = true;
        }
    }
    *p_plan = plan;
    return true;
}

/*** end of file ***/
