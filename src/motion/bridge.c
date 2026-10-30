/** @file bridge.c
 * @brief Map signed wheel duties to mutually exclusive bridge inputs.
 */
#include <stddef.h>
#include "bridge.h"

bool bridge_plan (const int16_t previous[2], int16_t left, int16_t right,
                      bridge_plan_t * p_plan)
{
    bridge_plan_t plan = {{left, right}, {0U, 0U}, false};
    unsigned int      wheel;

    if ((previous == NULL) || (p_plan == NULL))
    {
        return false;
    }
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        int32_t  duty = plan.duty[wheel];
        uint32_t compare;

        if ((duty < -1000) || (duty > 1000) || (previous[wheel] < -1000) ||
            (previous[wheel] > 1000))
        {
            return false;
        }
        compare =
            (BRIDGE_PERIOD * (uint32_t)(duty < 0 ? -duty : duty)) / 1000U;
        plan.compare[wheel] = duty < 0 ? compare << 16U : compare;
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
