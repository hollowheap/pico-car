/** @file test_bridge.c
 * @brief Exhaust signed duty range and verify bridge/reversal invariants.
 */
#include <assert.h>
#include <stddef.h>
#include <string.h>
#include "bridge.h"

int main (void)
{
    const int16_t previous[2] = {200, -200};
    const int16_t invalid_previous[2] = {1001, 0};
    bridge_plan_t plan;
    bridge_plan_t saved;
    int duty;

    for (duty = -1000; duty <= 1000; duty++)
    {
        uint32_t magnitude = (uint32_t)(duty < 0 ? -duty : duty);
        uint32_t compare = BRIDGE_PERIOD * magnitude / 1000U;

        assert(bridge_plan(previous, (int16_t)duty, (int16_t)-duty, &plan));
        assert(plan.duty[0] == duty && plan.duty[1] == -duty);
        assert(plan.compare[0] == (duty < 0 ? compare << 16U : compare));
        assert(plan.compare[1] == (duty > 0 ? compare << 16U : compare));
        assert((plan.compare[0] & UINT32_C(0xffff)) == 0U ||
               (plan.compare[0] >> 16U) == 0U);
        assert(plan.reversal == (duty < 0));
    }
    assert(bridge_plan(previous, 0, 0, &plan) && !plan.reversal);
    assert(plan.compare[0] == 0U && plan.compare[1] == 0U);
    saved = plan;
    assert(!bridge_plan(previous, 1001, 0, &plan));
    assert(!bridge_plan(previous, 0, -1001, &plan));
    assert(!bridge_plan(invalid_previous, 0, 0, &plan));
    assert(!bridge_plan(NULL, 0, 0, &plan));
    assert(!bridge_plan(previous, 0, 0, NULL));
    assert(memcmp(&plan, &saved, sizeof(plan)) == 0);
    return 0;
}
