#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "track/ir_capture.h"
#include "line_filter.h"
#include "navigation/mission_task.h"
#include "system/startup_task.h"

#define LINE_SAMPLE_MS 5U
#define TIMER_RAW_LOW  (TIMER_BASE + 0x28U)

EXPORT void line_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    while (!startup_ready())
    {
        if (startup_failed())
        {
            return;
        }
        (void)tk_dly_tsk(10);
    }

    line_filter_t filter;
    const ir_sensors_state_t initial = ir_get_state();
    line_filter_init(&filter, initial.left, initial.right);

    while (true)
    {
        const ir_sensors_state_t state = ir_get_state();
        line_filter_update(&filter, state.left, state.right);
        const mission_line_state_t line = {
            .left      = filter.left,
            .right     = filter.right,
            .sensed_us = in_w(TIMER_RAW_LOW),
        };

        (void)mission_post_line(&line);
        (void)tk_dly_tsk((RELTIM)LINE_SAMPLE_MS);
    }
}
