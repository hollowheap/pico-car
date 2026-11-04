#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "track/track.h"
#include "line_filter.h"
#include "track/line_recovery.h"
#include "motion/motion.h"
#include "navigation/navigation.h"
#include "system/startup_task.h"

#define LINE_SAMPLE_MS  5U
#define TRACK_WIDTH_UM  130000
#define TIMER_RAW_LOW   (TIMER_BASE + 0x28U)

EXPORT void line_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    while (!startup_ready())
    {
        if (startup_failed())
        {
            (void)tm_printf((UB *)"line task: startup failed\n");
            for (;;)
            {
                (void)tk_slp_tsk(TMO_FEVR);
            }
        }
        (void)tk_dly_tsk(10);
    }

    line_filter_t   filter;
    line_recovery_t recovery;
    const ir_sensors_state_t initial = ir_get_state();
    line_filter_init(&filter, initial.left, initial.right);
    (void)line_recovery_init(&recovery, TRACK_WIDTH_UM);

    while (true)
    {
        const ir_sensors_state_t state = ir_get_state();
        line_filter_update(&filter, state.left, state.right);

        int32_t heading_mdeg = 0;
        motion_feedback_t feedback;
        if ((motion_feedback_snapshot(&feedback) == E_OK) && feedback.fresh)
        {
            if (filter.left || filter.right)
            {
                line_recovery_reset(&recovery, feedback.left_displacement_um,
                                    feedback.right_displacement_um);
            }
            else
            {
                line_recovery_update(&recovery, feedback.left_displacement_um,
                                     feedback.right_displacement_um);
            }
            heading_mdeg = line_recovery_heading_mdeg(&recovery);
        }

        const mission_line_state_t line = {
            .left         = filter.left,
            .right        = filter.right,
            .sensed_us    = in_w(TIMER_RAW_LOW),
            .heading_mdeg = heading_mdeg,
        };

        (void)mission_post_line(&line);
        (void)tk_dly_tsk((RELTIM)LINE_SAMPLE_MS);
    }
}
