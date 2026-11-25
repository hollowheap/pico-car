#include <tk/tkernel.h>

#include "extras_line.h"
#include "track.h"
#include "motion/motion.h"
#include "motion/obstacle_extras/navigation.h"
#include "platform/platform.h"
#include "platform/stack.h"

#define EXTRAS_LINE_STACK_SIZE (4096U)
#define EXTRAS_LINE_SPEED_UM_S (163364)
#define EXTRAS_LINE_PERIOD_MS  (20)
#define EXTRAS_LINE_LEASE_US   (60000U)

static uint8_t g_extras_line_stack[EXTRAS_LINE_STACK_SIZE]
    __attribute__((aligned(PLATFORM_STACK_ALIGN)));

LOCAL void extras_line_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    if (!platform_wait_startup())
    {
        platform_park();
    }

    for (;;)
    {
        ir_sensors_state_t const state = ir_get_state();

        if (!extras_nav_step(state.left, state.right))
        {
            /* The original controller stops the inside wheel on a single
               hit and drives both wheels at the same speed otherwise. */
            int32_t const left = (state.left && !state.right) ?
                                    0 : EXTRAS_LINE_SPEED_UM_S;
            int32_t const right = (state.right && !state.left) ?
                                     0 : EXTRAS_LINE_SPEED_UM_S;
            (void)motion_set_speed(left, right, EXTRAS_LINE_LEASE_US);
        }
        (void)ir_wait_change(IR_TYPE_LINE, EXTRAS_LINE_PERIOD_MS);
    }
}

LOCAL T_CTSK g_ctsk_extras_line = {
    .itskpri = 3, /* NOLINT(readability-magic-numbers): task priority is literal */
    .stksz   = EXTRAS_LINE_STACK_SIZE,
    .task    = extras_line_task,
    .bufptr  = g_extras_line_stack,
#if TK_SUPPORT_SMP
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc  = TP_PRC1,
#else
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void extras_line_init (void)
{
    platform_stack_register(PLATFORM_STACK_LINE, g_extras_line_stack,
                            EXTRAS_LINE_STACK_SIZE);
}

bool extras_line_start (void)
{
    ID const tid = tk_cre_tsk(&g_ctsk_extras_line);

    return platform_task_made("tsk(extras-line)", tid) &&
           platform_task_started("tsk(extras-line)", tid);
}
