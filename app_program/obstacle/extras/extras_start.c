#include <tk/tkernel.h>

#include "extras.h"
#include "scan_task.h"
#include "platform/platform.h"
#include "platform/stack.h"

static uint8_t g_extras_stack[EXTRAS_TASK_STACK]
    __attribute__((aligned(PLATFORM_STACK_ALIGN)));

LOCAL void extras_task (INT stacd, void * exinf)
{
    if (!platform_wait_startup() || !obstacle_extras_ready())
    {
        platform_park();
    }

    extras_scan_task_run(stacd, exinf);
}

LOCAL T_CTSK g_ctsk_extras_obstacle = {
    .itskpri = 4, /* NOLINT(readability-magic-numbers): task priority is literal */
    .stksz   = EXTRAS_TASK_STACK,
    .task    = extras_task,
    .bufptr  = g_extras_stack,
#if TK_SUPPORT_SMP
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc  = TP_PRC1,
#else
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void obstacle_extras_init (void)
{
    platform_stack_register(PLATFORM_STACK_OBSTACLE, g_extras_stack,
                            EXTRAS_TASK_STACK);
}

bool obstacle_extras_start (void)
{
    ID const tid = tk_cre_tsk(&g_ctsk_extras_obstacle);

    return platform_task_made("tsk(obstacle-extras)", tid) &&
           platform_task_started("tsk(obstacle-extras)", tid);
}
