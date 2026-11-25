/** @file track_start.c
 * @brief Track init and the creation of the line task. See track.h.
 */

#include <tk/tkernel.h>

#include "ir/line/track.h"

#include "platform/platform.h"
#include "platform/stack.h"
#include "ir/line/line_task.h"

#define STACK_LINE_SZ (2560U)

static uint8_t g_line_stack[STACK_LINE_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

LOCAL void line_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    line_task_run();
}

/* Core 0, 3: the 5 ms line pass steers, below motion and the supervisor. */
LOCAL T_CTSK g_ctsk_line = {
    .itskpri = 3, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
    .stksz   = STACK_LINE_SZ,
    .task    = line_task,
    .bufptr  = g_line_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void track_init (void)
{
    platform_stack_register(PLATFORM_STACK_LINE, g_line_stack, STACK_LINE_SZ);
}

bool track_start (void)
{
    ID const tid = tk_cre_tsk(&g_ctsk_line);

    return platform_task_made("tsk(line)", tid) &&
           platform_task_started("tsk(line)", tid);
}

/*** end of file ***/
