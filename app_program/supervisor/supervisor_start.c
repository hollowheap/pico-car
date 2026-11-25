/** @file supervisor_start.c
 * @brief Supervisor init and the creation of its task and cyclic handler.
 *        See supervisor.h.
 */

#include <tk/tkernel.h>

#include "supervisor/supervisor.h"

#include "platform/platform.h"
#include "platform/stack.h"

#define STACK_SUPERVISOR_SZ (2560U)

static uint8_t g_supervisor_stack[STACK_SUPERVISOR_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

static ID g_supervisor_tid = 0;

LOCAL void supervisor_cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_supervisor_tid);
}

LOCAL void supervisor_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    supervisor_run();
}

/* Core 0, 2: the 20 ms stop and fault pass preempts line, sonar and IMU. */
LOCAL T_CTSK g_ctsk_supervisor = {
    .itskpri = 2, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
    .stksz   = STACK_SUPERVISOR_SZ,
    .task    = supervisor_task,
    .bufptr  = g_supervisor_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void supervisor_init (void)
{
    platform_stack_register(PLATFORM_STACK_SUPERVISOR, g_supervisor_stack, STACK_SUPERVISOR_SZ);
}

bool supervisor_start (void)
{
    T_CCYC ccyc = {0};
    ID const tid = tk_cre_tsk(&g_ctsk_supervisor);

    if (!platform_task_made("tsk(supervisor)", tid))
    {
        return false;
    }

    g_supervisor_tid = tid;
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)supervisor_cyclic_wakeup;
    ccyc.cyctim = (RELTIM)SUPERVISOR_PASS_MS;
    ccyc.cycphs = (RELTIM)SUPERVISOR_PASS_MS;
    if (!platform_task_made("cyc(supervisor)", tk_cre_cyc(&ccyc)))
    {
        return false;
    }

    return platform_task_started("tsk(supervisor)", tid);
}

/*** end of file ***/
