#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "demo_tasks.h"
#include "ir/ir_task.h"

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL T_CTSK ctsk_blink = {
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_line = {
    .itskpri = 9,
    .stksz   = STACK_SZ,
    .task    = line_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or print failure and stop. */
LOCAL BOOL made (const char * what, ID id)
{
    if (id <= E_OK)
    {
        tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
        return FALSE;
    }
    return TRUE;
}

EXPORT INT usermain (void)
{

    ID tid = tk_cre_tsk(&ctsk_blink);
    if (made("tsk(blink)", tid))
    {
        tk_sta_tsk(tid, 0);
    }
    tid = tk_cre_tsk(&ctsk_line);
    if (made("tsk(line)", tid))
    {
        tk_dly_tsk(3000);
        tk_sta_tsk(tid, 0);
    }

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
