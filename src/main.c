#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "demo_tasks.h"
#include "track/line_task.h"
#include "motion/motion.h"
#include "navigation/mission_task.h"
#include "system/startup_task.h"

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL void startup_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    startup_run();
}

LOCAL void motion_task (INT stacd, void * exinf)
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
    motion_run();
}

LOCAL void mission_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    mission_run();
}

LOCAL T_CTSK ctsk_blink = {
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_startup = {
    .itskpri = 2,
    .stksz   = STACK_SZ,
    .task    = startup_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_motion = {
    .itskpri = 1,
    .stksz   = STACK_SZ,
    .task    = motion_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_mission = {
    .itskpri = 3,
    .stksz   = STACK_SZ,
    .task    = mission_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_line = {
    .itskpri = 2,
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
    ER motion_result = motion_init();
    if (motion_result != E_OK)
    {
        tm_printf((UB *)"[init] motion_init failed, er=%d\n", motion_result);
    }

    ER mission_result = mission_init();
    if (mission_result != E_OK)
    {
        tm_printf((UB *)"[init] mission_init failed, er=%d\n",
                  mission_result);
    }

    ID tid = tk_cre_tsk(&ctsk_blink);
    if (made("tsk(blink)", tid))
    {
        tk_sta_tsk(tid, 0);
    }
    tid = tk_cre_tsk(&ctsk_startup);
    if (made("tsk(startup)", tid))
    {
        tk_sta_tsk(tid, 0);
    }
    tid = tk_cre_tsk(&ctsk_motion);
    if (made("tsk(motion)", tid))
    {
        tk_sta_tsk(tid, 0);
    }
    tid = tk_cre_tsk(&ctsk_mission);
    if (made("tsk(mission)", tid))
    {
        tk_sta_tsk(tid, 0);
    }
    tid = tk_cre_tsk(&ctsk_line);
    if (made("tsk(line)", tid))
    {
        tk_sta_tsk(tid, 0);
    }

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
