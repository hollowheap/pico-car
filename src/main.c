#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "track/track.h"
#include "motion/motion.h"
#include "navigation/navigation.h"
#include "system/startup_task.h"

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 *
 *  D62/D70: this composition root is the only place that creates or
 *  starts tasks. Subsystems never call tk_cre_tsk/tk_sta_tsk and never
 *  own a competing supervisor.
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
            (void)tm_printf((UB *)"motion task: startup failed\n");
            (void)motion_stop();
            for (;;)
            {
                (void)tk_slp_tsk(TMO_FEVR);
            }
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

LOCAL void emergency_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        UINT pattern = 0U;
        (void)tk_wai_flg(flg_ctrl_id, FLG_EMG_STOP,
                         TWF_ORW | TWF_CLR, &pattern, TMO_FEVR);
        motion_emergency_service();
    }
}

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

LOCAL T_CTSK ctsk_emergency = {
    .itskpri = 1,
    .stksz   = STACK_SZ,
    .task    = emergency_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_mission = {
    .itskpri = 2,
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
    .itskpri = 3,
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

LOCAL void started (const char * what, ID id)
{
    const ER result = tk_sta_tsk(id, 0);
    if (result != E_OK)
    {
        tm_printf((UB *)"[init] tk_sta_%s failed, er=%d\n", what, result);
    }
}

EXPORT INT usermain (void)
{
    const ER motion_result = motion_init();
    const BOOL motion_ready  = (motion_result == E_OK);
    if (!motion_ready)
    {
        tm_printf((UB *)"[init] motion_init failed, er=%d\n", motion_result);
    }

    const ER mission_result = mission_init();
    const BOOL mission_ready = (mission_result == E_OK);
    if (!mission_ready)
    {
        tm_printf((UB *)"[init] mission_init failed, er=%d\n",
                  mission_result);
    }

    ID tid = tk_cre_tsk(&ctsk_startup);
    if (made("tsk(startup)", tid))
    {
        started("tsk(startup)", tid);
    }
    if (motion_ready)
    {
        tid = tk_cre_tsk(&ctsk_emergency);
        if (made("tsk(emergency)", tid))
        {
            started("tsk(emergency)", tid);
        }
        tid = tk_cre_tsk(&ctsk_motion);
        if (made("tsk(motion)", tid))
        {
            started("tsk(motion)", tid);
        }
    }
    if (mission_ready)
    {
        tid = tk_cre_tsk(&ctsk_mission);
        if (made("tsk(mission)", tid))
        {
            started("tsk(mission)", tid);
        }
    }
    tid = tk_cre_tsk(&ctsk_line);
    if (made("tsk(line)", tid))
    {
        started("tsk(line)", tid);
    }

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
