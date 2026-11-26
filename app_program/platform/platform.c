/** @file platform.c
 * @brief The startup task and the startup barrier. See platform.h.
 */

#include <tk/tkernel.h>

#include "platform/platform.h"

#include "platform/faults.h"
#include "platform/log.h"
#include "platform/stack.h"

#define STACK_STARTUP_SZ (2048U)

/* Startup barrier event flag bits. */
#define STARTUP_FLAG_READY  (0x01U)
#define STARTUP_FLAG_FAILED (0x02U)

static ID g_startup_flg = 0;

static uint8_t g_startup_stack[STACK_STARTUP_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

void platform_init (void)
{
    T_CFLG cflg = {0};

    log_init();
    platform_stack_register(PLATFORM_STACK_STARTUP, g_startup_stack, STACK_STARTUP_SZ);

    cflg.flgatr  = TA_WMUL | TA_TFIFO;
    cflg.iflgptn = 0U;
    g_startup_flg = tk_cre_flg(&cflg);
}

bool platform_wait_startup (void)
{
    UINT flgptn = 0U;
    ER const er = tk_wai_flg(g_startup_flg,
                             STARTUP_FLAG_READY | STARTUP_FLAG_FAILED,
                             TWF_ORW, &flgptn, TMO_FEVR);

    if (E_OK != er)
    {
        return false;
    }

    return (0U != (flgptn & STARTUP_FLAG_READY));
}

void platform_park (void)
{
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

void platform_startup_failed (void)
{
    (void)tk_set_flg(g_startup_flg, STARTUP_FLAG_FAILED);
}

bool platform_task_made (const char * p_what, ID id)
{
    bool b_made = (id > E_OK);

    if (!b_made)
    {
        system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, FAULT_TASK_STALLED_BIT);
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "tk_cre_%s failed, er=%d", p_what,
                  (int)id);
    }
    return b_made;
}

bool platform_task_started (const char * p_what, ID tid)
{
    ER   result = tk_sta_tsk(tid, 0);
    bool b_started = (E_OK == result);

    if (!b_started)
    {
        system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, FAULT_TASK_STALLED_BIT);
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "tk_sta_%s failed, er=%d", p_what,
                  (int)result);
    }
    return b_started;
}

void platform_startup_ready (void)
{
    (void)tk_set_flg(g_startup_flg, STARTUP_FLAG_READY);
}

/* Task table. Core 0 (TP_PRC1) runs real-time control at priorities 1 to 5
 * and owns the GPIO bank IRQ; core 1 (TP_PRC2) runs the network at 6. A task
 * is pinned only for hardware ownership or a deadline. Telemetry and barcode
 * touch no hardware, so they stay unpinned at 7: they fill idle time on
 * either core, run on core 0 only when every control task waits, and reach
 * the pinned tasks through SMP-safe queues (app_program/comms/telemetry/queue.c).
 * The kernel tick is 10 ms (CNF_TIMER_PERIOD in
 * config/config.h): every timeout and delay can wait up
 * to one tick longer than asked. Each subsystem keeps its own task table
 * entry in its start function, with literal attributes. */

/* Core 0, 1: hardware and ISR setup on the IRQ core before any task runs. */
LOCAL T_CTSK g_ctsk_startup = {
    .itskpri = 1,
    .stksz   = STACK_STARTUP_SZ,
    .task    = NULL,
    .bufptr  = g_startup_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

bool platform_start (FP body)
{
    ID tid = 0;

    g_ctsk_startup.task = body;
    tid = tk_cre_tsk(&g_ctsk_startup);

    return platform_task_made("tsk(startup)", tid) &&
           platform_task_started("tsk(startup)", tid);
}

/*** end of file ***/
