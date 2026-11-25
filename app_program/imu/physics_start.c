/** @file physics_start.c
 * @brief Physics init and the creation of its task. See physics.h.
 */

#include <tk/tkernel.h>

#include "imu/physics.h"

#include "platform/platform.h"
#include "platform/stack.h"

#define STACK_PHYSICS_SZ (2816U)

static uint8_t g_physics_stack[STACK_PHYSICS_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

static ID g_physics_tid = 0;

LOCAL void physics_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    physics_run();
}

/* Core 0, 5 (lowest control): its I2C IMU waits never delay steering. */
LOCAL T_CTSK g_ctsk_physics = {
    .itskpri = 5, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
    .stksz   = STACK_PHYSICS_SZ,
    .task    = physics_task,
    .bufptr  = g_physics_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void physics_init (void)
{
    platform_stack_register(PLATFORM_STACK_PHYSICS, g_physics_stack, STACK_PHYSICS_SZ);
}

bool physics_start (void)
{
    g_physics_tid = tk_cre_tsk(&g_ctsk_physics);
    if (!platform_task_made("tsk(physics)", g_physics_tid))
    {
        g_physics_tid = 0;
        return false;
    }

    return platform_task_started("tsk(physics)", g_physics_tid);
}

ID physics_task_id (void)
{
    return g_physics_tid;
}

/*** end of file ***/
