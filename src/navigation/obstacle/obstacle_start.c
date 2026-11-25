/** @file obstacle_start.c
 * @brief Obstacle init and the creation of its task. See obstacle.h.
 */

#include <tk/tkernel.h>

#include "navigation/obstacle.h"

#include "platform/platform.h"
#include "platform/stack.h"

#define STACK_OBSTACLE_SZ (2048U)

static uint8_t g_obstacle_stack[STACK_OBSTACLE_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

LOCAL void obstacle_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    obstacle_run();
}

/* Core 0, 4: 50 ms sonar pings and servo yield to the 5 ms line pass. */
LOCAL T_CTSK g_ctsk_obstacle = {
    .itskpri = 4, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
    .stksz   = STACK_OBSTACLE_SZ,
    .task    = obstacle_task,
    .bufptr  = g_obstacle_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void obstacle_init (void)
{
    platform_stack_register(PLATFORM_STACK_OBSTACLE, g_obstacle_stack, STACK_OBSTACLE_SZ);
}

bool obstacle_start (void)
{
    ID const tid = tk_cre_tsk(&g_ctsk_obstacle);

    return platform_task_made("tsk(obstacle)", tid) &&
           platform_task_started("tsk(obstacle)", tid);
}

/*** end of file ***/
