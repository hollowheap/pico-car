/** @file obstacle_extras_bench_start.c
 * @brief Start the scanner and its simulated navigation task.
 */

#include <tk/tkernel.h>

#include "obstacle_extras_bench.h"
#include "obstacle/extras/extras.h"
#include "platform/log.h"
#include "platform/platform.h"
#include "platform/stack.h"

#define SIM_STACK (4096U)

static uint8_t g_sim_stack[SIM_STACK]
    __attribute__((aligned(PLATFORM_STACK_ALIGN)));

LOCAL T_CTSK g_ctsk_sim = {
    .itskpri = 8, /* NOLINT(readability-magic-numbers): task priority is literal */
    .stksz   = SIM_STACK,
    .task    = extras_bench_run,
    .bufptr  = g_sim_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void extras_bench_start (void)
{
    ID tid = 0;

    platform_init();
    obstacle_extras_init();
    platform_stack_register(PLATFORM_STACK_LINE, g_sim_stack, SIM_STACK);

    if (obstacle_extras_start())
    {
        tid = tk_cre_tsk(&g_ctsk_sim);
        if (platform_task_made("tsk(extras-sim)", tid) &&
            platform_task_started("tsk(extras-sim)", tid))
        {
            return;
        }
    }

    platform_startup_failed();
    log_flush();
}
