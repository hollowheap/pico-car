/** @file barcode_start.c
 * @brief Barcode init and the creation of its task. See barcode.h.
 */

#include <tk/tkernel.h>

#include "ir/barcode/barcode.h"

#include "platform/platform.h"
#include "platform/stack.h"

#define STACK_BARCODE_SZ (2304U)

static uint8_t g_barcode_stack[STACK_BARCODE_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

LOCAL void barcode_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    barcode_task_run();
}

/* Unpinned, 7: decodes the ISR edge ring; no hardware (see platform.c). */
LOCAL T_CTSK g_ctsk_barcode = {
    .itskpri = 7, /* NOLINT(readability-magic-numbers): the task table is kept literal by the project rules */
    .stksz   = STACK_BARCODE_SZ,
    .task    = barcode_task,
    .tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF,
    .bufptr  = g_barcode_stack,
};

void barcode_init (void)
{
    platform_stack_register(PLATFORM_STACK_BARCODE, g_barcode_stack, STACK_BARCODE_SZ);
}

bool barcode_start (void)
{
    ID const tid = tk_cre_tsk(&g_ctsk_barcode);
    return platform_task_made("tsk(barcode)", tid) &&
           platform_task_started("tsk(barcode)", tid);
}

/*** end of file ***/
