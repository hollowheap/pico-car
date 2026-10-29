/** @file startup_task.c
 * @brief Core 0 startup task; initialises shared services before producers run.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "track/ir_capture.h"
#include "platform/isr.h"
#include "system/startup_task.h"

static volatile bool ready;
static volatile bool failed;

bool startup_ready (void)
{
    return ready;
}

bool startup_failed (void)
{
    return failed;
}

void startup_run (void)
{
    ER result;

    result = isr_dispatcher_init();
    if (result == E_OK)
    {
        result = ir_capture_init();
    }

    if (result == E_OK)
    {
        ready = true;
    }
    else
    {
        failed = true;
        (void)tm_printf((UB *)"startup failed: %d\n", result);
    }

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/*** end of file ***/
