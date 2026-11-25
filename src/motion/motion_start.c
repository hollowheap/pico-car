/** @file motion_start.c
 * @brief Motion init and the creation of its task. See motor_setup.h.
 */

#include <tk/tkernel.h>

#include "motion/motor_setup.h"

#include "pwm.h"
#include "platform/platform.h"
#include "platform/stack.h"

#define STACK_MOTION_SZ (2560U)

static uint8_t g_motion_stack[STACK_MOTION_SZ] __attribute__((aligned(PLATFORM_STACK_ALIGN)));

static ID g_motion_tid = 0;

LOCAL void motion_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!platform_wait_startup())
    {
        platform_park();
    }

    motion_service_run();
}

/* Core 0, 1 (highest): the 10 ms wheel loop owns the PWM and encoders. */
LOCAL T_CTSK g_ctsk_motion = {
    .itskpri = 1,
    .stksz   = STACK_MOTION_SZ,
    .task    = motion_task,
    .bufptr  = g_motion_stack,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF,
#endif
};

void motion_init (void)
{
    /* The kernel boot table leaves GP8/GP9 (left motor) on I2C0 with
     * pull-ups. Hold every bridge input low before anything else runs; the
     * motion service repeats it when it takes the pins. */
    pwm_boot_brake();
    platform_stack_register(PLATFORM_STACK_MOTION, g_motion_stack, STACK_MOTION_SZ);
}

bool motion_start (void)
{
    g_motion_tid = tk_cre_tsk(&g_ctsk_motion);
    if (!platform_task_made("tsk(motion)", g_motion_tid))
    {
        g_motion_tid = 0;
        return false;
    }

    return platform_task_started("tsk(motion)", g_motion_tid);
}

ID motion_task_id (void)
{
    return g_motion_tid;
}

/*** end of file ***/
