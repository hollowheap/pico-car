/** @file obstacle_extras_main.c
 * @brief Run the obstacle-extras controller on the shared Pico W platform.
 */

#include <tk/tkernel.h>

#include "ir/ir.h"
#include "ir/line/extras_line.h"
#include "motion/motion.h"
#include "motion/obstacle_extras/navigation.h"
#include "motion/obstacle_extras/setup.h"
#include "obstacle/extras/extras.h"
#include "platform/drv_gpio_irq.h"
#include "platform/log.h"
#include "platform/platform.h"

#define EXTRAS_BOOT_SETTLE_MS (3000)

LOCAL void extras_hardware_start (INT stacd, void * exinf)
{
    ER result = E_OK;

    (void)stacd;
    (void)exinf;
    (void)tk_slp_tsk(EXTRAS_BOOT_SETTLE_MS);
    result = drv_gpio_irq_init();
    if (E_OK == result)
    {
        result = extras_motion_setup(motion_task_id());
    }
    if (E_OK == result)
    {
        result = ir_capture_init();
    }
    if (E_OK == result)
    {
        ER const sonar_result = extras_nav_init();

        if (E_OK != sonar_result)
        {
            /* Keep the original line-only fallback when sonar setup fails. */
            LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE,
                      "obstacle-extras sonar setup failed, er=%d", (int)sonar_result);
            log_flush();
        }
        platform_startup_ready();
    }
    else
    {
        platform_startup_failed();
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "obstacle-extras setup failed, er=%d",
                  (int)result);
        log_flush();
    }
    tk_exd_tsk();
}

EXPORT INT usermain (void)
{
    motion_init();
    platform_init();
    obstacle_extras_init();
    extras_line_init();

    if (!(motion_start() && obstacle_extras_start() && extras_line_start() &&
          platform_start(extras_hardware_start)))
    {
        platform_startup_failed();
    }
    platform_park();
    return 0;
}
