#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "motion/motion.h"
#include "motion/motor_setup.h"
#include "navigation/obstacle.h"
#include "physics/physics.h"
#include "platform/isr.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/platform.h"
#include "platform/static_assert.h"
#include "supervisor/supervisor.h"
#include "telemetry/cmd.h"
#include "telemetry/connection.h"
#include "telemetry/telemetry.h"
#include "track/barcode.h"
#include "track/track.h"

/* Kernel objects the firmware creates, against the limits in
   the pinned kernel config/config.h. Tasks: the nine started by the
   subsystems, plus the kernel's init, USB console and CYW43 tasks. Cyclic
   handlers: supervisor, motion and physics. Past a limit, tk_cre_* fails
   at boot with E_LIMIT. */
#define FIRMWARE_TASKS   (12U)
#define FIRMWARE_CYCLICS (3U)
STATIC_ASSERT(FIRMWARE_TASKS <= CNF_MAX_TSKID, "task limit");
STATIC_ASSERT(FIRMWARE_CYCLICS <= CNF_MAX_CYCID, "cyclic handler limit");

/* GP20 is the active-low operator START input in the main firmware. */
#define START_BUTTON_PIN             (20U)
#define START_BUTTON_PULL_UP         (1UL << 3U)
#define START_BUTTON_PULL_DOWN       (1UL << 2U)
#define START_BUTTON_SCHMITT         (1UL << 1U)

LOCAL bool start_button_pressed (void)
{
    return (0U == gpio_get_val(START_BUTTON_PIN));
}

/*!
 * @brief Give the supervisor the tasks that must show a pulse. A task that did
 *        not start is left out.
 */
LOCAL bool configure_supervisor (uint16_t optional_mask)
{
    supervisor_config_t config = {0};
    ER const button_result = gpio_set_pin(START_BUTTON_PIN, GPIO_MODE_IN);

    config.required_mask = (uint16_t)((1U << LIVE_MOTION) | (1U << LIVE_LINE) |
                                      (1U << LIVE_OBSTACLE) | (1U << LIVE_PHYSICS));
    config.required_mask |= optional_mask;
    config.p_line_edges = ir_line_edges;
    config.p_telemetry_seen = connection_mqtt_seen;
    if (E_OK == button_result)
    {
        clr_w(GPIO(START_BUTTON_PIN), START_BUTTON_PULL_DOWN);
        set_w(GPIO(START_BUTTON_PIN), START_BUTTON_PULL_UP | START_BUTTON_SCHMITT);
        config.p_start_pressed = start_button_pressed;
    }
    else
    {
        LOG_WARN(LOG_TASK_BOOT, LOG_MOD_NONE, "GP20 START input setup failed, er=%d", (int)button_result);
    }
    if (E_OK != supervisor_setup(&config))
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "supervisor_setup failed");
        return false;
    }

    return true;
}

/*!
 * @brief Start every subsystem except the startup task. Returns false when a
 *        required task failed. The optional tasks that exist are added to
 *        the supervisor mask.
 */
LOCAL bool start_tasks (uint16_t * p_optional_mask)
{
    bool b_ok = motion_start();
    b_ok = b_ok && physics_start();
    b_ok = b_ok && obstacle_start();
    b_ok = b_ok && track_start();
    b_ok = b_ok && supervisor_start();
    if (!b_ok)
    {
        return false;
    }
    telemetry_start();
    *p_optional_mask = telemetry_live_mask();
    if (barcode_start())
    {
        *p_optional_mask |= (uint16_t)(1U << LIVE_BARCODE);
    }

    return b_ok;
}

/*!
 * @brief Hand a telemetry command to the supervisor.
 *
 * The supervisor decides. The emergency stop also acts here, at once: it
 * never waits for the next supervisor pass. The latch needs a reboot.
 */
LOCAL void command_dispatch (const telemetry_cmd_t * cmd, void * context)
{
    (void)context;

    if ((uint8_t)TELEMETRY_CMD_ESTOP == cmd->opcode)
    {
        (void)motion_request_stop();
    }
    (void)vehicle_command_post_ids(cmd->opcode, VEHICLE_SRC_TELEMETRY,
                                   cmd->cmd_id, cmd->run_id);
}

LOCAL void startup_task (INT stacd, void * exinf)
{
    ER result = gpio_irq_init();

    (void)stacd;
    (void)exinf;

    if (E_OK == result)
    {
        result = motion_setup(motion_task_id());
    }
    if (E_OK == result)
    {
        result = physics_setup(physics_task_id());
    }
    if (E_OK == result)
    {
        result = ir_capture_init();
    }
    if (E_OK == result)
    {
        /* Obstacle avoidance. If it fails, line following still
         * runs, but this build stops instead of driving blind. */
        result = obstacle_line_init();
    }

    if (E_OK == result)
    {
        platform_startup_ready();
    }
    else
    {
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "startup failed, er=%d",
                  (int)result);
        /* The telemetry task parks on a failed startup, so print now. */
        log_flush();
        platform_startup_failed();
    }

    (void)tk_exd_tsk();
}

EXPORT INT usermain (void)
{
    uint16_t optional_mask = 0U;
    bool     b_ok          = false;

    motion_init();
    platform_init();
    telemetry_init(command_dispatch);
    physics_init();
    obstacle_init();
    track_init();
    barcode_init();
    supervisor_init();

    /* The startup task goes last: it reads the IDs of the tasks started
     * before it, and releases them from the startup barrier. */
    b_ok = start_tasks(&optional_mask) && configure_supervisor(optional_mask) &&
           platform_start(startup_task);
    if (!b_ok)
    {
        /* Tasks that already started wait on the startup barrier. */
        platform_startup_failed();
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE,
                  "task creation or start failed, system stays stopped");
        log_flush();
        platform_park();
    }

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
