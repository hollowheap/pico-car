/** @file app_main.c
 * @brief Run the obstacle-extras controller on the shared Pico W platform.
 */

#include <tk/tkernel.h>

#include "app.h"
#include "motion/obstacle_extras/navigation.h"
#include "obstacle/extras/extras.h"
#include "platform/drv_gpio_irq.h"
#include "platform/drv_start_button.h"
#include "platform/log.h"
#include "platform/platform.h"
#include "platform/static_assert.h"
#include "supervisor/health_ids.h"

/* Nine application tasks plus the kernel init, console and radio tasks. */
#define FIRMWARE_TASKS   (12U)
#define FIRMWARE_CYCLICS (3U)
STATIC_ASSERT(FIRMWARE_TASKS <= CNF_MAX_TSKID, "task limit");
STATIC_ASSERT(FIRMWARE_CYCLICS <= CNF_MAX_CYCID, "cyclic handler limit");

LOCAL void command_dispatch (const telemetry_cmd_t * p_cmd, void * p_context)
{
    (void)p_context;
    if ((uint8_t)TELEMETRY_CMD_ESTOP == p_cmd->opcode)
    {
        (void)motion_request_stop();
    }
    (void)vehicle_command_post_ids(p_cmd->opcode, VEHICLE_SRC_TELEMETRY,
                                   p_cmd->cmd_id, p_cmd->run_id);
}

LOCAL ER configure_supervisor (uint16_t optional_mask)
{
    supervisor_config_t config = {0};
    ER const button_result = drv_start_button_init();

    config.required_mask = (uint16_t)((1U << LIVE_MOTION) | (1U << LIVE_LINE) |
                                     (1U << LIVE_PHYSICS));
    config.required_mask |= optional_mask;
    config.p_line_edges = ir_line_edges;
    config.p_telemetry_seen = connection_mqtt_seen;
    if (E_OK == button_result)
    {
        config.p_start_pressed = drv_start_button_pressed;
    }
    else
    {
        LOG_WARN(LOG_TASK_BOOT, LOG_MOD_NONE, "GP20 START input setup failed, er=%d", (int)button_result);
    }

    return supervisor_setup(&config);
}

/* Optional tasks start in usermain() and wait at the startup barrier. Their
 * liveness joins the supervisor's required set only if they started. */
static uint16_t g_optional_mask;
static bool     g_b_obstacle_started;

LOCAL void start_optional_tasks (void)
{
    telemetry_start();
    g_optional_mask = telemetry_live_mask();
    if (barcode_start())
    {
        g_optional_mask |= (uint16_t)(1U << LIVE_BARCODE);
    }
    g_b_obstacle_started = obstacle_extras_start();
}

/* The sonar needs the GPIO IRQ service, so it is set up after it. */
LOCAL uint16_t optional_live_mask (void)
{
    uint16_t mask = g_optional_mask;
    if (g_b_obstacle_started)
    {
        ER const sonar = extras_nav_init();
        if (E_OK == sonar)
        {
            mask |= (uint16_t)(1U << LIVE_OBSTACLE);
        }
        else
        {
            LOG_WARN(LOG_TASK_BOOT, LOG_MOD_NONE, "sonar unavailable, er=%d", (int)sonar);
        }
    }
    return mask;
}

LOCAL void startup_task (INT stacd, void * exinf)
{
    ER result = drv_gpio_irq_init();

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
        result = configure_supervisor(optional_live_mask());
    }
    if (E_OK == result)
    {
        platform_startup_ready();
    }
    else
    {
        platform_startup_failed();
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "obstacle-extras setup failed, er=%d",
                  (int)result);
        log_flush();
    }
    (void)tk_exd_tsk();
}

EXPORT INT usermain (void)
{
    motion_init();
    platform_init();
    telemetry_init(command_dispatch);
    physics_init();
    obstacle_extras_init();
    track_init();
    barcode_init();
    supervisor_init();

    bool const b_required = motion_start() && physics_start() && track_start() && supervisor_start();
    if (b_required)
    {
        start_optional_tasks();
    }
    if (!(b_required && platform_start(startup_task)))
    {
        platform_startup_failed();
        LOG_ERROR(LOG_TASK_BOOT, LOG_MOD_NONE, "task creation or start failed");
        log_flush();
    }
    platform_park();
    return 0;
}
