#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "demo_tasks.h"
#include "ir/ir_capture.h"
#include "motion/motion_request.h"
#include "motion/motion_service.h"
#include "motion/encoder_capture.h"
#include "navigation/obstacle.h"
#include "physics/physics.h"
#include "sys/gpio_irq.h"

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096

/* Line following. 250 mm/s cruise and a 10 ms control period keep the car
 * within 2.5 mm of travel per pass. A one-sensor correction keeps the inner
 * wheel at LINE_TURN_PERMILLE of the cruise speed, so the car steers
 * proportionally instead of stopping a wheel at speed. Tune these on the
 * car; the duty cap in setup_motors() must leave headroom for the target. */
#define LINE_SPEED_UM_S    (250000U)
#define LINE_POLL_MS       (10U)
#define LINE_TURN_PERMILLE (450U)

/* Startup barrier. This file creates every task; one startup task owns the
 * shared hardware init and the others wait for it. Subsystems never call
 * tk_cre_tsk() or tk_sta_tsk(). */
typedef struct
{
    bool volatile b_ready;
    bool volatile b_failed;
    ID            physics_tid;
} startup_state_t;

static startup_state_t g_startup = {false, false, 0};

ER setup_motors (void)
{
    motion_service_config_t config   = {0};
    config.control.sampling.wheel[0] = (motion_scale_t){204204U, 2530U, -1};
    config.control.sampling.wheel[1] = (motion_scale_t){204204U, 2530U, 1};
    config.control.sampling.max_interval_us = 100000U;
    for (uint8_t wheel = 0U; wheel < 2U; wheel++)
    {
        config.control.speed[wheel] = (wheel_pid_config_t){
            .kp_q16            = 256U,
            .ki_q16            = 256U,
            .kd_q16            = 2U,
            .max_target_um_s   = 300000U,
            .max_interval_us   = 100000U,
            .max_sample_age_us = 600000U,
            .max_duty_permille = 400U,
        };
    }
    config.control.max_command_ttl_us  = 200000U;
    config.control.max_control_gap_us  = 100000U;
    config.control.feedback_timeout_us = 2000000U;
    config.control.reversal_brake_us   = 100000U;
    config.poll_delay_ms               = 10U;

    return motion_service_init(&config);
}

/*!
 * @brief Wait until the startup task has finished. Returns false on a
 *        startup failure; the caller must then park.
 */
LOCAL bool wait_startup (void)
{
    while (!g_startup.b_ready)
    {
        if (g_startup.b_failed)
        {
            return false;
        }
        (void)tk_dly_tsk(10);
    }

    return true;
}

/*!
 * @brief Park a task whose service could not start.
 */
LOCAL void park (void)
{
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

LOCAL void startup_task (INT stacd, void * exinf)
{
    ER result;

    (void)stacd;
    (void)exinf;

    result = gpio_irq_init();
    if (E_OK == result)
    {
        result = setup_motors();
    }
    if (E_OK == result)
    {
        result = physics_init(g_startup.physics_tid);
    }
    if (E_OK == result)
    {
        result = ir_capture_init();
    }
    if (E_OK == result)
    {
        /* Buddy 5 obstacle avoidance. If it fails, line following still
         * runs, but this build stops instead of driving blind. */
        result = obstacle_line_init();
    }

    if (E_OK == result)
    {
        g_startup.b_ready = true;
    }
    else
    {
        tm_printf((UB *)"[init] startup failed: %d\n", result);
        g_startup.b_failed = true;
    }

    park();
}

LOCAL void motion_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    motion_service_run();
}

LOCAL void physics_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    physics_run();
}

LOCAL void obstacle_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    obstacle_run();
}

LOCAL void line_task (INT stacd, void * exinf)
{
    const uint32_t speed      = LINE_SPEED_UM_S;
    const uint32_t turn_speed = (LINE_SPEED_UM_S * LINE_TURN_PERMILLE) / 1000U;
    /* Each speed lease must outlast one loop pass (up to LINE_POLL_MS plus
     * processing); an expired active lease latches MOTION_FAULT_EXPIRED. */
    const uint32_t lease_us = 3U * LINE_POLL_MS * 1000U;

    (void)stacd;
    (void)exinf;

    if (!wait_startup())
    {
        park();
    }

    while (true)
    {
        const ir_sensors_state_t state = ir_get_state();

        if (obstacle_line_step(state.left, state.right))
        {
            /* Buddy 5 is holding the car or driving round an obstacle. */
        }
        else if (state.left && state.right)
        {
            // TODO: run command at junction
            motion_service_set_speed(speed, speed, lease_us);
        }
        else if (state.left)
        {
            motion_service_set_speed(turn_speed, speed, lease_us);
        }
        else if (state.right)
        {
            motion_service_set_speed(speed, turn_speed, lease_us);
        }
        else
        {
            motion_service_set_speed(speed, speed, lease_us);
        }
        ir_wait_change(IR_TYPE_LINE, LINE_POLL_MS);
    }
}

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL T_CTSK ctsk_blink = {
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_startup = {
    .itskpri = 1,
    .stksz   = STACK_SZ,
    .task    = startup_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_motion = {
    .itskpri = 5,
    .stksz   = STACK_SZ,
    .task    = motion_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_physics = {
    .itskpri = 4,
    .stksz   = STACK_SZ,
    .task    = physics_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_obstacle = {
    .itskpri = 6,
    .stksz   = STACK_SZ,
    .task    = obstacle_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_line = {
    .itskpri = 9,
    .stksz   = STACK_SZ,
    .task    = line_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or print failure and keep going. */
LOCAL BOOL made (const char * what, ID id)
{
    if (id <= E_OK)
    {
        tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
        return FALSE;
    }
    return TRUE;
}

EXPORT INT usermain (void)
{
    bool b_ok = true;
    ID   startup_tid;
    ID   motion_tid;
    ID   physics_tid;
    ID   obstacle_tid;
    ID   line_tid;
    ID   blink_tid;

    /* D62/D70: this composition root creates every task. Subsystems only
     * prepare their resources and expose a task body. */
    startup_tid = tk_cre_tsk(&ctsk_startup);
    if (!made("tsk(startup)", startup_tid))
    {
        b_ok = false;
    }

    motion_tid = tk_cre_tsk(&ctsk_motion);
    if (!made("tsk(motion)", motion_tid))
    {
        b_ok = false;
    }

    physics_tid = tk_cre_tsk(&ctsk_physics);
    if (!made("tsk(physics)", physics_tid))
    {
        b_ok = false;
    }
    else
    {
        g_startup.physics_tid = physics_tid;
    }

    obstacle_tid = tk_cre_tsk(&ctsk_obstacle);
    if (!made("tsk(obstacle)", obstacle_tid))
    {
        b_ok = false;
    }

    line_tid = tk_cre_tsk(&ctsk_line);
    if (!made("tsk(line)", line_tid))
    {
        b_ok = false;
    }

    blink_tid = tk_cre_tsk(&ctsk_blink);
    if (!made("tsk(blink)", blink_tid))
    {
        b_ok = false;
    }

    if (!b_ok)
    {
        tm_printf((UB *)"[init] task creation failed, system stays stopped\n");
        park();
    }

    tk_sta_tsk(startup_tid, 0);
    tk_sta_tsk(motion_tid, 0);
    tk_sta_tsk(physics_tid, 0);
    tk_sta_tsk(obstacle_tid, 0);
    tk_sta_tsk(line_tid, 0);
    tk_sta_tsk(blink_tid, 0);

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
