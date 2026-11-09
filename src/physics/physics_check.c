/** @file physics_check.c
 *
 * @brief Buddy 4 bring-up and CSV logger (stages 1 and 2).
 *
 * Standalone application with its own usermain(), like the other bring-up checks.
 * Holds the motor bridge inputs low (brake) and never drives the wheels.
 * Starts only the Hump task, with the motion service not running, so
 * speed data is invalid and only static tilt and hand-tilt tests make sense.
 *
 * Output over the USB console:
 *   PHYS_H lines: CSV header, LSM303 init result.
 *   B4  lines: one CSV row per cycle (50 Hz).
 *   PHYS_S lines: counters every 5 s.
 *
 * Keep the car still for the first second: calibration needs rest.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "physics_task.h"

#define STATS_PERIOD_MS (5000)
#define MOTOR_PIN_COUNT (4U)
#define MOTOR_PIN_A     (8U)      /* GP8 to GP11.                          */
#define MOTOR_PIN_B     (9U)
#define MOTOR_PIN_C     (10U)
#define MOTOR_PIN_D     (11U)
#define CHECK_TASK_PRI  (4)       /* Replaced by cfg.priority below.       */
#define CHECK_STACK_SZ  (4096U)

/* Robo Pico motor inputs GP8-GP11 (motion/motion_pins.h). The kernel's boot-time
 * I2C0 init routes GP8/GP9 to I2C with pull-ups, so force them low. */
static void hold_motors (void)
{
    static const UINT pins[MOTOR_PIN_COUNT] = { MOTOR_PIN_A, MOTOR_PIN_B,
                                                MOTOR_PIN_C, MOTOR_PIN_D };
    unsigned int i = 0U;

    for (i = 0U; i < MOTOR_PIN_COUNT; i++)
    {
        (void)gpio_set_val (pins[i], 0U);
        (void)gpio_set_pin (pins[i], GPIO_MODE_OUT);
    }
}

LOCAL void check_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    physics_task_run();
}

EXPORT INT usermain (void)
{
    physics_task_config_t cfg;
    physics_task_stats_t stats;
    T_CTSK ctsk = {
        .exinf   = NULL,
        .itskpri = CHECK_TASK_PRI,
        .stksz   = CHECK_STACK_SZ,
        .task    = check_task,
        .tskatr  = TA_HLNG | TA_RNG3 | TA_ASSPRC,
        .assprc  = TP_PRC1,
    };
    ID tid = 0;
    ER er = E_OK;

    hold_motors ();

    physics_task_default_config (&cfg);
    cfg.log_every = 1U;
    cfg.read_motion = NULL;
    ctsk.itskpri = cfg.priority;
    ctsk.assprc = cfg.processor;

    tid = tk_cre_tsk (&ctsk);
    if (tid <= E_OK)
    {
        tm_printf ((UB *)"PHYS_H,tk_cre_tsk=%d\n", (int)tid);
        (void)tk_slp_tsk (TMO_FEVR);
        return 1;
    }

    er = physics_task_init (&cfg, tid);
    tm_printf ((UB *)"PHYS_H,physics_task_init=%d\n", (int)er);
    if (E_OK != er)
    {
        (void)tk_slp_tsk (TMO_FEVR);
        return 1;
    }

    er = tk_sta_tsk (tid, 0);
    if (E_OK != er)
    {
        tm_printf ((UB *)"PHYS_H,tk_sta_tsk=%d\n", (int)er);
        (void)tk_slp_tsk (TMO_FEVR);
        return 1;
    }

    for (;;)
    {
        (void)tk_dly_tsk (STATS_PERIOD_MS);
        physics_task_get_stats (&stats);
        tm_printf ((UB *)"PHYS_S,cycles=%u,overruns=%u,max_work_us=%u,"
                   "inits=%u,init=%d,i2c_tx=%u,retries=%u,nacks=%u,"
                   "timeouts=%u,fails=%u,nav=%u/%u,telem=%u/%u,word=%08x\n",
                   (unsigned int)stats.cycles, (unsigned int)stats.overruns,
                   (unsigned int)stats.max_work_us,
                   (unsigned int)stats.sensor_inits, (int)stats.last_init,
                   (unsigned int)stats.i2c.transactions,
                   (unsigned int)stats.i2c.retries,
                   (unsigned int)stats.i2c.nacks,
                   (unsigned int)stats.i2c.timeouts,
                   (unsigned int)stats.i2c.failures,
                   (unsigned int)stats.nav_sent,
                   (unsigned int)stats.nav_dropped,
                   (unsigned int)stats.telem_sent,
                   (unsigned int)stats.telem_dropped,
                   (unsigned int)stats.last_word);
    }
}

/*** end of file ***/
