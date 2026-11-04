/** @file buddy4_imu_check.c
 *
 * @brief Buddy 4 bring-up and CSV logger (stages 1 and 2).
 *
 * Standalone application with its own usermain(), like Buddy 2's checks.
 * Holds the motor bridge inputs low (brake) and never drives the wheels.
 * Starts only the Hump task, with Buddy 2's motion service not running, so
 * speed data is invalid and only static tilt and hand-tilt tests make sense.
 *
 * Output over the USB console:
 *   B4H lines: CSV header, LSM303 init result.
 *   B4  lines: one CSV row per cycle (50 Hz).
 *   B4S lines: counters every 5 s.
 *
 * Keep the car still for the first second: calibration needs rest.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "b4_task.h"

#define STATS_PERIOD_MS (5000)

/* Robo Pico motor inputs GP8-GP11 (buddy2_pins.h). The kernel's boot-time
 * I2C0 init routes GP8/GP9 to I2C with pull-ups, so force them low. */
static void hold_motors (void)
{
    static const UINT pins[4] = { 8U, 9U, 10U, 11U };
    unsigned int i;

    for (i = 0U; i < 4U; i++)
    {
        (void)gpio_set_val (pins[i], 0U);
        (void)gpio_set_pin (pins[i], GPIO_MODE_OUT);
    }
}

EXPORT INT usermain (void)
{
    b4_task_config_t cfg;
    b4_task_stats_t stats;
    ER er;

    hold_motors ();

    b4_task_default_config (&cfg);
    cfg.log_every = 1U;
    cfg.read_b2 = NULL;

    er = b4_task_start (&cfg);
    tm_printf ((UB *)"B4H,b4_task_start=%d\n", (int)er);
    if (E_OK != er)
    {
        (void)tk_slp_tsk (TMO_FEVR);
        return 1;
    }

    for (;;)
    {
        (void)tk_dly_tsk (STATS_PERIOD_MS);
        b4_task_get_stats (&stats);
        tm_printf ((UB *)"B4S,cycles=%u,overruns=%u,max_work_us=%u,"
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
