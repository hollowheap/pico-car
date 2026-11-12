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
 *
 * Guided checks (D162). The program prints PHYS_G lines that tell you what to do
 * and when. They answer the four checks "Why the hump state may always read
 * FLAT" of artifacts/notes/imu-lift-overturn.md. Save the whole console to a
 * file and run `python3 tools/physics_log_summary.py FILE --checks`.
 *
 *    0 to  4 s  keep the car still and level          check 1, calibration
 *    6 to 11 s  lift the NOSE to about 10 degrees     check 2, forward axis
 *   13 to 17 s  set it level and keep it still
 *   19 to 24 s  lift the LEFT side to about 10 degrees  check 2, side axis
 *   26 to 30 s  set it level and keep it still
 *   32 to 52 s  push the car by hand up and over a ramp  check 3, ramp angle
 *
 * Check 4 (driving against standing still) needs the wheels: see
 * docs/bench.md, "Hump check 4".
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

/* One guided phase: starts at start_s, tells the person what to do. */
typedef struct
{
    uint32_t start_s;
    char const * text;
} guided_phase_t;

static guided_phase_t const PHASES[] = {
    {0U, "KEEP STILL AND LEVEL (calibration)"},
    {6U, "LIFT THE NOSE to about 10 degrees and hold it"},
    {13U, "SET LEVEL and keep still"},
    {19U, "LIFT THE LEFT SIDE to about 10 degrees and hold it"},
    {26U, "SET LEVEL and keep still"},
    {32U, "PUSH THE CAR BY HAND up and over the ramp, slowly, a few times"},
    {52U, "END of the guided checks"},
};
#define PHASE_COUNT (sizeof (PHASES) / sizeof (PHASES[0]))
#define MS_PER_S    (1000U)

/* Sleep for STATS_PERIOD_MS and print each phase when its time comes. */
static void guided_wait (void)
{
    static uint32_t elapsed_s;
    uint32_t        step = 0U;

    for (step = 0U; step < (STATS_PERIOD_MS / MS_PER_S); step++)
    {
        uint32_t idx = 0U;

        for (idx = 0U; idx < PHASE_COUNT; idx++)
        {
            if (PHASES[idx].start_s == elapsed_s)
            {
                tm_printf ((UB *)"PHYS_G,%u,%u,%s\n", (unsigned int)elapsed_s,
                           (unsigned int)idx, PHASES[idx].text);
            }
        }
        (void)tk_dly_tsk (MS_PER_S);
        elapsed_s++;
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

    physics_car_config (&cfg);
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
        guided_wait ();
        physics_task_get_stats (&stats);
        tm_printf ((UB *)"PHYS_S,cycles=%u,overruns=%u,max_work_us=%u,"
                   "inits=%u,init=%d,i2c_tx=%u,retries=%u,nacks=%u,"
                   "timeouts=%u,fails=%u,word=%08x\n",
                   (unsigned int)stats.cycles, (unsigned int)stats.overruns,
                   (unsigned int)stats.max_work_us,
                   (unsigned int)stats.sensor_inits, (int)stats.last_init,
                   (unsigned int)stats.i2c.transactions,
                   (unsigned int)stats.i2c.retries,
                   (unsigned int)stats.i2c.nacks,
                   (unsigned int)stats.i2c.timeouts,
                   (unsigned int)stats.i2c.failures,
                   (unsigned int)stats.last_word);
    }
}

/*** end of file ***/
