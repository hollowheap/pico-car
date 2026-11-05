/** @file buddy4_drive_check.c
 *
 * @brief Buddy 4 powered S6 run: drive straight, log hump detection.
 *
 * Standalone application with its own usermain(). Never link it with
 * src/main.c. Starts Buddy 2's motion task (same order as src/main.c) and
 * the Buddy 4 Hump task with real wheel feedback and require_motion on.
 *
 * Use:
 *   1. Power on with the car still on the floor; wait for B4H,ready.
 *   2. Press GP20 to start. The car drives straight at DRIVE_SPEED_UM_S
 *      until GP21 is pressed (motion's hardware STOP, latched until a
 *      power cycle or reset).
 *   3. If motion latches a fault the wheels stop until reboot (Buddy 2's
 *      rule); monitor_task prints B4H,motion_fault with the reason.
 *
 * There is no steering: someone must guide or catch the car.
 *
 * Output over the USB console:
 *   B4H lines: header, init results, run start/stop.
 *   B4  lines: one CSV row per cycle (50 Hz), from b4_task.c.
 *   B4D lines: drive status every 1 s while driving (core 1).
 *   B4S lines: counters every 5 s, plus motion state and fault (core 1).
 * Nothing prints on core 0 while the wheels turn: see monitor_task.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "motion/motion.h"
#include "platform/isr.h"

#include "b4_b2_glue.h"
#include "b4_i2c_rp2040.h"
#include "b4_task.h"

#define STATS_PERIOD_MS  (5000)
#define STACK_SZ         (4096)

/* Requested 400 mm/s, but motion caps every target at max_target_um_s =
 * 100000 um/s in src/motion/config.c, and guidance rejects anything above
 * it (E_PAR and a brake). Raise both together if Buddy 2 agrees. */
#define DRIVE_SPEED_UM_S (100000)
#define DRIVE_PERIOD_MS  (10U)          /* Resubmit well inside the lease. */
#define DRIVE_LEASE_US   (200000U)      /* = max_command_ttl_us (200 ms).  */
#define DRIVE_REPORT_MS  (1000U)        /* B4D status line period.         */
#define CAL_SETTLE_MS    (3000U)        /* IMU calibration needs rest.     */
#define START_PIN        (20U)          /* Robo Pico button, active low.   */
#define STOP_PIN         (21U)          /* Motion's STOP button, active low. */
#define STOP_LOW_READS   (3U)           /* Debounce: 3 x 10 ms low.        */

/* Written by the drive task, printed by the core-1 monitor task. */
static volatile bool g_driving;
static volatile uint32_t g_drive_ms;
static volatile int g_last_submit;


static volatile bool g_isr_ready;
static volatile bool g_isr_failed;

/* Core 0: the GPIO dispatcher must exist before motion_run(). */
LOCAL void startup_task (INT stacd, void * exinf)
{
    ER er;

    (void)stacd;
    (void)exinf;
    er = isr_dispatcher_init ();
    if (E_OK == er)
    {
        g_isr_ready = true;
    }
    else
    {
        g_isr_failed = true;
        tm_printf ((UB *)"B4H,isr_dispatcher_init=%d\n", (int)er);
    }
    for (;;)
    {
        (void)tk_slp_tsk (TMO_FEVR);
    }
}

LOCAL void motion_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    while (!g_isr_ready)
    {
        if (g_isr_failed)
        {
            (void)motion_stop ();
            for (;;)
            {
                (void)tk_slp_tsk (TMO_FEVR);
            }
        }
        (void)tk_dly_tsk (10);
    }
    motion_run ();
}

LOCAL void emergency_task (INT stacd, void * exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        UINT pattern = 0U;

        (void)tk_wai_flg (flg_ctrl_id, FLG_EMG_STOP, TWF_ORW | TWF_CLR,
                          &pattern, TMO_FEVR);
        motion_emergency_service ();
    }
}

static const char * fault_name (motion_fault_t fault)
{
    const char * p_name;

    switch (fault)
    {
        case MOTION_FAULT_NONE:     p_name = "none";     break;
        case MOTION_FAULT_COMMAND:  p_name = "command";  break;
        case MOTION_FAULT_EXPIRED:  p_name = "expired";  break;
        case MOTION_FAULT_FEEDBACK: p_name = "feedback"; break;
        case MOTION_FAULT_GAP:      p_name = "gap";      break;
        case MOTION_FAULT_NO_EDGES: p_name = "no_edges"; break;
        case MOTION_FAULT_OUTPUT:   p_name = "output";   break;
        default:                    p_name = "unknown";  break;
    }
    return p_name;
}

/* Brake (ordinary, then latched) and report why. Never returns. */
static void drive_end (const char * p_why, int code)
{
    guidance_request_t hold = { 0 };

    hold.action = GUIDANCE_HOLD;
    hold.sensed_us = b4_time_us ();
    hold.valid_for_us = DRIVE_LEASE_US;
    (void)motion_submit (&hold);
    (void)tk_dly_tsk (200);
    (void)motion_stop ();
    g_driving = false;
    tm_printf ((UB *)"B4H,drive_end=%s,code=%d\n", p_why, code);
    for (;;)
    {
        (void)tk_slp_tsk (TMO_FEVR);
    }
}

LOCAL void drive_task (INT stacd, void * exinf)
{
    motion_status_t status;
    guidance_request_t req = { 0 };
    uint32_t start_us;
    uint32_t elapsed_ms;
    uint8_t low_count;
    ER er;

    (void)stacd;
    (void)exinf;

    while ((E_OK != motion_status (&status)) || !status.ready)
    {
        (void)tk_dly_tsk (10);
    }
    er = motion_bind (tk_get_tid ());
    if (E_OK != er)
    {
        drive_end ("bind", (int)er);
    }

    (void)tk_dly_tsk (CAL_SETTLE_MS);
    tm_printf ((UB *)"B4H,ready: press GP20 to drive %d um/s,"
               " GP21 to stop\n", DRIVE_SPEED_UM_S);
    while (0U != gpio_get_val (START_PIN))
    {
        (void)tk_dly_tsk (10);
    }
    tm_printf ((UB *)"B4H,drive_start\n");

    req.action = GUIDANCE_STRAIGHT;
    req.speed_um_s = DRIVE_SPEED_UM_S;
    req.inside_um_s = 0;
    req.valid_for_us = DRIVE_LEASE_US;
    start_us = b4_time_us ();
    low_count = 0U;
    for (;;)
    {
        elapsed_ms = (b4_time_us () - start_us) / 1000U;

        /* Our own GP21 check, debounced: 3 low reads in a row (30 ms). */
        if (0U == gpio_get_val (STOP_PIN))
        {
            if (low_count < 255U)
            {
                low_count++;
            }
        }
        else
        {
            low_count = 0U;
        }
        if (low_count >= STOP_LOW_READS)
        {
            drive_end ("gp21", 0);
        }

        if (E_OK != motion_status (&status))
        {
            status.stop_requested = false;
            status.state = MOTION_FAULT;
            status.fault = MOTION_FAULT_NONE;
        }
        if (status.stop_requested)
        {
            drive_end ("gp21", 0);   /* Motion has already latched a brake. */
        }
        /* A motion FAULT is latched inside Buddy 2's controller until
         * reboot. Do not print here (core 0); monitor_task reports it. */

        req.sensed_us = b4_time_us ();
        er = motion_submit (&req);
        g_drive_ms = elapsed_ms;
        g_last_submit = (int)er;
        g_driving = true;
        (void)tk_dly_tsk (DRIVE_PERIOD_MS);
    }
}

/* Core 1 only. tm_printf holds an interrupt-masking spinlock for the
 * whole call (libtm_printf.c), so a print on core 0, or a core-0 print
 * waiting for a core-1 print, delays the encoder interrupts. Missed
 * quadrature states make motion's speed invalid and it faults with
 * MOTION_FAULT_FEEDBACK. So nothing prints on core 0 while driving. */
LOCAL void monitor_task (INT stacd, void * exinf)
{
    b4_task_stats_t stats;
    motion_status_t mstat;
    motion_feedback_t fb;
    uint32_t prev_us;
    uint32_t now_us;
    uint32_t tick = 0U;
    bool fault_seen = false;

    (void)stacd;
    (void)exinf;
    prev_us = b4_time_us ();
    for (;;)
    {
        (void)tk_dly_tsk (DRIVE_REPORT_MS);
        tick++;
        if (E_OK != motion_status (&mstat))
        {
            mstat.ready = false;
            mstat.state = MOTION_FAULT;
            mstat.fault = MOTION_FAULT_NONE;
        }
        if (E_OK != motion_feedback_snapshot (&fb))
        {
            fb.left_speed_um_s = 0;
            fb.right_speed_um_s = 0;
            fb.speed_valid = false;
        }
        if (g_driving && !fault_seen && (MOTION_FAULT == mstat.state))
        {
            tm_printf ((UB *)"B4H,motion_fault=%d (%s) at %u ms\n",
                       (int)mstat.fault, fault_name (mstat.fault),
                       (unsigned int)g_drive_ms);
            fault_seen = true;
        }
        if (g_driving)
        {
            tm_printf ((UB *)"B4D,t_ms=%u,state=%d,fault=%d,submit=%d,"
                       "vl_mm_s=%d,vr_mm_s=%d,valid=%d\n",
                       (unsigned int)g_drive_ms, (int)mstat.state,
                       (int)mstat.fault, g_last_submit,
                       (int)(fb.left_speed_um_s / 1000),
                       (int)(fb.right_speed_um_s / 1000),
                       (int)fb.speed_valid);
        }
        if (0U != (tick % (STATS_PERIOD_MS / DRIVE_REPORT_MS)))
        {
            continue;
        }
        now_us = b4_time_us ();
        b4_task_get_stats (&stats);
        tm_printf ((UB *)"B4S,cycles=%u,overruns=%u,max_work_us=%u,"
                   "inits=%u,init=%d,i2c_tx=%u,retries=%u,nacks=%u,"
                   "timeouts=%u,fails=%u,nav=%u/%u,telem=%u/%u,word=%08x,"
                   "timer_5s=%u,m_ready=%d,m_state=%d,m_fault=%d\n",
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
                   (unsigned int)stats.last_word,
                   (unsigned int)(now_us - prev_us),
                   (int)mstat.ready, (int)mstat.state, (int)mstat.fault);
        prev_us = now_us;
    }
}

LOCAL T_CTSK ctsk_startup = {
    .itskpri = 2, .stksz = STACK_SZ, .task = startup_task,
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
};
LOCAL T_CTSK ctsk_motion = {
    .itskpri = 1, .stksz = STACK_SZ, .task = motion_task,
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
};
LOCAL T_CTSK ctsk_emergency = {
    .itskpri = 1, .stksz = STACK_SZ, .task = emergency_task,
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
};
LOCAL T_CTSK ctsk_monitor = {
    .itskpri = 6, .stksz = STACK_SZ, .task = monitor_task,
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC2,
};
LOCAL T_CTSK ctsk_drive = {
    .itskpri = 2, .stksz = STACK_SZ, .task = drive_task,
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC, .assprc = TP_PRC1,
};

/* Create and start one task; print the error if either step fails. */
static void start_task (const char * p_name, T_CTSK * p_ctsk)
{
    ID tid = tk_cre_tsk (p_ctsk);
    ER er;

    if (tid <= 0)
    {
        tm_printf ((UB *)"B4H,tk_cre_tsk(%s)=%d\n", p_name, (int)tid);
        return;
    }
    er = tk_sta_tsk (tid, 0);
    if (E_OK != er)
    {
        tm_printf ((UB *)"B4H,tk_sta_tsk(%s)=%d\n", p_name, (int)er);
    }
}

EXPORT INT usermain (void)
{
    b4_task_config_t cfg;
    ER er;

    /* 1 MHz timer first: motion's leases and speeds use the same counter. */
    b4_timer_tick_init ();

    er = motion_init ();          /* Also brakes GP8-GP11. */
    tm_printf ((UB *)"B4H,motion_init=%d\n", (int)er);
    if (E_OK != er)
    {
        (void)tk_slp_tsk (TMO_FEVR);
        return 1;
    }

    (void)gpio_set_pin (START_PIN, GPIO_MODE_IN);
    clr_w (GPIO (START_PIN), GPIO_PDE);
    set_w (GPIO (START_PIN), GPIO_PUE | GPIO_SHEMITT);

    start_task ("startup", &ctsk_startup);
    start_task ("emergency", &ctsk_emergency);
    start_task ("motion", &ctsk_motion);
    start_task ("drive", &ctsk_drive);

    b4_task_default_config (&cfg);

    /* Same mounting and tilt settings as buddy4_imu_check.c (car days 1-2,
     * S5 v3 within 0.7 deg). Keep the two files in step. */
    cfg.hump.fwd_axis = HUMP_AXIS_Y;
    cfg.hump.fwd_sign = -1;
    cfg.hump.mode = HUMP_MODE_TILT;
    cfg.hump.enter_deg = 3.0f;
    cfg.hump.exit_band_deg = 1.5f;
    cfg.hump.enter_samples = 10U;

    /* Wheel feedback is live here, so only count humps while moving. The
     * default 0.08 m/s is too close to the 0.1 m/s drive speed: the wheel
     * slows on the climb. */
    cfg.hump.require_motion = true;
    cfg.hump.min_speed_mps = 0.03f;

    cfg.motion.mag_yaw_sign = 1;
    cfg.motion.mag_offset_g[0] = 0.227f;
    cfg.motion.mag_offset_g[1] = -0.008f;
    cfg.motion.mag_offset_g[2] = 0.0f;

    /* 25 rows/s: half the USB traffic, which is serviced on core 0 next to
     * the encoder interrupts. Detection still runs every 20 ms. */
    cfg.log_every = 2U;
    cfg.read_b2 = b4_read_b2_motion_service;

    er = b4_task_start (&cfg);
    tm_printf ((UB *)"B4H,b4_task_start=%d\n", (int)er);
    if (E_OK != er)
    {
        (void)motion_stop ();
        (void)tk_slp_tsk (TMO_FEVR);
        return 1;
    }

    start_task ("monitor", &ctsk_monitor);
    (void)tk_slp_tsk (TMO_FEVR);   /* Must not return. */
    return 0;
}

/*** end of file ***/
