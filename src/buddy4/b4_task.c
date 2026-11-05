/** @file b4_task.c
 *
 * @brief Buddy 4 Hump task for micro T-Kernel 3.0 (target only).
 */

/* Kernel headers first: they define size_t and NULL, so <stddef.h> and
 * <string.h> are not used here; structures are zeroed by initialisers. */
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>

#include <stdbool.h>
#include <stdint.h>

#include "b4_b2_adapter.h"
#include "b4_i2c_rp2040.h"
#include "b4_pipeline.h"
#include "b4_task.h"
#include "hump_msg.h"
#include "lsm303.h"

#define B4_STACK_SIZE      (4096)   /* SMP kernel paths need ~4 KiB.     */
#define INIT_RETRY_CYCLES  (50U)    /* Retry a missing sensor every 1 s. */
#define SLEEP_TIMEOUT_MS   (200)    /* Wake-up missing for 10 periods.   */

static b4_task_config_t g_cfg;
static b4_pipeline_t    g_pipe;
static b4_b2_adapter_t  g_b2;
static lsm303_t         g_imu;
static b4_i2c_bus_t     g_bus;
static ID               g_task_id = 0;
static ID               g_nav_mbf = 0;
static ID               g_telem_mbf = 0;
static bool             g_started = false;

/* Counters: written by the task, copied by readers under the spinlock. */
static T_SPLOCK         g_stats_lock;
static b4_task_stats_t  g_stats;

/* Requests from other tasks: single-word flags, set elsewhere, cleared here. */
static volatile bool    g_req_new_run = false;
static volatile bool    g_req_clear_fault = false;

static void cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk (g_task_id);
}

static void stats_update (const b4_task_stats_t * p_local)
{
    UINT saved;

    (void)ISpinLock (&g_stats_lock, &saved);
    g_stats = *p_local;
    (void)ISpinUnlock (&g_stats_lock, saved);
}

/* p_total = p_base + p_now, field by field. lsm303_init zeroes the device
 * counters, so the task adds them to a base before each re-init. */
static void i2c_stats_sum (const lsm303_stats_t * p_base,
                           const lsm303_stats_t * p_now,
                           lsm303_stats_t * p_total)
{
    p_total->transactions = p_base->transactions + p_now->transactions;
    p_total->retries = p_base->retries + p_now->retries;
    p_total->nacks = p_base->nacks + p_now->nacks;
    p_total->timeouts = p_base->timeouts + p_now->timeouts;
    p_total->errors = p_base->errors + p_now->errors;
    p_total->failures = p_base->failures + p_now->failures;
}

static void send (ID mbf, b4_packed_hump_t word, uint32_t * p_sent,
                  uint32_t * p_dropped)
{
    ER er = tk_snd_mbf (mbf, &word, (INT)sizeof (word), TMO_POL);

    if (E_OK == er)
    {
        (*p_sent)++;
    }
    else
    {
        (*p_dropped)++;
    }
}

static int32_t scaled (float value, float scale)
{
    float x = value * scale;

    if (x > 2.0e9f)
    {
        x = 2.0e9f;
    }
    if (x < -2.0e9f)
    {
        x = -2.0e9f;
    }
    return (int32_t)x;
}

/* One CSV row. tm_printf has no %f, so values are scaled integers. */
static void log_row (uint32_t t_us, const lsm303_vec_t * p_acc, bool acc_ok,
                     const lsm303_vec_t * p_mag, bool mag_ok,
                     const b4_motion_in_t * p_mot,
                     const b4_step_out_t * p_step)
{
    tm_printf ((UB *)"B4,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%08x\n",
               (unsigned int)t_us,
               acc_ok ? (int)p_acc->raw[0] : 0,
               acc_ok ? (int)p_acc->raw[1] : 0,
               acc_ok ? (int)p_acc->raw[2] : 0,
               mag_ok ? (int)p_mag->raw[0] : 0,
               mag_ok ? (int)p_mag->raw[1] : 0,
               mag_ok ? (int)p_mag->raw[2] : 0,
               mag_ok ? 1 : 0,
               p_mot->valid ? 1 : 0,
               (int)scaled (p_mot->v_left_mps, 1000.0f),
               (int)scaled (p_mot->v_right_mps, 1000.0f),
               (int)scaled (p_mot->ds_m, 1.0e6f),
               (int)scaled (p_mot->accel_mps2, 1000.0f),
               p_step->sensors_ok ? (int)scaled (p_step->hump.pitch_deg, 100.0f)
                                  : 0,
               p_step->sensors_ok ? (int)p_step->hump.state : -1,
               (int)p_step->motion.motion,
               p_step->sensors_ok ? (int)scaled (p_step->motion.yaw_rate_dps, 10.0f)
                                  : 0,
               (p_step->sensors_ok && p_step->hump.calibrated) ? 1 : 0,
               p_step->sensors_ok ? (int)scaled (p_step->hump.theta0_deg, 100.0f)
                                  : 0,
               (unsigned int)p_step->word);
}

static void log_header (void)
{
    tm_printf ((UB *)"B4H,t_us,ax_mg,ay_mg,az_mg,mx_raw,my_raw,mz_raw,mag_new,"
               "b2_valid,vl_mm_s,vr_mm_s,ds_um,acc_mm_s2,pitch_cdeg,state,"
               "motion,yaw_ddps,cal,theta0_cdeg,word\n");
    tm_printf ((UB *)"B4H,mag raw: X/Y 1100 LSB/G, Z 980 LSB/G\n");
}

static void hump_task (INT stacd, void * p_exinf)
{
    b4_task_stats_t local = { 0 };
    lsm303_stats_t i2c_base = { 0 };
    lsm303_vec_t acc = { 0 };
    lsm303_vec_t mag = { 0 };
    uint32_t prev_us;
    uint32_t retry_count = 0U;
    uint8_t last_faults = 0xFFU;

    (void)stacd;
    (void)p_exinf;

    b4_i2c_rp2040_init (&g_bus);
    local.sensor_inits = 1U;
    local.last_init = lsm303_init (&g_imu, &g_bus, g_cfg.i2c_attempts);
    if (g_cfg.log_every > 0U)
    {
        log_header ();
        tm_printf ((UB *)"B4H,lsm303_init=%d\n", (int)local.last_init);
    }
    prev_us = b4_time_us ();

    for (;;)
    {
        b4_b2_reading_t reading = { 0 };
        b4_motion_in_t motion;
        b4_step_out_t step;
        b4_hump_fields_t fields;
        bool have_b2 = false;
        bool acc_ok = false;
        bool mag_ok = false;
        uint32_t now_us;
        uint32_t work_us;
        float dt_s;
        INT queued;
        ER er;

        er = tk_slp_tsk (SLEEP_TIMEOUT_MS);
        queued = tk_can_wup (TSK_SELF);
        if ((E_OK == er) && (queued > 0))
        {
            local.overruns += (uint32_t)queued;
        }
        else if (E_TMOUT == er)
        {
            local.overruns++;
        }
        else
        {
            /* Normal wake-up. */
        }

        now_us = b4_time_us ();
        dt_s = (float)(now_us - prev_us) * 1.0e-6f;
        prev_us = now_us;
        if (dt_s <= 0.0f)
        {
            dt_s = (float)g_cfg.period_ms * 1.0e-3f;
        }

        if (g_req_clear_fault)
        {
            g_req_clear_fault = false;
            lsm303_clear_fault (&g_imu);
        }
        if (g_req_new_run)
        {
            g_req_new_run = false;
            b4_pipeline_new_run (&g_pipe);
        }

        /* Bring the sensor up if it was missing, failed to configure, or was
         * lost (lsm303 clears ready after LSM303_LOST_AFTER bad reads, e.g.
         * a re-powered chip reading all zero). */
        if (!g_imu.ready)
        {
            retry_count++;
            if (retry_count >= INIT_RETRY_CYCLES)
            {
                lsm303_stats_t total;

                retry_count = 0U;
                i2c_stats_sum (&i2c_base, &g_imu.stats, &total);
                i2c_base = total;
                local.sensor_inits++;
                local.last_init = lsm303_init (&g_imu, &g_bus,
                                               g_cfg.i2c_attempts);
            }
        }

        if (g_imu.ready)
        {
            acc_ok = (LSM303_OK == lsm303_read_accel (&g_imu, &acc));
            if ((0U == (local.cycles % g_cfg.mag_every))
                && g_cfg.motion.mag_enabled)
            {
                mag_ok = (LSM303_OK == lsm303_read_mag (&g_imu, &mag));
            }
        }

        if (NULL != g_cfg.read_b2)
        {
            have_b2 = g_cfg.read_b2 (&reading);
        }
        (void)b4_b2_convert (&g_b2, have_b2 ? &reading : NULL, now_us,
                             &motion);

        (void)b4_pipeline_step (&g_pipe, acc_ok ? acc.value : NULL,
                                mag_ok ? mag.value : NULL, &motion, dt_s,
                                g_imu.fault || !g_imu.ready, &step);

        if (step.to_nav)
        {
            send (g_nav_mbf, step.word, &local.nav_sent, &local.nav_dropped);
        }
        if (step.to_telem)
        {
            send (g_telem_mbf, step.word, &local.telem_sent,
                  &local.telem_dropped);
        }

        (void)b4_hump_unpack (step.word, &fields);
        if ((fields.faults != last_faults) && (NULL != g_cfg.publish_faults))
        {
            g_cfg.publish_faults (fields.faults);
        }
        last_faults = fields.faults;

        if ((g_cfg.log_every > 0U) && (0U == (local.cycles % g_cfg.log_every)))
        {
            log_row (now_us, &acc, acc_ok, &mag, mag_ok, &motion, &step);
        }

        work_us = b4_time_us () - now_us;
        if (work_us > local.max_work_us)
        {
            local.max_work_us = work_us;
        }
        local.cycles++;
        i2c_stats_sum (&i2c_base, &g_imu.stats, &local.i2c);
        local.last_word = step.word;
        stats_update (&local);
    }
}

void b4_task_default_config (b4_task_config_t * p_cfg)
{
    if (NULL != p_cfg)
    {
        static const b4_task_config_t zero_cfg = { 0 };

        *p_cfg = zero_cfg;
        p_cfg->priority = 4;
        p_cfg->processor = TP_PRC2;
        p_cfg->period_ms = 20U;
        p_cfg->mag_every = 3U;
        p_cfg->telem_every = 10U;       /* 5 Hz periodic Telemetry.  */
        p_cfg->log_every = 0U;
        p_cfg->i2c_attempts = 4U;       /* First try + 3 retries.    */
        p_cfg->b2_accel_alpha = 0.3f;
        p_cfg->read_b2 = NULL;
        p_cfg->publish_faults = NULL;
        hump_default_config (&p_cfg->hump);
        imu_motion_default_config (&p_cfg->motion);
    }
}

ER b4_task_start (const b4_task_config_t * p_cfg)
{
    static const b4_task_stats_t zero_stats = { 0 };
    T_CMBF cmbf = { 0 };
    T_CTSK ctsk = { 0 };
    T_CCYC ccyc = { 0 };
    ID id;

    if (g_started)
    {
        return E_OBJ;
    }
    if ((NULL == p_cfg) || (p_cfg->period_ms < 1U) || (p_cfg->mag_every < 1U)
        || ((TP_PRC1 != p_cfg->processor) && (TP_PRC2 != p_cfg->processor)))
    {
        return E_PAR;
    }

    g_cfg = *p_cfg;
    if (!b4_pipeline_init (&g_pipe, &g_cfg.hump, &g_cfg.motion,
                           g_cfg.telem_every)
        || !b4_b2_adapter_init (&g_b2, g_cfg.b2_accel_alpha))
    {
        return E_PAR;
    }
    (void)InitSpinLock (&g_stats_lock);
    g_stats = zero_stats;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)B4_MBF_BUFSZ;
    cmbf.maxmsz = (INT)sizeof (b4_packed_hump_t);
    id = tk_cre_mbf (&cmbf);
    if (id < E_OK)
    {
        return (ER)id;
    }
    g_nav_mbf = id;
    id = tk_cre_mbf (&cmbf);
    if (id < E_OK)
    {
        return (ER)id;
    }
    g_telem_mbf = id;

    ctsk.task = (FP)hump_task;
    ctsk.itskpri = p_cfg->priority;
    ctsk.stksz = B4_STACK_SIZE;
    ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC;
    ctsk.assprc = p_cfg->processor;
    id = tk_cre_tsk (&ctsk);
    if (id < E_OK)
    {
        return (ER)id;
    }
    g_task_id = id;
    id = tk_sta_tsk (g_task_id, 0);
    if (id < E_OK)
    {
        return (ER)id;
    }

    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP)cyclic_wakeup;
    ccyc.cyctim = (RELTIM)p_cfg->period_ms;
    ccyc.cycphs = (RELTIM)p_cfg->period_ms;
    id = tk_cre_cyc (&ccyc);
    if (id < E_OK)
    {
        return (ER)id;
    }

    g_started = true;
    return E_OK;
}

ID b4_task_nav_mbf (void)
{
    return g_nav_mbf;
}

ID b4_task_telem_mbf (void)
{
    return g_telem_mbf;
}

void b4_task_get_stats (b4_task_stats_t * p_stats)
{
    UINT saved;

    if (NULL != p_stats)
    {
        (void)ISpinLock (&g_stats_lock, &saved);
        *p_stats = g_stats;
        (void)ISpinUnlock (&g_stats_lock, saved);
    }
}

void b4_task_request_new_run (void)
{
    g_req_new_run = true;
}

void b4_task_request_clear_fault (void)
{
    g_req_clear_fault = true;
}

/*** end of file ***/
