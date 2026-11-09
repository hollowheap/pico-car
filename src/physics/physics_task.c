/** @file physics_task.c
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

#include "motion_adapter.h"
#include "i2c_rp2040.h"
#include "pipeline.h"
#include "physics_task.h"
#include "hump_msg.h"
#include "lsm303.h"

#define INIT_RETRY_CYCLES  (50U)    /* Retry a missing sensor every 1 s. */
#define SLEEP_TIMEOUT_MS   (200)    /* Wake-up missing for 10 periods.   */
#define SCALE_LIMIT        (2.0e9f) /* Keeps the int32_t cast defined.   */
#define SCALE_DECI         (10.0f)
#define SCALE_MILLI        (1000.0f)
#define SCALE_MICRO        (1.0e6f)
#define FAULTS_UNSET       (0xFFU)  /* Matches no real fault word.       */
#define US_TO_S            (1.0e-6f)
#define MS_TO_S            (1.0e-3f)
#define ROUND_HALF         (0.5f)
#define HEADING_WRAP_DEG   (360U)

/* physics_task_default_config values. */
#define DEFAULT_TASK_PRIORITY (4)
#define DEFAULT_PERIOD_MS     (20U)
#define DEFAULT_MAG_EVERY     (3U)
#define DEFAULT_TELEM_EVERY   (10U)
#define DEFAULT_I2C_ATTEMPTS  (4U)
#define DEFAULT_ACCEL_ALPHA   (0.3f)

static physics_task_config_t g_cfg;
static pipeline_t    g_pipe;
static motion_adapter_t  g_motion;
static lsm303_t         g_imu;
static i2c_bus_t     g_bus;
static ID               g_task_id = 0;
static ID               g_nav_mbf = 0;
static ID               g_telem_mbf = 0;
static bool             g_started = false;

/* Counters: written by the task, copied by readers under the spinlock. */
static T_SPLOCK         g_stats_lock;
static physics_task_stats_t  g_stats;

/* Requests from other tasks: single-word flags, set elsewhere, cleared here. */
static volatile bool    g_req_new_run = false;
static volatile bool    g_req_clear_fault = false;

static void cyclic_wakeup (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk (g_task_id);
}

static void stats_update (const physics_task_stats_t * p_local)
{
    UINT saved = 0U;

    (void)ISpinLock (&g_stats_lock, &saved);
    g_stats = *p_local;
    (void)ISpinUnlock (&g_stats_lock, saved);
}

static void send (ID mbf, packed_hump_t word, uint32_t * p_sent,
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

    if (x > SCALE_LIMIT)
    {
        x = SCALE_LIMIT;
    }
    if (x < -SCALE_LIMIT)
    {
        x = -SCALE_LIMIT;
    }
    return (int32_t)x;
}

/* One CSV row. tm_printf has no %f, so values are scaled integers. */
static void log_row (uint32_t t_us, const lsm303_vec_t * p_acc, bool acc_ok,
                     const lsm303_vec_t * p_mag, bool mag_ok,
                     const motion_input_t * p_mot,
                     const pipeline_step_t * p_step)
{
    tm_printf ((UB *)"B4,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%08x\n",
               (unsigned int)t_us,
               acc_ok ? (int)p_acc->raw[LSM303_AXIS_X] : 0,
               acc_ok ? (int)p_acc->raw[LSM303_AXIS_Y] : 0,
               acc_ok ? (int)p_acc->raw[LSM303_AXIS_Z] : 0,
               mag_ok ? (int)p_mag->raw[LSM303_AXIS_X] : 0,
               mag_ok ? (int)p_mag->raw[LSM303_AXIS_Y] : 0,
               mag_ok ? (int)p_mag->raw[LSM303_AXIS_Z] : 0,
               mag_ok ? 1 : 0,
               p_mot->valid ? 1 : 0,
               (int)scaled (p_mot->v_left_mps, SCALE_MILLI),
               (int)scaled (p_mot->v_right_mps, SCALE_MILLI),
               (int)scaled (p_mot->ds_m, SCALE_MICRO),
               (int)scaled (p_mot->accel_mps2, SCALE_MILLI),
               p_step->sensors_ok ? (int)scaled (p_step->hump.pitch_deg, 100.0f)
                                  : 0,
               p_step->sensors_ok ? (int)p_step->hump.state : -1,
               (int)p_step->motion.motion,
               p_step->sensors_ok ? (int)scaled (p_step->motion.yaw_rate_dps, SCALE_DECI)
                                  : 0,
               (p_step->sensors_ok && p_step->hump.calibrated) ? 1 : 0,
               p_step->sensors_ok ? (int)scaled (p_step->hump.theta0_deg, 100.0f)
                                  : 0,
               (unsigned int)p_step->word);
}

static void log_header (void)
{
    tm_printf ((UB *)"PHYS_H,t_us,ax_mg,ay_mg,az_mg,mx_raw,my_raw,mz_raw,mag_new,"
               "motion_valid,vl_mm_s,vr_mm_s,ds_um,acc_mm_s2,pitch_cdeg,state,"
               "motion,yaw_ddps,cal,theta0_cdeg,word\n");
    tm_printf ((UB *)"PHYS_H,mag raw: X/Y 1100 LSB/G, Z 980 LSB/G\n");
}

/* Sleep until the next period and count missed wake-ups. */
static void wait_cycle (physics_task_stats_t * p_local)
{
    INT queued = 0;
    ER er = E_OK;

    er = tk_slp_tsk (SLEEP_TIMEOUT_MS);
    queued = tk_can_wup (TSK_SELF);
    if ((E_OK == er) && (queued > 0))
    {
        p_local->overruns += (uint32_t)queued;
    }
    else if (E_TMOUT == er)
    {
        p_local->overruns++;
    }
    else
    {
        /* Normal wake-up. */
    }
}

/* Bring the sensor up if it was missing or failed to configure. */
static void retry_sensor_init (physics_task_stats_t * p_local,
                               uint32_t * p_retry_count)
{
    if (!g_imu.ready)
    {
        (*p_retry_count)++;
        if (*p_retry_count >= INIT_RETRY_CYCLES)
        {
            *p_retry_count = 0U;
            p_local->sensor_inits++;
            p_local->last_init = lsm303_init (&g_imu, &g_bus,
                                              g_cfg.i2c_attempts);
        }
    }
}

/* Read the accelerometer every cycle and the magnetometer every mag_every. */
static void read_sensors (const physics_task_stats_t * p_local,
                          lsm303_vec_t * p_acc, lsm303_vec_t * p_mag,
                          bool * p_acc_ok, bool * p_mag_ok)
{
    if (g_imu.ready)
    {
        *p_acc_ok = (LSM303_OK == lsm303_read_accel (&g_imu, p_acc));
        if ((0U == (p_local->cycles % g_cfg.mag_every))
            && g_cfg.motion.mag_enabled)
        {
            *p_mag_ok = (LSM303_OK == lsm303_read_mag (&g_imu, p_mag));
        }
    }
}

/* Heading and calibration state for the stats; heading is whole degrees
 * wrapped into 0..359. */
static void update_heading (physics_task_stats_t * p_local,
                            const pipeline_step_t * p_step)
{
    p_local->heading_valid = p_step->sensors_ok && p_step->motion.heading_valid;
    p_local->calibrated = p_step->hump.calibrated;
    if (p_local->heading_valid)
    {
        uint32_t heading = (uint32_t)(p_step->motion.heading_deg + ROUND_HALF);

        if (heading >= HEADING_WRAP_DEG)
        {
            heading -= HEADING_WRAP_DEG;
        }
        p_local->heading_deg = (uint8_t)heading;
    }
    else
    {
        p_local->heading_deg = 0U;
    }
}

/* Send the word to the receivers the step selected, then report a change in
 * the physics fault bits. */
static void dispatch_step (physics_task_stats_t * p_local,
                           const pipeline_step_t * p_step,
                           uint8_t * p_last_faults)
{
    hump_fields_t fields;

    if (p_step->to_nav)
    {
        send (g_nav_mbf, p_step->word, &p_local->nav_sent,
              &p_local->nav_dropped);
    }
    if (p_step->to_telem)
    {
        send (g_telem_mbf, p_step->word, &p_local->telem_sent,
              &p_local->telem_dropped);
    }

    (void)hump_unpack (p_step->word, &fields);
    if ((fields.faults != *p_last_faults) && (NULL != g_cfg.publish_faults))
    {
        g_cfg.publish_faults (fields.faults);
    }
    *p_last_faults = fields.faults;
}

void physics_task_run (void)
{
    physics_task_stats_t local = { 0 };
    lsm303_vec_t acc = { 0 };
    lsm303_vec_t mag = { 0 };
    uint32_t prev_us = 0U;
    uint32_t retry_count = 0U;
    uint8_t last_faults = FAULTS_UNSET;

    i2c_rp2040_init (&g_bus);
    local.sensor_inits = 1U;
    local.last_init = lsm303_init (&g_imu, &g_bus, g_cfg.i2c_attempts);
    if (g_cfg.log_every > 0U)
    {
        log_header ();
        tm_printf ((UB *)"PHYS_H,lsm303_init=%d\n", (int)local.last_init);
    }
    prev_us = physics_time_us ();

    for (;;)
    {
        motion_reading_t reading = { 0 };
        motion_input_t motion;
        pipeline_step_t step;
        bool have_motion = false;
        bool acc_ok = false;
        bool mag_ok = false;
        uint32_t now_us = 0U;
        uint32_t work_us = 0U;
        float dt_s = 0.0f;

        wait_cycle (&local);

        now_us = physics_time_us ();
        dt_s = (float)(now_us - prev_us) * US_TO_S;
        prev_us = now_us;
        if (dt_s <= 0.0f)
        {
            dt_s = (float)g_cfg.period_ms * MS_TO_S;
        }

        if (g_req_clear_fault)
        {
            g_req_clear_fault = false;
            lsm303_clear_fault (&g_imu);
        }
        if (g_req_new_run)
        {
            g_req_new_run = false;
            pipeline_new_run (&g_pipe);
        }

        retry_sensor_init (&local, &retry_count);
        read_sensors (&local, &acc, &mag, &acc_ok, &mag_ok);

        if (NULL != g_cfg.read_motion)
        {
            have_motion = g_cfg.read_motion (&reading);
        }
        (void)motion_adapter_convert (&g_motion, have_motion ? &reading : NULL, now_us,
                             &motion);

        (void)pipeline_step (&g_pipe, acc_ok ? acc.value : NULL,
                                mag_ok ? mag.value : NULL, &motion, dt_s,
                                g_imu.fault || !g_imu.ready, &step);

        update_heading (&local, &step);
        dispatch_step (&local, &step, &last_faults);

        if ((g_cfg.log_every > 0U) && (0U == (local.cycles % g_cfg.log_every)))
        {
            log_row (now_us, &acc, acc_ok, &mag, mag_ok, &motion, &step);
        }

        work_us = physics_time_us () - now_us;
        if (work_us > local.max_work_us)
        {
            local.max_work_us = work_us;
        }
        local.cycles++;
        local.i2c = g_imu.stats;
        local.last_word = step.word;
        stats_update (&local);
    }
}

void physics_task_default_config (physics_task_config_t * p_cfg)
{
    if (NULL != p_cfg)
    {
        static const physics_task_config_t zero_cfg = { 0 };

        *p_cfg = zero_cfg;
        p_cfg->priority = DEFAULT_TASK_PRIORITY;
        p_cfg->processor = TP_PRC2;
        p_cfg->period_ms = DEFAULT_PERIOD_MS;
        p_cfg->mag_every = DEFAULT_MAG_EVERY;
        p_cfg->telem_every = DEFAULT_TELEM_EVERY;   /* 5 Hz periodic Telemetry. */
        p_cfg->log_every = 0U;
        p_cfg->i2c_attempts = DEFAULT_I2C_ATTEMPTS;  /* First try + 3 retries. */
        p_cfg->motion_accel_alpha = DEFAULT_ACCEL_ALPHA;
        p_cfg->read_motion = NULL;
        p_cfg->publish_faults = NULL;
        hump_default_config (&p_cfg->hump);
        imu_motion_default_config (&p_cfg->motion);
    }
}

ER physics_task_init (const physics_task_config_t * p_cfg, ID task_id)
{
    static const physics_task_stats_t zero_stats = { 0 };
    T_CMBF cmbf = { 0 };
    T_CCYC ccyc = { 0 };
    ID id = 0;

    if (g_started)
    {
        return E_OBJ;
    }
    if ((NULL == p_cfg) || (task_id <= 0) || (p_cfg->period_ms < 1U) ||
        (p_cfg->mag_every < 1U) ||
        ((TP_PRC1 != p_cfg->processor) && (TP_PRC2 != p_cfg->processor)))
    {
        return E_PAR;
    }

    g_cfg = *p_cfg;
    if (!pipeline_init (&g_pipe, &g_cfg.hump, &g_cfg.motion,
                           g_cfg.telem_every)
        || !motion_adapter_init (&g_motion, g_cfg.motion_accel_alpha))
    {
        return E_PAR;
    }
    (void)InitSpinLock (&g_stats_lock);
    g_stats = zero_stats;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)PHYSICS_MBF_BUFSZ;
    cmbf.maxmsz = (INT)sizeof (packed_hump_t);
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
    g_task_id   = task_id;

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

ID physics_task_nav_mbf (void)
{
    return g_nav_mbf;
}

ID physics_task_telem_mbf (void)
{
    return g_telem_mbf;
}

void physics_task_get_stats (physics_task_stats_t * p_stats)
{
    UINT saved = 0U;

    if (NULL != p_stats)
    {
        (void)ISpinLock (&g_stats_lock, &saved);
        *p_stats = g_stats;
        (void)ISpinUnlock (&g_stats_lock, saved);
    }
}

void physics_task_request_new_run (void)
{
    g_req_new_run = true;
}

void physics_task_request_clear_fault (void)
{
    g_req_clear_fault = true;
}

/*** end of file ***/
