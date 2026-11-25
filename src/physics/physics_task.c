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
#include "platform/faults.h"
#include "platform/liveness.h"
#include "platform/units.h"
#include "supervisor/health_ids.h"
#include "platform/log.h"
#include "telemetry/wire_schema.h"

#define INIT_RETRY_CYCLES  (50U)    /* Retry a missing sensor every 1 s. */
/* Attempts per bus transaction for a re-init and its readback. The bounded
 * waits of one transaction total at most 9 ms (see i2c_rp2040.h). */
#define REINIT_ATTEMPTS    (2U)
#define CONFIG_CHECK_CYCLES (100U)  /* Configuration readback every 2 s. */
#define SLEEP_TIMEOUT_MS   (200)    /* Wake-up missing for 10 periods.   */
#define SCALE_LIMIT        (2.0e9f) /* Keeps the int32_t cast defined.   */
#define SCALE_DECI         (10.0f)
#define SCALE_MILLI        (1000.0f)
#define SCALE_MICRO        (1.0e6f)
#define FAULTS_UNSET       (0xFFU)  /* Matches no real fault word.       */
#define US_TO_S            (1.0e-6f)
#define MS_TO_S            (1.0e-3f)
#define ROUND_HALF         (0.5f)
#define SYSTIM_HIGH_SHIFT  (32U)
#define SNAPSHOT_LOSS_MS   (PHYSICS_DEFAULT_PERIOD_MS * PIPELINE_LOST_CYCLES)
#define HEADING_WRAP_DDEG  (3600U)
#define SCALE_CENTI        (100.0f)
#define HALF_TURN_CDEG     (18000)
#define FULL_TURN_CDEG     (36000)
#define ROLL_FILTER_DIVISOR (8)

/* physics_task_default_config values. */
#define DEFAULT_TASK_PRIORITY (4)
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
static bool             g_started = false;

/* Counters: written by the task, copied by readers under the spinlock. */
static T_SPLOCK         g_stats_lock;
static physics_task_stats_t  g_stats;

/* Sample the latest requests under the publication lock. The task applies
 * them outside the lock and publishes the reference ID with the result. */
static uint32_t         g_requested_reference;
/* A level, not a request: the owner sets it every pass; the last value wins. */
static volatile bool    g_wheels_free = false;

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
    /* Scaled before the call: the lint check reads the scale macros as magic
     * numbers when they sit inside the LOG_NOW arguments. */
    int const v_left_mm_s = (int)scaled (p_mot->v_left_mps, SCALE_MILLI);
    int const v_right_mm_s = (int)scaled (p_mot->v_right_mps, SCALE_MILLI);
    int const ds_um       = (int)scaled (p_mot->ds_m, SCALE_MICRO);
    int const acc_mm_s2   = (int)scaled (p_mot->accel_mps2, SCALE_MILLI);
    int const yaw_ddps    = p_step->sensors_ok
                                ? (int)scaled (p_step->motion.yaw_rate_dps, SCALE_DECI)
                                : 0;

    LOG_NOW ("DATA", "physics:imu", "B4,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%08x\n",
               (unsigned int)t_us,
               acc_ok ? (int)p_acc->raw[LSM303_AXIS_X] : 0,
               acc_ok ? (int)p_acc->raw[LSM303_AXIS_Y] : 0,
               acc_ok ? (int)p_acc->raw[LSM303_AXIS_Z] : 0,
               mag_ok ? (int)p_mag->raw[LSM303_AXIS_X] : 0,
               mag_ok ? (int)p_mag->raw[LSM303_AXIS_Y] : 0,
               mag_ok ? (int)p_mag->raw[LSM303_AXIS_Z] : 0,
               mag_ok ? 1 : 0,
               p_mot->valid ? 1 : 0,
               v_left_mm_s, v_right_mm_s, ds_um, acc_mm_s2,
               p_step->sensors_ok ? (int)scaled (p_step->hump.pitch_deg, 100.0f)
                                  : 0,
               p_step->sensors_ok ? (int)p_step->hump.state : -1,
               (int)p_step->motion.motion,
               yaw_ddps,
               (p_step->sensors_ok && p_step->hump.calibrated) ? 1 : 0,
               p_step->sensors_ok ? (int)scaled (p_step->hump.theta0_deg, 100.0f)
                                  : 0,
               (unsigned int)p_step->word);
}

static void log_header (void)
{
    LOG_NOW ("DATA", "physics:imu", "columns: t_us,ax_mg,ay_mg,az_mg,mx_raw,my_raw,mz_raw,mag_new,"
               "motion_valid,vl_mm_s,vr_mm_s,ds_um,acc_mm_s2,pitch_cdeg,state,"
               "motion,yaw_ddps,cal,theta0_cdeg,word\n");
    LOG_NOW ("INFO", "physics:imu", "mag raw: X/Y 1100 LSB/G, Z 980 LSB/G\n");
}

bool physics_task_sample_recent (const SYSTIM * p_sample, const SYSTIM * p_now)
{
    uint64_t const sample_ms = ((uint64_t)(uint32_t)p_sample->hi << SYSTIM_HIGH_SHIFT) | p_sample->lo;
    uint64_t const now_ms = ((uint64_t)(uint32_t)p_now->hi << SYSTIM_HIGH_SHIFT) | p_now->lo;

    return (p_sample->hi >= 0) && (p_now->hi >= 0) && ((now_ms - sample_ms) < SNAPSHOT_LOSS_MS);
}

/* Sleep until the next period, count missed wake-ups and stamp the new cycle. */
static void wait_cycle (physics_task_stats_t * p_local)
{
    INT queued = 0;
    ER er = E_OK;
    uint32_t const overruns = p_local->overruns;
    SYSTIM const previous = p_local->sample_time;

    er = tk_slp_tsk (SLEEP_TIMEOUT_MS);
    queued = tk_can_wup (TSK_SELF);
    if ((E_OK == er) && (queued > 0))
    {
        p_local->overruns += (uint32_t)queued;
    }
    else if (E_TMOUT == er)
    {
        p_local->overruns++;
        system_fault_assign (FAULT_OWNER_PHYSICS,
                             (uint16_t) ((uint16_t) PHYSICS_FAULT_STALE << PHYSICS_SHARED_FAULT_SHIFT),
                             (uint16_t) ((uint16_t) PHYSICS_FAULT_STALE << PHYSICS_SHARED_FAULT_SHIFT));
    }
    else
    {
        /* Normal wake-up. */
    }
    if (E_OK != tk_get_otm (&p_local->sample_time))
    {
        p_local->sample_time.hi = -1;
    }
    if ((overruns != p_local->overruns) || !physics_task_sample_recent (&previous, &p_local->sample_time))
    {
        /* Keep the attitude latch. Only consecutive observations prove set-down. */
        if (p_local->collecting_rest)
        {
            p_local->rest_interruptions++;
            p_local->collecting_rest = false;
        }
        attitude_interrupt_evidence (&g_pipe.attitude);
        p_local->attitude_sample_valid = false;
        if (!g_pipe.hump.calibrated && (g_pipe.hump.cal_count > 0U))
        {
            hump_recalibrate (&g_pipe.hump);
        }
    }
}

/* Bring the sensor up if it was missing, lost or failed to configure. Every
 * CONFIG_CHECK_CYCLES the configuration is read back: a power dip resets the
 * sensor while the bus keeps answering, and only the readback sees that. */
static void retry_sensor_init (physics_task_stats_t * p_local,
                               uint32_t * p_retry_count)
{
    if (pipeline_config_readback (&g_pipe, &g_imu, p_local->cycles,
                                  CONFIG_CHECK_CYCLES, REINIT_ATTEMPTS))
    {
        LOG_WARN (LOG_TASK_PHYSICS, LOG_MOD_IMU,
                  "IMU configuration does not match, re-initialising");
        *p_retry_count = INIT_RETRY_CYCLES;
    }
    /* A successful configuration readback can coexist with a dead data
     * register. Keep the bounded retries going until acceleration returns. */
    if (!g_imu.ready || (g_pipe.failed_cycles >= PIPELINE_LOST_CYCLES))
    {
        (*p_retry_count)++;
        if (*p_retry_count >= INIT_RETRY_CYCLES)
        {
            *p_retry_count = 0U;
            p_local->sensor_inits++;
            p_local->last_init = lsm303_init (&g_imu, &g_bus, REINIT_ATTEMPTS);
            if (LSM303_OK == p_local->last_init)
            {
                LOG_INFO (LOG_TASK_PHYSICS, LOG_MOD_IMU, "IMU initialised");
            }
        }
    }
}

/* Log the lost edges. Losing the IMU takes the sensor down, so
 * retry_sensor_init runs the init again on the next cycle. */
static void track_lost (const pipeline_step_t * p_step,
                        uint32_t * p_retry_count)
{
    if (p_step->imu_lost_started)
    {
        LOG_WARN (LOG_TASK_PHYSICS, LOG_MOD_IMU,
                  "IMU lost: no accelerometer data, re-initialising");
        g_imu.ready = false;
        *p_retry_count = INIT_RETRY_CYCLES;
    }
    if (p_step->imu_recovered)
    {
        LOG_INFO (LOG_TASK_PHYSICS, LOG_MOD_IMU, "IMU data back");
    }
}

/* Clamp a scaled value into int16_t for the snapshot. */
static int16_t clamp_i16 (int32_t value)
{
    int32_t out = value;

    if (out > INT16_MAX)
    {
        out = INT16_MAX;
    }
    if (out < INT16_MIN)
    {
        out = INT16_MIN;
    }
    return (int16_t)out;
}

/* Read the accelerometer every cycle and the magnetometer every mag_every. */
static void read_sensors (physics_task_stats_t * p_local,
                          lsm303_vec_t * p_acc, lsm303_vec_t * p_mag,
                          bool * p_acc_ok, bool * p_mag_ok, bool * p_mag_fault)
{
    bool const b_mag_due = (0U == (p_local->cycles % g_cfg.mag_every))
                           && g_cfg.motion.mag_enabled;

    *p_acc_ok = pipeline_read_accel (&g_pipe, &g_imu, g_cfg.i2c_attempts,
                                     p_acc);
    *p_mag_ok = false;
    if (*p_acc_ok)
    {
        /* Stamp before magnetometer I2C so slow mag reads do not age this
         * witness, which is independent of wheel-derived acceleration. */
        p_local->forward_sample_us = physics_time_us ();
        p_local->forward_specific_mg = clamp_i16 (scaled (
            (float)g_cfg.hump.fwd_sign * p_acc->value[g_cfg.hump.fwd_axis], SCALE_MILLI));
        /* A bus that failed the accelerometer is not asked again, and a
         * skipped sample cannot prove that the magnetometer is back. */
        if (b_mag_due)
        {
            *p_mag_ok = pipeline_read_mag (&g_pipe, &g_imu, g_cfg.i2c_attempts,
                                           p_mag);
            *p_mag_fault = !*p_mag_ok;
        }
    }
}

/* Heading, yaw rate and forward residual for the stats. The heading is in
 * deci-degrees wrapped into 0..3599, so it fits a 16-bit field; the old
 * whole-degree byte wrapped at 256. */
static void update_heading (physics_task_stats_t * p_local,
                            const pipeline_step_t * p_step)
{
    p_local->heading_valid = p_step->sensors_ok && p_step->motion.heading_valid;
    p_local->heading_gated = p_step->sensors_ok && p_step->motion.heading_gated;
    p_local->calibrated = p_step->calibrated;
    p_local->yaw_rate_cdps = p_step->sensors_ok
        ? clamp_i16 (scaled (p_step->motion.yaw_rate_dps, SCALE_CENTI))
        : 0;
    p_local->a_fwd_mg = p_step->sensors_ok
        ? clamp_i16 (scaled (p_step->motion.fwd_residual_g, SCALE_MILLI))
        : 0;
    p_local->pivot_scale_pct = p_step->sensors_ok
        ? (uint8_t)scaled (p_step->motion.pivot_scale, SCALE_CENTI)
        : 0U;
    if (p_local->heading_valid)
    {
        uint32_t heading =
            (uint32_t)((p_step->motion.heading_deg * SCALE_DECI) + ROUND_HALF);

        if (heading >= HEADING_WRAP_DDEG)
        {
            heading -= HEADING_WRAP_DDEG;
        }
        p_local->heading_ddeg = (uint16_t)heading;
    }
    else
    {
        p_local->heading_ddeg = 0U;
    }
}

/* At 50 Hz, a one-eighth update has a time constant near 150 ms. Take the
 * shortest angular difference when the chassis passes through upside down. */
static int16_t smooth_roll (int16_t previous, int16_t current)
{
    int32_t delta = (int32_t)current - previous;
    if (delta > HALF_TURN_CDEG)
    {
        delta -= FULL_TURN_CDEG;
    }
    else if (delta < -HALF_TURN_CDEG)
    {
        delta += FULL_TURN_CDEG;
    }
    int32_t result = previous + (delta / ROLL_FILTER_DIVISOR);
    if (result > HALF_TURN_CDEG)
    {
        result -= FULL_TURN_CDEG;
    }
    else if (result < -HALF_TURN_CDEG)
    {
        result += FULL_TURN_CDEG;
    }
    return (int16_t)result;
}

/* Roll, vertical acceleration and the attitude state for the stats, and the
 * lifted and overturned bits of the shared fault word (bits 11 and 12). Only
 * a known attitude is published: while the IMU is lost or has no rest
 * reference, the last known state stays, so a sensor fault neither releases
 * a lift hold nor reads as a level pose, and attitude_unknown says so. */
static void update_attitude (physics_task_stats_t * p_local, const pipeline_step_t * p_step)
{
    bool const b_was_collecting = p_local->collecting_rest;
    bool const b_previous_valid = p_local->attitude_sample_valid;
    p_local->attitude_sample_valid = pipeline_attitude_known (p_step);
    p_local->attitude_unknown = p_step->attitude_unknown;
    p_local->level_still = pipeline_attitude_known (p_step) && p_step->attitude.level_still;
    p_local->collecting_rest = p_step->sensors_ok &&
        (p_step->calibrated
         ? ((ATTITUDE_OK == p_step->attitude.state) && (g_pipe.attitude.clear_count > 0U))
         : (g_pipe.hump.cal_count > 0U));
    /* A noisy sample can immediately seed a new window. Count the break
     * even if the supervisor only sees the next good sample. */
    if (b_was_collecting && (!p_local->collecting_rest ||
                            (!p_step->calibrated && (1U == g_pipe.hump.cal_count))))
    {
        p_local->rest_interruptions++;
    }
    if (p_step->sensors_ok)
    {
        p_local->az_mg   = clamp_i16 (scaled (p_step->attitude.az_g, SCALE_MILLI));
        p_local->amag_mg = (uint16_t) clamp_i16 (scaled (p_step->attitude.amag_g, SCALE_MILLI));
    }
    if (p_local->attitude_sample_valid)
    {
        p_local->roll_cdeg = clamp_i16 (scaled (p_step->attitude.roll_deg, SCALE_CENTI));
        p_local->report_roll_cdeg = b_previous_valid ?
            smooth_roll (p_local->report_roll_cdeg, p_local->roll_cdeg) : p_local->roll_cdeg;
        p_local->pitch_cdeg = clamp_i16 (scaled (p_step->hump.pitch_deg, SCALE_CENTI));
        p_local->attitude_state = (uint8_t) p_step->attitude.state;

        system_fault_assign (FAULT_OWNER_PHYSICS, FAULT_OVERTURNED_BIT,
                             (ATTITUDE_OVERTURNED == p_step->attitude.state) ? FAULT_OVERTURNED_BIT : 0U);
    }
}

/* The packed I2C fault is latched for diagnosis. The shared level follows
 * current reads so a recovered connection can pass the START gate. */
static void dispatch_faults (const hump_fields_t * p_fields, bool b_i2c_fault,
                             uint8_t * p_last_faults)
{
    uint8_t const level = (uint8_t) ((p_fields->faults & ~PHYSICS_FAULT_I2C) |
                                    (b_i2c_fault ? PHYSICS_FAULT_I2C : 0U));
    if (level != *p_last_faults)
    {
        system_fault_assign (FAULT_OWNER_PHYSICS, PHYSICS_SHARED_FAULT_MASK,
                             (uint16_t) ((uint16_t) level
                                         << PHYSICS_SHARED_FAULT_SHIFT));
    }
    *p_last_faults = level;
}

static uint8_t attitude_event (uint8_t previous, uint8_t current)
{
    if (previous != current)
    {
        if (ATTITUDE_LIFTED == current)
        {
            return (uint8_t)TELEMETRY_EVT_LIFT_START;
        }
        if (ATTITUDE_OVERTURNED == current)
        {
            return (uint8_t)TELEMETRY_EVT_OVERTURN;
        }
        if ((ATTITUDE_LIFTED == previous) && (ATTITUDE_OK == current))
        {
            return (uint8_t)TELEMETRY_EVT_LIFT_END;
        }
    }
    return 0U;
}

static void emit_sample_events (const physics_task_stats_t * p_stats, const pipeline_step_t * p_step,
                                const hump_fields_t * p_hump, const motion_reading_t * p_reading,
                                uint8_t previous_attitude, bool b_previous_hump)
{
    bool const b_hump_edge = p_hump->hump_active != b_previous_hump;
    uint8_t const type = attitude_event (previous_attitude, p_stats->attitude_state);
    uint8_t const source = (uint8_t)TELEMETRY_EVENT_SOURCE_PHYSICS;

    if ((NULL == g_cfg.emit_event) || (!b_hump_edge && (0U == type)))
    {
        return;
    }
    /* The motion snapshot retains displacement even without fresh speed. */
    uint32_t const odo = (NULL != p_reading)
        ? wheel_odometer_mm (p_reading->displacement_um[PHYSICS_WHEEL_LEFT],
                             p_reading->displacement_um[PHYSICS_WHEEL_RIGHT]) : 0U;

    if (b_hump_edge)
    {
        if (p_hump->hump_active)
        {
            (void)g_cfg.emit_event (source, (uint8_t)TELEMETRY_EVT_HUMP_START, 0, 0U, odo);
        }
        else
        {
            int16_t const peak_mm = (int16_t)((p_hump->hump_peak_m * SCALE_MILLI) + ROUND_HALF);

            (void)g_cfg.emit_event (source, (uint8_t)TELEMETRY_EVT_HUMP_END, peak_mm,
                                   p_hump->height_valid ? 1U : 0U, odo);
        }
    }
    if (0U != type)
    {
        /* Below one milligravity the wire magnitude is zero and tilt is
         * unavailable. The sentinel preserves the measured zero-g sample. */
        int16_t const tilt = (0U == p_stats->amag_mg) ? INT16_MAX
            : clamp_i16 (scaled (p_step->attitude.tilt_deg, SCALE_CENTI));

        (void)g_cfg.emit_event (source, type, tilt, p_stats->amag_mg, odo);
    }
}

static void apply_requests (physics_task_stats_t * p_local)
{
    UINT saved = 0U;
    (void)ISpinLock (&g_stats_lock, &saved);
    uint32_t const requested = g_requested_reference;
    (void)ISpinUnlock (&g_stats_lock, saved);

    if (p_local->reference_request != requested)
    {
        pipeline_new_run (&g_pipe);
        p_local->reference_request = requested;
    }
}

void physics_task_run (void)
{
    physics_task_stats_t local = { 0 };
    lsm303_vec_t acc = { 0 };
    lsm303_vec_t mag = { 0 };
    uint32_t prev_us = 0U;
    uint32_t retry_count = 0U;
    uint8_t last_faults = FAULTS_UNSET;
    bool b_mag_fault = false;
    bool b_hump_active = false;

    i2c_rp2040_init (&g_bus);
    local.sensor_inits = 1U;
    local.last_init = lsm303_init (&g_imu, &g_bus, g_cfg.i2c_attempts);
    if (g_cfg.log_every > 0U)
    {
        log_header ();
        LOG_NOW ("INFO", "physics:imu", "lsm303_init=%d\n", (int)local.last_init);
    }
    prev_us = physics_time_us ();

    for (;;)
    {
        motion_reading_t reading = { 0 };
        motion_input_t motion;
        pipeline_step_t step;
        hump_fields_t hump = {0};
        uint8_t const previous_attitude = local.attitude_state;
        bool have_motion = false;
        bool acc_ok = false;
        bool mag_ok = false;
        uint32_t now_us = 0U;
        uint32_t work_us = 0U;
        float dt_s = 0.0f;

        wait_cycle (&local);
        liveness_kick (LIVE_PHYSICS);

        now_us = physics_time_us ();
        dt_s = (float)(now_us - prev_us) * US_TO_S;
        prev_us = now_us;
        if (dt_s <= 0.0f)
        {
            dt_s = (float)g_cfg.period_ms * MS_TO_S;
        }

        apply_requests (&local);
        retry_sensor_init (&local, &retry_count);
        read_sensors (&local, &acc, &mag, &acc_ok, &mag_ok, &b_mag_fault);

        if (NULL != g_cfg.read_motion)
        {
            have_motion = g_cfg.read_motion (&reading);
        }
        (void)motion_adapter_convert (&g_motion, have_motion ? &reading : NULL, physics_time_us (),
                             &motion);

        g_pipe.wheels_free = g_wheels_free;
        (void)pipeline_step (&g_pipe, acc_ok ? acc.value : NULL,
                                mag_ok ? mag.value : NULL, &motion, dt_s,
                                g_imu.fault || !g_imu.ready, &step);

        track_lost (&step, &retry_count);

        if (step.hump_rejected)
        {
            int const pitch_cdeg = (int)scaled (step.hump.peak_pitch_deg,
                                                SCALE_CENTI);
            int const height_mm = (int)scaled (step.hump.peak_m, SCALE_MILLI);

            LOG_WARN (LOG_TASK_PHYSICS, LOG_MOD_HUMP,
                      "hump rejected: pitch %d cdeg, height %d mm",
                      pitch_cdeg, height_mm);
        }
        update_heading (&local, &step);
        update_attitude (&local, &step);
        (void)hump_unpack (step.word, &hump);
        dispatch_faults (&hump, !acc_ok || b_mag_fault, &last_faults);

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
        emit_sample_events (&local, &step, &hump, have_motion ? &reading : NULL,
                            previous_attitude, b_hump_active);
        b_hump_active = hump.hump_active;
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
        p_cfg->period_ms = PHYSICS_DEFAULT_PERIOD_MS;
        p_cfg->mag_every = DEFAULT_MAG_EVERY;
        p_cfg->telem_every = DEFAULT_TELEM_EVERY;   /* 5 Hz periodic Telemetry. */
        p_cfg->log_every = 0U;
        p_cfg->i2c_attempts = DEFAULT_I2C_ATTEMPTS;  /* First try + 3 retries. */
        p_cfg->motion_accel_alpha = DEFAULT_ACCEL_ALPHA;
        p_cfg->read_motion = NULL;
        hump_default_config (&p_cfg->hump);
        imu_motion_default_config (&p_cfg->motion);
    }
}

ER physics_task_init (const physics_task_config_t * p_cfg, ID task_id)
{
    static const physics_task_stats_t zero_stats = { 0 };
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

/* With +Z upright, raising the nose increased the old (-Y) roll channel. */
#define CAR_HUMP_ENTER_DEG     (3.0f)
#define CAR_HUMP_EXIT_BAND_DEG (2.0f)

void physics_car_config (physics_task_config_t * p_cfg)
{
    physics_task_default_config (p_cfg);
    p_cfg->hump.fwd_axis      = HUMP_AXIS_Y;
    p_cfg->hump.fwd_sign      = -1;
    p_cfg->hump.enter_deg     = CAR_HUMP_ENTER_DEG;
    p_cfg->hump.exit_band_deg = CAR_HUMP_EXIT_BAND_DEG;
}

void physics_task_set_wheels_free (bool b_free)
{
    g_wheels_free = b_free;
}

uint32_t physics_task_request_new_run (void)
{
    UINT saved = 0U;
    (void)ISpinLock (&g_stats_lock, &saved);
    g_requested_reference++;
    uint32_t const requested = g_requested_reference;
    (void)ISpinUnlock (&g_stats_lock, saved);
    return requested;
}

/*** end of file ***/
