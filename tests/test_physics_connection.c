/* Run the physics task with controlled I2C read results. */
#include <setjmp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hump_msg.h"
#include "i2c_rp2040.h"
#include "motion_glue.h"
#include "physics/physics.h"
#include "physics_task.h"
#include "platform/faults.h"
#include "platform/chassis.h"
#include "platform/log.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"
#include <tm/tmonitor.h>

#define SAMPLE_PERIOD_MS     (20U)
#define SNAPSHOT_LOSS_MS     (500U)
#define SNAPSHOT_FREEZE_CYCLE (8U)

static jmp_buf g_done;
static uint32_t g_cycle;
static unsigned g_failures;
static uint64_t g_now_ms;
static bool g_snapshot_case;
static bool g_acquisition_age_case;
static uint32_t g_acc_acquired_us;
static unsigned g_delayed_mag_reads;
static bool g_attitude_events;
static bool g_hump_events;
static bool g_stationary_start;
static uint32_t g_new_reference;
static bool g_stationary_setdown;
static bool g_calibration_gap;
static bool g_repeated_loss;
static bool g_start_witness;
static bool g_witness_motion_read;
static unsigned g_imu_inits;
static unsigned g_event_count;

#define CHECK(cond)                                                    \
    do                                                                 \
    {                                                                  \
        if (!(cond))                                                   \
        {                                                              \
            (void)fprintf (stderr, "cycle %u: %s\n", g_cycle, #cond);    \
            g_failures++;                                              \
        }                                                              \
    } while (0)

ID tk_cre_cyc (const T_CCYC * p_cyclic)
{
    (void)p_cyclic;
    return 1;
}

ER tk_wup_tsk (ID task)
{
    (void)task;
    return E_OK;
}

INT tk_can_wup (ID task)
{
    (void)task;
    return (g_attitude_events && (403U == g_cycle)) ? 1 : 0;
}

ER tk_get_otm (SYSTIM * p_time)
{
    *p_time = (SYSTIM){(int32_t)(g_now_ms >> 32U), (uint32_t)g_now_ms};
    return E_OK;
}

ER telemetry_try_emit_event (uint8_t source, uint8_t type, int16_t p1,
                            uint16_t p2, uint32_t odometer_mm)
{
    CHECK (TELEMETRY_EVENT_SOURCE_PHYSICS == source);
    CHECK (odometer_mm >= 100U);
    if (g_attitude_events && (TELEMETRY_EVT_LIFT_START == type || TELEMETRY_EVT_LIFT_END == type ||
                             TELEMETRY_EVT_OVERTURN == type))
    {
        static uint8_t const expected[] = {TELEMETRY_EVT_LIFT_START, TELEMETRY_EVT_LIFT_END,
                                           TELEMETRY_EVT_OVERTURN};

        CHECK (g_event_count < sizeof (expected));
        CHECK ((g_event_count >= sizeof (expected)) || (expected[g_event_count] == type));
        if (0U == g_event_count)
        {
            CHECK (g_cycle >= 70U && g_cycle < 90U);
            CHECK (p1 >= 3680 && p1 <= 3690);
            CHECK (p2 >= 1199U && p2 <= 1201U);
        }
        else if (TELEMETRY_EVT_LIFT_END == type)
        {
            CHECK (g_cycle >= 145U && g_cycle < 160U);
            CHECK (p1 <= 500 && p2 == 1000U);
        }
        else
        {
            CHECK (g_cycle >= 195U && g_cycle < 225U);
            CHECK (p1 >= 17999 && p2 == 1000U);
        }
        g_event_count++;
    }
    else if (g_hump_events)
    {
        CHECK (g_event_count < 2U);
        if (0U == g_event_count)
        {
            CHECK (TELEMETRY_EVT_HUMP_START == type && p1 == 0 && p2 == 0U);
        }
        else
        {
            CHECK (TELEMETRY_EVT_HUMP_END == type && p1 > 0 && p2 == 1U);
        }
        g_event_count++;
    }
    /* One queue rejection must not make the same sample transition repeat. */
    return E_TMOUT;
}

static void check_snapshot (bool b_ready)
{
    physics_status_t status;

    CHECK (E_OK == physics_snapshot (&status));
    CHECK (b_ready == status.ready);
    CHECK (b_ready == status.calibrated);
    CHECK (b_ready != status.attitude_unknown);
    CHECK (b_ready == status.hump_active);
    CHECK (b_ready == (HUMP_STATE_FLAT != status.state));
    CHECK (b_ready == (0 != status.roll_cdeg));
}

static void check_publication_age (void)
{
    if (SNAPSHOT_FREEZE_CYCLE == g_cycle)
    {
        uint64_t const last_sample_ms = g_now_ms;

        check_snapshot (true);
        g_now_ms += SNAPSHOT_LOSS_MS - 1U;
        check_snapshot (true);
        g_now_ms++;
        check_snapshot (false);
        /* A sample older than a whole low-word epoch stays expired. */
        g_now_ms = last_sample_ms + (UINT64_C(1) << 32U) + 1U;
        check_snapshot (false);
    }
    else if (g_cycle > SNAPSHOT_FREEZE_CYCLE)
    {
        check_snapshot (true);
        longjmp (g_done, 1);
    }
}

ER tk_slp_tsk (TMO timeout)
{
    static bool const expected_fault[] = {true, false, true, true, false};
    physics_task_stats_t stats;
    hump_fields_t fields;

    (void)timeout;
    if (g_start_witness)
    {
        if (65U == g_cycle)
        {
            physics_status_t status;
            CHECK (E_OK == physics_snapshot (&status));
            CHECK (status.ready && status.attitude_sample_valid);
            CHECK (40 == status.forward_specific_mg);
            CHECK (65U * 20000U == status.forward_sample_us);
            CHECK (0U == (status.faults & PHYSICS_FAULT_STALE));
            longjmp (g_done, 1);
        }
        g_witness_motion_read = false;
        g_now_ms += SAMPLE_PERIOD_MS;
        g_cycle++;
        return E_OK;
    }
    if (g_repeated_loss)
    {
        physics_status_t status;
        CHECK (E_OK == physics_snapshot (&status));
        if (80U == g_cycle)
        {
            CHECK (status.calibrated && !status.attitude_unknown && 1U == g_imu_inits);
        }
        if (120U == g_cycle)
        {
            CHECK (status.attitude_unknown && 2U == g_imu_inits);
        }
        /* Allow both fifty-sample windows after the retry: gravity
           reference first, then confirmed level and still. */
        if (280U == g_cycle)
        {
            CHECK (status.calibrated && !status.attitude_unknown && status.level_still);
            CHECK (3U == g_imu_inits);
            longjmp (g_done, 1);
        }
        g_now_ms += SAMPLE_PERIOD_MS;
        g_cycle++;
        return E_OK;
    }
    if (g_stationary_start || g_stationary_setdown || g_calibration_gap)
    {
        physics_status_t status;
        CHECK (E_OK == physics_snapshot (&status));
        if (g_cycle == 5U)
        {
            CHECK (status.ready && !status.calibrated && status.attitude_unknown);
            CHECK (status.az_mg > 1000 && status.amag_mg > 1000U);
        }
        if (g_cycle == 55U)
        {
            CHECK (status.calibrated != g_calibration_gap);
            CHECK (status.attitude_unknown == g_calibration_gap);
            CHECK (status.collecting_rest && !status.level_still);
        }
        if (g_stationary_setdown && g_cycle == 72U)
        {
            CHECK (!status.collecting_rest && !status.level_still && status.rest_interruptions == 1U);
        }
        if (g_stationary_setdown && g_cycle == 90U)
        {
            CHECK (status.collecting_rest && !status.level_still && status.rest_interruptions > 0U);
        }
        if (g_calibration_gap && g_cycle == 85U)
        {
            CHECK (status.calibrated && !status.attitude_unknown);
            longjmp (g_done, 1);
        }
        if (g_cycle == 130U)
        {
            CHECK (status.calibrated && status.level_still && status.collecting_rest);
            if (g_stationary_setdown) longjmp (g_done, 1);
            uint32_t const first = physics_request_new_run ();
            g_new_reference = physics_request_new_run ();
            CHECK (first != g_new_reference);
            CHECK (E_OK == physics_snapshot (&status));
            CHECK (status.calibrated && status.reference_request == 0U);
        }
        if (g_cycle == 131U)
        {
            CHECK (status.reference_request == g_new_reference && !status.calibrated);
        }
        if (g_cycle == 200U)
        {
            CHECK (status.reference_request == g_new_reference && status.calibrated);
            longjmp (g_done, 1);
        }
        g_now_ms += SAMPLE_PERIOD_MS;
        g_cycle++;
        return E_OK;
    }
    if (g_attitude_events || g_hump_events)
    {
        if (g_attitude_events && g_cycle >= 95U && g_cycle <= 99U)
        {
            CHECK (0U == (system_fault_get () & FAULT_LIFTED_BIT));
            CHECK (1U == g_event_count);
            physics_status_t snapshot;
            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (snapshot.calibrated && !snapshot.attitude_sample_valid);
            CHECK (TELEMETRY_ATTITUDE_STATE_LIFTED == snapshot.attitude_state);
        }
        if (g_attitude_events && 180U == g_cycle)
        {
            CHECK (2U == g_event_count && !(system_fault_get () & FAULT_LIFTED_BIT));
            physics_status_t snapshot;
            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (TELEMETRY_ATTITUDE_STATE_OK == snapshot.attitude_state);
        }
        if (g_attitude_events && g_cycle >= 220U)
        {
            physics_status_t snapshot;
            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (snapshot.ready && !snapshot.attitude_unknown);
            CHECK (TELEMETRY_ATTITUDE_STATE_OVERTURNED == snapshot.attitude_state);
            /* The failed read, expired publication and skipped wake each
               require a fresh window of fifty level samples. */
            bool const b_settled = ((350U <= g_cycle) && (g_cycle < 353U)) ||
                                   (402U == g_cycle) || (452U <= g_cycle);
            CHECK (b_settled == snapshot.level_still);
            if (352U == g_cycle)
            {
                g_now_ms += SNAPSHOT_LOSS_MS;
                CHECK (E_OK == physics_snapshot (&snapshot));
                CHECK (!snapshot.ready && !snapshot.level_still);
            }
        }
        if (g_hump_events && ((112U == g_cycle) || (242U == g_cycle)))
        {
            physics_status_t snapshot;
            CHECK (E_OK == physics_snapshot (&snapshot));
            if (112U == g_cycle)
            {
                CHECK (snapshot.pitch_cdeg > 300 && abs(snapshot.roll_cdeg) < 100);
            }
            else
            {
                CHECK (snapshot.pitch_cdeg >= 12 && snapshot.pitch_cdeg <= 20 && snapshot.roll_cdeg < -500);
            }
        }
        if (g_hump_events && 246U == g_cycle)
        {
            physics_status_t snapshot;
            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (snapshot.roll_cdeg > 1000 && snapshot.report_roll_cdeg < 0);
        }
        if (g_cycle == (g_attitude_events ? 453U : 250U))
        {
            CHECK (g_event_count == (g_attitude_events ? 3U : 2U));
            longjmp (g_done, 1);
        }
        g_now_ms += (g_attitude_events && (402U == g_cycle)) ? 2U * SAMPLE_PERIOD_MS : SAMPLE_PERIOD_MS;
        g_cycle++;
        return E_OK;
    }
    if (g_snapshot_case)
    {
        check_publication_age ();
        g_now_ms += SAMPLE_PERIOD_MS;
        g_cycle++;
        return E_OK;
    }
    if (g_acquisition_age_case)
    {
        if (1U == g_cycle)
        {
            physics_status_t snapshot;

            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (snapshot.ready && (1U == g_delayed_mag_reads));
            CHECK (physics_time_us () > g_acc_acquired_us);
            CHECK (snapshot.forward_sample_us == g_acc_acquired_us);
            CHECK (0U == (snapshot.faults & PHYSICS_FAULT_I2C));
            CHECK (0U == (system_fault_get () & FAULT_IMU_NACK_BIT));
        }
        else if (3U == g_cycle)
        {
            physics_status_t snapshot;

            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (1U == g_delayed_mag_reads);
            CHECK (physics_time_us () > g_acc_acquired_us);
            CHECK (snapshot.forward_sample_us == g_acc_acquired_us);
            CHECK (!snapshot.attitude_sample_valid && !snapshot.heading_valid);
            CHECK (0U != (snapshot.faults & PHYSICS_FAULT_I2C));
            CHECK (0U != (system_fault_get () & FAULT_IMU_NACK_BIT));
        }
        else if (5U == g_cycle)
        {
            physics_status_t snapshot;

            CHECK (E_OK == physics_snapshot (&snapshot));
            CHECK (snapshot.ready && (2U == g_delayed_mag_reads));
            CHECK (physics_time_us () > g_acc_acquired_us);
            CHECK (snapshot.forward_sample_us == g_acc_acquired_us);
            CHECK (0U != (snapshot.faults & PHYSICS_FAULT_I2C));
            CHECK (0U != (system_fault_get () & FAULT_IMU_NACK_BIT));
            longjmp (g_done, 1);
        }
        CHECK (g_cycle < 6U);
        if (g_cycle >= 6U)
        {
            longjmp (g_done, 1);
        }
        g_now_ms += SAMPLE_PERIOD_MS;
        g_cycle++;
        return E_OK;
    }
    if (g_cycle > 0U)
    {
        CHECK (expected_fault[g_cycle - 1U] ==
               (0U != (system_fault_get () & FAULT_IMU_NACK_BIT)));
        CHECK (0U != (system_fault_sticky_get () & FAULT_IMU_NACK_BIT));
        physics_task_get_stats (&stats);
        CHECK (hump_unpack (stats.last_word, &fields));
        CHECK (0U != (fields.faults & PHYSICS_FAULT_I2C));
    }
    if (g_cycle == (sizeof (expected_fault) / sizeof (expected_fault[0])))
    {
        longjmp (g_done, 1);
    }
    g_now_ms += SAMPLE_PERIOD_MS;
    g_cycle++;
    return E_OK;
}

void i2c_rp2040_init (i2c_bus_t * p_bus)
{
    (void)p_bus;
}

uint32_t physics_time_us (void)
{
    if (g_acquisition_age_case)
    {
        return (uint32_t)g_now_ms * 1000U;
    }
    return g_cycle * 20000U + (g_witness_motion_read ? 500U : 0U);
}

bool motion_glue_read (motion_reading_t * p_reading)
{
    if (g_start_witness)
    {
        /* A previous cycle clock is 500 us behind this otherwise fresh sample. */
        g_witness_motion_read = true;
        int32_t const speed = g_cycle > 60U ? (int32_t)(g_cycle - 60U) * 10000 : 0;
        *p_reading = (motion_reading_t){.available = true, .timestamp_us = physics_time_us (),
            .velocity_um_s = {speed, speed}, .speed_valid = {true, true}, .odometry_valid = {true, true}};
        return true;
    }
    if (g_attitude_events || g_hump_events)
    {
        int32_t const speed = (g_hump_events && g_cycle >= 60U) ? 150000 : 0;
        uint32_t const travel_cycles = (g_cycle >= 60U) ? g_cycle - 60U : 0U;
        int64_t const distance = 100000 + ((int64_t)speed * travel_cycles * SAMPLE_PERIOD_MS) / 1000;

        /* A braked car can lose fresh wheel speed while retaining its odometer. */
        *p_reading = (motion_reading_t){.available = !g_attitude_events || g_cycle < 95U,
            .timestamp_us = physics_time_us (),
            .velocity_um_s = {speed, speed}, .displacement_um = {distance, distance},
            .speed_valid = {true, true}, .odometry_valid = {true, true}};
    }
    return g_attitude_events || g_hump_events;
}

lsm303_result_t lsm303_init (lsm303_t * p_dev, const i2c_bus_t * p_bus, uint8_t attempts)
{
    (void)p_bus;
    (void)attempts;
    g_imu_inits++;
    p_dev->ready = true;
    return LSM303_OK;
}

lsm303_result_t lsm303_check_config (lsm303_t * p_dev)
{
    (void)p_dev;
    return LSM303_OK;
}

/* Triangle in millimetres, sampled at axle and caster contacts. */
static float hump_ground_height (float x_mm)
{
    float const height = 30.0f - (0.3f * fabsf (x_mm - 100.0f));

    return fmaxf (height, 0.0f);
}

lsm303_result_t lsm303_read_accel (lsm303_t * p_dev, lsm303_vec_t * p_out)
{
    if (g_acquisition_age_case)
    {
        if (3U == g_cycle)
        {
            p_dev->fault = true;
            return LSM303_BUS_FAULT;
        }
        g_acc_acquired_us = physics_time_us ();
        *p_out = (lsm303_vec_t){.value = {0.0f, 0.0f, 1.0f}};
        return LSM303_OK;
    }
    if (g_start_witness)
    {
        *p_out = (lsm303_vec_t){.value = {0.0f, g_cycle > 60U ? -0.04f : 0.0f, 1.0f}};
        return LSM303_OK;
    }
    if (g_repeated_loss)
    {
        /* Configuration succeeds during the outage, but the data register
           only starts answering after the next reinitialization. */
        *p_out = (lsm303_vec_t){.value = {0.0f, 0.0f, 1.0f}};
        return ((g_cycle > 80U) && (g_imu_inits < 3U)) ? LSM303_BUS_FAULT : LSM303_OK;
    }
    if (g_stationary_start || g_stationary_setdown || g_calibration_gap)
    {
        /* Stationary magnitudes in successive archived telemetry frames.
         * Repeat this input at the physics task rate to reproduce the boundary. */
        static const float magnitudes[] = {1.043f, 1.057f, 1.038f, 1.041f, 1.045f};
        if (g_calibration_gap && g_cycle == 25U) return LSM303_BUS_FAULT;
        float const g = (g_stationary_setdown && g_cycle <= 52U)
            ? 1.043f : magnitudes[g_cycle % 5U];
        p_out->value[LSM303_AXIS_X] = -0.05234f * g;
        p_out->value[LSM303_AXIS_Y] = 0.0f;
        p_out->value[LSM303_AXIS_Z] = 0.99863f * g;
        if (g_stationary_setdown && g_cycle == 72U)
        {
            p_out->value[LSM303_AXIS_X] = -0.55f;
            p_out->value[LSM303_AXIS_Z] = 0.89f;
        }
        return LSM303_OK;
    }
    if (g_attitude_events)
    {
        *p_out = (lsm303_vec_t){0};
        if (g_cycle >= 60U && g_cycle < 95U)
        {
            p_out->value[LSM303_AXIS_X] = 0.72f;
            p_out->value[LSM303_AXIS_Z] = 0.96f;
        }
        else if (g_cycle >= 95U && g_cycle < 100U)
        {
            return LSM303_BUS_FAULT;
        }
        else if (g_cycle >= 220U)
        {
            if (g_cycle < 240U)
            {
                p_out->value[LSM303_AXIS_X] = 0.8f;
                p_out->value[LSM303_AXIS_Z] = 0.6f;
            }
            else if (g_cycle == 300U)
            {
                return LSM303_BUS_FAULT;
            }
            else
            {
                p_out->value[LSM303_AXIS_Z] = (g_cycle < 260U) ? 1.2f : 1.0f;
            }
        }
        else if (g_cycle < 160U || g_cycle >= 190U)
        {
            p_out->value[LSM303_AXIS_Z] = (g_cycle >= 190U) ? -1.0f : 1.0f;
        }
        return LSM303_OK;
    }
    if (g_hump_events)
    {
        float const axle = (float)CHASSIS_AXLE_TO_CASTER_MM;
        float const x = ((float)g_cycle - 85.0f) * 3.0f;
        float const slope = (hump_ground_height (x) - hump_ground_height (x - axle)) / axle;

        *p_out = (lsm303_vec_t){0};
        p_out->value[LSM303_AXIS_Y] = -slope;
        p_out->value[LSM303_AXIS_Z] = sqrtf (1.0f - (slope * slope));
        if (g_cycle >= 230U)
        {
            /* Left side raised with a small, independently known nose-up angle. */
            p_out->value[LSM303_AXIS_Y] = -0.0028f;
            p_out->value[LSM303_AXIS_X] = 246U == g_cycle ? -0.4f : 0.1f;
            p_out->value[LSM303_AXIS_Z] = sqrtf (0.99f);
        }
        return LSM303_OK;
    }
    if (g_snapshot_case && (g_cycle > 1U))
    {
        p_out->value[LSM303_AXIS_X] = 0.13f;
        p_out->value[LSM303_AXIS_Y] = 0.14f;
        p_out->value[LSM303_AXIS_Z] = 0.98f;
        return LSM303_OK;
    }
    if (!g_snapshot_case && (1U == g_cycle))
    {
        p_dev->fault = true;
        return LSM303_BUS_FAULT;
    }
    p_out->value[LSM303_AXIS_Z] = 1.0f;
    return LSM303_OK;
}

lsm303_result_t lsm303_read_mag (lsm303_t * p_dev, lsm303_vec_t * p_out)
{
    if (g_acquisition_age_case)
    {
        g_delayed_mag_reads++;
        g_now_ms += 7U;
        if (5U == g_cycle)
        {
            p_dev->fault = true;
            return LSM303_BUS_FAULT;
        }
        *p_out = (lsm303_vec_t){.value = {0.3f, 0.1f, 0.0f}};
        return LSM303_OK;
    }
    if (!g_snapshot_case && !g_attitude_events && !g_hump_events && (3U == g_cycle))
    {
        p_dev->fault = true;
        return LSM303_BUS_FAULT;
    }
    return LSM303_OK;
}

void lsm303_clear_fault (lsm303_t * p_dev)
{
    p_dev->fault = false;
}

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod, const char * p_format, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_format;
}

void tm_printf (const UB * p_format, ...)
{
    (void)p_format;
}

int main (int argc, char ** argv)
{
    physics_task_config_t cfg;

    physics_task_default_config (&cfg);
    cfg.mag_every = 2U;
    g_snapshot_case = (2 == argc) && (0 == strcmp (argv[1], "snapshot-expiry"));
    g_acquisition_age_case = (2 == argc) && (0 == strcmp (argv[1], "acceleration-acquisition-time"));
    g_attitude_events = (2 == argc) && (0 == strcmp (argv[1], "attitude-events"));
    g_hump_events = (2 == argc) && (0 == strcmp (argv[1], "hump-events"));
    g_stationary_start = (2 == argc) && (0 == strcmp (argv[1], "stationary-start"));
    g_stationary_setdown = (2 == argc) && (0 == strcmp (argv[1], "stationary-setdown"));
    g_calibration_gap = (2 == argc) && (0 == strcmp (argv[1], "calibration-gap"));
    g_repeated_loss = (2 == argc) && (0 == strcmp (argv[1], "repeated-read-loss"));
    g_start_witness = (2 == argc) && (0 == strcmp (argv[1], "start-witness"));
    if (g_snapshot_case)
    {
        cfg.hump.cal_samples = 1U;
        cfg.hump.enter_samples = 1U;
        /* The sample is published just before a low-word clock wrap. */
        g_now_ms = (uint64_t)UINT32_MAX - 200U;
    }
    if (g_acquisition_age_case)
    {
        cfg.motion.mag_enabled = true;
        g_now_ms = 200U;
    }
    if (E_OK != ((g_attitude_events || g_hump_events || g_start_witness)
                 ? physics_init (1) : physics_task_init (&cfg, 1)))
    {
        return 1;
    }
    if (0 == setjmp (g_done))
    {
        physics_task_run ();
    }
    return (0U == g_failures) ? 0 : 1;
}
