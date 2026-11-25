/** @file drivers_test.c
 *
 * @brief Host tests for the LSM303 protocol, the Buddy 2 adapter and the
 *        20 ms pipeline, against a simulated chip.
 *
 * The simulated chip follows the datasheet addressing rules: the
 * accelerometer auto-increments only when bit 7 of the sub-address is set,
 * and the magnetometer pointer always increments, wrapping 08 -> 03 and
 * >= 12 -> 00. It cannot model electrical behaviour or the real RP2040 I2C
 * controller (i2c_rp2040.c), which still needs testing on the car.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motion_adapter.h"
#include "pipeline.h"
#include "hump_msg.h"
#include "drv_lsm303.h"

#define DT_S        (0.02f)
#define SPEED_MPS   (0.15f)
#define DEG_TO_RAD  (0.0174532925f)

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            printf ("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

/* ---------------------------------------------------------------- chip */

typedef struct
{
    uint8_t acc[128];
    uint8_t mag[64];
    bool    acc_absent;          /* Always NACK 0x19.                      */
    bool    mag_absent;
    uint32_t fail_next;          /* Fail this many transactions.           */
    i2c_bus_result_t fail_kind;
    uint32_t recovers;
    uint32_t calls;
    uint8_t  last_acc_sub;
} chip_t;

static void chip_init (chip_t * p_c)
{
    memset (p_c, 0, sizeof (*p_c));
    p_c->mag[0x0A] = 0x48U;
    p_c->mag[0x0B] = 0x34U;
    p_c->mag[0x0C] = 0x33U;
    p_c->fail_kind = I2C_BUS_NACK;
}

static uint8_t mag_next (uint8_t ptr)
{
    uint8_t next;

    if (8U == ptr)
    {
        next = 3U;
    }
    else if (ptr >= 12U)
    {
        next = 0U;
    }
    else
    {
        next = (uint8_t)(ptr + 1U);
    }
    return next;
}

static i2c_bus_result_t chip_xfer (void * p_ctx, uint8_t addr7,
                                  const uint8_t * p_wr, uint32_t wr_len,
                                  uint8_t * p_rd, uint32_t rd_len)
{
    chip_t * p_c = (chip_t *)p_ctx;
    uint32_t i;

    p_c->calls++;
    if (p_c->fail_next > 0U)
    {
        p_c->fail_next--;
        return p_c->fail_kind;
    }
    if (wr_len < 1U)
    {
        return I2C_BUS_ERROR;
    }

    if (LSM303_ACC_ADDR == addr7)
    {
        uint8_t reg = (uint8_t)(p_wr[0] & 0x7FU);
        bool inc = (p_wr[0] & 0x80U) != 0U;

        if (p_c->acc_absent)
        {
            return I2C_BUS_NACK;
        }
        p_c->last_acc_sub = p_wr[0];
        for (i = 1U; i < wr_len; i++)
        {
            p_c->acc[reg] = p_wr[i];
            reg = inc ? (uint8_t)((reg + 1U) & 0x7FU) : reg;
        }
        for (i = 0U; i < rd_len; i++)
        {
            p_rd[i] = p_c->acc[reg];
            reg = inc ? (uint8_t)((reg + 1U) & 0x7FU) : reg;
        }
        return I2C_BUS_OK;
    }

    if (LSM303_MAG_ADDR == addr7)
    {
        uint8_t ptr = (uint8_t)(p_wr[0] & 0x3FU);

        if (p_c->mag_absent)
        {
            return I2C_BUS_NACK;
        }
        for (i = 1U; i < wr_len; i++)
        {
            p_c->mag[ptr] = p_wr[i];
            ptr = mag_next (ptr);
        }
        for (i = 0U; i < rd_len; i++)
        {
            p_rd[i] = p_c->mag[ptr];
            ptr = mag_next (ptr);
        }
        return I2C_BUS_OK;
    }

    return I2C_BUS_NACK;
}

static void chip_recover (void * p_ctx)
{
    ((chip_t *)p_ctx)->recovers++;
}

static i2c_bus_t chip_bus (chip_t * p_c)
{
    i2c_bus_t bus;

    bus.p_ctx = p_c;
    bus.write_read = chip_xfer;
    bus.recover = chip_recover;
    return bus;
}

/* Accelerometer value in mg as left-justified 12-bit little-endian. */
static void chip_set_acc (chip_t * p_c, int32_t x_mg, int32_t y_mg,
                          int32_t z_mg)
{
    int32_t v[3];
    uint32_t a;

    v[0] = x_mg;
    v[1] = y_mg;
    v[2] = z_mg;
    for (a = 0U; a < 3U; a++)
    {
        uint16_t pattern = (uint16_t)((uint32_t)(v[a] * 16) & 0xFFFFU);

        p_c->acc[0x28U + (2U * a)] = (uint8_t)(pattern & 0xFFU);
        p_c->acc[0x29U + (2U * a)] = (uint8_t)(pattern >> 8U);
    }
}

/* Magnetometer counts, big-endian, register order X, Z, Y. */
static void chip_set_mag (chip_t * p_c, int32_t x, int32_t y, int32_t z)
{
    uint16_t px = (uint16_t)((uint32_t)x & 0xFFFFU);
    uint16_t py = (uint16_t)((uint32_t)y & 0xFFFFU);
    uint16_t pz = (uint16_t)((uint32_t)z & 0xFFFFU);

    p_c->mag[3] = (uint8_t)(px >> 8U);
    p_c->mag[4] = (uint8_t)(px & 0xFFU);
    p_c->mag[5] = (uint8_t)(pz >> 8U);
    p_c->mag[6] = (uint8_t)(pz & 0xFFU);
    p_c->mag[7] = (uint8_t)(py >> 8U);
    p_c->mag[8] = (uint8_t)(py & 0xFFU);
}

/* ---------------------------------------------------------- lsm303 tests */

static void test_counts (void)
{
    printf ("count_conversions\n");
    CHECK (2047 == lsm303_accel_counts (0xF0U, 0x7FU));
    CHECK (-2048 == lsm303_accel_counts (0x00U, 0x80U));
    CHECK (-1 == lsm303_accel_counts (0xF0U, 0xFFU));
    CHECK (1000 == lsm303_accel_counts (0x80U, 0x3EU));
    CHECK (32767 == lsm303_mag_counts (0x7FU, 0xFFU));
    CHECK (-32768 == lsm303_mag_counts (0x80U, 0x00U));
    CHECK (-1 == lsm303_mag_counts (0xFFU, 0xFFU));
}

static void test_init_ok (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev = {0};

    printf ("init_writes_and_verifies_config\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    CHECK (dev.ready);
    CHECK (!dev.fault);
    CHECK (0x47U == chip.acc[0x20]);
    CHECK (0x88U == chip.acc[0x23]);
    CHECK (0x14U == chip.mag[0x00]);
    CHECK (0x20U == chip.mag[0x01]);
    CHECK (0x00U == chip.mag[0x02]);
    CHECK (0U == dev.stats.retries);
}

static void test_init_failures (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev = {0};

    printf ("init_failures\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_BAD_ARG == lsm303_init (NULL, &bus, 4U));
    CHECK (LSM303_BAD_ARG == lsm303_init (&dev, NULL, 4U));
    CHECK (LSM303_BAD_ARG == lsm303_init (&dev, &bus, 0U));
    CHECK (LSM303_BAD_ARG == lsm303_init (&dev, &bus, 9U));

    /* Wrong chip on 0x1E. */
    chip.mag[0x0B] = 0x00U;
    CHECK (LSM303_BAD_ID == lsm303_init (&dev, &bus, 4U));
    CHECK (!dev.ready);

    /* Accelerometer missing: 4 attempts on the first accel write, then
     * the fault is latched. */
    chip_init (&chip);
    chip.acc_absent = true;
    bus = chip_bus (&chip);
    CHECK (LSM303_BUS_FAULT == lsm303_init (&dev, &bus, 4U));
    CHECK (dev.fault);
    CHECK (!dev.ready);
    CHECK (3U == dev.stats.retries);
    CHECK (4U == dev.stats.nacks);
    CHECK (1U == dev.stats.failures);

    /* Nothing on the bus at all. */
    chip_init (&chip);
    chip.mag_absent = true;
    chip.acc_absent = true;
    bus = chip_bus (&chip);
    CHECK (LSM303_BUS_FAULT == lsm303_init (&dev, &bus, 4U));
}

static void test_reads (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev = {0};
    lsm303_vec_t v;

    printf ("burst_reads_and_axis_order\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_BAD_ARG == lsm303_read_accel (&dev, &v));  /* No init. */
    dev.ready = false;
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));

    chip_set_acc (&chip, -500, 250, 1000);
    CHECK (LSM303_OK == lsm303_read_accel (&dev, &v));
    CHECK (0xA8U == chip.last_acc_sub);       /* Auto-increment bit set. */
    CHECK ((-500 == v.raw[0]) && (250 == v.raw[1]) && (1000 == v.raw[2]));
    CHECK (fabsf (v.value[0] + 0.5f) < 1.0e-6f);
    CHECK (fabsf (v.value[2] - 1.0f) < 1.0e-6f);

    /* 0.3 G X, -0.1 G Y, -0.4 G Z. */
    chip_set_mag (&chip, 330, -110, -392);
    CHECK (LSM303_OK == lsm303_read_mag (&dev, &v));
    CHECK ((330 == v.raw[0]) && (-110 == v.raw[1]) && (-392 == v.raw[2]));
    CHECK (fabsf (v.value[0] - 0.3f) < 1.0e-4f);
    CHECK (fabsf (v.value[1] + 0.1f) < 1.0e-4f);
    CHECK (fabsf (v.value[2] + 0.4f) < 1.0e-4f);
}

static void test_retries (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev = {0};
    lsm303_vec_t v;

    printf ("retry_recover_and_latch\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    chip_set_acc (&chip, 0, 0, 1000);

    /* One glitch: retried, no fault. */
    chip.fail_next = 1U;
    CHECK (LSM303_OK == lsm303_read_accel (&dev, &v));
    CHECK (1U == dev.stats.retries);
    CHECK (!dev.fault);

    /* Timeouts trigger bus recovery before each retry. */
    chip.fail_kind = I2C_BUS_TIMEOUT;
    chip.fail_next = 2U;
    CHECK (LSM303_OK == lsm303_read_accel (&dev, &v));
    CHECK (2U == chip.recovers);
    CHECK (2U == dev.stats.timeouts);
    CHECK (!dev.fault);

    /* A controller error (lost arbitration) recovers the bus too. */
    chip.recovers = 0U;
    chip.fail_kind = I2C_BUS_ERROR;
    chip.fail_next = 1U;
    CHECK (LSM303_OK == lsm303_read_accel (&dev, &v));
    CHECK (1U == chip.recovers);
    CHECK (1U == dev.stats.errors);
    CHECK (!dev.fault);

    /* Four failures in a row: fault latched, reading continues. */
    chip.fail_kind = I2C_BUS_NACK;
    chip.fail_next = 4U;
    CHECK (LSM303_BUS_FAULT == lsm303_read_accel (&dev, &v));
    CHECK (dev.fault);
    CHECK (LSM303_OK == lsm303_read_accel (&dev, &v));
    CHECK (dev.fault);                         /* Still latched. */
    lsm303_clear_fault (&dev);
    CHECK (!dev.fault);
}

/* The counters span re-inits: a sensor initialised again keeps the totals of
 * the attempts before it. */
static void test_counters_survive_reinit (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev = {0};
    lsm303_vec_t v;

    printf ("counters_survive_reinit\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    chip.fail_kind = I2C_BUS_NACK;
    chip.fail_next = 4U;
    CHECK (LSM303_BUS_FAULT == lsm303_read_accel (&dev, &v));

    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    CHECK (dev.ready);
    CHECK (4U == dev.stats.nacks);
    CHECK (3U == dev.stats.retries);
    CHECK (1U == dev.stats.failures);
}

/* A sensor that reset keeps answering on the bus with its power-on registers.
 * The readback catches that and marks the sensor not ready, so init runs
 * again; a bus failure during the readback is not a configuration mismatch. */
static void test_config_readback (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev;

    printf ("config_readback_marks_a_reset_sensor_not_ready\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    CHECK (LSM303_OK == lsm303_check_config (&dev));
    CHECK (dev.ready);

    chip.acc[LSM303_CTRL_REG1_A] = 0x07U;
    chip.acc[LSM303_CTRL_REG4_A] = 0x00U;
    CHECK (LSM303_CONFIG_MISMATCH == lsm303_check_config (&dev));
    CHECK (!dev.ready);

    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    CHECK (dev.ready);
    CHECK (0x47U == chip.acc[LSM303_CTRL_REG1_A]);
    CHECK (LSM303_OK == lsm303_check_config (&dev));

    chip.fail_kind = I2C_BUS_NACK;
    chip.fail_next = 4U;
    CHECK (LSM303_BUS_FAULT == lsm303_check_config (&dev));
    CHECK (dev.ready);
}

/* --------------------------------------------------------- adapter tests */

static void reading_init (motion_reading_t * p_r, uint32_t ts, int32_t v_um_s,
                          int64_t disp_um)
{
    memset (p_r, 0, sizeof (*p_r));
    p_r->available = true;
    p_r->timestamp_us = ts;
    p_r->velocity_um_s[0] = v_um_s;
    p_r->velocity_um_s[1] = v_um_s;
    p_r->displacement_um[0] = disp_um;
    p_r->displacement_um[1] = disp_um;
    p_r->speed_valid[0] = true;
    p_r->speed_valid[1] = true;
    p_r->odometry_valid[0] = true;
    p_r->odometry_valid[1] = true;
}

static void test_adapter (void)
{
    motion_adapter_t ad;
    motion_reading_t r;
    motion_input_t m;

    printf ("buddy2_adapter_units_distance_and_dvdt\n");
    CHECK (!motion_adapter_init (&ad, 0.0f));
    CHECK (motion_adapter_init (&ad, 1.0f));

    /* Not running. */
    CHECK (motion_adapter_convert (&ad, NULL, 1000U, &m));
    CHECK (!m.valid);

    /* First sample: units, age, no distance yet. */
    reading_init (&r, 1000000U, 100000, 50000);
    r.velocity_um_s[1] = 120000;
    CHECK (motion_adapter_convert (&ad, &r, 1005000U, &m));
    CHECK (m.valid && m.wheels_valid);
    CHECK (fabsf (m.v_left_mps - 0.100f) < 1.0e-6f);
    CHECK (fabsf (m.v_right_mps - 0.120f) < 1.0e-6f);
    CHECK (fabsf (m.speed_mps - 0.110f) < 1.0e-6f);
    CHECK (fabsf (m.age_s - 0.005f) < 1.0e-6f);
    CHECK (fabsf (m.ds_m) < 1.0e-9f);

    /* 20 ms later: +3 mm left, +2 mm right, speed 0.11 -> 0.13 m/s. */
    reading_init (&r, 1020000U, 120000, 0);
    r.velocity_um_s[1] = 140000;
    r.displacement_um[0] = 53000;
    r.displacement_um[1] = 52000;
    CHECK (motion_adapter_convert (&ad, &r, 1021000U, &m));
    CHECK (fabsf (m.ds_m - 0.0025f) < 1.0e-6f);
    CHECK (fabsf (m.accel_mps2 - 1.0f) < 1.0e-3f);

    /* Same Buddy 2 sample read twice: no new distance, dv/dt unchanged. */
    CHECK (motion_adapter_convert (&ad, &r, 1030000U, &m));
    CHECK (fabsf (m.ds_m) < 1.0e-9f);
    CHECK (fabsf (m.accel_mps2 - 1.0f) < 1.0e-3f);
    CHECK (fabsf (m.age_s - 0.010f) < 1.0e-6f);

    /* Timer wrap: age still right. */
    reading_init (&r, 0xFFFFF000U, 130000, 60000);
    CHECK (motion_adapter_convert (&ad, &r, 0x00000800U, &m));
    CHECK (fabsf (m.age_s - 0.006144f) < 1.0e-6f);

    /* Odometry lost: invalid, and distance restarts from zero. */
    r.odometry_valid[1] = false;
    CHECK (motion_adapter_convert (&ad, &r, 0x00001000U, &m));
    CHECK (!m.valid);
    r.odometry_valid[1] = true;
    r.displacement_um[0] = 90000;
    r.displacement_um[1] = 90000;
    CHECK (motion_adapter_convert (&ad, &r, 0x00002000U, &m));
    CHECK (fabsf (m.ds_m) < 1.0e-9f);

    /* Timestamp ahead of now (should not happen): invalid. */
    reading_init (&r, 5000U, 100000, 0);
    CHECK (motion_adapter_convert (&ad, &r, 4000U, &m));
    CHECK (!m.valid);
}

/* The motion service samples every 10 ms and physics reads every 20 ms: an
 * invalid sample between two reads leaves both flags valid, but the count of
 * invalid samples moved. */
static void test_adapter_glitch_between_reads (void)
{
    motion_adapter_t ad;
    motion_reading_t r;
    motion_input_t m;

    printf ("adapter_sees_invalid_sample_between_reads\n");
    CHECK (motion_adapter_init (&ad, 1.0f));

    reading_init (&r, 1000000U, 100000, 50000);
    CHECK (motion_adapter_convert (&ad, &r, 1001000U, &m));
    CHECK (m.valid);

    reading_init (&r, 1020000U, 100000, 52000);
    r.invalid_samples[1] = 1U;
    CHECK (motion_adapter_convert (&ad, &r, 1021000U, &m));
    CHECK (!m.valid);
    CHECK (fabsf (m.ds_m) < 1.0e-9f);

    /* The count holds: the next read is clean, and the distance restarts. */
    reading_init (&r, 1040000U, 100000, 54000);
    r.invalid_samples[1] = 1U;
    CHECK (motion_adapter_convert (&ad, &r, 1041000U, &m));
    CHECK (m.valid);
    CHECK (fabsf (m.ds_m) < 1.0e-9f);
    reading_init (&r, 1060000U, 100000, 56000);
    r.invalid_samples[1] = 1U;
    CHECK (motion_adapter_convert (&ad, &r, 1061000U, &m));
    CHECK (m.valid);
    CHECK (fabsf (m.ds_m - 0.002f) < 1.0e-6f);

    /* Left wheel too. */
    reading_init (&r, 1080000U, 100000, 58000);
    r.invalid_samples[0] = 2U;
    r.invalid_samples[1] = 1U;
    CHECK (motion_adapter_convert (&ad, &r, 1081000U, &m));
    CHECK (!m.valid);
}

/* -------------------------------------------------------- pipeline tests */

typedef struct
{
    uint32_t nav;
    uint32_t telem;
    uint32_t cal_msgs;
    uint32_t start_msgs;
    uint32_t end_msgs;
    float    end_peak_m;
    bool     end_valid;
    uint32_t fault_msgs;
} pipe_result_t;

/* Calibrate (60 cycles at rest), then drive a 15 degree, 39-sample hump.
 * Accelerometer reads fail for fail_n cycles from cycle fail_at (counted
 * after calibration); fail_n = 0 for none. */
static pipe_result_t run_pipeline (uint32_t fail_at, uint32_t fail_n)
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipe_result_t res;
    uint32_t i;
    motion_class_t prev_motion = MOTION_CLASS_UNKNOWN;

    memset (&res, 0, sizeof (res));
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));

    for (i = 0U; i < (60U + 200U); i++)
    {
        float acc[3];
        float slope = 0.0f;
        motion_input_t m;
        pipeline_step_t out;
        hump_fields_t f;
        uint32_t t = (i >= 80U) ? (i - 80U) : 1000U;
        bool moving = (i >= 60U);
        bool fail = (i >= (60U + fail_at)) && (i < (60U + fail_at + fail_n));

        if (t < 39U)
        {
            slope = 15.0f;
        }
        else if ((t >= 49U) && (t < 88U))
        {
            slope = -15.0f;
        }
        else
        {
            /* Level. */
        }
        acc[0] = sinf (slope * DEG_TO_RAD);
        acc[1] = 0.0f;
        acc[2] = cosf (slope * DEG_TO_RAD);

        memset (&m, 0, sizeof (m));
        m.valid = true;
        m.wheels_valid = true;
        m.age_s = 0.005f;
        m.speed_mps = moving ? SPEED_MPS : 0.0f;
        m.v_left_mps = m.speed_mps;
        m.v_right_mps = m.speed_mps;
        m.ds_m = m.speed_mps * DT_S;

        CHECK (pipeline_step (&pipe, fail ? NULL : acc, NULL, &m, DT_S,
                                 false, &out));
        CHECK (hump_unpack (out.word, &f));
        CHECK (out.to_telem || !out.to_nav);   /* Nav implies Telemetry. */

        if (out.to_nav)
        {
            res.nav++;
            if ((MOTION_CLASS_UNKNOWN == prev_motion)
                && (MOTION_CLASS_UNKNOWN != f.motion))
            {
                res.cal_msgs++;
            }
            if (f.hump_active && out.sensors_ok
                && (HUMP_EVENT_STARTED == out.hump.event))
            {
                res.start_msgs++;
            }
            if (f.finished)
            {
                res.end_msgs++;
                res.end_peak_m = f.hump_peak_m;
                res.end_valid = f.height_valid;
            }
            if ((f.faults & PHYSICS_FAULT_I2C) != 0U)
            {
                res.fault_msgs++;
                CHECK (!f.height_valid);
                CHECK (!f.finished);
            }
        }
        if (out.to_telem)
        {
            res.telem++;
        }
        prev_motion = f.motion;
    }
    return res;
}

/* A lift is reported through the pipeline: the attitude state and the LIFTED
 * motion class in the packed word. A failed read does not change it. */
static void test_pipeline_lift (void)
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    hump_fields_t f;
    motion_input_t m;
    uint32_t i;
    float acc[3] = { 0.0f, 0.0f, 1.0f };
    bool saw_lifted = false;

    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));
    m.valid = true;
    m.wheels_valid = true;
    m.age_s = 0.005f;

    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (ATTITUDE_OK == out.attitude.state);
    CHECK (out.attitude.calibrated);

    /* Picked up and tilted past the envelope for 30 cycles. */
    acc[0] = sinf (30.0f * DEG_TO_RAD);
    acc[2] = cosf (30.0f * DEG_TO_RAD);
    for (i = 0U; i < 30U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
        saw_lifted = saw_lifted || (ATTITUDE_LIFTED == out.attitude.state);
    }
    CHECK (saw_lifted);
    CHECK (hump_unpack (out.word, &f));
    CHECK (MOTION_CLASS_LIFTED == f.motion);

    /* A new run starts from a fresh reference. */
    pipeline_new_run (&pipe);
    acc[0] = 0.0f;
    acc[2] = 1.0f;
    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    CHECK (ATTITUDE_OK == out.attitude.state);
    CHECK (!out.attitude.calibrated);
}

static void test_pipeline (void)
{
    pipe_result_t clean;
    pipe_result_t failed;
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    motion_input_t m;
    pipeline_step_t out;
    float ideal = 39.0f * SPEED_MPS * DT_S * sinf (15.0f * DEG_TO_RAD);

    printf ("pipeline_messages_and_failed_read\n");
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    memset (&m, 0, sizeof (m));
    CHECK (!pipeline_init (&pipe, &hcfg, &mcfg, 0U));
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    CHECK (!pipeline_step (&pipe, NULL, NULL, &m, 0.0f, false, &out));
    CHECK (!pipeline_step (&pipe, NULL, NULL, NULL, DT_S, false, &out));

    clean = run_pipeline (0U, 0U);
    printf ("  clean: nav %u, telem %u, peak %.0f mm (ideal %.1f)\n",
            (unsigned int)clean.nav, (unsigned int)clean.telem,
            (double)(clean.end_peak_m * 1000.0f), (double)(ideal * 1000.0f));
    CHECK (1U == clean.cal_msgs);
    CHECK (1U == clean.start_msgs);
    CHECK (1U == clean.end_msgs);
    CHECK (clean.end_valid);
    CHECK (fabsf (clean.end_peak_m - ideal) <= 0.0015f);
    CHECK (clean.nav < 20U);                 /* Events only, not 50 Hz. */
    CHECK (clean.telem >= 26U);              /* 260 cycles / 10.        */

    /* 10 failed reads (30 mm of travel, ~8 mm of height) during the climb:
     * one fault message, then the fault clears, and the carried distance
     * keeps the height. */
    failed = run_pipeline (25U, 10U);
    printf ("  10 failed reads: fault msgs %u, peak %.0f mm\n",
            (unsigned int)failed.fault_msgs,
            (double)(failed.end_peak_m * 1000.0f));
    CHECK (1U == failed.fault_msgs);
    CHECK (1U == failed.end_msgs);
    CHECK (failed.end_valid);
    CHECK (fabsf (failed.end_peak_m - clean.end_peak_m) <= 0.0015f);

    /* Driver fault flag is sent even when the read itself succeeded. */
    {
        float acc[3] = { 0.0f, 0.0f, 1.0f };
        hump_fields_t f;

        CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
        CHECK (out.to_nav);
        CHECK (hump_unpack (out.word, &f));
        CHECK ((f.faults & PHYSICS_FAULT_I2C) != 0U);
        CHECK ((f.faults & PHYSICS_FAULT_STALE) != 0U);   /* m.valid false. */
    }
}

/* Reads during a dead bus: the first failed cycle uses every attempt on the
 * accelerometer and skips the magnetometer; later cycles try each
 * transaction twice, so a cycle stays inside its 20 ms period. The first good
 * read brings the retries back. */
static void test_read_after_latch (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev;
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    motion_input_t m;
    lsm303_vec_t acc;
    lsm303_vec_t mag;
    bool acc_ok = true;
    bool mag_ok = true;
    uint32_t calls = 0U;
    uint32_t i;

    printf ("one_attempt_per_cycle_after_a_latch\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    chip_set_acc (&chip, 0, 0, 1000);
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));

    chip.fail_kind = I2C_BUS_TIMEOUT;
    chip.fail_next = 1000U;
    calls = chip.calls;
    pipeline_read (&pipe, &dev, 4U, true, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (!acc_ok && !mag_ok);
    CHECK (dev.fault);
    CHECK (4U == (chip.calls - calls));
    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, dev.fault, &out));

    for (i = 0U; i < 3U; i++)
    {
        calls = chip.calls;
        pipeline_read (&pipe, &dev, 4U, true, &acc, &mag, &acc_ok, &mag_ok);
        CHECK (!acc_ok && !mag_ok);
        CHECK (2U == (chip.calls - calls));
        CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, dev.fault, &out));
    }

    /* The bus comes back: one attempt each is enough. */
    chip.fail_next = 0U;
    calls = chip.calls;
    pipeline_read (&pipe, &dev, 4U, true, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (acc_ok && mag_ok);
    CHECK (2U == (chip.calls - calls));
    CHECK (pipeline_step (&pipe, acc.value, mag.value, &m, DT_S, dev.fault,
                          &out));

    /* After a good cycle a glitch is retried again. */
    chip.fail_next = 2U;
    pipeline_read (&pipe, &dev, 4U, false, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (acc_ok);
    CHECK (!mag_ok);
}

/* One glitch on the cycle after a failed cycle is absorbed by the second
 * attempt instead of costing that cycle's reading. */
static void test_glitch_after_failed_cycle_is_retried (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev;
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    motion_input_t m;
    lsm303_vec_t acc;
    lsm303_vec_t mag;
    bool acc_ok = true;
    bool mag_ok = true;

    printf ("one_glitch_after_a_failed_cycle_is_retried\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    chip_set_acc (&chip, 0, 0, 1000);
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));

    chip.fail_kind = I2C_BUS_NACK;
    chip.fail_next = 4U;
    pipeline_read (&pipe, &dev, 4U, false, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (!acc_ok);
    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, dev.fault, &out));

    chip.fail_next = 1U;
    pipeline_read (&pipe, &dev, 4U, false, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (acc_ok);
    CHECK (pipeline_step (&pipe, acc.value, NULL, &m, DT_S, dev.fault, &out));
}

/* A lasting read failure: after PIPELINE_LOST_CYCLES failed cycles the lift
 * no longer stands, the attitude is the not-valid default, the rest
 * reference is gone and the I2C fault stays in the word. */
static void test_pipeline_imu_lost (void)
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    hump_fields_t f;
    motion_input_t m;
    uint32_t i;
    float acc[3] = { 0.0f, 0.0f, 1.0f };

    printf ("lasting_read_failure_drops_the_attitude\n");
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));
    m.valid = true;
    m.wheels_valid = true;
    m.age_s = 0.005f;

    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (out.hump.calibrated);
    acc[0] = sinf (30.0f * DEG_TO_RAD);
    acc[2] = cosf (30.0f * DEG_TO_RAD);
    for (i = 0U; i < 30U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (ATTITUDE_LIFTED == out.attitude.state);

    /* A short failure keeps the last state. */
    for (i = 0U; i < (PIPELINE_LOST_CYCLES - 1U); i++)
    {
        CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
        CHECK (!out.imu_lost);
    }
    CHECK (hump_unpack (out.word, &f));
    CHECK (MOTION_CLASS_LIFTED == f.motion);

    /* One more: the IMU is lost. */
    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    CHECK (out.imu_lost);
    CHECK (!out.sensors_ok);
    CHECK (ATTITUDE_OK == out.attitude.state);
    CHECK (!out.attitude.calibrated);
    CHECK (out.attitude.amag_g < 0.5f);
    CHECK (hump_unpack (out.word, &f));
    CHECK (MOTION_CLASS_UNKNOWN == f.motion);
    CHECK (HUMP_STATE_FLAT == f.state);
    CHECK (!f.hump_active);
    CHECK ((f.faults & PHYSICS_FAULT_I2C) != 0U);

    /* Reads come back on level ground with the driver fault still latched:
     * no lift, no rest reference until the car stands still, and the I2C
     * fault still shows. */
    acc[0] = 0.0f;
    acc[2] = 1.0f;
    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
    CHECK (out.sensors_ok);
    CHECK (!out.imu_lost);
    CHECK (ATTITUDE_OK == out.attitude.state);
    CHECK (!out.hump.calibrated);
    CHECK (hump_unpack (out.word, &f));
    CHECK ((f.faults & PHYSICS_FAULT_I2C) != 0U);
    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
    }
    CHECK (out.hump.calibrated);
}

/* A lost IMU leaves the attitude unknown until the rest reference has been
 * measured again. A short failure keeps the attitude known. */
static void test_lost_attitude_unknown (void)
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    motion_input_t m;
    uint32_t i;
    float acc[3] = { 0.0f, 0.0f, 1.0f };

    printf ("attitude_unknown_while_lost_and_until_recalibrated\n");
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));
    m.valid = true;
    m.wheels_valid = true;
    m.age_s = 0.005f;

    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (out.hump.calibrated);
    CHECK (!out.attitude_unknown);

    for (i = 0U; i < (PIPELINE_LOST_CYCLES - 1U); i++)
    {
        CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    }
    CHECK (!out.attitude_unknown);

    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    CHECK (out.imu_lost);
    CHECK (out.attitude_unknown);
    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    CHECK (out.attitude_unknown);

    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
    CHECK (!out.imu_lost);
    CHECK (!out.hump.calibrated);
    CHECK (out.attitude_unknown);

    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
    }
    CHECK (out.hump.calibrated);
    CHECK (!out.attitude_unknown);
}

/* The configuration readback runs on due cycles only: a ready sensor, a cycle
 * that is a multiple of the period, and a previous cycle whose accelerometer
 * read succeeded. A dead bus is given the re-init attempt count, and a
 * mismatch is reported so the sensor is initialised again. */
static void test_config_readback_budget (void)
{
    chip_t chip;
    i2c_bus_t bus;
    lsm303_t dev = {0};
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    motion_input_t m;
    lsm303_vec_t acc;
    lsm303_vec_t mag;
    bool acc_ok = false;
    bool mag_ok = false;
    uint32_t calls = 0U;

    printf ("config_readback_budget_and_skip_after_failure\n");
    chip_init (&chip);
    bus = chip_bus (&chip);
    CHECK (LSM303_OK == lsm303_init (&dev, &bus, 4U));
    chip_set_acc (&chip, 0, 0, 1000);
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));

    pipeline_read (&pipe, &dev, 4U, false, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (acc_ok);
    CHECK (pipeline_step (&pipe, acc.value, NULL, &m, DT_S, dev.fault, &out));

    calls = chip.calls;
    CHECK (!pipeline_config_readback (&pipe, &dev, 1U, 100U, 2U));
    CHECK (0U == (chip.calls - calls));

    calls = chip.calls;
    CHECK (!pipeline_config_readback (&pipe, &dev, 100U, 100U, 2U));
    CHECK (2U == (chip.calls - calls));

    chip.fail_kind = I2C_BUS_TIMEOUT;
    chip.fail_next = 1000U;
    calls = chip.calls;
    CHECK (!pipeline_config_readback (&pipe, &dev, 200U, 100U, 2U));
    CHECK (2U == (chip.calls - calls));
    CHECK (dev.ready);

    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, dev.fault, &out));
    calls = chip.calls;
    CHECK (!pipeline_config_readback (&pipe, &dev, 300U, 100U, 2U));
    CHECK (0U == (chip.calls - calls));

    chip.fail_next = 0U;
    pipeline_read (&pipe, &dev, 4U, false, &acc, &mag, &acc_ok, &mag_ok);
    CHECK (acc_ok);
    CHECK (pipeline_step (&pipe, acc.value, NULL, &m, DT_S, dev.fault, &out));

    chip.acc[LSM303_CTRL_REG1_A] = 0x07U;
    CHECK (pipeline_config_readback (&pipe, &dev, 400U, 100U, 2U));
    CHECK (!dev.ready);
    calls = chip.calls;
    CHECK (!pipeline_config_readback (&pipe, &dev, 500U, 100U, 2U));
    CHECK (0U == (chip.calls - calls));
}

/* The lost edges come once per loss: the cycle the loss starts, and the first
 * read that comes back. A short failure is not a loss. */
static void test_imu_lost_edges (void)
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    motion_input_t m;
    uint32_t i;
    float acc[3] = { 0.0f, 0.0f, 1.0f };

    printf ("imu_lost_edges_once_per_loss\n");
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));
    m.valid = true;
    m.wheels_valid = true;
    m.age_s = 0.005f;

    for (i = 0U; i < 3U; i++)
    {
        CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    }
    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    CHECK (!out.imu_recovered);
    CHECK (!out.imu_lost_started);

    for (i = 0U; i < (PIPELINE_LOST_CYCLES - 1U); i++)
    {
        CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
        CHECK (!out.imu_lost_started);
    }
    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    CHECK (out.imu_lost);
    CHECK (out.imu_lost_started);
    CHECK (!out.imu_recovered);

    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    CHECK (out.imu_lost);
    CHECK (!out.imu_lost_started);

    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
    CHECK (!out.imu_lost);
    CHECK (out.imu_recovered);
    CHECK (!out.imu_lost_started);

    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, true, &out));
    CHECK (!out.imu_recovered);
}

/* The attitude fields are published only on a cycle whose sensors were read
 * and whose rest reference is set. */
static void test_attitude_publish_gate (void)
{
    pipeline_step_t step;

    printf ("attitude_published_only_when_read_and_referenced\n");
    memset (&step, 0, sizeof (step));
    CHECK (!pipeline_attitude_known (NULL));
    CHECK (!pipeline_attitude_known (&step));
    step.sensors_ok = true;
    step.attitude_unknown = true;
    CHECK (!pipeline_attitude_known (&step));
    step.attitude_unknown = false;
    CHECK (pipeline_attitude_known (&step));
    step.sensors_ok = false;
    CHECK (!pipeline_attitude_known (&step));
}

/* The attitude reads unknown until a rest reference is set: at boot and after
 * a new run. A failed read keeps the reference, so the attitude stays known. */
static void test_attitude_unknown_without_reference (void)
{
    hump_config_t hcfg;
    imu_motion_config_t mcfg;
    pipeline_t pipe;
    pipeline_step_t out;
    motion_input_t m;
    uint32_t i;
    float acc[3] = { 0.0f, 0.0f, 1.0f };

    printf ("attitude_unknown_without_a_rest_reference\n");
    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));
    m.valid = true;
    m.wheels_valid = true;
    m.age_s = 0.005f;

    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    CHECK (!out.calibrated);
    CHECK (out.attitude_unknown);

    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (out.calibrated);
    CHECK (!out.attitude_unknown);

    CHECK (pipeline_step (&pipe, NULL, NULL, &m, DT_S, true, &out));
    CHECK (out.calibrated);
    CHECK (!out.attitude_unknown);

    pipeline_new_run (&pipe);
    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    CHECK (!out.calibrated);
    CHECK (out.attitude_unknown);
    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (out.calibrated);
    CHECK (!out.attitude_unknown);
}

/* ------------------------------------------------ lift cue and the hump */

typedef struct
{
    bool on_hump;
    bool lifted;
} shake_result_t;

static void pipe_rest_with (pipeline_t * p_pipe, const hump_config_t * p_hcfg)
{
    hump_config_t hcfg = *p_hcfg;
    imu_motion_config_t mcfg;
    pipeline_step_t out;
    motion_input_t m;
    float acc[3] = { 0.0f, 0.0f, 1.0f };
    uint32_t i;

    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (p_pipe, &hcfg, &mcfg, 10U));
    memset (&m, 0, sizeof (m));
    m.valid = true;
    m.wheels_valid = true;
    m.age_s = 0.005f;
    for (i = 0U; i < 60U; i++)
    {
        CHECK (pipeline_step (p_pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (out.attitude.calibrated);
}

static void pipe_at_rest (pipeline_t * p_pipe)
{
    hump_config_t hcfg;

    hump_default_config (&hcfg);
    pipe_rest_with (p_pipe, &hcfg);
}

/* The car runs 4 deg to enter and a 2 deg exit band (physics_car_config). */
#define CAR_ENTER_DEG (4.0f)

static void pipe_car_at_rest (pipeline_t * p_pipe)
{
    hump_config_t hcfg;

    hump_default_config (&hcfg);
    hcfg.enter_deg = CAR_ENTER_DEG;
    hcfg.exit_band_deg = 2.0f;
    pipe_rest_with (p_pipe, &hcfg);
}

/* Inject sustained upward load with ripple to exercise the hump mask. */
static shake_result_t apply_load_with_motion (pipeline_t * p_pipe, uint32_t cycles,
                                        float pitch_deg, const motion_input_t * p_motion,
                                        pipeline_step_t * p_out)
{
    shake_result_t res = { false, false };
    uint32_t i;

    for (i = 0U; i < cycles; i++)
    {
        float const mag = ((i % 2U) == 0U) ? 1.21f : 1.25f;
        float acc[3];

        acc[0] = mag * sinf (pitch_deg * DEG_TO_RAD);
        acc[1] = 0.0f;
        acc[2] = mag * cosf (pitch_deg * DEG_TO_RAD);
        CHECK (pipeline_step (p_pipe, acc, NULL, p_motion, DT_S, false, p_out));
        res.on_hump = res.on_hump || (HUMP_STATE_FLAT != p_out->hump.state);
        res.lifted = res.lifted || (ATTITUDE_LIFTED == p_out->attitude.state);
    }
    return res;
}

static motion_input_t forward_motion (float speed_mps)
{
    return (motion_input_t){ .valid = true, .wheels_valid = true, .age_s = 0.005f,
                             .speed_mps = speed_mps, .ds_m = speed_mps * DT_S,
                             .v_left_mps = speed_mps, .v_right_mps = speed_mps };
}

/* Production mounting has forward along -Y and left along +X. */
static void step_car_attitude (pipeline_t * p_pipe, const motion_input_t * p_motion,
                               float pitch_deg, float roll_deg, float magnitude,
                               pipeline_step_t * p_out)
{
    float const pitch = pitch_deg * DEG_TO_RAD;
    float const roll = roll_deg * DEG_TO_RAD;
    float const yz = magnitude * cosf (pitch);
    float const accel[3] = { -yz * sinf (roll), -magnitude * sinf (pitch),
                             yz * cosf (roll) };

    CHECK (pipeline_step (p_pipe, accel, NULL, p_motion, DT_S, false, p_out));
    CHECK (p_out->sensors_ok && p_out->calibrated && !p_out->attitude_unknown);
}

static void pipe_mounted_car_at_rest (pipeline_t * p_pipe)
{
    hump_config_t hcfg;

    hump_default_config (&hcfg);
    hcfg.fwd_axis = HUMP_AXIS_Y;
    hcfg.fwd_sign = -1;
    hcfg.enter_deg = 3.0f;
    hcfg.exit_band_deg = 2.0f;
    pipe_rest_with (p_pipe, &hcfg);
}

static void test_pipeline_lift_settled_gap (void)
{
    motion_input_t motion = forward_motion (0.118f);

    motion.v_left_mps = 0.122f;
    motion.v_right_mps = 0.114f;
    for (uint32_t wheel_cue = 0U; wheel_cue < 2U; wheel_cue++)
    {
        pipeline_t pipe;
        pipeline_step_t out;
        bool lifted = false;
        bool started_hump = false;

        pipe_mounted_car_at_rest (&pipe);
        pipe.wheels_free = 0U != wheel_cue;
        for (uint32_t i = 0U; i < 20U; i++)
        {
            bool const settled = (i >= 8U) && (i < 12U);

            step_car_attitude (&pipe, &motion, settled ? 0.0f : -6.5f,
                               settled ? 0.0f : 19.73f, settled ? 1.0f : 1.044f,
                               &out);
            lifted |= (ATTITUDE_LIFTED == out.attitude.state);
            CHECK (HUMP_STATE_FLAT == out.hump.state);
            if (settled && (0U != wheel_cue))
            {
                CHECK (ATTITUDE_CANDIDATE == out.attitude.state);
            }
        }
        CHECK (!lifted);
        for (uint32_t i = 0U; i < 10U; i++)
        {
            step_car_attitude (&pipe, &motion, 8.0f, 0.0f, 1.0f, &out);
            started_hump |= (HUMP_EVENT_STARTED == out.hump.event);
        }
        CHECK (started_hump);
    }
}

static void test_pipeline_sustained_lift_cues (void)
{
    motion_input_t const forward = forward_motion (0.118f);
    motion_input_t const stationary = forward_motion (0.0f);
    pipeline_t pipe;
    pipeline_step_t out;

    pipe_mounted_car_at_rest (&pipe);
    for (unsigned idx = 0U; idx < 8U; idx++)
    {
        step_car_attitude (&pipe, &forward, -6.5f, 19.73f, 1.044f, &out);
    }
    for (unsigned idx = 0U; idx < 20U; idx++)
    {
        CHECK (pipeline_step (&pipe, NULL, NULL, &forward, DT_S, true, &out));
    }
    for (unsigned idx = 0U; idx < 15U; idx++)
    {
        step_car_attitude (&pipe, &forward, -6.5f, 19.73f, 1.044f, &out);
        CHECK ((idx == 14U) == (ATTITUDE_LIFTED == out.attitude.state));
    }

    pipe_mounted_car_at_rest (&pipe);
    for (uint32_t i = 0U; i < 15U; i++)
    {
        /* A real combined lean still counts even if forward wheels turn and
         * the pitch detector labels the profile a hump. */
        step_car_attitude (&pipe, &forward, 6.5f, 19.73f, 1.044f, &out);
        CHECK (out.attitude.tilt_deg > 20.0f);
        CHECK ((i == 14U) == (ATTITUDE_LIFTED == out.attitude.state));
    }
    CHECK (HUMP_STATE_CLIMB == out.hump.state);

    pipe_mounted_car_at_rest (&pipe);
    for (uint32_t i = 0U; i < 17U; i++)
    {
        step_car_attitude (&pipe, &stationary, 0.0f, 0.0f, 0.0f, &out);
        CHECK (ATTITUDE_LIFTED != out.attitude.state);
    }

    /* Small upward changes are usable only with two fresh stationary wheels.
     * A pivot has zero mean speed too. */
    for (unsigned mode = 0U; mode < 4U; ++mode)
    {
        motion_input_t wheels = stationary;
        pipe_mounted_car_at_rest (&pipe);
        if (mode == 1U) wheels.age_s = 1.0f;
        if (mode == 2U) wheels.wheels_valid = false;
        if (mode == 3U) { wheels.v_left_mps = -0.10f; wheels.v_right_mps = 0.10f; }
        for (unsigned i = 0U; i < 8U; ++i)
        {
            step_car_attitude (&pipe, &wheels, 0.0f, 0.0f, 1.08f, &out);
            CHECK ((mode == 0U && i >= 4U) == (ATTITUDE_LIFTED == out.attitude.state));
        }
    }

    pipe_mounted_car_at_rest (&pipe);
    for (uint32_t i = 0U; i < 17U; i++)
    {
        /* The first short pulse ends. The second must qualify on its own. */
        bool const between = (i >= 3U) && (i < 7U);

        step_car_attitude (&pipe, &stationary, 0.0f, 0.0f,
                           between ? 1.0f : 1.32f, &out);
        CHECK ((i >= 11U) == (ATTITUDE_LIFTED == out.attitude.state));
    }
    CHECK (ATTITUDE_LIFTED == out.attitude.state);

    pipe_mounted_car_at_rest (&pipe);
    for (uint32_t i = 0U; i < 10U; i++)
    {
        step_car_attitude (&pipe, &forward, 6.5f, 70.0f, 1.0f, &out);
    }
    CHECK (ATTITUDE_OVERTURNED == out.attitude.state);
}

static shake_result_t apply_load (pipeline_t * p_pipe, uint32_t cycles,
                             float pitch_deg, float speed_mps, pipeline_step_t * p_out)
{
    motion_input_t const motion = forward_motion (speed_mps);

    return apply_load_with_motion (p_pipe, cycles, pitch_deg, &motion, p_out);
}

/* Bumps on a hump are not a lift while the car drives over it. The mask
 * follows the hump state: without it the same bumps lift the car. */
static void test_pipeline_hump_mutes_bumps (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    shake_result_t res;

    printf ("pipeline_hump_state_masks_unsettled_cue\n");
    pipe_at_rest (&pipe);
    res = apply_load (&pipe, 80U, 0.0f, SPEED_MPS, &out);
    CHECK (!res.on_hump);
    CHECK (res.lifted);

    pipe_at_rest (&pipe);
    res = apply_load (&pipe, 80U, 8.0f, SPEED_MPS, &out);
    CHECK (res.on_hump);
    CHECK (HUMP_STATE_FLAT != out.hump.state);
    CHECK (!res.lifted);
    CHECK (ATTITUDE_LIFTED != out.attitude.state);
}

static void test_pipeline_slow_grounded_hump (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    shake_result_t res;
    hump_fields_t fields;

    pipe_car_at_rest (&pipe);
    res = apply_load (&pipe, 24U, 8.0f, SPEED_MPS, &out);
    CHECK (res.on_hump && !res.lifted);
    res = apply_load (&pipe, 20U, 8.0f, 0.06f, &out);
    CHECK (res.on_hump && (HUMP_STATE_FLAT != out.hump.state));
    CHECK (!out.hump_rejected && !out.hump.speed_ok && !out.hump.motion_stale);
    CHECK (!out.hump.height_valid && !res.lifted);
    CHECK (hump_unpack (out.word, &fields));
    CHECK (MOTION_CLASS_LIFTED != fields.motion);

    res = apply_load (&pipe, 20U, 8.0f, 0.0f, &out);
    CHECK (res.on_hump && res.lifted && (ATTITUDE_LIFTED == out.attitude.state));
    CHECK (hump_unpack (out.word, &fields));
    CHECK (MOTION_CLASS_LIFTED == fields.motion);
}

static void test_pipeline_hump_requires_forward_travel (void)
{
    motion_input_t motion[] = { forward_motion (0.06f), forward_motion (0.06f),
                               forward_motion (0.06f), forward_motion (-0.06f) };

    motion[0].valid = false;
    motion[1].age_s = 1.0f;
    motion[2].ds_m = 0.0f;
    for (size_t i = 0U; i < (sizeof (motion) / sizeof (motion[0])); i++)
    {
        pipeline_t pipe;
        pipeline_step_t out;
        shake_result_t res;

        pipe_car_at_rest (&pipe);
        res = apply_load (&pipe, 24U, 8.0f, SPEED_MPS, &out);
        CHECK (res.on_hump && !res.lifted);
        res = apply_load_with_motion (&pipe, 20U, 8.0f, &motion[i], &out);
        CHECK (res.on_hump && res.lifted && (ATTITUDE_LIFTED == out.attitude.state));
    }
}

static void test_pipeline_slow_hump_ends_at_travel_cap (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    shake_result_t res;

    pipe_car_at_rest (&pipe);
    res = apply_load (&pipe, 24U, 8.0f, SPEED_MPS, &out);
    CHECK (res.on_hump && !res.lifted);
    /* This travel stays below the cap even if all initial samples counted. */
    res = apply_load (&pipe, 300U, 8.0f, 0.06f, &out);
    CHECK (res.on_hump && !res.lifted);
    res = apply_load (&pipe, 100U, 8.0f, 0.06f, &out);
    CHECK (HUMP_STATE_FLAT != out.hump.state);
    CHECK (!out.hump_rejected && !out.hump.speed_ok && res.lifted);
}

/* A car lifted nose-up after stopping on a climb must still report the lift. */
static void test_pipeline_lift_at_rest_on_hump_state (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    shake_result_t res;
    hump_fields_t f;

    printf ("pipeline_lift_nose_up_at_rest_is_lifted\n");
    pipe_at_rest (&pipe);
    res = apply_load (&pipe, 24U, 8.0f, SPEED_MPS, &out);
    CHECK (res.on_hump && !res.lifted);
    res = apply_load (&pipe, 60U, 8.0f, 0.0f, &out);
    CHECK (res.on_hump);
    CHECK (res.lifted);
    CHECK (ATTITUDE_LIFTED == out.attitude.state);
    CHECK (hump_unpack (out.word, &f));
    CHECK (MOTION_CLASS_LIFTED == f.motion);
}

/* A lifted car whose wheels spin free: the wheels turn on target, so the
 * speed reads as driving, but the free-wheel cue says it is not on the ground.
 * The hump mask holds while the hump state does, then it ends: here the
 * integrated height passes its limit, so the tilt is not a valid hump. */
static void test_pipeline_wheels_free_on_hump_state (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    shake_result_t res;

    printf ("pipeline_wheels_free_on_hump_state_is_lifted\n");
    pipe_at_rest (&pipe);
    pipe.wheels_free = true;
    res = apply_load (&pipe, 60U, 8.0f, SPEED_MPS, &out);
    CHECK (res.on_hump);
    CHECK (!res.lifted);
    res = apply_load (&pipe, 140U, 8.0f, SPEED_MPS, &out);
    CHECK (res.lifted);
    CHECK (ATTITUDE_LIFTED == out.attitude.state);
}

/* A hump state that never ends, under both limits: a 4.5 deg tilt gives only
 * 38 mm of height over the 480 mm cap. The mask ends at the travel cap. */
static void test_pipeline_lasting_hump_state_ends_at_cap (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    shake_result_t res;
    uint32_t const cap_cycles =
        (uint32_t)(((float)PIPELINE_HUMP_MASK_MM * 0.001f)
                   / (SPEED_MPS * DT_S));

    printf ("pipeline_lasting_hump_state_ends_at_travel_cap\n");
    pipe_car_at_rest (&pipe);
    pipe.wheels_free = true;
    res = apply_load (&pipe, cap_cycles - 20U, 4.5f, SPEED_MPS, &out);
    CHECK (res.on_hump);
    CHECK (!res.lifted);
    CHECK (HUMP_STATE_FLAT != out.hump.state);
    res = apply_load (&pipe, 60U, 4.5f, SPEED_MPS, &out);
    CHECK (res.lifted);
    CHECK (ATTITUDE_LIFTED == out.attitude.state);
}

/* ------------------------------------------------------- hump profile */

#define MM_TO_M    (0.001f)
#define AXLE_M     ((float)CHASSIS_AXLE_TO_CASTER_MM * MM_TO_M)
#define RAMP_M     ((float)PIPELINE_HUMP_RAMP_MM * MM_TO_M)

typedef struct
{
    float height_m;
    float ramp_m;
} profile_t;

/* Ground height under a contact at distance x_m along the test hump. */
static float ground_m (const profile_t * p_prof, float x_m)
{
    float h = 0.0f;

    if ((x_m > 0.0f) && (x_m < p_prof->ramp_m))
    {
        h = p_prof->height_m * (x_m / p_prof->ramp_m);
    }
    else if ((x_m >= p_prof->ramp_m) && (x_m < (2.0f * p_prof->ramp_m)))
    {
        h = p_prof->height_m
            * (((2.0f * p_prof->ramp_m) - x_m) / p_prof->ramp_m);
    }
    else
    {
        /* Flat floor. */
    }
    return h;
}

typedef struct
{
    bool     lifted;
    bool     bridge_seen;      /* The follower's hump flag was up.        */
    bool     bridge_at_end;
    bool     bridge_after_reject; /* The flag was up from the rejection on. */
    uint32_t rejected;         /* Cycles with hump_rejected set.          */
    uint32_t bridge_cycles;
    uint32_t last_bridge;      /* Cycle of the last bridge.               */
} drive_result_t;

typedef struct
{
    profile_t prof;
    float     speed_mps;
    bool      bumps;           /* 1.3 g / 0.7 g swing while pitched past
                                  the angle a hump is entered at.          */
    uint32_t  free_from_x_mm;  /* wheels_free from this travel, 0: never. */
} drive_t;

/* The follower and the supervisor read the hump flag from the word. */
static bool follower_bridge (packed_hump_t word)
{
    hump_fields_t f;

    CHECK (hump_unpack (word, &f));
    return f.hump_active || (HUMP_STATE_FLAT != f.state);
}

/* Drive the two-contact car over the profile: axle at x, caster at x - axle,
 * pitch = asin ((h(axle) - h(caster)) / axle). Level floor for 25 cycles
 * before and 60 after. Nothing here depends on the hump height or ramps. */
static drive_result_t drive_hump (pipeline_t * p_pipe, const drive_t * p_drive,
                                  pipeline_step_t * p_out)
{
    drive_result_t res;
    float const ds = p_drive->speed_mps * DT_S;
    float const end_m = (2.0f * p_drive->prof.ramp_m) + AXLE_M;
    float x = -25.0f * ds;
    uint32_t i = 0U;

    memset (&res, 0, sizeof (res));
    while (x < (end_m + (60.0f * ds)))
    {
        float const dh = ground_m (&p_drive->prof, x)
                         - ground_m (&p_drive->prof, x - AXLE_M);
        float const pitch = asinf (dh / AXLE_M);
        bool const on = p_drive->bumps
                        && (fabsf (pitch) > (CAR_ENTER_DEG * DEG_TO_RAD));
        float const mag = on ? (((i % 2U) == 0U) ? 1.3f : 0.7f) : 1.0f;
        float acc[3];
        motion_input_t m;
        bool bridge = false;

        acc[0] = mag * sinf (pitch);
        acc[1] = 0.0f;
        acc[2] = mag * cosf (pitch);
        memset (&m, 0, sizeof (m));
        m.valid = true;
        m.wheels_valid = true;
        m.age_s = 0.005f;
        m.speed_mps = p_drive->speed_mps;
        m.v_left_mps = p_drive->speed_mps;
        m.v_right_mps = p_drive->speed_mps;
        m.ds_m = ds;
        p_pipe->wheels_free = (p_drive->free_from_x_mm > 0U)
            && (x >= ((float)p_drive->free_from_x_mm * MM_TO_M));
        CHECK (pipeline_step (p_pipe, acc, NULL, &m, DT_S, false, p_out));
        bridge = follower_bridge (p_out->word);
        res.lifted = res.lifted || (ATTITUDE_LIFTED == p_out->attitude.state);
        res.bridge_seen = res.bridge_seen || bridge;
        res.bridge_at_end = bridge;
        res.rejected += p_out->hump_rejected ? 1U : 0U;
        res.bridge_after_reject = res.bridge_after_reject
                                  || (bridge && (res.rejected > 0U));
        if (bridge)
        {
            res.bridge_cycles++;
            res.last_bridge = i;
        }
        x += ds;
        i++;
    }
    return res;
}

/* Heights up to 30 mm on ramps of 80 to 150 mm per side, at car speeds, are
 * valid humps with no per-case tuning: the lift mask holds through the bumps
 * and the follower bridges the hump. A 10 mm hump on 150 mm ramps tilts only
 * 3.8 deg, under the 4 deg the car enters at, so the detector cannot see it
 * and it is left out. */
static void test_pipeline_valid_humps_are_masked (void)
{
    float const heights_mm[] = { 10.0f, 20.0f, 30.0f };
    float const ramps_mm[] = { 80.0f, 100.0f, 150.0f };
    float const speeds[] = { 0.12f, 0.15f, 0.18f };
    uint32_t h;
    uint32_t r;
    uint32_t v;

    printf ("pipeline_humps_up_to_30mm_are_valid\n");
    for (h = 0U; h < 3U; h++)
    {
        for (r = 0U; r < 3U; r++)
        {
            for (v = 0U; v < 3U; v++)
            {
                pipeline_t pipe;
                pipeline_step_t out;
                drive_t d;
                drive_result_t res;

                if ((0U == h) && (2U == r))
                {
                    continue;
                }
                pipe_car_at_rest (&pipe);
                d.prof.height_m = heights_mm[h] * MM_TO_M;
                d.prof.ramp_m = ramps_mm[r] * MM_TO_M;
                d.speed_mps = speeds[v];
                d.bumps = true;
                d.free_from_x_mm = 0U;
                res = drive_hump (&pipe, &d, &out);
                if (!res.bridge_seen || res.lifted || (0U != res.rejected)
                    || res.bridge_at_end)
                {
                    printf ("  case h=%d ramp=%d v=%d\n", (int)heights_mm[h],
                            (int)ramps_mm[r], (int)(speeds[v] * 100.0f));
                }
                CHECK (res.bridge_seen);
                CHECK (res.bridge_cycles > 20U);
                CHECK (!res.bridge_at_end);
                CHECK (!res.lifted);
                CHECK (0U == res.rejected);
            }
        }
    }
}

/* Past the limits it is not a valid hump: the flag clears at once, alternating
 * wheel vibration alone does not establish a pickup. 40 mm on 100 mm ramps tilts 19.5 deg;
 * 50 mm on 400 mm ramps tilts only 7 deg and fails on the height. */
static void test_pipeline_tall_humps_are_not_valid (void)
{
    float const heights_m[] = { 0.040f, 0.050f, 0.060f };
    float const ramps_m[] = { 0.100f, 0.400f, 0.200f };
    uint32_t k;

    printf ("pipeline_humps_over_30mm_are_not_valid\n");
    for (k = 0U; k < 3U; k++)
    {
        pipeline_t pipe;
        pipeline_step_t out;
        drive_t d;
        drive_result_t res;

        pipe_car_at_rest (&pipe);
        d.prof.height_m = heights_m[k];
        d.prof.ramp_m = ramps_m[k];
        d.speed_mps = SPEED_MPS;
        d.bumps = true;
        d.free_from_x_mm = 0U;
        res = drive_hump (&pipe, &d, &out);
        CHECK (1U == res.rejected);
        CHECK (!res.lifted);
        CHECK (!res.bridge_after_reject);
    }
}

/* A steady 20 degree ramp is past the pitch limit at once. */
static void test_pipeline_steep_ramp_is_not_valid (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    drive_t d;
    drive_result_t res;

    printf ("pipeline_20_degree_ramp_is_not_valid\n");
    pipe_car_at_rest (&pipe);
    d.prof.height_m = AXLE_M * sinf (20.0f * DEG_TO_RAD) * 3.0f;
    d.prof.ramp_m = 0.300f;
    d.speed_mps = SPEED_MPS;
    d.bumps = true;
    d.free_from_x_mm = 0U;
    res = drive_hump (&pipe, &d, &out);
    CHECK (1U == res.rejected);
    CHECK (!res.bridge_after_reject);
    CHECK (!res.lifted);
}

/* One bad hump does not spoil the next valid one. */
static void test_pipeline_rejection_clears_after_hump (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    drive_t d;
    drive_result_t res;

    printf ("pipeline_rejection_clears_after_the_hump\n");
    pipe_car_at_rest (&pipe);
    d.prof.height_m = 0.040f;
    d.prof.ramp_m = RAMP_M;
    d.speed_mps = SPEED_MPS;
    d.bumps = false;
    d.free_from_x_mm = 0U;
    res = drive_hump (&pipe, &d, &out);
    CHECK (1U == res.rejected);
    d.prof.height_m = 0.020f;
    res = drive_hump (&pipe, &d, &out);
    CHECK (0U == res.rejected);
    CHECK (res.bridge_cycles > 20U);
}

/* The wheels-free cue can rise on a downhill: a bumpy descent still reads as
 * a hump and does not become a lift. */
static void test_pipeline_wheels_free_descent_is_not_lifted (void)
{
    float const heights_mm[] = { 20.0f, 30.0f };
    uint32_t h;

    printf ("pipeline_wheels_free_on_descent_is_not_lifted\n");
    for (h = 0U; h < 2U; h++)
    {
        pipeline_t pipe;
        pipeline_step_t out;
        drive_t d;
        drive_result_t res;

        pipe_car_at_rest (&pipe);
        d.prof.height_m = heights_mm[h] * MM_TO_M;
        d.prof.ramp_m = RAMP_M;
        d.speed_mps = SPEED_MPS;
        d.bumps = true;
        d.free_from_x_mm = 120U;
        res = drive_hump (&pipe, &d, &out);
        CHECK (res.bridge_seen);
        CHECK (!res.lifted);
        CHECK (ATTITUDE_LIFTED != out.attitude.state);
    }
}

/* The pitch limit comes from the axle to caster distance and stays under the
 * tilt envelope of the lift check. */
static void test_pipeline_hump_limits (void)
{
    pipeline_t pipe;
    attitude_config_t acfg;
    float const peak_deg = asinf ((float)PIPELINE_HUMP_MAX_MM
                                  / (float)CHASSIS_AXLE_TO_CASTER_MM)
                           / DEG_TO_RAD;

    printf ("pipeline_hump_pitch_limit\n");
    pipe_at_rest (&pipe);
    attitude_default_config (&acfg);
    CHECK (pipe.hump_pitch_max_deg > peak_deg);
    CHECK (pipe.hump_pitch_max_deg < acfg.tilt_envelope_deg);
    CHECK (480 == PIPELINE_HUMP_MASK_MM);
}

/* An invalid sample between reads shows as the stale fault in the word. */
static void test_glitch_between_reads_sets_stale_fault (void)
{
    motion_adapter_t ad;
    motion_reading_t r;
    motion_input_t m;
    pipeline_t pipe;
    pipeline_step_t out;
    hump_fields_t f;
    float acc[3] = { 0.0f, 0.0f, 1.0f };
    uint32_t i;

    printf ("glitch_between_reads_sets_stale_fault\n");
    CHECK (motion_adapter_init (&ad, 1.0f));
    pipe_at_rest (&pipe);
    for (i = 0U; i < 3U; i++)
    {
        reading_init (&r, 1000000U + (i * 20000U), 0, 0);
        r.invalid_samples[0] = (uint8_t)((i < 2U) ? 0U : 1U);
        CHECK (motion_adapter_convert (&ad, &r, r.timestamp_us + 1000U, &m));
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
        CHECK (hump_unpack (out.word, &f));
        CHECK (((f.faults & PHYSICS_FAULT_STALE) != 0U) == (2U == i));
    }
}

/* A marginal encoder turns the stale fault on and off every other cycle; each
 * change of the fault bits is a telemetry event, so the bit stays set for
 * PIPELINE_STALE_HOLD_CYCLES clean cycles after the last stale one. */
static void test_stale_fault_holds_through_flicker (void)
{
    pipeline_t pipe;
    pipeline_step_t out;
    hump_fields_t f;
    motion_input_t m;
    float acc[3] = { 0.0f, 0.0f, 1.0f };
    uint32_t i;
    uint32_t changes = 0U;
    uint32_t events = 0U;
    bool last = false;

    printf ("stale_fault_holds_through_flicker\n");
    pipe_at_rest (&pipe);
    memset (&m, 0, sizeof (m));
    m.wheels_valid = true;
    m.age_s = 0.005f;
    for (i = 0U; i < 39U; i++)
    {
        m.valid = (0U != (i % 2U));
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
        CHECK (hump_unpack (out.word, &f));
        if (((f.faults & PHYSICS_FAULT_STALE) != 0U) != last)
        {
            changes++;
            last = !last;
        }
        events += out.to_nav ? 1U : 0U;
    }
    CHECK (1U == changes);
    CHECK (1U == events);
    CHECK (last);

    m.valid = true;
    for (i = 0U; i < PIPELINE_STALE_HOLD_CYCLES; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
        CHECK (hump_unpack (out.word, &f));
        CHECK ((f.faults & PHYSICS_FAULT_STALE) != 0U);
    }
    CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    CHECK (hump_unpack (out.word, &f));
    CHECK (0U == (f.faults & PHYSICS_FAULT_STALE));
    CHECK (out.to_nav);
}

static bool calibration_at_pose (const float acc[3])
{
    hump_config_t        hcfg;
    imu_motion_config_t  mcfg;
    pipeline_t           pipe;
    pipeline_step_t      out = {0};
    motion_input_t const m   = { .valid = true, .wheels_valid = true, .age_s = 0.005f };
    uint32_t             i   = 0U;

    hump_default_config (&hcfg);
    imu_motion_default_config (&mcfg);
    mcfg.mag_enabled = false;
    CHECK (pipeline_init (&pipe, &hcfg, &mcfg, 10U));
    for (i = 0U; i < 300U; i++)
    {
        CHECK (pipeline_step (&pipe, acc, NULL, &m, DT_S, false, &out));
    }
    CHECK (out.calibrated != out.attitude_unknown);
    return out.calibrated;
}

static void test_calibration_tilt_boundary (void)
{
    float const below = 9.9f * DEG_TO_RAD;
    float const above = 10.1f * DEG_TO_RAD;
    float const mild_pitch[3] = { sinf (below), 0.0f, cosf (below) };
    float const steep_pitch[3] = { -sinf (above), 0.0f, cosf (above) };
    float const steep_roll[3] = { 0.0f, sinf (above), cosf (above) };

    CHECK (calibration_at_pose (mild_pitch));
    CHECK (!calibration_at_pose (steep_pitch));
    CHECK (!calibration_at_pose (steep_roll));
}

static void test_calibration_inverted_pose (void)
{
    float const inverted[3] = { 0.0f, 0.0f, -1.0f };

    CHECK (!calibration_at_pose (inverted));
}

typedef struct
{
    const char * name;
    void (*run) (void);
} test_case_t;

#define CASE(name) { #name, test_ ## name }

static const test_case_t cases[] = {
    CASE(counts),
    CASE(init_ok),
    CASE(init_failures),
    CASE(reads),
    CASE(retries),
    CASE(counters_survive_reinit),
    CASE(config_readback),
    CASE(read_after_latch),
    CASE(glitch_after_failed_cycle_is_retried),
    CASE(adapter),
    CASE(adapter_glitch_between_reads),
    CASE(pipeline_lift),
    CASE(pipeline_imu_lost),
    CASE(lost_attitude_unknown),
    CASE(config_readback_budget),
    CASE(imu_lost_edges),
    CASE(attitude_publish_gate),
    CASE(attitude_unknown_without_reference),
    CASE(pipeline),
    CASE(pipeline_hump_mutes_bumps),
    CASE(pipeline_lift_settled_gap),
    CASE(pipeline_sustained_lift_cues),
    CASE(pipeline_slow_grounded_hump),
    CASE(pipeline_hump_requires_forward_travel),
    CASE(pipeline_slow_hump_ends_at_travel_cap),
    CASE(pipeline_lift_at_rest_on_hump_state),
    CASE(pipeline_wheels_free_on_hump_state),
    CASE(pipeline_lasting_hump_state_ends_at_cap),
    CASE(pipeline_hump_limits),
    CASE(pipeline_valid_humps_are_masked),
    CASE(pipeline_tall_humps_are_not_valid),
    CASE(pipeline_steep_ramp_is_not_valid),
    CASE(pipeline_rejection_clears_after_hump),
    CASE(pipeline_wheels_free_descent_is_not_lifted),
    CASE(glitch_between_reads_sets_stale_fault),
    CASE(stale_fault_holds_through_flicker),
    CASE(calibration_tilt_boundary),
    CASE(calibration_inverted_pose),
};

int main (int argc, char ** argv)
{
    size_t idx = 0U;

    if (2 == argc)
    {
        for (idx = 0U; idx < (sizeof (cases) / sizeof (cases[0])); idx++)
        {
            if (0 == strcmp (argv[1], cases[idx].name))
            {
                cases[idx].run ();
                return (g_failures == 0) ? 0 : 1;
            }
        }

        (void)fprintf (stderr, "Unknown case: %s\n", argv[1]);
        return 2;
    }
    if (1 != argc)
    {
        (void)fprintf (stderr, "Usage: %s [case]\n", argv[0]);
        return 2;
    }
    for (idx = 0U; idx < (sizeof (cases) / sizeof (cases[0])); idx++)
    {
        cases[idx].run ();
    }

    if (0 == g_failures)
    {
        printf ("\nALL TESTS PASSED\n");
    }
    else
    {
        printf ("\n%d CHECK(S) FAILED\n", g_failures);
    }
    return (0 == g_failures) ? 0 : 1;
}

/*** end of file ***/
