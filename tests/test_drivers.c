/** @file test_drivers.c
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
#include "lsm303.h"

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
    lsm303_t dev;

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
    lsm303_t dev;

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
    lsm303_t dev;
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

int main (void)
{
    test_counts ();
    test_init_ok ();
    test_init_failures ();
    test_reads ();
    test_retries ();
    test_adapter ();
    test_pipeline_lift ();
    test_pipeline ();

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
