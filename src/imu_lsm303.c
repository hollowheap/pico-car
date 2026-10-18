/** @file imu_lsm303.c
 *
 * @brief GY-511 LSM303DLHC driver on I2C1 GP2/GP3.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include "imu_lsm303.h"

/* Accel registers (Table 17). */
#define REG_CTRL1_A  0x20u
#define REG_CTRL4_A  0x23u
#define REG_OUT_X_L  0x28u
#define REG_AUTO_INC 0x80u

/* Mag registers. */
#define REG_CRA_M    0x00u
#define REG_CRB_M    0x01u
#define REG_MR_M     0x02u
#define REG_OUT_X_H  0x03u

#define VAL_CTRL1_A  0x57u
#define VAL_CTRL4_A  0x88u
#define VAL_CRA_M    0x10u
#define VAL_CRB_M    0x20u
#define VAL_MR_M     0x00u

#define RAD_TO_DEG   57.29578f

static bool write_reg (uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2u] = {reg, val};
    int     ret     = i2c_write_blocking(i2c1, addr, buf, 2u, false);

    return ret == 2;
}

static bool read_regs (uint8_t addr, uint8_t reg, uint8_t * p_buf, uint8_t len)
{
    int ret = i2c_write_blocking(i2c1, addr, &reg, 1u, true);

    if (ret != 1)
    {
        return false;
    }

    ret = i2c_read_blocking(i2c1, addr, p_buf, len, false);

    return ret == (int) len;
}

bool imu_lsm303_init (void)
{
    i2c_init(i2c1, IMU_I2C_BAUD_HZ);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    sleep_ms(10u);

    bool ok = true;

    ok = write_reg(IMU_ACCEL_ADDR, REG_CTRL1_A, VAL_CTRL1_A) && ok;
    ok = write_reg(IMU_ACCEL_ADDR, REG_CTRL4_A, VAL_CTRL4_A) && ok;
    ok = write_reg(IMU_MAG_ADDR, REG_CRA_M, VAL_CRA_M) && ok;
    ok = write_reg(IMU_MAG_ADDR, REG_CRB_M, VAL_CRB_M) && ok;
    ok = write_reg(IMU_MAG_ADDR, REG_MR_M, VAL_MR_M) && ok;

    return ok;
}

bool imu_accel_read_raw (imu_accel_raw_t * p_out)
{
    uint8_t raw[6u] = {0u};

    if (p_out == NULL)
    {
        return false;
    }

    if (!read_regs(IMU_ACCEL_ADDR, (uint8_t) (REG_OUT_X_L | REG_AUTO_INC),
                   raw, 6u))
    {
        return false;
    }

    /* Registers are 12-bit left-aligned; shift with sign extension. */
    int16_t x = (int16_t) (uint16_t) ((uint16_t) (((uint16_t) raw[1u] << 8u) |
                                                 (uint16_t) raw[0u]));
    int16_t y = (int16_t) (uint16_t) ((uint16_t) (((uint16_t) raw[3u] << 8u) |
                                                 (uint16_t) raw[2u]));
    int16_t z = (int16_t) (uint16_t) ((uint16_t) (((uint16_t) raw[5u] << 8u) |
                                                 (uint16_t) raw[4u]));

    p_out->x = (int16_t) (x >> 4);
    p_out->y = (int16_t) (y >> 4);
    p_out->z = (int16_t) (z >> 4);

    return true;
}

bool imu_mag_read_raw (imu_mag_raw_t * p_out)
{
    uint8_t raw[6u] = {0u};

    if (p_out == NULL)
    {
        return false;
    }

    /* Wire order is X, Z, Y (each H then L). */
    if (!read_regs(IMU_MAG_ADDR, REG_OUT_X_H, raw, 6u))
    {
        return false;
    }

    p_out->x = (int16_t) ((uint16_t) (((uint16_t) raw[0u] << 8u) |
                                     (uint16_t) raw[1u]));
    p_out->z = (int16_t) ((uint16_t) (((uint16_t) raw[2u] << 8u) |
                                     (uint16_t) raw[3u]));
    p_out->y = (int16_t) ((uint16_t) (((uint16_t) raw[4u] << 8u) |
                                     (uint16_t) raw[5u]));

    return true;
}

bool imu_accel_read_g (const imu_calib_t * p_calib, imu_accel_g_t * p_out)
{
    imu_accel_raw_t raw = {0};

    if ((p_calib == NULL) || (p_out == NULL))
    {
        return false;
    }

    if (!imu_accel_read_raw(&raw))
    {
        return false;
    }

    p_out->x_g =
        (((float) raw.x * IMU_ACCEL_SENS_MG_LSB) / 1000.0f) - p_calib->bias_x_g;
    p_out->y_g =
        (((float) raw.y * IMU_ACCEL_SENS_MG_LSB) / 1000.0f) - p_calib->bias_y_g;
    p_out->z_g =
        (((float) raw.z * IMU_ACCEL_SENS_MG_LSB) / 1000.0f) - p_calib->bias_z_g;

    return true;
}

bool imu_calibrate_level (imu_calib_t * p_calib)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    const uint8_t samples = 50u;

    if (p_calib == NULL)
    {
        return false;
    }

    for (uint8_t i = 0u; i < samples; i++)
    {
        imu_accel_raw_t raw = {0};

        if (!imu_accel_read_raw(&raw))
        {
            p_calib->valid = false;

            return false;
        }

        sum_x += ((float) raw.x * IMU_ACCEL_SENS_MG_LSB) / 1000.0f;
        sum_y += ((float) raw.y * IMU_ACCEL_SENS_MG_LSB) / 1000.0f;
        sum_z += ((float) raw.z * IMU_ACCEL_SENS_MG_LSB) / 1000.0f;
        sleep_ms(10u);
    }

    /* Level: x~0, y~0, z~1g. Keep z at 1g in bias so tilt math sees
     * gravity; callers subtract 1g only for linear-accel estimates. */
    p_calib->bias_x_g = sum_x / (float) samples;
    p_calib->bias_y_g = sum_y / (float) samples;
    p_calib->bias_z_g = (sum_z / (float) samples) - 1.0f;
    p_calib->valid   = true;

    return true;
}

float imu_pitch_deg (const imu_accel_g_t * p_g)
{
    float denom = sqrtf((p_g->y_g * p_g->y_g) + (p_g->z_g * p_g->z_g));

    if (denom < 0.05f)
    {
        denom = 0.05f;
    }

    return atan2f(p_g->x_g, denom) * RAD_TO_DEG;
}

float imu_roll_deg (const imu_accel_g_t * p_g)
{
    float denom = sqrtf((p_g->x_g * p_g->x_g) + (p_g->z_g * p_g->z_g));

    if (denom < 0.05f)
    {
        denom = 0.05f;
    }

    return atan2f(p_g->y_g, denom) * RAD_TO_DEG;
}

/*** end of file ***/
