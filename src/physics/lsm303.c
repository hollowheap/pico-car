/** @file lsm303.c
 *
 * @brief LSM303DLHC (GY-511) accelerometer and magnetometer protocol.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lsm303.h"

#define ACC_MG_PER_LSB     (0.001f)   /* +/-2 g, high resolution.         */
#define MAG_XY_LSB_PER_G   (1100.0f)  /* +/-1.3 gauss, table 75.          */
#define MAG_Z_LSB_PER_G    (980.0f)
#define MAX_ATTEMPTS_LIMIT (8U)

#define BYTE_SHIFT         (8U)       /* Bits per byte.                   */
#define ACC_LEFT_JUSTIFY   (4U)       /* 12-bit value in the top of 16.   */
#define ACC_BITS           (12U)
#define MAG_BITS           (16U)
#define WRITE_REG_LEN      (2U)       /* Register address + value.        */
#define ACC_REG_COUNT      (2U)
#define MAG_REG_COUNT      (3U)
#define MAG_ID_LEN         (3U)       /* IRA, IRB, IRC.                   */
#define MAG_ID_IRA         (0x48U)    /* 'H'                              */
#define MAG_ID_IRB         (0x34U)    /* '4'                              */
#define MAG_ID_IRC         (0x33U)    /* '3'                              */
#define MAG_ID_IRC_INDEX   (2U)       /* Third identity byte.             */
#define BURST_LEN          (6U)       /* Three axes, two bytes each.      */
#define BYTES_PER_AXIS     (2U)
#define MAG_BUF_X_HI       (0U)       /* Register order is X, Z, Y.       */
#define MAG_BUF_X_LO       (1U)
#define MAG_BUF_Z_HI       (2U)
#define MAG_BUF_Z_LO       (3U)
#define MAG_BUF_Y_HI       (4U)
#define MAG_BUF_Y_LO       (5U)

/* Two's complement from an unsigned pattern without bitwise operations on
 * signed values (BARR-C:2018 rule 5.3). */
static int16_t to_signed (uint32_t pattern, uint32_t bits)
{
    uint32_t half = (uint32_t)1U << (bits - 1U);
    int32_t value = (int32_t)pattern;

    if (pattern >= half)
    {
        value -= (int32_t)(half << 1U);   /* half * 2 = 2^bits. */
    }
    return (int16_t)value;
}

int16_t lsm303_accel_counts (uint8_t lo, uint8_t hi)
{
    /* 12-bit value, left-justified in 16 bits. */
    uint32_t pattern = (((uint32_t)hi << BYTE_SHIFT) | (uint32_t)lo)
                       >> ACC_LEFT_JUSTIFY;

    return to_signed (pattern, ACC_BITS);
}

int16_t lsm303_mag_counts (uint8_t hi, uint8_t lo)
{
    return to_signed (((uint32_t)hi << BYTE_SHIFT) | (uint32_t)lo, MAG_BITS);
}

/* One transaction with retries. Latches the fault after the last attempt. */
static bool transfer (lsm303_t * p_dev, uint8_t addr7,
                      const uint8_t * p_wr, uint32_t wr_len,
                      uint8_t * p_rd, uint32_t rd_len)
{
    uint8_t attempt = 0U;
    bool ok = false;

    p_dev->stats.transactions++;

    for (attempt = 0U; (attempt < p_dev->max_attempts) && !ok; attempt++)
    {
        i2c_bus_result_t result = I2C_BUS_OK;

        if (attempt > 0U)
        {
            p_dev->stats.retries++;
        }

        result = p_dev->bus.write_read (p_dev->bus.p_ctx, addr7,
                                        p_wr, wr_len, p_rd, rd_len);
        switch (result)
        {
            case I2C_BUS_OK:
                ok = true;
                break;

            case I2C_BUS_NACK:
                p_dev->stats.nacks++;
                break;

            case I2C_BUS_TIMEOUT:
                p_dev->stats.timeouts++;
                /* A slave holding SDA low can block every later transfer:
                 * clock it free before the next attempt. */
                if (NULL != p_dev->bus.recover)
                {
                    p_dev->bus.recover (p_dev->bus.p_ctx);
                }
                break;

            default:
                p_dev->stats.errors++;
                break;
        }
    }

    if (!ok)
    {
        p_dev->stats.failures++;
        p_dev->fault = true;
    }

    return ok;
}

static bool write_reg (lsm303_t * p_dev, uint8_t addr7, uint8_t reg,
                       uint8_t value)
{
    uint8_t buf[WRITE_REG_LEN];

    buf[0] = reg;
    buf[1] = value;
    return transfer (p_dev, addr7, buf, WRITE_REG_LEN, NULL, 0U);
}

static bool read_regs (lsm303_t * p_dev, uint8_t addr7, uint8_t reg,
                       uint8_t * p_buf, uint32_t len)
{
    return transfer (p_dev, addr7, &reg, 1U, p_buf, len);
}

lsm303_result_t lsm303_init (lsm303_t * p_dev, const i2c_bus_t * p_bus,
                             uint8_t max_attempts)
{
    static const uint8_t acc_regs[ACC_REG_COUNT] = { LSM303_CTRL_REG1_A,
                                                     LSM303_CTRL_REG4_A };
    static const uint8_t acc_vals[ACC_REG_COUNT] = { LSM303_CTRL_REG1_A_VAL,
                                                     LSM303_CTRL_REG4_A_VAL };
    static const uint8_t mag_vals[MAG_REG_COUNT] = { LSM303_CRA_REG_M_VAL,
                                                     LSM303_CRB_REG_M_VAL,
                                                     LSM303_MR_REG_M_VAL };
    uint8_t id[MAG_ID_LEN];
    uint8_t back[MAG_REG_COUNT];
    uint8_t i = 0U;

    if ((NULL == p_dev) || (NULL == p_bus) || (NULL == p_bus->write_read)
        || (max_attempts < 1U) || (max_attempts > MAX_ATTEMPTS_LIMIT))
    {
        return LSM303_BAD_ARG;
    }

    p_dev->bus = *p_bus;
    p_dev->max_attempts = max_attempts;
    p_dev->ready = false;
    p_dev->fault = false;
    p_dev->stats.transactions = 0U;
    p_dev->stats.retries = 0U;
    p_dev->stats.nacks = 0U;
    p_dev->stats.timeouts = 0U;
    p_dev->stats.errors = 0U;
    p_dev->stats.failures = 0U;

    /* Identity. The registers are consecutive and the magnetometer
     * auto-increments without a flag bit. */
    if (!read_regs (p_dev, LSM303_MAG_ADDR, LSM303_IRA_REG_M, id,
                    MAG_ID_LEN))
    {
        return LSM303_BUS_FAULT;
    }
    if ((MAG_ID_IRA != id[0]) || (MAG_ID_IRB != id[1])
        || (MAG_ID_IRC != id[MAG_ID_IRC_INDEX]))
    {
        return LSM303_BAD_ID;
    }

    for (i = 0U; i < ACC_REG_COUNT; i++)
    {
        if (!write_reg (p_dev, LSM303_ACC_ADDR, acc_regs[i], acc_vals[i]))
        {
            return LSM303_BUS_FAULT;
        }
    }
    for (i = 0U; i < MAG_REG_COUNT; i++)
    {
        if (!write_reg (p_dev, LSM303_MAG_ADDR,
                        (uint8_t)(LSM303_CRA_REG_M + i), mag_vals[i]))
        {
            return LSM303_BUS_FAULT;
        }
    }

    /* Read back: confirms the accelerometer is really there and that both
     * parts took the settings. */
    for (i = 0U; i < ACC_REG_COUNT; i++)
    {
        if (!read_regs (p_dev, LSM303_ACC_ADDR, acc_regs[i], &back[i], 1U))
        {
            return LSM303_BUS_FAULT;
        }
        if (back[i] != acc_vals[i])
        {
            return LSM303_CONFIG_MISMATCH;
        }
    }
    if (!read_regs (p_dev, LSM303_MAG_ADDR, LSM303_CRA_REG_M, back,
                    MAG_REG_COUNT))
    {
        return LSM303_BUS_FAULT;
    }
    for (i = 0U; i < MAG_REG_COUNT; i++)
    {
        if (back[i] != mag_vals[i])
        {
            return LSM303_CONFIG_MISMATCH;
        }
    }

    p_dev->ready = true;
    return LSM303_OK;
}

lsm303_result_t lsm303_read_accel (lsm303_t * p_dev, lsm303_vec_t * p_out)
{
    uint8_t buf[BURST_LEN];
    uint8_t axis = 0U;

    if ((NULL == p_dev) || (NULL == p_out) || !p_dev->ready)
    {
        return LSM303_BAD_ARG;
    }
    if (!read_regs (p_dev, LSM303_ACC_ADDR,
                    (uint8_t)(LSM303_OUT_X_L_A | LSM303_AUTO_INC_A), buf,
                    BURST_LEN))
    {
        return LSM303_BUS_FAULT;
    }

    /* X, Y, Z; low byte first (BLE = 0). */
    for (axis = 0U; axis < LSM303_AXIS_COUNT; axis++)
    {
        p_out->raw[axis] = lsm303_accel_counts (buf[BYTES_PER_AXIS * axis],
                                                buf[(BYTES_PER_AXIS * axis)
                                                    + 1U]);
        p_out->value[axis] = (float)p_out->raw[axis] * ACC_MG_PER_LSB;
    }

    return LSM303_OK;
}

lsm303_result_t lsm303_read_mag (lsm303_t * p_dev, lsm303_vec_t * p_out)
{
    uint8_t buf[BURST_LEN];

    if ((NULL == p_dev) || (NULL == p_out) || !p_dev->ready)
    {
        return LSM303_BAD_ARG;
    }
    if (!read_regs (p_dev, LSM303_MAG_ADDR, LSM303_OUT_X_H_M, buf, BURST_LEN))
    {
        return LSM303_BUS_FAULT;
    }

    /* Register order is X, Z, Y, high byte first. */
    p_out->raw[LSM303_AXIS_X] = lsm303_mag_counts (buf[MAG_BUF_X_HI],
                                                   buf[MAG_BUF_X_LO]);
    p_out->raw[LSM303_AXIS_Z] = lsm303_mag_counts (buf[MAG_BUF_Z_HI],
                                                   buf[MAG_BUF_Z_LO]);
    p_out->raw[LSM303_AXIS_Y] = lsm303_mag_counts (buf[MAG_BUF_Y_HI],
                                                   buf[MAG_BUF_Y_LO]);
    p_out->value[LSM303_AXIS_X] = (float)p_out->raw[LSM303_AXIS_X]
                                  / MAG_XY_LSB_PER_G;
    p_out->value[LSM303_AXIS_Y] = (float)p_out->raw[LSM303_AXIS_Y]
                                  / MAG_XY_LSB_PER_G;
    p_out->value[LSM303_AXIS_Z] = (float)p_out->raw[LSM303_AXIS_Z]
                                  / MAG_Z_LSB_PER_G;

    return LSM303_OK;
}

void lsm303_clear_fault (lsm303_t * p_dev)
{
    if (NULL != p_dev)
    {
        p_dev->fault = false;
    }
}

/*** end of file ***/
