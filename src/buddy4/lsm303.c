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

/* Two's complement from an unsigned pattern without bitwise operations on
 * signed values (BARR-C:2018 rule 5.3). */
static int16_t to_signed (uint32_t pattern, uint32_t bits)
{
    uint32_t half = (uint32_t)1U << (bits - 1U);
    int32_t value = (int32_t)pattern;

    if (pattern >= half)
    {
        value -= (int32_t)(half * 2U);
    }
    return (int16_t)value;
}

int16_t lsm303_accel_counts (uint8_t lo, uint8_t hi)
{
    /* 12-bit value, left-justified in 16 bits. */
    uint32_t pattern = (((uint32_t)hi << 8U) | (uint32_t)lo) >> 4U;

    return to_signed (pattern, 12U);
}

int16_t lsm303_mag_counts (uint8_t hi, uint8_t lo)
{
    return to_signed (((uint32_t)hi << 8U) | (uint32_t)lo, 16U);
}

/* One transaction with retries. Latches the fault after the last attempt. */
static bool transfer (lsm303_t * p_dev, uint8_t addr7,
                      const uint8_t * p_wr, uint32_t wr_len,
                      uint8_t * p_rd, uint32_t rd_len)
{
    uint8_t attempt;
    bool ok = false;

    p_dev->stats.transactions++;

    for (attempt = 0U; (attempt < p_dev->max_attempts) && !ok; attempt++)
    {
        b4_bus_result_t result;

        if (attempt > 0U)
        {
            p_dev->stats.retries++;
        }

        result = p_dev->bus.write_read (p_dev->bus.p_ctx, addr7,
                                        p_wr, wr_len, p_rd, rd_len);
        switch (result)
        {
            case B4_BUS_OK:
                ok = true;
                break;

            case B4_BUS_NACK:
                p_dev->stats.nacks++;
                break;

            case B4_BUS_TIMEOUT:
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
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = value;
    return transfer (p_dev, addr7, buf, 2U, NULL, 0U);
}

static bool read_regs (lsm303_t * p_dev, uint8_t addr7, uint8_t reg,
                       uint8_t * p_buf, uint32_t len)
{
    return transfer (p_dev, addr7, &reg, 1U, p_buf, len);
}

/* Track bad reads in a row; clear ready when the sensor looks lost. */
static void note_read (lsm303_t * p_dev, bool good)
{
    if (good)
    {
        p_dev->bad_reads = 0U;
    }
    else
    {
        p_dev->bad_reads++;
        if (p_dev->bad_reads >= LSM303_LOST_AFTER)
        {
            p_dev->bad_reads = 0U;
            p_dev->ready = false;
        }
    }
}

lsm303_result_t lsm303_init (lsm303_t * p_dev, const b4_i2c_bus_t * p_bus,
                             uint8_t max_attempts)
{
    static const uint8_t acc_regs[2] = { LSM303_CTRL_REG1_A,
                                         LSM303_CTRL_REG4_A };
    static const uint8_t acc_vals[2] = { LSM303_CTRL_REG1_A_VAL,
                                         LSM303_CTRL_REG4_A_VAL };
    static const uint8_t mag_vals[3] = { LSM303_CRA_REG_M_VAL,
                                         LSM303_CRB_REG_M_VAL,
                                         LSM303_MR_REG_M_VAL };
    uint8_t id[3];
    uint8_t back[3];
    uint8_t i;

    if ((NULL == p_dev) || (NULL == p_bus) || (NULL == p_bus->write_read)
        || (max_attempts < 1U) || (max_attempts > MAX_ATTEMPTS_LIMIT))
    {
        return LSM303_BAD_ARG;
    }

    p_dev->bus = *p_bus;
    p_dev->max_attempts = max_attempts;
    p_dev->ready = false;
    p_dev->fault = false;
    p_dev->bad_reads = 0U;
    p_dev->stats.transactions = 0U;
    p_dev->stats.retries = 0U;
    p_dev->stats.nacks = 0U;
    p_dev->stats.timeouts = 0U;
    p_dev->stats.errors = 0U;
    p_dev->stats.failures = 0U;

    /* Identity. The registers are consecutive and the magnetometer
     * auto-increments without a flag bit. */
    if (!read_regs (p_dev, LSM303_MAG_ADDR, LSM303_IRA_REG_M, id, 3U))
    {
        return LSM303_BUS_FAULT;
    }
    if ((0x48U != id[0]) || (0x34U != id[1]) || (0x33U != id[2]))
    {
        return LSM303_BAD_ID;
    }

    for (i = 0U; i < 2U; i++)
    {
        if (!write_reg (p_dev, LSM303_ACC_ADDR, acc_regs[i], acc_vals[i]))
        {
            return LSM303_BUS_FAULT;
        }
    }
    for (i = 0U; i < 3U; i++)
    {
        if (!write_reg (p_dev, LSM303_MAG_ADDR,
                        (uint8_t)(LSM303_CRA_REG_M + i), mag_vals[i]))
        {
            return LSM303_BUS_FAULT;
        }
    }

    /* Read back: confirms the accelerometer is really there and that both
     * parts took the settings. */
    for (i = 0U; i < 2U; i++)
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
    if (!read_regs (p_dev, LSM303_MAG_ADDR, LSM303_CRA_REG_M, back, 3U))
    {
        return LSM303_BUS_FAULT;
    }
    for (i = 0U; i < 3U; i++)
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
    uint8_t buf[6];
    uint8_t axis;

    if ((NULL == p_dev) || (NULL == p_out) || !p_dev->ready)
    {
        return LSM303_BAD_ARG;
    }
    if (!read_regs (p_dev, LSM303_ACC_ADDR,
                    (uint8_t)(LSM303_OUT_X_L_A | LSM303_AUTO_INC_A), buf, 6U))
    {
        note_read (p_dev, false);
        return LSM303_BUS_FAULT;
    }

    /* X, Y, Z; low byte first (BLE = 0). */
    for (axis = 0U; axis < 3U; axis++)
    {
        p_out->raw[axis] = lsm303_accel_counts (buf[2U * axis],
                                                buf[(2U * axis) + 1U]);
        p_out->value[axis] = (float)p_out->raw[axis] * ACC_MG_PER_LSB;
    }

    if ((0 == p_out->raw[0]) && (0 == p_out->raw[1]) && (0 == p_out->raw[2]))
    {
        note_read (p_dev, false);
        return LSM303_NO_DATA;
    }

    note_read (p_dev, true);
    return LSM303_OK;
}

lsm303_result_t lsm303_read_mag (lsm303_t * p_dev, lsm303_vec_t * p_out)
{
    uint8_t buf[6];

    if ((NULL == p_dev) || (NULL == p_out) || !p_dev->ready)
    {
        return LSM303_BAD_ARG;
    }
    if (!read_regs (p_dev, LSM303_MAG_ADDR, LSM303_OUT_X_H_M, buf, 6U))
    {
        note_read (p_dev, false);
        return LSM303_BUS_FAULT;
    }

    /* Register order is X, Z, Y, high byte first. */
    p_out->raw[0] = lsm303_mag_counts (buf[0], buf[1]);
    p_out->raw[2] = lsm303_mag_counts (buf[2], buf[3]);
    p_out->raw[1] = lsm303_mag_counts (buf[4], buf[5]);
    p_out->value[0] = (float)p_out->raw[0] / MAG_XY_LSB_PER_G;
    p_out->value[1] = (float)p_out->raw[1] / MAG_XY_LSB_PER_G;
    p_out->value[2] = (float)p_out->raw[2] / MAG_Z_LSB_PER_G;

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
