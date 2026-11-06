/** @file i2c_rp2040.c
 *
 * @brief Polled RP2040 I2C1 master on GP2 (SDA) / GP3 (SCL), 400 kHz.
 *
 * Transfer sequence follows pico-sdk 2.x hardware_i2c (write/read blocking
 * internals), with register offsets from the pinned SDK's
 * hardware/regs/i2c.h. IC_CON.TX_EMPTY_CTRL is set, so TX_EMPTY means the
 * last byte has left the shift register.
 */

/* No <stddef.h>: the kernel headers define size_t and NULL themselves. */
#include <tk/tkernel.h>

#include <stdbool.h>
#include <stdint.h>

#include "i2c_rp2040.h"
#include "lsm303.h"

/* Base addresses (RP2040 datasheet section 2.2). */
#define I2C1_BASE_ADDR      (0x40048000UL)
#define IO_BANK0_ADDR       (0x40014000UL)
#define PADS_BANK0_ADDR     (0x4001C000UL)
#define I2C_RESETS_ADDR         (0x4000C000UL)
#define SIO_ADDR            (0xD0000000UL)
#define TIMER_ADDR          (0x40054000UL)

/* I2C register offsets. */
#define IC_CON              (I2C1_BASE_ADDR + 0x00UL)
#define IC_TAR              (I2C1_BASE_ADDR + 0x04UL)
#define IC_DATA_CMD         (I2C1_BASE_ADDR + 0x10UL)
#define IC_FS_SCL_HCNT      (I2C1_BASE_ADDR + 0x1CUL)
#define IC_FS_SCL_LCNT      (I2C1_BASE_ADDR + 0x20UL)
#define IC_INTR_MASK        (I2C1_BASE_ADDR + 0x30UL)
#define IC_RAW_INTR_STAT    (I2C1_BASE_ADDR + 0x34UL)
#define IC_RX_TL            (I2C1_BASE_ADDR + 0x38UL)
#define IC_TX_TL            (I2C1_BASE_ADDR + 0x3CUL)
#define IC_CLR_TX_ABRT      (I2C1_BASE_ADDR + 0x54UL)
#define IC_CLR_STOP_DET     (I2C1_BASE_ADDR + 0x60UL)
#define IC_ENABLE           (I2C1_BASE_ADDR + 0x6CUL)
#define IC_TXFLR            (I2C1_BASE_ADDR + 0x74UL)
#define IC_RXFLR            (I2C1_BASE_ADDR + 0x78UL)
#define IC_SDA_HOLD         (I2C1_BASE_ADDR + 0x7CUL)
#define IC_TX_ABRT_SOURCE   (I2C1_BASE_ADDR + 0x80UL)
#define IC_FS_SPKLEN        (I2C1_BASE_ADDR + 0xA0UL)

/* I2C bits. */
#define CON_MASTER          (0x001UL)
#define CON_SPEED_FAST      (0x004UL)   /* SPEED field = 2.            */
#define CON_RESTART_EN      (0x020UL)
#define CON_SLAVE_DISABLE   (0x040UL)
#define CON_TX_EMPTY_CTRL   (0x100UL)
#define CMD_READ            (0x100UL)
#define CMD_STOP            (0x200UL)
#define CMD_RESTART         (0x400UL)
#define RAW_TX_EMPTY        (0x010UL)
#define RAW_STOP_DET        (0x200UL)
#define ENABLE_ENABLE       (0x1UL)
#define ENABLE_ABORT        (0x2UL)
#define ABRT_7B_ADDR_NOACK  (0x001UL)
#define ABRT_TXDATA_NOACK   (0x008UL)
#define TX_FIFO_DEPTH       (16UL)

/* 400 kHz at clk_sys 125 MHz, same arithmetic as pico-sdk
 * i2c_set_baudrate: period = 313, lcnt = 3/5, hold = 3 * f / 10 MHz + 1. */
#define FS_LCNT             (187UL)
#define FS_HCNT             (126UL)
#define FS_SPKLEN           (11UL)
#define SDA_TX_HOLD         (38UL)

/* GPIO. */
#define GPIO_CTRL_ADDR(pin) (IO_BANK0_ADDR + 0x04UL + (8UL * (uint32_t)(pin)))
#define PAD_ADDR(pin)       (PADS_BANK0_ADDR + 0x04UL + (4UL * (uint32_t)(pin)))
#define FUNCSEL_I2C         (3UL)
#define FUNCSEL_SIO         (5UL)
#define PAD_IE              (1UL << 6U)
#define PAD_DRIVE_4MA       (1UL << 4U)
#define PAD_PUE             (1UL << 3U)
#define PAD_SCHMITT         (1UL << 1U)
#define SIO_GPIO_IN         (SIO_ADDR + 0x04UL)
#define SIO_GPIO_OUT_CLR    (SIO_ADDR + 0x18UL)
#define SIO_GPIO_OE_SET     (SIO_ADDR + 0x24UL)
#define SIO_GPIO_OE_CLR     (SIO_ADDR + 0x28UL)

/* Resets. Atomic set/clear aliases are valid for peripheral registers. */
#define I2C_RESETS_RESET     (I2C_RESETS_ADDR + 0x0UL)
#define I2C_RESETS_DONE      (I2C_RESETS_ADDR + 0x8UL)
#define I2C_RESET_I2C1       (1UL << 4U)
#define LSM303_ALIAS_SET        (0x2000UL)
#define LSM303_ALIAS_CLR        (0x3000UL)

#define TIMER_RAWL          (TIMER_ADDR + 0x28UL)

#define SDA_MASK            (1UL << I2C_SDA_PIN)
#define SCL_MASK            (1UL << I2C_SCL_PIN)
#define RECOVER_HALF_US     (5U)        /* 100 kHz bit-banged clock.   */
#define ABORT_WAIT_US       (1000U)

uint32_t physics_time_us (void)
{
    return in_w (TIMER_RAWL);
}

static bool expired (uint32_t start_us, uint32_t limit_us)
{
    return (physics_time_us () - start_us) > limit_us;   /* Wrap-safe. */
}

static void wait_us (uint32_t us)
{
    uint32_t start = physics_time_us ();

    while (!expired (start, us))
    {
        /* Busy-wait: only used during bus recovery. */
    }
}

static void pins_to (uint32_t funcsel)
{
    out_w (GPIO_CTRL_ADDR (I2C_SDA_PIN), funcsel);
    out_w (GPIO_CTRL_ADDR (I2C_SCL_PIN), funcsel);
}

/* Stop a stuck transfer and leave the controller disabled. */
static void abort_transfer (void)
{
    uint32_t start = physics_time_us ();

    out_w (IC_ENABLE, ENABLE_ENABLE | ENABLE_ABORT);
    while (((in_w (IC_ENABLE) & ENABLE_ABORT) != 0UL)
           && !expired (start, ABORT_WAIT_US))
    {
        /* Wait for the controller to finish the abort. */
    }
    (void)in_w (IC_CLR_TX_ABRT);
    (void)in_w (IC_CLR_STOP_DET);
    out_w (IC_ENABLE, 0UL);
}

/* Bus clear (I2C spec 3.1.16): clock SCL up to 9 times until the slave
 * releases SDA, then send STOP. Pins are driven open-drain through SIO:
 * output low, toggled by output-enable, external/internal pull-ups high. */
static void recover (void * p_ctx)
{
    uint32_t pulse;

    (void)p_ctx;
    out_w (IC_ENABLE, 0UL);
    out_w (SIO_GPIO_OE_CLR, SDA_MASK | SCL_MASK);
    out_w (SIO_GPIO_OUT_CLR, SDA_MASK | SCL_MASK);
    pins_to (FUNCSEL_SIO);
    wait_us (RECOVER_HALF_US);

    for (pulse = 0U; (pulse < 9U) && ((in_w (SIO_GPIO_IN) & SDA_MASK) == 0UL);
         pulse++)
    {
        out_w (SIO_GPIO_OE_SET, SCL_MASK);      /* SCL low.  */
        wait_us (RECOVER_HALF_US);
        out_w (SIO_GPIO_OE_CLR, SCL_MASK);      /* SCL high. */
        wait_us (RECOVER_HALF_US);
    }

    /* STOP: SDA low while SCL low, SCL high, then SDA high. */
    out_w (SIO_GPIO_OE_SET, SCL_MASK);
    wait_us (RECOVER_HALF_US);
    out_w (SIO_GPIO_OE_SET, SDA_MASK);
    wait_us (RECOVER_HALF_US);
    out_w (SIO_GPIO_OE_CLR, SCL_MASK);
    wait_us (RECOVER_HALF_US);
    out_w (SIO_GPIO_OE_CLR, SDA_MASK);
    wait_us (RECOVER_HALF_US);

    pins_to (FUNCSEL_I2C);
}

static i2c_bus_result_t abort_result (uint32_t source)
{
    i2c_bus_result_t result = I2C_BUS_ERROR;

    if ((source & (ABRT_7B_ADDR_NOACK | ABRT_TXDATA_NOACK)) != 0UL)
    {
        result = I2C_BUS_NACK;
    }
    return result;
}

/* After an abort or the final STOP, wait for STOP_DET and clear it. */
static bool wait_stop (uint32_t start_us)
{
    while ((in_w (IC_RAW_INTR_STAT) & RAW_STOP_DET) == 0UL)
    {
        if (expired (start_us, I2C_TIMEOUT_US))
        {
            return false;
        }
    }
    (void)in_w (IC_CLR_STOP_DET);
    return true;
}

static i2c_bus_result_t write_read (void * p_ctx, uint8_t addr7,
                                   const uint8_t * p_wr, uint32_t wr_len,
                                   uint8_t * p_rd, uint32_t rd_len)
{
    uint32_t start = physics_time_us ();
    uint32_t i;
    uint32_t source;
    i2c_bus_result_t result = I2C_BUS_OK;

    (void)p_ctx;
    if (((wr_len > 0U) && (NULL == p_wr)) || ((rd_len > 0U) && (NULL == p_rd))
        || ((0U == wr_len) && (0U == rd_len)))
    {
        return I2C_BUS_ERROR;
    }

    out_w (IC_ENABLE, 0UL);
    out_w (IC_TAR, (uint32_t)addr7);
    out_w (IC_ENABLE, ENABLE_ENABLE);

    for (i = 0U; (i < wr_len) && (I2C_BUS_OK == result); i++)
    {
        bool last = ((i + 1U) == wr_len);
        uint32_t cmd = (uint32_t)p_wr[i];

        if (last && (0U == rd_len))
        {
            cmd |= CMD_STOP;
        }
        out_w (IC_DATA_CMD, cmd);

        while ((in_w (IC_RAW_INTR_STAT) & RAW_TX_EMPTY) == 0UL)
        {
            if (expired (start, I2C_TIMEOUT_US))
            {
                result = I2C_BUS_TIMEOUT;
                break;
            }
        }
        if (I2C_BUS_OK == result)
        {
            source = in_w (IC_TX_ABRT_SOURCE);
            if (source != 0UL)
            {
                (void)in_w (IC_CLR_TX_ABRT);
                result = abort_result (source);
                if (!wait_stop (start))
                {
                    result = I2C_BUS_TIMEOUT;
                }
            }
            else if (last && (0U == rd_len) && !wait_stop (start))
            {
                result = I2C_BUS_TIMEOUT;
            }
            else
            {
                /* Byte sent. */
            }
        }
    }

    for (i = 0U; (i < rd_len) && (I2C_BUS_OK == result); i++)
    {
        uint32_t cmd = CMD_READ;

        if ((0U == i) && (wr_len > 0U))
        {
            cmd |= CMD_RESTART;
        }
        if ((i + 1U) == rd_len)
        {
            cmd |= CMD_STOP;
        }

        while (in_w (IC_TXFLR) >= TX_FIFO_DEPTH)
        {
            if (expired (start, I2C_TIMEOUT_US))
            {
                result = I2C_BUS_TIMEOUT;
                break;
            }
        }
        if (I2C_BUS_OK != result)
        {
            break;
        }
        out_w (IC_DATA_CMD, cmd);

        while (0UL == in_w (IC_RXFLR))
        {
            source = in_w (IC_TX_ABRT_SOURCE);
            if (source != 0UL)
            {
                (void)in_w (IC_CLR_TX_ABRT);
                result = abort_result (source);
                (void)wait_stop (start);
                break;
            }
            if (expired (start, I2C_TIMEOUT_US))
            {
                result = I2C_BUS_TIMEOUT;
                break;
            }
        }
        if (I2C_BUS_OK == result)
        {
            p_rd[i] = (uint8_t)(in_w (IC_DATA_CMD) & 0xFFUL);
        }
    }

    if ((I2C_BUS_OK == result) && (rd_len > 0U) && !wait_stop (start))
    {
        result = I2C_BUS_TIMEOUT;
    }

    if (I2C_BUS_TIMEOUT == result)
    {
        abort_transfer ();
    }

    return result;
}

void i2c_rp2040_init (i2c_bus_t * p_bus)
{
    uint32_t pad = PAD_IE | PAD_DRIVE_4MA | PAD_PUE | PAD_SCHMITT;

    out_w (I2C_RESETS_RESET + LSM303_ALIAS_SET, I2C_RESET_I2C1);
    out_w (I2C_RESETS_RESET + LSM303_ALIAS_CLR, I2C_RESET_I2C1);
    while ((in_w (I2C_RESETS_DONE) & I2C_RESET_I2C1) == 0UL)
    {
        /* Wait for I2C1 to leave reset. */
    }

    out_w (IC_ENABLE, 0UL);
    out_w (IC_CON, CON_MASTER | CON_SPEED_FAST | CON_RESTART_EN
                   | CON_SLAVE_DISABLE | CON_TX_EMPTY_CTRL);
    out_w (IC_TX_TL, 0UL);
    out_w (IC_RX_TL, 0UL);
    out_w (IC_INTR_MASK, 0UL);
    out_w (IC_FS_SCL_HCNT, FS_HCNT);
    out_w (IC_FS_SCL_LCNT, FS_LCNT);
    out_w (IC_FS_SPKLEN, FS_SPKLEN);
    out_w (IC_SDA_HOLD, SDA_TX_HOLD);

    out_w (PAD_ADDR (I2C_SDA_PIN), pad);
    out_w (PAD_ADDR (I2C_SCL_PIN), pad);
    pins_to (FUNCSEL_I2C);

    if (NULL != p_bus)
    {
        p_bus->p_ctx = NULL;
        p_bus->write_read = write_read;
        p_bus->recover = recover;
    }
}

/*** end of file ***/
