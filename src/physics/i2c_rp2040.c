/** @file i2c_rp2040.c
 *
 * @brief Interrupt-driven RP2040 I2C1 master on GP2 (SDA) / GP3 (SCL), 400 kHz.
 *
 * Transfer sequence follows pico-sdk 2.x hardware_i2c, with register offsets
 * from the pinned SDK's hardware/regs/i2c.h. IC_CON.TX_EMPTY_CTRL is set, so
 * TX_EMPTY means the last byte has left the shift register.
 */

/* No <stddef.h>: the kernel headers define size_t and NULL themselves. */
#include <tk/tkernel.h>
#include <tk/smp.h>

#include <stdbool.h>
#include <stdint.h>

#include "platform/alarm.h"
#include "platform/log.h"
#include "platform/static_assert.h"

#include "i2c_rp2040.h"
#include "lsm303.h"

/* Base addresses (RP2040 datasheet section 2.2). */
#define I2C1_BASE_ADDR       (0x40048000UL)
#define IO_BANK0_ADDR        (0x40014000UL)
#define PADS_BANK0_ADDR      (0x4001C000UL)
#define I2C_RESETS_ADDR      (0x4000C000UL)
#define SIO_ADDR             (0xD0000000UL)
#define TIMER_BASE_ADDR      (0x40054000UL)

/* I2C register offsets. */
#define IC_CON               (I2C1_BASE_ADDR + 0x00UL)
#define IC_TAR               (I2C1_BASE_ADDR + 0x04UL)
#define IC_DATA_CMD          (I2C1_BASE_ADDR + 0x10UL)
#define IC_FS_SCL_HCNT       (I2C1_BASE_ADDR + 0x1CUL)
#define IC_FS_SCL_LCNT       (I2C1_BASE_ADDR + 0x20UL)
#define IC_INTR_MASK         (I2C1_BASE_ADDR + 0x30UL)
#define IC_RAW_INTR_STAT     (I2C1_BASE_ADDR + 0x34UL)
#define IC_RX_TL             (I2C1_BASE_ADDR + 0x38UL)
#define IC_TX_TL             (I2C1_BASE_ADDR + 0x3CUL)
#define IC_CLR_INTR          (I2C1_BASE_ADDR + 0x40UL)
#define IC_CLR_TX_ABRT       (I2C1_BASE_ADDR + 0x54UL)
#define IC_CLR_STOP_DET      (I2C1_BASE_ADDR + 0x60UL)
#define IC_ENABLE            (I2C1_BASE_ADDR + 0x6CUL)
#define IC_TXFLR             (I2C1_BASE_ADDR + 0x74UL)
#define IC_RXFLR             (I2C1_BASE_ADDR + 0x78UL)
#define IC_SDA_HOLD          (I2C1_BASE_ADDR + 0x7CUL)
#define IC_TX_ABRT_SOURCE    (I2C1_BASE_ADDR + 0x80UL)
#define IC_FS_SPKLEN         (I2C1_BASE_ADDR + 0xA0UL)
#define IC_ENABLE_STATUS     (I2C1_BASE_ADDR + 0x9CUL)

/* I2C bits. */
#define CON_MASTER           (0x001UL)
#define CON_SPEED_FAST       (0x004UL)   /* SPEED field = 2.            */
#define CON_RESTART_EN       (0x020UL)
#define CON_SLAVE_DISABLE    (0x040UL)
#define CON_TX_EMPTY_CTRL    (0x100UL)
#define CMD_READ             (0x100UL)
#define CMD_STOP             (0x200UL)
#define CMD_RESTART          (0x400UL)
#define INTR_RX_OVER         (0x002UL)
#define INTR_RX_FULL         (0x004UL)
#define INTR_TX_OVER         (0x008UL)
#define INTR_TX_EMPTY        (0x010UL)
#define INTR_TX_ABRT         (0x040UL)
#define INTR_STOP_DET        (0x200UL)
#define INTR_FIFO_ERR        (INTR_RX_OVER | INTR_TX_OVER)
#define INTR_BASE_MASK       (INTR_STOP_DET | INTR_TX_ABRT | INTR_FIFO_ERR)
#define ENABLE_ENABLE        (0x1UL)
#define ENABLE_ABORT         (0x2UL)
#define ENABLE_STATUS_IC_EN  (0x1UL)
#define ABRT_7B_ADDR_NOACK   (0x001UL)
#define ABRT_TXDATA_NOACK    (0x008UL)
#define ABRT_CAUSE_MASK      (0x0001FFFFUL)
#define TX_FIFO_DEPTH        (16UL)
#define RX_FIFO_DEPTH        (16UL)

/* 400 kHz at clk_sys 125 MHz, same arithmetic as pico-sdk
 * i2c_set_baudrate: period = 313, lcnt = 3/5, hold = 3 * f / 10 MHz + 1. */
#define FS_LCNT              (187UL)
#define FS_HCNT              (126UL)
#define FS_SPKLEN            (11UL)
#define SDA_TX_HOLD          (38UL)

/* GPIO. */
#define GPIO_CTRL_ADDR(pin)  (IO_BANK0_ADDR + 0x04UL + (8UL * (uint32_t)(pin)))
#define PAD_ADDR(pin)       (PADS_BANK0_ADDR + 0x04UL + (4UL * (uint32_t)(pin)))
#define FUNCSEL_I2C          (3UL)
#define FUNCSEL_SIO          (5UL)
#define PAD_IE               (1UL << 6U)
#define PAD_DRIVE_4MA        (1UL << 4U)
#define PAD_PUE              (1UL << 3U)
#define PAD_SCHMITT          (1UL << 1U)
#define SIO_GPIO_IN          (SIO_ADDR + 0x04UL)
#define SIO_GPIO_OUT_CLR     (SIO_ADDR + 0x18UL)
#define SIO_GPIO_OE_SET      (SIO_ADDR + 0x24UL)
#define SIO_GPIO_OE_CLR      (SIO_ADDR + 0x28UL)

/* Resets. Atomic set/clear aliases are valid for peripheral registers. */
#define I2C_RESETS_RESET     (I2C_RESETS_ADDR + 0x0UL)
#define I2C_RESETS_DONE      (I2C_RESETS_ADDR + 0x8UL)
#define I2C_RESET_I2C1       (1UL << 4U)
#define ALIAS_SET_OFFSET     (0x2000UL)
#define ALIAS_CLR_OFFSET     (0x3000UL)

/* Timer alarm for the transfer deadline. */
#define TIMER_INTR_ADDR      (TIMER_BASE_ADDR + 0x34UL)
#define TIMER_INTE_ADDR      (TIMER_BASE_ADDR + 0x38UL)
#define TIMER_ALARM_ADDR(n)  (TIMER_BASE_ADDR + 0x10UL + (4UL * (uint32_t)(n)))
#define I2C_TIMEOUT_ALARM_BIT (1UL << I2C_TIMEOUT_ALARM_NUM)

#define SDA_MASK             (1UL << I2C_SDA_PIN)
#define SCL_MASK             (1UL << I2C_SCL_PIN)
#define RECOVER_HALF_US      (5U)        /* 100 kHz bit-banged clock.   */
#define ABORT_WAIT_US        (1000U)
#define DISABLE_WAIT_US      (1000U)
#define RESET_WAIT_US        (1000U)
#define RECOVER_PULSES       (9U)        /* I2C spec 3.1.16 bus clear.  */
#define DATA_BYTE_MASK       (0xFFUL)    /* IC_DATA_CMD bits 7:0.       */
#define US_PER_MS            (1000U)
#define I2C_RP2040_IRQ_PRI   (2)
#define I2C_FLG_DONE         (1U)
#define I2C_FLG_TIMEOUT      (2U)

#define XFER_FLAG_ACTIVE     (1U << 0U)
#define XFER_FLAG_DONE       (1U << 1U)
#define XFER_FLAG_ABORTING   (1U << 2U)
#define XFER_FLAG_RETIRED    (1U << 3U)
#define I2C_XFER_META_BYTES  (32U)
#define I2C_XFER_PTR_COUNT   (2U)

typedef struct
{
    uint32_t wr_len;
    uint32_t rd_len;
    uint32_t tx_sent;
    uint32_t rd_issued;
    uint32_t rd_done;
    uint32_t start_us;
    uint32_t abort_source;
    uint8_t  result;
    uint8_t  flags;
} i2c_xfer_meta_t;

typedef struct
{
    const uint8_t * p_wr;
    uint8_t *       p_rd;
    i2c_xfer_meta_t meta;
} i2c_xfer_t;

typedef struct
{
    T_SPLOCK   lock;
    ID         done_flg;
    i2c_xfer_t xfer;
    bool       b_irq_ready;
} i2c_driver_t;

STATIC_ASSERT (sizeof (i2c_xfer_meta_t) == I2C_XFER_META_BYTES,
               i2c_xfer_meta_size);
STATIC_ASSERT (sizeof (i2c_xfer_t)
                   == ((I2C_XFER_PTR_COUNT * sizeof (void *))
                       + I2C_XFER_META_BYTES),
               i2c_xfer_packed_size);
STATIC_ASSERT (I2C_TIMEOUT_US >= US_PER_MS, i2c_timeout_at_least_one_ms);
STATIC_ASSERT (XFER_FLAG_RETIRED <= UINT8_MAX, i2c_xfer_flags_fit_uint8);

static i2c_driver_t g_i2c;

uint32_t physics_time_us (void)
{
    return in_w (TIMER_TIMERAWL);
}

static bool expired (uint32_t start_us, uint32_t limit_us)
{
    return (physics_time_us () - start_us) >= limit_us;   /* Wrap-safe. */
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

/* Status is cleared only while the controller is disabled. */
static bool disable_controller (void)
{
    out_w (IC_ENABLE, 0UL);
    if ((in_w (IC_ENABLE_STATUS) & ENABLE_STATUS_IC_EN) != 0UL)
    {
        uint32_t const start = physics_time_us ();

        while ((in_w (IC_ENABLE_STATUS) & ENABLE_STATUS_IC_EN) != 0UL)
        {
            if (expired (start, DISABLE_WAIT_US))
            {
                return false;
            }
        }
    }
    (void)in_w (IC_CLR_TX_ABRT);
    (void)in_w (IC_CLR_STOP_DET);
    (void)in_w (IC_CLR_INTR);
    return true;
}

/* A failed disable is retried by the next start, which resets the block. */
static void abort_transfer (void)
{
    uint32_t start = physics_time_us ();

    out_w (IC_INTR_MASK, 0UL);
    out_w (IC_ENABLE, ENABLE_ENABLE | ENABLE_ABORT);
    while (((in_w (IC_ENABLE) & ENABLE_ABORT) != 0UL)
           && !expired (start, ABORT_WAIT_US))
    {
        /* Wait for the controller to finish the abort. */
    }
    (void)disable_controller ();
}

/* Buffers stay set: every ISR path checks ACTIVE before touching them. */
static void retire_transfer_locked (void)
{
    g_i2c.xfer.meta.flags = (uint8_t)((g_i2c.xfer.meta.flags
                                       & (~XFER_FLAG_ACTIVE))
                                      | XFER_FLAG_RETIRED);
    out_w (IC_INTR_MASK, 0UL);
    DisableInt (I2C_RP2040_IRQ);
}

static void finish_transfer_locked (i2c_bus_result_t res, uint32_t extra_flags)
{
    g_i2c.xfer.meta.result = (uint8_t)res;
    g_i2c.xfer.meta.flags  = (uint8_t)((g_i2c.xfer.meta.flags
                                        & (~XFER_FLAG_ACTIVE))
                                       | XFER_FLAG_DONE | extra_flags);
    out_w (IC_INTR_MASK, 0UL);
}

static void drain_rx_fifo_locked (void)
{
    uint32_t const limit = g_i2c.xfer.meta.rd_issued - g_i2c.xfer.meta.rd_done;
    uint32_t       avail = 0U;

    if (0U == limit)
    {
        return;
    }
    avail = in_w (IC_RXFLR);
    if (avail > limit)
    {
        avail = limit;
    }
    while (avail > 0U)
    {
        g_i2c.xfer.p_rd[g_i2c.xfer.meta.rd_done] =
            (uint8_t)(in_w (IC_DATA_CMD) & DATA_BYTE_MASK);
        g_i2c.xfer.meta.rd_done++;
        avail--;
    }
}

static void feed_tx_fifo_locked (void)
{
    uint32_t const total_cmds = g_i2c.xfer.meta.wr_len + g_i2c.xfer.meta.rd_len;
    uint32_t const tx_level   = in_w (IC_TXFLR);
    uint32_t       tx_room    = 0U;

    if (tx_level < TX_FIFO_DEPTH)
    {
        tx_room = (uint32_t)(TX_FIFO_DEPTH - tx_level);
    }

    while ((g_i2c.xfer.meta.tx_sent < total_cmds) && (tx_room > 0U))
    {
        uint32_t cmd = 0UL;

        if (g_i2c.xfer.meta.tx_sent < g_i2c.xfer.meta.wr_len)
        {
            uint32_t const wr_idx = g_i2c.xfer.meta.tx_sent;

            cmd = (uint32_t)g_i2c.xfer.p_wr[wr_idx];
            if (((wr_idx + 1U) == g_i2c.xfer.meta.wr_len)
                && (0U == g_i2c.xfer.meta.rd_len))
            {
                cmd |= CMD_STOP;
            }
            out_w (IC_DATA_CMD, cmd);
            g_i2c.xfer.meta.tx_sent++;
            tx_room--;
        }
        else
        {
            uint32_t const rd_idx   = g_i2c.xfer.meta.tx_sent
                                      - g_i2c.xfer.meta.wr_len;
            uint32_t const inflight = g_i2c.xfer.meta.rd_issued
                                      - g_i2c.xfer.meta.rd_done;

            if (inflight >= RX_FIFO_DEPTH)
            {
                break;
            }
            cmd = CMD_READ;
            if ((0U == rd_idx) && (g_i2c.xfer.meta.wr_len > 0U))
            {
                cmd |= CMD_RESTART;
            }
            if ((rd_idx + 1U) == g_i2c.xfer.meta.rd_len)
            {
                cmd |= CMD_STOP;
            }
            out_w (IC_DATA_CMD, cmd);
            g_i2c.xfer.meta.tx_sent++;
            g_i2c.xfer.meta.rd_issued++;
            tx_room--;
        }
    }
}

static void arm_intr_mask_locked (void)
{
    uint32_t const total_cmds = g_i2c.xfer.meta.wr_len + g_i2c.xfer.meta.rd_len;
    uint32_t       mask       = INTR_BASE_MASK;

    if (g_i2c.xfer.meta.tx_sent < total_cmds)
    {
        uint32_t const inflight = g_i2c.xfer.meta.rd_issued
                                  - g_i2c.xfer.meta.rd_done;

        if ((g_i2c.xfer.meta.tx_sent >= g_i2c.xfer.meta.wr_len)
            && (inflight >= RX_FIFO_DEPTH))
        {
            mask |= INTR_RX_FULL;
        }
        else
        {
            mask |= INTR_TX_EMPTY;
        }
    }
    if (g_i2c.xfer.meta.rd_done < g_i2c.xfer.meta.rd_issued)
    {
        mask |= INTR_RX_FULL;
    }

    out_w (IC_INTR_MASK, mask);
}

static bool service_transfer_locked (void)
{
    uint32_t const raw        = in_w (IC_RAW_INTR_STAT);
    uint32_t const abrt_src   = in_w (IC_TX_ABRT_SOURCE) & ABRT_CAUSE_MASK;
    uint32_t const total_cmds = g_i2c.xfer.meta.wr_len + g_i2c.xfer.meta.rd_len;
    bool const     b_abrt     = ((raw & INTR_TX_ABRT) != 0UL)
                                || (abrt_src != 0UL);
    bool const     b_stop     = (raw & INTR_STOP_DET) != 0UL;
    bool const     b_all_sent = (g_i2c.xfer.meta.tx_sent == total_cmds);

    if (b_abrt)
    {
        if (0U == (g_i2c.xfer.meta.flags & XFER_FLAG_ABORTING))
        {
            g_i2c.xfer.meta.abort_source = abrt_src;
            g_i2c.xfer.meta.flags        = (uint8_t)(g_i2c.xfer.meta.flags
                                                     | XFER_FLAG_ABORTING);
        }
        (void)in_w (IC_CLR_TX_ABRT);
    }

    if ((g_i2c.xfer.meta.flags & XFER_FLAG_ABORTING) != 0U)
    {
        if (b_stop)
        {
            bool const b_nack = (g_i2c.xfer.meta.abort_source
                                 & (ABRT_7B_ADDR_NOACK | ABRT_TXDATA_NOACK))
                                != 0UL;

            (void)in_w (IC_CLR_STOP_DET);
            finish_transfer_locked (b_nack ? I2C_BUS_NACK : I2C_BUS_ERROR, 0U);
            return true;
        }
        out_w (IC_INTR_MASK, INTR_STOP_DET);
        return false;
    }

    if ((raw & INTR_FIFO_ERR) != 0UL)
    {
        (void)in_w (IC_CLR_INTR);
        finish_transfer_locked (I2C_BUS_ERROR, XFER_FLAG_RETIRED);
        return true;
    }

    drain_rx_fifo_locked ();
    feed_tx_fifo_locked ();

    if (b_stop)
    {
        (void)in_w (IC_CLR_STOP_DET);
        if (b_all_sent)
        {
            drain_rx_fifo_locked ();
            finish_transfer_locked (
                (g_i2c.xfer.meta.rd_done == g_i2c.xfer.meta.rd_len)
                    ? I2C_BUS_OK
                    : I2C_BUS_ERROR,
                0U);
            return true;
        }
    }

    arm_intr_mask_locked ();
    return false;
}

static void i2c1_isr (UINT intno)
{
    UINT saved    = 0U;
    bool b_wake   = false;
    ID   done_flg = 0;

    (void)intno;
    (void)ISpinLock (&g_i2c.lock, &saved);
    if ((g_i2c.xfer.meta.flags & XFER_FLAG_ACTIVE) == 0U)
    {
        out_w (IC_INTR_MASK, 0UL);
        (void)in_w (IC_CLR_TX_ABRT);
        (void)in_w (IC_CLR_STOP_DET);
        (void)in_w (IC_CLR_INTR);
    }
    else
    {
        b_wake   = service_transfer_locked ();
        done_flg = g_i2c.done_flg;
    }
    ClearInt (I2C_RP2040_IRQ);
    (void)ISpinUnlock (&g_i2c.lock, saved);

    /* Signal after releasing the spinlock so the kernel lock never nests
     * inside ISpinLock. */
    if (b_wake && (done_flg > 0))
    {
        (void)tk_set_flg (done_flg, I2C_FLG_DONE);
    }
}

/* Only an ACTIVE transfer past its deadline is signalled, so a late alarm
 * cannot time out a later transfer. */
static void i2c_timeout_isr (UINT intno)
{
    UINT saved    = 0U;
    bool b_expire = false;
    ID   done_flg = 0;

    (void)intno;
    out_w (TIMER_INTR_ADDR, I2C_TIMEOUT_ALARM_BIT);
    (void)ISpinLock (&g_i2c.lock, &saved);
    b_expire = ((g_i2c.xfer.meta.flags & XFER_FLAG_ACTIVE) != 0U)
               && expired (g_i2c.xfer.meta.start_us, I2C_TIMEOUT_US);
    done_flg = g_i2c.done_flg;
    (void)ISpinUnlock (&g_i2c.lock, saved);

    if (b_expire && (done_flg > 0))
    {
        (void)tk_set_flg (done_flg, I2C_FLG_TIMEOUT);
    }
}

/* Bus clear (I2C spec 3.1.16): clock SCL up to 9 times until the slave
 * releases SDA, then send STOP. Pins are driven open-drain through SIO:
 * output low, toggled by output-enable, external/internal pull-ups high. */
static void recover (void * p_ctx)
{
    uint32_t pulse = 0U;
    UINT     saved = 0U;

    (void)p_ctx;
    (void)ISpinLock (&g_i2c.lock, &saved);
    retire_transfer_locked ();
    (void)ISpinUnlock (&g_i2c.lock, saved);

    (void)disable_controller ();
    ClearInt (I2C_RP2040_IRQ);

    out_w (SIO_GPIO_OE_CLR, SDA_MASK | SCL_MASK);
    out_w (SIO_GPIO_OUT_CLR, SDA_MASK | SCL_MASK);
    pins_to (FUNCSEL_SIO);
    wait_us (RECOVER_HALF_US);

    for (pulse = 0U; (pulse < RECOVER_PULSES)
                     && ((in_w (SIO_GPIO_IN) & SDA_MASK) == 0UL);
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
    (void)in_w (IC_CLR_INTR);
    ClearInt (I2C_RP2040_IRQ);
}

static bool reset_and_configure_i2c1 (void)
{
    uint32_t const start = physics_time_us ();

    out_w (I2C_RESETS_RESET + ALIAS_SET_OFFSET, I2C_RESET_I2C1);
    out_w (I2C_RESETS_RESET + ALIAS_CLR_OFFSET, I2C_RESET_I2C1);
    while ((in_w (I2C_RESETS_DONE) & I2C_RESET_I2C1) == 0UL)
    {
        if (expired (start, RESET_WAIT_US))
        {
            return false;
        }
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
    (void)in_w (IC_CLR_INTR);

    return true;
}

static bool start_transfer (uint8_t addr7, const uint8_t * p_wr,
                            uint32_t wr_len, uint8_t * p_rd, uint32_t rd_len,
                            uint32_t * p_start_us)
{
    UINT     saved    = 0U;
    UINT     ptn      = 0U;
    uint32_t start_us = 0U;

    (void)tk_wai_flg (g_i2c.done_flg, I2C_FLG_DONE | I2C_FLG_TIMEOUT,
                      TWF_ORW | TWF_BITCLR, &ptn, TMO_POL);
    DisableInt (I2C_RP2040_IRQ);
    out_w (IC_INTR_MASK, 0UL);
    if (!disable_controller ())
    {
        if (!reset_and_configure_i2c1 ())
        {
            return false;
        }
    }
    out_w (IC_TAR, (uint32_t)addr7);

    (void)ISpinLock (&g_i2c.lock, &saved);
    out_w (IC_ENABLE, ENABLE_ENABLE);
    (void)in_w (IC_CLR_TX_ABRT);
    (void)in_w (IC_CLR_STOP_DET);
    (void)in_w (IC_CLR_INTR);
    ClearInt (I2C_RP2040_IRQ);

    g_i2c.xfer.p_wr              = p_wr;
    g_i2c.xfer.p_rd              = p_rd;
    g_i2c.xfer.meta.wr_len       = wr_len;
    g_i2c.xfer.meta.rd_len       = rd_len;
    g_i2c.xfer.meta.tx_sent      = 0U;
    g_i2c.xfer.meta.rd_issued    = 0U;
    g_i2c.xfer.meta.rd_done      = 0U;
    g_i2c.xfer.meta.abort_source = 0UL;
    g_i2c.xfer.meta.result       = (uint8_t)I2C_BUS_OK;
    g_i2c.xfer.meta.flags        = (uint8_t)XFER_FLAG_ACTIVE;

    feed_tx_fifo_locked ();
    arm_intr_mask_locked ();
    start_us = physics_time_us ();
    g_i2c.xfer.meta.start_us = start_us;
    out_w (TIMER_INTR_ADDR, I2C_TIMEOUT_ALARM_BIT);
    out_w (TIMER_ALARM_ADDR (I2C_TIMEOUT_ALARM_NUM), start_us + I2C_TIMEOUT_US);
    out_w (TIMER_INTE_ADDR + ALIAS_SET_OFFSET, I2C_TIMEOUT_ALARM_BIT);
    EnableInt (I2C_RP2040_IRQ, I2C_RP2040_IRQ_PRI);
    (void)ISpinUnlock (&g_i2c.lock, saved);

    *p_start_us = start_us;
    return true;
}

static i2c_bus_result_t wait_transfer (uint32_t start_us, bool * p_needs_abort)
{
    i2c_bus_result_t result    = I2C_BUS_ERROR;
    bool             b_settled = false;

    *p_needs_abort = false;
    while (!b_settled)
    {
        UINT saved     = 0U;
        UINT ptn       = 0U;
        ER   er        = E_OK;
        bool b_expired = false;

        if (!expired (start_us, I2C_TIMEOUT_US))
        {
            er = tk_wai_flg (g_i2c.done_flg, I2C_FLG_DONE | I2C_FLG_TIMEOUT,
                             TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
        }

        /* Re-check under lock: a higher-priority task may have preempted
         * us while the ISR finished this transfer. */
        (void)ISpinLock (&g_i2c.lock, &saved);
        b_expired = expired (start_us, I2C_TIMEOUT_US);
        if ((g_i2c.xfer.meta.flags & XFER_FLAG_DONE) != 0U)
        {
            result         = (i2c_bus_result_t)g_i2c.xfer.meta.result;
            *p_needs_abort = ((g_i2c.xfer.meta.flags & XFER_FLAG_RETIRED)
                              != 0U);
            b_settled      = true;
        }
        else if ((E_OK != er) || b_expired)
        {
            retire_transfer_locked ();
            result         = b_expired ? I2C_BUS_TIMEOUT : I2C_BUS_ERROR;
            *p_needs_abort = true;
            b_settled      = true;
        }
        else
        {
            /* Spurious wake before the deadline: wait again. */
        }
        (void)ISpinUnlock (&g_i2c.lock, saved);
    }

    return result;
}

static i2c_bus_result_t write_read (void * p_ctx, uint8_t addr7,
                                    const uint8_t * p_wr, uint32_t wr_len,
                                    uint8_t * p_rd, uint32_t rd_len)
{
    uint32_t         start_us      = 0U;
    bool             b_needs_abort = false;
    i2c_bus_result_t result        = I2C_BUS_OK;

    (void)p_ctx;
    if (!g_i2c.b_irq_ready || ((wr_len > 0U) && (NULL == p_wr))
        || ((rd_len > 0U) && (NULL == p_rd))
        || ((0U == wr_len) && (0U == rd_len))
        || (wr_len > (UINT32_MAX - rd_len)))
    {
        return I2C_BUS_ERROR;
    }

    if (!start_transfer (addr7, p_wr, wr_len, p_rd, rd_len, &start_us))
    {
        return I2C_BUS_ERROR;
    }
    result = wait_transfer (start_us, &b_needs_abort);
    out_w (TIMER_INTE_ADDR + ALIAS_CLR_OFFSET, I2C_TIMEOUT_ALARM_BIT);
    out_w (TIMER_INTR_ADDR, I2C_TIMEOUT_ALARM_BIT);
    ClearInt (I2C_TIMEOUT_ALARM_IRQ);

    if (b_needs_abort)
    {
        DisableInt (I2C_RP2040_IRQ);
        abort_transfer ();
        ClearInt (I2C_RP2040_IRQ);
    }
    else if (I2C_BUS_ERROR == result)
    {
        (void)disable_controller ();
    }
    else
    {
        /* Transfer finished cleanly on STOP_DET. */
    }

    return result;
}

void i2c_rp2040_init (i2c_bus_t * p_bus)
{
    uint32_t const pad  = PAD_IE | PAD_DRIVE_4MA | PAD_PUE | PAD_SCHMITT;
    T_DINT const dint_irq = {
        .intatr = TA_HLNG,
        .inthdr = (FP)i2c1_isr,
    };
    T_DINT const dint_alarm = {
        .intatr = TA_HLNG,
        .inthdr = (FP)i2c_timeout_isr,
    };
    i2c_xfer_t const zero_xfer = { 0 };

    DisableInt (I2C_RP2040_IRQ);
    g_i2c.b_irq_ready = false;

    bool const b_reset_ok = reset_and_configure_i2c1 ();
    if (!b_reset_ok)
    {
        LOG_ERROR (LOG_TASK_PHYSICS, LOG_MOD_IMU, "I2C1 reset timed out");
    }

    out_w (PAD_ADDR (I2C_SDA_PIN), pad);
    out_w (PAD_ADDR (I2C_SCL_PIN), pad);
    pins_to (FUNCSEL_I2C);

    (void)InitSpinLock (&g_i2c.lock);
    g_i2c.xfer = zero_xfer;

    if (g_i2c.done_flg <= 0)
    {
        T_CFLG const cflg = {
            .exinf   = NULL,
            .flgatr  = TA_TFIFO,
            .iflgptn = 0U,
        };
        ID const flg = tk_cre_flg (&cflg);

        if (flg > 0)
        {
            g_i2c.done_flg = flg;
        }
        else
        {
            LOG_ERROR (LOG_TASK_PHYSICS, LOG_MOD_IMU,
                       "I2C1 event flag create failed");
        }
    }

    if (b_reset_ok && (g_i2c.done_flg > 0))
    {
        ER const er_irq   = tk_def_int (I2C_RP2040_IRQ, &dint_irq);
        ER const er_alarm = tk_def_int (I2C_TIMEOUT_ALARM_IRQ, &dint_alarm);

        if ((E_OK == er_irq) && (E_OK == er_alarm))
        {
            ClearInt (I2C_RP2040_IRQ);
            EnableInt (I2C_RP2040_IRQ, I2C_RP2040_IRQ_PRI);
            ClearInt (I2C_TIMEOUT_ALARM_IRQ);
            EnableInt (I2C_TIMEOUT_ALARM_IRQ, I2C_TIMEOUT_ALARM_PRI);
            g_i2c.b_irq_ready = true;
        }
        else
        {
            LOG_ERROR (LOG_TASK_PHYSICS, LOG_MOD_IMU,
                       "I2C1 interrupt registration failed");
        }
    }

    if (NULL != p_bus)
    {
        p_bus->p_ctx      = NULL;
        p_bus->write_read = write_read;
        p_bus->recover    = recover;
    }
}

/*** end of file ***/
