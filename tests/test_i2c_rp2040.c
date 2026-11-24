/** @file test_i2c_rp2040.c
 *
 * @brief Host tests for the interrupt-driven RP2040 I2C1 driver against a
 *        scripted register, NVIC, and event-flag model.
 */

#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tk/flg_fake.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_rp2040.h"
#include "lsm303.h"
#include "platform/alarm.h"
#include "platform/log.h"

#define I2C1_BASE           (0x40048000UL)
#define I2C_REG_COUNT       (64U)
#define REG_DATA_CMD        (I2C1_BASE + 0x10UL)
#define REG_INTR_STAT       (I2C1_BASE + 0x2CUL)
#define REG_INTR_MASK       (I2C1_BASE + 0x30UL)
#define REG_RAW_INTR        (I2C1_BASE + 0x34UL)
#define REG_CLR_INTR        (I2C1_BASE + 0x40UL)
#define REG_CLR_TX_ABRT     (I2C1_BASE + 0x54UL)
#define REG_CLR_STOP_DET    (I2C1_BASE + 0x60UL)
#define REG_ENABLE          (I2C1_BASE + 0x6CUL)
#define REG_TXFLR           (I2C1_BASE + 0x74UL)
#define REG_RXFLR           (I2C1_BASE + 0x78UL)
#define REG_TX_ABRT_SRC     (I2C1_BASE + 0x80UL)
#define REG_RESETS_RESET_SET (0x4000E000UL)
#define REG_RESETS_DONE     (0x4000C008UL)
#define REG_SIO_GPIO_IN     (0xD0000004UL)
#define RESET_I2C1_BIT      (1U << 4U)

#define CMD_READ            (0x100U)
#define CMD_STOP            (0x200U)
#define CMD_RESTART         (0x400U)

#define RAW_RX_OVER         (0x002U)
#define RAW_RX_FULL         (0x004U)
#define RAW_TX_OVER         (0x008U)
#define RAW_TX_EMPTY        (0x010U)
#define RAW_TX_ABRT         (0x040U)
#define RAW_STOP_DET        (0x200U)
#define RAW_SW_CLEARABLE    (RAW_RX_OVER | RAW_TX_OVER | RAW_TX_ABRT | RAW_STOP_DET)

#define ABRT_7B_ADDR_NOACK  (0x001U)
#define ABRT_TXDATA_NOACK   (0x008U)
#define ABRT_ARB_LOST       (0x1000U)
#define ABRT_FLUSH_CNT_BITS (0x01800000U)

#define DEADLINE_US         (I2C_TIMEOUT_US)
#define PREEMPT_US          (DEADLINE_US + 1U)
#define TX_FIFO_FULL        (16U)
#define READ_LIMIT          (100000U)
#define MAX_EVENTS          (16U)
#define MAX_FIFO_BYTES      (64U)
#define MAX_TX_LOG          (64U)
#define US_PER_MS           (1000U)
#define TIMER_BASE_ADDR     (0x40054000UL)
#define REG_TIMER_ALARM1    (TIMER_BASE_ADDR + 0x14UL)
#define REG_TIMER_INTE      (TIMER_BASE_ADDR + 0x38UL)
#define REG_TIMER_INTE_SET  (REG_TIMER_INTE + 0x2000UL)
#define REG_TIMER_INTE_CLR  (REG_TIMER_INTE + 0x3000UL)
#define REG_ENABLE_STATUS   (I2C1_BASE + 0x9CUL)
#define REG_CON             (I2C1_BASE + 0x00UL)
#define REG_RX_TL           (I2C1_BASE + 0x38UL)
#define REG_TX_TL           (I2C1_BASE + 0x3CUL)
#define REG_FS_SCL_HCNT     (I2C1_BASE + 0x1CUL)
#define REG_FS_SCL_LCNT     (I2C1_BASE + 0x20UL)
#define REG_FS_SPKLEN       (I2C1_BASE + 0xA0UL)
#define REG_SDA_HOLD        (I2C1_BASE + 0x7CUL)
#define ENABLE_EN           (0x1U)
#define ENABLE_ABORT        (0x2U)
#define TIMER_ALARM_BIT     (1U << I2C_TIMEOUT_ALARM_NUM)
#define FLG_DONE_BIT        (1U)
#define FLG_TIMEOUT_BIT     (2U)
#define SHIM_INT_COUNT      (32U)
#define ATTEMPT_ABORT_ALLOWANCE_US (2U * US_PER_MS)
#define ATTEMPT_LIMIT_US    (20U * US_PER_MS)
#define PRE_ARM_LIMIT_US    (2U * US_PER_MS)
#define STUCK_ABORT_LIMIT_US (DEADLINE_US + 2U * US_PER_MS)
#define TIMING_SLACK_US     (100U)
/* Reset values from the pinned SDK's hardware/regs/i2c.h. */
#define SDK_RESET_CON        (0x65U)
#define SDK_RESET_FS_SCL_HCNT (0x6U)
#define SDK_RESET_FS_SCL_LCNT (0xDU)
#define SDK_RESET_FS_SPKLEN  (0x7U)
#define SDK_RESET_SDA_HOLD   (0x1U)
#define SDK_RESET_INTR_MASK  (0x8FFU)
#define CFG_CON              (0x165U)
#define CFG_FS_SCL_HCNT      (126U)
#define CFG_FS_SCL_LCNT      (187U)
#define CFG_FS_SPKLEN        (11U)
#define CFG_SDA_HOLD         (38U)

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

typedef struct
{
    uint32_t delay_us;
    uint32_t raw_set;
    uint32_t abrt_set;
    uint32_t txflr;
    bool     b_set_txflr;
    bool     b_spurious_flg;
    bool     b_preempt_jump;
    uint8_t  rx_len;
    uint8_t  rx_bytes[MAX_FIFO_BYTES];
} hw_event_t;

/* Shim globals referenced by tests/shim/tk/tkernel.h. */
int g_shim_int_enabled = 0;

static FP g_int_handlers[SHIM_INT_COUNT];

ER i2c_test_def_int (UINT intno, const T_DINT * p_definition)
{
    if (intno >= SHIM_INT_COUNT)
    {
        return E_PAR;
    }
    g_int_handlers[intno] = p_definition->inthdr;
    return E_OK;
}

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod,
                char const * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

/* Scripted controller & fake event flag state. */
static uint32_t   g_now_us;
static uint32_t   g_timer_reads;
static uint32_t   g_preempt_read;
static uint32_t   g_preempt_raw;
static uint32_t   g_preempt_txflr;
static bool       g_preempt_set_txflr;
static uint8_t    g_preempt_rx_len;
static uint8_t    g_preempt_rx_bytes[MAX_FIFO_BYTES];

static uint32_t   g_raw;
static uint32_t   g_intr_mask;
static uint32_t   g_txflr;
static uint32_t   g_txflr_on_enable;
static uint32_t   g_abrt_source;
static uint32_t   g_reads;
static uint32_t   g_isr_calls;
static uint32_t   g_wai_flg_calls;
static uint32_t   g_set_flg_calls;
static uint32_t   g_abort_writes;
static bool       g_completion_interrupt_armed;
static uint32_t   g_regs[I2C_REG_COUNT];

static uint8_t    g_rx_fifo[MAX_FIFO_BYTES];
static uint32_t   g_rx_head;
static uint32_t   g_rx_count;

static uint32_t   g_tx_cmds[MAX_TX_LOG];
static uint32_t   g_tx_cmd_count;

static hw_event_t g_events[MAX_EVENTS];
static uint32_t   g_event_count;
static uint32_t   g_event_idx;

static ID         g_flg_id = 1;
static UINT       g_flg_bits;
static bool       g_force_wai_err;
static bool       g_alarm_armed;
static uint32_t   g_alarm_target;
static uint32_t   g_alarm_fires;
static uint32_t   g_timer_inte;
static bool       g_abort_stuck;
static bool       g_controller_wedged;
static bool       g_stale_stop_on_enable;
static bool       g_reset_stuck;
static bool       g_advance_clock_on_alarm_arm;
static uint32_t   g_wai_calls;
static bool       g_cfg_written[I2C_REG_COUNT];
static bool       g_cfg_at_enable[I2C_REG_COUNT];

static void model_reset (void)
{
    memset (g_int_handlers, 0, sizeof (g_int_handlers));
    g_shim_int_enabled           = 0;
    g_now_us                     = 0U;
    g_timer_reads                = 0U;
    g_preempt_read               = 0U;
    g_preempt_raw                = 0U;
    g_preempt_txflr              = 0U;
    g_preempt_set_txflr          = false;
    g_preempt_rx_len             = 0U;
    g_raw                        = 0U;
    g_intr_mask                  = 0U;
    g_txflr                      = 0U;
    g_txflr_on_enable            = 0U;
    g_abrt_source                = 0U;
    g_reads                      = 0U;
    g_isr_calls                  = 0U;
    g_wai_flg_calls              = 0U;
    g_set_flg_calls              = 0U;
    g_abort_writes               = 0U;
    g_completion_interrupt_armed = false;
    g_rx_head                    = 0U;
    g_rx_count                   = 0U;
    g_tx_cmd_count               = 0U;
    g_event_count                = 0U;
    g_event_idx                  = 0U;
    g_flg_bits                   = 0U;
    g_force_wai_err              = false;
    g_alarm_armed                = false;
    g_alarm_target               = 0U;
    g_alarm_fires                = 0U;
    g_timer_inte                 = 0U;
    g_abort_stuck                = false;
    g_controller_wedged          = false;
    g_stale_stop_on_enable       = false;
    g_reset_stuck                = false;
    g_advance_clock_on_alarm_arm = false;
    g_wai_calls                  = 0U;
    memset (g_cfg_written, 0, sizeof (g_cfg_written));
    memset (g_cfg_at_enable, 0, sizeof (g_cfg_at_enable));
    memset (g_regs, 0, sizeof (g_regs));
    memset (g_rx_fifo, 0, sizeof (g_rx_fifo));
    memset (g_tx_cmds, 0, sizeof (g_tx_cmds));
    memset (g_events, 0, sizeof (g_events));
}

static void rx_fifo_push (const uint8_t * p_bytes, uint8_t len)
{
    uint8_t i = 0U;

    for (i = 0U; (i < len) && (g_rx_count < MAX_FIFO_BYTES); i++)
    {
        uint32_t const tail = (g_rx_head + g_rx_count) % MAX_FIFO_BYTES;

        g_rx_fifo[tail] = p_bytes[i];
        g_rx_count++;
    }
    if (g_rx_count > 0U)
    {
        g_raw |= RAW_RX_FULL;
    }
}

static uint8_t rx_fifo_pop (void)
{
    uint8_t byte = 0U;

    if (g_rx_count > 0U)
    {
        byte       = g_rx_fifo[g_rx_head];
        g_rx_head  = (g_rx_head + 1U) % MAX_FIFO_BYTES;
        g_rx_count--;
    }
    if (0U == g_rx_count)
    {
        g_raw &= ~RAW_RX_FULL;
    }
    return byte;
}

static void push_event (hw_event_t evt)
{
    if (g_event_count < MAX_EVENTS)
    {
        g_events[g_event_count] = evt;
        g_event_count++;
    }
}

static void deliver_irq_if_pending (void)
{
    if ((0 != g_shim_int_enabled) && (NULL != g_int_handlers[I2C_RP2040_IRQ])
        && ((g_raw & g_intr_mask) != 0UL))
    {
        void (*isr_fn) (UINT) = (void (*) (UINT))g_int_handlers[I2C_RP2040_IRQ];

        g_isr_calls++;
        isr_fn (I2C_RP2040_IRQ);
    }
}

static void apply_event (const hw_event_t * p_evt)
{
    if (p_evt->b_preempt_jump)
    {
        g_now_us += PREEMPT_US;
    }
    if (p_evt->b_set_txflr)
    {
        g_txflr = p_evt->txflr;
    }
    if (p_evt->rx_len > 0U)
    {
        rx_fifo_push (p_evt->rx_bytes, p_evt->rx_len);
    }
    g_abrt_source |= p_evt->abrt_set;
    g_raw         |= p_evt->raw_set;
    if (p_evt->b_spurious_flg)
    {
        (void)tk_set_flg (g_flg_id, 1U);
    }
    deliver_irq_if_pending ();
}

ID tk_cre_flg (const T_CFLG * pk_cflg)
{
    g_flg_bits = (NULL != pk_cflg) ? pk_cflg->iflgptn : 0U;
    return g_flg_id;
}

ER tk_set_flg (ID flgid, UINT setptn)
{
    if (flgid != g_flg_id)
    {
        return E_ID;
    }
    g_set_flg_calls++;
    g_flg_bits |= setptn;
    return E_OK;
}

static ER take_flg (UINT waiptn, UINT wfmode, UINT * p_flgptn)
{
    if (NULL != p_flgptn)
    {
        *p_flgptn = g_flg_bits;
    }
    if ((wfmode & TWF_BITCLR) != 0U)
    {
        g_flg_bits &= ~waiptn;
    }
    return E_OK;
}

static bool after (uint32_t t_us, uint32_t ref_us)
{
    return (int32_t)(t_us - ref_us) > 0;
}

static bool alarm_pending (void)
{
    return ((g_timer_inte & TIMER_ALARM_BIT) != 0U) && g_alarm_armed;
}

static void fire_alarm (void)
{
    g_alarm_armed = false;
    g_alarm_fires++;
    if (after (g_alarm_target, g_now_us))
    {
        g_now_us = g_alarm_target;
    }
    if (NULL != g_int_handlers[I2C_TIMEOUT_ALARM_IRQ])
    {
        ((void (*) (UINT))g_int_handlers[I2C_TIMEOUT_ALARM_IRQ])
            (I2C_TIMEOUT_ALARM_IRQ);
    }
}

ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout)
{
    g_wai_calls++;
    if (flgid != g_flg_id)
    {
        return E_ID;
    }
    if ((TMO_POL != tmout) && (TMO_FEVR != tmout))
    {
        printf ("  FAIL %s:%d: finite timeout, driver must not wait on ticks\n",
                __FILE__, __LINE__);
        g_failures++;
        return E_PAR;
    }
    if (g_force_wai_err && (TMO_POL != tmout))
    {
        return E_OBJ;
    }
    if ((g_flg_bits & waiptn) != 0U)
    {
        return take_flg (waiptn, wfmode, p_flgptn);
    }
    if (TMO_POL == tmout)
    {
        return E_TMOUT;
    }

    g_wai_flg_calls++;

    /* Deliver any already-pending interrupt (e.g. TX_EMPTY/NACK ready at arm). */
    deliver_irq_if_pending ();

    while ((g_flg_bits & waiptn) == 0U)
    {
        bool const         b_event = (g_event_idx < g_event_count);
        const hw_event_t * p_evt   = b_event ? &g_events[g_event_idx] : NULL;
        uint32_t const     next_us = b_event ? (g_now_us + p_evt->delay_us)
                                             : 0U;

        if (alarm_pending () && (!b_event || after (next_us, g_alarm_target)))
        {
            fire_alarm ();
            continue;
        }
        if (!b_event)
        {
            break;
        }
        g_now_us = next_us;
        g_event_idx++;
        apply_event (p_evt);
    }

    if ((g_flg_bits & waiptn) != 0U)
    {
        return take_flg (waiptn, wfmode, p_flgptn);
    }
    printf ("  FAIL %s:%d: untimed wait has no event or alarm left\n",
            __FILE__, __LINE__);
    g_failures++;
    return E_TMOUT;
}

static uint32_t timer_read (void)
{
    uint32_t value = 0U;

    g_timer_reads++;
    if ((g_preempt_read > 0U) && (g_timer_reads == g_preempt_read))
    {
        g_now_us += PREEMPT_US;
        if (g_preempt_set_txflr)
        {
            g_txflr = g_preempt_txflr;
        }
        if (g_preempt_rx_len > 0U)
        {
            rx_fifo_push (g_preempt_rx_bytes, g_preempt_rx_len);
        }
        g_raw |= g_preempt_raw;
        deliver_irq_if_pending ();
    }
    value = g_now_us;
    g_now_us++;
    return value;
}

static bool in_i2c1 (uint32_t addr)
{
    return (addr >= I2C1_BASE) && (addr < (I2C1_BASE + (4U * I2C_REG_COUNT)));
}

uint32_t shim_reg_read (uint32_t addr)
{
    uint32_t value = 0U;

    g_reads++;
    if (g_reads > READ_LIMIT)
    {
        printf ("  FAIL %s:%d: register loop exceeded read limit\n", __FILE__,
                __LINE__);
        exit (1);
    }
    if (TIMER_TIMERAWL == addr)
    {
        value = timer_read ();
    }
    else if (REG_INTR_STAT == addr)
    {
        value = g_raw & g_intr_mask;
    }
    else if (REG_INTR_MASK == addr)
    {
        value = g_intr_mask;
    }
    else if (REG_RAW_INTR == addr)
    {
        value = g_raw;
    }
    else if (REG_CLR_TX_ABRT == addr)
    {
        g_raw         &= ~RAW_TX_ABRT;
        g_abrt_source  = 0U;
        value          = 0U;
    }
    else if (REG_CLR_STOP_DET == addr)
    {
        g_raw &= ~RAW_STOP_DET;
        value  = 0U;
    }
    else if (REG_CLR_INTR == addr)
    {
        g_raw         &= ~RAW_SW_CLEARABLE;
        g_abrt_source  = 0U;
        value          = 0U;
    }
    else if (REG_RXFLR == addr)
    {
        value = g_rx_count;
    }
    else if (REG_TXFLR == addr)
    {
        value = g_txflr;
    }
    else if (REG_DATA_CMD == addr)
    {
        value = (uint32_t)rx_fifo_pop ();
    }
    else if (REG_TX_ABRT_SRC == addr)
    {
        value = g_abrt_source;
    }
    else if (REG_ENABLE_STATUS == addr)
    {
        value = g_regs[(REG_ENABLE - I2C1_BASE) / 4U] & ENABLE_EN;
    }
    else if (REG_RESETS_DONE == addr)
    {
        value = g_reset_stuck ? ~RESET_I2C1_BIT : 0xFFFFFFFFU;
    }
    else if (REG_SIO_GPIO_IN == addr)
    {
        value = 0xFFFFFFFFU;
    }
    else if (in_i2c1 (addr))
    {
        value = g_regs[(addr - I2C1_BASE) / 4U];
    }
    else
    {
        /* Unmodelled peripheral register. */
    }
    return value;
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    if ((REG_ENABLE == addr) && ((value & ENABLE_EN) != 0U))
    {
        memcpy (g_cfg_at_enable, g_cfg_written, sizeof (g_cfg_written));
    }
    if (REG_INTR_MASK == addr)
    {
        g_intr_mask = value;
        if ((value & RAW_STOP_DET) != 0U)
        {
            g_completion_interrupt_armed = true;
        }
    }
    else if (REG_DATA_CMD == addr)
    {
        if (g_tx_cmd_count < MAX_TX_LOG)
        {
            g_tx_cmds[g_tx_cmd_count] = value;
            g_tx_cmd_count++;
        }
        if (g_txflr < TX_FIFO_FULL)
        {
            g_txflr++;
        }
    }
    else if (REG_ENABLE == addr)
    {
        uint32_t const was_enabled = g_regs[(REG_ENABLE - I2C1_BASE) / 4U]
                                     & ENABLE_EN;

        if ((value & ENABLE_ABORT) != 0UL)
        {
            g_abort_writes++;
            g_controller_wedged = g_abort_stuck;
        }
        value = g_controller_wedged ? (ENABLE_EN | ENABLE_ABORT)
                                    : (value & ~ENABLE_ABORT);
        if (0UL == (value & ENABLE_EN))
        {
            g_txflr    = 0U;
            g_rx_head  = 0U;
            g_rx_count = 0U;
            g_raw     &= ~RAW_RX_FULL;
        }
        else
        {
            if ((0UL == was_enabled) && g_stale_stop_on_enable)
            {
                g_raw |= RAW_STOP_DET;
            }
            if (g_txflr_on_enable > 0U)
            {
                g_txflr = g_txflr_on_enable;
            }
        }
    }
    else if (REG_RESETS_RESET_SET == addr)
    {
        if (((value & RESET_I2C1_BIT) != 0U) && !g_reset_stuck)
        {
            g_controller_wedged                   = false;
            g_regs[(REG_ENABLE - I2C1_BASE) / 4U] = 0U;
            g_txflr                               = 0U;
            g_rx_head                             = 0U;
            g_rx_count                            = 0U;
            g_raw                                 = 0U;
            g_abrt_source                         = 0U;
            g_intr_mask                           = SDK_RESET_INTR_MASK;
            g_regs[(REG_CON - I2C1_BASE) / 4U]    = SDK_RESET_CON;
            g_regs[(REG_TX_TL - I2C1_BASE) / 4U]  = 0U;
            g_regs[(REG_RX_TL - I2C1_BASE) / 4U]  = 0U;
            g_regs[(REG_FS_SCL_HCNT - I2C1_BASE) / 4U] = SDK_RESET_FS_SCL_HCNT;
            g_regs[(REG_FS_SCL_LCNT - I2C1_BASE) / 4U] = SDK_RESET_FS_SCL_LCNT;
            g_regs[(REG_FS_SPKLEN - I2C1_BASE) / 4U]   = SDK_RESET_FS_SPKLEN;
            g_regs[(REG_SDA_HOLD - I2C1_BASE) / 4U]    = SDK_RESET_SDA_HOLD;
            memset (g_cfg_written, 0, sizeof (g_cfg_written));
        }
    }
    else if (REG_TIMER_ALARM1 == addr)
    {
        if (g_advance_clock_on_alarm_arm)
        {
            g_now_us                     = value + PREEMPT_US;
            g_advance_clock_on_alarm_arm = false;
        }
        g_alarm_target = value;
        g_alarm_armed  = after (value, g_now_us);
    }
    else if (REG_TIMER_INTE_SET == addr)
    {
        g_timer_inte |= value;
    }
    else if (REG_TIMER_INTE_CLR == addr)
    {
        g_timer_inte &= ~value;
    }
    else
    {
        /* Other registers recorded below if inside I2C1. */
    }

    if (in_i2c1 (addr))
    {
        g_regs[(addr - I2C1_BASE) / 4U]       = value;
        g_cfg_written[(addr - I2C1_BASE) / 4U] = true;
    }
}

/* A write of one register byte that completes while the task is preempted
 * past the deadline is a success, not a timeout. */
static void test_write_done_during_preemption (void)
{
    i2c_bus_t bus;
    uint8_t   reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("write_done_during_preemption_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_preempt_read = g_timer_reads + 2U;
    g_preempt_raw  = RAW_TX_EMPTY | RAW_STOP_DET;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, NULL, 0U));
    CHECK (g_completion_interrupt_armed);
    CHECK (0U == g_intr_mask);
}

/* A read whose data arrives while the task is preempted past the deadline
 * returns that data. */
static void test_read_done_during_preemption (void)
{
    i2c_bus_t bus;
    uint8_t   reg    = (uint8_t)(LSM303_OUT_X_L_A | LSM303_AUTO_INC_A);
    uint8_t   buf[1] = { 0U };

    printf ("read_done_during_preemption_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_preempt_read        = g_timer_reads + 2U;
    g_preempt_raw         = RAW_STOP_DET;
    g_preempt_rx_len      = 1U;
    g_preempt_rx_bytes[0] = 0x5AU;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x5AU == buf[0]);
    CHECK (g_completion_interrupt_armed);
}

/* A STOP that lands while the task is preempted past the deadline ends a write
 * with no read, and the write is a success. */
static void test_stop_after_deadline_is_not_a_timeout (void)
{
    i2c_bus_t  bus;
    uint8_t    reg = (uint8_t)LSM303_CTRL_REG1_A;
    hw_event_t evt = { 0 };

    printf ("stop_after_deadline_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    evt.b_preempt_jump = true;
    evt.raw_set        = RAW_STOP_DET;
    push_event (evt);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, NULL, 0U));
    CHECK (g_wai_flg_calls >= 1U);
    CHECK (g_completion_interrupt_armed);
}

/* A TX FIFO that frees its slot while the task is preempted past the deadline
 * takes the write and read commands via TX_EMPTY IRQ, and the read completes. */
static void test_tx_room_after_deadline_is_not_a_timeout (void)
{
    i2c_bus_t  bus;
    uint8_t    reg    = (uint8_t)LSM303_CTRL_REG1_A;
    uint8_t    buf[1] = { 0U };
    hw_event_t ev1    = { 0 };
    hw_event_t ev2    = { 0 };

    printf ("tx_room_after_deadline_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_txflr_on_enable = TX_FIFO_FULL;

    /* TX FIFO frees up while preempted: TX_EMPTY IRQ queues write + read. */
    ev1.delay_us    = 100U;
    ev1.b_set_txflr = true;
    ev1.txflr       = 0U;
    ev1.raw_set     = RAW_TX_EMPTY;
    push_event (ev1);

    /* Hardware completes the queued read after preemption past the deadline. */
    ev2.b_preempt_jump = true;
    ev2.raw_set        = RAW_STOP_DET;
    ev2.rx_len         = 1U;
    ev2.rx_bytes[0]    = 0x3CU;
    push_event (ev2);

    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x3CU == buf[0]);
    CHECK (2U == g_isr_calls);
    CHECK (2U == g_tx_cmd_count);
    CHECK ((uint32_t)reg == g_tx_cmds[0]);
    CHECK ((CMD_READ | CMD_RESTART | CMD_STOP) == g_tx_cmds[1]);
}

/* A NACK is reported as a NACK at once via TX_ABRT + STOP_DET, not a timeout. */
static void test_nack_is_not_a_timeout (void)
{
    i2c_bus_t  bus;
    uint8_t    reg = (uint8_t)LSM303_CTRL_REG1_A;
    hw_event_t evt = { 0 };

    printf ("nack_reported_without_waiting_for_the_deadline\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    evt.delay_us = 60U;
    evt.raw_set  = RAW_TX_ABRT | RAW_STOP_DET;
    evt.abrt_set = ABRT_7B_ADDR_NOACK;
    push_event (evt);
    CHECK (I2C_BUS_NACK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                           1U, NULL, 0U));
    CHECK (g_now_us < DEADLINE_US);
    CHECK (0U == g_intr_mask);
    CHECK (0U == g_abort_writes);
}

/* Address NACK arriving before STOP_DET waits for STOP_DET and reports NACK;
 * data NACK with non-zero TX_FLUSH_CNT also reports NACK. */
static void test_nack_two_phase_and_data_nack (void)
{
    i2c_bus_t  bus;
    uint8_t    wr[2] = { (uint8_t)LSM303_CTRL_REG1_A, LSM303_CTRL_REG1_A_VAL };
    hw_event_t ev1   = { 0 };
    hw_event_t ev2   = { 0 };

    printf ("nack_two_phase_and_data_nack\n");
    model_reset ();
    i2c_rp2040_init (&bus);

    ev1.delay_us = 80U;
    ev1.raw_set  = RAW_TX_ABRT;
    ev1.abrt_set = ABRT_7B_ADDR_NOACK;
    push_event (ev1);

    ev2.delay_us = 5U;
    ev2.raw_set  = RAW_STOP_DET;
    push_event (ev2);

    CHECK (I2C_BUS_NACK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, wr,
                                           2U, NULL, 0U));
    CHECK (2U == g_isr_calls);
    CHECK (g_now_us < DEADLINE_US);

    /* Second transfer: data NACK with TX_FLUSH_CNT set in upper bits. */
    g_event_count = 0U;
    g_event_idx   = 0U;
    memset (&ev1, 0, sizeof (ev1));
    ev1.delay_us = 90U;
    ev1.raw_set  = RAW_TX_ABRT | RAW_STOP_DET;
    ev1.abrt_set = ABRT_TXDATA_NOACK | ABRT_FLUSH_CNT_BITS;
    push_event (ev1);

    CHECK (I2C_BUS_NACK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, wr,
                                           2U, NULL, 0U));
    CHECK (0U == g_intr_mask);
}

/* Controller aborts (arbitration lost, stuck STOP after TX_ABRT, FIFO error). */
static void test_abort_errors (void)
{
    i2c_bus_t  bus;
    uint8_t    reg = (uint8_t)LSM303_CTRL_REG1_A;
    hw_event_t evt = { 0 };

    printf ("abort_errors\n");
    model_reset ();
    i2c_rp2040_init (&bus);

    /* Non-NACK abort (arbitration lost) + STOP_DET -> I2C_BUS_ERROR. */
    evt.delay_us = 70U;
    evt.raw_set  = RAW_TX_ABRT | RAW_STOP_DET;
    evt.abrt_set = ABRT_ARB_LOST;
    push_event (evt);
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                            1U, NULL, 0U));
    CHECK (g_now_us < DEADLINE_US);
    CHECK (0U == g_intr_mask);

    /* TX_ABRT without STOP_DET ever arriving -> timeout + controller abort. */
    model_reset ();
    i2c_rp2040_init (&bus);
    memset (&evt, 0, sizeof (evt));
    evt.delay_us = 80U;
    evt.raw_set  = RAW_TX_ABRT;
    evt.abrt_set = ABRT_7B_ADDR_NOACK;
    push_event (evt);
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                              1U, NULL, 0U));
    CHECK (g_abort_writes >= 1U);
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);

    /* FIFO overflow error -> I2C_BUS_ERROR + controller abort. */
    model_reset ();
    i2c_rp2040_init (&bus);
    memset (&evt, 0, sizeof (evt));
    evt.delay_us = 40U;
    evt.raw_set  = RAW_RX_OVER;
    push_event (evt);
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                            1U, NULL, 0U));
    CHECK (g_abort_writes >= 1U);
    CHECK (0U == g_intr_mask);
}

/* A transfer that never completes times out, masks/disables IRQ, and aborts. */
static void test_stuck_bus_times_out (void)
{
    i2c_bus_t bus;
    uint8_t   reg = (uint8_t)LSM303_CTRL_REG1_A;
    uint32_t  attempt = 0U;

    printf ("stuck_transfer_still_times_out\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    for (attempt = 0U; attempt < 2U; attempt++)
    {
        uint32_t const t0         = g_now_us;
        uint32_t       elapsed_us = 0U;

        CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                                  &reg, 1U, NULL, 0U));
        elapsed_us = g_now_us - t0;
        CHECK (elapsed_us >= DEADLINE_US);
        CHECK (elapsed_us <= (DEADLINE_US + ATTEMPT_ABORT_ALLOWANCE_US));
        CHECK (elapsed_us < ATTEMPT_LIMIT_US);
    }
    CHECK (2U == g_alarm_fires);
    CHECK (g_wai_flg_calls >= 2U);
    CHECK (g_abort_writes >= 2U);
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);
}

/* A deadline that passes before the wait starts is a timeout, not a hang. */
static void test_deadline_passed_before_wait (void)
{
    i2c_bus_t bus;
    uint8_t   reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("deadline_passed_before_wait_is_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_preempt_read = g_timer_reads + 2U;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));
    CHECK (0U == g_wai_flg_calls);
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);
}

/* An alarm that fires after its transfer completed neither signals the flag
 * nor times out the next transfer. */
static void test_late_alarm_after_completed_transfer (void)
{
    i2c_bus_t  bus;
    uint8_t    reg    = (uint8_t)LSM303_OUT_X_L_A;
    uint8_t    buf[1] = { 0U };
    hw_event_t evt    = { 0 };
    uint32_t   set_before = 0U;

    printf ("late_alarm_after_completed_transfer\n");
    model_reset ();
    i2c_rp2040_init (&bus);

    evt.delay_us    = 50U;
    evt.raw_set     = RAW_STOP_DET;
    evt.rx_len      = 1U;
    evt.rx_bytes[0] = 0x6BU;
    push_event (evt);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x6BU == buf[0]);

    g_now_us  += 2U * DEADLINE_US;
    set_before = g_set_flg_calls;
    CHECK (NULL != g_int_handlers[I2C_TIMEOUT_ALARM_IRQ]);
    if (NULL == g_int_handlers[I2C_TIMEOUT_ALARM_IRQ])
    {
        return;
    }
    ((void (*) (UINT))g_int_handlers[I2C_TIMEOUT_ALARM_IRQ])
        (I2C_TIMEOUT_ALARM_IRQ);
    CHECK (set_before == g_set_flg_calls);
    CHECK (0U == (g_flg_bits & FLG_TIMEOUT_BIT));

    g_event_count   = 0U;
    g_event_idx     = 0U;
    evt.rx_bytes[0] = 0x2DU;
    push_event (evt);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x2DU == buf[0]);
}

/* An alarm that fires late after a timed-out and retired transfer neither
 * signals the flag nor times out the next transfer. */
static void test_late_alarm_after_timed_out_transfer (void)
{
    i2c_bus_t  bus;
    uint8_t    reg    = (uint8_t)LSM303_OUT_X_L_A;
    uint8_t    buf[1] = { 0U };
    hw_event_t evt    = { 0 };
    uint32_t   set_before = 0U;

    printf ("late_alarm_after_timed_out_transfer\n");
    model_reset ();
    i2c_rp2040_init (&bus);

    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));
    CHECK (0U == g_intr_mask);

    g_now_us  += 2U * DEADLINE_US;
    set_before = g_set_flg_calls;
    CHECK (NULL != g_int_handlers[I2C_TIMEOUT_ALARM_IRQ]);
    if (NULL == g_int_handlers[I2C_TIMEOUT_ALARM_IRQ])
    {
        return;
    }
    ((void (*) (UINT))g_int_handlers[I2C_TIMEOUT_ALARM_IRQ])
        (I2C_TIMEOUT_ALARM_IRQ);
    CHECK (set_before == g_set_flg_calls);
    CHECK (0U == (g_flg_bits & FLG_TIMEOUT_BIT));

    evt.delay_us    = 50U;
    evt.raw_set     = RAW_STOP_DET;
    evt.rx_len      = 1U;
    evt.rx_bytes[0] = 0x4EU;
    push_event (evt);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x4EU == buf[0]);
}

/* A STOP_DET raised when a stale transfer's controller is re-enabled must not
 * complete the next write before its own STOP arrives. */
static void test_stale_stop_after_reenable (void)
{
    i2c_bus_t  bus;
    uint8_t    reg = (uint8_t)LSM303_CTRL_REG1_A;
    hw_event_t evt = { 0 };
    uint32_t   t0  = 0U;

    printf ("stale_stop_after_reenable_does_not_complete_next_write\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));

    g_stale_stop_on_enable = true;
    g_tx_cmd_count         = 0U;
    evt.delay_us           = 300U;
    evt.raw_set            = RAW_STOP_DET;
    push_event (evt);
    t0 = g_now_us;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, NULL, 0U));
    CHECK ((g_now_us - t0) >= 300U);
    CHECK (1U == g_tx_cmd_count);
}

/* A controller that stays enabled after an abort is reset before the next
 * transfer, and that transfer completes. */
static void test_wedged_controller_is_reset_before_next_transfer (void)
{
    i2c_bus_t  bus;
    uint8_t    reg    = (uint8_t)LSM303_CTRL_REG1_A;
    uint8_t    buf[1] = { 0xEEU };
    hw_event_t evt    = { 0 };
    uint32_t   t0     = 0U;
    uint32_t   pre_arm_us = 0U;

    printf ("wedged_controller_is_reset_before_next_transfer\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_abort_stuck = true;
    t0            = g_now_us;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));
    CHECK ((g_now_us - t0) <= (STUCK_ABORT_LIMIT_US + TIMING_SLACK_US));

    g_tx_cmd_count  = 0U;
    evt.delay_us    = 100U;
    evt.raw_set     = RAW_STOP_DET;
    evt.rx_len      = 1U;
    evt.rx_bytes[0] = 0x77U;
    push_event (evt);
    t0 = g_now_us;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                         &reg, 1U, buf, 1U));
    pre_arm_us = (g_alarm_target - DEADLINE_US) - t0;
    CHECK (pre_arm_us <= (PRE_ARM_LIMIT_US + TIMING_SLACK_US));
    CHECK (0x77U == buf[0]);
    CHECK (2U == g_tx_cmd_count);
    CHECK ((CMD_READ | CMD_RESTART | CMD_STOP) == g_tx_cmds[1]);
    CHECK (0U == g_intr_mask);
    CHECK (!g_controller_wedged);
    CHECK (CFG_CON == g_regs[(REG_CON - I2C1_BASE) / 4U]);
    CHECK (CFG_FS_SCL_HCNT == g_regs[(REG_FS_SCL_HCNT - I2C1_BASE) / 4U]);
    CHECK (CFG_FS_SCL_LCNT == g_regs[(REG_FS_SCL_LCNT - I2C1_BASE) / 4U]);
    CHECK (CFG_FS_SPKLEN == g_regs[(REG_FS_SPKLEN - I2C1_BASE) / 4U]);
    CHECK (CFG_SDA_HOLD == g_regs[(REG_SDA_HOLD - I2C1_BASE) / 4U]);
    CHECK (0U == g_regs[(REG_TX_TL - I2C1_BASE) / 4U]);
    CHECK (0U == g_regs[(REG_RX_TL - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_ENABLE - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_CON - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_FS_SCL_HCNT - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_FS_SCL_LCNT - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_FS_SPKLEN - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_SDA_HOLD - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_TX_TL - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_RX_TL - I2C1_BASE) / 4U]);
    CHECK (g_cfg_at_enable[(REG_INTR_MASK - I2C1_BASE) / 4U]);
}

/* A wedged controller whose block reset also fails returns an error within
 * the reset bound, without starting a transfer. */
static void test_wedged_controller_reset_failure_is_an_error (void)
{
    i2c_bus_t bus;
    uint8_t   reg        = (uint8_t)LSM303_CTRL_REG1_A;
    uint8_t   buf[1]     = { 0xEEU };
    uint32_t  t0         = 0U;
    uint32_t  elapsed_us = 0U;

    printf ("wedged_controller_reset_failure_is_an_error\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_abort_stuck = true;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));

    g_reset_stuck  = true;
    g_tx_cmd_count = 0U;
    t0             = g_now_us;
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                            &reg, 1U, buf, 1U));
    elapsed_us = g_now_us - t0;
    CHECK (elapsed_us < ATTEMPT_LIMIT_US);
    CHECK (elapsed_us <= (PRE_ARM_LIMIT_US + TIMING_SLACK_US));
    CHECK (0U == g_tx_cmd_count);
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);
    CHECK (0xEEU == buf[0]);
}

static void test_init_reset_failure_leaves_bus_down (void)
{
    i2c_bus_t bus;
    uint8_t   reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("init_reset_failure_leaves_bus_down\n");
    model_reset ();
    g_reset_stuck = true;
    i2c_rp2040_init (&bus);
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                            &reg, 1U, NULL, 0U));
    CHECK (0U == g_wai_calls);
    CHECK (0U == g_tx_cmd_count);
    CHECK (NULL == g_int_handlers[I2C_RP2040_IRQ]);
}

/* An alarm whose target is already past when armed neither waits nor fires. */
static void test_alarm_armed_past_deadline_is_a_timeout (void)
{
    i2c_bus_t bus;
    uint8_t   reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("alarm_armed_past_deadline_is_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_advance_clock_on_alarm_arm = true;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));
    CHECK (0U == g_wai_flg_calls);
    CHECK (0U == g_alarm_fires);
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);
}

/* Partial reads: incremental multi-IRQ burst, truncated read on early STOP,
 * NACK mid-read, and >16-byte transfer exceeding FIFO depth. */
static void test_partial_reads (void)
{
    i2c_bus_t  bus;
    uint8_t    reg     = (uint8_t)(LSM303_OUT_X_L_A | LSM303_AUTO_INC_A);
    uint8_t    buf6[6] = { 0U };
    uint8_t    buf20[20];
    uint8_t    idx     = 0U;
    hw_event_t ev1     = { 0 };
    hw_event_t ev2     = { 0 };
    hw_event_t ev3     = { 0 };

    printf ("partial_reads_incremental_truncated_and_long\n");

    /* 1. 6-byte burst delivered as 2 + 2 + 2 across three interrupts. */
    model_reset ();
    i2c_rp2040_init (&bus);
    ev1.delay_us    = 80U;
    ev1.rx_len      = 2U;
    ev1.rx_bytes[0] = 0x11U;
    ev1.rx_bytes[1] = 0x22U;
    push_event (ev1);

    ev2.delay_us    = 60U;
    ev2.rx_len      = 2U;
    ev2.rx_bytes[0] = 0x33U;
    ev2.rx_bytes[1] = 0x44U;
    push_event (ev2);

    ev3.delay_us    = 60U;
    ev3.raw_set     = RAW_STOP_DET;
    ev3.rx_len      = 2U;
    ev3.rx_bytes[0] = 0x55U;
    ev3.rx_bytes[1] = 0x66U;
    push_event (ev3);

    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf6, 6U));
    CHECK (3U == g_isr_calls);
    CHECK (0x11U == buf6[0]);
    CHECK (0x22U == buf6[1]);
    CHECK (0x33U == buf6[2]);
    CHECK (0x44U == buf6[3]);
    CHECK (0x55U == buf6[4]);
    CHECK (0x66U == buf6[5]);
    CHECK (7U == g_tx_cmd_count);
    CHECK ((uint32_t)reg == g_tx_cmds[0]);
    CHECK ((CMD_READ | CMD_RESTART) == g_tx_cmds[1]);
    CHECK (CMD_READ == g_tx_cmds[5]);
    CHECK ((CMD_READ | CMD_STOP) == g_tx_cmds[6]);

    /* 2. Truncated partial read: 2 of 6 bytes then STOP_DET -> I2C_BUS_ERROR. */
    model_reset ();
    i2c_rp2040_init (&bus);
    memset (&ev1, 0, sizeof (ev1));
    memset (&ev2, 0, sizeof (ev2));
    ev1.delay_us    = 80U;
    ev1.rx_len      = 2U;
    ev1.rx_bytes[0] = 0xA1U;
    ev1.rx_bytes[1] = 0xB2U;
    push_event (ev1);

    ev2.delay_us = 40U;
    ev2.raw_set  = RAW_STOP_DET;
    push_event (ev2);

    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                            1U, buf6, 6U));
    CHECK (0U == g_intr_mask);

    /* 3. Partial read aborted mid-stream by NACK -> I2C_BUS_NACK. */
    model_reset ();
    i2c_rp2040_init (&bus);
    memset (&ev1, 0, sizeof (ev1));
    memset (&ev2, 0, sizeof (ev2));
    ev1.delay_us    = 80U;
    ev1.rx_len      = 2U;
    ev1.rx_bytes[0] = 0x01U;
    ev1.rx_bytes[1] = 0x02U;
    push_event (ev1);

    ev2.delay_us = 40U;
    ev2.raw_set  = RAW_TX_ABRT | RAW_STOP_DET;
    ev2.abrt_set = ABRT_7B_ADDR_NOACK;
    push_event (ev2);

    CHECK (I2C_BUS_NACK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                           1U, buf6, 6U));

    /* 4. Long read (>16-byte FIFO depth): 1 write + 20 read bytes in batches. */
    model_reset ();
    i2c_rp2040_init (&bus);
    memset (buf20, 0, sizeof (buf20));
    memset (&ev1, 0, sizeof (ev1));
    memset (&ev2, 0, sizeof (ev2));
    ev1.delay_us    = 200U;
    ev1.b_set_txflr = true;
    ev1.txflr       = 4U;
    ev1.raw_set     = RAW_TX_EMPTY;
    ev1.rx_len      = 12U;
    for (idx = 0U; idx < 12U; idx++)
    {
        ev1.rx_bytes[idx] = (uint8_t)(0x40U + idx);
    }
    push_event (ev1);

    ev2.delay_us = 200U;
    ev2.raw_set  = RAW_STOP_DET;
    ev2.rx_len   = 8U;
    for (idx = 0U; idx < 8U; idx++)
    {
        ev2.rx_bytes[idx] = (uint8_t)(0x4CU + idx);
    }
    push_event (ev2);

    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf20, 20U));
    CHECK (21U == g_tx_cmd_count);
    CHECK ((CMD_READ | CMD_STOP) == g_tx_cmds[20]);
    for (idx = 0U; idx < 20U; idx++)
    {
        CHECK ((uint8_t)(0x40U + idx) == buf20[idx]);
    }
}

/* Deadline boundary edges: exact deadline completion, 32-bit timer wrap, and
 * spurious early flag wake without hardware completion. */
static void test_deadline_edges (void)
{
    i2c_bus_t  bus;
    uint8_t    reg    = (uint8_t)LSM303_CTRL_REG1_A;
    uint8_t    buf[2] = { 0U, 0U };
    hw_event_t ev1    = { 0 };
    hw_event_t ev2    = { 0 };

    printf ("deadline_edges_exact_wrap_and_spurious_wake\n");

    /* 1. Completion arriving at exact deadline (elapsed == DEADLINE_US). */
    model_reset ();
    i2c_rp2040_init (&bus);
    ev1.delay_us = DEADLINE_US - 2U; /* Accounts for 2 timer reads before wait. */
    ev1.raw_set  = RAW_STOP_DET;
    push_event (ev1);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, NULL, 0U));

    /* 2. 32-bit TIMERAWL wraparound during an active transfer. */
    model_reset ();
    i2c_rp2040_init (&bus);
    g_now_us     = 0xFFFFF800U;
    memset (&ev1, 0, sizeof (ev1));
    ev1.delay_us    = 3000U; /* Crosses 0xFFFFFFFF -> 0x000003B8. */
    ev1.raw_set     = RAW_STOP_DET;
    ev1.rx_len      = 2U;
    ev1.rx_bytes[0] = 0x71U;
    ev1.rx_bytes[1] = 0x82U;
    push_event (ev1);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 2U));
    CHECK (0x71U == buf[0]);
    CHECK (0x82U == buf[1]);

    /* 3. Timeout across 32-bit TIMERAWL wraparound. */
    model_reset ();
    i2c_rp2040_init (&bus);
    g_now_us = 0xFFFFF000U;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                              1U, NULL, 0U));

    /* 4. Spurious early flag wake with no hardware completion keeps waiting. */
    model_reset ();
    i2c_rp2040_init (&bus);
    memset (&ev1, 0, sizeof (ev1));
    memset (&ev2, 0, sizeof (ev2));
    ev1.delay_us       = 50U;
    ev1.b_spurious_flg = true;
    push_event (ev1);

    ev2.delay_us    = 250U;
    ev2.raw_set     = RAW_STOP_DET;
    ev2.rx_len      = 2U;
    ev2.rx_bytes[0] = 0x9AU;
    ev2.rx_bytes[1] = 0xBCU;
    push_event (ev2);

    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 2U));
    CHECK (g_wai_flg_calls >= 2U);
    CHECK (0x9AU == buf[0]);
    CHECK (0xBCU == buf[1]);
}

/* A late interrupt arriving after an aborted/timed-out transfer must neither
 * touch the retired buffer, post a wakeup, nor satisfy the next transfer. */
static void test_late_interrupt_after_aborted_transfer (void)
{
    i2c_bus_t  bus;
    uint8_t    reg     = (uint8_t)LSM303_OUT_X_L_A;
    uint8_t    buf1[4] = { 0xEEU, 0xEEU, 0xEEU, 0xEEU };
    uint8_t    buf2[2] = { 0xDDU, 0xDDU };
    uint8_t    buf3[2] = { 0x00U, 0x00U };
    uint8_t    stale[4] = { 0xAAU, 0xBBU, 0xCCU, 0xDDU };
    hw_event_t evt     = { 0 };
    uint32_t   set_before = 0U;

    printf ("late_interrupt_after_aborted_transfer\n");
    model_reset ();
    i2c_rp2040_init (&bus);

    /* Transfer 1 times out and retires atomically. */
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                              1U, buf1, 4U));
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);
    CHECK (NULL != g_int_handlers[I2C_RP2040_IRQ]);
    CHECK (NULL != g_int_handlers[I2C_TIMEOUT_ALARM_IRQ]);

    /* Force-inject a late ISR from the expired transfer with RX data + STOP. */
    set_before = g_set_flg_calls;
    g_raw      = RAW_STOP_DET;
    rx_fifo_push (stale, 4U);
    CHECK (NULL != g_int_handlers[I2C_RP2040_IRQ]);
    if (NULL == g_int_handlers[I2C_RP2040_IRQ])
    {
        return;
    }
    ((void (*) (UINT))g_int_handlers[I2C_RP2040_IRQ]) (I2C_RP2040_IRQ);

    /* Expired buffer untouched, no wake signalled, STOP cleared, mask stays 0. */
    CHECK (0xEEU == buf1[0]);
    CHECK (0xEEU == buf1[1]);
    CHECK (0xEEU == buf1[2]);
    CHECK (0xEEU == buf1[3]);
    CHECK (set_before == g_set_flg_calls);
    CHECK (0U == g_flg_bits);
    CHECK (0U == g_intr_mask);
    CHECK (0UL == (g_raw & RAW_STOP_DET));

    /* Even with a stray flag bit set before Transfer 2 arms, Transfer 2 on a
     * stuck bus must NOT complete from the prior transfer. */
    g_flg_bits = 1U;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                              1U, buf2, 2U));
    CHECK (0xDDU == buf2[0]);
    CHECK (0xDDU == buf2[1]);

    /* Transfer 3 completes cleanly with its own interrupt and fresh data. */
    g_event_count = 0U;
    g_event_idx   = 0U;
    evt.delay_us    = 150U;
    evt.raw_set     = RAW_STOP_DET;
    evt.rx_len      = 2U;
    evt.rx_bytes[0] = 0x42U;
    evt.rx_bytes[1] = 0x99U;
    push_event (evt);

    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf3, 2U));
    CHECK (0x42U == buf3[0]);
    CHECK (0x99U == buf3[1]);
}

/* Invalid arguments fail safely; recover retires/masks cleanly. */
static void test_invalid_args_and_recover (void)
{
    i2c_bus_t  bus;
    uint8_t    reg    = (uint8_t)LSM303_CTRL_REG1_A;
    uint8_t    buf[1] = { 0U };
    hw_event_t evt    = { 0 };

    printf ("invalid_args_and_recover\n");
    model_reset ();
    i2c_rp2040_init (&bus);

    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, NULL,
                                            1U, NULL, 0U));
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, NULL,
                                            0U, NULL, 1U));
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, NULL,
                                            0U, NULL, 0U));
    CHECK (!g_completion_interrupt_armed);

    bus.recover (bus.p_ctx);
    CHECK (0U == g_intr_mask);
    CHECK (0 == g_shim_int_enabled);

    /* Wait error from kernel retires and aborts cleanly. */
    g_force_wai_err = true;
    CHECK (I2C_BUS_ERROR == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                            1U, NULL, 0U));
    CHECK (g_abort_writes >= 1U);
    CHECK (0U == g_intr_mask);
    g_force_wai_err = false;

    /* Read-only transfer (wr_len == 0, rd_len == 1) works after recovery. */
    g_tx_cmd_count  = 0U;
    evt.delay_us    = 100U;
    evt.raw_set     = RAW_STOP_DET;
    evt.rx_len      = 1U;
    evt.rx_bytes[0] = 0x55U;
    push_event (evt);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, NULL,
                                         0U, buf, 1U));
    CHECK (0x55U == buf[0]);
    CHECK (1U == g_tx_cmd_count);
    CHECK ((CMD_READ | CMD_STOP) == g_tx_cmds[0]);
}

static void test_completion_interrupt (void)
{
    i2c_bus_t  bus;
    uint8_t    reg = (uint8_t)LSM303_CTRL_REG1_A;
    hw_event_t evt = { 0 };

    model_reset ();
    i2c_rp2040_init (&bus);
    evt.delay_us = 120U;
    evt.raw_set  = RAW_TX_EMPTY | RAW_STOP_DET;
    push_event (evt);
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg, 1U,
                                         NULL, 0U));
    CHECK (g_completion_interrupt_armed);
    CHECK (g_wai_flg_calls >= 1U);
    CHECK (1U == g_isr_calls);
    CHECK (0U == g_intr_mask);
}

int main (int argc, char ** argv)
{
    if ((argc == 2) && (strcmp (argv[1], "completion-interrupt-armed") == 0))
    {
        test_completion_interrupt ();
        return g_failures != 0;
    }
    if (argc != 1)
    {
        return 1;
    }
    test_write_done_during_preemption ();
    test_read_done_during_preemption ();
    test_stop_after_deadline_is_not_a_timeout ();
    test_tx_room_after_deadline_is_not_a_timeout ();
    test_nack_is_not_a_timeout ();
    test_nack_two_phase_and_data_nack ();
    test_abort_errors ();
    test_stuck_bus_times_out ();
    test_deadline_passed_before_wait ();
    test_partial_reads ();
    test_deadline_edges ();
    test_late_interrupt_after_aborted_transfer ();
    test_late_alarm_after_completed_transfer ();
    test_late_alarm_after_timed_out_transfer ();
    test_alarm_armed_past_deadline_is_a_timeout ();
    test_stale_stop_after_reenable ();
    test_wedged_controller_is_reset_before_next_transfer ();
    test_wedged_controller_reset_failure_is_an_error ();
    test_init_reset_failure_leaves_bus_down ();
    test_invalid_args_and_recover ();
    test_completion_interrupt ();

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
