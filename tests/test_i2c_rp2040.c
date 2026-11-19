/** @file test_i2c_rp2040.c
 *
 * @brief Host tests for the polled RP2040 I2C1 driver against a scripted
 *        register model.
 *
 * The model answers the status reads that write_read polls. Its timer moves
 * one microsecond per read, as a poll loop sees it. A preemption is modelled
 * as one timer read that jumps past the transfer deadline; the transfer
 * completes on that same read, as it would while this task was not running.
 * Every register read counts against a limit, so a poll loop that never sees
 * its deadline ends the run with a FAIL instead of hanging.
 */

#include <tk/tkernel.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_rp2040.h"
#include "lsm303.h"

#define I2C1_BASE         (0x40048000UL)
#define I2C_REG_COUNT     (64U)
#define REG_RAW_INTR      (I2C1_BASE + 0x34UL)
#define REG_DATA_CMD      (I2C1_BASE + 0x10UL)
#define REG_TX_ABRT_SRC   (I2C1_BASE + 0x80UL)
#define REG_TXFLR         (I2C1_BASE + 0x74UL)
#define REG_RXFLR         (I2C1_BASE + 0x78UL)
#define REG_RESETS_DONE   (0x4000C008UL)
#define REG_SIO_GPIO_IN   (0xD0000004UL)
#define RAW_TX_EMPTY      (0x010UL)
#define RAW_STOP_DET      (0x200UL)
#define ABRT_7B_ADDR_NOACK (0x001UL)
#define DEADLINE_US       (5000U)
#define PREEMPT_US        (DEADLINE_US + 1U)
#define TX_FIFO_FULL      (16U)
#define READ_LIMIT        (100000U)

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

static uint32_t g_now_us;
static uint32_t g_timer_reads;
static uint32_t g_event_read;
static uint32_t g_event_raw;
static uint32_t g_event_rxflr;
static uint32_t g_event_txflr;
static uint32_t g_raw;
static uint32_t g_rxflr;
static uint32_t g_txflr;
static uint32_t g_rx_byte;
static uint32_t g_abrt_source;
static uint32_t g_reads;
static uint32_t g_regs[I2C_REG_COUNT];

static void model_reset (void)
{
    g_now_us = 0U;
    g_timer_reads = 0U;
    g_event_read = 0U;
    g_event_raw = 0U;
    g_event_rxflr = 0U;
    g_event_txflr = 0U;
    g_raw = 0U;
    g_rxflr = 0U;
    g_txflr = 0U;
    g_rx_byte = 0U;
    g_abrt_source = 0U;
    g_reads = 0U;
    memset (g_regs, 0, sizeof (g_regs));
}

static uint32_t timer_read (void)
{
    uint32_t value = 0U;

    g_timer_reads++;
    if (g_timer_reads == g_event_read)
    {
        g_now_us += PREEMPT_US;
        g_raw |= g_event_raw;
        g_rxflr = g_event_rxflr;
        g_txflr = g_event_txflr;
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
        printf ("  FAIL %s:%d: poll loop never saw its deadline\n", __FILE__,
                __LINE__);
        exit (1);
    }
    if (TIMER_TIMERAWL == addr)
    {
        value = timer_read ();
    }
    else if (REG_RAW_INTR == addr)
    {
        value = g_raw;
    }
    else if (REG_RXFLR == addr)
    {
        value = g_rxflr;
    }
    else if (REG_TXFLR == addr)
    {
        value = g_txflr;
    }
    else if (REG_DATA_CMD == addr)
    {
        value = g_rx_byte;
    }
    else if (REG_TX_ABRT_SRC == addr)
    {
        value = g_abrt_source;
    }
    else if (REG_RESETS_DONE == addr)
    {
        value = 0xFFFFFFFFU;
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
        /* Other registers are not modelled. */
    }
    return value;
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    if (in_i2c1 (addr))
    {
        g_regs[(addr - I2C1_BASE) / 4U] = value;
    }
}

/* A write of one register byte that completes while the task is preempted
 * past the deadline is a success, not a timeout. */
static void test_write_done_during_preemption (void)
{
    i2c_bus_t bus;
    uint8_t reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("write_done_during_preemption_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_raw = 0U;
    g_event_read = 2U;
    g_event_raw = RAW_TX_EMPTY | RAW_STOP_DET;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, NULL, 0U));
}

/* A read whose data arrives while the task is preempted past the deadline
 * returns that data. */
static void test_read_done_during_preemption (void)
{
    i2c_bus_t bus;
    uint8_t reg = (uint8_t)(LSM303_OUT_X_L_A | LSM303_AUTO_INC_A);
    uint8_t buf[1] = { 0U };

    printf ("read_done_during_preemption_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_raw = RAW_TX_EMPTY;
    g_rx_byte = 0x5AU;
    g_event_read = 2U;
    g_event_raw = RAW_STOP_DET;
    g_event_rxflr = 1U;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x5AU == buf[0]);
}

/* A STOP that lands while the task is preempted past the deadline ends a write
 * with no read, and the write is a success. */
static void test_stop_after_deadline_is_not_a_timeout (void)
{
    i2c_bus_t bus;
    uint8_t reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("stop_after_deadline_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_raw = RAW_TX_EMPTY;
    g_event_read = 2U;
    g_event_raw = RAW_STOP_DET;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, NULL, 0U));
}

/* A TX FIFO that frees its slot while the task is preempted past the deadline
 * takes the read command, and the read completes. */
static void test_tx_room_after_deadline_is_not_a_timeout (void)
{
    i2c_bus_t bus;
    uint8_t reg = (uint8_t)LSM303_CTRL_REG1_A;
    uint8_t buf[1] = { 0U };

    printf ("tx_room_after_deadline_is_not_a_timeout\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_raw = RAW_TX_EMPTY | RAW_STOP_DET;
    g_txflr = TX_FIFO_FULL;
    g_rxflr = 1U;
    g_rx_byte = 0x3CU;
    g_event_read = 2U;
    g_event_txflr = 0U;
    g_event_rxflr = 1U;
    CHECK (I2C_BUS_OK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                         1U, buf, 1U));
    CHECK (0x3CU == buf[0]);
}

/* A NACK is reported as a NACK, at once, not as a timeout. */
static void test_nack_is_not_a_timeout (void)
{
    i2c_bus_t bus;
    uint8_t reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("nack_reported_without_waiting_for_the_deadline\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_raw = RAW_TX_EMPTY | RAW_STOP_DET;
    g_abrt_source = ABRT_7B_ADDR_NOACK;
    CHECK (I2C_BUS_NACK == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR, &reg,
                                           1U, NULL, 0U));
    CHECK (g_now_us < DEADLINE_US);
}

/* A transfer that never completes still times out, and the abort returns. */
static void test_stuck_bus_times_out (void)
{
    i2c_bus_t bus;
    uint8_t reg = (uint8_t)LSM303_CTRL_REG1_A;

    printf ("stuck_transfer_still_times_out\n");
    model_reset ();
    i2c_rp2040_init (&bus);
    g_raw = 0U;
    CHECK (I2C_BUS_TIMEOUT == bus.write_read (bus.p_ctx, LSM303_ACC_ADDR,
                                              &reg, 1U, NULL, 0U));
    CHECK (g_now_us >= DEADLINE_US);
    CHECK (g_now_us < (2U * DEADLINE_US));
}

int main (void)
{
    test_write_done_during_preemption ();
    test_read_done_during_preemption ();
    test_stop_after_deadline_is_not_a_timeout ();
    test_tx_room_after_deadline_is_not_a_timeout ();
    test_nack_is_not_a_timeout ();
    test_stuck_bus_times_out ();

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
