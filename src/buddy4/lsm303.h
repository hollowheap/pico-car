/** @file lsm303.h
 *
 * @brief LSM303DLHC (GY-511) accelerometer and magnetometer protocol.
 *
 * Talks to the chip through an abstract I2C bus (b4_i2c_bus_t), so the same
 * code runs against the RP2040 driver (b4_i2c_rp2040.c) on the car and a
 * simulated chip in the host tests.
 *
 * Settings, from SE01_PostTri1 and the datasheet (DocID018771 Rev 2):
 *   Accelerometer 0x19: CTRL_REG1_A (0x20) = 0x47  50 Hz, X/Y/Z on
 *                       CTRL_REG4_A (0x23) = 0x88  BDU, +/-2 g, high-res
 *                       1 mg per LSB, 12-bit left-justified, little-endian.
 *   Magnetometer 0x1E:  CRA_REG_M (0x00) = 0x14    30 Hz (read at 16.7 Hz,
 *                                                 so every read is new)
 *                       CRB_REG_M (0x01) = 0x20    +/-1.3 gauss
 *                       MR_REG_M  (0x02) = 0x00    continuous
 *                       1100 LSB/gauss X and Y, 980 Z (table 75).
 *                       Output order X, Z, Y, high byte first.
 *   Identity: IRA/IRB/IRC_REG_M (0x0A-0x0C) read 'H', '4', '3'.
 *   The accelerometer has no identity register; init writes its control
 *   registers and reads them back instead.
 *
 * Every transaction is retried up to max_attempts times. A bus that times
 * out is given a recovery (9 clocks + STOP) before the retry. A transaction
 * that fails every attempt latches the fault flag; reading continues.
 *
 * Lost sensor: an accelerometer read of exactly (0, 0, 0) is rejected with
 * LSM303_NO_DATA, because gravity makes it impossible on a working part; it
 * is what a re-powered LSM303 returns before its registers are reloaded
 * (car day S9). After LSM303_LOST_AFTER bad reads in a row (bus failure or
 * no data, accelerometer or magnetometer) ready is cleared, so the caller
 * runs lsm303_init again.
 */

#ifndef LSM303_H
#define LSM303_H

#include <stdbool.h>
#include <stdint.h>

#define LSM303_ACC_ADDR      (0x19U)
#define LSM303_MAG_ADDR      (0x1EU)

#define LSM303_CTRL_REG1_A   (0x20U)
#define LSM303_CTRL_REG4_A   (0x23U)
#define LSM303_OUT_X_L_A     (0x28U)
#define LSM303_AUTO_INC_A    (0x80U)   /* Sub-address MSB: auto-increment. */
#define LSM303_CRA_REG_M     (0x00U)
#define LSM303_CRB_REG_M     (0x01U)
#define LSM303_MR_REG_M      (0x02U)
#define LSM303_OUT_X_H_M     (0x03U)
#define LSM303_IRA_REG_M     (0x0AU)

#define LSM303_CTRL_REG1_A_VAL (0x47U)
#define LSM303_CTRL_REG4_A_VAL (0x88U)
#define LSM303_CRA_REG_M_VAL   (0x14U)
#define LSM303_CRB_REG_M_VAL   (0x20U)
#define LSM303_MR_REG_M_VAL    (0x00U)

/* Bad reads in a row before the sensor counts as lost (100 ms at 50 Hz). */
#define LSM303_LOST_AFTER      (5U)

typedef enum
{
    B4_BUS_OK = 0,
    B4_BUS_NACK,                 /* Address or data not acknowledged.      */
    B4_BUS_TIMEOUT,              /* Transfer did not finish in time.       */
    B4_BUS_ERROR                 /* Any other controller error.            */
} b4_bus_result_t;

/* Write wr_len bytes, then (if rd_len > 0) repeated-start and read rd_len
 * bytes, then STOP. One call = one complete transaction. */
typedef struct
{
    void * p_ctx;
    b4_bus_result_t (*write_read) (void * p_ctx, uint8_t addr7,
                                   const uint8_t * p_wr, uint32_t wr_len,
                                   uint8_t * p_rd, uint32_t rd_len);
    void (*recover) (void * p_ctx);  /* Optional; may be NULL. */
} b4_i2c_bus_t;

typedef enum
{
    LSM303_OK = 0,
    LSM303_BAD_ARG,
    LSM303_BUS_FAULT,            /* Every attempt failed; fault latched.   */
    LSM303_BAD_ID,               /* Magnetometer identity is not 'H43'.    */
    LSM303_CONFIG_MISMATCH,      /* Read-back differs from what was set.   */
    LSM303_NO_DATA               /* Accelerometer read all zero.           */
} lsm303_result_t;

typedef struct
{
    uint32_t transactions;
    uint32_t retries;            /* Extra attempts after a failure.        */
    uint32_t nacks;
    uint32_t timeouts;
    uint32_t errors;
    uint32_t failures;           /* Transactions that used every attempt.  */
} lsm303_stats_t;

typedef struct
{
    b4_i2c_bus_t   bus;
    uint8_t        max_attempts;
    bool           ready;        /* init succeeded and sensor not lost.    */
    bool           fault;        /* Latched; see lsm303_clear_fault.       */
    uint8_t        bad_reads;    /* Bad reads in a row.                    */
    lsm303_stats_t stats;
} lsm303_t;

/* Raw counts and converted values for one read. */
typedef struct
{
    int16_t raw[3];              /* X, Y, Z in device counts.              */
    float   value[3];            /* X, Y, Z in g or gauss.                 */
} lsm303_vec_t;

/* Check the identity, write the configuration and read it back.
 * max_attempts must be 1 to 8. */
lsm303_result_t lsm303_init (lsm303_t * p_dev, const b4_i2c_bus_t * p_bus,
                             uint8_t max_attempts);

/* One 6-byte burst read. Requires a successful init. A failed read or an
 * all-zero accelerometer read counts toward LSM303_LOST_AFTER. */
lsm303_result_t lsm303_read_accel (lsm303_t * p_dev, lsm303_vec_t * p_out);
lsm303_result_t lsm303_read_mag (lsm303_t * p_dev, lsm303_vec_t * p_out);

void lsm303_clear_fault (lsm303_t * p_dev);

/* Conversions, exposed for tests. */
int16_t lsm303_accel_counts (uint8_t lo, uint8_t hi);   /* 12-bit value.  */
int16_t lsm303_mag_counts (uint8_t hi, uint8_t lo);     /* 16-bit value.  */

#endif /* LSM303_H */

/*** end of file ***/
