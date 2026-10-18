/** @file imu_lsm303.h
 *
 * @brief B4 IMU driver for GY-511 (LSM303DLHC) accelerometer + magnetometer.
 *
 * Task 1 result: the kit IMU is NOT a 6-axis gyro part. It is a 3-axis
 * accelerometer + 3-axis magnetometer, no gyroscope. Turn-rate must be
 * derived from magnetometer yaw and/or wheel encoders, not from gyro.
 *
 * Wiring: GY-511 SDA -> GP2 (I2C1 SDA), SCL -> GP3 (I2C1 SCL). This is
 * Robo Pico Grove 2 / Maker Port (shared). 3-5 V module supply is
 * 3V3-tolerant on I2C. External 10 k pull-ups as per LSM303DLHC Fig.4;
 * Grove/Maker port already provides pull-ups - verify on scope if NACK.
 *
 * Default mount assumption (verify on car, correct with
 * imu_set_mount_flip if wrong): module flat, chip X forward, Y left,
 * Z up. Calibrate level bias at startup on flat ground.
 */

#ifndef IMU_LSM303_H
#define IMU_LSM303_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_I2C_SDA_PIN        2u
#define IMU_I2C_SCL_PIN        3u
#define IMU_I2C_BAUD_HZ        100000u

/* 7-bit addresses from datasheet Table 14/16. */
#define IMU_ACCEL_ADDR         0x19u
#define IMU_MAG_ADDR           0x1Eu

/* Accel full-scale selection. Only +/-2g is used for hump work. */
#define IMU_ACCEL_SENS_MG_LSB  1.0f

/* Raw readings as signed 12-bit left-aligned in 16-bit registers. */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_raw_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_mag_raw_t;

/* Calibrated physical units. */
typedef struct
{
    float x_g;
    float y_g;
    float z_g;
} imu_accel_g_t;

/* Level-ground bias estimated at startup (in g). */
typedef struct
{
    float bias_x_g;
    float bias_y_g;
    float bias_z_g;
    bool  valid;
} imu_calib_t;

/*!
 * @brief Init I2C1 on GP2/GP3 and configure accel + mag.
 *
 * Accel: 100 Hz normal mode, all axes, +/-2g, high-res, BDU set
 *   CTRL_REG1_A = 0x57, CTRL_REG4_A = 0x88.
 * Mag: 15 Hz, gain +/-1.3 gauss, continuous conversion
 *   CRA = 0x10, CRB = 0x20, MR = 0x00.
 *
 * @return true if both WHO-AM-I paths ACK and config writes succeed.
 */
bool imu_lsm303_init (void);

/*!
 * @brief Read accelerometer XYZ as raw 12-bit signed values.
 *
 * @param[out] p_out Destination for raw sample.
 * @return true on successful I2C read.
 */
bool imu_accel_read_raw (imu_accel_raw_t * p_out);

/*!
 * @brief Read magnetometer XYZ as raw 16-bit signed values.
 *
 * Note output order on wire is X_H/X_L, Z_H/Z_L, Y_H/Y_L.
 *
 * @param[out] p_out Destination for raw sample.
 * @return true on successful I2C read.
 */
bool imu_mag_read_raw (imu_mag_raw_t * p_out);

/*!
 * @brief Read accelerometer converted to g with bias removed.
 *
 * @param[in]  p_calib Calibration from imu_calibrate_level.
 * @param[out] p_out   Calibrated sample in g.
 * @return true on successful read.
 */
bool imu_accel_read_g (const imu_calib_t * p_calib, imu_accel_g_t * p_out);

/*!
 * @brief Estimate level bias by averaging flat-ground samples.
 *
 * Call with car stationary on flat ground. Averages 50 samples at
 * ~10 ms spacing. Expected: x~0, y~0, z~1g before bias removal;
 * stored bias makes level read x~0, y~0, z~0 after removal with
 * gravity re-added by caller for tilt math (see hump.c).
 *
 * @param[out] p_calib Destination for bias.
 * @return true if calibration succeeded.
 */
bool imu_calibrate_level (imu_calib_t * p_calib);

/*!
 * @brief Pitch from accelerometer (deg, + = nose up).
 *
 * pitch = atan2(ax, sqrt(ay^2 + az^2)). Uses gravity vector only;
 * valid when linear accel is small vs 1g (low-pass before calling).
 *
 * @param[in] p_g Calibrated accel in g (z includes 1g gravity).
 * @return Pitch in degrees, range -90 to +90.
 */
float imu_pitch_deg (const imu_accel_g_t * p_g);

/*!
 * @brief Roll from accelerometer (deg).
 *
 * @param[in] p_g Calibrated accel in g.
 * @return Roll in degrees.
 */
float imu_roll_deg (const imu_accel_g_t * p_g);

#endif /* IMU_LSM303_H */

/*** end of file ***/
