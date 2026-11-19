/** @file i2c_rp2040.h
 *
 * @brief Polled RP2040 I2C1 master on GP2 (SDA) / GP3 (SCL), 400 kHz.
 *
 * Target-only. Register-level, like the motion subsystem's PWM and encoder code, because
 * the firmware build does not compile the pico-sdk hardware_i2c, and the
 * kernel's "iic" driver puts I2C1 on GP6/GP7, not GP2/GP3, with a 1000 ms
 * timeout.
 *
 * The Hump task is the only user of I2C1. No interrupts:
 * a 6-byte read takes about 0.2 ms at 400 kHz, so busy-waiting costs about
 * 1% of one core at 50 Hz. Timeouts use the 1 MHz TIMERAWL counter.
 */

#ifndef I2C_RP2040_H
#define I2C_RP2040_H

#include <stdint.h>

#include "lsm303.h"

#define I2C_SDA_PIN     (2U)
#define I2C_SCL_PIN     (3U)
/* Every register poll of one transaction stops this long after the transaction
 * starts. A failed attempt then costs at most 5 ms of polling, 1 ms of abort
 * and about 0.1 ms of bus clear, so a read that fails all its attempts holds
 * the physics task (priority 5, core 0) for about 6 ms per attempt: 25 ms at
 * 4 attempts, past its 20 ms period. Control tasks at priorities 1 to 4 still
 * preempt it. */
#define I2C_TIMEOUT_US  (5000U)

/* Reset I2C1, route GP2/GP3 to it and set 400 kHz for clk_sys = 125 MHz.
 * Fills *p_bus with this driver's callbacks. */
void i2c_rp2040_init (i2c_bus_t * p_bus);

/* Microsecond timestamp from the RP2040 timer (wraps every 71.6 min). */
uint32_t physics_time_us (void);

#endif /* I2C_RP2040_H */

/*** end of file ***/
