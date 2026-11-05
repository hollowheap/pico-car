/** @file b4_i2c_rp2040.h
 *
 * @brief Polled RP2040 I2C1 master on GP2 (SDA) / GP3 (SCL), 400 kHz.
 *
 * Target-only. Register-level, like Buddy 2's PWM and encoder code, because
 * the team's trimmed pico-sdk has no hardware_i2c, and the kernel's "iic"
 * driver puts I2C1 on GP6/GP7 (Buddy 3's IR pins) with a 1000 ms timeout.
 *
 * The Hump task is the only user of I2C1 (SE01_PostTri1). No interrupts:
 * a 6-byte read takes about 0.2 ms at 400 kHz, so busy-waiting costs about
 * 1% of one core at 50 Hz. Timeouts use the 1 MHz TIMERAWL counter.
 */

#ifndef B4_I2C_RP2040_H
#define B4_I2C_RP2040_H

#include <stdint.h>

#include "lsm303.h"

#define B4_I2C_SDA_PIN     (2U)
#define B4_I2C_SCL_PIN     (3U)
#define B4_I2C_TIMEOUT_US  (5000U)   /* Per transaction (PostTri1).  */

/* Reset I2C1, route GP2/GP3 to it and set 400 kHz for clk_sys = 125 MHz.
 * Fills *p_bus with this driver's callbacks. */
void b4_i2c_rp2040_init (b4_i2c_bus_t * p_bus);

/* Microsecond timestamp from the RP2040 timer (wraps every 71.6 min). */
uint32_t b4_time_us (void);

#endif /* B4_I2C_RP2040_H */

/*** end of file ***/
