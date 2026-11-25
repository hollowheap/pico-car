/** @file drv_i2c.h
 *
 * @brief Interrupt-driven RP2040 I2C1 master on GP2 (SDA) / GP3 (SCL), 400 kHz.
 *
 * Target-only. Register-level, like the motion subsystem's PWM and encoder
 * code, because the firmware build does not compile the pico-sdk hardware_i2c,
 * and the kernel's "iic" driver puts I2C1 on GP6/GP7, not GP2/GP3, with a
 * 1000 ms timeout.
 *
 * The physics task is the sole owner of I2C1. Controller completion and FIFO
 * interrupts wake the waiting task through a uT-Kernel event flag. A TIMER
 * alarm sets a timeout bit on the same flag I2C_TIMEOUT_US after the transfer
 * starts, so the wait is not quantised by the kernel tick.
 */

#ifndef I2C_RP2040_H
#define I2C_RP2040_H

#include <stdint.h>

#include "drv_lsm303.h"

#define I2C_SDA_PIN    (2U)
#define I2C_SCL_PIN    (3U)
#define I2C_RP2040_IRQ (24U)
/* Microsecond deadline for one transaction, counted from the moment its alarm
 * is armed. A stuck transfer retires at this deadline, then aborts and
 * disables the controller (each up to 1 ms), so a timed-out attempt ends
 * within I2C_TIMEOUT_US + 2 ms of arming. A wedged controller adds up to 2 ms
 * before arming (disable, then block reset). The bounded waits of one
 * transaction total at most 9 ms: 2 ms pre-arm, 5 ms deadline, 2 ms abort and
 * disable. */
#define I2C_TIMEOUT_US (5000U)

/* Reset I2C1, route GP2/GP3 to it, register the I2C1 completion interrupt,
 * and set 400 kHz for clk_sys = 125 MHz. Fills *p_bus with this driver's
 * callbacks. A failed block reset here leaves the bus down until reboot:
 * write_read returns I2C_BUS_ERROR and no interrupt is registered. */
void i2c_rp2040_init (i2c_bus_t * p_bus);

/* Microsecond timestamp from the RP2040 timer (wraps every 71.6 min). */
uint32_t physics_time_us (void);

#endif /* I2C_RP2040_H */

/*** end of file ***/
