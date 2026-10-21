/** @file motor_pwm.h
 * @brief Single-task ROBO-PICO motor output, with an explicit stop latch.
 */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/* Startup only, before the owner task initialises PWM. No PWM is enabled. */
void motor_pwm_boot_brake (void);

/* Call once from the motor-owner task on core 0. Uses only PWM slices 4/5,
 * GP8/9 and GP10/11, at 10 kHz with the pinned 125 MHz system clock.
 * Refuses active slices/interrupt use. Do not allocate these slices as
 * physical timers or write motor pins from other tasks after initialisation. */
ER motor_pwm_init (void);

/* Owner-task calls only. wheel=0 is physical left, wheel=1 right. channel_a
 * chooses the A-side input, not calibrated car-forward. Duty is 0..1000.
 * A new drive requires an intervening brake; only one wheel is driven here. */
ER motor_pwm_drive (unsigned int wheel, bool channel_a, uint16_t duty_permille);
ER motor_pwm_brake (void);

/* Brakes and rejects further drive commands until reboot. This is a software
 * stop, not a guarantee against task/CPU failure or carrier-button override. */
ER motor_pwm_latch_stop (void);

#endif /* MOTOR_PWM_H */
