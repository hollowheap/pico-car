/** @file pwm.h
 * @brief Single-task ROBO-PICO motor output, with an explicit stop latch.
 */
#ifndef PWM_H
#define PWM_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

/* Startup only, before the owner task initialises PWM. No PWM is enabled. */
void pwm_boot_brake (void);

/* Call once from the motor-owner task on core 0. Uses only PWM slices 4/5,
 * GP8/9 and GP10/11, at 10 kHz with the pinned 125 MHz system clock.
 * Refuses active slices/interrupt use. Do not allocate these slices as
 * physical timers or write motor pins from other tasks after initialisation. */
ER pwm_init (void);

/* Owner-only two-wheel output. Signed permille -1000..1000, positive forward
 * for the physically verified car. Same-direction duties may update in place.
 * A requested reversal brakes BOTH wheels and returns E_BUSY; wait a control
 * interval before retrying from brake. No automatic reversal or dead-time
 * duration guarantee. Both zero brakes immediately. Hardware compare writes
 * are per-slice, not an atomic two-wheel transaction; do not assume zero
 * output skew. */
ER pwm_set_pair (int16_t left, int16_t right);
ER pwm_brake (void);

/* Direct emergency path. Latches stopped and forces outputs low without the
 * owner check. Call from task or ISR context only for an emergency. */
void pwm_emergency_stop (void);

/* Brakes and rejects further drive commands until reboot. This is a software
 * stop, not a guarantee against task/CPU failure or carrier-button override. */
ER pwm_latch_stop (void);

#endif /* PWM_H */
