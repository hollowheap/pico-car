/** @file drv_sonar.h
 *
 * @brief Buddy 5 hardware: HC-SR04+ on GP0/GP1, SG90 on GP12 (PWM slice 6
 *        channel A), the 1 MHz TIMER and the ECHO edge ISR.
 */

#ifndef EXTRAS_SONAR_H
#define EXTRAS_SONAR_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

ER       extras_sonar_init (ID h_echo_flag);
void     extras_sonar_servo_set (int16_t deci);
uint32_t extras_sonar_now_us (void);
bool     extras_sonar_rearm (void);
void     extras_sonar_trigger (void);
uint32_t extras_sonar_mailbox (void);
bool     extras_sonar_is_echo_high (void);

#endif /* EXTRAS_SONAR_H */

/*** end of file ***/
