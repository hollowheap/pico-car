/** @file scan_hw.h
 *
 * @brief Buddy 5 hardware: HC-SR04+ on GP0/GP1, SG90 on GP12 (PWM slice 6
 *        channel A), the 1 MHz TIMER and the ECHO edge ISR.
 */

#ifndef SCAN_HW_H
#define SCAN_HW_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

ER       scan_hw_init (ID h_echo_flag);
void     scan_hw_servo_set (int16_t deci);
uint32_t scan_hw_now_us (void);
bool     scan_hw_rearm (void);
bool     scan_hw_trigger (void);
uint32_t scan_hw_echo_result (void);
bool     scan_hw_is_echo_high (void);

#endif /* SCAN_HW_H */

/*** end of file ***/

