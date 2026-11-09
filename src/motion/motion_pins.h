/** @file motion_pins.h
 * @brief Motion wiring verified on the assembled car.
 */
#ifndef MOTION_PINS_H
#define MOTION_PINS_H

#define MOTOR_LEFT_A  8U
#define MOTOR_LEFT_B  9U
#define MOTOR_RIGHT_A 10U
#define MOTOR_RIGHT_B 11U
#define MOTOR_ENCODER_LEFT_A 4U
#define MOTOR_ENCODER_LEFT_B 5U
#define MOTOR_ENCODER_RIGHT_A 16U
#define MOTOR_ENCODER_RIGHT_B 17U

/* GPIO pad control bits: unsigned copies of the kernel's signed GPIO_PUE,
 * GPIO_PDE, GPIO_IE and GPIO_SHEMITT, which BARR-C 5.3.b forbids in bitwise
 * expressions. */
#define MOTION_PAD_PULL_UP       (1UL << 3U)
#define MOTION_PAD_PULL_DOWN     (1UL << 2U)
#define MOTION_PAD_INPUT_ENABLE  (1UL << 6U)
#define MOTION_PAD_SCHMITT       (1UL << 1U)

#endif /* MOTION_PINS_H */

/*** end of file ***/
