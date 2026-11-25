#ifndef TEST_OBSTACLE_SERVO_BSP_H
#define TEST_OBSTACLE_SERVO_BSP_H

#include <tk/smp.h>

typedef UINT BOOL;

#define TRUE (1U)
#define E_SYS (-5)
#define XOSC_MHz (12U)
#define RESETS_RESET (0x4000C000U)
#define RESETS_RESET_DONE (0x4000C008U)
#define RESETS_RESET_PWM (0x00004000U)
#define RESETS_RESET_TIMER (0x00200000U)
#define PWM_BASE (0x40050000U)
#define PWM_CHx_CSR (0U)
#define PWM_CHx_DIV (4U)
#define GPIO_IN (0xD0000004U)
#define GPIO(pin) (0x4001C004U + (4U * (pin)))
#define GPIO_MODE_IN (0U)
#define GPIO_MODE_OUT (1U)
#define DI(saved) ((saved) = 0U)
#define EI(saved) ((void)(saved))

void WaitUsec(UW usec);
void clr_w(UW address, UW bits);
ER gpio_set_val(UINT pin, UINT value);
ER gpio_set_pin(UINT pin, UINT mode);
UINT gpio_get_val(UINT pin);
ER pwm_set_cc(UINT pin, UW compare);
ER pwm_set_wrap(UINT pin, UW wrap);
ER pwm_set_pin(UINT pin);
ER pwm_set_enabled(UINT pin, BOOL enabled);
ER tk_set_flg(ID flag, UINT mask);

#endif
