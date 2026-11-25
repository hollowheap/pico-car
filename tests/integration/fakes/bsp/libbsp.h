#ifndef INTEGRATION_LIBBSP_H
#define INTEGRATION_LIBBSP_H

#include <tk/smp.h>

#define GPIO_MODE_IN  (0U)
#define GPIO_MODE_OUT (1U)

ER   gpio_set_pin (UINT pin, UINT mode);
UINT gpio_get_val (UINT pin);
UW   GPIO (UINT pin);
void clr_w (UW address, UW bits);

#endif
