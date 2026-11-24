#ifndef TEST_I2C_RP2040_KERNEL_H
#define TEST_I2C_RP2040_KERNEL_H

#include <tk/tkernel.h>

#define tk_def_int i2c_test_def_int

ER i2c_test_def_int (UINT intno, const T_DINT * p_definition);

#endif
