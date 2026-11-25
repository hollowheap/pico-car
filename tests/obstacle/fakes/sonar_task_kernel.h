#ifndef TEST_SONAR_TASK_KERNEL_H
#define TEST_SONAR_TASK_KERNEL_H

#include <tk/flg_fake.h>

typedef struct { int32_t hi; uint32_t lo; } SYSTIM;

ER tk_get_otm (SYSTIM * p_time);
ER tk_clr_flg (ID flag, UINT pattern);

#endif
