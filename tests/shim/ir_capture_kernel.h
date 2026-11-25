#ifndef TEST_IR_CAPTURE_KERNEL_H
#define TEST_IR_CAPTURE_KERNEL_H

#include <stddef.h>
#include <tk/tkernel.h>

#define TA_FIRST 0U
#define GPIO_MODE_IN 0U
#define GPIO_IN 0xD0000004U
#define GPIO(pin) (0x4001C004U + (4U * (pin)))
#define TIMER_ALARM0 (TIMER_BASE + 0x10U)
#define TIMER_INTR (TIMER_BASE + 0x34U)
#define TIMER_INTE (TIMER_BASE + 0x38U)

typedef struct
{
    void * exinf;
    ATR sematr;
    INT isemcnt;
    INT maxsem;
} T_CSEM;

ID tk_cre_sem (const T_CSEM * p_def);
ER tk_del_sem (ID sem);
ER tk_sig_sem (ID sem, INT count);
ER tk_wai_sem (ID sem, INT count, TMO timeout);
ER gpio_set_pin (UINT pin, UINT mode);
ER ir_capture_test_def_int (UINT vector, const T_DINT * p_definition);

#define tk_def_int ir_capture_test_def_int

static inline void clr_w (UW addr, UW bits)
{
    shim_reg_write (addr, shim_reg_read (addr) & ~bits);
}

#endif
