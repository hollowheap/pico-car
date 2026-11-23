#ifndef TEST_MOTION_BSP_H
#define TEST_MOTION_BSP_H

#include <tk/smp.h>

#define E_BUSY (-65)
#define E_IO (-57)
#define E_SYS (-5)
#define TSK_SELF (0)
#define TA_STA (0x02U)
#define TA_ASSPRC (0x04U)
#define GPIO_MODE_IN (0U)
#define TIMER_TIMERAWH (TIMER_BASE + 0x24U)

typedef uint32_t RELTIM;
typedef struct
{
    ATR cycatr;
    FP cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
    UINT assprc;
} T_CCYC;

ID tk_get_tid(void);
ID tk_cre_cyc(const T_CCYC * p_config);
ER tk_del_cyc(ID cyclic_id);
ER tk_wup_tsk(ID task_id);
ER tk_slp_tsk(TMO timeout);
INT tk_can_wup(ID task_id);
ER gpio_set_pin(UINT pin, UINT mode);
UINT gpio_get_val(UINT pin);
UW GPIO(UINT pin);
void clr_w(UW address, UW bits);

static inline ER ISpinLock(T_SPLOCK * p_lock, UINT * p_saved)
{
    *p_saved = 0U;
    return SpinLock(p_lock);
}

static inline ER ISpinUnlock(T_SPLOCK * p_lock, UINT saved)
{
    (void)saved;
    return SpinUnlock(p_lock);
}

#endif
