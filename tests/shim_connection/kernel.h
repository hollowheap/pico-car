#ifndef TEST_CONNECTION_KERNEL_H
#define TEST_CONNECTION_KERNEL_H

#include <tk/smp.h>

#define TA_STA (2U)
#define TA_ASSPRC (4U)
#define TSK_SELF (0)

typedef uint32_t RELTIM;

typedef struct
{
    ATR cycatr;
    FP cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
    UINT assprc;
} T_CCYC;

ID tk_get_tid (void);
ID tk_cre_cyc (const T_CCYC * p_config);
ER tk_slp_tsk (TMO timeout);
ER tk_wup_tsk (ID task);
INT tk_can_wup (ID task);

ER ISpinLock (T_SPLOCK * p_lock, UINT * p_saved);
ER ISpinUnlock (T_SPLOCK * p_lock, UINT saved);

#endif
