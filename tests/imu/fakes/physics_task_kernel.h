#ifndef TEST_PHYSICS_TASK_KERNEL_H
#define TEST_PHYSICS_TASK_KERNEL_H

#include <stddef.h>
#include <tk/smp.h>

typedef uint32_t RELTIM;
typedef struct { int32_t hi; uint32_t lo; } SYSTIM;
typedef struct
{
    ATR cycatr;
    FP cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
} T_CCYC;

#define TA_STA (2U)
#define TSK_SELF (0)

ID tk_cre_cyc (const T_CCYC * p_cyclic);
ER tk_slp_tsk (TMO timeout);
ER tk_get_otm (SYSTIM * p_time);
ER tk_wup_tsk (ID task);
INT tk_can_wup (ID task);

static inline ER ISpinLock (T_SPLOCK * lock, UINT * saved)
{
    *saved = 0U;
    return SpinLock (lock);
}

static inline ER ISpinUnlock (T_SPLOCK * lock, UINT saved)
{
    (void)saved;
    return SpinUnlock (lock);
}

#endif
