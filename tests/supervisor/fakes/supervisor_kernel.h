#ifndef TEST_SUPERVISOR_KERNEL_H
#define TEST_SUPERVISOR_KERNEL_H

#include <stddef.h>
#include <tk/smp.h>

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

ER tk_slp_tsk (TMO timeout);
typedef struct { int32_t hi; uint32_t lo; } SYSTIM;
ER tk_get_otm (SYSTIM * p_time);

/* Match the firmware's 4-byte alignment for this supervisor configuration
 * structure while retaining native host pointers and its size assertion. */
#pragma pack(push, 4)
#include "supervisor/supervisor.h"
#pragma pack(pop)

#endif
