#ifndef TEST_EMIT_KERNEL_H
#define TEST_EMIT_KERNEL_H

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

#endif
