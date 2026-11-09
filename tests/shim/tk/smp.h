/** @file smp.h
 * @brief Host shim for the uT-Kernel SMP spinlock and processor API.
 */
#ifndef SHIM_TK_SMP_H
#define SHIM_TK_SMP_H

#include <tk/tkernel.h>

typedef volatile UW T_SPLOCK;

static inline ER InitSpinLock (T_SPLOCK * lock)
{
    *lock = 0U;
    return E_OK;
}

static inline ER SpinLock (T_SPLOCK * lock)
{
    *lock = 1U;
    return E_OK;
}

static inline ER SpinUnlock (T_SPLOCK * lock)
{
    *lock = 0U;
    return E_OK;
}

static inline void mp_memory_barrir (void)
{
}

/** Single-threaded host stand-ins for the kernel atomics. Both return the
 *  value held before the update, as the kernel does. */
static inline UW atomic_inc (UW * addr)
{
    const UW old = *addr;

    *addr = old + 1U;
    return old;
}

static inline UW atomic_add (UW * addr, UW val)
{
    const UW old = *addr;

    *addr = old + val;
    return old;
}

/** Processor the caller runs on, from 1. Defined and set by the test. */
extern INT g_shim_prc;

static inline INT tk_get_prc (void)
{
    return g_shim_prc;
}

#endif /* SHIM_TK_SMP_H */
