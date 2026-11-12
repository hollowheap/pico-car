/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	faults.c
 *	Shared 16-bit system fault register (SIO Spinlock 1).
 *
 *	The kernel's atomic_bitset() and atomic_bitclr() claim SIO Spinlock 1 for
 *	the read-modify-write and mask local interrupts while they hold it.  The
 *	accessors below add no DI/EI of their own: the kernel already provides that
 *	protection, and a second mask would only repeat it.  Plain SRAM bit
 *	operations (hw_set_bits and similar) are not used here.
 */

#include <stdint.h>

#include <tk/tkernel.h>
#include <tk/smp.h>

#include "platform/faults.h"

/*
 * The address escapes to the external kernel atomics, which access it through
 * a volatile pointer (smp_lock.c atomic_bitset), so the compiler cannot cache it.
 */
/* Kernel atomics operate on 32-bit words; the public API masks to 16 bits. */
static UW g_fault_reg = 0U;
static UW g_sticky_reg = 0U;

void system_fault_set(uint16_t fault_bit)
{
    UW const bits = (UW)(fault_bit & FAULT_ALL_MASK);

    (void)atomic_bitset(&g_fault_reg, bits);
    (void)atomic_bitset(&g_sticky_reg, bits);
}

void system_fault_clear(uint16_t fault_bit)
{
    (void)atomic_bitclr(&g_fault_reg, (UW)(fault_bit & FAULT_ALL_MASK));
}

void system_fault_assign(uint16_t mask, uint16_t active)
{
    uint16_t const to_set   = (uint16_t)(mask & active);
    uint16_t const to_clear = (uint16_t)(mask & (uint16_t)~active);

    if (0U != to_set)
    {
        system_fault_set(to_set);
    }
    if (0U != to_clear)
    {
        system_fault_clear(to_clear);
    }
}

uint16_t system_fault_get(void)
{
    /* Setting no bits returns the current word under the same lock. */
    UW value = atomic_bitset(&g_fault_reg, 0U);

    return (uint16_t)value;
}

uint16_t system_fault_sticky_get(void)
{
    UW value = atomic_bitset(&g_sticky_reg, 0U);

    return (uint16_t)value;
}

void system_fault_sticky_clear(uint16_t fault_bit)
{
    (void)atomic_bitclr(&g_sticky_reg, (UW)(fault_bit & FAULT_ALL_MASK));
}
