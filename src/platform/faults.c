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
static UW s_fault_reg = 0U;

void system_fault_set(uint16_t fault_bit)
{
    (void)atomic_bitset(&s_fault_reg, (UW)(fault_bit & FAULT_ALL_MASK));
}

void system_fault_clear(uint16_t fault_bit)
{
    (void)atomic_bitclr(&s_fault_reg, (UW)(fault_bit & FAULT_ALL_MASK));
}

uint16_t system_fault_get(void)
{
    /* Setting no bits returns the current word under the same lock. */
    UW value = atomic_bitset(&s_fault_reg, 0U);

    return (uint16_t)value;
}
