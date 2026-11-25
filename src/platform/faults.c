/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	faults.c
 *	Shared system fault register (SIO Spinlock 1).
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
static UW g_fault_reg = 0U;
static UW g_sticky_reg = 0U;
static system_fault_observer_t gp_observer = NULL;

static uint32_t const g_owner_mask[FAULT_OWNER_COUNT] = {
    FAULT_WIFI_DISCONN_BIT | FAULT_MQTT_NACK_BIT,
    FAULT_MOTOR_STALL_BIT | FAULT_ENCODER_DEAD_BIT | FAULT_ESTOP_LATCHED_BIT,
    FAULT_LINE_LOST_BIT,
    FAULT_IMU_NACK_BIT | FAULT_STALE_SPEED_BIT | FAULT_INVALID_TILT_BIT |
        FAULT_LIFTED_BIT | FAULT_OVERTURNED_BIT,
    FAULT_SONAR_NO_ECHO_BIT | FAULT_ECHO_LINE_BIT | FAULT_OBSTACLE_STOPPED_BIT,
    FAULT_TASK_STALLED_BIT | FAULT_ABNORMAL_RESET_BIT,
};

void system_fault_bind_observer(system_fault_observer_t p_observer)
{
    gp_observer = p_observer;
}

static void notify_observer (UW changed, UW level)
{
    T_RSYS state;

    if ((0U != changed) && (NULL != gp_observer) && (E_OK == tk_ref_sys(&state)) &&
        (0U == (state.sysstat & (TSS_INDP | TSS_DINT | TSS_DDSP))))
    {
        gp_observer((uint32_t)changed, (uint32_t)level);
    }
}

uint32_t system_fault_owner_mask(fault_owner_t owner)
{
    uint32_t mask = 0U;

    if ((uint32_t)owner < (uint32_t)FAULT_OWNER_COUNT)
    {
        mask = g_owner_mask[owner];
    }

    return mask;
}

void system_fault_assign(fault_owner_t owner, uint32_t mask, uint32_t active)
{
    uint32_t const owned    = mask & system_fault_owner_mask(owner);
    UW const       to_set   = (UW)(owned & active);
    UW const       to_clear = (UW)(owned & ~active);

    if (0U != to_set)
    {
        UW const prior = atomic_bitset(&g_fault_reg, to_set);

        (void)atomic_bitset(&g_sticky_reg, to_set);
        notify_observer(to_set & ~prior, prior | to_set);
    }
    if (0U != to_clear)
    {
        UW const prior = atomic_bitclr(&g_fault_reg, to_clear);

        notify_observer(to_clear & prior, prior & ~to_clear);
    }
}

uint32_t system_fault_get(void)
{
    /* Setting no bits returns the current word under the same lock. */
    return (uint32_t)atomic_bitset(&g_fault_reg, 0U);
}

uint32_t system_fault_sticky_get(void)
{
    return (uint32_t)atomic_bitset(&g_sticky_reg, 0U);
}

void system_fault_sticky_clear(uint32_t fault_bit)
{
    UW const bits = (UW)(fault_bit & FAULT_ALL_MASK);

    (void)atomic_bitclr(&g_sticky_reg, bits);
    /* A bit set between the two calls is in the sticky word either way. */
    (void)atomic_bitset(&g_sticky_reg, bits & atomic_bitset(&g_fault_reg, 0U));
}
