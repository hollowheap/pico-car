/*
 *	liveness.c
 *	Task liveness bits. See liveness.h.
 */

#include <stdint.h>

#include <tk/smp.h>
#include <tk/tkernel.h>

#include "platform/liveness.h"

/* The kernel atomics work on 32-bit words. */
static UW g_live_bits = 0U;

void liveness_kick (uint8_t task_bit)
{
    (void) atomic_bitset (&g_live_bits, (UW) (1UL << task_bit));
}

uint16_t liveness_take (void)
{
    UW const bits = atomic_bitset (&g_live_bits, 0U);

    (void) atomic_bitclr (&g_live_bits, bits);

    return (uint16_t) bits;
}
