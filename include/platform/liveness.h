/*
 *	liveness.h
 *	Task liveness bits (D160). Each task kicks its bit once per pass. The
 *	supervisor takes the word every 100 ms. Same SIO-spinlock atomics as the
 *	fault register: no lock is held by the caller.
 */

#ifndef PLATFORM_LIVENESS_H
#define PLATFORM_LIVENESS_H

#include <stdint.h>

/* Set the bit of one task. Safe from any task on either core. */
void liveness_kick (uint8_t task_bit);

/* Return the bits kicked since the last call and clear them. Call from the
   supervisor only. */
uint16_t liveness_take (void);

#endif /* PLATFORM_LIVENESS_H */
