/*
 *	liveness.h
 *	Task liveness bits. Each task kicks its bit once per pass. The
 *	supervisor takes the word every 100 ms. Same SIO-spinlock atomics as the
 *	fault register: no lock is held by the caller.
 */

#ifndef PLATFORM_LIVENESS_H
#define PLATFORM_LIVENESS_H

#include <stdint.h>

/* A task that blocks between kicks waits at most this long per wait, then
   kicks again, so it stays well inside the supervisor's stall budget
   (LIVENESS_MISS_LIMIT checks of 100 ms). */
#define LIVENESS_WAIT_MS (100U)

/* Set the bit of one task (a LIVE_* id). Safe from any task on either core.
   An id of LIVE_TASKS or more is ignored. */
void liveness_kick (uint8_t task_bit);

/* Return the bits kicked since the last call and clear them. Call from the
   supervisor only. */
uint16_t liveness_take (void);

#endif /* PLATFORM_LIVENESS_H */
