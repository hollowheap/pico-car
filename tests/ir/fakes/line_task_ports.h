/** Kernel and supervisor configuration seams for the line-task host probe. */
#ifndef LINE_TASK_TEST_PORTS_H
#define LINE_TASK_TEST_PORTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include "supervisor/health_ids.h"

typedef struct
{
    int32_t hi;
    uint32_t lo;
} SYSTIM;

ER tk_get_otm(SYSTIM * p_time);

ER ISpinLock(T_SPLOCK * p_lock, UINT * p_saved);

static inline ER ISpinUnlock(T_SPLOCK * p_lock, UINT saved)
{
    (void)saved;
    return SpinUnlock(p_lock);
}

/* The firmware supervisor's startup configuration is 32-bit ABI only. The
 * test drives the line task through the supervisor command queries it uses. */
#define SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_NO_TURN (0xFFU)
uint8_t supervisor_poll_turn(uint8_t * p_source);
/* Select or cancel only the route instruction whose opcode still matches. */
bool supervisor_route_pending(uint8_t expected_opcode, bool b_consume);
uint8_t supervisor_reset_count(void);
bool supervisor_wheels_loaded(void);
uint8_t supervisor_start_snapshot(bool * p_continuous_floor);
uint8_t supervisor_armed_start(bool * p_armed);
void supervisor_start_check_failed(uint8_t generation);
uint8_t supervisor_recovery_snapshot(bool * p_clear);
void supervisor_set_turning(bool b_turning);
void supervisor_turn_refused(uint8_t opcode);

#endif
