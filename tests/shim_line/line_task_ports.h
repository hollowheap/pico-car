/** Kernel and supervisor configuration seams for the line-task host probe. */
#ifndef LINE_TASK_TEST_PORTS_H
#define LINE_TASK_TEST_PORTS_H

#include <stdbool.h>
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

static inline ER ISpinLock(T_SPLOCK * p_lock, UINT * p_saved)
{
    *p_saved = 0U;
    return SpinLock(p_lock);
}

static inline ER ISpinUnlock(T_SPLOCK * p_lock, UINT saved)
{
    (void)saved;
    return SpinUnlock(p_lock);
}

/* The firmware supervisor's startup configuration is 32-bit ABI only. The
 * test drives the line task through the supervisor command queries it uses. */
#define SUPERVISOR_SUPERVISOR_H
#define SUPERVISOR_NO_TURN (0xFFU)
uint8_t supervisor_take_turn(uint8_t * p_source);
uint8_t supervisor_reset_count(void);
uint8_t supervisor_start_count(void);
void supervisor_set_turning(bool b_turning);
void supervisor_turn_refused(uint8_t opcode);

#endif
