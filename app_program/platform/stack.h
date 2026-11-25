/** @file stack.h
 *
 * @brief Task stack registry and the free-stack report.
 *
 * Every task stack is a static array of its subsystem. The subsystem
 * registers it in its init function, before any task exists, and the
 * registry fills it with a known byte. The telemetry task then counts how
 * much of each stack is still untouched.
 *
 * Each stack is the deepest call chain of the task, from the GCC call graph
 * (-fstack-usage -fcallgraph-info=su, function pointers followed by hand),
 * plus a quarter, plus 1152 bytes for what lands on top of it. The chain
 * includes the SMP kernel path, which runs the ready-queue assignment of each
 * critical section on the calling task's stack. On top: three nested
 * interrupt levels (SysTick 240 B, GPIO 344 B, I2C or UART 288 B, a 32-byte
 * frame each) and the dispatcher's saved registers. Physics adds 400 bytes
 * for newlib's sinf, atan2f and acosf, which the call graph does not see.
 * Rounded up to 256 bytes. Deepest chain per task: startup 672, motion 984,
 * physics 880, supervisor 1016, obstacle 568, line 936, telemetry 1208,
 * connection 680, barcode 808.
 */

#ifndef PLATFORM_STACK_H
#define PLATFORM_STACK_H

#include <stdint.h>

#define PLATFORM_STACK_ALIGN (8U)

/* Slot order is the order of the stack report. */
typedef enum
{
    PLATFORM_STACK_STARTUP = 0,
    PLATFORM_STACK_MOTION,
    PLATFORM_STACK_PHYSICS,
    PLATFORM_STACK_OBSTACLE,
    PLATFORM_STACK_LINE,
    PLATFORM_STACK_TELEMETRY,
    PLATFORM_STACK_CONNECTION,
    PLATFORM_STACK_BARCODE,
    PLATFORM_STACK_SUPERVISOR,
    PLATFORM_STACK_COUNT
} platform_stack_slot_t;

/* The smallest free stack seen by platform_stack_report (low 16 bits) and its
 * slot (bits 16 to 23), packed in one word for the diag packet. */
#define PLATFORM_STACK_MARK_FREE_MASK  (0xFFFFU)
#define PLATFORM_STACK_MARK_SLOT_SHIFT (16U)
#define PLATFORM_STACK_MARK_SLOT_MASK  (0xFFU)

/* Record a stack and fill it with the known byte. Call before the task that
   uses it is created. */
void platform_stack_register (platform_stack_slot_t slot, uint8_t * p_base,
                              uint32_t size);

/* Scan the stacks and log their free bytes. The telemetry task calls this once
   a second; it scans on the first call and then every 30th. */
void platform_stack_report (void);

/* The packed smallest free stack, see PLATFORM_STACK_MARK_*. */
uint32_t platform_stack_mark (void);

#endif /* PLATFORM_STACK_H */

/*** end of file ***/
