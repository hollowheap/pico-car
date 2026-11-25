/** @file stack.c
 * @brief Task stack registry and the free-stack report. See stack.h.
 */

#include "platform/stack.h"

#include "platform/log.h"
#include "platform/static_assert.h"

#define STACK_FILL_BYTE    (0xA5U)
#define STACK_MIN_FREE     (512U)
#define STACK_REPORT_CALLS (30U)
STATIC_ASSERT(PLATFORM_STACK_COUNT <= (PLATFORM_STACK_MARK_SLOT_MASK + 1U),
              "task index fits the stack mark");

static uint8_t *         g_stack_base[PLATFORM_STACK_COUNT];
static uint32_t          g_stack_size[PLATFORM_STACK_COUNT];
static volatile uint32_t g_stack_mark = PLATFORM_STACK_MARK_FREE_MASK;

void platform_stack_register (platform_stack_slot_t slot, uint8_t * p_base,
                              uint32_t size)
{
    uint32_t byte = 0U;

    g_stack_base[slot] = p_base;
    g_stack_size[slot] = size;
    for (byte = 0U; byte < size; byte++)
    {
        p_base[byte] = STACK_FILL_BYTE;
    }
}

uint32_t platform_stack_mark (void)
{
    return g_stack_mark;
}

/* Untouched bytes of one stack, from the deep (low) end. */
static uint32_t stack_free (uint32_t idx)
{
    uint32_t free_bytes = 0U;

    while ((free_bytes < g_stack_size[idx]) &&
           (STACK_FILL_BYTE == g_stack_base[idx][free_bytes]))
    {
        free_bytes++;
    }

    return free_bytes;
}

/* Every STACK_REPORT_CALLS calls it logs the free bytes of every stack, and a
 * warning for a thin margin. It runs at the telemetry priority, below every
 * control task, so control load delays a scan instead of the scan delaying
 * control. */
void platform_stack_report (void)
{
    static uint32_t s_calls = 0U;
    uint32_t        free_bytes[PLATFORM_STACK_COUNT];
    uint32_t        idx     = 0U;
    uint32_t        thin    = 0U;

    s_calls++;
    /* The first call scans too, so the diag packet has marks after 1 s. */
    if ((1U != s_calls) && (0U != (s_calls % STACK_REPORT_CALLS)))
    {
        return;
    }
    for (idx = 0U; idx < PLATFORM_STACK_COUNT; idx++)
    {
        free_bytes[idx] = stack_free(idx);

        if (free_bytes[idx] < STACK_MIN_FREE)
        {
            thin |= (1UL << idx);
        }
        /* Keep the smallest mark for telemetry (no scan per packet). The
         * stored mark is at most 0xFFFF, so a smaller one fits 16 bits. */
        if (free_bytes[idx] < (g_stack_mark & PLATFORM_STACK_MARK_FREE_MASK))
        {
            g_stack_mark = (idx << PLATFORM_STACK_MARK_SLOT_SHIFT) | free_bytes[idx];
        }
    }
    LOG_INFO(LOG_TASK_TELEMETRY, LOG_MOD_NONE,
             "stack free: startup %d motion %d physics %d obstacle %d",
             (int)free_bytes[0], (int)free_bytes[1], (int)free_bytes[2], (int)free_bytes[3]);
    LOG_INFO(LOG_TASK_TELEMETRY, LOG_MOD_NONE,
             "stack free: line %d telemetry %d connection %d barcode %d",
             (int)free_bytes[4], (int)free_bytes[5], (int)free_bytes[6], (int)free_bytes[7]);
    LOG_INFO(LOG_TASK_TELEMETRY, LOG_MOD_NONE, "stack free: supervisor %d", (int)free_bytes[8]);
    if (0U != thin)
    {
        LOG_WARN(LOG_TASK_TELEMETRY, LOG_MOD_NONE, "thin stack margin, mask=%d", (int)thin);
    }
}

/*** end of file ***/
