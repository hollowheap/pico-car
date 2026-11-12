/** @file log_core.c
 * @brief Pure parts of the log module (D100). See log_core.h.
 */

#include <stddef.h>

#include "log_core.h"

#define RING_MASK (LOG_RING_LEN - 1U)

static char const * const g_sev_name[LOG_SEV_COUNT] = {
    "ERROR", "WARN", "INFO", "DEBUG"};

static char const * const g_task_name[LOG_TASK_COUNT] = {
    "boot", "motion",   "supervisor", "line",      "obstacle",
    "physics", "barcode", "telemetry", "connection", "log"};

/* Index 0 (LOG_MOD_NONE) has no name. */
static char const * const g_mod_name[LOG_MOD_COUNT] = {
    "", "nav", "scan", "tcp", "mqtt", "imu", "hump", "pid"};

STATIC_ASSERT (sizeof(g_sev_name) / sizeof(g_sev_name[0]) == LOG_SEV_COUNT,
               "severity name table");
STATIC_ASSERT (sizeof(g_task_name) / sizeof(g_task_name[0]) == LOG_TASK_COUNT,
               "task name table");
STATIC_ASSERT (sizeof(g_mod_name) / sizeof(g_mod_name[0]) == LOG_MOD_COUNT,
               "module name table");

/* Append text to a buffer; keep the terminator; return the new length. */
static uint32_t append (char * p_buf, uint32_t size, uint32_t len, char const * p_text)
{
    uint32_t out = len;

    while (('\0' != *p_text) && ((out + 1U) < size))
    {
        p_buf[out] = *p_text;
        out++;
        p_text++;
    }
    p_buf[out] = '\0';

    return out;
}

void log_ring_init (log_ring_t * p_ring)
{
    if (NULL != p_ring)
    {
        p_ring->head    = 0U;
        p_ring->count   = 0U;
        p_ring->dropped = 0U;
    }
}

bool log_ring_push (log_ring_t * p_ring, log_record_t const * p_rec)
{
    bool b_stored = false;

    if ((NULL != p_ring) && (NULL != p_rec))
    {
        if (p_ring->count < LOG_RING_LEN)
        {
            p_ring->rec[p_ring->head] = *p_rec;
            p_ring->head              = (uint16_t)((p_ring->head + 1U) & RING_MASK);
            p_ring->count++;
            b_stored = true;
        }
        else
        {
            p_ring->dropped++;
            if (p_rec->sev <= (uint8_t)LOG_SEV_WARN)
            {
                /* Full: the new record replaces the oldest one. The oldest
                 * slot is the one the head points at. */
                p_ring->rec[p_ring->head] = *p_rec;
                p_ring->head              = (uint16_t)((p_ring->head + 1U) & RING_MASK);
                b_stored                  = true;
            }
        }
    }

    return b_stored;
}

bool log_ring_pop (log_ring_t * p_ring, log_record_t * p_rec)
{
    bool b_popped = false;

    if ((NULL != p_ring) && (NULL != p_rec) && (p_ring->count > 0U))
    {
        uint16_t const oldest = (uint16_t)(
            ((uint32_t)p_ring->head - (uint32_t)p_ring->count) & RING_MASK);

        *p_rec = p_ring->rec[oldest];
        p_ring->count--;
        b_popped = true;
    }

    return b_popped;
}

uint16_t log_ring_count (log_ring_t const * p_ring)
{
    return (NULL != p_ring) ? p_ring->count : 0U;
}

uint32_t log_ring_dropped (log_ring_t const * p_ring)
{
    return (NULL != p_ring) ? p_ring->dropped : 0U;
}

/* Characters that can sit between '%' and the conversion letter. */
static bool is_spec_prefix (char ch)
{
    bool b_prefix = false;
    char const * p_set = "-+ #.0123456789lhzjt";

    while (('\0' != *p_set) && (!b_prefix))
    {
        b_prefix = (ch == *p_set);
        p_set++;
    }

    return b_prefix;
}

uint8_t log_count_args (char const * p_fmt)
{
    uint8_t count = 0U;

    if (NULL != p_fmt)
    {
        while (('\0' != *p_fmt) && (count < LOG_MAX_ARGS))
        {
            if ('%' != *p_fmt)
            {
                p_fmt++;
            }
            else
            {
                p_fmt++;
                if ('%' == *p_fmt)
                {
                    p_fmt++;
                }
                else
                {
                    while (('\0' != *p_fmt) && is_spec_prefix (*p_fmt))
                    {
                        p_fmt++;
                    }
                    if ('\0' != *p_fmt)
                    {
                        /* The conversion letter itself. */
                        count++;
                        p_fmt++;
                    }
                }
            }
        }
    }

    return count;
}

uint32_t log_prefix (char * p_buf, uint32_t size, log_record_t const * p_rec)
{
    uint32_t len = 0U;

    if ((NULL != p_buf) && (size > 0U))
    {
        p_buf[0] = '\0';
        if (NULL != p_rec)
        {
            len = append (p_buf, size, len, "[");
            len = append (p_buf, size, len,
                          (p_rec->sev < (uint8_t)LOG_SEV_COUNT)
                              ? g_sev_name[p_rec->sev]
                              : "?");
            len = append (p_buf, size, len, "](");
            len = append (p_buf, size, len,
                          (p_rec->task < (uint8_t)LOG_TASK_COUNT)
                              ? g_task_name[p_rec->task]
                              : "?");
            if ((p_rec->mod > (uint8_t)LOG_MOD_NONE) &&
                (p_rec->mod < (uint8_t)LOG_MOD_COUNT))
            {
                len = append (p_buf, size, len, ":");
                len = append (p_buf, size, len, g_mod_name[p_rec->mod]);
            }
            len = append (p_buf, size, len, ") ");
        }
    }

    return len;
}

bool log_same (log_record_t const * p_a, log_record_t const * p_b)
{
    bool b_same = false;

    if ((NULL != p_a) && (NULL != p_b) && (p_a->p_fmt == p_b->p_fmt) &&
        (p_a->sev == p_b->sev) && (p_a->task == p_b->task) &&
        (p_a->mod == p_b->mod) && (p_a->nargs == p_b->nargs))
    {
        uint8_t idx = 0U;

        b_same = true;
        for (idx = 0U; idx < p_a->nargs; idx++)
        {
            if (p_a->arg[idx] != p_b->arg[idx])
            {
                b_same = false;
            }
        }
    }

    return b_same;
}

/*** end of file ***/
