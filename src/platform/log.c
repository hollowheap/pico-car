/** @file log.c
 * @brief Target side of the log module: the lock, the producer, the drain
 *        and the flush (D100). The ring and the text parts are in log_core.c.
 *
 * Producers run in any task on either core. They copy one record under a
 * spinlock and return. The drain runs in one low-priority task, pops records
 * and prints them. Nothing here blocks a real-time task.
 *
 * NOTE: The kernel types UINT, UB, INT, SYSTIM and T_SPLOCK are used where
 * the kernel API requires them (deviation from BARR-C 5.2.a, see
 * artifacts/notes/barr-c-deviations.md).
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>

#include "platform/log.h"
#include "log_core.h"
#include "../usb_console_compat.h"

/* tm_usb_state() value once a serial monitor has opened the port. */
#define USB_OPEN (3U)

/* Lines printed per drain pass, and the most bytes left in the console ring
 * before another line is printed. Same pacing as the obstacle flight log. */
#define LINES_PER_PASS (4U)
#define PENDING_MAX    (1024U)

/* log_flush() waits at most FLUSH_STEPS x FLUSH_STEP_US for the console. */
#define FLUSH_STEPS   (200U)
#define FLUSH_STEP_US (1000U)

/* Argument slots, named so the print call reads without bare numbers. */
enum
{
    ARG_0 = 0,
    ARG_1,
    ARG_2,
    ARG_3,
    ARG_4,
    ARG_5,
    ARG_6,
    ARG_7
};

static log_ring_t   g_ring;
static T_SPLOCK     g_lock;
static log_record_t g_last;
static uint32_t     g_repeat;
static bool         g_have_last;

static uint32_t now_ms (void)
{
    SYSTIM tim = {0, 0U};

    (void)tk_get_otm (&tim);

    return tim.lo;
}

static bool pop_record (log_record_t * p_rec)
{
    UINT saved  = 0U;
    bool b_have = false;

    (void)ISpinLock (&g_lock, &saved);
    b_have = log_ring_pop (&g_ring, p_rec);
    (void)ISpinUnlock (&g_lock, saved);

    return b_have;
}

/* Print one record: prefix, then the message with its stored arguments. The
 * format may use fewer than eight; the rest are ignored. */
static void print_record (log_record_t const * p_rec)
{
    char     line[LOG_LINE_MAX];
    uint32_t len = log_prefix (line, (uint32_t)sizeof (line), p_rec);

    (void)tm_sprintf ((UB *)&line[len], (UB const *)p_rec->p_fmt,
                      (int)p_rec->arg[ARG_0], (int)p_rec->arg[ARG_1],
                      (int)p_rec->arg[ARG_2], (int)p_rec->arg[ARG_3],
                      (int)p_rec->arg[ARG_4], (int)p_rec->arg[ARG_5],
                      (int)p_rec->arg[ARG_6], (int)p_rec->arg[ARG_7]);
    (void)tm_putstring ((UB const *)line);
    (void)tm_putstring ((UB const *)"\n");
}

/* Say how many identical records were skipped. */
static void print_repeats (void)
{
    if (g_repeat > 0U)
    {
        log_record_t note;

        note.t_ms  = 0U;
        note.p_fmt = "previous message repeated %d times";
        note.sev   = (uint8_t)LOG_SEV_INFO;
        note.task  = (uint8_t)LOG_TASK_LOG;
        note.mod   = (uint8_t)LOG_MOD_NONE;
        note.nargs = 1U;
        note.arg[ARG_0] = (int32_t)g_repeat;
        print_record (&note);
        g_repeat = 0U;
    }
}

/* Print records until the limit, the console is full, or the ring is empty.
 * Returns the number of lines printed. */
static uint32_t drain_lines (uint32_t limit)
{
    uint32_t     lines = 0U;
    bool         b_more = true;
    log_record_t rec;

    while (b_more && (lines < limit) && (tm_usb_pending () < PENDING_MAX))
    {
        if (!pop_record (&rec))
        {
            print_repeats ();
            b_more = false;
        }
        else if (g_have_last && log_same (&rec, &g_last))
        {
            g_repeat++;
        }
        else
        {
            print_repeats ();
            print_record (&rec);
            g_last      = rec;
            g_have_last = true;
            lines++;
        }
    }

    return lines;
}

void log_init (void)
{
    (void)InitSpinLock (&g_lock);
    log_ring_init (&g_ring);
    g_repeat    = 0U;
    g_have_last = false;
}

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod,
                char const * p_fmt, ...)
{
    log_record_t rec;
    va_list      ap;
    UINT         saved = 0U;
    uint8_t      idx   = 0U;

    rec.t_ms  = now_ms ();
    rec.p_fmt = p_fmt;
    rec.sev   = (uint8_t)sev;
    rec.task  = (uint8_t)task;
    rec.mod   = (uint8_t)mod;
    rec.nargs = log_count_args (p_fmt);

    va_start (ap, p_fmt);
    for (idx = 0U; idx < LOG_MAX_ARGS; idx++)
    {
        /* Every argument is int sized, and a %s argument is a 32-bit
         * address on this target. Unused slots stay zero. */
        rec.arg[idx] = (idx < rec.nargs) ? (int32_t)va_arg (ap, int) : 0;
    }
    va_end (ap);

    (void)ISpinLock (&g_lock, &saved);
    (void)log_ring_push (&g_ring, &rec);
    (void)ISpinUnlock (&g_lock, saved);
}

void log_drain (void)
{
    if (tm_usb_state () >= USB_OPEN)
    {
        (void)drain_lines (LINES_PER_PASS);
    }
}

void log_flush (void)
{
    uint32_t step = 0U;

    while ((step < FLUSH_STEPS) && (tm_usb_state () >= USB_OPEN))
    {
        if (tm_usb_pending () >= PENDING_MAX)
        {
            WaitUsec (FLUSH_STEP_US);
            step++;
        }
        else if (0U == drain_lines (LINES_PER_PASS))
        {
            step = FLUSH_STEPS;
        }
        else
        {
            /* Printed a batch; go round again. */
        }
    }
}

uint32_t log_dropped (void)
{
    UINT     saved = 0U;
    uint32_t count = 0U;

    (void)ISpinLock (&g_lock, &saved);
    count = log_ring_dropped (&g_ring);
    (void)ISpinUnlock (&g_lock, saved);

    return count;
}

/*** end of file ***/
