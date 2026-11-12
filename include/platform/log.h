/** @file log.h
 * @brief One logging paradigm for the whole firmware (D100, S41).
 *
 * Every line has the same shape:
 *
 *     [<SEVERITY>](<task>) message
 *     [<SEVERITY>](<task>:<module>) message
 *
 * Two ways to log. Pick by where the call runs.
 *
 *   LOG_ERROR / LOG_WARN / LOG_INFO / LOG_DEBUG
 *       Deferred. The call stores one fixed record in a ring and returns. It
 *       does not format and does not touch the console. A low-priority drain
 *       (log_drain) prints the record later. Use it in every real-time task
 *       and for every ERROR. At most LOG_MAX_ARGS integer arguments.
 *
 *   LOG_NOW
 *       Immediate. The call prints at once with the same prefix. Use it only
 *       where the caller already paces its own output (the flight recorder
 *       dump, the bench builds) or prints many values (the physics CSV rows).
 *
 * Rules:
 *   - Do not call tm_printf directly. `check_firmware_policy.py logging`
 *     fails the build checks if you do.
 *   - Do not log from an interrupt handler. Count in the handler and log the
 *     count from a task.
 *   - Arguments are int sized. Cast to (int). A %s argument must be a string
 *     literal or another string that lives for ever. Messages must stay under
 *     LOG_LINE_MAX characters after formatting.
 *   - The format string is a literal. The ring stores its address.
 *   - Name every number in the message: "turn left done in 910 ms".
 *
 * This header needs no kernel include.
 */

#ifndef PLATFORM_LOG_H
#define PLATFORM_LOG_H

#include <stdint.h>

#include "platform/static_assert.h"

/** Severity. Order matters: lower is more important. */
typedef enum
{
    LOG_SEV_ERROR = 0, /* A feature failed or the car stopped.            */
    LOG_SEV_WARN,      /* Degraded, or a bad input was rejected.          */
    LOG_SEV_INFO,      /* A state change or a finished action.            */
    LOG_SEV_DEBUG,     /* Per-pass detail for a bring-up session.         */
    LOG_SEV_COUNT
} log_sev_t;

STATIC_ASSERT (sizeof(log_sev_t) <= 4U, "log_sev_t size");

/** The kernel task that runs the code. */
typedef enum
{
    LOG_TASK_BOOT = 0,
    LOG_TASK_MOTION,
    LOG_TASK_SUPERVISOR,
    LOG_TASK_LINE,
    LOG_TASK_OBSTACLE,
    LOG_TASK_PHYSICS,
    LOG_TASK_BARCODE,
    LOG_TASK_TELEMETRY,
    LOG_TASK_CONNECTION,
    LOG_TASK_LOG,
    LOG_TASK_COUNT
} log_task_t;

STATIC_ASSERT (sizeof(log_task_t) <= 4U, "log_task_t size");

/** Optional owner of the code when it differs from the task. */
typedef enum
{
    LOG_MOD_NONE = 0,
    LOG_MOD_NAV,
    LOG_MOD_SCAN,
    LOG_MOD_TCP,
    LOG_MOD_MQTT,
    LOG_MOD_IMU,
    LOG_MOD_HUMP,
    LOG_MOD_PID,
    LOG_MOD_COUNT
} log_mod_t;

STATIC_ASSERT (sizeof(log_mod_t) <= 4U, "log_mod_t size");

/** Integer arguments one deferred record can hold. */
#define LOG_MAX_ARGS (8U)

/** Longest printed line, prefix and newline included. */
#define LOG_LINE_MAX (160U)

/**
 * Store one record for later printing. Safe from any task on either core.
 * Not safe from an interrupt handler.
 *
 * @param[in] sev   Severity.
 * @param[in] task  Kernel task that runs the caller.
 * @param[in] mod   Optional module tag, LOG_MOD_NONE if none.
 * @param[in] p_fmt Format literal. Conversions: d i u x X c s (no floats).
 *
 * The ring is full: ERROR and WARN evict the oldest record, the others are
 * dropped and counted.
 */
void log_write (log_sev_t sev, log_task_t task, log_mod_t mod,
                char const * p_fmt, ...);

#define LOG_ERROR(task, mod, ...) \
    log_write (LOG_SEV_ERROR, (task), (mod), __VA_ARGS__)
#define LOG_WARN(task, mod, ...) \
    log_write (LOG_SEV_WARN, (task), (mod), __VA_ARGS__)
#define LOG_INFO(task, mod, ...) \
    log_write (LOG_SEV_INFO, (task), (mod), __VA_ARGS__)
#define LOG_DEBUG(task, mod, ...) \
    log_write (LOG_SEV_DEBUG, (task), (mod), __VA_ARGS__)

/** Set up the ring and its lock. Call once, first, from usermain. */
void log_init (void);

/**
 * Print stored records while a serial monitor is open and its buffer has
 * room. Call from one low-priority task every pass.
 */
void log_drain (void);

/**
 * Print everything that fits now, waiting up to about 200 ms for the console
 * buffer. For boot and failure paths where no drain task may ever run.
 */
void log_flush (void);

/** Records lost because the ring was full. */
uint32_t log_dropped (void);

/**
 * Immediate print with the standard prefix. `sev` and `tag` are string
 * literals; the format is the first variadic argument and must be a literal.
 * The caller includes <tm/tmonitor.h>.
 *
 *   LOG_NOW ("DATA", "physics:imu", "B4,%u,%d\n", t_us, value);
 */
#define LOG_NOW(sev, tag, ...) \
    tm_printf ((UB const *)"[" sev "](" tag ") " __VA_ARGS__)

/**
 * Continue a line that LOG_NOW started: same output path, no prefix. The
 * format is the first variadic argument and must be a literal. Use it only
 * for output built in several pieces, such as a table row.
 */
#define LOG_CONT(...) tm_printf ((UB const *)"" __VA_ARGS__)

#endif /* PLATFORM_LOG_H */

/*** end of file ***/
