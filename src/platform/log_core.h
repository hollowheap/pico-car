/** @file log_core.h
 * @brief Pure parts of the log module: the record ring, the prefix and the
 *        argument counter. No kernel, no console. Host-tested (D100).
 *
 * The ring has no lock. The caller (log.c) holds one around every call.
 */

#ifndef PLATFORM_LOG_CORE_H
#define PLATFORM_LOG_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/log.h"
#include "platform/static_assert.h"

/** Records in the ring. Must be a power of two. */
#define LOG_RING_LEN (64U)

STATIC_ASSERT (0U == (LOG_RING_LEN & (LOG_RING_LEN - 1U)),
               "LOG_RING_LEN must be a power of two");

typedef struct
{
    uint32_t     t_ms;
    char const * p_fmt;
    int32_t      arg[LOG_MAX_ARGS];
    uint8_t      sev;
    uint8_t      task;
    uint8_t      mod;
    uint8_t      nargs;
} log_record_t;

/* The target has 32-bit pointers: 4 + 4 + 32 + 4. */
#if UINTPTR_MAX == UINT32_MAX
STATIC_ASSERT (sizeof(log_record_t) == 44U, "log_record_t size");
#endif

typedef struct
{
    log_record_t rec[LOG_RING_LEN];
    uint32_t     dropped;
    uint16_t     head;  /* Next write index.        */
    uint16_t     count; /* Records held.            */
} log_ring_t;

/** Empty the ring and clear the drop count. */
void log_ring_init (log_ring_t * p_ring);

/**
 * Add a record.
 *
 * Room left: stored. Ring full: an ERROR or WARN evicts the oldest record and
 * is stored; any other severity is dropped. Both count as dropped.
 *
 * @return True if the record was stored.
 */
bool log_ring_push (log_ring_t * p_ring, log_record_t const * p_rec);

/** Remove the oldest record. Returns false when the ring is empty. */
bool log_ring_pop (log_ring_t * p_ring, log_record_t * p_rec);

/** Records held. */
uint16_t log_ring_count (log_ring_t const * p_ring);

/** Records lost since init. */
uint32_t log_ring_dropped (log_ring_t const * p_ring);

/**
 * Count the arguments a format consumes: one per conversion, "%%" excluded,
 * capped at LOG_MAX_ARGS. A NULL format has none.
 */
uint8_t log_count_args (char const * p_fmt);

/* No size_t here: the kernel headers define their own, and this header is
 * included together with them. */

/**
 * Write "[SEV](task) " or "[SEV](task:module) " into a buffer.
 *
 * @return Characters written, not counting the terminator. The buffer is
 *         always terminated when size is at least 1.
 */
uint32_t log_prefix (char * p_buf, uint32_t size, log_record_t const * p_rec);

/** True when two records are the same message with the same arguments. */
bool log_same (log_record_t const * p_a, log_record_t const * p_b);

#endif /* PLATFORM_LOG_CORE_H */

/*** end of file ***/
