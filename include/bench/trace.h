#ifndef BENCH_TRACE_H
#define BENCH_TRACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform/static_assert.h"

typedef enum
{
    BENCH_TRACE_SERVO = 0,
    BENCH_TRACE_TRIGGER,
    BENCH_TRACE_REPLY,
    BENCH_TRACE_SCHEDULER,
    BENCH_TRACE_KIND_COUNT
} bench_trace_kind_t;

typedef enum
{
    BENCH_TRACE_RELEASE = 0,
    BENCH_TRACE_CAPTURE,
    BENCH_TRACE_COMPLETE,
    BENCH_TRACE_CONTENTION,
    BENCH_TRACE_MARKER_COUNT
} bench_trace_marker_t;

typedef enum
{
    BENCH_TRACE_ECHO_VALID = 0,
    BENCH_TRACE_ECHO_OPEN,
    BENCH_TRACE_ECHO_TOO_CLOSE,
    BENCH_TRACE_ECHO_NO_ECHO,
    BENCH_TRACE_ECHO_GLITCH,
    BENCH_TRACE_ECHO_STUCK,
    BENCH_TRACE_ECHO_COUNT
} bench_trace_echo_t;

typedef struct
{
    char const * build;
    char const * fixture;
    char const * trial;
} bench_trace_header_t;

typedef struct
{
    char const * p_cue;
    uint32_t     time_us;
    int16_t      requested_deci;
    int16_t      effective_deci;
    uint16_t     pwm_us;
    uint16_t     range_mm;
    uint8_t      kind;
    uint8_t      marker;
    uint8_t      echo;
} bench_trace_record_t;

STATIC_ASSERT(sizeof(bench_trace_record_t) == sizeof(void *) + 16U,
              "bench trace row size");

typedef struct
{
    bench_trace_header_t   header;
    bench_trace_record_t * p_records;
    size_t                 capacity;
    size_t                 count;
    uint32_t               dropped;
    bool                   b_stopped;
} bench_trace_t;

typedef bool (*bench_trace_sink_t)(void * p_ctx, char const * p_data,
                                   size_t length);

/** Storage and identifier strings must remain valid through export. IDs use
 * at most 32 ASCII letters, digits, dots, underscores, plus signs or hyphens.
 */
bool bench_trace_start (bench_trace_t *              p_trace,
                        bench_trace_header_t const * p_header,
                        bench_trace_record_t * p_records, size_t capacity);
/** Append one row from the trial's single writer. Use elapsed device
 * microseconds. */
bool bench_trace_record (bench_trace_t *              p_trace,
                         bench_trace_record_t const * p_record);
void bench_trace_stop (bench_trace_t * p_trace);
/** Send complete NUL-terminated JSONL lines. The length excludes the NUL. */
bool bench_trace_export (bench_trace_t const * p_trace, bench_trace_sink_t sink,
                         void * p_ctx);

#endif /* BENCH_TRACE_H */
