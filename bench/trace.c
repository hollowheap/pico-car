#include <limits.h>

#include "bench/trace.h"

#define TRACE_ID_MAX   (32U)
#define TRACE_LINE_MAX (192U)
#define DECIMAL_BASE   (10U)

typedef struct
{
    char   text[TRACE_LINE_MAX];
    size_t length;
} trace_line_t;

static char const * const g_kinds[BENCH_TRACE_KIND_COUNT] = {
    "servo", "trigger", "reply", "scheduler"};

static char const * const g_markers[BENCH_TRACE_MARKER_COUNT] = {
    "release", "capture", "complete", "contention"};

static char const * const g_echoes[BENCH_TRACE_ECHO_COUNT] = {
    "valid", "open", "too_close", "no_echo", "glitch", "stuck"};

static bool valid_id (char const * p_id)
{
    size_t length = 0U;

    if (p_id == NULL)
    {
        return false;
    }
    for (; (length <= TRACE_ID_MAX) && (p_id[length] != '\0'); length++)
    {
        char const c = p_id[length];

        if (!(((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
              ((c >= '0') && (c <= '9')) || (c == '.') || (c == '_') ||
              (c == '+') || (c == '-')))
        {
            return false;
        }
    }
    return (length > 0U) && (length <= TRACE_ID_MAX);
}

bool bench_trace_start (bench_trace_t *              p_trace,
                        bench_trace_header_t const * p_header,
                        bench_trace_record_t * p_records, size_t capacity)
{
    if ((p_trace == NULL) || (p_header == NULL) || (p_records == NULL) ||
        (capacity == 0U) || !valid_id(p_header->build) ||
        !valid_id(p_header->fixture) || !valid_id(p_header->trial))
    {
        return false;
    }
    *p_trace = (bench_trace_t){
        .header = *p_header, .p_records = p_records, .capacity = capacity};
    return true;
}

bool bench_trace_record (bench_trace_t *              p_trace,
                         bench_trace_record_t const * p_record)
{
    if ((p_trace == NULL) || (p_record == NULL) ||
        (p_trace->p_records == NULL) || p_trace->b_stopped)
    {
        return false;
    }
    if (p_trace->count == p_trace->capacity)
    {
        if (p_trace->dropped != UINT32_MAX)
        {
            p_trace->dropped++;
        }
        return false;
    }
    p_trace->p_records[p_trace->count] = *p_record;
    p_trace->count++;
    return true;
}

void bench_trace_stop (bench_trace_t * p_trace)
{
    if (p_trace != NULL)
    {
        p_trace->b_stopped = true;
    }
}

static void append_text (trace_line_t * p_line, char const * p_text)
{
    for (; (*p_text != '\0') && (p_line->length < sizeof(p_line->text));
         p_text++)
    {
        p_line->text[p_line->length] = *p_text;
        p_line->length++;
    }
}

static void append_unsigned (trace_line_t * p_line, uintmax_t value)
{
    char   digits[sizeof(value) * CHAR_BIT];
    size_t pos = sizeof(digits) - 1U;

    digits[pos] = '\0';
    do
    {
        pos--;
        digits[pos] = (char)('0' + (value % DECIMAL_BASE));
        value /= DECIMAL_BASE;
    } while (value != 0U);

    append_text(p_line, &digits[pos]);
}

static void append_bearing (trace_line_t * p_line, int16_t angle)
{
    if (angle < 0)
    {
        append_text(p_line, "-");
    }
    append_unsigned(p_line, (uintmax_t)(angle < 0 ? -(int32_t)angle : angle));
}

/* Metadata and cues use the safe identifier alphabet checked at the boundary.
 */
static void append_string_field (trace_line_t * p_line, char const * p_key,
                                 char const * p_value)
{
    append_text(p_line, ",\"");
    append_text(p_line, p_key);
    append_text(p_line, "\":\"");
    append_text(p_line, p_value);
    append_text(p_line, "\"");
}

static bool format_row (trace_line_t *               p_line,
                        bench_trace_record_t const * p_row, size_t seq)
{
    if ((p_row->kind >= BENCH_TRACE_KIND_COUNT) ||
        ((p_row->p_cue != NULL) && !valid_id(p_row->p_cue)) ||
        ((p_row->kind == BENCH_TRACE_REPLY) &&
         (p_row->echo >= BENCH_TRACE_ECHO_COUNT)) ||
        ((p_row->kind == BENCH_TRACE_SCHEDULER) &&
         (p_row->marker >= BENCH_TRACE_MARKER_COUNT)))
    {
        return false;
    }
    append_text(p_line, "{\"kind\":\"");
    append_text(p_line, g_kinds[p_row->kind]);
    append_text(p_line, "\",\"seq\":");
    append_unsigned(p_line, seq);
    append_text(p_line, ",\"timeUs\":");
    append_unsigned(p_line, p_row->time_us);
    if (p_row->p_cue != NULL)
    {
        append_string_field(p_line, "cue", p_row->p_cue);
    }
    if (p_row->kind == BENCH_TRACE_SCHEDULER)
    {
        append_string_field(p_line, "marker", g_markers[p_row->marker]);
        return true;
    }
    if (p_row->kind == BENCH_TRACE_SERVO)
    {
        append_text(p_line, ",\"pwmUs\":");
        append_unsigned(p_line, p_row->pwm_us);
    }
    append_text(p_line, ",\"requestedDeci\":");
    append_bearing(p_line, p_row->requested_deci);
    append_text(p_line, ",\"effectiveDeci\":");
    append_bearing(p_line, p_row->effective_deci);
    if (p_row->kind == BENCH_TRACE_REPLY)
    {
        append_string_field(p_line, "echo", g_echoes[p_row->echo]);
        append_text(p_line, ",\"rangeMm\":");
        append_unsigned(p_line, p_row->range_mm);
    }
    return true;
}

static bool write_line (trace_line_t * p_line, bench_trace_sink_t sink,
                        void * p_ctx)
{
    bool b_written = false;

    append_text(p_line, "}\n");
    if (p_line->length < sizeof(p_line->text))
    {
        p_line->text[p_line->length] = '\0';
        b_written = sink(p_ctx, p_line->text, p_line->length);
    }
    p_line->length = 0U;
    return b_written;
}

bool bench_trace_export (bench_trace_t const * p_trace, bench_trace_sink_t sink,
                         void * p_ctx)
{
    trace_line_t line = {.length = 0U};
    size_t       idx  = 0U;

    if ((p_trace == NULL) || (p_trace->p_records == NULL) ||
        !p_trace->b_stopped || (sink == NULL))
    {
        return false;
    }
    append_text(&line, "{\"kind\":\"header\",\"version\":1");
    append_string_field(&line, "build", p_trace->header.build);
    append_string_field(&line, "fixture", p_trace->header.fixture);
    append_string_field(&line, "trial", p_trace->header.trial);
    if (!write_line(&line, sink, p_ctx))
    {
        return false;
    }
    for (; idx < p_trace->count; idx++)
    {
        if (!format_row(&line, &p_trace->p_records[idx], idx) ||
            !write_line(&line, sink, p_ctx))
        {
            return false;
        }
    }
    append_text(&line, "{\"kind\":\"end\",\"dropped\":");
    append_unsigned(&line, p_trace->dropped);
    return write_line(&line, sink, p_ctx);
}
