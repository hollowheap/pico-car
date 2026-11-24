#include <assert.h>
#include <string.h>

#include "bench/trace.h"

typedef struct
{
    char text[1024];
    size_t length;
    unsigned calls;
} sink_t;

static bool write_bytes(void *ctx, char const *data, size_t length)
{
    sink_t *sink = ctx;
    assert(length < sizeof(sink->text) - sink->length);
    memcpy(sink->text + sink->length, data, length);
    sink->length += length;
    sink->text[sink->length] = '\0';
    ++sink->calls;
    return true;
}

static void test_capture_overflow(void)
{
    bench_trace_t trace;
    bench_trace_record_t records[2];
    sink_t sink = {0};
    bench_trace_header_t const header = {.build = "fixture", .fixture = "servo", .trial = "center-return"};
    bench_trace_record_t const first = {.time_us = 100U, .kind = BENCH_TRACE_SERVO, .pwm_us = 1494U};
    bench_trace_record_t const second = {.time_us = 200U, .kind = BENCH_TRACE_SERVO, .pwm_us = 1382U};

    assert(bench_trace_start(&trace, &header, records, 2U));
    assert(bench_trace_record(&trace, &first));
    assert(bench_trace_record(&trace, &second));
    assert(!bench_trace_record(&trace, &second));
    assert(!bench_trace_export(&trace, write_bytes, &sink));
    assert(sink.calls == 0U && "the export sink must not run during capture");

    bench_trace_stop(&trace);
    assert(!bench_trace_record(&trace, &first));
    assert(bench_trace_export(&trace, write_bytes, &sink));
    assert(sink.calls == 4U);
    assert(strstr(sink.text, "\"trial\":\"center-return\"") != NULL);
    char const *first_line = strstr(sink.text, "\"timeUs\":100");
    char const *second_line = strstr(sink.text, "\"timeUs\":200");
    assert(first_line != NULL && second_line != NULL && first_line < second_line);
    assert(strstr(sink.text, "\"dropped\":1") != NULL);
}

static void test_export_trace_rows(void)
{
    bench_trace_t trace;
    bench_trace_record_t records[3];
    sink_t sink = {0};
    bench_trace_header_t const header = {.build = "abc123+dirty", .fixture = "wire", .trial = "markers"};
    bench_trace_record_t const scheduler = {
        .time_us = 0U, .kind = BENCH_TRACE_SCHEDULER, .marker = BENCH_TRACE_RELEASE
    };
    bench_trace_record_t const trigger = {.p_cue = "left", .time_us = 5U, .kind = BENCH_TRACE_TRIGGER,
                                          .requested_deci = -220, .effective_deci = -200};
    bench_trace_record_t const reply = {.p_cue = "left", .time_us = UINT32_MAX, .kind = BENCH_TRACE_REPLY,
                                        .requested_deci = INT16_MIN, .effective_deci = -200,
                                        .echo = BENCH_TRACE_ECHO_STUCK};
    char const expected[] =
        "{\"kind\":\"header\",\"version\":1,\"build\":\"abc123+dirty\",\"fixture\":\"wire\",\"trial\":\"markers\"}\n"
        "{\"kind\":\"scheduler\",\"seq\":0,\"timeUs\":0,\"marker\":\"release\"}\n"
        "{\"kind\":\"trigger\",\"seq\":1,\"timeUs\":5,\"cue\":\"left\",\"requestedDeci\":-220,\"effectiveDeci\":-200}\n"
        "{\"kind\":\"reply\",\"seq\":2,\"timeUs\":4294967295,\"cue\":\"left\","
        "\"requestedDeci\":-32768,\"effectiveDeci\":-200,"
        "\"echo\":\"stuck\",\"rangeMm\":0}\n"
        "{\"kind\":\"end\",\"dropped\":0}\n";

    assert(bench_trace_start(&trace, &header, records, 3U));
    assert(bench_trace_record(&trace, &scheduler));
    assert(bench_trace_record(&trace, &trigger));
    assert(bench_trace_record(&trace, &reply));
    bench_trace_stop(&trace);
    assert(bench_trace_export(&trace, write_bytes, &sink));
    assert(strcmp(sink.text, expected) == 0);
}

int main(void)
{
    test_capture_overflow();
    test_export_trace_rows();
    return 0;
}
