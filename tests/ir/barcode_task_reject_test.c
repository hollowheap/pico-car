/** Replay timestamped IR edges through the production barcode task and parser. */
#include <assert.h>
#include <setjmp.h>
#include <string.h>

#include "ir/barcode/action.h"
#include "ir/barcode/parser.h"
#include "platform/drv_timer.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "supervisor/command.h"
#include "comms/telemetry/assemble.h"
#include "comms/telemetry/emit.h"
#include "comms/telemetry/packet.h"
#include "comms/telemetry/wire_schema.h"
#include "ir/barcode/barcode.h"
#include "ir/line/track.h"

/* The 163 mm/s extras cruise request is halved at motion admission. */
#define EXTRAS_HALF_CRUISE_MM_S (82U)

static jmp_buf g_finish;
static uint8_t const g_delimiter[] = {1U, 3U, 1U, 1U, 3U, 1U, 3U, 1U, 1U, 1U};
static uint8_t const g_reverse_delimiter[] = {1U, 1U, 3U, 1U, 3U, 1U, 1U, 3U, 1U, 1U};
static uint8_t const g_parity[] = {3U, 1U, 3U, 1U, 3U, 1U, 1U, 1U, 1U};
static uint8_t const g_element[] = {1U, 1U, 3U, 1U, 3U, 1U, 1U, 3U, 2U};
static uint8_t const g_a[] = {3U, 1U, 1U, 1U, 1U, 3U, 1U, 1U, 3U, 1U};
static uint8_t const g_h[] = {3U, 1U, 1U, 1U, 1U, 3U, 3U, 1U, 1U, 1U};
static uint8_t const g_l[] = {1U, 1U, 3U, 1U, 1U, 1U, 1U, 3U, 3U, 1U};
static uint8_t const g_reverse_a[] = {3U, 1U, 1U, 3U, 1U, 1U, 1U, 1U, 3U, 1U};
static uint8_t const g_blocks[] = {60U, 1U, 61U};
static uint8_t const g_abort_block[] = {60U};
static uint8_t const * gp_delimiter;
static uint8_t const * gp_widths;
static size_t g_count;
static size_t g_edge;
static uint32_t g_timestamp_us;
static uint32_t g_clock_us;
static uint32_t g_capture_drops;
static uint32_t g_command_delay_us;
static bool g_b_drop_during_wait;
static bool g_b_drop_during_terminal_sample;
static bool g_b_edge_after_empty_pop;
static bool g_b_capture_qualification_lag;
static bool g_b_wake_for_edge;
static bool g_b_follow_ready;
static uint8_t g_last_bar_width_mm;
static uint32_t g_feedback_us;
static uint32_t g_odometer_mm;
static size_t g_skew_open_edge = SIZE_MAX;
static int32_t g_open_skew_us;
static int32_t g_close_skew_us;
static uint32_t g_sample_odometer_mm;
static uint32_t g_speed_mm_s;
static uint32_t g_end_speed_mm_s;
static int64_t g_travel_um;
static uint32_t g_start_us;
static uint32_t g_start_mm;
static uint32_t g_tick_remainder;
static bool g_b_drop_phase;
static bool g_b_stall_letter_report;
static unsigned g_stalled_emit_count;
static uint8_t g_expected_opcode = VEHICLE_CMD_LEFT;
static unsigned g_command_count;
static unsigned g_idle_waits;
static size_t g_invalid_speed_stripe = SIZE_MAX;
static bool g_b_late_invalid_opening;
static size_t g_recovered_odometry_edge = SIZE_MAX;
static size_t g_odometry_rebase_edge = SIZE_MAX;
static bool g_b_delayed_pair;
static bool g_b_recent_recovery;
static size_t g_loss_edge = SIZE_MAX;
static bool g_follow_with_clean_marker;
static size_t g_odometer_jump_edge = SIZE_MAX;
static size_t g_invalid_odometer_edge = SIZE_MAX;
static uint8_t g_status_at_edge[31];
static uint8_t g_prefix_mm[31];
static uint8_t g_progress[31];

typedef struct
{
    uint8_t type;
    int16_t p1;
    uint16_t p2;
    size_t stripe;
    uint32_t timestamp_us;
    uint32_t odometer_mm;
} event_t;

static event_t g_events[8];
static size_t g_event_count;
static unsigned g_capture_starts;
static unsigned g_capture_ends;
static telemetry_barcode_sample_t g_samples[64];
static unsigned g_sample_count;
static bool g_b_override_z;

bool telemetry_barcode_binding (uint8_t letter, uint8_t * p_opcode)
{
    if (!g_b_override_z || letter != ('Z' - 'A')) return false;
    *p_opcode = VEHICLE_CMD_RIGHT;
    return true;
}

ER telemetry_try_emit_barcode_sample(telemetry_barcode_sample_t const * sample)
{
    assert(g_sample_count < sizeof(g_samples) / sizeof(g_samples[0]));
    g_samples[g_sample_count++] = *sample;
    return E_TMOUT; /* A full telemetry queue cannot interrupt decoding. */
}

static void travel_um (uint32_t distance_um)
{
    /* The capture ring timestamps have 2 µs resolution. Carry the fractional
     * tick across elements so a constant physical speed gives the correct total. */
    uint32_t const ticks = distance_um * 500U + g_tick_remainder;
    g_timestamp_us += 2U * (ticks / g_speed_mm_s);
    g_tick_remainder = ticks % g_speed_mm_s;
    g_travel_um += (int64_t)distance_um;
}

static void travel_mm (uint32_t distance)
{
    travel_um(distance * 1000U);
    g_odometer_mm += distance;
}

/* A glitch of the other level splits one element in its middle: a paper
 * fibre or a ragged tape edge. */
#define GLITCH_UM (200U)
static size_t g_glitch_stripe = SIZE_MAX;
static unsigned g_glitch_step;
static uint32_t g_glitch_pulse_us;

static void observe_edge_time (void)
{
    if ((int32_t)(g_timestamp_us - g_clock_us) > 0)
    {
        g_clock_us = g_timestamp_us;
    }
}

ir_barcode_read_t ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level)
{
    size_t const delimiter_count = gp_delimiter == NULL ? 0U : sizeof(g_delimiter);

    if (g_edge != 0U)
    {
        assert(g_edge <= sizeof(g_status_at_edge));
        g_status_at_edge[g_edge - 1U] = barcode_status_bits();
        g_progress[g_edge - 1U] = barcode_progress_count(&g_prefix_mm[g_edge - 1U]);
    }
    if (g_edge > delimiter_count + g_count)
    {
        if (!g_follow_with_clean_marker || !g_b_follow_ready)
        {
            return IR_BARCODE_EMPTY;
        }
        /* The next opening bar arrives after a 51 mm white gap. */
        g_follow_with_clean_marker = false;
        g_b_follow_ready = false;
        travel_mm(51U);
        g_loss_edge = SIZE_MAX;
        g_edge = 0U;
    }
    if (g_edge != 0U)
    {
        size_t const stripe = g_edge - 1U;
        size_t const data_index = stripe - delimiter_count;
        uint8_t width;

        if (stripe < delimiter_count)
        {
            width = gp_delimiter[stripe];
        }
        else if (gp_delimiter != NULL && data_index >= sizeof(g_a))
        {
            width = gp_delimiter[data_index - sizeof(g_a)];
        }
        else
        {
            width = gp_widths[data_index];
        }
        if ((0U != g_last_bar_width_mm) && ((stripe + 1U) == delimiter_count + g_count))
        {
            width = g_last_bar_width_mm;
        }
        if (stripe == g_glitch_stripe)
        {
            uint32_t const glitch_um = (0U == g_glitch_pulse_us) ? GLITCH_UM : 0U;
            uint32_t const half_um = ((uint32_t)width * 1000U - glitch_um) / 2U;

            if ((1U == g_glitch_step) && g_b_edge_after_empty_pop)
            {
                /* The consumer saw an empty ring, then a pulse queued before
                 * the consumer resumed past the provisional deadline. */
                g_b_edge_after_empty_pop = false;
                g_b_wake_for_edge = true;
                if (!g_b_capture_qualification_lag)
                {
                    g_clock_us += 6000U;
                }
                return IR_BARCODE_EMPTY;
            }
            if (g_glitch_step < 2U)
            {
                if (1U == g_glitch_step)
                {
                    g_timestamp_us += g_glitch_pulse_us;
                }
                travel_um((0U == g_glitch_step) ? half_um : glitch_um);
                g_glitch_step++;
                observe_edge_time();
                if (g_b_capture_qualification_lag && (1U == g_glitch_step))
                {
                    /* The task drains the first fragment after its optical
                     * guard, while the returning edge is still withheld. */
                    g_clock_us += 6000U;
                }
                *p_timestamp_us = g_timestamp_us;
                *p_level = (1U == g_glitch_step) ? ((g_edge % 2U) == 0U) : ((g_edge % 2U) != 0U);
                return IR_BARCODE_EDGE;
            }
            travel_um((uint32_t)width * 1000U - glitch_um - half_um);
            g_odometer_mm += width;
            g_glitch_step = 0U;
        }
        else
        {
            travel_mm(width);
        }
        if (g_edge == g_odometer_jump_edge)
        {
            g_odometer_mm += 60U;
        }
    }
    observe_edge_time();
    *p_timestamp_us = g_timestamp_us;
    /* The first rising edge enters the first black bar from white floor. */
    *p_level = (g_edge % 2U) == 0U;
    g_edge++;
    return (g_edge - 1U == g_loss_edge) ? IR_BARCODE_EDGE_AFTER_LOSS : IR_BARCODE_EDGE;
}

uint32_t shim_reg_read (uint32_t address)
{
    assert(TIMER_TIMERAWL == address);
    return g_clock_us;
}

uint32_t ir_barcode_drops (void)
{
    return g_capture_drops;
}

uint32_t ir_barcode_drain_limit_us (void)
{
    return g_clock_us - (g_b_capture_qualification_lag ? 5300U : 0U);
}

ER ir_wait_change (uint8_t type, TMO timeout)
{
    assert(type == IR_TYPE_BARCODE && timeout > 0);
    if (g_b_wake_for_edge)
    {
        g_b_wake_for_edge = false;
        return E_OK;
    }
    if (timeout < (TMO)LIVENESS_WAIT_MS)
    {
        g_clock_us += (uint32_t)timeout * 1000U;
        if (g_b_drop_during_wait)
        {
            g_capture_drops++;
            g_b_drop_during_wait = false;
        }
        return E_TMOUT;
    }
    if (g_follow_with_clean_marker)
    {
        g_b_follow_ready = true;
        return E_OK;
    }
    if (g_idle_waits > 0U)
    {
        g_idle_waits--;
        return E_TMOUT;
    }
    longjmp(g_finish, 1);
}

int motion_feedback_snapshot (motion_feedback_t * p_feedback)
{
    if (g_b_drop_during_terminal_sample && (30U == g_edge))
    {
        g_capture_drops++;
        g_b_drop_during_terminal_sample = false;
    }
    bool const b_late_invalid = g_b_late_invalid_opening && (1U == g_edge);
    bool const b_speed_valid = !b_late_invalid && (g_edge - 1U != g_invalid_speed_stripe);
    bool const b_odometer_valid = b_speed_valid && (g_edge - 1U != g_invalid_odometer_edge);
    uint32_t const speed = (g_edge > 1U && g_end_speed_mm_s != 0U) ? g_end_speed_mm_s : g_speed_mm_s;
    uint8_t const invalid = (g_b_late_invalid_opening || (g_edge - 1U >= g_invalid_speed_stripe) ||
                            g_b_recent_recovery || (g_edge - 1U >= g_recovered_odometry_edge)) ? 1U : 0U;

    /* An invalid control interval can end after its optical edge. A short
     * pulse can instead have both edges in the same valid control interval. */
    g_feedback_us = g_timestamp_us + (b_late_invalid ? 6000U : 0U)
                    - ((2U == g_glitch_step) ? g_glitch_pulse_us : 0U);
    int64_t feedback_travel = g_travel_um;
    if (g_b_delayed_pair && g_edge <= 2U)
    {
        /* Both optical edges pop after the same motion update. */
        g_feedback_us = g_start_us + 12000U;
        feedback_travel += (int64_t)(g_feedback_us - g_timestamp_us) * (int64_t)speed / 1000;
    }
    if (g_skew_open_edge != SIZE_MAX)
    {
        int32_t const offset_us = (g_edge == g_skew_open_edge) ? g_open_skew_us :
            (g_edge == g_skew_open_edge + 1U) ? g_close_skew_us : 0;

        g_feedback_us += (uint32_t)offset_us;
        feedback_travel += (int64_t)offset_us * (int64_t)speed / 1000;
        g_sample_odometer_mm = (uint32_t)(feedback_travel / 1000);
    }
    uint32_t span_us = (uint32_t)INT32_MAX;
    if (g_b_recent_recovery)
    {
        /* The invalid interval ended 2 ms after the opening optical edge. */
        span_us = g_feedback_us - (g_start_us + 2000U);
    }
    else if (g_edge - 1U == g_odometry_rebase_edge)
    {
        span_us = 10000U;
    }
    *p_feedback = (motion_feedback_t){
        .fresh = true, .speed_valid = {b_speed_valid, b_speed_valid}, .timestamp_us = g_feedback_us,
        .invalid_samples = {invalid, invalid}, .odometry_span_us = span_us,
        .odometry_valid = {b_odometer_valid, b_odometer_valid},
        .left_speed_um_s = (int32_t)(speed * 1000U),
        .right_speed_um_s = (int32_t)(speed * 1000U),
        .left_displacement_um = feedback_travel,
        .right_displacement_um = feedback_travel,
    };
    return E_OK;
}

uint32_t telemetry_odometer_mm (const motion_feedback_t * p_feedback)
{
    assert(p_feedback->timestamp_us == g_feedback_us);
    return (g_skew_open_edge != SIZE_MAX) ? g_sample_odometer_mm : g_odometer_mm;
}

static void record_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
                          uint32_t odometer_mm)
{
    assert(source == TELEMETRY_EVENT_SOURCE_BARCODE);
    if ((type == TELEMETRY_EVT_BARCODE_PHASE) && ((p2 & BARCODE_PHASE_UNFRAMED) != 0U))
    {
        if (p1 == TELEMETRY_BARCODE_PHASE_START)
        {
            g_capture_starts++;
        }
        else
        {
            assert(p1 == TELEMETRY_BARCODE_PHASE_ABORTED);
            g_capture_ends++;
        }
        return;
    }
    uint32_t const latest_odo_mm = telemetry_odometer_mm(&(motion_feedback_t){.timestamp_us = g_feedback_us});
    assert(odometer_mm == latest_odo_mm ||
           (g_sample_count > 0U && odometer_mm == g_samples[g_sample_count - 1U].odometer_mm));
    assert(g_event_count < sizeof(g_events) / sizeof(g_events[0]));
    g_events[g_event_count++] = (event_t){type, p1, p2, g_edge - 1U, g_timestamp_us, odometer_mm};
}

ER telemetry_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2,
                         uint32_t odometer_mm)
{
    (void)source;
    (void)type;
    (void)p1;
    (void)p2;
    (void)odometer_mm;
    assert(false && "barcode_task must not call blocking telemetry_emit_event");
    return E_TMOUT;
}

ER telemetry_try_emit_event (uint8_t source, uint8_t type, int16_t p1,
                             uint16_t p2, uint32_t odometer_mm)
{
    assert((type == TELEMETRY_EVT_BARCODE_PHASE) ||
           (type == TELEMETRY_EVT_BARCODE) ||
           (type == TELEMETRY_EVT_BARCODE_BAD) ||
           (type == TELEMETRY_EVT_BARCODE_BLOCK));
    record_event(source, type, p1, p2, odometer_mm);
    bool const b_stalled = (type != TELEMETRY_EVT_BARCODE_PHASE) &&
                           g_b_stall_letter_report;
    if (b_stalled)
    {
        g_stalled_emit_count++;
    }
    bool const b_blocked = ((type == TELEMETRY_EVT_BARCODE_PHASE) &&
                            g_b_drop_phase) || b_stalled;
    return b_blocked ? E_TMOUT : E_OK;
}

ER vehicle_command_post (uint8_t opcode, uint8_t source)
{
    assert(opcode == g_expected_opcode && source == VEHICLE_SRC_BARCODE);
    assert((g_event_count == 0U) ||
           (g_events[g_event_count - 1U].type != TELEMETRY_EVT_BARCODE));
    g_command_count++;
    g_command_delay_us = g_clock_us - g_timestamp_us;
    return E_OK;
}

void liveness_kick (uint8_t task) { (void)task; }

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod, const char * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

static void replay (uint8_t const * p_delimiter, uint8_t const * p_widths, size_t count)
{
    gp_delimiter   = p_delimiter;
    gp_widths      = p_widths;
    g_count        = count;
    g_edge         = 0U;
    g_timestamp_us += 5000000U;
    g_odometer_mm += 500U;
    g_travel_um += 500000;
    g_start_us = g_timestamp_us;
    g_clock_us = g_timestamp_us;
    g_start_mm = g_odometer_mm;
    g_tick_remainder = 0U;
    g_event_count        = 0U;
    g_capture_starts     = 0U;
    g_capture_ends       = 0U;
    g_sample_count       = 0U;
    g_stalled_emit_count = 0U;
    if (setjmp(g_finish) == 0)
    {
        barcode_task_run();
    }
}

static void expect_event (size_t index, uint8_t type, int16_t p1, uint16_t p2, size_t stripe)
{
    assert(index < g_event_count);
    assert(g_events[index].type == type);
    assert(g_events[index].p1 == p1);
    assert(g_events[index].p2 == p2);
    assert(g_events[index].stripe == stripe);
}

static void expect_phase (size_t index, int16_t phase, uint16_t details, size_t stripe)
{
    expect_event(index, TELEMETRY_EVT_BARCODE_PHASE, phase, details, stripe);
}

static void expect_status (uint8_t bits, uint8_t element, uint8_t phase)
{
    assert((bits & 7U) == 0U);
    assert(((bits >> 3U) & 3U) == element);
    assert((bits >> 5U) == phase);
}

static void test_active_reject (uint8_t const * p_delimiter, uint8_t const * p_widths, size_t count,
                                uint8_t reason, uint16_t width_um, uint16_t reverse)
{
    replay(p_delimiter, p_widths, count);
    assert(g_event_count == 4U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, reverse, 10U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_DECODING, reverse, 11U);
    expect_phase(2U, TELEMETRY_BARCODE_PHASE_ABORTED, (uint16_t)((reason << 8U) | reverse), 10U + count);
    expect_event(3U, TELEMETRY_EVT_BARCODE_BAD, reason, width_um, 10U + count);
}

static void test_complete (uint8_t const * p_delimiter, uint8_t const * p_data, uint16_t reverse)
{
    replay(p_delimiter, p_data, 19U);
    assert(g_event_count == 4U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, reverse, 10U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_DECODING, reverse, 11U);
    expect_phase(2U, TELEMETRY_BARCODE_PHASE_COMPLETE, reverse, 29U);
    expect_event(3U, TELEMETRY_EVT_BARCODE, 'A',
                 (uint16_t)((250U << 4U) | ((uint32_t)reverse << 3U) | BARCODE_ACTION_REJECTED), 29U);
    expect_status(g_status_at_edge[10], TELEMETRY_BARCODE_ELEMENT_SPACE, TELEMETRY_BARCODE_PHASE_START);
    expect_status(g_status_at_edge[11], TELEMETRY_BARCODE_ELEMENT_BAR, TELEMETRY_BARCODE_PHASE_DECODING);
    expect_status(barcode_status_bits(), TELEMETRY_BARCODE_ELEMENT_BAR, TELEMETRY_BARCODE_PHASE_COMPLETE);
}

static void test_full_sized_marker (uint32_t speed_mm_s, bool reverse)
{
    uint8_t delimiter[10];
    uint8_t data[10];
    /* Reverse the nine elements in each symbol. The separator stays last. */
    for (unsigned i = 0; i < 10U; ++i)
    {
        unsigned const source = (reverse && i < 9U) ? 8U - i : i;
        delimiter[i] = (uint8_t)(3U * g_delimiter[source]);
        data[i] = (uint8_t)(3U * g_l[source]);
    }
    unsigned const commands = g_command_count;
    g_speed_mm_s = speed_mm_s;
    g_follow_with_clean_marker = true;
    replay(delimiter, data, 19U);
    assert(g_command_count == commands + 2U && g_event_count == 8U);
    assert(g_prefix_mm[5] >= 20U && g_prefix_mm[10] == 0U);
    assert(g_progress[5] == g_progress[1] && g_progress[11] != g_progress[10]);
    for (size_t marker = 0; marker < 2U; ++marker)
    {
        size_t const event = marker * 4U;
        uint16_t const orientation = reverse ? 1U : 0U;
        expect_phase(event, TELEMETRY_BARCODE_PHASE_START, orientation, 10U);
        expect_phase(event + 1U, TELEMETRY_BARCODE_PHASE_DECODING, orientation, 11U);
        expect_phase(event + 2U, TELEMETRY_BARCODE_PHASE_COMPLETE, orientation, 29U);
        event_t const * decoded = &g_events[event + 3U];
        assert(decoded->type == TELEMETRY_EVT_BARCODE && decoded->p1 == 'L' && decoded->stripe == 29U);
        assert((decoded->p2 & 15U) == (BARCODE_ACTION_LEFT | (orientation << 3U)));
        unsigned const narrow_um = (decoded->p2 >> 4U) * 4U;
        assert(narrow_um >= 2996U && narrow_um <= 3000U);
        /* A second complete marker follows 51 mm of clear floor in the same run. */
        uint32_t const distance_mm = (marker == 0U) ? 141U : 333U;
        uint32_t const elapsed_us = decoded->timestamp_us - g_start_us;
        uint32_t const expected_us = distance_mm * 1000000U / speed_mm_s;
        assert(decoded->odometer_mm == g_start_mm + distance_mm);
        assert(elapsed_us <= expected_us && elapsed_us + 2U > expected_us);
    }
}

static void test_full_sized_markers (void)
{
    test_full_sized_marker(EXTRAS_HALF_CRUISE_MM_S, false);
    test_full_sized_marker(EXTRAS_HALF_CRUISE_MM_S, true);
    test_full_sized_marker(11U, true);
    test_full_sized_marker(120U, false);
    test_full_sized_marker(180U, false);
    test_full_sized_marker(120U, true);
    test_full_sized_marker(180U, true);
}

static void test_halt_with_stalled_report (void)
{
    uint8_t const  saved_opcode   = g_expected_opcode;
    uint32_t const saved_speed    = g_speed_mm_s;
    unsigned const saved_commands = g_command_count;

    g_expected_opcode = VEHICLE_CMD_STOP;
    g_speed_mm_s      = EXTRAS_HALF_CRUISE_MM_S;
    g_command_count   = 0U;

    replay(g_delimiter, g_h, 19U);
    assert(g_command_count == 1U);
    assert((g_event_count == 4U) && (g_stalled_emit_count == 0U));
    assert((g_events[3U].type == TELEMETRY_EVT_BARCODE) &&
           (g_events[3U].p1 == 'H') &&
           ((g_events[3U].p2 & 15U) == BARCODE_ACTION_STOP) &&
           (g_events[3U].stripe == 29U));

    g_command_count            = 0U;
    g_b_stall_letter_report    = true;
    g_follow_with_clean_marker = true;
    replay(g_delimiter, g_h, 19U);

    barcode_stats_t stats;
    barcode_stats(&stats);
    assert((stats.last_letter == 'H') &&
           (stats.last_action == BARCODE_ACTION_STOP));
    assert(g_command_count == 1U);
    assert((g_event_count == 8U) && (g_stalled_emit_count == 2U));
    expect_phase(6U, TELEMETRY_BARCODE_PHASE_COMPLETE, 0U, 29U);
    expect_event(7U, TELEMETRY_EVT_BARCODE_BAD,
                 BARCODE_REJECT_DUPLICATE, 0U, 29U);

    g_b_stall_letter_report    = false;
    g_follow_with_clean_marker = false;
    g_expected_opcode          = saved_opcode;
    g_speed_mm_s               = saved_speed;
    g_command_count            = saved_commands;
}

static void test_element_travel (void)
{
    static uint8_t const bar[] = {3U};

    /* Wheel telemetry on the flat run varied as the follower steered.
     * The closing speed cannot stand in for the 60 mm/s interval average. */
    g_speed_mm_s = 60U;
    g_end_speed_mm_s = 31U;
    replay(NULL, bar, sizeof(bar));
    assert(g_sample_count == 2U);
    assert(g_samples[1].interval_us == 50000U);
    assert(g_samples[1].width_valid && g_samples[1].width_um == 3000U);
}

static void test_late_invalid_opening (void)
{
    g_speed_mm_s = 25U;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_b_late_invalid_opening = true;
    replay(g_delimiter, g_h, 19U);

    assert(g_command_count == 0U);
    assert(g_sample_count == 30U);
    assert(!g_samples[1].width_valid);
    expect_event(0U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_SPEED, 0U, 1U);
}

static void test_submicrometre_glitch (void)
{
    g_speed_mm_s = 25U;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_glitch_stripe = 10U;
    g_glitch_pulse_us = 30U;
    replay(g_delimiter, g_h, 19U);

    assert(g_command_count == 1U);
    assert(g_sample_count == 32U);
    assert(g_samples[12].interval_us == g_glitch_pulse_us);
    assert(g_samples[12].width_valid && g_samples[12].width_um == 0U);
}

static void test_invalid_block_span (bool b_rebased)
{
    g_speed_mm_s = 200U;
    if (b_rebased)
    {
        g_odometry_rebase_edge = 11U;
    }
    else
    {
        g_recovered_odometry_edge = 11U;
    }
    replay(g_delimiter, g_abort_block, sizeof(g_abort_block));

    assert(g_sample_count == 12U && !g_samples[11].width_valid);
    assert(g_event_count == 3U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, 0U, 10U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_ABORTED, BARCODE_REJECT_SPEED << 8U, 11U);
    expect_event(2U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_SPEED, 0U, 11U);
}

static void test_delayed_pair (bool b_recovered)
{
    g_speed_mm_s = 120U;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_b_delayed_pair = true;
    g_b_recent_recovery = b_recovered;
    replay(g_delimiter, g_h, 19U);

    assert(g_command_count == (b_recovered ? 0U : 1U));
    assert(g_sample_count == 30U && g_samples[1].interval_us == 8332U);
    assert(g_samples[1].width_valid == !b_recovered);
    if (!b_recovered)
    {
        assert(g_samples[1].width_um == 999U);
    }
}

static void test_aligned_stop_bar (void)
{
    static uint8_t const delimiter[] = {12U, 42U, 12U, 12U, 42U, 12U, 42U, 12U, 12U, 12U};
    static uint8_t const halt[] = {48U, 12U, 12U, 12U, 12U, 42U, 42U, 12U, 12U, 12U};

    g_speed_mm_s = 180U;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_skew_open_edge = 11U;
    g_open_skew_us = -9000;
    g_close_skew_us = 14334;
    replay(delimiter, halt, 19U);

    assert(g_samples[11].odometer_mm - g_samples[10].odometer_mm == 52U);
    assert(g_samples[11].width_valid && g_samples[11].width_um == 47999U);
    assert(g_command_count == 1U);
    assert(g_event_count == 4U && g_events[3].type == TELEMETRY_EVT_BARCODE && g_events[3].p1 == 'H');
}

static void test_aligned_block (void)
{
    static uint8_t const block[] = {52U};

    g_speed_mm_s = 180U;
    g_skew_open_edge = 1U;
    g_open_skew_us = 9000;
    g_close_skew_us = -9888;
    replay(NULL, block, sizeof(block));

    assert(g_samples[1].odometer_mm - g_samples[0].odometer_mm == 49U);
    assert(g_samples[1].width_valid && g_samples[1].width_um == 52000U);
    assert(g_event_count == 1U);
    expect_event(0U, TELEMETRY_EVT_BARCODE_BLOCK, 52, 0U, 1U);
}

static void test_split_bar (bool b_reverse, bool b_glitch)
{
    uint8_t delimiter[10];
    uint8_t halt[10];
    for (unsigned i = 0U; i < 10U; ++i)
    {
        unsigned const source = (b_reverse && i < 9U) ? (8U - i) : i;
        delimiter[i] = (uint8_t)(3U * g_delimiter[source]);
        halt[i] = (uint8_t)(3U * g_h[source]);
    }
    size_t const stripe = b_reverse ? 10U : 12U;
    unsigned const commands = g_command_count;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_speed_mm_s = 180U;
    g_glitch_stripe = b_glitch ? stripe : SIZE_MAX;
    replay(delimiter, halt, 19U);

    assert(g_command_count == commands + 1U && g_event_count == 4U);
    assert(g_events[3].type == TELEMETRY_EVT_BARCODE && g_events[3].p1 == 'H');
    assert(g_command_delay_us >= 5000U && g_command_delay_us <= 6000U);
    assert(g_sample_count == (b_glitch ? 32U : 30U));
    if (b_glitch)
    {
        assert(g_samples[stripe + 2U].interval_us == 1112U ||
               g_samples[stripe + 2U].interval_us == 1110U);
        assert(g_samples[stripe + 1U].width_valid && g_samples[stripe + 1U].width_um == 1400U);
        assert(g_samples[stripe + 2U].width_valid && g_samples[stripe + 2U].width_um == 200U);
        assert(g_samples[stripe + 3U].width_valid && g_samples[stripe + 3U].width_um == 1400U);
    }
}

static void test_invalid_final_bar (bool b_glitch)
{
    static uint8_t const delimiter[] = {3U, 9U, 3U, 3U, 9U, 3U, 9U, 3U, 3U, 3U};
    static uint8_t const left[] = {3U, 3U, 9U, 3U, 3U, 3U, 3U, 9U, 9U, 3U};
    g_speed_mm_s = 180U;
    g_last_bar_width_mm = 6U;
    g_glitch_stripe = b_glitch ? 28U : SIZE_MAX;
    replay(delimiter, left, 19U);

    assert(g_command_count == 0U && g_event_count == 4U);
    expect_event(3U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_PARITY, 6000U, 29U);
}

static void test_capture_loss_during_close (bool b_during_sample)
{
    uint32_t const drops = g_capture_drops;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_speed_mm_s = 180U;
    g_b_drop_during_wait = !b_during_sample;
    g_b_drop_during_terminal_sample = b_during_sample;
    replay(g_delimiter, g_h, 19U);

    assert(!g_b_drop_during_wait && !g_b_drop_during_terminal_sample);
    assert(g_capture_drops == drops + 1U);
    assert(g_command_count == 0U && g_event_count == 4U);
    expect_event(3U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_CAPTURE, 0U, 29U);
}

static void test_close_before_later_invalid_width (void)
{
    g_speed_mm_s = 180U;
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_invalid_speed_stripe = 30U;
    replay(g_delimiter, g_h, 20U);

    assert(g_sample_count == 31U);
    assert(g_samples[29].width_valid && !g_samples[30].width_valid);
    assert(g_command_count == 1U && g_event_count == 5U);
    assert(g_events[3].type == TELEMETRY_EVT_BARCODE && g_events[3].p1 == 'H');
    assert(g_events[3].odometer_mm == g_samples[29].odometer_mm);
    expect_event(4U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_SPEED, 0U, 30U);
}

static void test_genuine_short_bar (void)
{
    static uint8_t const delimiter[] = {3U, 9U, 3U, 3U, 9U, 3U, 9U, 3U, 3U, 3U};
    static uint8_t const halt[] = {9U, 3U, 1U, 3U, 3U, 9U, 9U, 3U, 3U, 3U};
    g_expected_opcode = VEHICLE_CMD_STOP;
    g_speed_mm_s = 180U;
    replay(delimiter, halt, 19U);

    /* The 1 mm bar is above the glitch limit, so it is not merged. Ranked
     * with its own colour it is narrow, as on recorded car reads. */
    assert(g_command_count == 1U && g_event_count == 4U);
    assert(g_events[3].type == TELEMETRY_EVT_BARCODE && g_events[3].p1 == 'H');
    assert(g_sample_count == 30U && g_samples[13].width_um == 1000U);
}

int main (int argc, char ** argv)
{
    if (argc == 2 && strcmp(argv[1], "split-narrow-bar") == 0)
    {
        test_split_bar(false, true);
        test_split_bar(true, true);
        test_split_bar(false, false);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "invalid-final-bar") == 0)
    {
        test_invalid_final_bar(false);
        test_invalid_final_bar(true);
        g_b_edge_after_empty_pop = true;
        test_invalid_final_bar(true);
        assert(!g_b_edge_after_empty_pop && !g_b_wake_for_edge);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "capture-loss-during-close") == 0)
    {
        test_capture_loss_during_close(false);
        test_capture_loss_during_close(true);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "pending-capture-guard") == 0)
    {
        g_b_capture_qualification_lag = true;
        g_b_edge_after_empty_pop = true;
        test_invalid_final_bar(true);
        assert(!g_b_edge_after_empty_pop && !g_b_wake_for_edge);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "close-before-later-invalid-width") == 0)
    {
        test_close_before_later_invalid_width();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "genuine-short-bar") == 0)
    {
        test_genuine_short_bar();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "aligned-stop-bar") == 0)
    {
        test_aligned_stop_bar();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "aligned-block") == 0)
    {
        test_aligned_block();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "late-clean-pair") == 0)
    {
        test_delayed_pair(false);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "late-recovered-pair") == 0)
    {
        test_delayed_pair(true);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "rebased-block-span") == 0)
    {
        test_invalid_block_span(true);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "invalid-block-span") == 0)
    {
        test_invalid_block_span(false);
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "late-invalid-opening") == 0)
    {
        test_late_invalid_opening();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "submicrometre-glitch") == 0)
    {
        test_submicrometre_glitch();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "element-travel") == 0)
    {
        test_element_travel();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "full-sized") == 0)
    {
        test_full_sized_markers();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "halt-with-stalled-report") == 0)
    {
        test_halt_with_stalled_report();
        return 0;
    }
    assert(argc == 1);
    expect_status(barcode_status_bits(), TELEMETRY_BARCODE_ELEMENT_UNKNOWN, TELEMETRY_BARCODE_PHASE_UNKNOWN);
    g_speed_mm_s = 100U;
    static uint8_t const invalid_start[] = {1U, 4U};
    replay(NULL, invalid_start, sizeof(invalid_start));
    assert(g_sample_count == 3U);
    assert(g_samples[0].interval_us == 0U && !g_samples[0].width_valid);
    assert(g_samples[1].interval_us == 10000U && !g_samples[1].level_high);
    assert(g_samples[1].width_valid && g_samples[1].width_um == 1000U);
    assert(g_samples[2].edge_us - g_samples[1].edge_us == 40000U);
    assert(g_samples[2].level_high && g_samples[2].seq == (uint8_t)(g_samples[1].seq + 1U));
    /* A character is judged only once its nine elements are in. */
    assert(g_event_count == 0U);
    expect_status(barcode_status_bits(), TELEMETRY_BARCODE_ELEMENT_SPACE, TELEMETRY_BARCODE_PHASE_UNKNOWN);

    test_active_reject(g_delimiter, g_parity, sizeof(g_parity), BARCODE_REJECT_PARITY, 1000U, 0U);
    test_active_reject(g_reverse_delimiter, g_element, sizeof(g_element), BARCODE_REJECT_PARITY, 2000U, 1U);
    test_complete(g_delimiter, g_a, 0U);
    g_b_drop_phase = true;
    test_complete(g_reverse_delimiter, g_reverse_a, 1U);
    g_b_drop_phase = false;

    g_idle_waits = 10U;
    replay(g_reverse_delimiter, NULL, 0U);
    assert(g_event_count == 3U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, 1U, 10U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_ABORTED, (BARCODE_REJECT_DELIMITER << 8U) | 1U, 10U);
    expect_event(2U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_DELIMITER, 0U, 10U);
    expect_status(barcode_status_bits(), TELEMETRY_BARCODE_ELEMENT_SPACE, TELEMETRY_BARCODE_PHASE_ABORTED);

    g_invalid_speed_stripe = 11U;
    replay(g_delimiter, g_a, 1U);
    assert(g_samples[11].interval_us == 30000U && !g_samples[11].width_valid);
    g_invalid_speed_stripe = SIZE_MAX;
    assert(g_event_count == 3U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, 0U, 10U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_ABORTED, BARCODE_REJECT_SPEED << 8U, 11U);
    expect_event(2U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_SPEED, 0U, 11U);
    expect_status(barcode_status_bits(), TELEMETRY_BARCODE_ELEMENT_SPACE, TELEMETRY_BARCODE_PHASE_ABORTED);

    replay(NULL, g_blocks, sizeof(g_blocks));
    assert(g_event_count == 1U);
    expect_event(0U, TELEMETRY_EVT_BARCODE_BLOCK, 60, 0U, 1U);
    /* A rate-limited second block still updates last measured element. */
    expect_status(g_status_at_edge[2], TELEMETRY_BARCODE_ELEMENT_SPACE, TELEMETRY_BARCODE_PHASE_ABORTED);
    expect_status(barcode_status_bits(), TELEMETRY_BARCODE_ELEMENT_BLOCK, TELEMETRY_BARCODE_PHASE_ABORTED);

    /* An in-window measured block cancels before any payload is accepted. */
    g_speed_mm_s = 200U;
    replay(g_delimiter, g_abort_block, sizeof(g_abort_block));
    assert(g_event_count == 3U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, 0U, 10U);
    expect_event(1U, TELEMETRY_EVT_BARCODE_BLOCK, 60, 0U, 11U);
    expect_phase(2U, TELEMETRY_BARCODE_PHASE_ABORTED, 0U, 11U);

    g_speed_mm_s = 100U;
    /* A qualified width governs classification even when raw odometer disagrees. */
    g_odometer_jump_edge = 11U;
    replay(g_delimiter, g_a, 1U);
    g_odometer_jump_edge = SIZE_MAX;
    assert(g_samples[11].width_valid && g_samples[11].width_um == 3000U);
    assert(g_event_count == 2U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_DECODING, 0U, 11U);

    /* Invalid displacement cannot veto an otherwise valid speed-based width. */
    g_odometer_jump_edge = 11U;
    g_invalid_odometer_edge = 11U;
    replay(g_delimiter, g_a, 1U);
    g_invalid_odometer_edge = SIZE_MAX;
    g_odometer_jump_edge = SIZE_MAX;
    assert(g_event_count == 2U);
    expect_phase(1U, TELEMETRY_BARCODE_PHASE_DECODING, 0U, 11U);

    static const uint8_t non_letter[] = {3U, 1U, 1U, 3U, 1U, 1U, 1U, 1U, 3U, 1U};
    test_active_reject(g_delimiter, non_letter, 19U, BARCODE_REJECT_SYMBOL, 1000U, 0U);

    /* A lost pulse in the final delimiter must prevent posting this L. */
    g_loss_edge = 29U;
    replay(g_delimiter, g_l, 19U);
    assert(g_samples[29].capture_loss && !g_samples[29].interval_us && !g_samples[29].width_valid);
    g_loss_edge = SIZE_MAX;
    assert(g_event_count == 4U);
    expect_phase(2U, TELEMETRY_BARCODE_PHASE_ABORTED, BARCODE_REJECT_CAPTURE << 8U, 29U);
    expect_event(3U, TELEMETRY_EVT_BARCODE_BAD, BARCODE_REJECT_CAPTURE, 0U, 29U);
    assert(g_command_count == 0U);
    g_loss_edge = 29U;
    g_follow_with_clean_marker = true;
    replay(g_delimiter, g_l, 19U);
    assert(g_event_count == 8U);
    expect_phase(2U, TELEMETRY_BARCODE_PHASE_ABORTED, BARCODE_REJECT_CAPTURE << 8U, 29U);
    expect_phase(6U, TELEMETRY_BARCODE_PHASE_COMPLETE, 0U, 29U);
    assert(g_events[7U].type == TELEMETRY_EVT_BARCODE && g_events[7U].p1 == 'L');
    assert(g_command_count == 1U);
    /* Drop every raw packet and still report and dispatch the complete Z. */
    static uint8_t const delim_two[] = {1,2,1,1,2,1,2,1,1,1};
    static uint8_t const z_two[] = {1,2,2,1,2,1,1,1,1,1};
    g_expected_opcode = VEHICLE_CMD_STOP;
    replay(delim_two, z_two, 19U);
    assert(g_sample_count == 30U && g_event_count == 4U);
    assert(g_events[3].type == TELEMETRY_EVT_BARCODE && g_events[3].p1 == 'Z');
    barcode_stats_t stats;
    barcode_stats(&stats);
    assert(stats.last_letter == 'Z' && stats.last_action == BARCODE_ACTION_STOP && g_command_count == 2U);
    g_b_override_z = true;
    g_expected_opcode = VEHICLE_CMD_RIGHT;
    replay(delim_two, z_two, 19U);
    barcode_stats(&stats);
    assert(stats.last_letter == 'Z' && stats.last_action == BARCODE_ACTION_RIGHT && g_command_count == 3U);
    g_b_override_z = false;
    g_expected_opcode = VEHICLE_CMD_LEFT;
    g_loss_edge = 0U;
    g_idle_waits = 10U;
    replay(g_delimiter, g_l, 0U);
    assert(g_capture_starts == 1U && g_capture_ends == 1U);
    g_loss_edge = SIZE_MAX;
    test_full_sized_markers();

    /* A glitch inside the first wide bar of the A joins the bar again. The
     * glitch shows only after the bar's first part has been read, so the
     * decoding phase may be reported one stripe early; the symbol is not. */
    g_speed_mm_s = 100U;
    g_glitch_stripe = 10U;
    replay(g_delimiter, g_a, 19U);
    g_glitch_stripe = SIZE_MAX;
    assert(g_event_count == 4U);
    expect_phase(0U, TELEMETRY_BARCODE_PHASE_START, 0U, 10U);
    assert((g_events[1].type == TELEMETRY_EVT_BARCODE_PHASE) &&
           (g_events[1].p1 == TELEMETRY_BARCODE_PHASE_DECODING));
    expect_phase(2U, TELEMETRY_BARCODE_PHASE_COMPLETE, 0U, 29U);
    expect_event(3U, TELEMETRY_EVT_BARCODE, 'A', (uint16_t)((250U << 4U) | BARCODE_ACTION_REJECTED), 29U);
    test_halt_with_stalled_report();
    return 0;
}
