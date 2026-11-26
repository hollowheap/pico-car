/** @file barcode_task.c
 *
 * @brief Decode Code 39 elements from the ISR edge ring, and report each
 *        barcode.
 *
 * A letter accepted by the distance gate becomes a `barcode` event. H, S,
 * L, R and U also request a supervisor command. A rejected read emits
 * `barcode_bad` with its reason.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "ir/barcode/action.h"
#include "ir/barcode/parser.h"
#include "motion/motion.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/static_assert.h"
#include "platform/units.h"
#include "supervisor/supervisor.h"
#include "comms/comms.h"
#include "ir/barcode/barcode.h"
#include "ir/line/track.h"

#include "ir/line/ir_pins.h"

#define US_PER_MS            1000U
#define BARCODE_GAP_US       1000000U
#define BARCODE_GAP_MS       (BARCODE_GAP_US / US_PER_MS)

/* The task waits in liveness slices and kicks after each one, so an idle gap
   is this many timed-out waits in a row. */
#define BARCODE_GAP_WAITS    (BARCODE_GAP_MS / LIVENESS_WAIT_MS)
STATIC_ASSERT((BARCODE_GAP_WAITS * LIVENESS_WAIT_MS) == BARCODE_GAP_MS, barcode_gap_is_whole_waits);

/* Event p2 holds the learned narrow width in 4 µm units, reverse and action. */
#define EVENT_WIDTH_SHIFT   (4U)
#define EVENT_WIDTH_DIV     (4U)
#define EVENT_WIDTH_MAX     (UINT16_MAX >> EVENT_WIDTH_SHIFT)
#define EVENT_REVERSE_SHIFT (3U)

#define STATUS_ELEMENT_MASK (3U << TELEMETRY_LINE_BITS_BIT_BARCODE_ELEMENT_LSB)
#define STATUS_PHASE_MASK   (7U << TELEMETRY_LINE_BITS_BIT_BARCODE_PHASE_LSB)
#define PHASE_REJECT_SHIFT  (8U)

static volatile uint16_t g_ok;
static volatile uint16_t g_bad;
static volatile uint8_t  g_progress_count;
static volatile uint32_t g_start_prefix_um;
static volatile uint8_t  g_last_letter;
static volatile uint8_t  g_last_action;
static volatile uint8_t  g_status_bits;
static volatile int32_t  g_tape_heading_deci = BARCODE_TAPE_HEADING_UNKNOWN;

void barcode_set_tape_heading (int32_t heading_deci)
{
    g_tape_heading_deci = heading_deci;
}
static barcode_gate_t    g_gate;
static barcode_block_t   g_block = BARCODE_BLOCK_INIT;

void barcode_stats (barcode_stats_t * p_stats)
{
    if (p_stats != NULL)
    {
        p_stats->ok          = g_ok;
        p_stats->bad         = g_bad;
        p_stats->last_letter = g_last_letter;
        p_stats->last_action = g_last_action;
    }
}

uint8_t barcode_progress_count (uint8_t * p_prefix_mm)
{
    uint32_t const mm = g_start_prefix_um / UM_PER_MM;
    *p_prefix_mm = (uint8_t)((mm > UINT8_MAX) ? UINT8_MAX : mm);
    return g_progress_count;
}

uint8_t barcode_status_bits (void)
{
    return g_status_bits;
}

typedef struct
{
    barcode_parser_t before;
    uint32_t         interval_us;
    uint32_t         edge_us;
    uint32_t         odo_mm;
    uint32_t         guard_us;
    uint32_t         drops;
    uint8_t          progress_count;
    uint16_t         width_um;
    bool             b_pending;
    bool             is_space;
    bool             b_fed;
    bool             b_glitch;
} barcode_merge_t;

static bool phase_reported (uint8_t phase)
{
    return ((uint32_t)g_status_bits & STATUS_PHASE_MASK) ==
           ((uint32_t)phase << TELEMETRY_LINE_BITS_BIT_BARCODE_PHASE_LSB);
}

bool barcode_read_framed (void)
{
    uint32_t const phase = ((uint32_t)g_status_bits & STATUS_PHASE_MASK) >>
                           TELEMETRY_LINE_BITS_BIT_BARCODE_PHASE_LSB;
    return (phase == TELEMETRY_BARCODE_PHASE_START) || (phase == TELEMETRY_BARCODE_PHASE_DECODING);
}

static void record_element (uint8_t element)
{
    uint32_t const bits = ((uint32_t)g_status_bits & ~STATUS_ELEMENT_MASK) |
                          ((uint32_t)element << TELEMETRY_LINE_BITS_BIT_BARCODE_ELEMENT_LSB);
    g_status_bits = (uint8_t)bits;
}

static void report_phase (uint8_t phase, bool b_reverse, uint8_t reason, uint32_t odo_mm)
{
    uint32_t const bits = ((uint32_t)g_status_bits & ~STATUS_PHASE_MASK) |
                          ((uint32_t)phase << TELEMETRY_LINE_BITS_BIT_BARCODE_PHASE_LSB);
    uint16_t const details = (uint16_t)(((uint32_t)reason << PHASE_REJECT_SHIFT) |
                                      (b_reverse ? 1U : 0U));

    g_status_bits = (uint8_t)bits;
    (void)telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_BARCODE, TELEMETRY_EVT_BARCODE_PHASE,
                                   (int16_t)phase, details, odo_mm);
}

static void report_capture_phase (uint8_t phase, uint32_t odo_mm)
{
    uint16_t const details = (BARCODE_REJECT_CAPTURE << PHASE_REJECT_SHIFT) | BARCODE_PHASE_UNFRAMED;

    (void)telemetry_try_emit_event(TELEMETRY_EVENT_SOURCE_BARCODE, TELEMETRY_EVT_BARCODE_PHASE,
                                   (int16_t)phase, details, odo_mm);
}

static void report_abort (barcode_parser_t const * p_parser, uint8_t reason, uint32_t odo_mm)
{
    g_start_prefix_um = 0U;
    if (p_parser->char_index != 0U)
    {
        report_phase(TELEMETRY_BARCODE_PHASE_ABORTED, p_parser->is_reverse, reason, odo_mm);
    }
}

static void report_bad (uint8_t reason, uint16_t element_width_um, uint32_t odo_mm)
{
    g_bad++;
    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                                   (uint8_t)TELEMETRY_EVT_BARCODE_BAD,
                                   (int16_t)reason, element_width_um,
                                   odo_mm);
    LOG_WARN(LOG_TASK_BARCODE, LOG_MOD_NONE, "barcode refused, reason %d", (int)reason);
}

static void reject_partial (barcode_parser_t * p_parser, uint8_t reason, uint32_t odo_mm)
{
    if (0U != (p_parser->elem_index | p_parser->char_index))
    {
        report_abort(p_parser, reason, odo_mm);
        report_bad(reason, 0U, odo_mm);
    }
    *p_parser = BARCODE_PARSER_INIT;
}

/* Submit any mapped command before non-blocking letter telemetry. */
static void report_decoded (uint8_t code, const barcode_info_t * p_info, uint32_t odo)
{
    uint8_t opcode = 0U;
    barcode_action_t const act = telemetry_barcode_binding(code, &opcode)
                                     ? barcode_action_for_command(opcode) : barcode_action_of(code);
    uint8_t const          letter = (uint8_t)(code + (uint8_t)'A');
    uint16_t const         width  = (uint16_t)(p_info->narrow_width_um / EVENT_WIDTH_DIV);
    uint32_t const         bits   = ((uint32_t)((width > EVENT_WIDTH_MAX) ? EVENT_WIDTH_MAX : width) << EVENT_WIDTH_SHIFT) |
                                    ((p_info->is_reverse ? 1UL : 0UL) << EVENT_REVERSE_SHIFT) |
                                    (uint32_t)act.action;
    uint16_t const         detail = (uint16_t)bits;

    if (!barcode_gate_pass(&g_gate, odo))
    {
        report_bad(BARCODE_REJECT_DUPLICATE, 0U, odo);
        return;
    }

    g_ok++;
    g_last_letter = letter;
    g_last_action = act.action;
    if (BARCODE_ACTION_REJECTED != act.action)
    {
        (void)vehicle_command_post(act.opcode, VEHICLE_SRC_BARCODE);
    }
    (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                                   (uint8_t)TELEMETRY_EVT_BARCODE,
                                   (int16_t)letter, detail, odo);
    LOG_INFO(LOG_TASK_BARCODE, LOG_MOD_NONE, "read %c, action %d%s", (int)letter,
             (int)act.action, p_info->is_reverse ? ", reversed" : "");
}

/* Only displacement sampled near each edge can measure the intervening block. */
static bool report_block (bool level, uint32_t odo_mm, bool b_span_interrupted,
                          const uint16_t * p_width_um)
{
    bool const is_black = (0U != BARCODE_LEVEL_SPACE) ? !level : level;
    bool b_measured = false;

    /* A block can outlive the parser's idle gap but still needs continuous travel. */
    if (b_span_interrupted)
    {
        g_block.b_black = 0U;
    }
    uint16_t const block_mm = barcode_block_edge(&g_block, is_black, odo_mm, p_width_um, &b_measured);

    if (b_measured)
    {
        record_element(TELEMETRY_BARCODE_ELEMENT_BLOCK);
    }

    if (0U != block_mm)
    {
        (void)telemetry_try_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                                       (uint8_t)TELEMETRY_EVT_BARCODE_BLOCK,
                                       (int16_t)block_mm, 0U, odo_mm);
        LOG_INFO(LOG_TASK_BARCODE, LOG_MOD_NONE, "block of %d mm on the track", (int)block_mm);
    }
    return b_measured;
}

static bool feed_element (barcode_parser_t * p_parser, bool is_space,
                          uint16_t width_um, uint32_t odo_mm, bool b_provisional)
{
    barcode_parser_t const before = *p_parser;
    uint8_t const char_index = before.char_index;
    uint8_t const elem_index = before.elem_index;
    barcode_info_t info;
    uint8_t const code = barcode_parser_input_info(p_parser, is_space, width_um, &info);

    /* A closing bar can still be split. */
    if (b_provisional && (BARCODE_NO_CHAR != code))
    {
        *p_parser = before;
        return true;
    }

    /* Credit only the measured portion of this tentative start symbol.
     * Rejected candidates lose the allowance on the same element. */
    if ((p_parser->char_index == 0U) && (p_parser->elem_index != 0U))
    {
        g_start_prefix_um = ((p_parser->elem_index == 1U) ? 0U : g_start_prefix_um) + width_um;
    }
    else
    {
        g_start_prefix_um = 0U;
    }
    if ((BARCODE_REJECT_NONE == info.reject) && ((p_parser->char_index != 0U) || (BARCODE_NO_CHAR != code)))
    {
        /* A verified start delimiter gives subsequent accepted elements meaning. */
        ++g_progress_count;
    }

    if (BARCODE_REJECT_NONE != info.reject)
    {
        if (char_index != 0U)
        {
            report_phase(TELEMETRY_BARCODE_PHASE_ABORTED, info.is_reverse, info.reject, odo_mm);
        }
        report_bad(info.reject, width_um, odo_mm);
    }
    else if (BARCODE_NO_CHAR != code)
    {
        report_phase(TELEMETRY_BARCODE_PHASE_COMPLETE, info.is_reverse, BARCODE_REJECT_NONE, odo_mm);
        report_decoded(code, &info, odo_mm);
    }
    else if ((char_index == 0U) && (p_parser->char_index == 1U))
    {
        report_phase(TELEMETRY_BARCODE_PHASE_START, info.is_reverse, BARCODE_REJECT_NONE, odo_mm);
    }
    /* A rolled-back element that began this character has already reported decoding. */
    else if ((char_index == 1U) && (elem_index == 0U) && (p_parser->elem_index == 1U) &&
             !phase_reported(TELEMETRY_BARCODE_PHASE_DECODING))
    {
        report_phase(TELEMETRY_BARCODE_PHASE_DECODING, info.is_reverse, BARCODE_REJECT_NONE, odo_mm);
    }
    else
    {
        /* No parser phase change. */
    }
    return false;
}

static bool is_in_progress (barcode_parser_t const * p_parser)
{
    return (p_parser->char_index != 0U) || (p_parser->elem_index != 0U);
}

static uint16_t sum_width (uint16_t a_um, uint16_t b_um)
{
    uint32_t const sum = (uint32_t)a_um + (uint32_t)b_um;
    return (uint16_t)((sum > UINT16_MAX) ? UINT16_MAX : sum);
}

static uint32_t sum_intervals (uint32_t a_us, uint32_t b_us)
{
    uint32_t const sum = a_us + b_us;
    return (sum > BARCODE_GAP_US) ? BARCODE_GAP_US : sum;
}

/* Convert the same distance threshold used by the merge into a wait on the
 * measured pace of this stripe. The existing parser idle limit bounds it. */
static uint32_t glitch_guard_us (const barcode_parser_t * p_parser, const barcode_merge_t * p_merge)
{
    uint64_t const scaled = (uint64_t)barcode_parser_glitch_limit_um(p_parser) * p_merge->interval_us;
    uint64_t const duration = (scaled + p_merge->width_um - 1U) / p_merge->width_um;
    return (duration > BARCODE_GAP_US) ? BARCODE_GAP_US : (uint32_t)duration;
}

static void finish_pending (barcode_parser_t * p_parser, barcode_merge_t * p_merge)
{
    if (p_merge->b_pending)
    {
        if (p_merge->drops != ir_barcode_drops())
        {
            reject_partial(p_parser, BARCODE_REJECT_CAPTURE, p_merge->odo_mm);
        }
        else
        {
            (void)feed_element(p_parser, p_merge->is_space, p_merge->width_um, p_merge->odo_mm, false);
        }
        p_merge->b_pending = false;
    }
}

/* Rolling back restores the learnt widths and progress count that the split part contributed. */
static void read_stripe (barcode_parser_t * p_parser, barcode_merge_t * p_merge, bool is_space,
                         const telemetry_barcode_sample_t * p_sample, uint32_t odo_mm, uint32_t drops)
{
    bool const b_in_progress = is_in_progress(p_parser);
    if (0U == p_sample->width_valid)
    {
        /* Invalid or reversing wheel evidence cannot measure this stripe. */
        report_abort(p_parser, BARCODE_REJECT_SPEED, odo_mm);
        report_bad(BARCODE_REJECT_SPEED, 0U, odo_mm);
        *p_parser = BARCODE_PARSER_INIT;
        *p_merge = (barcode_merge_t){0};
        return;
    }
    uint16_t width_um = p_sample->width_um;
    uint32_t interval_us = p_sample->interval_us;

    if (0U != width_um)
    {
        record_element(is_space ? TELEMETRY_BARCODE_ELEMENT_SPACE : TELEMETRY_BARCODE_ELEMENT_BAR);
    }
    if (p_merge->b_glitch && b_in_progress)
    {
        width_um = sum_width(p_merge->width_um, width_um);
        interval_us = sum_intervals(p_merge->interval_us, interval_us);
        is_space = p_merge->is_space;
        p_merge->b_glitch = false;
    }
    else if ((p_merge->b_fed || p_merge->b_pending) && b_in_progress &&
             barcode_parser_is_glitch(&p_merge->before, width_um))
    {
        *p_parser = p_merge->before;
        g_progress_count = p_merge->progress_count;
        p_merge->width_um = sum_width(p_merge->width_um, width_um);
        p_merge->interval_us = sum_intervals(p_merge->interval_us, interval_us);
        p_merge->b_glitch = true;
        p_merge->b_fed = false;
        p_merge->b_pending = false;
        return;
    }
    else
    {
        finish_pending(p_parser, p_merge);
        p_merge->before = *p_parser;
        p_merge->progress_count = g_progress_count;
        p_merge->b_glitch = false;
    }
    p_merge->b_pending = feed_element(p_parser, is_space, width_um, odo_mm, true);
    p_merge->width_um = width_um;
    p_merge->is_space = is_space;
    p_merge->interval_us = interval_us;
    p_merge->b_fed = !p_merge->b_pending && (p_merge->before.char_index != 0U) &&
                     is_in_progress(p_parser);
    if (p_merge->b_pending)
    {
        p_merge->edge_us = p_sample->edge_us;
        p_merge->odo_mm = odo_mm;
        p_merge->drops = drops;
        p_merge->guard_us = glitch_guard_us(p_parser, p_merge);
    }
}

/* Wait one liveness slice for an edge. A provisional stripe can wake sooner. */
static bool wait_for_gap (uint32_t * p_idle_waits, TMO timeout)
{
    bool b_gap = false;

    if (E_TMOUT != ir_wait_change(IR_TYPE_BARCODE, timeout))
    {
        *p_idle_waits = 0U;
    }
    else if ((TMO)LIVENESS_WAIT_MS == timeout)
    {
        (*p_idle_waits)++;
        if (*p_idle_waits >= BARCODE_GAP_WAITS)
        {
            *p_idle_waits = 0U;
            b_gap = true;
        }
    }

    return b_gap;
}

static TMO pending_timeout (barcode_parser_t * p_parser, barcode_merge_t * p_merge, uint32_t drained_us)
{
    if (p_merge->b_pending)
    {
        int32_t const checked_us = (int32_t)(drained_us - p_merge->edge_us);
        uint32_t const elapsed_us = (checked_us > 0) ? (uint32_t)checked_us : 0U;
        if ((elapsed_us > p_merge->guard_us) || (p_merge->drops != ir_barcode_drops()))
        {
            finish_pending(p_parser, p_merge);
        }
        else
        {
            uint32_t const ms = (p_merge->guard_us - elapsed_us + US_PER_MS) / US_PER_MS;
            return (TMO)((ms > LIVENESS_WAIT_MS) ? LIVENESS_WAIT_MS : ms);
        }
    }
    return (TMO)LIVENESS_WAIT_MS;
}

/* An empty ring need not mean capture has qualified every transition yet. */
static ir_barcode_read_t pop_stripe (uint32_t * p_timestamp_us, bool * p_level, uint32_t * p_drained_us)
{
    *p_drained_us = ir_barcode_drain_limit_us();
    return ir_barcode_pop(p_timestamp_us, p_level);
}

/* Keep the odometer for event context even when it cannot measure this edge. */
static bool sample_odometer (uint32_t timestamp_us, motion_feedback_t * p_feedback, uint32_t * p_odo_mm)
{
    if (E_OK != motion_feedback_snapshot(p_feedback))
    {
        p_feedback->fresh = false;
        *p_odo_mm = 0U;
        return false;
    }
    *p_odo_mm = telemetry_odometer_mm(p_feedback);

    return p_feedback->fresh && p_feedback->odometry_valid[0] && p_feedback->odometry_valid[1] &&
           (((timestamp_us - p_feedback->timestamp_us) <= BARCODE_SPEED_SLACK_US) ||
            ((p_feedback->timestamp_us - timestamp_us) <= BARCODE_SPEED_SLACK_US));
}

/* Settle a pending split bar, then end the read on a capture loss or a
 * timing gap. A loss before any character is reported as an unframed start. */
static void handle_break (barcode_parser_t * p_parser, barcode_merge_t * p_merge, bool b_lost,
                          bool b_parser_gap, uint32_t gap_us, uint32_t odo_mm,
                          bool * p_unframed_loss, bool * p_have_previous)
{
    if (!b_lost && p_merge->b_pending && (gap_us > p_merge->guard_us))
    {
        finish_pending(p_parser, p_merge);
    }
    if (b_parser_gap && *p_unframed_loss)
    {
        report_capture_phase(TELEMETRY_BARCODE_PHASE_ABORTED, odo_mm);
        *p_unframed_loss = false;
    }
    if (b_lost && (0U == p_parser->char_index) && !*p_unframed_loss)
    {
        report_capture_phase(TELEMETRY_BARCODE_PHASE_START, odo_mm);
        *p_unframed_loss = true;
    }
    if (b_lost || b_parser_gap)
    {
        reject_partial(p_parser, b_lost ? BARCODE_REJECT_CAPTURE : BARCODE_REJECT_DELIMITER, odo_mm);
        *p_merge = (barcode_merge_t){0};
        *p_have_previous = false;
    }
}

void barcode_task_run (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    motion_feedback_t previous_feedback = {0};
    uint32_t         previous_us = 0U;
    uint32_t         edge_odo_mm = 0U;
    bool             previous_level = false;
    bool             have_previous = false;
    bool             b_unframed_loss = false;
    int32_t          tape_heading_deci = BARCODE_TAPE_HEADING_UNKNOWN;
    uint32_t         idle_waits = 0U;
    uint8_t          sample_seq = 0U;
    barcode_merge_t  merge = {0};

    g_start_prefix_um = 0U;
    barcode_gate_init(&g_gate);

    for (;;)
    {
        uint32_t timestamp_us = 0U;
        bool     level        = false;
        uint32_t drained_us = 0U;

        liveness_kick(LIVE_BARCODE);
        for (ir_barcode_read_t edge = pop_stripe(&timestamp_us, &level, &drained_us);
             IR_BARCODE_EMPTY != edge; edge = pop_stripe(&timestamp_us, &level, &drained_us))
        {
            uint32_t const drops = ir_barcode_drops();
            uint32_t const gap_us = timestamp_us - previous_us;
            bool const b_lost = (IR_BARCODE_EDGE_AFTER_LOSS == edge) ||
                                (have_previous && (level == previous_level)) ||
                                (merge.b_pending && (merge.drops != drops));
            motion_feedback_t feedback = {0};
            bool const b_have_odometer = sample_odometer(timestamp_us, &feedback, &edge_odo_mm);
            bool const b_block_wheel_gap = (0U != g_block.b_black) &&
                ((previous_feedback.invalid_samples[0] != feedback.invalid_samples[0]) ||
                 (previous_feedback.invalid_samples[1] != feedback.invalid_samples[1]) ||
                 !barcode_odometry_covers_start(previous_us, &feedback));
            uint32_t const sampled_odo_mm = b_have_odometer ? edge_odo_mm : UINT32_MAX;
            bool const b_parser_gap = have_previous && (gap_us > BARCODE_GAP_US);
            bool const b_parser_span = have_previous && !b_lost && !b_parser_gap;
            bool const b_block_span = (0U != g_block.b_black) && !b_block_wheel_gap && b_have_odometer &&
                                      (level != previous_level) && !b_lost;
            uint16_t width_um = 0U;
            bool const b_width_valid = (b_parser_span || b_block_span) && barcode_width_between_edges(
                previous_us, timestamp_us, &previous_feedback, &feedback, tape_heading_deci, &width_um);
            bool const b_measured_block = report_block(level, sampled_odo_mm,
                b_lost || b_block_wheel_gap, b_width_valid ? &width_um : NULL);

            handle_break(&parser, &merge, b_lost, b_parser_gap, gap_us, edge_odo_mm,
                         &b_unframed_loss, &have_previous);

            telemetry_barcode_sample_t sample = {
                .edge_us = timestamp_us,
                .interval_us = (b_parser_span || b_block_span) ? gap_us : 0U,
                .odometer_mm = sampled_odo_mm,
                .width_um = b_width_valid ? width_um : 0U,
                .seq = sample_seq++,
                .level_high = level,
                .width_valid = b_width_valid,
                .capture_loss = b_lost,
            };
            if (b_measured_block)
            {
                /* Encoder travel across this black stripe already rules out a bar. */
                report_abort(&parser, BARCODE_REJECT_NONE, edge_odo_mm);
                parser = BARCODE_PARSER_INIT;
                merge = (barcode_merge_t){0};
            }
            else if (have_previous)
            {
                bool const is_space = ((0U != BARCODE_LEVEL_SPACE) == previous_level);
                read_stripe(&parser, &merge, is_space, &sample, edge_odo_mm, drops);
            }
            /* The edge remains a boundary for the next stripe even when
             * speed was unusable over the interval that just ended. */
            if (!have_previous)
            {
                /* Keep the approach tangent while the reader crosses the marker. */
                tape_heading_deci = g_tape_heading_deci;
            }
            have_previous = true;
            previous_us    = timestamp_us;
            previous_level = level;
            previous_feedback = feedback;
            (void)telemetry_try_emit_barcode_sample(&sample);
        }

        if (wait_for_gap(&idle_waits, pending_timeout(&parser, &merge, drained_us)))
        {
            if (b_unframed_loss)
            {
                report_capture_phase(TELEMETRY_BARCODE_PHASE_ABORTED, edge_odo_mm);
                b_unframed_loss = false;
            }
            reject_partial(&parser, merge.b_pending ? BARCODE_REJECT_CAPTURE : BARCODE_REJECT_DELIMITER,
                           edge_odo_mm);
            merge = (barcode_merge_t){0};
            have_previous = false;
        }
    }
}

/*** end of file ***/
