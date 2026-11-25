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

#include "barcode/action.h"
#include "barcode/parser.h"
#include "motion/motion.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "platform/static_assert.h"
#include "platform/units.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"
#include "track/barcode.h"
#include "track/track.h"

#include "ir_pins.h"

#define US_PER_MS            1000U
#define BARCODE_GAP_US       500000U
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

static void report_abort (barcode_parser_t const * p_parser, uint8_t reason, uint32_t odo_mm)
{
    g_start_prefix_um = 0U;
    if (p_parser->char_index != 0U)
    {
        report_phase(TELEMETRY_BARCODE_PHASE_ABORTED, p_parser->is_reverse, reason, odo_mm);
    }
}

static uint32_t odometer_mm (void)
{
    motion_feedback_t feedback;
    uint32_t          mm = 0U;

    if (E_OK == motion_feedback_snapshot(&feedback))
    {
        mm = telemetry_odometer_mm(&feedback);
    }

    return mm;
}

static void report_bad (uint8_t reason, uint16_t element_width_um)
{
    g_bad++;
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                               (uint8_t)TELEMETRY_EVT_BARCODE_BAD, (int16_t)reason,
                               element_width_um,
                               odometer_mm());
    LOG_WARN(LOG_TASK_BARCODE, LOG_MOD_NONE, "barcode refused, reason %d", (int)reason);
}

static void reject_partial (barcode_parser_t * p_parser, uint8_t reason, uint32_t odo_mm)
{
    if (0U != (p_parser->elem_index | p_parser->char_index))
    {
        report_abort(p_parser, reason, odo_mm);
        report_bad(reason, 0U);
    }
    *p_parser = BARCODE_PARSER_INIT;
}

/* Publish an accepted letter and pass any mapped command to the supervisor. */
static void report_decoded (uint8_t code, const barcode_info_t * p_info)
{
    uint32_t const         odo    = odometer_mm();
    barcode_action_t const act    = barcode_action_of(code);
    uint8_t const          letter = (uint8_t)(code + (uint8_t)'A');
    uint16_t const         width  = (uint16_t)(p_info->narrow_width_um / EVENT_WIDTH_DIV);
    uint32_t const         bits   = ((uint32_t)((width > EVENT_WIDTH_MAX) ? EVENT_WIDTH_MAX : width) << EVENT_WIDTH_SHIFT) |
                                    ((p_info->is_reverse ? 1UL : 0UL) << EVENT_REVERSE_SHIFT) |
                                    (uint32_t)act.action;
    uint16_t const         detail = (uint16_t)bits;

    if (!barcode_gate_pass(&g_gate, odo))
    {
        report_bad(BARCODE_REJECT_DUPLICATE, 0U);
        return;
    }

    g_ok++;
    g_last_letter = letter;
    g_last_action = act.action;
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                               (uint8_t)TELEMETRY_EVT_BARCODE, (int16_t)letter, detail, odo);
    if (BARCODE_ACTION_REJECTED != act.action)
    {
        (void)vehicle_command_post(act.opcode, VEHICLE_SRC_BARCODE);
    }
    LOG_INFO(LOG_TASK_BARCODE, LOG_MOD_NONE, "read %c, action %d%s", (int)letter,
             (int)act.action, p_info->is_reverse ? ", reversed" : "");
}

/* Only displacement sampled near each edge can measure the intervening block. */
static bool report_block (bool level, uint32_t odo_mm, bool b_have_odometer)
{
    bool const        is_black = (0U != BARCODE_LEVEL_SPACE) ? !level : level;
    bool              b_measured = false;
    uint16_t          block_mm = 0U;

    if (b_have_odometer)
    {
        block_mm = barcode_block_edge(&g_block, is_black, odo_mm, &b_measured);
    }
    else
    {
        g_block.b_black = 0U;
    }

    if (b_measured)
    {
        record_element(TELEMETRY_BARCODE_ELEMENT_BLOCK);
    }

    if (0U != block_mm)
    {
        (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                                   (uint8_t)TELEMETRY_EVT_BARCODE_BLOCK, (int16_t)block_mm, 0U,
                                   odo_mm);
        LOG_INFO(LOG_TASK_BARCODE, LOG_MOD_NONE, "block of %d mm on the track", (int)block_mm);
    }
    return b_measured;
}

static void feed_element (barcode_parser_t * p_parser, bool is_space, uint16_t width_um,
                          uint32_t odo_mm)
{
    uint8_t const char_index = p_parser->char_index;
    uint8_t const elem_index = p_parser->elem_index;
    barcode_info_t info;
    uint8_t const code = barcode_parser_input_info(p_parser, is_space, width_um, &info);

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
        report_bad(info.reject, width_um);
    }
    else if (BARCODE_NO_CHAR != code)
    {
        report_phase(TELEMETRY_BARCODE_PHASE_COMPLETE, info.is_reverse, BARCODE_REJECT_NONE, odo_mm);
        report_decoded(code, &info);
    }
    else if ((char_index == 0U) && (p_parser->char_index == 1U))
    {
        report_phase(TELEMETRY_BARCODE_PHASE_START, info.is_reverse, BARCODE_REJECT_NONE, odo_mm);
    }
    else if ((char_index == 1U) && (elem_index == 0U) && (p_parser->elem_index == 1U))
    {
        report_phase(TELEMETRY_BARCODE_PHASE_DECODING, info.is_reverse, BARCODE_REJECT_NONE, odo_mm);
    }
    else
    {
        /* No parser phase change. */
    }
}

static void read_stripe (barcode_parser_t * p_parser, bool is_space, uint16_t width_um,
                         uint32_t odo_mm)
{
    if (barcode_element_is_block(is_space, width_um))
    {
        record_element(TELEMETRY_BARCODE_ELEMENT_BLOCK);
        report_abort(p_parser, BARCODE_REJECT_NONE, odo_mm);
        *p_parser = BARCODE_PARSER_INIT;
    }
    else
    {
        if (width_um != 0U)
        {
            record_element(is_space ? TELEMETRY_BARCODE_ELEMENT_SPACE : TELEMETRY_BARCODE_ELEMENT_BAR);
        }
        feed_element(p_parser, is_space, width_um, odo_mm);
    }
}

/* Wait one liveness slice for an edge. True once a whole gap passed without one. */
static bool wait_for_gap (uint32_t * p_idle_waits)
{
    bool b_gap = false;

    if (E_TMOUT != ir_wait_change(IR_TYPE_BARCODE, (TMO)LIVENESS_WAIT_MS))
    {
        *p_idle_waits = 0U;
    }
    else
    {
        (*p_idle_waits)++;
        if (*p_idle_waits >= BARCODE_GAP_WAITS)
        {
            *p_idle_waits = 0U;
            b_gap         = true;
        }
    }

    return b_gap;
}

void barcode_task_run (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    uint32_t         previous_us = 0U;
    uint32_t         edge_odo_mm = 0U;
    bool             previous_level = false;
    bool             have_previous = false;
    uint32_t         idle_waits = 0U;
    uint8_t          sample_seq = 0U;

    g_start_prefix_um = 0U;
    barcode_gate_init(&g_gate);

    for (;;)
    {
        uint32_t timestamp_us = 0U;
        bool     level        = false;

        liveness_kick(LIVE_BARCODE);
        for (ir_barcode_read_t edge = ir_barcode_pop(&timestamp_us, &level);
             IR_BARCODE_EMPTY != edge; edge = ir_barcode_pop(&timestamp_us, &level))
        {
            uint32_t const gap_us = timestamp_us - previous_us;
            bool const b_lost = (IR_BARCODE_EDGE_AFTER_LOSS == edge) ||
                                (have_previous && (level == previous_level));
            motion_feedback_t feedback = {0};
            bool const b_feedback = (E_OK == motion_feedback_snapshot(&feedback));
            edge_odo_mm = b_feedback ? telemetry_odometer_mm(&feedback) : 0U;
            if (b_lost)
            {
                /* Travel across missing transitions cannot measure a black block. */
                g_block.b_black = 0U;
            }
            if (!b_feedback)
            {
                feedback.fresh = false;
            }
            bool const b_have_odometer = feedback.fresh && feedback.odometry_valid[0] && feedback.odometry_valid[1] &&
                (((timestamp_us - feedback.timestamp_us) <= BARCODE_SPEED_SLACK_US) ||
                 ((feedback.timestamp_us - timestamp_us) <= BARCODE_SPEED_SLACK_US));
            bool const b_measured_block = report_block(level, edge_odo_mm, b_have_odometer);

            if (b_lost || (have_previous && (gap_us > BARCODE_GAP_US)))
            {
                reject_partial(&parser, b_lost ? BARCODE_REJECT_CAPTURE : BARCODE_REJECT_DELIMITER,
                               edge_odo_mm);
                have_previous = false;
            }

            telemetry_barcode_sample_t sample = {
                .edge_us = timestamp_us,
                .interval_us = have_previous ? gap_us : 0U,
                .odometer_mm = b_have_odometer ? edge_odo_mm : UINT32_MAX,
                .seq = sample_seq++,
                .level_high = level,
                .capture_loss = b_lost,
            };
            sample.width_valid = have_previous && barcode_width_um(gap_us, timestamp_us, &feedback, &sample.width_um);
            if (b_measured_block)
            {
                /* Encoder travel across this black stripe already rules out a bar. */
                report_abort(&parser, BARCODE_REJECT_NONE, edge_odo_mm);
                parser = BARCODE_PARSER_INIT;
            }
            else if (have_previous)
            {
                if (0U == sample.width_valid)
                {
                    /* A stale snapshot, zero speed, or reversing speed rejects
                     * the current character instead of decoding a time-based width. */
                    report_abort(&parser, BARCODE_REJECT_SPEED, edge_odo_mm);
                    report_bad(BARCODE_REJECT_SPEED, 0U);
                    parser        = BARCODE_PARSER_INIT;
                }
                else
                {
                    bool const is_space = (0U != BARCODE_LEVEL_SPACE)
                                              ? previous_level
                                              : !previous_level;
                    read_stripe(&parser, is_space, sample.width_um, edge_odo_mm);
                }
            }
            /* The edge remains a boundary for the next stripe even when
             * speed was unusable over the interval that just ended. */
            have_previous = true;
            previous_us    = timestamp_us;
            previous_level = level;
            (void)telemetry_try_emit_barcode_sample(&sample);
        }

        if (wait_for_gap(&idle_waits))
        {
            reject_partial(&parser, BARCODE_REJECT_DELIMITER, edge_odo_mm);
            have_previous = false;
        }
    }
}

/*** end of file ***/
