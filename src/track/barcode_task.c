/** @file barcode_task.c
 *
 * @brief Decode Code 39 elements from the ISR edge ring, and report each
 *        barcode (D130).
 *
 * A decoded letter becomes the event `barcode`. S, L, R and U also become a
 * command for the supervisor with source barcode. Every barcode that is
 * refused (a parser error, a stale speed, the same marker twice) becomes the
 * event `barcode_bad` with a reason.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "barcode/action.h"
#include "barcode/parser.h"
#include "motion/motion.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "supervisor/supervisor.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/wire_schema.h"
#include "track/barcode.h"
#include "track/track.h"

#define US_PER_MS            1000U
#define BARCODE_GAP_US       500000U
#define BARCODE_GAP_MS       (BARCODE_GAP_US / US_PER_MS)
#define BARCODE_LEVEL_SPACE  1U

/* Event p2 of `barcode`: width/4 um in bits 4..15, reverse in bit 3, action in
   bits 0..2. A narrow width is at most 16383 um, so width/4 fits 12 bits. */
#define EVENT_WIDTH_SHIFT   (4U)
#define EVENT_WIDTH_DIV     (4U)
#define EVENT_REVERSE_SHIFT (3U)

static volatile uint16_t g_ok;
static volatile uint16_t g_bad;
static volatile uint8_t  g_last_letter;
static volatile uint8_t  g_last_action;
static barcode_gate_t    g_gate;

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

static void report_bad (uint8_t reason)
{
    g_bad++;
    (void)telemetry_emit_event((uint8_t)TELEMETRY_EVENT_SOURCE_BARCODE,
                               (uint8_t)TELEMETRY_EVT_BARCODE_BAD, (int16_t)reason, 0U,
                               odometer_mm());
    LOG_WARN(LOG_TASK_BARCODE, LOG_MOD_NONE, "barcode refused, reason %d", (int)reason);
}

/* A decoded letter: gate it, report it, and post the command for S, L, R, U. */
static void report_decoded (uint8_t code, const barcode_info_t * p_info)
{
    uint32_t const         odo    = odometer_mm();
    barcode_action_t const act    = barcode_action_of(code);
    uint8_t const          letter = (uint8_t)(code + (uint8_t)'A');
    uint16_t const         width  = (uint16_t)(p_info->narrow_width_um / EVENT_WIDTH_DIV);
    uint32_t const         bits   = ((uint32_t)width << EVENT_WIDTH_SHIFT) |
                                    ((p_info->is_reverse ? 1UL : 0UL) << EVENT_REVERSE_SHIFT) |
                                    (uint32_t)act.action;
    uint16_t const         detail = (uint16_t)bits;

    if (!barcode_gate_pass(&g_gate, odo))
    {
        report_bad(BARCODE_REJECT_DUPLICATE);
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

void barcode_task_run (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    uint32_t         previous_us = 0U;
    bool             previous_level = false;
    bool             have_previous = false;

    barcode_gate_init(&g_gate);

    for (;;)
    {
        uint32_t timestamp_us = 0U;
        bool     level        = false;
        ER       er           = E_OK;

        liveness_kick(LIVE_BARCODE);
        while (ir_barcode_pop(&timestamp_us, &level))
        {
            uint32_t const gap_us = timestamp_us - previous_us;

            if (have_previous && (gap_us > BARCODE_GAP_US))
            {
                /* A new barcode starts after a long gap. A barcode that had
                 * begun was never closed. */
                if (0U != (parser.elem_index | parser.char_index))
                {
                    report_bad(BARCODE_REJECT_DELIMITER);
                }
                parser        = (barcode_parser_t)BARCODE_PARSER_INIT;
                have_previous = false;
            }

            if (have_previous)
            {
                motion_feedback_t feedback = {0};
                uint16_t          width_um = 0U;
                bool              ok       = false;

                if (E_OK == motion_feedback_snapshot(&feedback))
                {
                    ok = barcode_width_um(gap_us, &feedback, &width_um);
                }

                if (!ok)
                {
                    /* A stale snapshot, zero speed, or reversing speed rejects
                     * the current character instead of decoding a time-based width. */
                    report_bad(BARCODE_REJECT_SPEED);
                    parser        = (barcode_parser_t)BARCODE_PARSER_INIT;
                    have_previous = false;
                }
                else
                {
                    bool const is_space = (0U != BARCODE_LEVEL_SPACE)
                                              ? previous_level
                                              : !previous_level;
                    barcode_info_t info = {0U, false, BARCODE_REJECT_NONE, false};
                    uint8_t const  code =
                        barcode_parser_input_info(&parser, is_space, width_um, &info);

                    if (BARCODE_REJECT_NONE != info.reject)
                    {
                        report_bad(info.reject);
                    }
                    if (BARCODE_NO_CHAR != code)
                    {
                        report_decoded(code, &info);
                    }
                }
            }
            else
            {
                have_previous = true;
            }

            previous_us    = timestamp_us;
            previous_level = level;
        }

        er = ir_wait_change(IR_TYPE_BARCODE, (TMO)BARCODE_GAP_MS);
        if (E_TMOUT == er)
        {
            if (0U != (parser.elem_index | parser.char_index))
            {
                report_bad(BARCODE_REJECT_DELIMITER);
            }
            parser        = (barcode_parser_t)BARCODE_PARSER_INIT;
            have_previous = false;
        }
    }
}

/*** end of file ***/
