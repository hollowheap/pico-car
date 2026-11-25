/** @file barcode_task.c
 *
 * @brief Decode Code 39 elements from the ISR edge ring.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "barcode/parser.h"
#include "track/barcode.h"
#include "track/track.h"

#define BARCODE_POLL_MS      10U
#define BARCODE_GAP_US       500000U
#define BARCODE_LEVEL_SPACE  1U

static barcode_handler_t g_handler;

void barcode_set_handler (barcode_handler_t handler)
{
    g_handler = handler;
}

void barcode_task_run (void)
{
    barcode_parser_t parser = BARCODE_PARSER_INIT;
    uint32_t         previous_us = 0U;
    bool             previous_level = false;
    bool             have_previous = false;

    for (;;)
    {
        uint32_t timestamp_us;
        bool     level;

        while (ir_barcode_pop(&timestamp_us, &level))
        {
            uint32_t const gap_us = timestamp_us - previous_us;

            if (have_previous && (gap_us > BARCODE_GAP_US))
            {
                /* A new barcode starts after a long gap. */
                parser        = (barcode_parser_t)BARCODE_PARSER_INIT;
                have_previous = false;
            }

            if (have_previous)
            {
                bool const is_space = (0U != BARCODE_LEVEL_SPACE)
                                          ? previous_level
                                          : !previous_level;
                uint16_t const width_um =
                    (gap_us > UINT16_MAX) ? (uint16_t)UINT16_MAX
                                          : (uint16_t)gap_us;
                uint8_t const code =
                    barcode_parser_input(&parser, is_space, width_um);

                if ((BARCODE_NO_CHAR != code) && (NULL != g_handler))
                {
                    g_handler(code);
                }
            }

            previous_us    = timestamp_us;
            previous_level = level;
            have_previous  = true;
        }

        (void)tk_dly_tsk(BARCODE_POLL_MS);
    }
}

/*** end of file ***/
