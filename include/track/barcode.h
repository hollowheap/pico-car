/** @file barcode.h
 *
 * @brief Code 39 barcode decode task facade.
 *
 * The line ISR timestamps each barcode edge into a bounded ring. The barcode
 * task drains the ring and feeds element widths to the parser. A decoded
 * letter becomes an event and, for S, L, R and U, a command for the
 * supervisor (D130). The barcode task never drives the car itself.
 */

#ifndef TRACK_BARCODE_H
#define TRACK_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion/motion.h"
#include "platform/static_assert.h"

/* Counters for the diag packet. Written by the barcode task only. */
typedef struct
{
    uint16_t ok;          /* barcodes decoded and acted on or rejected as a letter */
    uint16_t bad;         /* barcodes the parser, the speed check or the gate refused */
    uint8_t  last_letter; /* ASCII, 0 before the first */
    uint8_t  last_action; /* BARCODE_ACTION_* */
} barcode_stats_t;

STATIC_ASSERT (sizeof(barcode_stats_t) == 6U, "barcode_stats_t size");

/* Consistent enough for telemetry: each field is read whole. */
void barcode_stats (barcode_stats_t * p_stats);

/* Task body. Never returns. */
void barcode_task_run (void);

/**
 * @brief Convert barcode element pulse interval to distance with motion
 * feedback snapshot (D81, S33).
 *
 * @param interval_us Duration of element pulse in microseconds.
 * @param p_feedback Pointer to motion feedback snapshot.
 * @param p_width_um Output pointer for element width in micrometers.
 *
 * @return true on success, false if snapshot is stale, invalid, or speed is non-positive.
 */
bool barcode_width_um (uint32_t                interval_us,
                       motion_feedback_t const * p_feedback,
                       uint16_t                * p_width_um);

#endif /* TRACK_BARCODE_H */

/*** end of file ***/
