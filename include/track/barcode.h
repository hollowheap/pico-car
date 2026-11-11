/** @file barcode.h
 *
 * @brief Code 39 barcode decode task facade.
 *
 * The line ISR timestamps each barcode edge into a bounded ring. The barcode
 * task drains the ring and feeds element widths to the parser; a decoded
 * character is handed to the composition root's callback.
 */

#ifndef TRACK_BARCODE_H
#define TRACK_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion/motion.h"

/* Receives 0..25 for 'A'..'Z'. */
typedef void (*barcode_handler_t)(uint8_t code);

void barcode_set_handler (barcode_handler_t handler);

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
