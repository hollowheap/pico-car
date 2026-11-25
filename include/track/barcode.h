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

#include <stdint.h>

/* Receives 0..25 for 'A'..'Z'. */
typedef void (*barcode_handler_t)(uint8_t code);

void barcode_set_handler (barcode_handler_t handler);

/* Task body. Never returns. */
void barcode_task_run (void);

#endif /* TRACK_BARCODE_H */

/*** end of file ***/
