#ifndef IR_PINS_H
#define IR_PINS_H

#include "platform/static_assert.h"

#define IR_BARCODE_PIN 26U
#define IR_LEFT_PIN    27U
#define IR_RIGHT_PIN   28U

/* Pin level of a line sensor that is over the line, and the level for no line.
 * An unplugged sensor must read as no line, so the pull goes to that level. */
#define IR_LEVEL_LINE    1U
#define IR_LEVEL_NO_LINE 0U

STATIC_ASSERT(IR_LEVEL_LINE != IR_LEVEL_NO_LINE, ir_line_levels_differ);
STATIC_ASSERT(IR_LEVEL_LINE <= 1U, ir_line_level_is_one_bit);

/* Pin level of the barcode sensor over a space (white). The pull goes to it,
 * so an unplugged sensor decodes no bars. */
#define BARCODE_LEVEL_SPACE 0U

STATIC_ASSERT(BARCODE_LEVEL_SPACE <= 1U, barcode_level_is_one_bit);

/* GPIO pad control pull bits, unsigned copies of the kernel GPIO_PUE and
 * GPIO_PDE, which BARR-C 5.3.b forbids in bitwise expressions. */
#define IR_PAD_PULL_UP   (1UL << 3U)
#define IR_PAD_PULL_DOWN (1UL << 2U)

/* Pull of a line sensor pin that makes a missing sensor read as no line. */
#define IR_PAD_PULL_NO_LINE \
    ((0U == IR_LEVEL_NO_LINE) ? IR_PAD_PULL_DOWN : IR_PAD_PULL_UP)
#define IR_PAD_PULL_LINE \
    ((0U == IR_LEVEL_NO_LINE) ? IR_PAD_PULL_UP : IR_PAD_PULL_DOWN)
#define IR_PAD_PULL_SPACE \
    ((0U == BARCODE_LEVEL_SPACE) ? IR_PAD_PULL_DOWN : IR_PAD_PULL_UP)
#define IR_PAD_PULL_BAR \
    ((0U == BARCODE_LEVEL_SPACE) ? IR_PAD_PULL_UP : IR_PAD_PULL_DOWN)

#endif /* IR_PINS_H */

/*** end of file ***/
