/** @file ir_edge.h
 * @brief Pure helpers for `ir_capture.c`: barcode edge decision and alarm
 * catch-up. No kernel, no hardware.
 */
#ifndef IR_EDGE_H
#define IR_EDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "ir_pins.h"

typedef enum
{
    IR_BARCODE_NONE,
    IR_BARCODE_RISE,
    IR_BARCODE_FALL,
    /** A rise and a fall in one snapshot: the pulse was shorter than the ISR
     * latency, the order is unknown and the level is back where it was. */
    IR_BARCODE_PULSE
} ir_barcode_event_t;

/** Decide the barcode pin event from the edge snapshot of the GPIO ISR. */
static inline ir_barcode_event_t ir_barcode_event (uint32_t rise_mask,
                                                   uint32_t fall_mask)
{
    bool const rise = (0U != ((rise_mask >> IR_BARCODE_PIN) & 1U));
    bool const fall = (0U != ((fall_mask >> IR_BARCODE_PIN) & 1U));

    if (rise && fall)
    {
        return IR_BARCODE_PULSE;
    }
    if (rise)
    {
        return IR_BARCODE_RISE;
    }
    return fall ? IR_BARCODE_FALL : IR_BARCODE_NONE;
}

/**
 * Next alarm target: the first `target + n * period` (n >= 1) after `now`.
 * `*p_missed` receives the number of periods that had already passed, without
 * looping, so a stalled clock cannot hold the ISR.
 */
static inline uint32_t ir_alarm_next (uint32_t target, uint32_t now,
                                      uint32_t period, uint32_t * p_missed)
{
    uint32_t const next   = target + period;
    int32_t const  behind = (int32_t)(now - next);
    uint32_t       missed = 0U;

    if (behind >= 0)
    {
        missed = ((uint32_t)behind / period) + 1U;
    }
    *p_missed = missed;
    return next + (missed * period);
}

#endif /* IR_EDGE_H */

/*** end of file ***/
