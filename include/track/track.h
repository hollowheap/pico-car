#ifndef TRACK_TRACK_H
#define TRACK_TRACK_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "platform/static_assert.h"

#define IR_TYPE_BARCODE 0U
#define IR_TYPE_LINE    1U

typedef struct
{
    /*
     * The barcode sensor.
     */
    bool barcode : 1;
    /*
     * The left line sensor.
     */
    bool left : 1;
    /*
     * The right line sensor.
     */
    bool right : 1;
} ir_sensors_state_t;

STATIC_ASSERT(sizeof(ir_sensors_state_t) == 1U,
              ir_sensors_state_must_be_1_byte);

/**
 * Line sample with timestamp (D88, S31).
 */
typedef struct
{
    uint32_t           timestamp_us;
    ir_sensors_state_t state;
} line_sample_t;

STATIC_ASSERT(sizeof(line_sample_t) == 8U,
              line_sample_must_be_8_bytes);

/** Initialise the IR sensor interrupt. */
ER ir_capture_init (void);
/** Get the current state of the IR sensors. */
ir_sensors_state_t ir_get_state (void);
/** Get the latest line sample with timestamp. */
line_sample_t ir_get_line_sample (void);
/** Wait for a change in the IR sensors of the given type. */
ER ir_wait_change (uint8_t type, TMO tmout);

/**
 * Pop one barcode edge. Returns false when the ring is empty. The timestamp
 * has 2 us resolution (bit 0 is always 0).
 */
bool ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level);

/** Barcode edges dropped because the ISR ring was full. */
uint32_t ir_barcode_drops (void);

/** Line timer alarm overruns. */
uint32_t ir_line_overruns (void);

#endif /* TRACK_TRACK_H */

/*** end of file ***/
