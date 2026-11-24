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

/**
 * Initialise line sampling and barcode edge capture. Returns the setup error
 * on failure, after releasing semaphores and interrupt resources.
 */
ER ir_capture_init (void);
/** Get the current state of the IR sensors. */
ir_sensors_state_t ir_get_state (void);
/** On the capture core, get the alarm-sampled barcode and debounced line levels
 *  with their TIMER timestamp. */
line_sample_t ir_get_line_sample (void);
/** Wait for a change in the IR sensors of the given type. */
ER ir_wait_change (uint8_t type, TMO tmout);

typedef enum
{
    IR_BARCODE_EMPTY = 0,
    IR_BARCODE_EDGE,
    IR_BARCODE_EDGE_AFTER_LOSS
} ir_barcode_read_t;

STATIC_ASSERT(sizeof(ir_barcode_read_t) <= 4U, "ir_barcode_read_t size");

/** Pop a timestamped edge (2 us resolution). After a capture loss the next
 * edge establishes a new baseline for measuring stripe widths. */
ir_barcode_read_t ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level);

/** Barcode edges lost to an ambiguous transition or a full ISR ring. */
uint32_t ir_barcode_drops (void);

/* A queued barcode edge qualifies for line suppression for BARCODE_SUPPRESS_US
 * from its capture timestamp. After a lost edge, require a full quiet interval
 * of that length before the next queued edge can qualify. */
bool ir_barcode_recent (void);

/** Line timer alarm overruns. */
uint32_t ir_line_overruns (void);

/** Filtered line sensor state changes since init (both sensors counted). */
uint32_t ir_line_edges (void);
/** Raw comparator changes since init. Compare with ir_line_edges to see chatter. */
uint32_t ir_line_raw_edges (void);

#endif /* TRACK_TRACK_H */

/*** end of file ***/
