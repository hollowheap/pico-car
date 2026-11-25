/** @file barcode.h
 *
 * @brief Code 39 barcode decode task facade.
 *
 * The line ISR timestamps each barcode edge into a bounded ring. The barcode
 * task drains the ring and feeds element widths to the parser. A decoded
 * letter becomes an event and, for S, L, R and U, a command for the
 * supervisor. The barcode task never drives the car itself.
 */

#ifndef TRACK_BARCODE_H
#define TRACK_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "motion/motion_state.h"
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

/* Return the wrapping accepted-element count after a verified start. Also
 * report the measured length of the current tentative start, zero on rejection. */
uint8_t barcode_progress_count (uint8_t * p_prefix_mm);

/* Last measured element and parser phase packed for the status line byte. */
uint8_t barcode_status_bits (void);

/* Register the task stack. */
void barcode_init (void);

/* Create and start the barcode task. The task is optional: returns true when it
   was created, even if it then failed to start. */
bool barcode_start (void);

/* Task body. Never returns. */
void barcode_task_run (void);

/**
 * @brief Convert barcode element pulse interval to distance with motion
 * feedback snapshot.
 *
 * @param interval_us Duration of element pulse in microseconds.
 * @param end_us      Timestamp of the edge that ends the element.
 * @param p_feedback Pointer to motion feedback snapshot.
 * @param p_width_um Output pointer for element width in micrometers.
 *
 * @return true on success, false if snapshot is stale, invalid, or speed is
 *         non-positive, or the speed was sampled more than
 *         BARCODE_SPEED_SLACK_US outside the element.
 */
bool barcode_width_um (uint32_t                interval_us,
                       uint32_t                end_us,
                       motion_feedback_t const * p_feedback,
                       uint16_t                * p_width_um);

/** How far outside an element the speed sample may be: two 10 ms motion
 *  control periods. A guess until measured on the car. */
#define BARCODE_SPEED_SLACK_US (20000U)

/** Longest black element that can be a Code 39 bar: a wide bar is 2 to 3
 *  narrow ones, and a narrow one is at most 16.4 mm. */
#define BARCODE_BLOCK_UM (50000U)

/**
 * @brief Tell a solid black block on the track from a barcode bar.
 *
 * @param is_space True for a white element.
 * @param width_um Element width from barcode_width_um (live speed).
 *
 * @return true for a black element longer than BARCODE_BLOCK_UM.
 */
bool barcode_element_is_block (bool is_space, uint16_t width_um);

/** Least travel between two `barcode_block` events: at most one event
 *  per 200 mm, so a striped floor cannot flood the event queue. */
#define BARCODE_BLOCK_EVENT_MM (200U)

/* Block measurement for the `barcode_block` event. The odometer is kept
 * modulo 2^31 mm so the flag shares its word. */
typedef struct
{
    uint32_t last_event_mm;  /* odometer at the last event              */
    uint32_t black_mm : 31;  /* odometer at the edge where black began  */
    uint32_t b_black  : 1;   /* a black stretch is open                 */
} barcode_block_t;

STATIC_ASSERT (sizeof(barcode_block_t) == 8U, "barcode_block_t size");

/** Initial value: the first event passes the spacing check at any odometer. */
#define BARCODE_BLOCK_INIT {0U - BARCODE_BLOCK_EVENT_MM, 0U, 0U}

/** Start with no black stretch open; the first block reports at once. */
void barcode_block_init (barcode_block_t * p_block);

/**
 * @brief Feed one barcode-sensor edge. The length of a black stretch
 *        is the odometer travel between its two edges, so it does not
 *        depend on the 16-bit element width or on the 500 ms barcode gap.
 *
 * @param p_block  State.
 * @param is_black The element that starts at this edge is black.
 * @param odo_mm   Odometer at the edge.
 * @param p_measured True for any measured block ending here, even when its
 *                   event is throttled. May be NULL.
 *
 * @return The length in mm of a block that just ended, saturated at
 *         INT16_MAX, when it is longer than BARCODE_BLOCK_UM and the last
 *         event is BARCODE_BLOCK_EVENT_MM back; otherwise 0.
 */
uint16_t barcode_block_edge (barcode_block_t * p_block, bool is_black, uint32_t odo_mm,
                            bool * p_measured);

#endif /* TRACK_BARCODE_H */

/*** end of file ***/
