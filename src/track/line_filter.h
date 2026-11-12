/** @file line_filter.h
 * @brief Pure per-sensor debounce for the line sensors (S09).
 *
 * The 5 ms TIMER alarm feeds one raw sample per sensor. A sensor changes its
 * output only after LINE_DEBOUNCE_SAMPLES consecutive raw samples disagree
 * with the present output: 15 ms at the 5 ms sample period, in both
 * directions. A single-sample reflection, glare or dropout never reaches the
 * follower. No kernel, no hardware, integer only.
 */

#ifndef TRACK_LINE_FILTER_H
#define TRACK_LINE_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/** Consecutive disagreeing samples needed to change the output. */
#define LINE_DEBOUNCE_SAMPLES (3U)

typedef struct
{
    /** Consecutive raw samples that disagree with the output. */
    uint8_t count;
    /** Filtered output: true = the sensor is over the line. */
    bool b_level;
} line_debounce_t;

STATIC_ASSERT (sizeof(line_debounce_t) == 2U, "line_debounce_t size");

/**
 * Start a filter at a known level, for example the first raw sample, so the
 * output does not report a false level for the first 15 ms.
 *
 * @param[out] p_filter Filter to set up. A NULL pointer is ignored.
 * @param[in]  b_level  Initial output level.
 */
void line_debounce_init (line_debounce_t * p_filter, bool b_level);

/**
 * Feed one raw sample and return the filtered output.
 *
 * @param[in,out] p_filter Filter state. A NULL pointer returns false.
 * @param[in]     b_raw    Raw sensor level for this sample.
 *
 * @return The filtered level after this sample.
 */
bool line_debounce_update (line_debounce_t * p_filter, bool b_raw);

#endif /* TRACK_LINE_FILTER_H */

/*** end of file ***/
