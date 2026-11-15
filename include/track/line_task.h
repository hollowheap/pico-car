/** @file line_task.h
 *
 * @brief Line task facade: line following, one-shot turns and the hand-over
 *        to the obstacle step.
 *
 * The line task is the only writer of the wheel speeds while it follows the
 * line. It takes turns the supervisor allowed and runs them as pivots. On
 * every pass it lets the obstacle step take the motors.
 */

#ifndef TRACK_LINE_TASK_H
#define TRACK_LINE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/* Counters for the status and diag packets. Written by the line task only. */
typedef struct
{
    uint16_t lost_count; /* line-lost entries since boot */
    uint16_t search_mm;  /* longest stretch driven while lost */
    bool     b_turning;  /* a one-shot turn is running */
} line_task_stats_t;

STATIC_ASSERT (sizeof(line_task_stats_t) <= 6U, "line_task_stats_t size");

/* Consistent enough for telemetry: each field is read whole. */
void line_task_stats (line_task_stats_t * p_stats);

/* Task body. Never returns. */
void line_task_run (void);

#endif /* TRACK_LINE_TASK_H */
