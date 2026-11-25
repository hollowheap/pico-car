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
    bool     b_turning;  /* a one-shot turn is running */
} line_task_stats_t;

STATIC_ASSERT (sizeof(line_task_stats_t) <= 4U, "line_task_stats_t size");

/* Consistent enough for telemetry: each field is read whole. */
void line_task_stats (line_task_stats_t * p_stats);

/* The hug snapshot for the obstacle frame: two words (telemetry/hug_word.h)
 * read together under one spinlock. Both are 0 until the pilot has run, and
 * with the pilot off. p_words holds HUG_WORD_COUNT (2) words. */
void line_task_hug (uint32_t * p_words);

/* Task body. Never returns. */
void line_task_run (void);

#endif /* TRACK_LINE_TASK_H */
