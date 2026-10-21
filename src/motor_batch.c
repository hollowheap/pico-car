/** @file motor_batch.c
 * @brief Short pulses with braked pauses and a terminal stop latch.
 */
#include <stddef.h>
#include "motor_batch.h"

static const int16_t duties[MOTOR_BATCH_STEPS][2] = {
    { MOTOR_BATCH_DUTY, 0 }, { 0, MOTOR_BATCH_DUTY },
    { MOTOR_BATCH_DUTY, MOTOR_BATCH_DUTY },
    { -MOTOR_BATCH_DUTY, 0 }, { 0, -MOTOR_BATCH_DUTY },
    { -MOTOR_BATCH_DUTY, -MOTOR_BATCH_DUTY }
};

void motor_batch_init (motor_batch_t * p_batch)
{
    if (p_batch != NULL)
    {
        *p_batch = (motor_batch_t){0};
    }
}

motor_batch_phase_t motor_batch_update (motor_batch_t * p_batch,
    uint32_t now_us, bool start_pressed, bool stop_or_fault)
{
    if (p_batch == NULL)
    {
        return MOTOR_BATCH_STOPPED;
    }
    if (p_batch->phase == MOTOR_BATCH_RUN)
    {
        uint32_t gap = now_us - p_batch->previous_us;

        if (gap > p_batch->max_gap_us)
        {
            p_batch->max_gap_us = gap;
        }
        if (gap > MOTOR_BATCH_MAX_GAP_US)
        {
            p_batch->gap_fault = true;
        }
    }
    p_batch->previous_us = now_us;
    if (stop_or_fault || p_batch->gap_fault)
    {
        p_batch->phase = MOTOR_BATCH_STOPPED;
    }
    switch (p_batch->phase)
    {
        case MOTOR_BATCH_WAIT:
            if (motor_pulse_update(&p_batch->gate, now_us, start_pressed,
                                   false) != MOTOR_PULSE_OFF)
            {
                p_batch->phase = MOTOR_BATCH_RUN;
                p_batch->changed_us = now_us;
            }
            break;
        case MOTOR_BATCH_RUN:
            if ((uint32_t)(now_us - p_batch->changed_us) >= MOTOR_BATCH_PULSE_US)
            {
                p_batch->phase = MOTOR_BATCH_PAUSE;
                p_batch->changed_us = now_us;
            }
            break;
        case MOTOR_BATCH_PAUSE:
            if ((uint32_t)(now_us - p_batch->changed_us) >= MOTOR_BATCH_PAUSE_US)
            {
                p_batch->step++;
                p_batch->phase = p_batch->step < MOTOR_BATCH_STEPS ?
                    MOTOR_BATCH_RUN : MOTOR_BATCH_DONE;
                p_batch->changed_us = now_us;
            }
            break;
        case MOTOR_BATCH_DONE:
        case MOTOR_BATCH_STOPPED:
            break;
        default:
            p_batch->phase = MOTOR_BATCH_STOPPED;
            break;
    }
    return p_batch->phase;
}

void motor_batch_duties (const motor_batch_t * p_batch,
                         int16_t * p_left, int16_t * p_right)
{
    if ((p_left != NULL) && (p_right != NULL))
    {
        *p_left = 0;
        *p_right = 0;
        if ((p_batch != NULL) && (p_batch->phase == MOTOR_BATCH_RUN) &&
            (p_batch->step < MOTOR_BATCH_STEPS))
        {
            *p_left = duties[p_batch->step][0];
            *p_right = duties[p_batch->step][1];
        }
    }
}

motor_batch_feedback_t motor_batch_feedback (
    const encoder_quad_counts_t start[ENCODER_WHEELS],
    const encoder_quad_counts_t end[ENCODER_WHEELS], int16_t left, int16_t right)
{
    const int16_t requested[2] = {left, right};
    unsigned int wheel;

    if ((start == NULL) || (end == NULL))
    {
        return MOTOR_BATCH_FEEDBACK_INVALID;
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        if ((uint32_t)(end[wheel].invalid - start[wheel].invalid) != 0U)
        {
            return MOTOR_BATCH_FEEDBACK_INVALID;
        }
    }
    for (wheel = 0U; wheel < ENCODER_WHEELS; wheel++)
    {
        int64_t net = (int64_t)(uint32_t)(end[wheel].positive - start[wheel].positive) -
                      (int64_t)(uint32_t)(end[wheel].negative - start[wheel].negative);

        if (wheel == ENCODER_LEFT)
        {
            net = -net;
        }
        if (requested[wheel] != 0)
        {
            if (net == 0)
            {
                return MOTOR_BATCH_FEEDBACK_MISSING;
            }
            if (((requested[wheel] > 0) && (net < 0)) ||
                ((requested[wheel] < 0) && (net > 0)))
            {
                return MOTOR_BATCH_FEEDBACK_REVERSED;
            }
        }
    }
    return MOTOR_BATCH_FEEDBACK_OK;
}
