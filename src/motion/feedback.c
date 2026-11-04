#include <stddef.h>

#include "motion/feedback.h"

void motion_feedback_evaluate (motion_feedback_t * p_feedback, uint32_t now_us)
{
    if (p_feedback == NULL)
    {
        return;
    }

    p_feedback->age_us = now_us - p_feedback->timestamp_us;
    p_feedback->fresh  = p_feedback->speed_valid &&
                        p_feedback->odometry_valid &&
                        (p_feedback->age_us < MOTION_FEEDBACK_MAX_AGE_US);
}
