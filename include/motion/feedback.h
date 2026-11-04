#ifndef MOTION_FEEDBACK_H
#define MOTION_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

#define MOTION_FEEDBACK_MAX_AGE_US 25000U

typedef struct
{
    int32_t  left_speed_um_s;
    int32_t  right_speed_um_s;
    int64_t  left_displacement_um;
    int64_t  right_displacement_um;
    uint32_t timestamp_us;
    uint32_t age_us;
    bool     speed_valid;
    bool     odometry_valid;
    bool     fresh;
} motion_feedback_t;

/** Fill age and freshness without changing measurement fields. */
void motion_feedback_evaluate (motion_feedback_t * p_feedback, uint32_t now_us);

#endif /* MOTION_FEEDBACK_H */
