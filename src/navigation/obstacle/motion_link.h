#ifndef OBSTACLE_MOTION_LINK_H
#define OBSTACLE_MOTION_LINK_H

#include "motion/motion.h"

/* Travel of the physical wheel, positive forwards, after wiring correction. */
int64_t obstacle_wheel_um (motion_feedback_t const * p_feedback, uint32_t wheel);

#endif /* OBSTACLE_MOTION_LINK_H */
