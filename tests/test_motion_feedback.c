#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motion/feedback.h"

static unsigned int failures;

static void check (int condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

int main (void)
{
    motion_feedback_t feedback = {
        .left_speed_um_s       = -150,
        .right_speed_um_s      = 200,
        .left_displacement_um  = -1000,
        .right_displacement_um = 2000,
        .timestamp_us          = 1000U,
        .speed_valid           = true,
        .odometry_valid        = true,
    };

    motion_feedback_evaluate(&feedback, 2000U);
    check(feedback.age_us == 1000U, "age");
    check(feedback.fresh, "fresh");
    check(feedback.left_speed_um_s == -150, "signed speed");
    check(feedback.left_displacement_um == -1000, "signed displacement");

    motion_feedback_evaluate(&feedback, 30000U);
    check(!feedback.fresh, "stale");

    feedback.timestamp_us = UINT32_MAX - 10U;
    motion_feedback_evaluate(&feedback, 5U);
    check(feedback.age_us == 16U, "wrap age");
    check(feedback.fresh, "wrap fresh");

    feedback.speed_valid = false;
    motion_feedback_evaluate(&feedback, 6U);
    check(!feedback.fresh, "invalid speed");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("motion feedback OK\n");
    return 0;
}
