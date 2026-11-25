/** @file motion_link.c
 * @brief Bind the sonar service to the car's wheel travel at startup.
 */
#include "motion_link.h"

#include "nav_core.h"
#include "obstacle_config.h"
#include "navigation/obstacle.h"
#if !PICO_CAR_PILOT
#include "obstacle_nav.h"
#endif

/* Only the sonar task advances these totals. */
static nav_travel_t g_travel;

int64_t obstacle_wheel_um (motion_feedback_t const * p_feedback, uint32_t wheel)
{
#if OBSTACLE_ENCODERS_CROSSED
    /* Crossed plugs reverse both the wheel assignment and its forward sign. */
    return (wheel == NAV_CORE_LEFT) ? -p_feedback->right_displacement_um : -p_feedback->left_displacement_um;
#else
    return (wheel == NAV_CORE_LEFT) ? p_feedback->left_displacement_um : p_feedback->right_displacement_um;
#endif
}

static ER sonar_travel (obstacle_travel_t * p_travel)
{
    motion_feedback_t feedback;
    ER                result = E_OBJ;

    if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
    {
        int64_t const left_um = obstacle_wheel_um(&feedback, NAV_CORE_LEFT);
        int64_t const right_um = obstacle_wheel_um(&feedback, NAV_CORE_RIGHT);
        nav_core_travel_update(&g_travel, (left_um + right_um) / (int64_t)MOTION_WHEELS);
        p_travel->fwd_mm  = g_travel.fwd_mm;
        p_travel->back_mm = g_travel.back_mm;
        result            = E_OK;
    }

    return (result);
}

ER obstacle_line_init (void)
{
#if PICO_CAR_PILOT
    obstacle_links_t const links = {.p_get_distance_mm = sonar_travel};
    return obstacle_init(&links);
#else
    return obstacle_nav_init(sonar_travel);
#endif
}
