/** @file motion_link.c
 * @brief Bind the sonar service to the car's wheel travel at startup.
 */
#include "motion_link.h"

#include "obstacle_config.h"
#include "navigation/obstacle.h"
#include "platform/units.h"

#define WHEEL_LEFT  (0U)
#define WHEEL_RIGHT (1U)

typedef struct
{
    uint32_t fwd_mm;
    uint32_t back_mm;
    int32_t  last_mm;
    bool     b_base;
} sonar_travel_t;

/* Only the sonar task advances these totals. */
static sonar_travel_t g_travel;

int64_t obstacle_wheel_um (motion_feedback_t const * p_feedback, uint32_t wheel)
{
#if OBSTACLE_ENCODERS_CROSSED
    /* Crossed plugs reverse both the wheel assignment and its forward sign. */
    return (wheel == WHEEL_LEFT) ? -p_feedback->right_displacement_um : -p_feedback->left_displacement_um;
#else
    return (wheel == WHEEL_LEFT) ? p_feedback->left_displacement_um : p_feedback->right_displacement_um;
#endif
}

static void travel_update (sonar_travel_t * p_travel, int64_t net_um)
{
    int32_t const net_mm = (int32_t)(net_um / UM_PER_MM);

    if (p_travel->b_base)
    {
        int32_t const step = net_mm - p_travel->last_mm;

        if (step > 0)
        {
            p_travel->fwd_mm += (uint32_t)step;
        }
        else
        {
            p_travel->back_mm += (uint32_t)(-step);
        }
    }
    p_travel->last_mm = net_mm;
    p_travel->b_base  = true;
}

static ER sonar_travel (obstacle_travel_t * p_travel)
{
    motion_feedback_t feedback;
    ER                result = E_OBJ;

    if ((E_OK == motion_feedback_snapshot(&feedback)) && feedback.fresh)
    {
        int64_t const left_um = obstacle_wheel_um(&feedback, WHEEL_LEFT);
        int64_t const right_um = obstacle_wheel_um(&feedback, WHEEL_RIGHT);
        travel_update(&g_travel, (left_um + right_um) / (int64_t)MOTION_WHEELS);
        p_travel->fwd_mm  = g_travel.fwd_mm;
        p_travel->back_mm = g_travel.back_mm;
        result            = E_OK;
    }

    return (result);
}

ER obstacle_line_init (void)
{
    obstacle_links_t const links = {.p_get_distance_mm = sonar_travel};
    return obstacle_setup(&links);
}
