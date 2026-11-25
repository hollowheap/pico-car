/* Start the production sonar adapter without linking any movement command writer. */
#include <stdio.h>
#include <stdlib.h>

#include "motion/motion_state.h"
#include "obstacle/obstacle.h"

#define CHECK(condition) do {                                        \
    if (!(condition)) {                                               \
        (void)fprintf(stderr, "line %d: %s\n", __LINE__, #condition);  \
        abort();                                                      \
    }                                                                 \
} while (0)

static obstacle_links_t installed;
static motion_feedback_t feedback;
static ER init_result;

ER obstacle_setup (obstacle_links_t const * p_links)
{
    installed = *p_links;
    return init_result;
}

int motion_feedback_snapshot (motion_feedback_t * p_out)
{
    *p_out = feedback;
    return E_OK;
}

int main (void)
{
    obstacle_travel_t travel = {0};
    init_result = E_OBJ;
    CHECK(obstacle_line_init() == E_OBJ);
    init_result = E_OK;
    CHECK(obstacle_line_init() == E_OK);
    CHECK(installed.p_stop == NULL && installed.p_get_distance_mm != NULL);

    feedback.fresh = true;
    CHECK(installed.p_get_distance_mm(&travel) == E_OK);
    feedback.left_displacement_um = 10000;
    feedback.right_displacement_um = 14000;
    CHECK(installed.p_get_distance_mm(&travel) == E_OK);
    CHECK(travel.fwd_mm == 12U && travel.back_mm == 0U);
    feedback.fresh = false;
    feedback.left_displacement_um = 8000;
    feedback.right_displacement_um = 10000;
    CHECK(installed.p_get_distance_mm(&travel) == E_OBJ);
    CHECK(travel.fwd_mm == 12U && travel.back_mm == 0U);
    feedback.fresh = true;
    CHECK(installed.p_get_distance_mm(&travel) == E_OK);
    CHECK(travel.fwd_mm == 12U && travel.back_mm == 3U);
    return 0;
}
