/* The obstacle stop cause is never authority to begin another run. */

#include <stdio.h>

#include "obstacle_pack.h"

int main (void)
{
    uint32_t cause = 0U;
    unsigned restarted = 0U;

    for (cause = 0U; cause < OBSTACLE_CAUSE_COUNT; cause++)
    {
        if (obstacle_pack_cause_may_auto_restart (cause))
        {
            (void)fprintf (stderr, "Obstacle cause %lu permits automatic RESET and START\n",
                           (unsigned long)cause);
            restarted++;
        }
    }
    return (0U == restarted) ? 0 : 1;
}
