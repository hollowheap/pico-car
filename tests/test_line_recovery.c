#include <stdio.h>

#include "track/line_recovery.h"

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
    line_recovery_t recovery;

    check(line_recovery_init(&recovery, 130000), "init");
    line_recovery_update(&recovery, 100000, 100000);
    check(line_recovery_heading_mdeg(&recovery) == 0, "straight");

    line_recovery_reset(&recovery, 0, 0);
    line_recovery_update(&recovery, 0, 100000);
    check(line_recovery_heading_mdeg(&recovery) == 44073, "right turn");

    line_recovery_reset(&recovery, 0, 0);
    line_recovery_update(&recovery, 100000, 0);
    check(line_recovery_heading_mdeg(&recovery) == -44073, "left turn");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("line recovery OK\n");
    return 0;
}
