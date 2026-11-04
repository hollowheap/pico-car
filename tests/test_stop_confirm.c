#include <stdio.h>

#include "navigation/stop_confirm.h"

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
    stop_confirm_t confirm;

    stop_confirm_init(&confirm, 1000U, 0, 0);
    check(!stop_confirm_ready(&confirm, 50999U), "not ready before 50 ms");
    check(stop_confirm_ready(&confirm, 51000U), "ready at 50 ms");

    stop_confirm_update(&confirm, 52000U, 100, 100);
    check(!stop_confirm_ready(&confirm, 101999U), "movement resets timer");
    check(stop_confirm_ready(&confirm, 102000U), "ready after quiet");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("stop confirm OK\n");
    return 0;
}
