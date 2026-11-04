#include <stdio.h>

#include "navigation/mission_events.h"

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
    bool stop = false;

    check(mission_apply_events(MISSION_STATE_READY,
                               MISSION_EVENT_STOP | MISSION_EVENT_RESET |
                                   MISSION_EVENT_START,
                               &stop) == MISSION_STATE_STOPPED,
          "STOP dominates RESET/START");
    check(stop, "STOP requests brake");

    check(mission_apply_events(MISSION_STATE_RUNNING,
                               MISSION_EVENT_FAULT | MISSION_EVENT_RESET,
                               &stop) == MISSION_STATE_STOPPED,
          "FAULT dominates RESET");
    check(stop, "FAULT requests brake");

    check(mission_apply_events(MISSION_STATE_STOPPED, MISSION_EVENT_RESET,
                               &stop) == MISSION_STATE_READY,
          "RESET after stop");
    check(!stop, "RESET does not brake");

    check(mission_apply_events(MISSION_STATE_READY, MISSION_EVENT_START,
                               &stop) == MISSION_STATE_RUNNING,
          "START from ready");

    check(mission_apply_events(MISSION_STATE_STOPPED, MISSION_EVENT_START,
                               &stop) == MISSION_STATE_STOPPED,
          "START cannot revive stopped");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("mission events OK\n");
    return 0;
}
