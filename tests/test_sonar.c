#include <stdio.h>

#include "navigation/sonar.h"

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
    sonar_state_t state = {0};

    check(sonar_echo_to_mm(1166U) == 200U, "200 mm echo");
    check(sonar_classify_echo(1166U, SONAR_ECHO_TIMEOUT_US) == SONAR_VALID,
          "valid echo");
    check(sonar_classify_echo(0U, SONAR_ECHO_TIMEOUT_US) == SONAR_TIMEOUT,
          "zero echo timeout");
    check(sonar_classify_echo(SONAR_ECHO_TIMEOUT_US,
                              SONAR_ECHO_TIMEOUT_US) == SONAR_TIMEOUT,
          "timeout boundary");
    check(sonar_classify_echo(25000U, SONAR_ECHO_TIMEOUT_US) == SONAR_OPEN,
          "open range");

    check(sonar_should_stop(200U), "stop at 200 mm");
    check(!sonar_should_stop(201U), "no stop past 200 mm");

    sonar_note_echo(&state, SONAR_TIMEOUT);
    sonar_note_echo(&state, SONAR_TIMEOUT);
    check(!state.fault_latched, "fault after two timeouts");
    sonar_note_echo(&state, SONAR_TIMEOUT);
    check(state.fault_latched, "fault after three timeouts");

    if (failures != 0U)
    {
        return 1;
    }
    (void)printf("sonar classifier OK\n");
    return 0;
}
