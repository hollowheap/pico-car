#include <stddef.h>

#include "navigation/sonar.h"

#define SONAR_MM_PER_US_NUM 343U
#define SONAR_MM_PER_US_DEN 2000U
#define SONAR_OPEN_ECHO_US  23324U

uint32_t sonar_echo_to_mm (uint32_t echo_us)
{
    return ((echo_us * SONAR_MM_PER_US_NUM) + (SONAR_MM_PER_US_DEN / 2U)) /
           SONAR_MM_PER_US_DEN;
}

sonar_echo_t sonar_classify_echo (uint32_t echo_us, uint32_t timeout_us)
{
    if ((echo_us == 0U) || (echo_us >= timeout_us))
    {
        return SONAR_TIMEOUT;
    }
    if (echo_us >= SONAR_OPEN_ECHO_US)
    {
        return SONAR_OPEN;
    }
    return SONAR_VALID;
}

bool sonar_should_stop (uint32_t distance_mm)
{
    return distance_mm <= SONAR_STOP_STANDOFF_MM;
}

void sonar_note_echo (sonar_state_t * p_state, sonar_echo_t echo)
{
    if (p_state == NULL)
    {
        return;
    }

    if (echo == SONAR_TIMEOUT)
    {
        if (p_state->consecutive_timeouts < UINT8_MAX)
        {
            p_state->consecutive_timeouts++;
        }
        if (p_state->consecutive_timeouts >= 3U)
        {
            p_state->fault_latched = true;
        }
    }
    else
    {
        p_state->consecutive_timeouts = 0U;
    }
}
