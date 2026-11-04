#ifndef NAVIGATION_SONAR_H
#define NAVIGATION_SONAR_H

#include <stdbool.h>
#include <stdint.h>

#define SONAR_STOP_STANDOFF_MM 200U
#define SONAR_ECHO_TIMEOUT_US  35000U
#define SONAR_MAX_RANGE_MM     4000U

typedef enum
{
    SONAR_VALID = 0,
    SONAR_OPEN,
    SONAR_TIMEOUT,
    SONAR_FAULT
} sonar_echo_t;

typedef struct
{
    uint8_t consecutive_timeouts;
    bool    fault_latched;
} sonar_state_t;

uint32_t sonar_echo_to_mm (uint32_t echo_us);
sonar_echo_t sonar_classify_echo (uint32_t echo_us, uint32_t timeout_us);
bool sonar_should_stop (uint32_t distance_mm);
void sonar_note_echo (sonar_state_t * p_state, sonar_echo_t echo);

#endif /* NAVIGATION_SONAR_H */
