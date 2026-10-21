/** @file test_motor_pulse.c
 * @brief Verify no unintended start, timeout, stop priority and clock wrap.
 */
#include <stdio.h>
#include <stddef.h>
#include "motor_pulse.h"

static unsigned int failures;

static void check (bool condition, const char * p_name)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", p_name);
        failures++;
    }
}

static uint32_t trigger (motor_pulse_t * p_pulse, uint32_t time)
{
    (void)motor_pulse_update(p_pulse, time, false, false);
    time += MOTOR_RELEASE_US;
    (void)motor_pulse_update(p_pulse, time, false, false);
    time++;
    (void)motor_pulse_update(p_pulse, time, true, false);
    time += MOTOR_PRESS_US;
    (void)motor_pulse_update(p_pulse, time, true, false);
    return time;
}

int main (void)
{
    motor_pulse_t pulse;
    uint32_t time;
    unsigned int i;

    motor_pulse_init(NULL);
    check(motor_pulse_update(NULL, 0U, true, false) == MOTOR_PULSE_OFF,
          "null policy cannot request motion");

    motor_pulse_init(&pulse);
    check(motor_pulse_update(&pulse, 0U, true, false) == MOTOR_PULSE_OFF,
          "held start at boot does not run");
    check(motor_pulse_update(&pulse, UINT32_C(1000000), true, false) ==
          MOTOR_PULSE_OFF, "long-held boot button does not run");
    time = trigger(&pulse, UINT32_C(1000001));
    check(pulse.active == MOTOR_PULSE_M1A, "fresh released/pressed button runs M1A");
    check(motor_pulse_update(&pulse, time + MOTOR_PULSE_US - 1U, true, false) ==
          MOTOR_PULSE_M1A, "pulse remains active before deadline");
    check(motor_pulse_update(&pulse, time + MOTOR_PULSE_US, true, false) ==
          MOTOR_PULSE_OFF, "deadline requests brake");
    check(motor_pulse_update(&pulse, time + UINT32_C(3000000), true, false) ==
          MOTOR_PULSE_OFF, "holding button never advances after timeout");
    time = trigger(&pulse, time + UINT32_C(3000001));
    check(pulse.active == MOTOR_PULSE_M1B, "separate press selects M1B");
    check(motor_pulse_update(&pulse, time + 1U, true, true) == MOTOR_PULSE_OFF,
          "stop takes priority during motion");
    check(pulse.stopped, "stop is latched");
    time = trigger(&pulse, time + UINT32_C(1000000));
    check(pulse.active == MOTOR_PULSE_OFF, "fresh start cannot clear stop latch");

    motor_pulse_init(&pulse);
    time = 0U;
    for (i = 0U; i < 4U; i++)
    {
        time = trigger(&pulse, time);
        check((unsigned int)pulse.active == i + 1U, "four separately selected phases");
        time += MOTOR_PULSE_US;
        check(motor_pulse_update(&pulse, time, false, false) == MOTOR_PULSE_OFF,
              "every phase terminates");
        time++;
    }
    time = trigger(&pulse, time);
    check(pulse.next == 4U && pulse.active == MOTOR_PULSE_OFF,
          "finished sequence never repeats automatically");

    motor_pulse_init(&pulse);
    time = trigger(&pulse, UINT32_MAX - UINT32_C(60000));
    check(pulse.active == MOTOR_PULSE_M1A, "button timing tolerates clock wrap");
    motor_pulse_init(&pulse);
    time = trigger(&pulse, UINT32_MAX - UINT32_C(470001));
    check(motor_pulse_update(&pulse, time + MOTOR_PULSE_US - 1U, false, false) ==
          MOTOR_PULSE_M1A, "running pulse remains active across clock wrap");
    check(motor_pulse_update(&pulse, time + MOTOR_PULSE_US, false, false) ==
          MOTOR_PULSE_OFF, "pulse deadline tolerates clock wrap");

    motor_pulse_init(&pulse);
    (void)motor_pulse_update(&pulse, 0U, false, false);
    (void)motor_pulse_update(&pulse, MOTOR_RELEASE_US - 1U, false, false);
    (void)motor_pulse_update(&pulse, MOTOR_RELEASE_US, true, false);
    check(pulse.active == MOTOR_PULSE_OFF, "insufficient release does not arm");
    time = trigger(&pulse, UINT32_C(100000));
    (void)motor_pulse_update(&pulse, time + 1U, false, false);
    (void)motor_pulse_update(&pulse, time + MOTOR_RELEASE_US + 1U, false, false);
    (void)motor_pulse_update(&pulse, time + MOTOR_RELEASE_US + 2U, true, false);
    (void)motor_pulse_update(&pulse, time + MOTOR_PULSE_US, true, false);
    check(pulse.active == MOTOR_PULSE_OFF && pulse.next == 1U,
          "press during motion is not queued as another pulse");

    motor_pulse_init(&pulse);
    time = trigger(&pulse, 0U);
    (void)motor_pulse_update(&pulse, time + 1U, false, true);
    check(pulse.stopped && pulse.active == MOTOR_PULSE_OFF,
          "stop works with start released");

    return failures == 0U ? 0 : 1;
}
