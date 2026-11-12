/** @file test_faults.c
 * @brief Host test for the shared fault register (S02, S46): the level word
 *        and the sticky word, against the single-threaded kernel shim.
 */
#include <stdint.h>
#include <stdio.h>

#include "platform/faults.h"
#include "telemetry/wire_schema.h"

static unsigned g_fail;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            g_fail++;                                                        \
            (void)printf("FAIL line %d: %s\n", __LINE__, #cond);             \
        }                                                                    \
    } while (0)

/* The bit positions in faults.h and in protocol/telemetry.yaml are one list. */
static void test_bits_match_the_schema (void)
{
    CHECK(FAULT_WIFI_DISCONN_BIT == (1U << TELEMETRY_FAULT_BIT_WIFI_DISCONN));
    CHECK(FAULT_MQTT_NACK_BIT == (1U << TELEMETRY_FAULT_BIT_MQTT_NACK));
    CHECK(FAULT_MOTOR_STALL_BIT == (1U << TELEMETRY_FAULT_BIT_MOTOR_STALL));
    CHECK(FAULT_ENCODER_DEAD_BIT == (1U << TELEMETRY_FAULT_BIT_ENCODER_DEAD));
    CHECK(FAULT_LINE_LOST_BIT == (1U << TELEMETRY_FAULT_BIT_LINE_LOST));
    CHECK(FAULT_IMU_NACK_BIT == (1U << TELEMETRY_FAULT_BIT_IMU_NACK));
    CHECK(FAULT_STALE_SPEED_BIT == (1U << TELEMETRY_FAULT_BIT_STALE_SPEED));
    CHECK(FAULT_INVALID_TILT_BIT == (1U << TELEMETRY_FAULT_BIT_INVALID_TILT));
    CHECK(FAULT_SONAR_NO_ECHO_BIT == (1U << TELEMETRY_FAULT_BIT_SONAR_NO_ECHO));
    CHECK(FAULT_ESTOP_LATCHED_BIT == (1U << TELEMETRY_FAULT_BIT_ESTOP_LATCHED));
    CHECK(FAULT_ECHO_LINE_BIT == (1U << TELEMETRY_FAULT_BIT_ECHO_LINE));
    CHECK(FAULT_LIFTED_BIT == (1U << TELEMETRY_FAULT_BIT_LIFTED));
    CHECK(FAULT_OVERTURNED_BIT == (1U << TELEMETRY_FAULT_BIT_OVERTURNED));
    CHECK(FAULT_OBSTACLE_STOPPED_BIT ==
          (1U << TELEMETRY_FAULT_BIT_OBSTACLE_STOPPED));
    CHECK(FAULT_TASK_STALLED_BIT == (1U << TELEMETRY_FAULT_BIT_TASK_STALLED));
    CHECK(FAULT_ABNORMAL_RESET_BIT == (1U << TELEMETRY_FAULT_BIT_ABNORMAL_RESET));
    CHECK(FAULT_ALL_MASK == 0xFFFFU);
}

static void test_level_word (void)
{
    system_fault_clear(FAULT_ALL_MASK);
    CHECK(system_fault_get() == 0U);

    system_fault_set(FAULT_LINE_LOST_BIT);
    CHECK(system_fault_get() == FAULT_LINE_LOST_BIT);
    system_fault_set(FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT);
    CHECK(system_fault_get() ==
          (FAULT_LINE_LOST_BIT | FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT));
    /* Setting again keeps the bit. */
    system_fault_set(FAULT_LINE_LOST_BIT);
    CHECK((system_fault_get() & FAULT_LINE_LOST_BIT) != 0U);

    system_fault_clear(FAULT_LINE_LOST_BIT);
    CHECK(system_fault_get() == (FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT));
    /* Clearing a clear bit changes nothing. */
    system_fault_clear(FAULT_LINE_LOST_BIT);
    CHECK(system_fault_get() == (FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT));

    system_fault_clear(FAULT_ALL_MASK);
    CHECK(system_fault_get() == 0U);
}

/* A fault that came and went stays in the sticky word until it is cleared. */
static void test_sticky_word (void)
{
    system_fault_clear(FAULT_ALL_MASK);
    system_fault_sticky_clear(FAULT_ALL_MASK);
    CHECK(system_fault_sticky_get() == 0U);

    system_fault_set(FAULT_TASK_STALLED_BIT);
    system_fault_clear(FAULT_TASK_STALLED_BIT);
    CHECK(system_fault_get() == 0U);
    CHECK(system_fault_sticky_get() == FAULT_TASK_STALLED_BIT);

    system_fault_set(FAULT_OVERTURNED_BIT);
    CHECK(system_fault_sticky_get() ==
          (FAULT_TASK_STALLED_BIT | FAULT_OVERTURNED_BIT));
    system_fault_sticky_clear(FAULT_TASK_STALLED_BIT);
    CHECK(system_fault_sticky_get() == FAULT_OVERTURNED_BIT);
    /* Clearing the sticky word never touches the level word. */
    CHECK(system_fault_get() == FAULT_OVERTURNED_BIT);

    system_fault_clear(FAULT_ALL_MASK);
    system_fault_sticky_clear(FAULT_ALL_MASK);
    CHECK(system_fault_get() == 0U);
    CHECK(system_fault_sticky_get() == 0U);
}

/* An owner reports only its own bits; the other owners' bits stay as they are. */
static void test_assign_touches_only_the_mask (void)
{
    uint16_t const mine = FAULT_MOTOR_STALL_BIT | FAULT_ENCODER_DEAD_BIT;

    system_fault_set(FAULT_LINE_LOST_BIT | FAULT_ENCODER_DEAD_BIT);
    system_fault_assign(mine, FAULT_MOTOR_STALL_BIT);
    CHECK(system_fault_get() == (FAULT_LINE_LOST_BIT | FAULT_MOTOR_STALL_BIT));
    /* A bit outside the mask in "active" is ignored. */
    system_fault_assign(mine, FAULT_LIFTED_BIT);
    CHECK(system_fault_get() == FAULT_LINE_LOST_BIT);
    CHECK(system_fault_sticky_get() ==
          (FAULT_LINE_LOST_BIT | FAULT_ENCODER_DEAD_BIT | FAULT_MOTOR_STALL_BIT));

    system_fault_clear(FAULT_ALL_MASK);
    system_fault_sticky_clear(FAULT_ALL_MASK);
}

int main (void)
{
    test_bits_match_the_schema();
    test_level_word();
    test_sticky_word();
    test_assign_touches_only_the_mask();

    if (g_fail != 0U)
    {
        (void)printf("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf("test_faults: all checks passed\n");
    return 0;
}
