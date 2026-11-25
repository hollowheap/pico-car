/** @file faults_test.c
 * @brief Host test for the shared fault register: the level word, the sticky
 *        word and the bit owners, against the single-threaded kernel shim.
 */
#include <stdint.h>
#include <stdio.h>

#include <tk/tkernel.h>

#include "platform/faults.h"
#include "comms/telemetry/wire_schema.h"

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

#define ALL FAULT_ALL_MASK

/* Clear both words through the owners, as nothing else may write them. */
static void reset_words (void)
{
    uint8_t owner = 0U;

    for (owner = 0U; owner < (uint8_t)FAULT_OWNER_COUNT; owner++)
    {
        system_fault_assign((fault_owner_t)owner, ALL, 0U);
    }
    system_fault_sticky_clear(ALL);
}

/* Every bit has exactly one owner. */
static void test_owners_cover_every_bit_once (void)
{
    uint32_t seen  = 0U;
    uint8_t  owner = 0U;

    for (owner = 0U; owner < (uint8_t)FAULT_OWNER_COUNT; owner++)
    {
        uint32_t const mask = system_fault_owner_mask((fault_owner_t)owner);

        CHECK(0U != mask);
        CHECK(0U == (seen & mask));
        seen |= mask;
    }
    CHECK(seen == FAULT_ALL_MASK);
    CHECK(0U == system_fault_owner_mask(FAULT_OWNER_COUNT));
}

static void test_level_word (void)
{
    reset_words();
    CHECK(system_fault_get() == 0U);

    system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, FAULT_LINE_LOST_BIT);
    CHECK(system_fault_get() == FAULT_LINE_LOST_BIT);
    system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_LIFTED_BIT, FAULT_LIFTED_BIT);
    system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_ABNORMAL_RESET_BIT,
                        FAULT_ABNORMAL_RESET_BIT);
    system_fault_assign(FAULT_OWNER_PHYSICS, FAULT_LIFTED_BIT, 0U);
    CHECK(system_fault_get() ==
          (FAULT_LINE_LOST_BIT | FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT));

    system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, 0U);
    CHECK(system_fault_get() == (FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT));
    /* Clearing a clear bit changes nothing. */
    system_fault_assign(FAULT_OWNER_LINE, FAULT_LINE_LOST_BIT, 0U);
    CHECK(system_fault_get() == (FAULT_LIFTED_BIT | FAULT_ABNORMAL_RESET_BIT));

    reset_words();
    CHECK(system_fault_get() == 0U);
}

/* A fault that came and went stays in the sticky word until it is cleared. */
static void test_sticky_word (void)
{
    reset_words();
    CHECK(system_fault_sticky_get() == 0U);

    system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT,
                        FAULT_TASK_STALLED_BIT);
    system_fault_assign(FAULT_OWNER_SUPERVISOR, FAULT_TASK_STALLED_BIT, 0U);
    CHECK(system_fault_get() == 0U);
    CHECK(system_fault_sticky_get() == FAULT_TASK_STALLED_BIT);

    system_fault_assign(FAULT_OWNER_PHYSICS, FAULT_OVERTURNED_BIT, FAULT_OVERTURNED_BIT);
    CHECK(system_fault_sticky_get() ==
          (FAULT_TASK_STALLED_BIT | FAULT_OVERTURNED_BIT));
    /* A clear keeps the bits that are still active: they are asserted now. */
    system_fault_sticky_clear(ALL);
    CHECK(system_fault_sticky_get() == FAULT_OVERTURNED_BIT);
    /* Clearing the sticky word never touches the level word. */
    CHECK(system_fault_get() == FAULT_OVERTURNED_BIT);

    reset_words();
    CHECK(system_fault_get() == 0U);
    CHECK(system_fault_sticky_get() == 0U);
}

/* An owner changes only its own bits in its mask; every other bit stays. */
static void test_assign_touches_only_owned_bits (void)
{
    uint32_t const mine = FAULT_MOTOR_STALL_BIT | FAULT_ENCODER_DEAD_BIT;

    reset_words();
    system_fault_assign(FAULT_OWNER_LINE, ALL, FAULT_LINE_LOST_BIT);
    system_fault_assign(FAULT_OWNER_MOTION, ALL, FAULT_ENCODER_DEAD_BIT);
    system_fault_assign(FAULT_OWNER_MOTION, mine, FAULT_MOTOR_STALL_BIT);
    CHECK(system_fault_get() == (FAULT_LINE_LOST_BIT | FAULT_MOTOR_STALL_BIT));
    /* A bit outside the mask in "active" is ignored. */
    system_fault_assign(FAULT_OWNER_MOTION, mine, FAULT_LIFTED_BIT);
    CHECK(system_fault_get() == FAULT_LINE_LOST_BIT);
    CHECK(system_fault_sticky_get() ==
          (FAULT_LINE_LOST_BIT | FAULT_ENCODER_DEAD_BIT | FAULT_MOTOR_STALL_BIT));

    /* Another owner's bits, even in the mask, are not touched. */
    system_fault_assign(FAULT_OWNER_MOTION, ALL, ALL);
    CHECK(system_fault_get() == (FAULT_LINE_LOST_BIT | mine | FAULT_ESTOP_LATCHED_BIT));
    system_fault_assign(FAULT_OWNER_OBSTACLE, ALL, 0U);
    CHECK(0U != (system_fault_get() & FAULT_LINE_LOST_BIT));
    /* An unknown owner owns nothing. */
    system_fault_assign(FAULT_OWNER_COUNT, ALL, 0U);
    CHECK(system_fault_get() == (FAULT_LINE_LOST_BIT | mine | FAULT_ESTOP_LATCHED_BIT));

    reset_words();
}

/* The word is 32 bits wide; bits not yet defined never appear. */
static void test_undefined_bits_ignored (void)
{
    reset_words();
    system_fault_assign(FAULT_OWNER_SUPERVISOR, 0xFFFFFFFFU, 0xFFFFFFFFU);
    CHECK(system_fault_get() == (FAULT_LIFTED_BIT | FAULT_TASK_STALLED_BIT | FAULT_ABNORMAL_RESET_BIT));
    CHECK(0U == (system_fault_sticky_get() & ~FAULT_ALL_MASK));
    reset_words();
}

typedef struct
{
    uint32_t changed;
    uint32_t level;
} fault_change_t;

#define EXPECTED_CHANGES (6U)

static fault_change_t g_changes[EXPECTED_CHANGES];
static unsigned g_change_count;
static UINT g_sysstat;

ER tk_ref_sys (T_RSYS * p_state)
{
    *p_state = (T_RSYS){g_sysstat, 1, 1};
    return E_OK;
}

static void record_change (uint32_t changed, uint32_t level)
{
    if (g_change_count < EXPECTED_CHANGES)
    {
        g_changes[g_change_count] = (fault_change_t){changed, level};
    }
    g_change_count++;
}

static void test_observer_captures_every_level_transition (void)
{
    uint32_t const motion = FAULT_ENCODER_DEAD_BIT | FAULT_ESTOP_LATCHED_BIT;
    uint32_t const line   = FAULT_LINE_LOST_BIT;
    uint32_t const stall  = FAULT_MOTOR_STALL_BIT;

    reset_words();
    g_change_count = 0U;
    system_fault_bind_observer(record_change);

    /* This complete pulse can occur in between two telemetry snapshots. */
    system_fault_assign(FAULT_OWNER_LINE, line, line);
    system_fault_assign(FAULT_OWNER_LINE, line, line);
    system_fault_assign(FAULT_OWNER_MOTION, line, 0U);
    system_fault_assign(FAULT_OWNER_LINE, line, 0U);
    system_fault_assign(FAULT_OWNER_LINE, line, 0U);
    CHECK(system_fault_get() == 0U);
    CHECK(system_fault_sticky_get() == line);
    CHECK(g_change_count == 2U);

    system_fault_assign(FAULT_OWNER_MOTION, motion, motion);
    system_fault_assign(FAULT_OWNER_LINE, line, line);
    system_fault_assign(FAULT_OWNER_MOTION, motion | stall, stall);
    system_fault_sticky_clear(ALL);
    system_fault_bind_observer(NULL);

    CHECK(g_change_count == EXPECTED_CHANGES);
    CHECK(g_changes[0].changed == line && g_changes[0].level == line);
    CHECK(g_changes[1].changed == line && g_changes[1].level == 0U);
    CHECK(g_changes[2].changed == motion && g_changes[2].level == motion);
    CHECK(g_changes[3].changed == line && g_changes[3].level == (motion | line));
    CHECK(g_changes[4].changed == stall && g_changes[4].level == (motion | line | stall));
    CHECK(g_changes[5].changed == motion && g_changes[5].level == (line | stall));
    CHECK(system_fault_get() == (line | stall));
    CHECK(system_fault_sticky_get() == (line | stall));

    reset_words();
    CHECK(g_change_count == EXPECTED_CHANGES);
}

static void test_observer_requires_dispatchable_task_context (void)
{
    uint32_t const line = FAULT_LINE_LOST_BIT;

    reset_words();
    g_change_count = 0U;
    system_fault_bind_observer(record_change);

    g_sysstat = TSS_INDP;
    system_fault_assign(FAULT_OWNER_LINE, line, line);
    CHECK(system_fault_get() == line);
    CHECK(system_fault_sticky_get() == line);
    g_sysstat = TSS_DINT;
    system_fault_assign(FAULT_OWNER_LINE, line, 0U);
    g_sysstat = TSS_DDSP;
    system_fault_assign(FAULT_OWNER_LINE, line, line);
    CHECK(g_change_count == 0U);

    g_sysstat = 0U;
    system_fault_assign(FAULT_OWNER_LINE, line, 0U);
    CHECK(g_change_count == 1U);
    CHECK(g_changes[0].changed == line && g_changes[0].level == 0U);
    system_fault_bind_observer(NULL);
    reset_words();
}

int main (void)
{
    test_bits_match_the_schema();
    test_owners_cover_every_bit_once();
    test_level_word();
    test_sticky_word();
    test_assign_touches_only_owned_bits();
    test_undefined_bits_ignored();
    test_observer_captures_every_level_transition();
    test_observer_requires_dispatchable_task_context();

    if (g_fail != 0U)
    {
        (void)printf("%u check(s) failed\n", g_fail);
        return 1;
    }
    (void)printf("test_faults: all checks passed\n");
    return 0;
}
