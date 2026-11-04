/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	test_faults.c
 *	Host test for the shared 16-bit fault register.
 *
 *	The kernel atomics are replaced by link-time stubs with the same
 *	signatures.  A single mutex stands in for SIO Spinlock 1 and the interrupt
 *	mask, so the threaded test exercises real concurrent read-modify-write.
 *
 *	Run from the repository root:
 *	  cc -std=c11 -Wall -Wextra -Werror -Iinclude -Itests/shim \
 *	     tests/test_faults.c src/platform/faults.c -pthread \
 *	     -o /tmp/test_faults && /tmp/test_faults
 */

#define _POSIX_C_SOURCE 200809L

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

#include <tk/tkernel.h>
#include <tk/smp.h>

#include "platform/faults.h"

#define STRESS_ITERATIONS   20000

static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
static int s_failures = 0;

/* Host stubs for the kernel atomics.  Each returns the word's value before the update. */
UW atomic_bitset(UW *addr, UW setptn)
{
    UW old;

    (void)pthread_mutex_lock(&s_lock);
    old = *addr;
    *addr = old | setptn;
    (void)pthread_mutex_unlock(&s_lock);

    return old;
}

UW atomic_bitclr(UW *addr, UW clrptn)
{
    UW old;

    (void)pthread_mutex_lock(&s_lock);
    old = *addr;
    *addr = old & ~clrptn;
    (void)pthread_mutex_unlock(&s_lock);

    return old;
}

static void expect_u16(const char *what, uint16_t got, uint16_t want)
{
    if (got != want) {
        (void)fprintf(stderr, "FAIL %s: got 0x%04X, want 0x%04X\n",
                      what, (unsigned)got, (unsigned)want);
        s_failures++;
    }
}

/* Clear every defined bit so each test starts from an empty register. */
static void reset_faults(void)
{
    system_fault_clear(FAULT_ALL_MASK);
    expect_u16("reset", system_fault_get(), 0U);
}

static void test_round_trip(void)
{
    reset_faults();
    system_fault_set(FAULT_MOTOR_STALL_BIT);
    expect_u16("round trip set", system_fault_get(), 0x0004U);
    system_fault_clear(FAULT_MOTOR_STALL_BIT);
    expect_u16("round trip clear", system_fault_get(), 0x0000U);
}

static void test_idempotent_set(void)
{
    reset_faults();
    system_fault_set(FAULT_WIFI_DISCONN_BIT);
    system_fault_set(FAULT_WIFI_DISCONN_BIT);
    expect_u16("idempotent set", system_fault_get(), 0x0001U);
}

static void test_clear_unset_bit(void)
{
    reset_faults();
    system_fault_set(FAULT_MQTT_NACK_BIT);
    system_fault_clear(FAULT_IMU_NACK_BIT);
    expect_u16("clear unset bit", system_fault_get(), 0x0002U);
}

static void test_high_bits_masked(void)
{
    reset_faults();
    system_fault_set(0xFFFFU);
    expect_u16("set 0xFFFF reads 0x01FF", system_fault_get(), 0x01FFU);
    system_fault_clear(0xFFFFU);
    expect_u16("clear 0xFFFF empties", system_fault_get(), 0x0000U);

    reset_faults();
    system_fault_set(0x8000U);
    expect_u16("set 0x8000 latches nothing", system_fault_get(), 0x0000U);

    system_fault_set(FAULT_ENCODER_DEAD_BIT);
    system_fault_clear(0x8000U);
    expect_u16("clear 0x8000 keeps defined bits", system_fault_get(), 0x0008U);
}

static void test_multi_bit(void)
{
    reset_faults();
    system_fault_set(FAULT_WIFI_DISCONN_BIT | FAULT_LINE_LOST_BIT | FAULT_SONAR_NO_ECHO_BIT);
    expect_u16("multi-bit set", system_fault_get(), 0x0111U);
    system_fault_clear(FAULT_LINE_LOST_BIT);
    expect_u16("multi-bit partial clear", system_fault_get(), 0x0101U);
    system_fault_set(0x00FFU);
    expect_u16("multi-bit merge", system_fault_get(), 0x01FFU);
}

/* Each worker owns a disjoint bit set and is the only writer of those bits. */
struct stress_arg {
    uint16_t own;
    int failures;
};

static void *stress_worker(void *arg)
{
    struct stress_arg *sa = (struct stress_arg *)arg;
    int i;

    for (i = 0; i < STRESS_ITERATIONS; i++) {
        system_fault_set(sa->own);
        if ((system_fault_get() & sa->own) != sa->own) {
            sa->failures++;
        }
        system_fault_clear(sa->own);
        if ((system_fault_get() & sa->own) != 0U) {
            sa->failures++;
        }
    }
    system_fault_set(sa->own);

    return NULL;
}

static void *same_bit_set_worker(void *arg)
{
    struct stress_arg *sa = (struct stress_arg *)arg;
    int i;

    for (i = 0; i < STRESS_ITERATIONS; i++) {
        system_fault_set(sa->own);
    }

    return NULL;
}

static void test_same_bit_set(void)
{
    struct stress_arg a = { FAULT_MOTOR_STALL_BIT, 0 };
    struct stress_arg b = { FAULT_MOTOR_STALL_BIT, 0 };
    pthread_t ta;
    pthread_t tb;

    reset_faults();
    if (pthread_create(&ta, NULL, same_bit_set_worker, &a) != 0
        || pthread_create(&tb, NULL, same_bit_set_worker, &b) != 0) {
        (void)fprintf(stderr, "FAIL same-bit set: pthread_create\n");
        s_failures++;
        return;
    }
    (void)pthread_join(ta, NULL);
    (void)pthread_join(tb, NULL);
    expect_u16("same-bit set", system_fault_get(), FAULT_MOTOR_STALL_BIT);
}

static void test_threaded_disjoint(void)
{
    struct stress_arg a = { FAULT_WIFI_DISCONN_BIT | FAULT_MQTT_NACK_BIT
                            | FAULT_MOTOR_STALL_BIT | FAULT_ENCODER_DEAD_BIT, 0 };
    struct stress_arg b = { FAULT_LINE_LOST_BIT | FAULT_IMU_NACK_BIT
                            | FAULT_STALE_SPEED_BIT | FAULT_INVALID_TILT_BIT
                            | FAULT_SONAR_NO_ECHO_BIT, 0 };
    pthread_t ta;
    pthread_t tb;

    reset_faults();
    if (pthread_create(&ta, NULL, stress_worker, &a) != 0
        || pthread_create(&tb, NULL, stress_worker, &b) != 0) {
        (void)fprintf(stderr, "FAIL threaded: pthread_create\n");
        s_failures++;
        return;
    }
    (void)pthread_join(ta, NULL);
    (void)pthread_join(tb, NULL);

    if (a.failures != 0 || b.failures != 0) {
        (void)fprintf(stderr, "FAIL threaded: per-thread mismatches %d / %d\n",
                      a.failures, b.failures);
        s_failures++;
    }
    expect_u16("threaded final state", system_fault_get(), 0x01FFU);
}

int main(void)
{
    test_round_trip();
    test_idempotent_set();
    test_clear_unset_bit();
    test_high_bits_masked();
    test_multi_bit();
    test_same_bit_set();
    test_threaded_disjoint();

    if (s_failures != 0) {
        (void)fprintf(stderr, "test_faults: %d failure(s)\n", s_failures);
        return 1;
    }
    (void)printf("test_faults: all checks passed\n");
    return 0;
}
