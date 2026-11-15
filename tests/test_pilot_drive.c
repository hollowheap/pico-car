/** @file test_pilot_drive.c
 * @brief Host tests for the pilot drive layer.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pilot/drive.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define UM_PER_MM   1000
#define STEP_MS     10U
#define SETTLE_RUNS 200
#define TURN_90_DPS 900

static void step(drive_state_t * p_d, int32_t speed, int32_t turn, uint32_t dt,
                 int32_t * p_l, int32_t * p_r)
{
    drive_step(p_d, speed, turn, dt, p_l, p_r);
}

/* Run until the targets stop changing. */
static void settle(drive_state_t * p_d, int32_t speed, int32_t turn,
                   int32_t * p_l, int32_t * p_r)
{
    int i;
    for (i = 0; i < SETTLE_RUNS; i++)
    {
        step(p_d, speed, turn, DRIVE_DT_MAX_MS, p_l, p_r);
    }
}

static void test_straight(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;

    drive_init(&d);
    settle(&d, 200, 0, &l, &r);
    assert(l == 200 * UM_PER_MM);
    assert(r == 200 * UM_PER_MM);
    settle(&d, -150, 0, &l, &r);
    assert(l == -150 * UM_PER_MM);
    assert(r == -150 * UM_PER_MM);
}

static void test_pure_turn(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;

    drive_init(&d);
    /* 900 deci/s = 1.5707 rad/s; half track 53.5 mm -> 84.1 mm/s */
    settle(&d, 0, TURN_90_DPS, &l, &r);
    assert(r > 0 && l < 0);
    assert(r == -l);
    assert(abs(r - 84000) < 300);
    settle(&d, 0, -TURN_90_DPS, &l, &r);
    assert(l > 0 && r < 0);
}

static void test_curve(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;

    drive_init(&d);
    settle(&d, 150, 300, &l, &r);
    assert(r > 150 * UM_PER_MM && l < 150 * UM_PER_MM);
    assert((l + r) / 2 == 150 * UM_PER_MM);
}

static void test_accel_limit(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;
    int32_t prev;
    int i;

    drive_init(&d);
    step(&d, 200, 0, STEP_MS, &l, &r);
    assert(l == DRIVE_START_MM_S * UM_PER_MM);
    prev = l;
    for (i = 0; i < 100 && l < 200 * UM_PER_MM; i++)
    {
        step(&d, 200, 0, STEP_MS, &l, &r);
        assert(l - prev <= DRIVE_ACCEL_MM_S2 * (int32_t)STEP_MS);
        assert(l > prev);
        prev = l;
    }
    assert(l == 200 * UM_PER_MM);
    /* Slowing is limited the same way. */
    step(&d, 100, 0, STEP_MS, &l, &r);
    assert(prev - l == DRIVE_ACCEL_MM_S2 * (int32_t)STEP_MS);
}

static void test_start_kick(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;

    drive_init(&d);
    step(&d, 0, TURN_90_DPS, STEP_MS, &l, &r);
    assert(r == DRIVE_START_MM_S * UM_PER_MM);
    assert(l == -DRIVE_START_MM_S * UM_PER_MM);

    /* A target under the kick speed still starts at the kick speed. */
    drive_init(&d);
    step(&d, 30, 0, STEP_MS, &l, &r);
    assert(l == DRIVE_START_MM_S * UM_PER_MM);
    drive_init(&d);
    step(&d, 100, 0, STEP_MS, &l, &r);
    assert(l == DRIVE_START_MM_S * UM_PER_MM);
}

static void test_small_target_is_zero(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;

    drive_init(&d);
    step(&d, DRIVE_STOP_MM_S - 1, 0, STEP_MS, &l, &r);
    assert(l == 0 && r == 0);
    settle(&d, 100, 0, &l, &r);
    settle(&d, DRIVE_STOP_MM_S - 1, 0, &l, &r);
    assert(l == 0 && r == 0);
}

static void test_sign_change_goes_through_zero(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;
    int32_t prev;
    int seen_zero = 0;
    int i;

    drive_init(&d);
    settle(&d, 100, 0, &l, &r);
    prev = l;
    for (i = 0; i < 100; i++)
    {
        step(&d, -100, 0, STEP_MS, &l, &r);
        assert(!(prev > 0 && l < 0));
        assert(!(prev < 0 && l > 0));
        if (l == 0)
        {
            seen_zero = 1;
        }
        prev = l;
    }
    assert(seen_zero);
    assert(l == -100 * UM_PER_MM);
}

static void test_saturation_keeps_turn(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;
    int32_t diff;

    drive_init(&d);
    settle(&d, 0, TURN_90_DPS, &l, &r);
    diff = r - l;
    /* Too fast to fit with this turn: speed gives way, turn stays. */
    drive_init(&d);
    settle(&d, 300, TURN_90_DPS, &l, &r);
    assert(r <= DRIVE_MAX_TARGET_UM_S && l <= DRIVE_MAX_TARGET_UM_S);
    assert(r == DRIVE_MAX_TARGET_UM_S);
    assert(abs((r - l) - diff) <= 2);

    drive_init(&d);
    settle(&d, -300, TURN_90_DPS, &l, &r);
    assert(l >= -DRIVE_MAX_TARGET_UM_S && l == -DRIVE_MAX_TARGET_UM_S);
    assert(abs((r - l) - diff) <= 2);

    /* The turn alone is over the limit: the turn scales, speed is zero. */
    drive_init(&d);
    settle(&d, 100, 10000, &l, &r);
    assert(r == DRIVE_MAX_TARGET_UM_S && l == -DRIVE_MAX_TARGET_UM_S);
}

static void test_dt_clamp(void)
{
    drive_state_t a;
    drive_state_t b;
    int32_t l1;
    int32_t r1;
    int32_t l2;
    int32_t r2;

    drive_init(&a);
    drive_init(&b);
    step(&a, 250, 0, 0U, &l1, &r1);
    step(&b, 250, 0, 0U, &l2, &r2);
    step(&a, 250, 0, 5000U, &l1, &r1);
    step(&b, 250, 0, DRIVE_DT_MAX_MS, &l2, &r2);
    assert(l1 == l2 && r1 == r2);
    assert(l1 == DRIVE_START_MM_S * UM_PER_MM
                     + DRIVE_ACCEL_MM_S2 * (int32_t)DRIVE_DT_MAX_MS);
}

int main(void)
{
    test_straight();
    test_pure_turn();
    test_curve();
    test_accel_limit();
    test_start_kick();
    test_small_target_is_zero();
    test_sign_change_goes_through_zero();
    test_saturation_keeps_turn();
    test_dt_clamp();
    puts("test_pilot_drive: 9 tests passed");
    return 0;
}
