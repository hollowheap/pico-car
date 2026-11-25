/** @file test_pilot_drive.c
 * @brief Host tests for the pilot drive layer.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pilot/drive.h"
#include "pilot/pilot.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define STEP_MS     10U
#define SETTLE_RUNS 200
#define TURN_90_DPS 900

/* Wheel speeds of the rise test: a car driving, a one-shot turn at its fast
 * and slow pivot speeds, and a reversed search pivot. */
#define RISE_DRIVE_UM_S   (150 * UM_PER_MM)
#define RISE_REVERSE_UM_S LINE_SEARCH_UM_S
#define RISE_PASSES_MAX   (100)
#define RISE_HOLD_PASSES  (30)
#define TURN_SETTLE_PASSES (20)

static void step(drive_state_t * p_d, int16_t speed, int16_t turn, uint32_t dt,
                 int32_t * p_l, int32_t * p_r)
{
    drive_step(p_d, speed, turn, dt, p_l, p_r);
}

/* Run until the targets stop changing. */
static void settle(drive_state_t * p_d, int16_t speed, int16_t turn,
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

static void test_forward_radius(void)
{
    /* With a stopped inner wheel the axle traces half the 107 mm track.
     * Round the predicted footprint outward in either direction. */
    assert(drive_forward_radius_mm(0, 100000) == 54);
    assert(drive_forward_radius_mm(100000, 0) == -54);
    assert(drive_forward_radius_mm(20000, 100000) == 81);
    assert(drive_forward_radius_mm(100000, 20000) == -81);
    assert(drive_forward_radius_mm(50000, 50000) == 0);
    assert(drive_forward_radius_mm(50000, 50001) == 0);
    assert(drive_forward_radius_mm(-20000, 100000) == 0);
    assert(drive_forward_radius_mm(0, 20000) == 0);
    assert(drive_forward_radius_mm(0, INT32_MAX) == 0);
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

/* A safety stop at 300 mm/s: the line task resets the drive, so the next step
 * commands 0 at once. Through the slew alone the stop would take 0.6 s. */
static void test_safety_stop_skips_slew(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;

    drive_init(&d);
    settle(&d, 300, 0, &l, &r);
    assert(l == 300 * UM_PER_MM && r == 300 * UM_PER_MM);
    step(&d, 0, 0, STEP_MS, &l, &r);
    assert(l == (300 * UM_PER_MM) - (DRIVE_ACCEL_MM_S2 * (int32_t)STEP_MS));

    settle(&d, 300, 0, &l, &r);
    drive_safety_stop(&d, &l, &r);
    assert(l == 0 && r == 0);
    step(&d, 0, 0, STEP_MS, &l, &r);
    assert(l == 0 && r == 0);
    /* The drive starts again from rest. */
    step(&d, 100, 0, STEP_MS, &l, &r);
    assert(l == DRIVE_START_MM_S * UM_PER_MM);
}

/* The line follower's command reaches the wheels in one pass: 300 mm/s to a
 * pivot about the inner wheel. Through the slew the inner wheel would need
 * 0.6 s. The next slewed step starts from these targets. */
static void test_follow_skips_slew(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;
    int32_t r_follow;
    /* Half the wheel difference is 150 mm/s on the 107 mm track: 1606 ddeg/s. */
    int16_t const turn = 1606;

    drive_init(&d);
    settle(&d, 300, 0, &l, &r);
    drive_follow(&d, 150, turn, STEP_MS, &l, &r);
    assert(l < 2 * UM_PER_MM && l > -2 * UM_PER_MM);
    assert(r > 298 * UM_PER_MM && r <= DRIVE_MAX_TARGET_UM_S);
    r_follow = r;
    step(&d, 150, 0, STEP_MS, &l, &r);
    assert(r == r_follow - (DRIVE_ACCEL_MM_S2 * (int32_t)STEP_MS));
}

/* The follower asks for cruise as it starts: at boot, after a hug, after a
 * back-off and after a hold. The forward speed rises at the slew limit, so the
 * wheels do not spin. The turn of a spot search rises the same way from the
 * start speed, and slows at once. */
static void test_follow_start_ramps(void)
{
    drive_state_t d;
    int32_t l;
    int32_t r;
    int32_t const rise = DRIVE_ACCEL_MM_S2 * (int32_t)STEP_MS;

    drive_init(&d);
    drive_follow(&d, 300, 0, STEP_MS, &l, &r);
    assert(l == rise && r == rise);
    drive_follow(&d, 300, 0, STEP_MS, &l, &r);
    assert(l == 2 * rise && r == 2 * rise);

    /* From the pilot's back-off at -60 mm/s. */
    drive_init(&d);
    settle(&d, -60, 0, &l, &r);
    assert(l == -60 * UM_PER_MM);
    drive_follow(&d, 300, 0, STEP_MS, &l, &r);
    assert(l == rise && r == rise);

    drive_init(&d);
    drive_follow(&d, 0, TURN_90_DPS, STEP_MS, &l, &r);
    assert(l == -DRIVE_START_MM_S * UM_PER_MM && r == DRIVE_START_MM_S * UM_PER_MM);
    drive_follow(&d, 0, TURN_90_DPS, STEP_MS, &l, &r);
    assert(r == (DRIVE_START_MM_S * UM_PER_MM) + rise && l == -r);
    for (int i = 0; i < TURN_SETTLE_PASSES; i++)
    {
        drive_follow(&d, 0, TURN_90_DPS, STEP_MS, &l, &r);
    }
    assert(l < -80 * UM_PER_MM && r > 80 * UM_PER_MM && l == -r);
    int32_t const full = r;
    drive_follow(&d, 0, TURN_90_DPS / 2, STEP_MS, &l, &r);
    assert(r <= (full / 2) + 1 && r >= (full / 2) - 1 && l == -r);
}

/* Turn targets speed up at the slew limit and slow at once, so a pivot does
 * not spin the wheels on paper and still stops on the tape. A pivot entered
 * while driving stays mirrored; a reversed turn starts again from the start
 * speed. */
static void test_rise_targets(void)
{
    drive_state_t d = {RISE_DRIVE_UM_S, RISE_DRIVE_UM_S};
    int32_t const rise = DRIVE_ACCEL_MM_S2 * (int32_t)STEP_MS;
    int32_t const start = DRIVE_START_MM_S * UM_PER_MM;
    int32_t l;
    int32_t r;
    int32_t last_r = 0;
    int i;

    drive_rise(&d, -LINE_TURN_FAST_UM_S, LINE_TURN_FAST_UM_S, STEP_MS, &l, &r);
    assert(l == -start && r == start);
    for (i = 0; (i < RISE_PASSES_MAX) && (r < LINE_TURN_FAST_UM_S); i++)
    {
        last_r = r;
        drive_rise(&d, -LINE_TURN_FAST_UM_S, LINE_TURN_FAST_UM_S, STEP_MS, &l, &r);
        assert(r - last_r <= rise && l == -r);
    }
    for (i = 0; i < RISE_HOLD_PASSES; i++)
    {
        drive_rise(&d, -LINE_TURN_FAST_UM_S, LINE_TURN_FAST_UM_S, STEP_MS, &l, &r);
    }
    assert(l == -LINE_TURN_FAST_UM_S && r == LINE_TURN_FAST_UM_S);
    drive_rise(&d, -LINE_TURN_SLOW_UM_S, LINE_TURN_SLOW_UM_S, STEP_MS, &l, &r);
    assert(l == -LINE_TURN_SLOW_UM_S && r == LINE_TURN_SLOW_UM_S);
    drive_rise(&d, RISE_REVERSE_UM_S, -RISE_REVERSE_UM_S, STEP_MS, &l, &r);
    assert(l == start && r == -start);
    drive_rise(&d, 0, 0, STEP_MS, &l, &r);
    assert(l == 0 && r == 0 && d.left_um_s == 0 && d.right_um_s == 0);
}

/* Both mirrored searches can pick up the opposite sensor while the encoders
 * still report the old pivot. A forward command cannot reverse either wheel. */
static void test_follow_acquisition_after_pivot(void)
{
    for (int32_t side = -1; side <= 1; side += 2)
    {
        pilot_state_t pilot;
        pilot_meas_t meas = {.b_bounded_course = true, .b_line_left = (side > 0), .b_line_right = (side < 0),
                             .ping_echo = LOOK_ECHO_OPEN};
        drive_state_t drive;
        pilot_cmd_t cmd;
        int32_t left;
        int32_t right;

        pilot_init(&pilot);
        drive_init(&drive);
        pilot_step(&pilot, &meas, &cmd);
        meas.b_line_left = false;
        meas.b_line_right = false;
        for (int32_t mm = 1; mm <= LINE_GAP_MM; mm++)
        {
            meas.left_um = (int64_t)mm * UM_PER_MM;
            meas.right_um = meas.left_um;
            meas.now_ms += STEP_MS;
            meas.stamp_ms = meas.now_ms;
            meas.ping_seq = (uint8_t)(mm / 5);
            pilot_step(&pilot, &meas, &cmd);
            pilot_drive(&drive, &cmd, STEP_MS, &left, &right);
        }
        assert(LINE_FOLLOW_SEARCHING == pilot.follow.status);
        assert(left == -right && side * right > 0);

        for (unsigned pass = 0U; pass < 5U; pass++)
        {
            int32_t const coast_um = (side * LINE_SEARCH_UM_S * (int32_t)STEP_MS) / 1000;

            meas.left_um -= coast_um;
            meas.right_um += coast_um;
            meas.now_ms += STEP_MS;
            meas.stamp_ms = meas.now_ms;
            meas.b_line_left = (side < 0);
            meas.b_line_right = (side > 0);
            pilot_step(&pilot, &meas, &cmd);
            assert(LINE_FOLLOW_FOLLOWING == pilot.follow.status);
            assert(cmd.behaviour == PILOT_BEHAVIOUR_FOLLOW_LINE && cmd.speed_mm_s > 0);
            assert(side * cmd.turn_deci_s < 0);
            pilot_drive(&drive, &cmd, STEP_MS, &left, &right);
            assert(left >= 0 && right >= 0);
            assert(side * (left - right) > 0);
        }
    }
}

/* A scan pause and a sonar fault each hold both targets at zero. */
static void test_pilot_stationary_targets(void)
{
    drive_state_t d;
    pilot_cmd_t   cmd = { 0 };
    int32_t       l;
    int32_t       r;

    drive_init(&d);
    settle(&d, 120, 0, &l, &r);
    cmd.behaviour = (uint8_t)PILOT_BEHAVIOUR_HUG;
    cmd.reason    = (uint8_t)PILOT_REASON_NONE;
    pilot_drive(&d, &cmd, STEP_MS, &l, &r);
    assert(l == 0 && r == 0);
    pilot_drive(&d, &cmd, STEP_MS, &l, &r);
    assert(l == 0 && r == 0);
    cmd.turn_deci_s = HUG_PIVOT_MIN_DECI_S;
    pilot_drive(&d, &cmd, STEP_MS, &l, &r);
    assert(l < 0 && r > 0);

    settle(&d, 120, 0, &l, &r);
    cmd.turn_deci_s = 0;
    cmd.reason = (uint8_t)PILOT_REASON_SONAR;
    pilot_drive(&d, &cmd, STEP_MS, &l, &r);
    assert(l == 0 && r == 0);
}

/* A stuck car backs off from rest: its wheels do not turn, and a slew down
 * from 300 mm/s would push on the blockage for 0.6 s first. */
static void test_pilot_backoff_from_rest(void)
{
    drive_state_t d;
    pilot_cmd_t   cmd = { 0 };
    int32_t       l;
    int32_t       r;

    drive_init(&d);
    settle(&d, 300, 0, &l, &r);
    cmd.speed_mm_s = -PILOT_BACKOFF_MM_S;
    cmd.behaviour  = (uint8_t)PILOT_BEHAVIOUR_SAFETY;
    cmd.reason     = (uint8_t)PILOT_REASON_STUCK;
    pilot_drive(&d, &cmd, STEP_MS, &l, &r);
    assert(l == -DRIVE_START_MM_S * UM_PER_MM && r == l);
}

int main(int argc, char * argv[])
{
    if (2 == argc && 0 == strcmp("sonar-sets-zero", argv[1]))
    {
        test_pilot_stationary_targets();
        return EXIT_SUCCESS;
    }
    if (1 != argc)
    {
        (void)fprintf(stderr, "Usage: %s [sonar-sets-zero]\n", argv[0]);
        return EXIT_FAILURE;
    }
    test_straight();
    test_pure_turn();
    test_curve();
    test_forward_radius();
    test_accel_limit();
    test_start_kick();
    test_small_target_is_zero();
    test_sign_change_goes_through_zero();
    test_saturation_keeps_turn();
    test_dt_clamp();
    test_safety_stop_skips_slew();
    test_follow_skips_slew();
    test_follow_start_ramps();
    test_rise_targets();
    test_follow_acquisition_after_pivot();
    test_pilot_stationary_targets();
    test_pilot_backoff_from_rest();
    puts("test_pilot_drive: 17 tests passed");
    return 0;
}
