/** @file drive.c
 * @brief Pure drive layer: forward speed and turn rate to wheel targets.
 *
 * Integer math only. The state holds the last wheel targets. Each pass limits
 * the change of a wheel target, lifts a wheel that starts from rest to the
 * start speed, and sends a wheel that must reverse through zero first.
 */

#include "motion/pilot/drive.h"

#include <stdbool.h>

#include "platform/chassis.h"
#include "platform/units.h"

#define HALF_DIVISOR    (2)

#define START_UM_S      (DRIVE_START_MM_S * UM_PER_MM)
#define STOP_UM_S       (DRIVE_STOP_MM_S * UM_PER_MM)

/* Half the track times the turn rate, in um/s per deci-degree/s, as a
 * numerator and denominator so no precision is lost. */
#define TURN_NUM        ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
#define TURN_DEN        ((int64_t)DECI_PER_RAD * HALF_DIVISOR)

/* Starting values for the uphill trim. The deadband keeps floor and IMU
 * noise from turning ordinary left-right speed variation into a correction. */
#define ROLL_DEADBAND_CDEG     (200)
#define ROLL_GAIN_UM_S_PER_CDEG (20)
#define ROLL_MAX_TRIM_UM_S      (12000)

int16_t drive_forward_radius_mm(int32_t left_um_s, int32_t right_um_s)
{
    if ((left_um_s < 0) || (right_um_s < 0) || (left_um_s == right_um_s) ||
        (left_um_s > DRIVE_MAX_TARGET_UM_S) || (right_um_s > DRIVE_MAX_TARGET_UM_S) ||
        ((left_um_s < START_UM_S) && (right_um_s < START_UM_S)))
    {
        return 0;
    }
    int32_t const diff = right_um_s - left_um_s;
    int32_t const den = HALF_DIVISOR * ((diff < 0) ? -diff : diff);
    int32_t const radius = ((left_um_s + right_um_s) * CHASSIS_WHEEL_TRACK_MM + den - 1) / den;
    return (radius > INT16_MAX) ? 0 : (int16_t)((diff < 0) ? -radius : radius);
}

/**
 * Next target of one wheel.
 *
 * @param prev_um_s Last target of the wheel.
 * @param want_um_s Wanted target.
 * @param step_um_s Largest change of the target in this pass.
 */
static int32_t wheel_next(int32_t prev_um_s, int32_t want_um_s,
                          int32_t step_um_s)
{
    int32_t want = want_um_s;
    int32_t next = 0;

    if (((prev_um_s > 0) && (want < 0)) || ((prev_um_s < 0) && (want > 0)))
    {
        want = 0;
    }

    if ((want > -STOP_UM_S) && (want < STOP_UM_S) && (prev_um_s == 0))
    {
        next = 0;
    }
    else if (prev_um_s == 0)
    {
        next = clamp_i32(want, -step_um_s, step_um_s);
        if (want > 0)
        {
            next = (next < START_UM_S) ? START_UM_S : next;
        }
        else
        {
            next = (next > -START_UM_S) ? -START_UM_S : next;
        }
    }
    else
    {
        next = clamp_i32(want, prev_um_s - step_um_s, prev_um_s + step_um_s);
        if ((next > -STOP_UM_S) && (next < STOP_UM_S))
        {
            next = 0;
        }
    }

    return next;
}

/* Speed up by at most step_um_s from a value of the same sign, else from
 * rest with the start speed; slow at once. */
static int32_t wheel_rise(int32_t prev_um_s, int32_t want_um_s, int32_t step_um_s)
{
    bool const b_same = ((prev_um_s > 0) && (want_um_s > 0)) || ((prev_um_s < 0) && (want_um_s < 0));
    int32_t const base = b_same ? prev_um_s : 0;
    int32_t next = want_um_s;

    if (want_um_s > 0)
    {
        int32_t const up = (base + step_um_s < START_UM_S) ? START_UM_S : (base + step_um_s);
        next = (want_um_s > up) ? up : want_um_s;
    }
    else if (want_um_s < 0)
    {
        int32_t const down = (base - step_um_s > -START_UM_S) ? -START_UM_S : (base - step_um_s);
        next = (want_um_s < down) ? down : want_um_s;
    }
    else
    {
        /* At rest. */
    }

    return next;
}

void drive_init(drive_state_t * p_drive)
{
    p_drive->left_um_s  = 0;
    p_drive->right_um_s = 0;
}

/* Reserve wheel speed for the turn before limiting the forward component. */
static void drive_components(int16_t speed_mm_s, int16_t turn_deci_s,
                             int32_t * p_forward_um_s, int32_t * p_half_diff_um_s)
{
    int32_t const turn = (int32_t)(((int64_t)turn_deci_s * TURN_NUM) / TURN_DEN);
    int32_t const half = clamp_i32(turn, -DRIVE_MAX_TARGET_UM_S, DRIVE_MAX_TARGET_UM_S);
    int32_t const room = DRIVE_MAX_TARGET_UM_S - ((half < 0) ? -half : half);

    *p_half_diff_um_s = half;
    *p_forward_um_s = clamp_i32((int32_t)speed_mm_s * UM_PER_MM, -room, room);
}

static int32_t acceleration_step_um_s(uint32_t dt_ms)
{
    uint32_t const dt = (dt_ms > DRIVE_DT_MAX_MS) ? DRIVE_DT_MAX_MS : dt_ms;

    return DRIVE_ACCEL_MM_S2 * (int32_t)dt;
}

void drive_step(drive_state_t * p_drive, int16_t speed_mm_s,
                int16_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                int32_t * p_right_um_s)
{
    int32_t const step = acceleration_step_um_s(dt_ms);
    int32_t forward = 0;
    int32_t half = 0;

    drive_components(speed_mm_s, turn_deci_s, &forward, &half);
    p_drive->left_um_s  = wheel_next(p_drive->left_um_s, forward - half, step);
    p_drive->right_um_s = wheel_next(p_drive->right_um_s, forward + half, step);
    *p_left_um_s        = p_drive->left_um_s;
    *p_right_um_s       = p_drive->right_um_s;
}

void drive_follow(drive_state_t * p_drive, int16_t speed_mm_s,
                  int16_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                  int32_t * p_right_um_s)
{
    int32_t last = (p_drive->left_um_s + p_drive->right_um_s) / HALF_DIVISOR;
    int32_t forward = 0;
    int32_t half = 0;

    drive_components(speed_mm_s, turn_deci_s, &forward, &half);
    if (0 == speed_mm_s)
    {
        drive_rise(p_drive, -half, half, dt_ms, p_left_um_s, p_right_um_s);
        return;
    }
    /* The follower asks for cruise at once as it starts; from rest or a
     * back-off that would spin the wheels. */
    last = ((last < 0) ? 0 : last) + acceleration_step_um_s(dt_ms);
    forward = (forward > last) ? last : forward;
    if (speed_mm_s > 0)
    {
        /* A correction after a pivot can exceed the ramped forward speed.
         * Slow the inner wheel to zero without driving it backward. */
        half = clamp_i32(half, -forward, forward);
    }

    p_drive->left_um_s  = forward - half;
    p_drive->right_um_s = forward + half;
    *p_left_um_s        = p_drive->left_um_s;
    *p_right_um_s       = p_drive->right_um_s;
}

void drive_rise(drive_state_t * p_drive, int32_t left_um_s, int32_t right_um_s,
                uint32_t dt_ms, int32_t * p_left_um_s, int32_t * p_right_um_s)
{
    int32_t const step = acceleration_step_um_s(dt_ms);
    /* Rise the forward and turn components, so the wheels of a pivot stay
     * mirrored and the axle does not drift as one wheel slows at once. */
    int32_t const forward = wheel_rise((p_drive->left_um_s + p_drive->right_um_s) / HALF_DIVISOR,
                                       (left_um_s + right_um_s) / HALF_DIVISOR, step);
    int32_t const half = wheel_rise((p_drive->right_um_s - p_drive->left_um_s) / HALF_DIVISOR,
                                    (right_um_s - left_um_s) / HALF_DIVISOR, step);

    p_drive->left_um_s  = forward - half;
    p_drive->right_um_s = forward + half;
    *p_left_um_s        = p_drive->left_um_s;
    *p_right_um_s       = p_drive->right_um_s;
}

void drive_compensate_roll(drive_state_t * p_drive, int16_t roll_cdeg,
                           int32_t left_speed_um_s, int32_t right_speed_um_s)
{
    bool const b_left_up = (roll_cdeg < 0);
    int32_t const roll = b_left_up ? -(int32_t)roll_cdeg : (int32_t)roll_cdeg;
    int32_t const target = p_drive->left_um_s;

    if ((roll > ROLL_DEADBAND_CDEG) && (left_speed_um_s >= 0) &&
        (right_speed_um_s >= 0) && (target == p_drive->right_um_s) &&
        (target > 0) && (target < DRIVE_MAX_TARGET_UM_S))
    {
        int32_t const error = b_left_up ? (right_speed_um_s - left_speed_um_s)
                                       : (left_speed_um_s - right_speed_um_s);
        int32_t boost = clamp_i32(error / HALF_DIVISOR, 0, ROLL_MAX_TRIM_UM_S);

        boost = clamp_i32(boost, 0, (roll - ROLL_DEADBAND_CDEG) * ROLL_GAIN_UM_S_PER_CDEG);
        boost = clamp_i32(boost, 0, target / HALF_DIVISOR);
        boost = clamp_i32(boost, 0, DRIVE_MAX_TARGET_UM_S - target);
        p_drive->left_um_s  += b_left_up ? boost : -boost;
        p_drive->right_um_s += b_left_up ? -boost : boost;
    }
}

void drive_safety_stop(drive_state_t * p_drive, int32_t * p_left_um_s,
                       int32_t * p_right_um_s)
{
    drive_init(p_drive);
    *p_left_um_s  = 0;
    *p_right_um_s = 0;
}
