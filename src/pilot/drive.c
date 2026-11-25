/** @file drive.c
 * @brief Pure drive layer: forward speed and turn rate to wheel targets.
 *
 * Integer math only. The state holds the last wheel targets. Each pass limits
 * the change of a wheel target, lifts a wheel that starts from rest to the
 * start speed, and sends a wheel that must reverse through zero first.
 */

#include "pilot/drive.h"

#include "platform/chassis.h"
#include "platform/units.h"

#define HALF_DIVISOR    (2)

#define START_UM_S      (DRIVE_START_MM_S * UM_PER_MM)
#define STOP_UM_S       (DRIVE_STOP_MM_S * UM_PER_MM)

/* Half the track times the turn rate, in um/s per deci-degree/s, as a
 * numerator and denominator so no precision is lost. */
#define TURN_NUM        ((int64_t)CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
#define TURN_DEN        ((int64_t)DECI_PER_RAD * HALF_DIVISOR)

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

void drive_init(drive_state_t * p_drive)
{
    p_drive->left_um_s  = 0;
    p_drive->right_um_s = 0;
}

void drive_step(drive_state_t * p_drive, int16_t speed_mm_s,
                int16_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                int32_t * p_right_um_s)
{
    uint32_t dt   = (dt_ms > DRIVE_DT_MAX_MS) ? DRIVE_DT_MAX_MS : dt_ms;
    int32_t  step = DRIVE_ACCEL_MM_S2 * (int32_t)dt;
    int32_t  v    = (int32_t)speed_mm_s * UM_PER_MM;
    /* The product alone needs 64 bits. */
    int32_t  half = (int32_t)(((int64_t)turn_deci_s * TURN_NUM) / TURN_DEN);
    int32_t  room = 0;

    /* Keep the curve: the turn has first claim on the wheel limit and the
     * forward speed gives way. */
    half = clamp_i32(half, -DRIVE_MAX_TARGET_UM_S, DRIVE_MAX_TARGET_UM_S);
    room = DRIVE_MAX_TARGET_UM_S - ((half < 0) ? -half : half);
    v    = clamp_i32(v, -room, room);

    p_drive->left_um_s  = wheel_next(p_drive->left_um_s, v - half, step);
    p_drive->right_um_s = wheel_next(p_drive->right_um_s, v + half, step);
    *p_left_um_s        = p_drive->left_um_s;
    *p_right_um_s       = p_drive->right_um_s;
}

void drive_follow(drive_state_t * p_drive, int16_t speed_mm_s,
                  int16_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                  int32_t * p_right_um_s)
{
    uint32_t dt   = (dt_ms > DRIVE_DT_MAX_MS) ? DRIVE_DT_MAX_MS : dt_ms;
    int32_t  last = (p_drive->left_um_s + p_drive->right_um_s) / HALF_DIVISOR;
    int32_t  v    = (int32_t)speed_mm_s * UM_PER_MM;
    int32_t  half = (int32_t)(((int64_t)turn_deci_s * TURN_NUM) / TURN_DEN);
    int32_t  room = 0;

    half = clamp_i32(half, -DRIVE_MAX_TARGET_UM_S, DRIVE_MAX_TARGET_UM_S);
    room = DRIVE_MAX_TARGET_UM_S - ((half < 0) ? -half : half);
    v    = clamp_i32(v, -room, room);
    /* The follower asks for cruise at once as it starts; from rest or a
     * back-off that would spin the wheels. */
    last = ((last < 0) ? 0 : last) + (DRIVE_ACCEL_MM_S2 * (int32_t)dt);
    v    = (v > last) ? last : v;

    p_drive->left_um_s  = v - half;
    p_drive->right_um_s = v + half;
    *p_left_um_s        = p_drive->left_um_s;
    *p_right_um_s       = p_drive->right_um_s;
}

void drive_safety_stop(drive_state_t * p_drive, int32_t * p_left_um_s,
                       int32_t * p_right_um_s)
{
    drive_init(p_drive);
    *p_left_um_s  = 0;
    *p_right_um_s = 0;
}
