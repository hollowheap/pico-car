/** @file drive.c
 * @brief Pure drive layer: forward speed and turn rate to wheel targets.
 *
 * Integer math only. The state holds the last wheel targets. Each pass limits
 * the change of a wheel target, lifts a wheel that starts from rest to the
 * start speed, and sends a wheel that must reverse through zero first.
 */

#include "pilot/drive.h"

#define UM_PER_MM       (1000)
#define DECI_PER_RAD    (573)
#define HALF_DIVISOR    (2)

#define START_UM_S      (DRIVE_START_MM_S * UM_PER_MM)
#define STOP_UM_S       (DRIVE_STOP_MM_S * UM_PER_MM)

/* Half the track times the turn rate, in um/s per deci-degree/s, as a
 * numerator and denominator so no precision is lost. */
#define TURN_NUM        ((int64_t)DRIVE_WHEEL_TRACK_MM * UM_PER_MM)
#define TURN_DEN        ((int64_t)DECI_PER_RAD * HALF_DIVISOR)

/* Larger than any speed or turn the caller can pass, so the products below
 * cannot overflow 64 bits. */
#define INPUT_CLAMP     (1000000000)

static int64_t clamp64(int64_t value, int64_t low, int64_t high)
{
    int64_t result = value;

    if (result < low)
    {
        result = low;
    }
    if (result > high)
    {
        result = high;
    }
    return result;
}

/**
 * Next target of one wheel.
 *
 * @param prev_um_s Last target of the wheel.
 * @param want_um_s Wanted target.
 * @param step_um_s Largest change of the target in this pass.
 */
static int32_t wheel_next(int32_t prev_um_s, int64_t want_um_s,
                          int64_t step_um_s)
{
    int64_t want = want_um_s;
    int64_t next = 0;

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
        next = clamp64(want, -step_um_s, step_um_s);
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
        next = clamp64(want, (int64_t)prev_um_s - step_um_s,
                       (int64_t)prev_um_s + step_um_s);
        if ((next > -STOP_UM_S) && (next < STOP_UM_S))
        {
            next = 0;
        }
    }

    return (int32_t)next;
}

void drive_init(drive_state_t * p_drive)
{
    p_drive->left_um_s  = 0;
    p_drive->right_um_s = 0;
}

void drive_step(drive_state_t * p_drive, int32_t speed_mm_s,
                int32_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                int32_t * p_right_um_s)
{
    uint32_t dt   = (dt_ms > DRIVE_DT_MAX_MS) ? DRIVE_DT_MAX_MS : dt_ms;
    int64_t  step = (int64_t)DRIVE_ACCEL_MM_S2 * (int64_t)dt;
    int64_t  v    = (int64_t)clamp64(speed_mm_s, -INPUT_CLAMP, INPUT_CLAMP)
                    * UM_PER_MM;
    int64_t  half = (clamp64(turn_deci_s, -INPUT_CLAMP, INPUT_CLAMP)
                     * TURN_NUM) / TURN_DEN;
    int64_t  room = 0;

    /* Keep the curve: the turn has first claim on the wheel limit and the
     * forward speed gives way. */
    half = clamp64(half, -DRIVE_MAX_TARGET_UM_S, DRIVE_MAX_TARGET_UM_S);
    room = DRIVE_MAX_TARGET_UM_S - ((half < 0) ? -half : half);
    v    = clamp64(v, -room, room);

    p_drive->left_um_s  = wheel_next(p_drive->left_um_s, v - half, step);
    p_drive->right_um_s = wheel_next(p_drive->right_um_s, v + half, step);
    *p_left_um_s        = p_drive->left_um_s;
    *p_right_um_s       = p_drive->right_um_s;
}
