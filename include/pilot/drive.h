/** @file drive.h
 * @brief Pure drive layer: forward speed and turn rate to wheel targets.
 *
 * No kernel, no hardware and no floating point. The caller owns the clock and
 * sends the returned targets to the motion service.
 *
 * Units: speed in mm/s, turn rate in deci-degrees per second (counter-clockwise
 * positive), wheel targets in um/s (positive = wheel forward).
 *
 * The constants marked "tune on the car" are starting values, not measured
 * ones.
 */

#ifndef PILOT_DRIVE_H
#define PILOT_DRIVE_H

#include <stdint.h>

#include "platform/static_assert.h"

/** Largest wheel target the motion service accepts (MOTOR_MAX_TARGET_UM_S). */
#define DRIVE_MAX_TARGET_UM_S (300000)

/** Limit of the change of a wheel target. Tune on the car. */
#define DRIVE_ACCEL_MM_S2 (500)

/**
 * A wheel that starts from rest gets at least this speed, so the motor does not
 * sit in its deadband. Tune on the car: it must clear the real deadband.
 */
#define DRIVE_START_MM_S (45)

/** A target slower than this is zero, not a crawl. Tune on the car. */
#define DRIVE_STOP_MM_S (20)

/** A pass this much later than the last one counts as this long (ms). */
#define DRIVE_DT_MAX_MS (50U)

/** Last wheel targets in um/s. Set up with drive_init(). */
typedef struct
{
    int32_t left_um_s;
    int32_t right_um_s;
} drive_state_t;

STATIC_ASSERT(sizeof(drive_state_t) == 8U, "drive_state_t size");

/** Both wheels at rest. */
void drive_init(drive_state_t * p_drive);

/**
 * One drive pass.
 *
 * @param[in,out] p_drive      Drive state.
 * @param[in]     speed_mm_s   Forward speed in mm/s.
 * @param[in]     turn_deci_s  Turn rate in deci-degrees per second, CCW positive.
 * @param[in]     dt_ms        Time since the last pass, clamped to DRIVE_DT_MAX_MS.
 * @param[out]    p_left_um_s  Left wheel target in um/s.
 * @param[out]    p_right_um_s Right wheel target in um/s.
 */
void drive_step(drive_state_t * p_drive, int16_t speed_mm_s,
                int16_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                int32_t * p_right_um_s);

/**
 * One pass of the line follower: the turn and a slower speed reach the wheel
 * targets at once, with the turn first as in drive_step(). A slewed steering
 * correction comes too late for a 150 mm corner. A faster forward speed rises
 * at DRIVE_ACCEL_MM_S2 from the mean of the last targets (0 if reversing). The
 * next drive_step() slews from these targets.
 *
 * @param[in,out] p_drive      Drive state.
 * @param[in]     speed_mm_s   Forward speed in mm/s.
 * @param[in]     turn_deci_s  Turn rate in deci-degrees per second, CCW positive.
 * @param[in]     dt_ms        Time since the last pass, clamped to DRIVE_DT_MAX_MS.
 * @param[out]    p_left_um_s  Left wheel target in um/s.
 * @param[out]    p_right_um_s Right wheel target in um/s.
 */
void drive_follow(drive_state_t * p_drive, int16_t speed_mm_s,
                  int16_t turn_deci_s, uint32_t dt_ms, int32_t * p_left_um_s,
                  int32_t * p_right_um_s);

/**
 * A safety stop: both wheel targets go to 0 at once, not through the slew
 * limit, which would take 0.3 s from 150 mm/s. The next drive_step() starts
 * from rest.
 *
 * @param[in,out] p_drive      Drive state.
 * @param[out]    p_left_um_s  Left wheel target, 0.
 * @param[out]    p_right_um_s Right wheel target, 0.
 */
void drive_safety_stop(drive_state_t * p_drive, int32_t * p_left_um_s,
                       int32_t * p_right_um_s);

#endif /* PILOT_DRIVE_H */

/*** end of file ***/
