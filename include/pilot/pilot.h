/** @file pilot.h
 * @brief Pilot: the one owner of the wheels. One motion command per tick.
 *
 * Pure C, no kernel, no driver and no floating point. The caller owns the
 * clock, gathers the latest measurements and sends the returned command to
 * the motion layer.
 *
 * Each tick the behaviours run in priority order and the first one that asks
 * for control sets the command. The choice is made again every tick; the
 * behaviours do not hand anything over to each other.
 *
 * Units: speed in mm/s (positive = forward), turn rate in deci-degrees per
 * second (counter-clockwise positive), travel in um, time in ms.
 */

#ifndef PILOT_PILOT_H
#define PILOT_PILOT_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"
#include "track/line_control.h"

/** Measurements older than this give a stop with reason STALE (ms). */
#define PILOT_STALE_MS            (100U)
/** A command never exceeds these. The turn limit is the largest turn rate of
 *  the follower: both wheels at LINE_CRUISE_UM_S, opposite. */
#define PILOT_SPEED_MAX_MM_S      (300)
#define PILOT_TURN_MAX_DECI_S     (3300)

/** Why the command is a stop. Only ESTOP latches. */
typedef enum
{
    PILOT_REASON_NONE = 0,
    PILOT_REASON_STALE,
    PILOT_REASON_ESTOP,
    PILOT_REASON_LINE_LOST_STOP
} pilot_reason_t;

STATIC_ASSERT(sizeof(pilot_reason_t) <= 4U, "pilot_reason_t size");

/** Behaviour that set the command, in priority order. */
typedef enum
{
    PILOT_BEHAVIOUR_NONE = 0,
    PILOT_BEHAVIOUR_SAFETY,
    PILOT_BEHAVIOUR_FOLLOW_LINE
} pilot_behaviour_t;

STATIC_ASSERT(sizeof(pilot_behaviour_t) <= 4U, "pilot_behaviour_t size");

/** Latest measurements, taken together at stamp_ms. */
typedef struct
{
    int64_t  left_um;       /* left wheel displacement, positive forward  */
    int64_t  right_um;      /* right wheel displacement, positive forward */
    uint32_t now_ms;        /* free-running clock (wraps at 2^32)         */
    uint32_t stamp_ms;      /* clock at the time of the measurements      */
    bool     b_line_left;   /* left sensor is over the line               */
    bool     b_line_right;  /* right sensor is over the line              */
} pilot_meas_t;

STATIC_ASSERT(sizeof(pilot_meas_t) <= 32U, "pilot_meas_t size");

/** The command of one tick. behaviour and reason are for the decision log. */
typedef struct
{
    int16_t speed_mm_s;
    int16_t turn_deci_s;
    uint8_t behaviour;      /* pilot_behaviour_t */
    uint8_t reason;         /* pilot_reason_t    */
} pilot_cmd_t;

STATIC_ASSERT(sizeof(pilot_cmd_t) == 6U, "pilot_cmd_t size");

typedef struct
{
    line_follow_t follow;
    uint8_t       b_estop;  /* latched until pilot_reset() */
} pilot_state_t;

STATIC_ASSERT(sizeof(pilot_state_t) <= 32U, "pilot_state_t size");

/** Clear the state: no latch, follower at its start. */
void pilot_init(pilot_state_t * p_state);

/**
 * One pilot tick.
 *
 * @param[in,out] p_state State.
 * @param[in]     p_meas  Latest measurements.
 * @param[out]    p_cmd   The command, within the speed and turn limits.
 */
void pilot_step(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                pilot_cmd_t * p_cmd);

/** Operator emergency stop. The stop holds until pilot_reset(). */
void pilot_estop(pilot_state_t * p_state);

/** End an emergency stop and restart the follower. */
void pilot_reset(pilot_state_t * p_state);

#endif /* PILOT_PILOT_H */

/*** end of file ***/
