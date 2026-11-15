/** @file follow.h
 *
 * @brief Bug2 face follow for the obstacle bypass.
 *        Pure C99, integer only, no kernel calls: the firmware navigation and
 *        the host simulation call the same code.
 *
 * Frame: the pose of the stationary scan. x grows to the left, y
 * along the original line, heading 0 is the original line and a positive
 * heading turns left.
 *
 * The car follows the obstacle face on one side while it moves. It drives a
 * short leg parallel to the face, then arcs toward the face until the ping at
 * the servo limit sees it, records the face point and arcs back. A face that
 * is not seen any more is an outer corner: the car drives past it and turns
 * toward the next face. An echo ahead during a leg is an inner corner: the
 * car turns away. The bypass ends where the IR filter confirms the tape past
 * the hit point; the car then turns to the original heading.
 */

#ifndef OBSTACLE_FOLLOW_H
#define OBSTACLE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

#include "platform/static_assert.h"

/* Left and right. */
#define FOLLOW_WHEELS (2U)

/* follow_out_t.result */
#define FOLLOW_RUN  (0U) /* keep stepping                                  */
#define FOLLOW_LINE (1U) /* on the tape, original heading: hand back       */
#define FOLLOW_GAVE_UP (2U) /* stop: cause BYPASS_GAVE_UP, follow_out_t.reason */

/* follow_out_t.reason: why the bypass gave up (3 bits on the wire). */
#define FOLLOW_REASON_NONE        (0U)
#define FOLLOW_REASON_CIRCLED     (1U) /* back at the hit point             */
#define FOLLOW_REASON_NO_PROGRESS (2U) /* no gain along the original line   */
#define FOLLOW_REASON_WHEEL_STALL (3U) /* a driven wheel does not turn      */
#define FOLLOW_REASON_ECHO_LOST   (4U) /* the back-up tries are used up     */

/* One navigation pass of input. */
typedef struct
{
    int32_t  x_mm;      /* axle middle, scan frame                         */
    int32_t  y_mm;
    int32_t  head_deci; /* not wrapped                                     */
    uint16_t dist_mm;   /* the new ping, taken at the last servo_deci      */
    uint8_t  cls;       /* OBSTACLE_CLASS_*                                */
    bool     b_ping;    /* a ping arrived since the last step              */
    bool     b_line;    /* the IR filter confirms the tape                 */
    uint16_t travel_mm; /* wheel path since the scan began, saturating; the
                           value at follow_start is the base of the hard stops */
} follow_in_t;

/* What the navigation applies after one step. */
typedef struct
{
    int32_t left_um_s;
    int32_t right_um_s;
    int16_t servo_deci; /* servo target for the next ping, +-servo limit   */
    uint8_t steer;      /* OBSTACLE_STEER_*                                */
    uint8_t result;     /* FOLLOW_*                                        */
    uint8_t reason;     /* FOLLOW_REASON_* when result is FOLLOW_GAVE_UP    */
    bool    b_turning;  /* the body turns: pings are guard-only            */
} follow_out_t;

/* Follow state: 16 bytes. Positions in mm fit int16_t (+-32 m). */
typedef struct
{
    int16_t face_x_mm;      /* the last face point, or the corner reference */
    int16_t face_y_mm;
    int16_t face_head_deci; /* travel direction along the face              */
    int16_t mark_x_mm;      /* where the present phase started              */
    int16_t mark_y_mm;
    uint8_t phase   : 4;
    uint8_t b_point : 1;    /* face_x/face_y hold a point of this face      */
    uint8_t b_left  : 1;    /* the car passes on the left: face on the right */
    uint8_t misses  : 2;    /* pings in a row without the face              */
    uint8_t b_settled : 1;  /* the glance heading held on an earlier step   */
    uint8_t b_seed    : 1;  /* the point is the scan estimate, not an echo  */
    uint8_t tries     : 2;  /* echo-loss recoveries on this face            */
    uint8_t reason    : 3;  /* FOLLOW_REASON_* once the bypass gave up      */
    int16_t  best_y_mm;     /* furthest point along the original line       */
    uint16_t best_travel_mm; /* travel when best_y_mm was reached           */
} follow_t;

STATIC_ASSERT (sizeof(follow_t) <= 16U, "follow_t stays small");

/* Virtual bumper state: 8 bytes. */
typedef struct
{
    int16_t  cmd_mm_s[FOLLOW_WHEELS]; /* last command, left and right */
    uint16_t settle_ms;   /* time since the command last changed      */
    uint16_t low_ms;      /* time a driven wheel has been too slow    */
} follow_bump_t;

/*!
 * @brief Start following after the escape turn.
 *
 * @param[out] p_follow  State.
 * @param[in]  b_left    The car passes on the left (the face is on its right).
 * @param[in]  p_in      Present pose. The first face is the box front, at
 *                       +-90 degrees from the original line.
 * @param[in]  hit_mm    Scan origin (axle) to the front face straight ahead,
 *                       from the reading that stopped the car; 0 if unknown.
 *                       A face must be seen before its end counts as a corner.
 */
void follow_start (follow_t * p_follow, bool b_left, follow_in_t const * p_in, int32_t hit_mm);

/*!
 * @brief One navigation pass.
 *
 * @param[in,out] p_follow State.
 * @param[in]     p_in     Pose, the new ping and the tape.
 * @param[out]    p_out    Wheels, servo and result.
 */
void follow_step (follow_t * p_follow, follow_in_t const * p_in, follow_out_t * p_out);

/*!
 * @brief Virtual bumper: a wheel told to run at OBSTACLE_BUMP_MIN_MM_S
 *        or more that measures under OBSTACLE_BUMP_PERCENT of it for
 *        OBSTACLE_BUMP_MS has hit something. Not checked for
 *        OBSTACLE_BUMP_SETTLE_MS after the command changes.
 *
 * @param[in,out] p_bump    State; zero it to start.
 * @param[in]     p_cmd_um_s  Commanded wheel speeds, left and right.
 * @param[in]     p_meas_um_s Measured wheel speeds, left and right.
 * @param[in]     dt_ms     Time since the last call.
 *
 * @return True when a wheel stalls.
 */
bool follow_bump_update (follow_bump_t * p_bump, int32_t const p_cmd_um_s[FOLLOW_WHEELS],
                         int32_t const p_meas_um_s[FOLLOW_WHEELS], uint32_t dt_ms);

#endif /* OBSTACLE_FOLLOW_H */
