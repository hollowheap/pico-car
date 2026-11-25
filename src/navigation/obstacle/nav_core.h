/** @file nav_core.h
 *
 * @brief Pure part of the obstacle navigation glue: the dead
 *        reckoning, the servo wait, the IR gate, the ping freshness and the
 *        repeat gate of the handshake words. Pure C99, integer only, no
 *        kernel calls: obstacle_nav.c and the host simulation call the same
 *        code.
 *
 * Frame: the pose of the stationary scan. x grows to the left, y
 * along the original line, and a positive heading turns left.
 */

#ifndef OBSTACLE_NAV_CORE_H
#define OBSTACLE_NAV_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "follow.h"
#include "platform/static_assert.h"

#define NAV_CORE_LEFT   (0U)
#define NAV_CORE_RIGHT  (1U)
#define NAV_CORE_WHEELS (2U)

/* State: 64 bytes, the 8-byte fields first. Only the line task
 * touches it. */
typedef struct
{
    int64_t  base_um[NAV_CORE_WHEELS]; /* wheel travel at heading 0          */
    int64_t  prev_um[NAV_CORE_WHEELS]; /* wheel travel at the last pose step */
    int64_t  x_um;
    int64_t  y_um;
    uint32_t path_um;      /* wheel travel since the reset, both ways        */
    uint32_t wait_ms;      /* start of the servo wait                        */
    uint32_t line_ms;      /* last repeated handshake word                   */
    uint8_t  wait_seq;     /* status sequence at the start of the servo wait */
    uint8_t  follow_seq;   /* status sequence of the last ping the follow saw */
    bool     b_servo_wait;
    bool     b_ir_cleared; /* both IR sensors have left the start tape       */
} nav_core_t;

STATIC_ASSERT (sizeof(nav_core_t) == 64U, "nav_core_t size");

/*!
 * @brief Start the dead reckoning: the present wheel travel is heading 0 and
 *        position (0, 0). The IR gate closes.
 *
 * @param[out] p_core     State.
 * @param[in]  p_wheel_um Wheel travel, left and right.
 */
void nav_core_reset_pose (nav_core_t * p_core, int64_t const p_wheel_um[NAV_CORE_WHEELS]);

/*!
 * @brief Heading since the reset, from the wheel difference:
 *        (right - left) / track, positive = left.
 *
 * @return Heading in deci-degrees, not wrapped.
 */
int32_t nav_core_heading_deci (nav_core_t const * p_core,
                               int64_t const p_wheel_um[NAV_CORE_WHEELS]);

/*!
 * @brief Move the position on by the wheel travel since the last call, along
 *        the present heading.
 */
void nav_core_update_pose (nav_core_t * p_core, int64_t const p_wheel_um[NAV_CORE_WHEELS]);

/*!
 * @brief The servo turns on this state change: wait for the next ping, which
 *        comes only after the servo has settled.
 *
 * @param[in] status Packed status word.
 * @param[in] now    Current time, ms.
 */
void nav_core_servo_wait (nav_core_t * p_core, uint32_t status, uint32_t now);

/*!
 * @brief Tell whether the servo may still be turning. The wait ends with the
 *        first new ping or after OBSTACLE_NAV_SERVO_WAIT_MS.
 *
 * @return True while the car must stand still for the servo.
 */
bool nav_core_servo_moving (nav_core_t * p_core, uint32_t status, uint32_t now);

/*!
 * @brief Gate for a handshake word that is repeated until the statechart
 *        acts on it (TURNED, LINE_FOUND, REJECT).
 *
 * @return True once every OBSTACLE_NAV_LINE_REPEAT_MS: post the word now.
 */
bool nav_core_repeat_due (nav_core_t * p_core, uint32_t now);

/*!
 * @brief Entering REACQUIRE: the input for follow_start(). The IR gate
 *        closes and the present ping counts as seen.
 *
 * @param[in]  status     Packed status word.
 * @param[in]  p_wheel_um Wheel travel, left and right.
 * @param[out] p_in       Pose and the travel base; no ping, no line.
 */
void nav_core_follow_begin (nav_core_t * p_core, uint32_t status,
                            int64_t const p_wheel_um[NAV_CORE_WHEELS], follow_in_t * p_in);

/*!
 * @brief One REACQUIRE pass: the input for follow_step(). A ping is new when
 *        the status sequence changed. The tape counts only after both IR
 *        sensors have left the tape the car started on.
 *
 * @param[in]  status      Packed status word.
 * @param[in]  p_wheel_um  Wheel travel, left and right.
 * @param[in]  b_line_seen An IR sensor sees the line.
 * @param[out] p_in        Pose, ping and line.
 *
 * @return True on the pass that opens the IR gate.
 */
bool nav_core_follow_input (nav_core_t * p_core, uint32_t status,
                            int64_t const p_wheel_um[NAV_CORE_WHEELS], bool b_line_seen,
                            follow_in_t * p_in);

#endif /* OBSTACLE_NAV_CORE_H */
