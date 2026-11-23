/** @file nav_core.c
 *
 * @brief Pure part of the obstacle navigation glue. See nav_core.h.
 */

#include "nav_core.h"

#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "platform/units.h"

#define Q14_ONE   (16384)

/* Wheel path in mm for follow_in_t.travel_mm, saturating. */
static uint16_t travel_mm (nav_core_t const * p_core)
{
    uint32_t const mm = p_core->path_um / (uint32_t)UM_PER_MM;

    return ((mm > UINT16_MAX) ? UINT16_MAX : (uint16_t)mm);
}

/* Pose fields of the follow input. Casts are safe: a bypass moves the car
 * far less than 2 km. */
static void fill_pose (nav_core_t const * p_core, int64_t const p_wheel_um[NAV_CORE_WHEELS],
                       follow_in_t * p_in)
{
    p_in->x_mm      = (int32_t)(p_core->x_um / UM_PER_MM);
    p_in->y_mm      = (int32_t)(p_core->y_um / UM_PER_MM);
    p_in->head_deci = nav_core_heading_deci(p_core, p_wheel_um);
    p_in->travel_mm = travel_mm(p_core);
}

void nav_core_reset_pose (nav_core_t * p_core, int64_t const p_wheel_um[NAV_CORE_WHEELS])
{
    uint32_t wheel = 0U;

    for (wheel = 0U; wheel < NAV_CORE_WHEELS; wheel++)
    {
        p_core->base_um[wheel] = p_wheel_um[wheel];
        p_core->prev_um[wheel] = p_wheel_um[wheel];
    }
    p_core->x_um         = 0;
    p_core->y_um         = 0;
    p_core->path_um      = 0U;
    p_core->b_ir_cleared = false;
}

int32_t nav_core_heading_deci (nav_core_t const * p_core,
                               int64_t const p_wheel_um[NAV_CORE_WHEELS])
{
    int64_t const left_um  = p_wheel_um[NAV_CORE_LEFT] - p_core->base_um[NAV_CORE_LEFT];
    int64_t const right_um = p_wheel_um[NAV_CORE_RIGHT] - p_core->base_um[NAV_CORE_RIGHT];

    /* Cast is safe: a bypass turns the car far less than 2^31 deci-deg. */
    return (int32_t)wheel_yaw_deci(right_um - left_um);
}

void nav_core_update_pose (nav_core_t * p_core, int64_t const p_wheel_um[NAV_CORE_WHEELS])
{
    int64_t const left_um  = p_wheel_um[NAV_CORE_LEFT] - p_core->prev_um[NAV_CORE_LEFT];
    int64_t const right_um = p_wheel_um[NAV_CORE_RIGHT] - p_core->prev_um[NAV_CORE_RIGHT];
    int64_t const step_um  = (left_um + right_um) / (int64_t)NAV_CORE_WHEELS;
    int32_t const heading  = nav_core_heading_deci(p_core, p_wheel_um);

    /* x grows to the left: sin of a left (positive) heading. The path
     * length counts travel both ways, for the no-progress stop. */
    p_core->path_um += (uint32_t)((step_um < 0) ? -step_um : step_um);
    p_core->x_um += (step_um * obstacle_plan_sin_q14(heading)) / Q14_ONE;
    p_core->y_um += (step_um * obstacle_plan_cos_q14(heading)) / Q14_ONE;
    p_core->prev_um[NAV_CORE_LEFT]  = p_wheel_um[NAV_CORE_LEFT];
    p_core->prev_um[NAV_CORE_RIGHT] = p_wheel_um[NAV_CORE_RIGHT];
}

void nav_core_servo_wait (nav_core_t * p_core, uint32_t status, uint32_t now)
{
    /* Cast is safe: the sequence field is 5 bits. */
    p_core->b_servo_wait = true;
    p_core->wait_seq = (uint8_t)obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);
    p_core->wait_ms  = now;
}

bool nav_core_servo_moving (nav_core_t * p_core, uint32_t status, uint32_t now)
{
    uint32_t const seq = obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);

    if (p_core->b_servo_wait && ((seq != p_core->wait_seq) ||
                                 ((now - p_core->wait_ms) >= OBSTACLE_NAV_SERVO_WAIT_MS)))
    {
        p_core->b_servo_wait = false;
    }

    return (p_core->b_servo_wait);
}

bool nav_core_repeat_due (nav_core_t * p_core, uint32_t now)
{
    bool const b_due = ((now - p_core->line_ms) >= OBSTACLE_NAV_LINE_REPEAT_MS);

    if (b_due)
    {
        p_core->line_ms = now;
    }

    return (b_due);
}

void nav_core_follow_begin (nav_core_t * p_core, uint32_t ping,
                            int64_t const p_wheel_um[NAV_CORE_WHEELS], follow_in_t * p_in)
{
    fill_pose(p_core, p_wheel_um, p_in);
    p_in->dist_mm = 0U;
    p_in->cls     = OBSTACLE_CLASS_OPEN;
    p_in->b_ping  = false;
    p_in->b_line  = false;
    /* Cast is safe: the sequence field is 6 bits. */
    p_core->follow_seq =
        (uint8_t)obstacle_pack_get(ping, OBSTACLE_PING_SEQ_SH, OBSTACLE_PING_SEQ_MK);
    p_core->b_ir_cleared = false;
}

bool nav_core_follow_input (nav_core_t * p_core, uint32_t ping, int16_t look_deci,
                            int64_t const p_wheel_um[NAV_CORE_WHEELS], bool b_line_seen,
                            follow_in_t * p_in)
{
    uint32_t const seq     = obstacle_pack_get(ping, OBSTACLE_PING_SEQ_SH, OBSTACLE_PING_SEQ_MK);
    /* The ping word keeps whole degrees. A ping taken before the servo
     * reached the look is not a reading at the look. */
    bool const     b_aimed = (obstacle_pack_ping_servo(ping) ==
                              (((int32_t)look_deci / OBSTACLE_DECI_PER_DEG) * OBSTACLE_DECI_PER_DEG));
    bool const     b_opens = (!p_core->b_ir_cleared) && (!b_line_seen);

    if (b_opens)
    {
        p_core->b_ir_cleared = true;
    }
    fill_pose(p_core, p_wheel_um, p_in);
    /* Casts are safe: the class field is 3 bits, the distance field 12 bits
     * and the sequence field 6 bits. */
    p_in->b_ping  = (seq != (uint32_t)p_core->follow_seq) && b_aimed;
    p_in->cls     = (uint8_t)obstacle_pack_get(ping, OBSTACLE_PING_CLASS_SH, OBSTACLE_PING_CLASS_MK);
    p_in->dist_mm = (uint16_t)obstacle_pack_get(ping, OBSTACLE_PING_DIST_SH, OBSTACLE_PING_DIST_MK);
    p_in->b_line  = p_core->b_ir_cleared && b_line_seen;
    p_core->follow_seq = (uint8_t)seq;

    return (b_opens);
}

void nav_core_travel_update (nav_travel_t * p_travel, int64_t net_um)
{
    /* Cast is safe: +-2147 km of travel fits in 32 bits. */
    int32_t const net_mm = (int32_t)(net_um / UM_PER_MM);

    if (p_travel->b_base)
    {
        int32_t const step = net_mm - p_travel->last_mm;

        /* Casts are safe: the step is made positive first. */
        if (step > 0)
        {
            p_travel->fwd_mm += (uint32_t)step;
        }
        else
        {
            p_travel->back_mm += (uint32_t)(-step);
        }
    }
    p_travel->last_mm = net_mm;
    p_travel->b_base  = true;
}
