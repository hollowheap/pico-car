/** @file pilot.c
 * @brief Pilot core: safety stop and line follow, in priority order.
 *
 * Integer math only. Each behaviour is a function that returns true when it
 * takes control and has then filled the command. Avoid and search go between
 * safety and follow line.
 */

#include "pilot/pilot.h"

#define UM_PER_MM            (1000)
#define DECI_PER_RAD         (573)
#define TRACK_DIVISOR        (LINE_WHEEL_TRACK_MM * UM_PER_MM)
#define HALF_DIVISOR         (2)

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

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

static void set_cmd(pilot_cmd_t * p_cmd, int32_t speed_mm_s,
                    int32_t turn_deci_s, pilot_behaviour_t behaviour,
                    pilot_reason_t reason)
{
    p_cmd->speed_mm_s  = (int16_t)clamp_i32(speed_mm_s, -PILOT_SPEED_MAX_MM_S,
                                            PILOT_SPEED_MAX_MM_S);
    p_cmd->turn_deci_s = (int16_t)clamp_i32(turn_deci_s, -PILOT_TURN_MAX_DECI_S,
                                            PILOT_TURN_MAX_DECI_S);
    p_cmd->behaviour   = (uint8_t)behaviour;
    p_cmd->reason      = (uint8_t)reason;
}

static bool behaviour_safety(pilot_state_t const * p_state,
                             pilot_meas_t const * p_meas, pilot_cmd_t * p_cmd)
{
    bool b_takes = false;

    if (0U != p_state->b_estop)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ESTOP);
        b_takes = true;
    }
    /* The unsigned difference is right across the wrap of the clock. */
    else if ((p_meas->now_ms - p_meas->stamp_ms) > PILOT_STALE_MS)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_STALE);
        b_takes = true;
    }
    else
    {
        /* Nothing to do: leave control to the next behaviour. */
    }
    return b_takes;
}

static bool behaviour_follow_line(pilot_state_t * p_state,
                                  pilot_meas_t const * p_meas,
                                  pilot_cmd_t * p_cmd)
{
    line_targets_t targets = {0, 0};
    line_follow_status_t const status = line_follow_update(
        &p_state->follow, p_meas->b_line_left, p_meas->b_line_right,
        line_heading_deci(p_meas->left_um, p_meas->right_um),
        (p_meas->left_um + p_meas->right_um) / HALF_DIVISOR, p_meas->now_ms,
        &targets);

    if (LINE_FOLLOW_STOPPED == status)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_FOLLOW_LINE,
                PILOT_REASON_LINE_LOST_STOP);
    }
    else
    {
        int32_t const speed_um_s = (targets.left_um_s + targets.right_um_s) /
                                   HALF_DIVISOR;
        int32_t const diff_um_s = targets.right_um_s - targets.left_um_s;

        set_cmd(p_cmd, speed_um_s / UM_PER_MM,
                (diff_um_s * DECI_PER_RAD) / TRACK_DIVISOR,
                PILOT_BEHAVIOUR_FOLLOW_LINE, PILOT_REASON_NONE);
    }
    return true;
}

void pilot_init(pilot_state_t * p_state)
{
    line_follow_init(&p_state->follow);
    p_state->b_estop = 0U;
}

void pilot_step(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                pilot_cmd_t * p_cmd)
{
    set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_NONE, PILOT_REASON_NONE);
    if (!behaviour_safety(p_state, p_meas, p_cmd))
    {
        (void)behaviour_follow_line(p_state, p_meas, p_cmd);
    }
}

void pilot_estop(pilot_state_t * p_state)
{
    p_state->b_estop = 1U;
}

void pilot_reset(pilot_state_t * p_state)
{
    pilot_init(p_state);
}

/*** end of file ***/
