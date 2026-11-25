/** @file pilot.c
 * @brief Pilot core: safety stop and line follow, in priority order.
 *
 * Integer math only. Each behaviour is a function that returns true when it
 * takes control and has then filled the command; follow line always takes
 * control and returns nothing. The hug (go round an
 * obstacle) goes between safety and follow line; search is not written yet.
 */

#include "pilot/pilot.h"

#include "platform/chassis.h"
#include "platform/units.h"

#define TRACK_DIVISOR        (CHASSIS_WHEEL_TRACK_MM * UM_PER_MM)
#define HALF_DIVISOR         (2)

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
    p_cmd->ping_servo_deci = 0;
    p_cmd->ping_seq    = 0U;
    p_cmd->b_ping_req  = 0U;
}

#define FLAG_MOVING          (0x01UL)  /* the last command asked for motion       */
#define FLAG_LOW_LEFT        (0x02UL)  /* left wheel commanded below the minimum  */
#define FLAG_LOW_RIGHT       (0x04UL)  /* right wheel commanded below the minimum */
#define FLAG_REVERSE         (0x08UL)  /* last command: line reverse */
/* A run is a stretch of sonar fault or stale wheel feedback. */
#define FLAG_SONAR_BAD       (0x10UL)  /* a run is open                    */
#define FLAG_SONAR_LATCH     (0x20UL)  /* the run outlasted the window     */
#define FLAG_SONAR_STALE     (0x40UL)  /* the latched stop is STALE        */
#define FLAGS_SONAR (FLAG_SONAR_BAD | FLAG_SONAR_LATCH | FLAG_SONAR_STALE)
#define WINDOW_MAX_UM        (0x7FFFFFFFUL)

static void flag_set(pilot_state_t * p_state, uint32_t mask, bool b_on)
{
    uint32_t flags = p_state->flags;

    flags = b_on ? (flags | mask) : (flags & ~mask);
    p_state->flags = (uint8_t)flags;
}

static bool flag_get(pilot_state_t const * p_state, uint32_t mask)
{
    return 0U != ((uint32_t)p_state->flags & mask);
}

static void window_start(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                         pilot_phase_t phase)
{
    p_state->ref_left_um  = (uint32_t)p_meas->left_um;
    p_state->ref_right_um = (uint32_t)p_meas->right_um;
    p_state->since_ms     = p_meas->now_ms;
    p_state->phase        = (uint8_t)phase;
}

static bool backing_off(uint8_t phase)
{
    return ((uint8_t)PILOT_PHASE_BACKOFF == phase) ||
           ((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL == phase);
}

/* Travel since the start of the window as the unsigned difference of the low
 * 32 bits: right across the wrap, and no signed conversion. A value above
 * WINDOW_MAX_UM is travel backward. */
static uint32_t travel_um(int64_t now_um, uint32_t ref_um)
{
    return (uint32_t)now_um - ref_um;
}

static bool wheel_moved(uint32_t um)
{
    uint32_t const back_um = 0U - um;

    return (um >= (uint32_t)PILOT_MOVE_MIN_UM) &&
           (back_um >= (uint32_t)PILOT_MOVE_MIN_UM);
}

static bool wheel_back(uint32_t um, uint32_t min_um)
{
    uint32_t const back_um = 0U - um;

    return (back_um >= min_um) && (back_um <= WINDOW_MAX_UM);
}

/* Stuck: both wheels without travel while motion is commanded. The car backs
 * off, then tries again. One wheel without travel in a try and again in the
 * back-off after it is a dead encoder (or a jammed wheel): stop. */
static bool behaviour_stuck(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                            pilot_cmd_t * p_cmd)
{
    bool b_takes = false;
    uint32_t const dl = travel_um(p_meas->left_um, p_state->ref_left_um);
    uint32_t const dr = travel_um(p_meas->right_um, p_state->ref_right_um);
    bool const b_left = wheel_moved(dl);
    bool const b_right = wheel_moved(dr);
    uint32_t const age_ms = p_meas->now_ms - p_state->since_ms;

    if (!flag_get(p_state, FLAG_MOVING))
    {
        /* Nothing was commanded: the window starts again, and a retry stays one. */
        window_start(p_state, p_meas,
                     ((uint8_t)PILOT_PHASE_RETRY == p_state->phase) ? PILOT_PHASE_RETRY
                                                                    : PILOT_PHASE_TRY);
    }
    else if (!backing_off(p_state->phase))
    {
        /* A wheel commanded below the minimum may rightly stand still. */
        bool const b_need_l = !flag_get(p_state, FLAG_LOW_LEFT);
        bool const b_need_r = !flag_get(p_state, FLAG_LOW_RIGHT);

        if ((b_left || !b_need_l) && (b_right || !b_need_r))
        {
            window_start(p_state, p_meas, PILOT_PHASE_TRY);
        }
        else if ((age_ms >= PILOT_STUCK_MS) && flag_get(p_state, FLAG_REVERSE))
        {
            /* The back-off assumes the blockage is ahead. A blocked reverse is
             * ended by the follower; backing off would push further into it. */
            window_start(p_state, p_meas, PILOT_PHASE_TRY);
        }
        else if (age_ms >= PILOT_STUCK_MS)
        {
            window_start(p_state, p_meas,
                         ((b_left != b_right) && b_need_l && b_need_r)
                             ? PILOT_PHASE_BACKOFF_ONE_WHEEL
                             : PILOT_PHASE_BACKOFF);
        }
        else
        {
            /* Still within the window. */
        }
    }
    else if (wheel_back(dl, (uint32_t)PILOT_BACKOFF_UM) ||
             wheel_back(dr, (uint32_t)PILOT_BACKOFF_UM) ||
             (age_ms >= PILOT_BACKOFF_MAX_MS))
    {
        if (((uint8_t)PILOT_PHASE_BACKOFF_ONE_WHEEL == p_state->phase) && (b_left != b_right))
        {
            p_state->b_encoder = 1U;
        }
        window_start(p_state, p_meas, PILOT_PHASE_RETRY);
    }
    else
    {
        /* Still backing off. */
    }
    if (0U != p_state->b_encoder)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ENCODER);
        b_takes = true;
    }
    else if (backing_off(p_state->phase))
    {
        set_cmd(p_cmd, -PILOT_BACKOFF_MM_S, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_STUCK);
        b_takes = true;
    }
    else
    {
        /* Drive: the follower runs. */
    }
    return b_takes;
}

/* The stop for a sonar fault or stale wheel feedback: its reason while the run
 * lasts, and after it for good once the run has outlasted
 * PILOT_SONAR_RESUME_MS. The latched reason keeps the cause of the moment the
 * latch sets. Stale feedback while motion holds the wheels is no run: HELD
 * answers. The run starts at since_ms: a stop is no motion, so the stuck window
 * starts again after it and does not need the value. */
static pilot_reason_t data_stop(pilot_state_t * p_state,
                                pilot_meas_t const * p_meas)
{
    /* The unsigned difference is right across the wrap of the clock. */
    bool const b_stale = !p_meas->b_held &&
                         ((p_meas->now_ms - p_meas->stamp_ms) > PILOT_STALE_MS);
    bool const b_bad = p_meas->b_sonar_fault || b_stale;
    pilot_reason_t reason = PILOT_REASON_NONE;

    if (!b_bad)
    {
        flag_set(p_state, FLAG_SONAR_BAD, false);
    }
    else if (!flag_get(p_state, FLAG_SONAR_BAD))
    {
        flag_set(p_state, FLAG_SONAR_BAD, true);
        p_state->since_ms = p_meas->now_ms;
    }
    else if (!flag_get(p_state, FLAG_SONAR_LATCH) &&
             ((p_meas->now_ms - p_state->since_ms) >= PILOT_SONAR_RESUME_MS))
    {
        flag_set(p_state, FLAG_SONAR_LATCH, true);
        flag_set(p_state, FLAG_SONAR_STALE, !p_meas->b_sonar_fault);
    }
    else
    {
        /* Within the window, or latched already. */
    }
    if (flag_get(p_state, FLAG_SONAR_LATCH))
    {
        reason = flag_get(p_state, FLAG_SONAR_STALE)
                     ? PILOT_REASON_STALE_LATCHED
                     : PILOT_REASON_SONAR_LATCHED;
    }
    else if (b_bad)
    {
        reason = p_meas->b_sonar_fault ? PILOT_REASON_SONAR
                                       : PILOT_REASON_STALE;
    }
    else
    {
        /* Good data. */
    }
    return reason;
}

static bool behaviour_safety(pilot_state_t * p_state,
                             pilot_meas_t const * p_meas, pilot_cmd_t * p_cmd)
{
    pilot_reason_t const data = data_stop(p_state, p_meas);
    bool b_takes = false;

    if (0U != p_state->b_estop)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ESTOP);
        b_takes = true;
    }
    else if (0U != p_state->b_encoder)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_ENCODER);
        b_takes = true;
    }
    else if (0U != p_state->b_timeout)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HUG_TIMEOUT);
        b_takes = true;
    }
    else if ((PILOT_REASON_SONAR == data) ||
             (PILOT_REASON_SONAR_LATCHED == data))
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, data);
        b_takes = true;
    }
    /* Motion refuses commands while it holds the car: no travel is no proof
     * of a stuck car, so the stuck window restarts, as for stale data. */
    else if (p_meas->b_held)
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HELD);
        p_state->phase = (uint8_t)PILOT_PHASE_TRY;
        b_takes = true;
    }
    else if ((PILOT_REASON_STALE == data) ||
             (PILOT_REASON_STALE_LATCHED == data))
    {
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, data);
        p_state->phase = (uint8_t)PILOT_PHASE_TRY;
        b_takes = true;
    }
    else
    {
        b_takes = behaviour_stuck(p_state, p_meas, p_cmd);
    }
    return b_takes;
}

static void behaviour_follow_line(pilot_state_t * p_state,
                                  pilot_meas_t const * p_meas,
                                  pilot_cmd_t * p_cmd)
{
    line_targets_t targets = {0, 0};
    line_follow_status_t status = (line_follow_status_t)p_state->follow.status;
    /* Barcode stripes under the sensors look like a lost line: drive straight
     * and leave the follower alone. Not while it searches: the straight would
     * drive forward over the reverse. A stopped car stays stopped. */
    bool const b_straight = p_meas->b_barcode &&
                            ((LINE_FOLLOW_FOLLOWING == status) ||
                             (LINE_FOLLOW_BRIDGING == status));

    if (b_straight)
    {
        targets.left_um_s  = LINE_CRUISE_UM_S;
        targets.right_um_s = LINE_CRUISE_UM_S;
        line_follow_reanchor(&p_state->follow);
    }
    else
    {
        status = line_follow_update(
            &p_state->follow, p_meas->b_line_left, p_meas->b_line_right,
            line_heading_deci(p_meas->left_um, p_meas->right_um),
            (p_meas->left_um + p_meas->right_um) / HALF_DIVISOR, p_meas->now_ms,
            &targets);
    }

    bool const b_ranged = (!p_meas->b_ping_reply) && (p_meas->ping_mm > 0U) &&
                          look_echo_is_point(p_meas->ping_echo);

    line_targets_cap(&targets, line_bumper_cap_um_s(b_ranged, p_meas->ping_mm));
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
}

void pilot_init(pilot_state_t * p_state)
{
    line_follow_init(&p_state->follow);
    p_state->b_estop    = 0U;
    p_state->b_encoder  = 0U;
    p_state->b_timeout  = 0U;
    p_state->flags      = 0U;
    p_state->phase      = (uint8_t)PILOT_PHASE_TRY;
    p_state->since_ms   = 0U;
    p_state->ref_left_um  = 0U;
    p_state->ref_right_um = 0U;
    hug_init(&p_state->hug);
}

/* Note which wheels the last command asks to run below the minimum speed. */
static void note_wheels(pilot_state_t * p_state, pilot_cmd_t const * p_cmd)
{
    int32_t const speed = p_cmd->speed_mm_s;
    int32_t const half = ((int32_t)p_cmd->turn_deci_s * CHASSIS_WHEEL_TRACK_MM) /
                         (DECI_PER_RAD * HALF_DIVISOR);
    int32_t const left = speed - half;
    int32_t const right = speed + half;

    flag_set(p_state, FLAG_LOW_LEFT,
             (left < PILOT_WHEEL_MIN_MM_S) && (left > -PILOT_WHEEL_MIN_MM_S));
    flag_set(p_state, FLAG_LOW_RIGHT,
             (right < PILOT_WHEEL_MIN_MM_S) && (right > -PILOT_WHEEL_MIN_MM_S));
}

/* The hug watches the forward pings and tracks the pose on every tick; it
 * takes the wheels and the sonar when it has something to go round. */
static bool behaviour_hug(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                          bool b_other, bool b_stall, pilot_cmd_t * p_cmd)
{
    hug_in_t in;
    hug_out_t out;
    bool const b_was = hug_active(&p_state->hug);
    bool b_takes = false;

    in.left_um = p_meas->left_um;
    in.right_um = p_meas->right_um;
    in.now_ms = p_meas->now_ms;
    in.ping_mm = p_meas->ping_mm;
    in.ping_servo_deci = p_meas->ping_servo_deci;
    in.ping_echo = p_meas->ping_echo;
    in.ping_seq = p_meas->ping_seq;
    in.b_ping_reply = p_meas->b_ping_reply;
    in.b_line_left = p_meas->b_line_left;
    in.b_line_right = p_meas->b_line_right;
    in.b_stall = b_stall;
    in.b_other = b_other;
    b_takes = hug_step(&p_state->hug, &p_state->look, &in, &out);
    if (0U != out.b_timeout)
    {
        /* The car is beside the obstacle and off the line: the follower must
         * not take over, so the stop holds. */
        p_state->b_timeout = 1U;
        set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_SAFETY, PILOT_REASON_HUG_TIMEOUT);
        b_takes = true;
    }
    else if (b_takes)
    {
        set_cmd(p_cmd, out.speed_mm_s, out.turn_deci_s, PILOT_BEHAVIOUR_HUG,
                (0U != out.b_stop) ? PILOT_REASON_SONAR : PILOT_REASON_NONE);
        p_cmd->ping_servo_deci = out.servo_deci;
        p_cmd->ping_seq = out.seq;
        p_cmd->b_ping_req = out.b_req;
    }
    else if (b_was && !hug_active(&p_state->hug))
    {
        /* The hug gave the wheels back: the follower starts afresh. */
        line_follow_init(&p_state->follow);
    }
    else
    {
        /* Not the hug's tick. */
    }
    return b_takes;
}

void pilot_step(pilot_state_t * p_state, pilot_meas_t const * p_meas,
                pilot_cmd_t * p_cmd)
{
    bool const b_was_back = backing_off(p_state->phase);
    bool b_safe = false;
    bool b_moving = false;

    set_cmd(p_cmd, 0, 0, PILOT_BEHAVIOUR_NONE, PILOT_REASON_NONE);
    b_safe = behaviour_safety(p_state, p_meas, p_cmd);
    if (!behaviour_hug(p_state, p_meas, b_safe,
                       b_was_back && !backing_off(p_state->phase), p_cmd) && !b_safe)
    {
        behaviour_follow_line(p_state, p_meas, p_cmd);
    }
    b_moving = (0 != p_cmd->speed_mm_s) || (0 != p_cmd->turn_deci_s);
    if ((PILOT_REASON_NONE == p_cmd->reason) && ((uint8_t)PILOT_PHASE_RETRY == p_state->phase) &&
        b_moving)
    {
        p_cmd->reason = (uint8_t)PILOT_REASON_STUCK;
    }
    flag_set(p_state, FLAG_MOVING, b_moving);
    flag_set(p_state, FLAG_REVERSE,
             ((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour) &&
                 (p_cmd->speed_mm_s < 0));
    note_wheels(p_state, p_cmd);
}

void pilot_estop(pilot_state_t * p_state)
{
    p_state->b_estop = 1U;
}

void pilot_clear_latches(pilot_state_t * p_state)
{
    p_state->b_encoder = 0U;
    p_state->b_timeout = 0U;
    /* No motion is recorded: the next tick opens a new stuck window. This
     * clears the sonar and stale latch too. */
    p_state->flags = 0U;
    p_state->phase = (uint8_t)PILOT_PHASE_TRY;
    /* The side of the line stays: a one-shot turn has just set it. */
    line_follow_set_side(&p_state->follow, (line_side_t)p_state->follow.side);
}

void pilot_release_sonar(pilot_state_t * p_state)
{
    flag_set(p_state, FLAGS_SONAR, false);
}

bool pilot_count_moved(uint8_t * p_seen, uint8_t count)
{
    bool const b_moved = (count != *p_seen);

    *p_seen = count;
    return b_moved;
}

void pilot_drive(drive_state_t * p_drive, pilot_cmd_t const * p_cmd, uint32_t dt_ms,
                 int32_t * p_left_um_s, int32_t * p_right_um_s)
{
    /* A stop for a cause must hold the car within 150 ms of 150 mm/s; the
     * slew alone would take 0.3 s. A stop with no reason is a hug pause to
     * look, and slews. */
    if (((uint8_t)PILOT_REASON_NONE != p_cmd->reason) && (0 == p_cmd->speed_mm_s) &&
        (0 == p_cmd->turn_deci_s))
    {
        drive_safety_stop(p_drive, p_left_um_s, p_right_um_s);
    }
    else if (((uint8_t)PILOT_REASON_STUCK == p_cmd->reason) &&
             ((p_drive->left_um_s > 0) || (p_drive->right_um_s > 0)))
    {
        /* The wheels do not turn: back off from rest, not through a 0.6 s
         * slew down that pushes on the blockage. */
        drive_init(p_drive);
        drive_step(p_drive, p_cmd->speed_mm_s, p_cmd->turn_deci_s, dt_ms, p_left_um_s,
                   p_right_um_s);
    }
    else if ((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == p_cmd->behaviour)
    {
        drive_follow(p_drive, p_cmd->speed_mm_s, p_cmd->turn_deci_s, dt_ms, p_left_um_s,
                     p_right_um_s);
    }
    else
    {
        drive_step(p_drive, p_cmd->speed_mm_s, p_cmd->turn_deci_s, dt_ms, p_left_um_s,
                   p_right_um_s);
    }
}

/*** end of file ***/
