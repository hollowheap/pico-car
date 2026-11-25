/** @file test_pilot_rejoin.c
 * @brief Host tests for the heading report after an obstacle bypass rejoins
 *        the line, and for the recovery from a wrong-way rejoin.
 *
 * The scripted bypass drives pilot_step with the wheel totals and sonar replies
 * the car would report.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "navigation/obstacle_ping.h"
#include "pilot/hug.h"
#include "pilot/pilot.h"
#include "platform/chassis.h"
#include "platform/units.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define TICK_MS           (40U)
#define REPORT_TOL_DECI   (2)
#define REPORT_LATERAL_MM (20)
#define FOLLOW_HEAD_DECI  (30)
#define WRONG_HEAD_DECI   (1700)
#define CONTACT_MM        (205)
#define FAR_RANGE_MM      (143)
#define NEAR_RANGE_MM     (158)
#define INWARD_NEAR_DECI  (300)
#define CONTACT_TURN_DECI (900)
/* Starting the crossing 30 mm short places the settle 20 mm off the entry line. */
#define CROSSING_START_MM (53)

typedef struct
{
    pilot_state_t pilot;
    pilot_meas_t  meas;
    pilot_cmd_t   cmd;
    uint8_t       forward_seq;
    unsigned      reports;
    int16_t       report_head_deci;
    int16_t       report_y_mm;
} fixture_t;

static hug_t * hug_of(fixture_t * f)
{
    return &f->pilot.avoidance.hug;
}

static void fixture_init(fixture_t * f)
{
    memset(f, 0, sizeof(*f));
    pilot_init(&f->pilot);
}

static void tick(fixture_t * f, unsigned echo, int mm)
{
    bool const b_reply = (0U != f->cmd.b_ping_req);

    f->meas.now_ms += TICK_MS;
    f->meas.stamp_ms = f->meas.now_ms;
    f->meas.b_ping_reply = b_reply;
    f->meas.ping_seq = b_reply ? f->cmd.ping_seq : (uint8_t)(++f->forward_seq & OBSTACLE_PING_SEQ_MK);
    f->meas.ping_servo_deci = b_reply ? f->cmd.ping_servo_deci : 0;
    f->meas.ping_echo = (uint8_t)echo;
    f->meas.ping_mm = (uint16_t)mm;
    pilot_step(&f->pilot, &f->meas, &f->cmd);
    if (f->cmd.b_rejoin_check)
    {
        f->reports++;
        f->report_head_deci = f->cmd.rejoin_head_deci;
        f->report_y_mm = f->cmd.rejoin_y_mm;
    }
}

/* Wheel displacement for a pivot about the midpoint of the axle. */
static void turn_encoder(fixture_t * f, int deci)
{
    int64_t const wheel = ((int64_t)deci * CHASSIS_WHEEL_TRACK_MM * UM_PER_MM) / (2 * DECI_PER_RAD);

    f->meas.left_um -= wheel;
    f->meas.right_um += wheel;
}

static void translate(fixture_t * f, int mm)
{
    f->meas.left_um += (int64_t)mm * UM_PER_MM;
    f->meas.right_um += (int64_t)mm * UM_PER_MM;
}

static void wait_motion(fixture_t * f, unsigned echo, int mm, int forward, int turn)
{
    for (unsigned i = 0; i < 100U; ++i)
    {
        tick(f, echo, mm);
        assert(!f->cmd.b_rejoined);
        assert((uint8_t)PILOT_REASON_HUG_TIMEOUT != f->cmd.reason);
        assert((uint8_t)PILOT_BEHAVIOUR_REJOIN_SEARCH != f->cmd.behaviour);
        if (f->cmd.turn_deci_s || f->cmd.speed_mm_s)
        {
            assert(forward ? f->cmd.speed_mm_s * forward > 0 : 0 == f->cmd.speed_mm_s);
            assert(turn ? f->cmd.turn_deci_s * turn > 0 : 0 == f->cmd.turn_deci_s);
            return;
        }
    }
    assert(!"no motor target after usable readings");
}

static void contact(fixture_t * f, int side)
{
    f->meas.b_stationary = true;
    tick(f, LOOK_ECHO_VALID, CONTACT_MM);
    tick(f, LOOK_ECHO_VALID, CONTACT_MM);
    for (unsigned i = 0; i < 40U; ++i)
    {
        int const angle = (0U != f->cmd.b_ping_req) ? side * f->cmd.ping_servo_deci : 0;
        bool const gap = (angle == -100);
        bool const open = angle >= 100;
        tick(f, gap ? LOOK_ECHO_NONE : open ? LOOK_ECHO_OPEN : LOOK_ECHO_VALID,
             (gap || open) ? 0 : CONTACT_MM);
        if (f->cmd.speed_mm_s > 0)
        {
            assert(!f->cmd.turn_deci_s);
            return;
        }
    }
    assert(!"no departure after the contact survey");
}

static void displaced_view(fixture_t * f, int side, int inward_deci)
{
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * (CONTACT_TURN_DECI + inward_deci));
    unsigned rays = 0;
    for (unsigned i = 0; i < 80U; ++i)
    {
        bool const aimed = (0U != f->cmd.b_ping_req);
        if (aimed)
        {
            assert(side * f->cmd.ping_servo_deci >= 0 && abs(f->cmd.ping_servo_deci) <= 200);
            ++rays;
        }
        bool const hit = aimed && (0 == f->cmd.ping_servo_deci);
        int const contact_range = (inward_deci > INWARD_NEAR_DECI) ? NEAR_RANGE_MM : FAR_RANGE_MM;
        tick(f, hit ? LOOK_ECHO_VALID : LOOK_ECHO_OPEN, hit ? contact_range : 0);
        assert(!f->cmd.speed_mm_s && (uint8_t)PILOT_REASON_HUG_TIMEOUT != f->cmd.reason);
        if (f->cmd.turn_deci_s)
        {
            assert(f->cmd.turn_deci_s * side > 0 && rays >= 1);
            turn_encoder(f, side * (CONTACT_TURN_DECI + inward_deci));
            break;
        }
        assert(i < 79);
    }
    for (unsigned i = 0; i < 5U; ++i)
    {
        tick(f, LOOK_ECHO_NONE, 0);
        assert(!f->cmd.speed_mm_s);
    }
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
}

static void read_far_end(fixture_t * f, int side)
{
    hug_t const * const h = hug_of(f);

    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * 450);
    unsigned hits = 0;
    unsigned pings = 0;
    for (unsigned i = 0; i < 50U && h->breadth_mm == 0U; i++)
    {
        int const gaze = side * f->cmd.ping_servo_deci;
        bool const hit = (0U != f->cmd.b_ping_req) && gaze >= -200 && gaze <= -100;
        if (0U != f->cmd.b_ping_req)
        {
            ++pings;
        }
        tick(f, hit ? LOOK_ECHO_VALID : LOOK_ECHO_OPEN, hit ? 110 + (gaze + 200) / 10 : 0);
        if (hit)
        {
            ++hits;
        }
        assert(!f->cmd.speed_mm_s);
    }
    assert(hits == 3 && pings <= 5U && h->breadth_mm > 0);
    assert(h->side_end_x_mm == h->bypass.contact_x_mm + h->breadth_mm);
    assert(side * (h->bypass.last_hit_deci - h->bypass.requested_head) == -100);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(f, side * 450);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(f, 255);
}

/* The outward leg, the parallel leg and the departure back to the line. */
static void to_parallel(fixture_t * f, int side)
{
    contact(f, side);
    translate(f, 82);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, side);
    int const fast_rate = abs(f->cmd.turn_deci_s);
    assert(fast_rate >= 1000);
    turn_encoder(f, side * 830);
    tick(f, LOOK_ECHO_OPEN, 0);
    assert(side * f->cmd.turn_deci_s > 0 && abs(f->cmd.turn_deci_s) < fast_rate / 2);
    turn_encoder(f, side * 70);
    tick(f, LOOK_ECHO_OPEN, 0);
    turn_encoder(f, side * 80);
    tick(f, LOOK_ECHO_OPEN, 0);
    assert(!f->cmd.speed_mm_s && f->cmd.turn_deci_s * side < 0);
    turn_encoder(f, -side * 80);
    for (unsigned i = 0; i < 10U; ++i)
    {
        tick(f, i == 1 ? LOOK_ECHO_VALID : LOOK_ECHO_NONE, i == 1 ? 90 : 0);
        assert(!f->cmd.speed_mm_s && !f->cmd.turn_deci_s);
    }
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
    for (unsigned i = 0; i < 12U; ++i)
    {
        tick(f, LOOK_ECHO_OPEN, 0);
        assert(f->cmd.speed_mm_s > 0 && !f->cmd.turn_deci_s);
    }
    translate(f, 88);
    displaced_view(f, side, 245);
    translate(f, 127);
    tick(f, LOOK_ECHO_OPEN, 0);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * 900);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(f, 237);
    read_far_end(f, side);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * 900);

    f->meas.b_line_left = true;
    wait_motion(f, LOOK_ECHO_VALID, 115, 0, side);
    turn_encoder(f, side * 900);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(f, 110);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * 900);
    f->meas.b_line_left = false;

    int const angles[] = {400, -800, 400, -side * 100, -side * 100, side * 100, side * 100};
    for (unsigned i = 0; i < sizeof(angles) / sizeof(angles[0]); ++i)
    {
        wait_motion(f, LOOK_ECHO_OPEN, 0, 0, angles[i] > 0 ? 1 : -1);
        turn_encoder(f, angles[i]);
    }
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
}

/* The obstacle is cleared without a crossing: the bypass ends and hands back
 * the wheels with no exit tape. */
static void bare_floor_exit(fixture_t * f, int side)
{
    translate(f, 180);
    tick(f, LOOK_ECHO_OPEN, 0);
    translate(f, 37);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(f, side * 900);
    for (unsigned i = 0; i < 5U; ++i)
    {
        tick(f, LOOK_ECHO_NONE, 0);
        assert(hug_active(hug_of(f)) && !f->cmd.speed_mm_s && !f->cmd.turn_deci_s);
    }
    fixture_t blocked = *f;
    tick(&blocked, LOOK_ECHO_TOO_CLOSE, 0);
    assert(hug_active(hug_of(&blocked)) && !blocked.cmd.b_rejoined && !blocked.cmd.speed_mm_s);
    assert(BYPASS_CONTACT == hug_of(&blocked)->bypass.phase);
    tick(f, LOOK_ECHO_OPEN, 0);
    assert(!hug_active(hug_of(f)) && !f->cmd.b_rejoined);
}

/* The crossing settles the axle on the exit tape. With b_early the tape is
 * raised before the second exit turn, so the settle is taken at the heading
 * of the first turn rather than the entry heading. */
static void cross_exit(fixture_t * f, int side, bool b_early)
{
    translate(f, CROSSING_START_MM);
    f->meas.b_line_left = true;
    tick(f, LOOK_ECHO_OPEN, 0);
    translate(f, 3);
    f->meas.b_line_left = false;
    tick(f, LOOK_ECHO_OPEN, 0);
    assert(f->cmd.speed_mm_s > 0 && !f->cmd.b_rejoined);
    f->meas.b_line_right = true;
    tick(f, LOOK_ECHO_OPEN, 0);
    translate(f, 9);
    tick(f, LOOK_ECHO_OPEN, 0);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(f, 129);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, side);
    assert(!f->cmd.b_rejoined);
    f->meas.b_line_right = false;
    turn_encoder(f, side * 400);
    tick(f, LOOK_ECHO_OPEN, 0);
    assert(!f->cmd.b_rejoined);
    f->meas.b_line_left = true;
    if (!b_early)
    {
        turn_encoder(f, side * 500);
    }
    for (unsigned i = 0; i < 5U && !f->cmd.b_rejoined; ++i)
    {
        tick(f, LOOK_ECHO_OPEN, 0);
    }
    assert(f->cmd.b_rejoined && !hug_active(hug_of(f)));
}

static void bypass_to_exit(fixture_t * f, int side, bool b_early)
{
    to_parallel(f, side);
    cross_exit(f, side, b_early);
}

static void bypass_without_exit(fixture_t * f, int side)
{
    to_parallel(f, side);
    bare_floor_exit(f, side);
}

/* Turn the car so its heading relative to the entry line becomes target_deci,
 * then travel the hold distance in two steps: the first stays short of it. */
static void follow_to_hold(fixture_t * f, int target_deci)
{
    int32_t const settle_deci = (int32_t)wheel_yaw_deci(hug_of(f)->diff_um);

    f->meas.pivot_scale_pct = 100;
    turn_encoder(f, target_deci - settle_deci);
    translate(f, PILOT_REJOIN_HOLD_MM - 1);
    tick(f, LOOK_ECHO_OPEN, 0);
    assert(0U == f->reports);
    translate(f, 1);
    tick(f, LOOK_ECHO_OPEN, 0);
}

static void outcome_reported(int side, bool b_early)
{
    fixture_t f;

    fixture_init(&f);
    bypass_to_exit(&f, side, b_early);
    assert(0U == f.reports);
    assert(b_early == (0 != wheel_yaw_deci(hug_of(&f)->diff_um)));
    assert(side * REPORT_LATERAL_MM == hug_of(&f)->y_um / UM_PER_MM);

    follow_to_hold(&f, FOLLOW_HEAD_DECI);
    assert(1U == f.reports);
    assert(abs(f.report_head_deci - FOLLOW_HEAD_DECI) <= REPORT_TOL_DECI);
    assert(side * REPORT_LATERAL_MM == f.report_y_mm);
    assert((uint8_t)PILOT_BEHAVIOUR_FOLLOW_LINE == f.cmd.behaviour);
    assert(!line_follow_local_search(&f.pilot.follow));

    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(1U == f.reports);
}

static void wrong_way_searches(int side, bool b_early)
{
    fixture_t f;

    fixture_init(&f);
    bypass_to_exit(&f, side, b_early);
    assert(0U == f.reports);

    follow_to_hold(&f, WRONG_HEAD_DECI);
    assert(1U == f.reports);
    assert(abs(f.report_head_deci - WRONG_HEAD_DECI) <= REPORT_TOL_DECI);
    assert(side * REPORT_LATERAL_MM == f.report_y_mm);
    assert(line_follow_local_search(&f.pilot.follow));
    assert((uint8_t)PILOT_BEHAVIOUR_REJOIN_SEARCH == f.cmd.behaviour);
    assert((uint8_t)PILOT_REASON_REJOIN_SEARCH == f.cmd.reason);

    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(1U == f.reports);
}

static void bypass_end_silent(int side)
{
    fixture_t f;

    fixture_init(&f);
    bypass_without_exit(&f, side);
    assert(0U == f.reports);
    translate(&f, 2 * PILOT_REJOIN_HOLD_MM);
    for (unsigned i = 0; i < 10U; ++i)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
    }
    assert(0U == f.reports);
}

int main(int argc, char ** argv)
{
    char const * const which = (argc > 1) ? argv[1] : "";

    for (int side = 1; side >= -1; side -= 2)
    {
        if (0 == strcmp(which, "outcome-reported"))
        {
            outcome_reported(side, false);
            outcome_reported(side, true);
        }
        else if (0 == strcmp(which, "wrong-way-searches"))
        {
            wrong_way_searches(side, false);
            wrong_way_searches(side, true);
        }
        else if (0 == strcmp(which, "bypass-end-silent"))
        {
            bypass_end_silent(side);
        }
        else
        {
            return 2;
        }
    }
    return 0;
}

/*** end of file ***/
