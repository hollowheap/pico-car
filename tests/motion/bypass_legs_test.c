/* Independent wheel totals and echo replies at the production boundary. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "motion/pilot/hug.h"
#include "motion/pilot/bypass/internal.h"
#include "ir/ir.h"

/* Nine bearings plus the single decision-critical repeat. */
#define FAN_ALLOWANCE (10U)

typedef struct
{
    hug_t hug;
    look_t look;
    hug_in_t in;
    hug_out_t out;
    uint8_t forward_seq;
    bool drop_replies;
} fixture_t;

static void tick(fixture_t *f, int echo, int mm)
{
    f->in.now_ms += 40U;
    f->in.b_ping_reply = (f->out.b_req != 0) && !f->drop_replies;
    f->in.ping_seq = f->in.b_ping_reply ? f->out.seq : (uint8_t)(++f->forward_seq & 63U);
    f->in.ping_servo_deci = f->in.b_ping_reply ? f->out.servo_deci : 0;
    f->in.ping_echo = (uint8_t)echo;
    f->in.ping_mm = (uint16_t)mm;
    (void)hug_step(&f->hug, &f->look, &f->in, &f->out);
}

static void turn_encoder(fixture_t *f, int deci)
{
    /* Wheel displacement for a pivot about the midpoint of the 107 mm axle. */
    int64_t const wheel = ((int64_t)deci * 107000) / (2 * 573);
    f->in.left_um -= wheel;
    f->in.right_um += wheel;
}

static void translate(fixture_t *f, int mm)
{
    f->in.left_um += mm * 1000;
    f->in.right_um += mm * 1000;
}

static void wait_motion(fixture_t *f, int echo, int mm, int forward, int turn)
{
    for (unsigned i = 0; i < 100; ++i)
    {
        tick(f, echo, mm);
        assert(!f->out.b_timeout && !f->out.b_no_exit && !f->out.b_rejoined);
        if (f->out.turn_deci_s || f->out.speed_mm_s)
        {
            if ((forward ? f->out.speed_mm_s * forward <= 0 : f->out.speed_mm_s != 0) ||
                (turn ? f->out.turn_deci_s * turn <= 0 : f->out.turn_deci_s != 0))
                fprintf(stderr, "phase=%d x=%ld y=%ld speed=%d turn=%d expected=%d,%d\n",
                        f->hug.bypass.phase, (long)f->hug.x_um / 1000, (long)f->hug.y_um / 1000,
                        f->out.speed_mm_s, f->out.turn_deci_s, forward, turn);
            assert(forward ? f->out.speed_mm_s * forward > 0 : !f->out.speed_mm_s);
            assert(turn ? f->out.turn_deci_s * turn > 0 : !f->out.turn_deci_s);
            return;
        }
    }
    assert(!"no motor target after usable readings");
}

static void contact(fixture_t *f, int side, bool missing_chosen)
{
    hug_init(&f->hug);
    f->in.b_stationary = true;
    tick(f, LOOK_ECHO_VALID, 205);
    tick(f, LOOK_ECHO_VALID, 205);
    for (unsigned i = 0; i < 40U; ++i)
    {
        int const angle = f->out.b_req ? side * f->out.servo_deci : 0;
        bool const gap = (angle == -100) || (missing_chosen && (angle == 50));
        bool const open = angle >= 100;
        tick(f, gap ? LOOK_ECHO_NONE : open ? LOOK_ECHO_OPEN : LOOK_ECHO_VALID, gap || open ? 0 : 205);
        if (missing_chosen)
        {
            /* Wider views may be requested to establish another departure. */
            assert(!f->out.speed_mm_s);
        }
        else if (f->out.speed_mm_s > 0)
        {
            assert(!f->out.turn_deci_s);
            return;
        }
    }
    assert(missing_chosen);
}

/* Axle is 82 mm forward of the survey. The first surface was at x=275.
 * At 88 mm lateral displacement its bearing is about 24 degrees inward,
 * and at 120 mm it is about 32 degrees inward. */
typedef enum { NO_RETURN, CONFIRMED_EDGE, UNRELATED_RETURN, GAP_THEN_HIT } view_echo_t;

static void displaced_view(fixture_t *f, int side, int inward_deci, view_echo_t view)
{
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * (900 + inward_deci));
    unsigned rays = 0;
    for (unsigned i = 0; i < 80; ++i)
    {
        bool const aimed = f->out.b_req != 0;
        if (aimed)
        {
            assert(side * f->out.servo_deci >= 0 && abs(f->out.servo_deci) <= 200);
            ++rays;
        }
        bool const hit = aimed && view != NO_RETURN && (f->out.servo_deci == 0 ||
            (view == GAP_THEN_HIT && side * f->out.servo_deci == 100));
        int const contact_range = view == UNRELATED_RETURN ? 300 : inward_deci > 300 ? 158 : 143;
        tick(f, hit ? LOOK_ECHO_VALID : view == NO_RETURN ? LOOK_ECHO_NONE : LOOK_ECHO_OPEN,
             hit ? contact_range : 0);
        assert(!f->out.speed_mm_s && !f->out.b_timeout);
        if (f->out.turn_deci_s)
        {
            assert(f->out.turn_deci_s * side > 0 && rays >= 1);
            turn_encoder(f, side * (900 + inward_deci));
            break;
        }
        assert(i < 79);
    }
    /* A back-looking return cannot authorize the outward drive. */
    for (unsigned i = 0; i < 5; ++i)
    {
        tick(f, LOOK_ECHO_NONE, 0);
        assert(!f->out.speed_mm_s);
    }
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
}

static void second_view_and_blocked_retry(int side, bool blocked)
{
    fixture_t f = {0};
    contact(&f, side, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(&f, side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 88);
    /* No matching response ever arrives at the first shifted position. */
    f.drop_replies = true;
    displaced_view(&f, side, 245, NO_RETURN);
    f.drop_replies = false;
    if (blocked)
    {
        translate(&f, 12);
        wait_motion(&f, LOOK_ECHO_VALID, 65, -1, 0);
        translate(&f, -15);
        tick(&f, LOOK_ECHO_VALID, 65);
        assert(f.out.speed_mm_s < 0 && !f.out.turn_deci_s);
        translate(&f, -87);
        wait_motion(&f, LOOK_ECHO_VALID, 65, 0, -side);
        return;
    }
    translate(&f, 32);
    displaced_view(&f, side, 315, CONFIRMED_EDGE);
    translate(&f, 104);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(&f, -side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
}

/* At a stopped oblique view the three adjoining returns follow the side,
 * then the next bearing sees through its forward end. */
static void read_far_end(fixture_t * f, int side)
{
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(f, -side * 450);
    unsigned hits = 0;
    unsigned pings = 0;
    for (unsigned i = 0; i < 50U && f->hug.breadth_mm == 0U; i++)
    {
        int const gaze = side * f->out.servo_deci;
        bool const hit = f->out.b_req && gaze >= -200 && gaze <= -100;
        if (f->out.b_req) ++pings;
        tick(f, hit ? (int)LOOK_ECHO_VALID : (int)LOOK_ECHO_OPEN, hit ? 110 + (gaze + 200) / 10 : 0);
        if (hit) ++hits;
        assert(!f->out.speed_mm_s);
    }
    assert(hits == 3 && pings <= 5U && f->hug.breadth_mm > 0);
    assert(f->hug.side_end_x_mm == f->hug.bypass.contact_x_mm + f->hug.breadth_mm);
    assert(side * (f->hug.bypass.last_hit_deci - f->hug.bypass.requested_head) == -100);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(f, side * 450);
    wait_motion(f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(f, 255);
}

static void outward_and_parallel(int side, bool unknown_return, bool bare_floor)
{
    fixture_t f = {0};
    contact(&f, side, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
    /* Fast traverse through the open angle, with a slower final approach. */
    int fast_rate = abs(f.out.turn_deci_s);
    assert(fast_rate >= 1000);
    turn_encoder(&f, side * 830);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(side * f.out.turn_deci_s > 0 && abs(f.out.turn_deci_s) < fast_rate / 2);
    turn_encoder(&f, side * 70);
    tick(&f, LOOK_ECHO_OPEN, 0);
    /* Report coast after the heading first reaches 90 degrees. A centre
     * return at that displaced heading cannot authorize the side leg. */
    turn_encoder(&f, side * 80);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(!f.out.speed_mm_s && f.out.turn_deci_s * side < 0);
    turn_encoder(&f, -side * 80);

    /* A new obstacle or unknown forward clearance at the departure heading
     * prohibits advancing. The opposite-side gap did not veto this choice. */
    for (unsigned i = 0; i < 10; ++i)
    {
        tick(&f, i == 1 ? LOOK_ECHO_VALID : LOOK_ECHO_NONE, i == 1 ? 90 : 0);
        assert(!f.out.speed_mm_s && !f.out.turn_deci_s);
    }
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    for (unsigned i = 0; i < 12; ++i)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.out.speed_mm_s > 0 && !f.out.turn_deci_s);
    }
    translate(&f, 88);
    displaced_view(&f, side, 245, unknown_return ? UNRELATED_RETURN : CONFIRMED_EDGE);
    if (unknown_return)
    {
        translate(&f, 32);
        displaced_view(&f, side, 315, GAP_THEN_HIT);
    }
    /* Crossing the same axle coordinate distinguishes a useful measurement
     * from the conservative 228 mm fallback, including both partial steps. */
    translate(&f, unknown_return ? 95 : 127);
    tick(&f, LOOK_ECHO_OPEN, 0);
    if (unknown_return)
    {
        assert(f.out.speed_mm_s > 0 && !f.out.turn_deci_s);
        translate(&f, 15);
    }
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(&f, -side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    fixture_t extension = f;
    translate(&extension, 10);
    wait_motion(&extension, unknown_return ? LOOK_ECHO_TOO_CLOSE : LOOK_ECHO_VALID,
                unknown_return ? 0 : 87, -1, 0);
    translate(&extension, -11);
    wait_motion(&extension, LOOK_ECHO_VALID, 87, 0, side);
    turn_encoder(&extension, side * 900);
    for (unsigned i = 0; i < 5; i++)
    {
        tick(&extension, LOOK_ECHO_NONE, 0);
        assert(!extension.out.speed_mm_s);
    }
    wait_motion(&extension, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 237);
    read_far_end(&f, side);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(&f, -side * 900);

    /* Looking inward, a near obstacle makes the car turn parallel, drive a
     * measured increment and face the line again. Tape here cannot hand off. */
    f.in.b_line_left = true;
    wait_motion(&f, LOOK_ECHO_VALID, 115, 0, side);
    turn_encoder(&f, side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 110);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(&f, -side * 900);
    f.in.b_line_left = false;

    /* Centred and two displaced views cover the sweep on a +/-20 degree
     * mount. Both tracking passes use independent encoder snapshots. */
    int const angles[] = {400, -800, 400, -side * 100, -side * 100, side * 100, side * 100};
    for (unsigned i = 0; i < sizeof(angles) / sizeof(angles[0]); ++i)
    {
        wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, angles[i] > 0 ? 1 : -1);
        turn_encoder(&f, angles[i]);
    }
    if (unknown_return)
    {
        /* A lost return at the final gaze cannot authorize crossing. */
        wait_motion(&f, LOOK_ECHO_NONE, 0, 0, -side);
        turn_encoder(&f, -side * 100);
        wait_motion(&f, LOOK_ECHO_NONE, 0, 0, -side);
        turn_encoder(&f, -side * 100);
        wait_motion(&f, LOOK_ECHO_NONE, 0, 0, side);
        turn_encoder(&f, side * 100);
        wait_motion(&f, LOOK_ECHO_NONE, 0, 0, side);
        turn_encoder(&f, side * 100);
        wait_motion(&f, LOOK_ECHO_NONE, 0, 0, side);
        turn_encoder(&f, side * 900);
        wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
        return;
    }
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    if (bare_floor)
    {
        /* The inward crossing is known clear. Bring the axle back to its
         * entry lateral coordinate, then restore the original forward heading. */
        translate(&f, 180);
        tick(&f, LOOK_ECHO_OPEN, 0);
        translate(&f, 37);
        wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
        turn_encoder(&f, side * 900);
        for (unsigned i = 0; i < 5; ++i)
        {
            tick(&f, LOOK_ECHO_NONE, 0);
            assert(hug_active(&f.hug) && !f.out.speed_mm_s && !f.out.turn_deci_s);
        }
        fixture_t blocked = f;
        tick(&blocked, LOOK_ECHO_TOO_CLOSE, 0);
        assert(hug_active(&blocked.hug) && !blocked.out.b_rejoined && !blocked.out.speed_mm_s);
        assert(blocked.hug.bypass.phase == BYPASS_CONTACT);
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(!hug_active(&f.hug) && !f.out.b_rejoined && !f.out.b_no_exit);
        return;
    }
    translate(&f, unknown_return ? 98 : 83);
    f.in.b_line_left = true;
    tick(&f, LOOK_ECHO_OPEN, 0);
    translate(&f, 3);
    f.in.b_line_left = false;
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.out.speed_mm_s > 0 && !f.out.b_rejoined);
    f.in.b_line_right = true;
    tick(&f, LOOK_ECHO_OPEN, 0);
    translate(&f, 9);
    tick(&f, LOOK_ECHO_OPEN, 0);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 129);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
    assert(!f.out.b_rejoined);
    f.in.b_line_right = false;
    turn_encoder(&f, side * 400);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(!f.out.b_rejoined);
    f.in.b_line_left = true;
    turn_encoder(&f, side * 500);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.out.b_rejoined && !hug_active(&f.hug));
}

static void consumes_last_center_range(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    wait_motion(&f, LOOK_ECHO_VALID, 108, 1, 0);
    /* The same centered echo is still fresh in time. The leg has already
     * used its free distance beyond the tip and stopping reserve. */
    translate(&f, 29);
    f.in.now_ms += 210U;
    (void)hug_step(&f.hug, &f.look, &f.in, &f.out);
    assert(f.out.speed_mm_s == 0 && f.out.b_stop);
    wait_motion(&f, LOOK_ECHO_VALID, 200, 1, 0);
}

static void vanished_inside_nose(int lost_echo)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 50);
    tick(&f, LOOK_ECHO_VALID, 50);
    /* A fan of open or missing echoes cannot clear a known post that
     * was already alongside the tips. Regain a view on the approach. */
    wait_motion(&f, lost_echo, 0, -1, 0);
    translate(&f, -80);
    for (unsigned i = 0U; i < 64U && hug_active(&f.hug); ++i)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.out.speed_mm_s == 0 && f.out.turn_deci_s == 0);
    }
    assert(!hug_active(&f.hug));
}

static void blocked_departure(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 40);
    /* A second obstacle 70 mm ahead is observed from the displaced axle.
     * Retrace exactly that 40 mm before looking for a new departure. */
    wait_motion(&f, LOOK_ECHO_VALID, 70, -1, 0);
    translate(&f, -41);
    wait_motion(&f, LOOK_ECHO_VALID, 70, 0, -1);
    turn_encoder(&f, -900);
    bool selected_right = false;
    for (unsigned i = 0; i < 50 && !selected_right; ++i)
    {
        bool const b_open = f.out.b_req && f.out.servo_deci <= -100;
        tick(&f, b_open ? LOOK_ECHO_OPEN : LOOK_ECHO_VALID, b_open ? 0 : 123);
        assert(f.out.speed_mm_s == 0 && f.out.turn_deci_s <= 0);
        selected_right = f.out.turn_deci_s < 0;
    }
    assert(selected_right);
}

/* A continuous face across the usable stationary fan has no visible endpoint.
 * Give the pilot fresh independent ranges and measured axle displacement. */
static void long_wall(int missing_deci, bool isolated_far)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    for (unsigned i = 0; i < 60 && !f.out.speed_mm_s; ++i)
    {
        int const angle = f.out.b_req ? f.out.servo_deci : 0;
        tick(&f, f.out.b_req && angle == missing_deci ? LOOK_ECHO_NONE : LOOK_ECHO_VALID,
             isolated_far && angle == 200 ? 757 : 205);
        assert(!f.out.turn_deci_s);
    }
    assert(f.out.speed_mm_s > 0 && f.hug.bypass.b_wall);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_VALID, 123, 0, 1);
    turn_encoder(&f, 900);
    for (unsigned step = 0; step < 4; ++step)
    {
        for (unsigned i = 0; i < 5; ++i)
        {
            tick(&f, LOOK_ECHO_NONE, 0);
            assert(!f.out.speed_mm_s && !f.out.b_rejoined);
        }
        wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
        translate(&f, 105);
        f.in.b_line_left = step == 2; /* neighbouring tape at 300 mm */
        wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
        turn_encoder(&f, -900);
        if (step < 3)
        {
            wait_motion(&f, LOOK_ECHO_VALID, 123, 0, 1);
            turn_encoder(&f, 900);
        }
    }
    /* The first opening still needs clearance for the inward side of the car. */
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 140);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 240);
    read_far_end(&f, 1);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -900);
    int const inward[] = {400, -800, 400, -100, -100, 100, 100};
    for (unsigned i = 0; i < sizeof(inward) / sizeof(inward[0]); ++i)
    {
        wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, inward[i] > 0 ? 1 : -1);
        turn_encoder(&f, inward[i]);
    }
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 136);
    f.in.b_line_left = true; /* IR is on the parallel route */
    tick(&f, LOOK_ECHO_OPEN, 0);
    translate(&f, 255);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(!f.hug.bypass.b_exit_tape && f.out.speed_mm_s > 0);
    f.in.b_line_left = false;
    tick(&f, LOOK_ECHO_OPEN, 0);
    translate(&f, 44);
    f.in.b_line_right = true;
    tick(&f, LOOK_ECHO_OPEN, 0);
    translate(&f, 10);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.hug.bypass.b_exit_tape);

}

static void cleared_while_held(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    f.in.b_other = true;
    tick(&f, LOOK_ECHO_VALID, 100);
    tick(&f, LOOK_ECHO_VALID, 100);
    f.in.b_other = false;
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(!hug_active(&f.hug) && !f.out.b_req);
}

/* A clear inward lane by itself cannot locate the other end of a face. */
static void no_unseen_corner(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 88);
    displaced_view(&f, 1, 245, CONFIRMED_EDGE);
    translate(&f, 145);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -900);
    for (unsigned i = 0; i < 1600 && !f.out.b_no_exit && hug_active(&f.hug); i++)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.hug.bypass.phase != BYPASS_TO_LINE && !f.out.b_rejoined);
        if (f.out.turn_deci_s) turn_encoder(&f, f.out.turn_deci_s > 0 ? 10 : -10);
        if (f.out.speed_mm_s) translate(&f, f.out.speed_mm_s > 0 ? 5 : -5);
    }
}

/* The contact is on the route. At a sideways heading a 130 mm reader
 * reaches the neighbouring tape before the axle has gone 170 mm. */
static void guided_departure(int side)
{
    fixture_t f = {0};
    f.in.b_line_left = true;
    contact(&f, side, false);
    f.in.b_line_left = false;
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(&f, side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 88);
    displaced_view(&f, side, 245, CONFIRMED_EDGE);
    for (int outward = 88; outward < 175; outward += 2)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        if (f.out.turn_deci_s * side < 0) return;
        if (f.out.speed_mm_s > 0)
        {
            assert(outward + 130 + 8 < 300);
            translate(&f, 2);
        }
    }
    assert(!"no usable parallel heading inside the tape corridor");
}

static void look_back_after_coast(int side)
{
    fixture_t f = {0};
    contact(&f, side, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(&f, side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 88);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.hug.bypass.phase == BYPASS_LOOK_BACK);

    /* The bridge logged a stopped heading 1.9–2.0 degrees short of sideways. */
    turn_encoder(&f, -side * 20);
    f.in.b_stationary = false;
    tick(&f, LOOK_ECHO_VALID, 1993);
    f.in.b_stationary = true;
    f.forward_seq--;
    tick(&f, LOOK_ECHO_VALID, 1993);
    assert(!f.out.turn_deci_s && !f.out.speed_mm_s);
    tick(&f, LOOK_ECHO_VALID, 70);
    assert(!f.out.turn_deci_s && !f.out.speed_mm_s);
    tick(&f, LOOK_ECHO_VALID, 1993);
    assert(f.hug.bypass.phase == BYPASS_LOOK_BACK && f.out.turn_deci_s * side < 0);
}

static void off_center_guided_post(int side)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    f.in.b_line_left = true;
    tick(&f, LOOK_ECHO_VALID, 210);
    tick(&f, LOOK_ECHO_VALID, 210);
    for (unsigned i = 0; i < 60U && !f.out.speed_mm_s; ++i)
    {
        int const bearing = side * f.out.servo_deci;
        bool const hit = f.out.b_req && bearing <= -100;
        tick(&f, LOOK_ECHO_VALID, hit ? 193 : 3700);
    }
    assert(f.out.speed_mm_s > 0 && f.hug.side == side);
    f.in.b_line_left = false;
    translate(&f, 72);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, side);
    turn_encoder(&f, side * 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    /* Model both first looks missing the post and reaching the tape limit. */
    translate(&f, GUIDED_LATERAL_MM + 1);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(&f, -side * 900);
    fixture_t missing_forward = f;
    for (unsigned i = 0; i < 100U && hug_active(&missing_forward.hug); ++i)
    {
        tick(&missing_forward, LOOK_ECHO_NONE, 0);
        assert(missing_forward.out.speed_mm_s == 0 && missing_forward.out.turn_deci_s == 0);
        assert(missing_forward.hug.side == side);
    }
    assert(missing_forward.out.b_no_exit && !hug_active(&missing_forward.hug));
    wait_motion(&f, LOOK_ECHO_VALID, 3500, 1, 0);
    assert(f.hug.bypass.phase == BYPASS_SIDE_STEP);
    translate(&f, 50);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -side);
    turn_encoder(&f, -side * 450);
    /* A distant return beyond the post supplies the clearance transition. */
    for (unsigned i = 0; i < 40U && f.hug.bypass.phase == BYPASS_SIDE_FAN; ++i)
    {
        int const gaze = side * f.out.servo_deci;
        int const range = gaze == -150 ? 146 : gaze == -100 ? 150 : gaze == -50 ? 165 : 1400;
        tick(&f, LOOK_ECHO_VALID, range);
        assert(f.out.speed_mm_s == 0 && f.hug.side == side);
    }
    assert(f.hug.bypass.phase == BYPASS_LEVEL && f.hug.breadth_mm > 0U);
}

static void side_fan_holds_through_aim_jitter(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 240);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 120);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -450);
    /* Settle on the first ray, then bounce 17 deci-deg (past AIM_TOL=15). */
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.out.b_req && f.out.turn_deci_s == 0);
    tick(&f, LOOK_ECHO_VALID, 110);
    assert(f.look.count == 1U);
    turn_encoder(&f, -22);
    tick(&f, LOOK_ECHO_VALID, 115);
    assert(f.out.turn_deci_s == 0 && f.look.count >= 1U);
}

static void round_obstacle_look_back_without_reverse(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 82);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 240);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -900);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 120);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -450);
    /* First station sees the curved side of a round obstacle (all rays hit, no far end yet). */
    for (unsigned i = 0; i < 40U && f.hug.bypass.phase == BYPASS_SIDE_FAN; i++)
    {
        int const gaze = f.out.servo_deci;
        tick(&f, LOOK_ECHO_VALID, 120 + (gaze + 200) / 8);
    }
    assert(f.hug.bypass.phase == BYPASS_SIDE_STEP && f.hug.bypass.b_saw_side);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 450);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    translate(&f, 70);
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, -1);
    turn_encoder(&f, -450);
    /* Second station: forward-inward 45 deg fan is now all OPEN past the round obstacle. */
    for (unsigned i = 0; i < 60U && f.hug.goal_head == -450; i++)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.out.speed_mm_s >= 0);
        if (f.out.turn_deci_s != 0) turn_encoder(&f, f.out.turn_deci_s > 0 ? 15 : -15);
    }
    /* Must pivot further inward (look-back to -90 deg), never reverse. */
    assert(f.hug.goal_head == -900);
    fixture_t missing = f;
    for (unsigned i = 0; i < 150U && hug_active(&missing.hug); i++)
    {
        tick(&missing, LOOK_ECHO_NONE, 0);
        assert(missing.out.speed_mm_s == 0 && missing.hug.breadth_mm == 0U);
        if (missing.out.turn_deci_s != 0) turn_encoder(&missing, missing.out.turn_deci_s > 0 ? 15 : -15);
    }
    assert(missing.out.b_no_exit && !hug_active(&missing.hug));

    fixture_t silent = f;
    silent.drop_replies = true;
    for (unsigned i = 0; i < 150U && hug_active(&silent.hug); i++)
    {
        tick(&silent, LOOK_ECHO_OPEN, 0);
        assert(!silent.out.speed_mm_s);
        if (silent.out.turn_deci_s != 0) turn_encoder(&silent, silent.out.turn_deci_s > 0 ? 15 : -15);
    }
    assert(silent.out.b_no_exit && !hug_active(&silent.hug));

    for (unsigned i = 0; i < 80U && f.hug.bypass.phase != BYPASS_LEVEL; i++)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.out.speed_mm_s >= 0);
        if (f.out.turn_deci_s != 0) turn_encoder(&f, f.out.turn_deci_s > 0 ? 15 : -15);
    }
    assert(f.hug.bypass.phase == BYPASS_LEVEL && f.hug.breadth_mm == 0U);

    fixture_t no_clear = f;
    wait_motion(&no_clear, LOOK_ECHO_NONE, 0, 0, 1);
    turn_encoder(&no_clear, 900);
    for (unsigned i = 0; i < 100U && hug_active(&no_clear.hug); i++)
    {
        tick(&no_clear, LOOK_ECHO_NONE, 0);
        assert(!no_clear.out.speed_mm_s && !no_clear.out.turn_deci_s);
    }
    assert(no_clear.out.b_no_exit && !hug_active(&no_clear.hug));

    wait_motion(&f, LOOK_ECHO_OPEN, 0, 0, 1);
    turn_encoder(&f, 900);
    /* Clearance returns before the last measured turn's movement deadline. */
    for (unsigned i = 0; i < 20U; i++)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        assert(f.hug.bypass.phase == BYPASS_LEVEL && !f.out.speed_mm_s && !f.out.b_no_exit);
    }
    wait_motion(&f, LOOK_ECHO_OPEN, 0, 1, 0);
    // Hold the wheel totals fixed until recovery asks for a sonar probe.
    for (unsigned i = 0; i < 100U && !f.hug.bypass.b_probe; i++)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
    }
    assert(f.hug.bypass.b_probe);
    f.drop_replies = true;
    tick(&f, LOOK_ECHO_NONE, 0);
    assert(f.out.b_req);
    for (unsigned i = 0; i < 100U && hug_active(&f.hug); i++)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        assert(!f.out.speed_mm_s && !f.out.turn_deci_s);
    }
    assert(f.out.b_timeout && !hug_active(&f.hug));
}

static void test_frozen_leg_ends_at_deadline(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 2);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.out.speed_mm_s > 0 && f.out.turn_deci_s == 0);
    uint32_t const last_progress = f.in.now_ms;

    f.in.b_stationary = false;
    do
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
    } while (f.in.now_ms - last_progress < 1000U && f.out.speed_mm_s > 0);

    assert(f.out.speed_mm_s <= 0 && "the frozen leg must relinquish its forward demand by 1000 ms");
}

static void test_clearance_wait_keeps_deadline(void)
{
    fixture_t f = {0};
    contact(&f, 1, false);
    translate(&f, 2);
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.out.speed_mm_s > 0);
    uint32_t const progress_ms = f.in.now_ms;
    while (f.in.now_ms - progress_ms < 840U)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.out.speed_mm_s > 0);
    }
    f.in.b_stationary = true;
    while (f.in.now_ms - progress_ms < 1160U)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        assert(!f.out.speed_mm_s && !f.out.b_req && f.out.b_stop);
    }
    tick(&f, LOOK_ECHO_OPEN, 0);
    assert(f.out.speed_mm_s <= 0 && "a clearance wait cannot extend a frozen leg");
}

static void test_bypass_handoff_deadline(bool b_independent_hold, bool b_retreat)
{
    for (int side = -1; side <= 1; side += 2)
    {
        fixture_t f = {0};
        contact(&f, side, false);
        translate(&f, 2);
        tick(&f, LOOK_ECHO_OPEN, 0);
        assert(f.out.speed_mm_s > 0);
        uint32_t const progress_ms = f.in.now_ms;
        while (f.in.now_ms - progress_ms < 840U)
        {
            tick(&f, LOOK_ECHO_OPEN, 0);
            assert(f.out.speed_mm_s > 0);
        }
        f.in.b_stationary = true;
        f.in.b_other = true;
        f.in.b_contact_backoff = !b_independent_hold;
        f.in.b_independent_hold = b_independent_hold;
        tick(&f, LOOK_ECHO_OPEN, 0);
        while (f.in.now_ms - progress_ms < 1160U)
        {
            if (b_retreat && f.in.now_ms - progress_ms == 920U)
            {
                f.in.b_stationary = false;
                translate(&f, -2);
            }
            tick(&f, LOOK_ECHO_OPEN, 0);
            f.in.b_stationary = true;
            assert(!f.out.speed_mm_s && !f.out.turn_deci_s);
        }
        f.in.b_other = false;
        f.in.b_contact_backoff = false;
        f.in.b_independent_hold = false;
        for (unsigned i = 0U; i < 4U && !f.out.speed_mm_s; i++)
        {
            tick(&f, LOOK_ECHO_OPEN, 0);
        }
        assert((f.out.speed_mm_s > 0) == (b_independent_hold || b_retreat) &&
               "only a real hold or measured retreat preserves the forward leg");
    }
}

static void test_same_pose_cannot_repeat_fan(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    assert(f.out.b_req);
    uint32_t const start = f.in.now_ms;
    bool seen[(2 * OBSTACLE_SERVO_LIMIT_DECI / 50) + 1] = {false};
    unsigned distinct = 0U;
    unsigned repeats = 0U;
    int last_request = -1;

    while (f.in.now_ms - start < 1000U)
    {
        if (f.out.b_req && f.out.seq != last_request)
        {
            int const slot = (f.out.servo_deci + OBSTACLE_SERVO_LIMIT_DECI) / 50;
            assert(f.out.servo_deci % 50 == 0 && slot >= 0 &&
                   (unsigned)slot < sizeof(seen) / sizeof(seen[0]));
            last_request = f.out.seq;
            if (seen[slot])
            {
                ++repeats;
            }
            else
            {
                ++distinct;
            }
            seen[slot] = true;
        }
        tick(&f, LOOK_ECHO_NONE, 0);
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
    }
    assert(repeats <= 1U && "a fixed pose permits at most one repeat of a fine-fan bearing");
    assert(distinct <= 9U && "the fine-fan budget is independent of servo travel");
}

static void test_contact_fan_uses_measured_edge(int side, int open_from, int outer_bearing)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);

    bool observed_outer_edge = false;
    bool observed_open = false;
    bool departed = false;
    unsigned requests = 0U;
    for (unsigned pass = 0U; pass < 30U; ++pass)
    {
        bool const requested = f.out.b_req != 0;
        int const bearing = requested ? side * f.out.servo_deci : 0;
        if (requested)
        {
            assert(!observed_outer_edge && "a measured edge must end the fine fan on either side");
            ++requests;
        }
        bool const open = (bearing >= open_from);
        bool const unknown = (bearing < 0);
        tick(&f, open ? LOOK_ECHO_OPEN : unknown ? LOOK_ECHO_NONE : LOOK_ECHO_VALID,
             open || unknown ? 0 : 205);
        observed_open = observed_open || (requested && open);
        observed_outer_edge = observed_outer_edge || (requested && bearing == outer_bearing);
        if (f.out.speed_mm_s > 0)
        {
            departed = true;
            break;
        }
        assert(!f.out.b_rejoined);
    }
    assert(observed_open && departed && requests <= 9U && "use the supported edge within the fixed fan budget");
}

static void test_fan_glitch_keeps_deadline(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    assert(f.out.b_req);
    uint32_t const start = f.in.now_ms;
    unsigned glitches = 0U;
    unsigned requests = 1U;
    int last_seq = (int)f.out.seq;

    for (unsigned i = 0U; i < 60U; ++i)
    {
        bool const b_glitch = (0U == glitches) && (f.in.now_ms - start >= 200U);
        f.in.b_stationary = !b_glitch;
        glitches += b_glitch ? 1U : 0U;
        tick(&f, LOOK_ECHO_NONE, 0);
        if (f.out.b_req && ((int)f.out.seq != last_seq))
        {
            last_seq = (int)f.out.seq;
            ++requests;
        }
        if (f.in.now_ms - start >= 1000U)
        {
            assert(!f.out.b_req && "a glitch cannot renew the fan deadline");
        }
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
    }
    assert(1U == glitches);
    assert(requests <= FAN_ALLOWANCE && "a glitch cannot restart the fan");
}

static void test_fixed_pose_fan_once(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    assert(f.out.b_req);
    uint32_t const start = f.in.now_ms;
    unsigned requests = 1U;
    int last_seq = (int)f.out.seq;

    while (f.in.now_ms - start < 10000U)
    {
        tick(&f, LOOK_ECHO_VALID, 205);
        if (f.out.b_req && ((int)f.out.seq != last_seq))
        {
            last_seq = (int)f.out.seq;
            ++requests;
        }
    }
    assert(requests >= 9U && "the first fan must run its full budget");
    assert(requests <= FAN_ALLOWANCE && "a fixed pose gets one fan and one repeat over 10 s");
}

static void test_fixed_pose_hands_off_once(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    assert(f.out.b_req);
    f.drop_replies = true;
    uint32_t const start = f.in.now_ms;
    unsigned exits = 0U;
    unsigned requests_after_exit = 0U;

    while (f.in.now_ms - start < 10000U)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
        if (f.out.b_no_exit)
        {
            ++exits;
        }
        if ((exits > 0U) && f.out.b_req)
        {
            ++requests_after_exit;
        }
    }
    assert(1U == exits && "the fixed pose hands off exactly once");
    assert(0U == requests_after_exit && "no fan follows the handoff");
}

static void test_hold_clock_excludes_fan_gap(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = false;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    uint32_t const begin = f.in.now_ms;
    for (unsigned i = 0U; i < 8U; ++i)
    {
        tick(&f, LOOK_ECHO_VALID, 205);
    }
    f.in.b_stationary = true;
    f.drop_replies = true;
    bool exited = false;
    while (!exited && (f.in.now_ms - begin < 4000U))
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        exited = f.out.b_no_exit != 0U;
    }
    assert(exited && "the fan hands off once its recovery budget is spent");
    assert((f.in.now_ms - begin) <= 2000U + 40U && "the fan's own scan time is not recovery time");
}

static void stall_tick(fixture_t *f)
{
    f->in.b_stall = true;
    tick(f, LOOK_ECHO_OPEN, 0);
    f->in.b_stall = false;
}

static void fixed_fan_to_handoff(fixture_t *f)
{
    hug_init(&f->hug);
    f->in.b_stationary = true;
    tick(f, LOOK_ECHO_VALID, 205);
    tick(f, LOOK_ECHO_VALID, 205);
    assert(f->out.b_req);
    f->drop_replies = true;
    for (unsigned i = 0U; (i < 200U) && !f->out.b_no_exit; ++i)
    {
        tick(f, LOOK_ECHO_NONE, 0);
        assert(f->out.speed_mm_s <= 0 && !f->out.b_rejoined);
    }
    assert(f->out.b_no_exit && "a fixed fan hands off once its budget is spent");
}

static void test_fixed_pose_stall_retrigger_once(void)
{
    fixture_t f = {0};
    unsigned requests = 1U;
    unsigned ends = 0U;
    unsigned requests_after_end = 0U;
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    assert(f.out.b_req);
    f.drop_replies = true;
    int last_seq = (int)f.out.seq;
    uint32_t const start = f.in.now_ms;

    while (f.in.now_ms - start < 20000U)
    {
        if (ends > 0U)
        {
            stall_tick(&f);
        }
        else
        {
            tick(&f, LOOK_ECHO_NONE, 0);
        }
        if (f.out.b_req && ((int)f.out.seq != last_seq))
        {
            last_seq = (int)f.out.seq;
            if (ends > 0U)
            {
                ++requests_after_end;
            }
            else
            {
                ++requests;
            }
        }
        if (f.out.b_no_exit || f.out.b_timeout)
        {
            ++ends;
        }
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
    }
    assert(1U == ends && "a stalled fixed pose hands off exactly once");
    assert(0U == requests_after_end && "a stall at a fixed pose starts no second fan");
    assert(requests <= FAN_ALLOWANCE && "a fixed pose gets one fan and one repeat over 20 s");
}

static void test_fixed_pose_relook_cannot_restart_fan(void)
{
    fixture_t f = {0};
    fixed_fan_to_handoff(&f);
    uint32_t const start = f.in.now_ms;

    while (f.in.now_ms - start < 10000U)
    {
        f.in.b_relook = true;
        tick(&f, LOOK_ECHO_NONE, 0);
        f.in.b_relook = false;
        assert(!f.out.b_req && !f.out.b_timeout && !f.out.b_no_exit &&
               "a relook at an unmoved pose starts no fan");
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
    }
}

static unsigned fans_after_quiet_run(int quiet_echo)
{
    fixture_t f = {0};
    fixed_fan_to_handoff(&f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        tick(&f, quiet_echo, 0);
    }
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    uint32_t const start = f.in.now_ms;
    unsigned requests = 0U;
    int last_seq = (int)f.out.seq;

    while (f.in.now_ms - start < 5000U)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        if (f.out.b_req && ((int)f.out.seq != last_seq))
        {
            last_seq = (int)f.out.seq;
            ++requests;
        }
    }
    return requests;
}

static void test_dropouts_are_not_a_new_encounter(void)
{
    assert(0U == fans_after_quiet_run(LOOK_ECHO_NONE) &&
           "unusable echoes at an unmoved pose cannot re-arm the fan");
}

static void test_new_encounter_gets_bounded_fan(void)
{
    fixture_t f = {0};
    fixed_fan_to_handoff(&f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        tick(&f, LOOK_ECHO_OPEN, 0);
    }
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    uint32_t const start = f.in.now_ms;
    unsigned requests = 0U;
    unsigned exits = 0U;
    int last_seq = (int)f.out.seq;

    while (f.in.now_ms - start < 5000U)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        if (f.out.b_req && ((int)f.out.seq != last_seq))
        {
            last_seq = (int)f.out.seq;
            ++requests;
        }
        exits += f.out.b_no_exit ? 1U : 0U;
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
    }
    assert(requests > 0U && "a contact after a clear interval gets a fan");
    assert(1U == exits && "that fan ends in one bounded handoff");
}

static void test_nonstationary_expired_fan_hands_off_once(void)
{
    fixture_t f = {0};
    hug_init(&f.hug);
    f.in.b_stationary = true;
    tick(&f, LOOK_ECHO_VALID, 205);
    tick(&f, LOOK_ECHO_VALID, 205);
    assert(f.out.b_req);
    f.in.b_stationary = false;
    f.drop_replies = true;
    uint32_t const start = f.in.now_ms;
    unsigned exits = 0U;
    uint32_t exit_ms = 0U;

    while (f.in.now_ms - start < 10000U)
    {
        tick(&f, LOOK_ECHO_NONE, 0);
        assert(f.out.speed_mm_s <= 0 && !f.out.b_rejoined);
        if (f.out.b_no_exit)
        {
            ++exits;
            exit_ms = f.in.now_ms - start;
        }
    }
    assert(1U == exits && "an expired fan hands off once while the wheels move");
    assert(exit_ms <= 2000U + 40U &&
           "the handoff comes within the deadline plus the recovery budget");
}

static bool begin_output_within(fixture_t *f, uint32_t ms)
{
    uint32_t const start = f->in.now_ms;
    bool seen = false;

    while (!seen && (f->in.now_ms - start < ms))
    {
        tick(f, LOOK_ECHO_NONE, 0);
        seen = f->out.b_req || f->out.b_timeout || f->out.b_no_exit;
    }
    return seen;
}

static void test_single_wheel_travel_keeps_fan_closed(void)
{
    fixture_t f = {0};
    fixed_fan_to_handoff(&f);
    tick(&f, LOOK_ECHO_NONE, 0);
    f.in.left_um += 3000;
    stall_tick(&f);
    assert(!begin_output_within(&f, 3000U) && "one wheel's travel cannot reopen a fan at its pose");
    translate(&f, 3);
    stall_tick(&f);
    assert(begin_output_within(&f, 3000U) && "travel on both required wheels reopens the fan");
}

int main(int argc, char *argv[])
{
    if (argc == 2 && strcmp(argv[1], "frozen-leg-ends-at-deadline") == 0)
    {
        test_frozen_leg_ends_at_deadline();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "clearance-wait-keeps-deadline") == 0)
    {
        test_clearance_wait_keeps_deadline();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "contact-handoff-keeps-deadline") == 0)
    {
        test_bypass_handoff_deadline(false, false);
        test_bypass_handoff_deadline(false, true);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "independent-hold-pauses-deadline") == 0)
    {
        test_bypass_handoff_deadline(true, false);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "fan-glitch-keeps-deadline") == 0)
    {
        test_fan_glitch_keeps_deadline();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "fixed-pose-fan-once") == 0)
    {
        test_fixed_pose_fan_once();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "hold-clock-excludes-fan-gap") == 0)
    {
        test_hold_clock_excludes_fan_gap();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "fixed-pose-hands-off-once") == 0)
    {
        test_fixed_pose_hands_off_once();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "same-pose-cannot-repeat-fan") == 0)
    {
        test_same_pose_cannot_repeat_fan();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "contact-fan-uses-measured-edge") == 0)
    {
        test_contact_fan_uses_measured_edge(1, 150, 200);
        test_contact_fan_uses_measured_edge(-1, 150, 200);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "qualified-fan-reaches-outer-edge") == 0)
    {
        assert(OBSTACLE_SERVO_LIMIT_DECI >= 300 && "run this case with the qualified 30-degree cap");
        test_contact_fan_uses_measured_edge(1, 250, 300);
        test_contact_fan_uses_measured_edge(-1, 250, 300);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "contact-fan-preserves-unknown") == 0)
    {
        fixture_t f = {0};
        contact(&f, 1, true);
        memset(&f, 0, sizeof(f));
        contact(&f, -1, true);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "fixed-pose-stall-retrigger-once") == 0)
    {
        test_fixed_pose_stall_retrigger_once();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "fixed-pose-relook-cannot-restart-fan") == 0)
    {
        test_fixed_pose_relook_cannot_restart_fan();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "dropouts-are-not-a-new-encounter") == 0)
    {
        test_dropouts_are_not_a_new_encounter();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "new-encounter-gets-bounded-fan") == 0)
    {
        test_new_encounter_gets_bounded_fan();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "nonstationary-expired-fan-hands-off-once") == 0)
    {
        test_nonstationary_expired_fan_hands_off_once();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "single-wheel-travel-keeps-fan-closed") == 0)
    {
        test_single_wheel_travel_keeps_fan_closed();
        return EXIT_SUCCESS;
    }
    if (argc != 1)
    {
        return EXIT_FAILURE;
    }
    no_unseen_corner();
    test_clearance_wait_keeps_deadline();
    test_bypass_handoff_deadline(false, false);
    test_bypass_handoff_deadline(false, true);
    test_bypass_handoff_deadline(true, false);
    /* On the right bend the protruding left reader sweeps into this echo.
     * The mirrored left bend clears it on the ordinary tire side. */
    for (unsigned left = 0U; left <= 1U; left++)
    {
        fixture_t f = {0};
        hug_init(&f.hug);
        f.in.bend_radius_mm = 120U;
        f.in.b_line_left = left != 0U;
        f.in.b_line_right = !f.in.b_line_left;
        tick(&f, LOOK_ECHO_VALID, 215);
        tick(&f, LOOK_ECHO_VALID, 215);
        assert(hug_active(&f.hug) == !f.in.b_line_left);
    }
    for (int side = -1; side <= 1; side += 2)
    {
        look_back_after_coast(side);
        off_center_guided_post(side);
        guided_departure(side);
        fixture_t unknown = {0};
        contact(&unknown, side, true);
        outward_and_parallel(side, false, false);
        outward_and_parallel(side, true, false);
        outward_and_parallel(side, false, true);
        second_view_and_blocked_retry(side, false);
        second_view_and_blocked_retry(side, true);
    }
    long_wall(999, false);
    long_wall(-50, false);
    long_wall(0, false);
    long_wall(999, true);
    cleared_while_held();
    consumes_last_center_range();
    vanished_inside_nose(LOOK_ECHO_OPEN);
    vanished_inside_nose(LOOK_ECHO_NONE);
    blocked_departure();
    side_fan_holds_through_aim_jitter();
    round_obstacle_look_back_without_reverse();
    return 0;
}
