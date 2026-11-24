/* Independent wheel totals and echo replies at the production boundary. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "pilot/hug.h"

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
    for (unsigned i = 0; i < 50U && f->hug.breadth_mm == 0U; i++)
    {
        int const gaze = side * f->out.servo_deci;
        bool const hit = f->out.b_req && gaze >= -200 && gaze <= -100;
        tick(f, hit ? (int)LOOK_ECHO_VALID : (int)LOOK_ECHO_OPEN, hit ? 110 + (gaze + 200) / 10 : 0);
        if (hit) ++hits;
        assert(!f->out.speed_mm_s);
    }
    assert(hits == 3 && f->hug.breadth_mm > 0);
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

int main(void)
{
    no_unseen_corner();
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
    return 0;
}
