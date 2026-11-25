/** @file test_obstacle_follow.c
 *
 * @brief Host tests for the Bug2 face follow: the leg, the
 *        glance toward the face, the slant update, the outer and inner
 *        corners and the leave on the tape. The pose is set by hand; the
 *        full drive is in test_obstacle_bypass_sim.c.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "follow.h"
#include "geometry.h"
#include "obstacle_config.h"
#include "obstacle_pack.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            (void)fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,   \
                          __LINE__, #cond);                                \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static follow_in_t pose(int32_t x_mm, int32_t y_mm, int32_t head_deci)
{
    follow_in_t in = {0};

    in.x_mm      = x_mm;
    in.y_mm      = y_mm;
    in.head_deci = head_deci;
    in.cls       = OBSTACLE_CLASS_OPEN;
    return in;
}

static follow_in_t with_ping(follow_in_t in, uint8_t cls, uint16_t dist_mm)
{
    in.b_ping  = true;
    in.cls     = cls;
    in.dist_mm = dist_mm;
    return in;
}

/* Passing on the left, the first face is the box front: the car travels at
 * +90 degrees with the face on its right. */
static void start_left(follow_t * p_f)
{
    follow_in_t const in = pose(0, 0, 900);

    follow_start(p_f, true, &in, 0);
}

/* As start_left, with the front face seen at the stop, 250 mm ahead. */
static void start_left_seen(follow_t * p_f)
{
    follow_in_t const in = pose(0, 0, 900);

    follow_start(p_f, true, &in, 250);
}

/* Step at a fixed pose and return the output. */
static follow_out_t step(follow_t * p_f, follow_in_t in)
{
    follow_out_t out;

    follow_step(p_f, &in, &out);
    return out;
}

/* A leg drives forward with the servo at its limit toward the face. */
static void test_leg_drives_with_servo_at_face(void)
{
    follow_t           f;
    follow_out_t const out = (start_left(&f), step(&f, pose(0, 0, 900)));

    CHECK(out.result == FOLLOW_RUN);
    CHECK(!out.b_turning);
    CHECK(out.left_um_s > 0);
    CHECK(out.right_um_s > 0);
    /* Face on the right: the servo looks right (negative), at the limit. */
    CHECK(out.servo_deci == -(int16_t)OBSTACLE_FOLLOW_LOOK_DECI);
}

/* After a leg the car arcs toward the face while it keeps moving. */
static void test_leg_then_glance_arcs_toward_face(void)
{
    follow_t     f;
    follow_out_t out;

    start_left(&f);
    (void)step(&f, pose(0, 0, 900));
    /* Travel at +90 is toward +x. */
    out = step(&f, pose((int32_t)OBSTACLE_FOLLOW_LEG_MM + 1, 0, 900));
    CHECK(!out.b_turning);
    CHECK(out.right_um_s > 0);
    /* Toward a face on the right: the left wheel runs faster. */
    CHECK(out.left_um_s > out.right_um_s);
}

/* Drive the follower into the glance at the toward heading. */
static void to_glance(follow_t * p_f, int32_t x_mm)
{
    start_left(p_f);
    (void)step(p_f, pose(0, 0, 900));
    (void)step(p_f, pose(x_mm, 0, 900));
    (void)step(p_f, pose(x_mm, 0, 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI)); /* settled */
}

/* After the glance misses on a seen face: back up, turn deeper (look_head)
 * and miss again, which confirms a corner. */
static void confirm_corner(follow_t * p_f, int32_t x_mm, int32_t look_head)
{
    uint32_t idx = 0U;

    (void)step(p_f, pose(x_mm, -70, look_head + 600)); /* backed up: start the turn */
    (void)step(p_f, pose(x_mm, -70, look_head));       /* turned, settled */
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        (void)step(p_f, with_ping(pose(x_mm, -70, look_head), OBSTACLE_CLASS_OPEN, 0U));
    }
}

/* The same, with the front face seen at the stop. */
static void to_glance_seen(follow_t * p_f, int32_t x_mm)
{
    start_left_seen(p_f);
    (void)step(p_f, pose(0, 0, 900));
    (void)step(p_f, pose(x_mm, 0, 900));
    (void)step(p_f, pose(x_mm, 0, 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI)); /* settled */
}

/* A face echo in the glance arcs the car back, away from the face. */
static void test_echo_in_glance_arcs_back(void)
{
    follow_t     f;
    follow_out_t out;
    int32_t const x = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;

    to_glance(&f, x);
    out = step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_VALID, 200U));
    CHECK(out.right_um_s > out.left_um_s);
    CHECK(f.b_point);
}

/* Two face points on a line slanted 30 degrees change the face direction. */
static void test_slanted_face_updates_direction(void)
{
    follow_t      f;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    int32_t const before  = 900;

    to_glance(&f, x);
    (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_VALID, 200U));
    {
        int32_t const first_x = f.face_x_mm;
        int32_t const first_y = f.face_y_mm;

        /* A second glance 400 mm on, where the face has come closer: it
         * crosses the path from the right, so the car must turn left. */
        (void)step(&f, pose(x, 0, 900));
        (void)step(&f, pose(x + 200, 0, 900));
        (void)step(&f, pose(x + 400, 0, in_head));
        (void)step(&f, with_ping(pose(x + 400, 0, in_head), OBSTACLE_CLASS_VALID, 66U));
        CHECK((f.face_x_mm != first_x) || (f.face_y_mm != first_y));
    }
    /* The face direction turns left, by less than the slant limit. */
    CHECK(f.face_head_deci > before);
    CHECK(f.face_head_deci < (before + (int32_t)OBSTACLE_FOLLOW_SLANT_MAX_DECI));
}

/* No face echo in the glance: drive past the corner, then turn toward it. */
static void test_lost_face_turns_the_corner(void)
{
    follow_t      f;
    follow_out_t  out;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    uint32_t      idx     = 0U;
    int16_t       face_x  = 0;

    to_glance_seen(&f, x);
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        out = step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
    }
    /* First it backs up to look deeper. */
    CHECK(out.left_um_s < 0);
    CHECK(out.right_um_s < 0);
    confirm_corner(&f, x, in_head - (int32_t)OBSTACLE_FOLLOW_DEEPER_DECI);
    /* Past the corner: straight on the face direction, not turning yet. */
    out = step(&f, pose(x, 0, 900));
    CHECK(!out.b_turning);
    CHECK(out.left_um_s > 0);
    CHECK(out.right_um_s > 0);
    face_x = f.face_x_mm;
    out    = step(&f, pose(face_x + (int32_t)OBSTACLE_FOLLOW_CORNER_PAST_MM + 1, 0, 900));
    /* Turn toward the face (right) on the spot. */
    CHECK(out.b_turning);
    CHECK(out.left_um_s > 0);
    CHECK(out.right_um_s < 0);
}

/* An echo close ahead during a leg is an inner corner: turn away. */
static void test_echo_ahead_turns_away(void)
{
    follow_t     f;
    follow_out_t out;

    start_left(&f);
    out = step(&f, with_ping(pose(0, 0, 900), OBSTACLE_CLASS_VALID,
                             (uint16_t)(OBSTACLE_FOLLOW_INNER_MM - 1U)));
    CHECK(out.b_turning);
    /* Away from a face on the right is a left turn. */
    CHECK(out.left_um_s < 0);
    CHECK(out.right_um_s > 0);
}

/* The tape before the hit point does not end the bypass; past it, the car
 * creeps onto the tape, turns to heading 0 and hands back. */
static void test_leave_only_past_the_hit_point(void)
{
    follow_t      f;
    follow_out_t  out;
    follow_in_t   in   = pose(0, 0, 900);
    int32_t const y    = (int32_t)OBSTACLE_FOLLOW_LEAVE_AHEAD_MM + 1;
    uint32_t      pass = 0U;

    start_left(&f);
    in.b_line = true;
    out       = step(&f, in);
    CHECK(out.result == FOLLOW_RUN);
    CHECK(!out.b_turning);

    /* On the far face, heading -90, past the hit point. */
    in = pose(40, y, -900);
    in.b_line = true;
    out = step(&f, in);
    CHECK(out.result == FOLLOW_RUN);
    CHECK(out.left_um_s > 0);
    CHECK(out.right_um_s > 0);

    in = pose(40 - (int32_t)OBSTACLE_FOLLOW_LINE_LEAD_MM - 1, y, -900);
    out = step(&f, in);
    CHECK(out.b_turning);
    CHECK(out.right_um_s > 0); /* back to heading 0 is a left turn */

    for (pass = 0U; (pass < 3U) && (out.result != FOLLOW_LINE); pass++)
    {
        out = step(&f, pose(in.x_mm, y, 0));
    }
    CHECK(out.result == FOLLOW_LINE);
    CHECK(out.left_um_s == 0);
    CHECK(out.right_um_s == 0);
}

/* The mirror case: passing on the right, the servo looks left. */
static void test_right_side_mirrors(void)
{
    follow_t           f;
    follow_in_t const  in  = pose(0, 0, -900);
    follow_out_t       out;

    follow_start(&f, false, &in, 0);
    out = step(&f, in);
    CHECK(out.servo_deci == (int16_t)OBSTACLE_FOLLOW_LOOK_DECI);
    out = step(&f, pose(-(int32_t)OBSTACLE_FOLLOW_LEG_MM - 1, 0, -900));
    CHECK(out.right_um_s > out.left_um_s);
}

/* On a leg the car holds OBSTACLE_FOLLOW_HOLD_MM: too far leans toward the
 * face (right), too close leans away (left). */
static void test_leg_trim_holds_the_gap(void)
{
    follow_t      f;
    follow_out_t  out;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;

    to_glance(&f, x);
    (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_VALID, 200U));
    (void)step(&f, pose(x, 0, 900)); /* back on the leg; the face is ~223 mm */
    out = step(&f, pose(x, -100, 900));
    CHECK(out.left_um_s > out.right_um_s);
    out = step(&f, pose(x, 50, 900));
    CHECK(out.right_um_s > out.left_um_s);
}

/* The corner needs exactly OBSTACLE_FOLLOW_MISS_MAX glance pings without the
 * face; a step without a ping does not count. */
static void test_corner_on_the_last_miss(void)
{
    follow_t      f;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    uint32_t      idx     = 0U;

    int32_t const look    = in_head - (int32_t)OBSTACLE_FOLLOW_DEEPER_DECI;

    to_glance_seen(&f, x);
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
    }
    (void)step(&f, pose(x, -70, look + 600));
    (void)step(&f, pose(x, -70, look));
    for (idx = 0U; idx < (OBSTACLE_FOLLOW_MISS_MAX - 1U); idx++)
    {
        (void)step(&f, with_ping(pose(x, -70, look), OBSTACLE_CLASS_OPEN, 0U));
        (void)step(&f, pose(x, -70, look));
    }
    CHECK(f.face_x_mm == 0); /* still the seed point */
    (void)step(&f, with_ping(pose(x, -70, look), OBSTACLE_CLASS_OPEN, 0U));
    CHECK(f.face_x_mm > x); /* the corner reference is where the beam met the face line */
}

/* TOO_CLOSE on a leg is a face ahead, whatever its distance field says. */
static void test_too_close_is_a_face_ahead(void)
{
    follow_t     f;
    follow_out_t out;

    start_left(&f);
    out = step(&f, with_ping(pose(0, 0, 900), OBSTACLE_CLASS_TOO_CLOSE, 4000U));
    CHECK(out.b_turning);
    CHECK(out.right_um_s > 0);
}

/* A face ahead while the car drives past a corner turns it away too. */
static void test_face_ahead_during_corner(void)
{
    follow_t      f;
    follow_out_t  out;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    uint32_t      idx     = 0U;

    to_glance_seen(&f, x);
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
    }
    confirm_corner(&f, x, in_head - (int32_t)OBSTACLE_FOLLOW_DEEPER_DECI);
    out = step(&f, with_ping(pose(x + 20, 0, 900), OBSTACLE_CLASS_VALID, 100U));
    CHECK(out.b_turning);
    CHECK(out.right_um_s > 0); /* away from a face on the right */
}

/* A tape away from the original line (x outside the window) is not the
 * line to rejoin. */
static void test_tape_outside_window_is_ignored(void)
{
    follow_t      f;
    follow_out_t  out;
    int32_t const out_x = (int32_t)OBSTACLE_FOLLOW_LINE_WINDOW_MM + (int32_t)OBSTACLE_FOLLOW_LINE_LEAD_MM + 1;
    follow_in_t   in    = pose(out_x, (int32_t)OBSTACLE_FOLLOW_LEAVE_AHEAD_MM + 1, -900);

    start_left(&f);
    in.b_line = true;
    (void)step(&f, in);
    in.x_mm = out_x - (int32_t)OBSTACLE_FOLLOW_LINE_LEAD_MM - 1;
    out     = step(&f, in);
    CHECK(out.result == FOLLOW_RUN);
    CHECK(!out.b_turning);
}

/* The face direction stays wrapped through any number of turns, and the
 * spin target stays next to the unwrapped heading. */
static void test_face_direction_wraps(void)
{
    follow_t     f;
    follow_out_t out;
    uint16_t const close = (uint16_t)(OBSTACLE_FOLLOW_INNER_MM - 1U);

    start_left(&f);
    (void)step(&f, with_ping(pose(0, 0, 900), OBSTACLE_CLASS_VALID, close));
    (void)step(&f, pose(0, 0, 1800)); /* turned away: on a leg again */
    out = step(&f, with_ping(pose(0, 0, 1800), OBSTACLE_CLASS_VALID, close));
    CHECK(f.face_head_deci == -900); /* 2700 wrapped */
    CHECK(out.b_turning);
    CHECK(out.right_um_s > 0); /* still a left turn, to 2700 */
}

/* Passing on the right mirrors the corner and the face-ahead turns. */
static void test_right_side_turns_mirror(void)
{
    follow_t          f;
    follow_in_t const in = pose(0, 0, -900);
    follow_out_t      out;
    int32_t const     x       = -(int32_t)OBSTACLE_FOLLOW_LEG_MM - 1;
    int32_t const     in_head = -900 + (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    uint32_t          idx     = 0U;

    follow_start(&f, false, &in, 0);
    out = step(&f, with_ping(in, OBSTACLE_CLASS_VALID, (uint16_t)(OBSTACLE_FOLLOW_INNER_MM - 1U)));
    CHECK(out.b_turning);
    CHECK(out.left_um_s > 0); /* away from a face on the left is right */

    follow_start(&f, false, &in, 250);
    (void)step(&f, in);
    (void)step(&f, pose(x, 0, -900));
    (void)step(&f, pose(x, 0, in_head)); /* settled */
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
    }
    confirm_corner(&f, x, in_head + (int32_t)OBSTACLE_FOLLOW_DEEPER_DECI);
    out = step(&f, pose(f.face_x_mm - (int32_t)OBSTACLE_FOLLOW_CORNER_PAST_MM - 1, 0, -900));
    CHECK(out.b_turning);
    CHECK(out.right_um_s > 0); /* wrap toward a face on the left is left */
}

/* A face that was never seen does not end: a glance that misses it arcs
 * back and drives on (the face starts further on). */
static void test_unseen_face_is_not_a_corner(void)
{
    follow_t      f;
    follow_out_t  out;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    uint32_t      idx     = 0U;

    to_glance(&f, x);
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        out = step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
    }
    CHECK(!out.b_turning);
    CHECK(out.right_um_s > out.left_um_s); /* arcs back, away from the face */
    CHECK(f.face_x_mm == 0);               /* no corner reference */
}

/* A glance ping on the first step at the glance heading was taken before
 * it was reached: it is not a miss. */
static void test_first_glance_ping_is_not_a_miss(void)
{
    follow_t      f;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    uint32_t      idx     = 0U;

    start_left_seen(&f);
    (void)step(&f, pose(0, 0, 900));
    (void)step(&f, pose(x, 0, 900));
    for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
    {
        (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
    }
    CHECK(f.face_x_mm == 0); /* one ping short of the face-lost check */
    {
        follow_out_t const out = step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));

        CHECK(out.left_um_s < 0); /* backs up for the deeper look */
    }
}

/* The scan estimate is replaced by the first echo, without a direction
 * change, so a short face still gets a real anchor. */
static void test_seed_is_replaced_by_first_echo(void)
{
    follow_t      f;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;

    to_glance_seen(&f, x);
    (void)step(&f, with_ping(pose(x, 0, in_head), OBSTACLE_CLASS_VALID, 200U));
    CHECK(f.face_x_mm != 0);
    CHECK(f.face_head_deci == 900);
    CHECK(f.b_seed == 0U);
}

/* a seen face that stops answering is checked by backing up and
 * looking deeper; an echo there is an echo loss, not a corner. After
 * OBSTACLE_FOLLOW_TRIES_MAX recoveries on one face the bypass gives up. */
static void test_echo_loss_backs_up_then_gives_up(void)
{
    follow_t      f;
    follow_out_t  out;
    int32_t const x       = (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
    int32_t const in_head = 900 - (int32_t)OBSTACLE_FOLLOW_TOWARD_DECI;
    int32_t const look    = in_head - (int32_t)OBSTACLE_FOLLOW_DEEPER_DECI;
    uint32_t      tryno   = 0U;
    uint32_t      idx     = 0U;

    int32_t       cx      = x;

    to_glance_seen(&f, x);
    for (tryno = 0U; tryno < OBSTACLE_FOLLOW_TRIES_MAX; tryno++)
    {
        for (idx = 0U; idx < OBSTACLE_FOLLOW_MISS_MAX; idx++)
        {
            out = step(&f, with_ping(pose(cx, 0, in_head), OBSTACLE_CLASS_OPEN, 0U));
        }
        CHECK(out.left_um_s < 0); /* backs up */
        (void)step(&f, pose(cx, -70, look + 600));
        out = step(&f, pose(cx, -70, look));
        /* The face answers the deeper look, about 250 mm off: echo loss. */
        out = step(&f, with_ping(pose(cx, -70, look), OBSTACLE_CLASS_VALID, 330U));
        if ((tryno + 1U) < OBSTACLE_FOLLOW_TRIES_MAX)
        {
            CHECK(out.result == FOLLOW_RUN);
            CHECK(out.right_um_s > out.left_um_s); /* arcs back, away */
            /* On along the same face to the next glance. */
            (void)step(&f, pose(cx, 0, 900));
            cx += (int32_t)OBSTACLE_FOLLOW_LEG_MM + 1;
            (void)step(&f, pose(cx, 0, 900));
            (void)step(&f, pose(cx, 0, in_head));
        }
    }
    CHECK(out.result == FOLLOW_GAVE_UP);
    CHECK(out.reason == FOLLOW_REASON_ECHO_LOST);
    CHECK(out.left_um_s == 0);
    CHECK(out.right_um_s == 0);
}

/* back at the hit point after going round the obstacle. */
static void test_circled_gives_up(void)
{
    follow_t     f;
    follow_out_t out;
    follow_in_t  in = pose(0, 0, 900);

    start_left(&f);
    in.travel_mm = (uint16_t)(OBSTACLE_FOLLOW_CIRCLE_MIN_MM + 1U);
    in.x_mm      = 50;
    in.y_mm      = -50;
    out          = step(&f, in);
    CHECK(out.result == FOLLOW_GAVE_UP);
    CHECK(out.reason == FOLLOW_REASON_CIRCLED);
    out = step(&f, in); /* it stays stopped */
    CHECK(out.result == FOLLOW_GAVE_UP);
}

/* no gain along the original line for OBSTACLE_FOLLOW_NO_PROGRESS_MM
 * of travel; a gain restarts the count. */
static void test_no_progress_gives_up(void)
{
    follow_t     f;
    follow_out_t out;
    follow_in_t  in = pose(500, 0, 900);

    start_left(&f);
    in.travel_mm = (uint16_t)(OBSTACLE_FOLLOW_NO_PROGRESS_MM - 100U);
    in.y_mm      = 10; /* a gain */
    out          = step(&f, in);
    CHECK(out.result == FOLLOW_RUN);
    in.travel_mm = (uint16_t)((2U * OBSTACLE_FOLLOW_NO_PROGRESS_MM) - 101U);
    out          = step(&f, in);
    CHECK(out.result == FOLLOW_RUN);
    in.travel_mm = (uint16_t)((2U * OBSTACLE_FOLLOW_NO_PROGRESS_MM) - 99U);
    out          = step(&f, in);
    CHECK(out.result == FOLLOW_GAVE_UP);
    CHECK(out.reason == FOLLOW_REASON_NO_PROGRESS);
}

/* the virtual bumper trips on a slow driven wheel after the settle
 * time, not on a command change or a slow command. */
static void test_virtual_bumper(void)
{
    follow_bump_t bump      = {{0, 0}, 0U, 0U};
    int32_t const cmd[2]    = {140000, 140000};
    int32_t const fast[2]   = {140000, 138000};
    int32_t const stuck[2]  = {140000, 20000};
    int32_t const slow[2]   = {50000, 50000};
    int32_t const zero[2]   = {0, 0};
    uint32_t      t         = 0U;
    bool          b_tripped = false;

    for (t = 0U; t < 400U; t += 20U)
    {
        b_tripped = b_tripped || follow_bump_update(&bump, cmd, fast, 20U);
    }
    CHECK(!b_tripped);
    for (t = 0U; t < (OBSTACLE_BUMP_MS - 20U); t += 20U)
    {
        b_tripped = b_tripped || follow_bump_update(&bump, cmd, stuck, 20U);
    }
    CHECK(!b_tripped); /* not long enough yet */
    CHECK(follow_bump_update(&bump, cmd, stuck, 20U));

    /* A new command restarts the settle time. */
    (void)follow_bump_update(&bump, slow, zero, 20U);
    CHECK(!follow_bump_update(&bump, cmd, stuck, 20U));
    /* Under the minimum speed nothing counts. */
    for (t = 0U; t < 1000U; t += 20U)
    {
        CHECK(!follow_bump_update(&bump, slow, zero, 20U));
    }
}

int main(void)
{
    test_leg_trim_holds_the_gap();
    test_corner_on_the_last_miss();
    test_too_close_is_a_face_ahead();
    test_face_ahead_during_corner();
    test_tape_outside_window_is_ignored();
    test_face_direction_wraps();
    test_right_side_turns_mirror();
    test_unseen_face_is_not_a_corner();
    test_first_glance_ping_is_not_a_miss();
    test_seed_is_replaced_by_first_echo();
    test_echo_loss_backs_up_then_gives_up();
    test_circled_gives_up();
    test_no_progress_gives_up();
    test_virtual_bumper();
    test_leg_drives_with_servo_at_face();
    test_leg_then_glance_arcs_toward_face();
    test_echo_in_glance_arcs_back();
    test_slanted_face_updates_direction();
    test_lost_face_turns_the_corner();
    test_echo_ahead_turns_away();
    test_leave_only_past_the_hit_point();
    test_right_side_mirrors();

    if (g_failures != 0)
    {
        (void)fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    (void)printf("obstacle_follow: all tests passed\n");
    return 0;
}
