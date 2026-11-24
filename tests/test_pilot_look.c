/** @file test_pilot_look.c
 * @brief Host tests for the look module: face fit and edge finding.
 *
 * A test-only ping model (floating point) reads a flat face the way the
 * sonar does: the nearest face point inside the 9.5 degree beam, or OPEN.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pilot/look.h"

#define CHECK(cond)                                                    \
    do                                                                 \
    {                                                                  \
        if (!(cond))                                                   \
        {                                                              \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                              \
        }                                                              \
    } while (0)

#define PI_RAD        3.14159265358979323846
#define RAD_PER_DECI  (PI_RAD / 1800.0)
#define SENSOR_TO_FACE_MM (250.0)
#define FACE_X_MM     (SENSOR_TO_FACE_MM + (double)LOOK_SONAR_FWD_MM)
#define BEAM_DECI     (95.0)
#define BLIND_DECI    (300.0)
#define CORNER_TOL_MM (12)

static int g_failures;
static look_pose_t const g_origin = {0, 0, 0};

static bool edge_of(look_t const * p_look, look_face_t const * p_face, look_side_t side,
                    look_edge_t * p_edge)
{
    look_edge_t edge[2] = {0};
    bool        have[2];

    look_edges(p_look, p_face, edge, have);
    if (have[side])
    {
        *p_edge = edge[side];
    }
    return have[side];
}

/* The face is the line x = FACE_X_MM, from y = -half_mm to y = +half_mm. */
static void ping(double half_mm, int32_t servo_deci, uint8_t * p_echo, int32_t * p_range)
{
    double const lo = SENSOR_TO_FACE_MM * tan(((double)servo_deci - BEAM_DECI) * RAD_PER_DECI);
    double const hi = SENSOR_TO_FACE_MM * tan(((double)servo_deci + BEAM_DECI) * RAD_PER_DECI);
    double       a  = (lo > -half_mm) ? lo : -half_mm;
    double       b  = (hi < half_mm) ? hi : half_mm;

    *p_echo  = (uint8_t)LOOK_ECHO_OPEN;
    *p_range = 0;
    if ((a <= b) && (fabs((double)servo_deci) <= BLIND_DECI))
    {
        double y = 0.0;

        if (a > 0.0)
        {
            y = a;
        }
        else if (b < 0.0)
        {
            y = b;
        }
        else
        {
            y = 0.0;
        }
        *p_echo  = (uint8_t)LOOK_ECHO_VALID;
        *p_range = (int32_t)lround(sqrt(SENSOR_TO_FACE_MM * SENSOR_TO_FACE_MM + y * y));
    }
}

static look_pose_t car(void)
{
    look_pose_t pose = {0, 0, 0};
    return pose;
}

static void sweep(look_t * p_look, double half_mm, int32_t from_deci, int32_t to_deci)
{
    look_pose_t const pose = car();

    for (int32_t s = from_deci; s <= to_deci; s += 50)
    {
        uint8_t echo  = 0U;
        int32_t range = 0;

        ping(half_mm, s, &echo, &range);
        (void)look_add(p_look, &pose, s, echo, range);
    }
}

static bool near_corner(look_edge_t const * p_edge, int32_t y_mm)
{
    return (abs(p_edge->x_mm - (int32_t)lround(FACE_X_MM)) <= CORNER_TOL_MM)
           && (abs(p_edge->y_mm - y_mm) <= CORNER_TOL_MM);
}

static void test_narrow_face_has_both_edges(void)
{
    look_t      look;
    look_face_t face;
    look_edge_t left = {0};
    look_edge_t right = {0};

    look_clear(&look);
    sweep(&look, 60.0, -250, 250);

    CHECK(look_face(&look, &g_origin, &face));
    /* Rays near the corners read the corner, not the beam centre, so the fit sits a few mm near. */
    CHECK(abs(face.dist_mm - (int32_t)lround(FACE_X_MM)) <= 10);
    CHECK(abs(face.normal_deci) <= 10);

    CHECK(edge_of(&look, &face, LOOK_SIDE_LEFT, &left));
    CHECK(near_corner(&left, 60));
    CHECK(edge_of(&look, &face, LOOK_SIDE_RIGHT, &right));
    CHECK(near_corner(&right, -60));
}

static void test_wide_face_has_no_edge(void)
{
    look_t      look;
    look_face_t face;
    look_edge_t edge = {0};

    /* Corners at 21.8 degrees: the first OPEN ray is at 35, past the trust. */
    look_clear(&look);
    sweep(&look, 100.0, -350, 350);
    CHECK(look_face(&look, &g_origin, &face));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_LEFT, &edge));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_RIGHT, &edge));
}

static void test_wall_has_no_edge(void)
{
    look_t      look;
    look_face_t face;
    look_edge_t edge = {0};

    look_clear(&look);
    sweep(&look, 5000.0, -250, 250);
    CHECK(look_face(&look, &g_origin, &face));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_LEFT, &edge));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_RIGHT, &edge));
}

static void test_open_past_trust_is_not_an_edge(void)
{
    look_t            look;
    look_face_t       face;
    look_edge_t       edge = {0};
    look_pose_t const pose = car();

    look_clear(&look);
    for (int32_t s = 0; s <= 200; s += 50)
    {
        double const c = cos((double)s * RAD_PER_DECI);
        CHECK(look_add(&look, &pose, s, LOOK_ECHO_VALID, (int32_t)lround(SENSOR_TO_FACE_MM / c)));
    }
    CHECK(look_add(&look, &pose, 260, LOOK_ECHO_OPEN, 0));
    CHECK(look_face(&look, &g_origin, &face));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_LEFT, &edge));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_RIGHT, &edge));

    /* The same ray at 25 degrees is trusted. */
    look_clear(&look);
    for (int32_t s = 0; s <= 200; s += 50)
    {
        double const c = cos((double)s * RAD_PER_DECI);
        CHECK(look_add(&look, &pose, s, LOOK_ECHO_VALID, (int32_t)lround(SENSOR_TO_FACE_MM / c)));
    }
    CHECK(look_add(&look, &pose, 250, LOOK_ECHO_OPEN, 0));
    CHECK(look_face(&look, &g_origin, &face));
    CHECK(edge_of(&look, &face, LOOK_SIDE_LEFT, &edge));
    CHECK((abs(edge.x_mm - 318) <= 1) && (abs(edge.y_mm - 57) <= 1));
}

static void test_valid_reading_beyond_step_is_an_edge(void)
{
    look_t            look;
    look_face_t       face;
    look_edge_t       edge = {0};
    look_pose_t const pose = car();
    double const      c20  = cos(200.0 * RAD_PER_DECI);

    look_clear(&look);
    for (int32_t s = -100; s <= 150; s += 50)
    {
        double const c = cos((double)s * RAD_PER_DECI);
        CHECK(look_add(&look, &pose, s, LOOK_ECHO_VALID, (int32_t)lround(SENSOR_TO_FACE_MM / c)));
    }
    CHECK(look_face(&look, &g_origin, &face));

    CHECK(look_add(&look, &pose, 200, LOOK_ECHO_VALID,
                   (int32_t)lround((SENSOR_TO_FACE_MM + 61.0) / c20)));
    CHECK(edge_of(&look, &face, LOOK_SIDE_LEFT, &edge));
    CHECK((abs(edge.x_mm - 319) <= 1) && (abs(edge.y_mm - 35) <= 1));

    /* 58 mm beyond is still the face. */
    look_clear(&look);
    for (int32_t s = -100; s <= 150; s += 50)
    {
        double const c = cos((double)s * RAD_PER_DECI);
        CHECK(look_add(&look, &pose, s, LOOK_ECHO_VALID, (int32_t)lround(SENSOR_TO_FACE_MM / c)));
    }
    CHECK(look_add(&look, &pose, 200, LOOK_ECHO_VALID,
                   (int32_t)lround((SENSOR_TO_FACE_MM + 58.0) / c20)));
    CHECK(!edge_of(&look, &face, LOOK_SIDE_LEFT, &edge));
}

static void test_full_look_drops_and_counts(void)
{
    look_t            look;
    look_pose_t const pose = car();

    look_clear(&look);
    for (int32_t i = 0; i < (int32_t)LOOK_RAYS_MAX; i++)
    {
        CHECK(look_add(&look, &pose, i * 10, LOOK_ECHO_VALID, 300));
    }
    CHECK(LOOK_RAYS_MAX == look.count);

    CHECK(!look_add(&look, &pose, 900, LOOK_ECHO_VALID, 300));
    CHECK(!look_add(&look, &pose, 910, LOOK_ECHO_OPEN, 0));
    CHECK(LOOK_RAYS_MAX == look.count);
    CHECK(0 == look_ray_angle_deci(&look.rays[0]));
    CHECK(230 == look_ray_angle_deci(&look.rays[LOOK_RAYS_MAX - 1U]));

    look_clear(&look);
    CHECK(0U == look.count);
}

static void test_ray_is_placed_from_the_pose(void)
{
    look_t            look;
    look_pose_t const pose = {100, 200, 900};

    look_clear(&look);
    CHECK(look_add(&look, &pose, 900, LOOK_ECHO_VALID, 300));
    /* Sonar 70 mm ahead along +y, beam toward -x at 1800. */
    CHECK(100 == look.rays[0].x_mm);
    CHECK(abs(look.rays[0].y_mm - 270) <= 1);
    CHECK(1800 == look_ray_angle_deci(&look.rays[0]));
    CHECK(300U == look.rays[0].range_mm);
}

static void test_scan_buffer_keeps_full_range_in_eight_byte_rays(void)
{
    look_t look;

    CHECK(8U == sizeof(look_ray_t));
    CHECK(194U == sizeof(look_t));
    look_clear(&look);
    CHECK(look_add(&look, &g_origin, 1801, LOOK_ECHO_TOO_CLOSE, UINT16_MAX));
    CHECK(look_add(&look, &g_origin, 1799, LOOK_ECHO_OPEN, 4000));
    CHECK(-1799 == look_ray_angle_deci(&look.rays[0]));
    CHECK(LOOK_ECHO_TOO_CLOSE == look_ray_echo(&look.rays[0]));
    CHECK(UINT16_MAX == look.rays[0].range_mm);
    CHECK(1799 == look_ray_angle_deci(&look.rays[1]));
    CHECK(LOOK_ECHO_OPEN == look_ray_echo(&look.rays[1]));
    CHECK(0U == look.rays[1].range_mm);

    look_ray_set_angle(&look.rays[0], 1800);
    CHECK(1800 == look_ray_angle_deci(&look.rays[0]));
    CHECK(LOOK_ECHO_TOO_CLOSE == look_ray_echo(&look.rays[0]));
    look_ray_mark_open(&look.rays[0]);
    CHECK(1800 == look_ray_angle_deci(&look.rays[0]));
    CHECK(LOOK_ECHO_OPEN == look_ray_echo(&look.rays[0]));
    CHECK(0U == look.rays[0].range_mm);
}

static void test_too_few_points_give_no_face(void)
{
    look_t            look;
    look_face_t       face;
    look_pose_t const pose = car();

    look_clear(&look);
    CHECK(!look_face(&look, &g_origin, &face));

    CHECK(look_add(&look, &pose, 0, LOOK_ECHO_VALID, 300));
    CHECK(!look_face(&look, &g_origin, &face));

    CHECK(look_add(&look, &pose, 100, LOOK_ECHO_OPEN, 0));
    CHECK(look_add(&look, &pose, -100, LOOK_ECHO_OPEN, 0));
    CHECK(!look_face(&look, &g_origin, &face));

    CHECK(look_add(&look, &pose, 100, LOOK_ECHO_TOO_CLOSE, 100));
    CHECK(look_face(&look, &g_origin, &face));
}

/* One stray echo among the rays of a flat face. */
#define STRAY_SERVO_DECI   (100)
#define STRAY_NEAR_MM      (120.0)
#define SMALL_NEAR_MM      (20.0)
#define FACE_NORMAL_TOL    (10)
#define FACE_DIST_TOL_MM   (5)

static void flat_face_with_stray(look_t * p_look, double near_mm)
{
    look_pose_t const pose = car();

    look_clear(p_look);
    for (int32_t s = -150; s <= 150; s += 50)
    {
        double const c = cos((double)s * RAD_PER_DECI);
        double const range = (SENSOR_TO_FACE_MM / c) - ((STRAY_SERVO_DECI == s) ? near_mm : 0.0);
        CHECK(look_add(p_look, &pose, s, LOOK_ECHO_VALID, (int32_t)lround(range)));
    }
}

static void test_stray_ray_leaves_the_face(void)
{
    look_t      look;
    look_face_t clean;
    look_face_t face;

    flat_face_with_stray(&look, 0.0);
    CHECK(look_face(&look, &g_origin, &clean));

    flat_face_with_stray(&look, STRAY_NEAR_MM);
    CHECK(look_face(&look, &g_origin, &face));
    CHECK(abs(face.normal_deci - clean.normal_deci) <= FACE_NORMAL_TOL);
    CHECK(abs(face.dist_mm - clean.dist_mm) <= FACE_DIST_TOL_MM);

    /* A ray within the face tolerance stays in the fit. */
    flat_face_with_stray(&look, SMALL_NEAR_MM);
    CHECK(look_face(&look, &g_origin, &face));
    CHECK(face.dist_mm < clean.dist_mm);
}

int main(void)
{
    test_narrow_face_has_both_edges();
    test_wide_face_has_no_edge();
    test_wall_has_no_edge();
    test_open_past_trust_is_not_an_edge();
    test_valid_reading_beyond_step_is_an_edge();
    test_full_look_drops_and_counts();
    test_ray_is_placed_from_the_pose();
    test_scan_buffer_keeps_full_range_in_eight_byte_rays();
    test_too_few_points_give_no_face();
    test_stray_ray_leaves_the_face();

    if (0 != g_failures)
    {
        printf("test_pilot_look: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("test_pilot_look: all passed\n");
    return EXIT_SUCCESS;
}
