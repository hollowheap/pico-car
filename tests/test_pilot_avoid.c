/** @file test_pilot_avoid.c
 * @brief Host tests for the reactive avoid module.
 *
 * Includes a point car (floating point, test only) that drives around one box
 * toward a goal 1.5 m ahead.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pilot/avoid.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define PI_RAD       3.14159265358979323846
#define DECI_PER_RAD (1800.0 / PI_RAD)
#define TOL_MM       (3)

static avoid_pose_t pose_at(int32_t x_mm, int32_t y_mm, int32_t head_deci)
{
    avoid_pose_t pose = {x_mm, y_mm, head_deci};
    return pose;
}

static void add_box(avoid_map_t * p_map, int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    /* Fill a box with points every 20 mm through the pose-free path: a car at
     * the origin facing +x reads straight ahead only, so store them directly. */
    for (int32_t x = x0; x <= x1; x += 20)
    {
        for (int32_t y = y0; y <= y1; y += 20)
        {
            p_map->points[p_map->next].x_mm = (int16_t)x;
            p_map->points[p_map->next].y_mm = (int16_t)y;
            p_map->next = (uint8_t)((p_map->next + 1U) % AVOID_MAP_POINTS);
            if (p_map->count < AVOID_MAP_POINTS)
            {
                p_map->count++;
            }
        }
    }
}

static void test_point_conversion(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(100, 200, 0);

    avoid_clear(&map);
    assert(0U == map.count);

    /* Straight ahead: sensor at x = 170, range 300. */
    assert(avoid_add_point(&map, &pose, 0, 300));
    assert(abs(map.points[0].x_mm - 470) <= TOL_MM);
    assert(abs(map.points[0].y_mm - 200) <= TOL_MM);

    /* Servo 90 degrees left: the beam goes along +y. */
    assert(avoid_add_point(&map, &pose, 900, 300));
    assert(abs(map.points[1].x_mm - 170) <= TOL_MM);
    assert(abs(map.points[1].y_mm - 500) <= TOL_MM);

    /* Servo 90 degrees right. */
    assert(avoid_add_point(&map, &pose, -900, 300));
    assert(abs(map.points[2].x_mm - 170) <= TOL_MM);
    assert(abs(map.points[2].y_mm - (-100)) <= TOL_MM);

    /* Car heading 90 degrees: front is +y, sensor offset follows. */
    pose = pose_at(0, 0, 900);
    assert(avoid_add_point(&map, &pose, 0, 100));
    assert(abs(map.points[3].x_mm) <= TOL_MM);
    assert(abs(map.points[3].y_mm - (AVOID_SONAR_FWD_MM + 100)) <= TOL_MM);
    assert(4U == map.count);
}

static void test_far_readings_ignored(void)
{
    avoid_map_t        map;
    avoid_pose_t const pose = pose_at(0, 0, 0);

    avoid_clear(&map);
    assert(!avoid_add_point(&map, &pose, 0, AVOID_MAX_RANGE_MM));
    assert(!avoid_add_point(&map, &pose, 0, AVOID_MAX_RANGE_MM + 1000));
    assert(!avoid_add_point(&map, &pose, 0, 0));
    assert(!avoid_add_point(&map, &pose, 0, -5));
    assert(0U == map.count);
    assert(avoid_add_point(&map, &pose, 0, AVOID_MAX_RANGE_MM - 1));
    assert(1U == map.count);
}

static void test_ring_wrap(void)
{
    avoid_map_t        map;
    avoid_pose_t const pose = pose_at(0, 0, 0);

    avoid_clear(&map);
    for (int32_t i = 0; i < (int32_t)AVOID_MAP_POINTS + 5; i++)
    {
        assert(avoid_add_point(&map, &pose, 0, 100 + i));
    }
    assert(AVOID_MAP_POINTS == map.count);
    assert(5U == map.next);
    /* Slot 0 now holds reading number 32 (range 132). */
    assert(abs(map.points[0].x_mm - (AVOID_SONAR_FWD_MM + 132)) <= TOL_MM);
    /* Slot 5 still holds reading number 5 (range 105). */
    assert(abs(map.points[5].x_mm - (AVOID_SONAR_FWD_MM + 105)) <= TOL_MM);
}

static void test_no_points(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 0);
    avoid_out_t  out;

    avoid_clear(&map);
    avoid_steer(&map, &pose, 1000, 0, &out);
    assert(AVOID_GOAL_CLEAR == out.status);
    assert(0 == out.head_deci);
    assert(AVOID_MAX_SPEED_MM_S == out.speed_mm_s);

    avoid_steer(&map, &pose, 0, 1000, &out);
    assert(AVOID_GOAL_CLEAR == out.status);
    assert(abs(out.head_deci - 900) <= 2);

    /* Goal behind the car: the bearing is still the wanted heading. */
    avoid_steer(&map, &pose, -1000, 0, &out);
    assert(AVOID_GOAL_CLEAR == out.status);
    assert(1800 == out.head_deci);
}

static void test_box_in_front(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 0);
    avoid_out_t  out;

    /* Box 250..310 ahead, slightly more to the left (y -40..+100). */
    avoid_clear(&map);
    add_box(&map, 250, -40, 310, 100);
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(AVOID_DETOUR == out.status);
    assert(out.head_deci < 0); /* right is the nearer free side */
    assert(out.speed_mm_s > 0);
    /* The chosen heading really clears every stored point. */
    {
        double const h = (double)out.head_deci / DECI_PER_RAD;
        for (uint32_t i = 0U; i < map.count; i++)
        {
            double const x = map.points[i].x_mm;
            double const y = map.points[i].y_mm;
            double const along = (x * cos(h)) + (y * sin(h));
            double const side  = (y * cos(h)) - (x * sin(h));
            assert(!((along >= 0.0) && (along <= AVOID_LOOK_MM) &&
                     (fabs(side) < (AVOID_HALF_WIDTH_MM + AVOID_MARGIN_MM) - 1.0)));
        }
    }

    /* Mirror: the box sits more to the right, the detour goes left. */
    avoid_clear(&map);
    add_box(&map, 250, -100, 310, 40);
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(AVOID_DETOUR == out.status);
    assert(out.head_deci > 0);
}

static void test_tie_takes_smaller_turn(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 100);
    avoid_out_t  out;

    /* Symmetric box: both sides are equally good; the car already turns left. */
    avoid_clear(&map);
    add_box(&map, 250, -40, 310, 40);
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(AVOID_DETOUR == out.status);
    assert(out.head_deci > 0);

    pose = pose_at(0, 0, -100);
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(AVOID_DETOUR == out.status);
    assert(out.head_deci < 0);
}

static void test_wall(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 0);
    avoid_out_t  out;

    /* Wall across the whole view: all points at x = 200 from y = -300..300
     * would need more than 32 points at 20 mm; use 40 mm spacing. */
    avoid_clear(&map);
    for (int32_t y = -300; y <= 300; y += 40)
    {
        map.points[map.next].x_mm = 200;
        map.points[map.next].y_mm = (int16_t)y;
        map.next = (uint8_t)((map.next + 1U) % AVOID_MAP_POINTS);
        map.count++;
    }
    /* Corridor 110 wide: gaps of 40 mm leave no free forward heading, but the
     * sides at +-90 and the rear stay free, so the car turns along the wall. */
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(AVOID_DETOUR == out.status);
    assert(abs(out.head_deci) >= 600);

    /* A wall all around the car leaves nothing free. */
    avoid_clear(&map);
    for (int32_t a = 0; a < 360; a += 12)
    {
        double const r = (double)a * PI_RAD / 180.0;
        map.points[map.next].x_mm = (int16_t)lround(150.0 * cos(r));
        map.points[map.next].y_mm = (int16_t)lround(150.0 * sin(r));
        map.next = (uint8_t)((map.next + 1U) % AVOID_MAP_POINTS);
        map.count++;
    }
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(AVOID_BLOCKED == out.status);
    assert(0 == out.speed_mm_s);
    assert(0 == out.head_deci);
}

static void test_edge_hugging(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 0);
    avoid_out_t  out;

    /* Single point ahead at y = 0, goal slightly to the left: the free heading
     * nearest the goal bearing is the first one past the corridor edge. */
    avoid_clear(&map);
    add_box(&map, 300, 0, 300, 0);
    avoid_steer(&map, &pose, 1500, 300, &out); /* bearing about 113 */
    assert(AVOID_DETOUR == out.status || AVOID_GOAL_CLEAR == out.status);
    {
        /* Point at 300 mm, corridor 110: clear when sin(h) * 300 >= 110,
         * that is h >= 21.5 degrees. The bearing is 11.3 degrees, so the
         * pick is the smallest step of 10 degrees around it that clears. */
        assert(out.head_deci > 113);
        assert(out.head_deci <= 113 + (2 * AVOID_STEP_DECI));
    }
    /* A goal that moves further left lets the car turn back toward it. */
    avoid_steer(&map, &pose, 1500, 1500, &out);
    assert(AVOID_GOAL_CLEAR == out.status);
}

static void test_speed(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 0);
    avoid_out_t  out;
    int32_t      prev = AVOID_MAX_SPEED_MM_S + 1;

    /* One point beside the free heading: closer means slower. */
    for (int32_t dist = 380; dist >= 120; dist -= 20)
    {
        avoid_clear(&map);
        map.points[0].x_mm = (int16_t)dist;
        map.points[0].y_mm = (int16_t)(AVOID_HALF_WIDTH_MM + AVOID_MARGIN_MM);
        map.count = 1U;
        avoid_steer(&map, &pose, 1500, 0, &out);
        assert(AVOID_GOAL_CLEAR == out.status);
        assert(out.speed_mm_s <= prev);
        assert(out.speed_mm_s >= AVOID_MIN_SPEED_MM_S);
        prev = out.speed_mm_s;
    }
    assert(prev < AVOID_MAX_SPEED_MM_S);

    /* Inside the stop distance the car does not move. */
    avoid_clear(&map);
    map.points[0].x_mm = (int16_t)(AVOID_STOP_MM / 2);
    map.points[0].y_mm = 0;
    map.count = 1U;
    avoid_steer(&map, &pose, 1500, 0, &out);
    assert(0 == out.speed_mm_s);
}

/* Point car with a sonar that scans -60..+60 degrees in 20 degree steps. */
static void test_drive_around_box(void)
{
    avoid_map_t  map;
    avoid_pose_t pose = pose_at(0, 0, 0);
    double       x = 0.0;
    double       y = 0.0;
    double       head = 0.0;
    double       min_gap = 1.0e9;
    bool         reached = false;
    int32_t const box_x0 = 500;
    int32_t const box_x1 = 560;
    int32_t const box_y0 = -50;
    int32_t const box_y1 = 150;

    avoid_clear(&map);
    for (int32_t step = 0; (step < 3000) && !reached; step++)
    {
        avoid_out_t out;

        /* Scan: ray-march each beam to the box. */
        for (int32_t servo = -600; servo <= 600; servo += 200)
        {
            double const bx = x + (AVOID_SONAR_FWD_MM * cos(head));
            double const by = y + (AVOID_SONAR_FWD_MM * sin(head));
            double const a  = head + ((double)servo / DECI_PER_RAD);
            for (int32_t r = 10; r < AVOID_MAX_RANGE_MM; r += 5)
            {
                double const px = bx + (r * cos(a));
                double const py = by + (r * sin(a));
                if ((px >= box_x0) && (px <= box_x1) && (py >= box_y0) && (py <= box_y1))
                {
                    (void)avoid_add_point(&map, &pose, servo, r);
                    break;
                }
            }
        }
        avoid_steer(&map, &pose, 1500, 0, &out);
        assert(AVOID_BLOCKED != out.status);
        {
            /* The car turns at most 2 rad/s, as a pivot-capable drive. */
            double       want = (double)out.head_deci / DECI_PER_RAD;
            double       err  = want - head;
            double const dt   = 0.02;
            while (err > PI_RAD) { err -= 2.0 * PI_RAD; }
            while (err < -PI_RAD) { err += 2.0 * PI_RAD; }
            if (fabs(err) > (2.0 * dt)) { err = (err > 0.0) ? (2.0 * dt) : (-2.0 * dt); }
            head += err;
            x += ((double)out.speed_mm_s * dt) * cos(head);
            y += ((double)out.speed_mm_s * dt) * sin(head);
        }
        {
            double const cx  = fmin(fmax(x, box_x0), box_x1);
            double const cy  = fmin(fmax(y, box_y0), box_y1);
            double const gap = hypot(x - cx, y - cy);
            if (gap < min_gap) { min_gap = gap; }
        }
        pose = pose_at((int32_t)lround(x), (int32_t)lround(y), (int32_t)lround(head * DECI_PER_RAD));
        reached = (x >= 1450.0);
    }
    assert(reached);
    assert(min_gap >= (double)AVOID_HALF_WIDTH_MM);
    printf("drive-around: min gap %.0f mm\n", min_gap);
}

int main(void)
{
    test_point_conversion();
    test_far_readings_ignored();
    test_ring_wrap();
    test_no_points();
    test_box_in_front();
    test_tie_takes_smaller_turn();
    test_wall();
    test_edge_hugging();
    test_speed();
    test_drive_around_box();
    printf("test_pilot_avoid: all passed\n");
    return 0;
}
