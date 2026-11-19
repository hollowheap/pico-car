/** @file test_obstacle_bypass_sim.c
 *
 * @brief Closed-loop host simulation of the obstacle bypass.
 *
 * The real statechart.c and planner.c run unchanged. Around them the test
 * models what the car adds:
 *   - the scan task (servo pre-delay and settle, 60 ms ping spacing),
 *   - the navigation glue of obstacle_nav.c (hold, handshake, TURNED, the
 *     servo look request). The dead reckoning, the servo wait, the IR gate,
 *     the ping freshness and the repeat gate are the real nav_core.c
 *, and the bypass itself is the real follow.c,
 *   - a differential-drive car (107 mm track, 140 mm/s, wheel lag),
 *   - a sonar with a 9.5 degree half cone and no echo beyond
 *     OBSTACLE_ECHO_INCIDENCE_MAX_DECI, and a world of walls, boxes
 *     (optionally rotated) and lines.
 * Coordinates: x is sideways, positive to the LEFT of the original line; y
 * is along the line. Heading is positive to the left, so a car with heading
 * h moves along (sin h, cos h).
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "follow.h"
#include "nav_core.h"
#include "obstacle_config.h"
#include "obstacle_pack.h"
#include "planner.h"
#include "scan_algo.h"
#include "statechart.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

#define PI_D            (3.14159265358979323846)
#define DEG2RAD         (PI_D / 180.0)
#define DECI2RAD        (PI_D / 1800.0)
#define MAX_RECTS       (6)
#define MAX_LINES       (3)
#define SONAR_RAYS      (9)
#define SONAR_RANGE_MM  (4030.0)
#define SONAR_BLIND_MM  (20.0)
#define SENSOR_FWD_MM   (60.0)
#define IR_FWD_MM       (40.0)
#define LINE_HALF_MM    (12.0)
#define WHEEL_LAG_MS    (80.0)
#define NAV_PERIOD_MS   (20)
#define TASK_QUEUE      (4)
#define EVT_QUEUE       (8)
#define CAR_HALF_W_MM   (63.0)
#define CAR_FRONT_MM    (95.0)
#define CAR_REAR_MM     (95.0)
#define APPROACH_MM     (140.0)

typedef struct
{
    double x0;
    double y0;
    double x1;
    double y1;
    double rot; /* rotation about the centre, radians, positive to the left */
} rect_t;

typedef struct
{
    /* World. */
    rect_t rect[MAX_RECTS];
    int    rects;
    double line_x[MAX_LINES];
    int    lines;

    /* Faults injected into the side readings (the ray at the hug angle). */
    uint32_t open_from_ms;
    uint32_t open_to_ms;
    uint32_t drop_every;
    /* Pings while the face follow turns read the face at 200 mm: the beam
     * sweeps across it mid-turn, off the look line the follow asked for. */
    bool b_guard_sweep;

    /* Wheel scale errors: true travel = odometry travel * scale. */
    double scale_left;
    double scale_right;

    /* Run length. */
    uint32_t max_ms;
    bool     b_stop_on_monitor;
} sim_cfg_t;

typedef struct
{
    sim_cfg_t cfg;

    /* Clock and car. */
    uint32_t now;
    double   x;
    double   y;
    double   th;
    double   v_left;
    double   v_right;
    double   cmd_left;
    double   cmd_right;
    double   wheel_left_um;
    double   wheel_right_um;
    bool     b_hold;

    /* Scan task. */
    obstacle_statechart_t fsm;
    int16_t  servo;
    bool     b_servo_known;
    uint32_t last_trig;
    uint32_t busy_until;
    int      task_state;
    uint8_t  cmd_q[TASK_QUEUE];
    int      cmd_n;
    uint32_t evt_q[EVT_QUEUE];
    int      evt_n;
    uint32_t face_pings;

    /* Navigation glue: the pure part is the firmware's nav_core.c; the
     * handshake around it mirrors obstacle_nav.c. */
    nav_core_t core;
    bool     b_scan_pending;
    bool     b_turned;
    bool     b_wall;
    uint8_t  req_id;
    uint8_t  next_id;
    uint8_t  last_state;
    uint8_t  side;
    uint8_t  start_deg;
    uint32_t hold_ms;

    /* Face follow (follow.c). */
    follow_t follow;
    int16_t  follow_servo;
    bool     b_follow_turning;

    /* Observations. */
    uint32_t rescans;
    uint32_t reacquire_pings;
    uint32_t reacquire_start;
    uint32_t servo_over_limit;
    double   body_min_mm;
    double   min_x;
    double   min_y;
    double   min_th;
    unsigned min_phase;
    double   x_max;
    double   y_max;
    uint8_t  last_cause;
    uint8_t  gave_up_reason;
    uint32_t stop_ms;
    bool     b_reached_monitor_after_bypass;
    uint32_t bypass_count;
    uint32_t turn_pings; /* pings the guard path took in REACQUIRE */
} sim_t;

/* ---------------------------------------------------------------- */
/* World geometry.                                                   */
/* ---------------------------------------------------------------- */

/* A point in the frame of the unrotated rectangle. */
static void to_rect_frame(rect_t const * p_r, double px, double py, double * p_lx, double * p_ly)
{
    double const cx = (p_r->x0 + p_r->x1) / 2.0;
    double const cy = (p_r->y0 + p_r->y1) / 2.0;
    double const c  = cos(-p_r->rot);
    double const sn = sin(-p_r->rot);

    *p_lx = cx + (((px - cx) * c) - ((py - cy) * sn));
    *p_ly = cy + (((px - cx) * sn) + ((py - cy) * c));
}

static bool ray_rect(double wox, double woy, double wdx, double wdy, rect_t const * p_r,
                     double * p_t, double * p_cos)
{
    double ox = 0.0;
    double oy = 0.0;
    double dx = (wdx * cos(-p_r->rot)) - (wdy * sin(-p_r->rot));
    double dy = (wdx * sin(-p_r->rot)) + (wdy * cos(-p_r->rot));

    double tmin = 0.0;
    double tmax = 1.0e9;
    double nx   = 0.0;
    double ny   = 0.0;
    int    axis = 0;
    double lo[2];
    double hi[2];
    double org[2];
    double dir[2];

    to_rect_frame(p_r, wox, woy, &ox, &oy);
    lo[0] = p_r->x0;  hi[0] = p_r->x1;  org[0] = ox;  dir[0] = dx;
    lo[1] = p_r->y0;  hi[1] = p_r->y1;  org[1] = oy;  dir[1] = dy;

    for (axis = 0; axis < 2; axis++)
    {
        if (fabs(dir[axis]) < 1.0e-9)
        {
            if ((org[axis] < lo[axis]) || (org[axis] > hi[axis]))
            {
                return false;
            }
        }
        else
        {
            double t1 = (lo[axis] - org[axis]) / dir[axis];
            double t2 = (hi[axis] - org[axis]) / dir[axis];
            double s  = -1.0;

            if (t1 > t2)
            {
                double tmp = t1;

                t1  = t2;
                t2  = tmp;
                s   = 1.0;
            }
            if (t1 > tmin)
            {
                tmin = t1;
                nx   = (0 == axis) ? s : 0.0;
                ny   = (1 == axis) ? s : 0.0;
            }
            if (t2 < tmax)
            {
                tmax = t2;
            }
            if (tmin > tmax)
            {
                return false;
            }
        }
    }

    if (tmin <= 0.0)
    {
        return false;
    }

    *p_t   = tmin;
    *p_cos = fabs((dx * nx) + (dy * ny));

    return true;
}

/* Sonar: nearest echo inside the cone; faces seen at a slant of more than
 * SONAR_MAX_INCIDENCE_DEG do not return. */
static void sonar(sim_t const * p_s, double servo_deci, uint8_t * p_cls, uint16_t * p_dist)
{
    double const sx    = p_s->x + (SENSOR_FWD_MM * sin(p_s->th));
    double const sy    = p_s->y + (SENSOR_FWD_MM * cos(p_s->th));
    double const phi   = p_s->th + (servo_deci * DECI2RAD);
    double const half  = OBSTACLE_BEAM_HALF_DECI * DECI2RAD;
    double const cmin  = cos(OBSTACLE_ECHO_INCIDENCE_MAX_DECI * DECI2RAD);
    double       best  = 1.0e9;
    int          ray   = 0;
    int          idx   = 0;

    for (ray = 0; ray < SONAR_RAYS; ray++)
    {
        double const a  = phi + (half * (((2.0 * ray) / (SONAR_RAYS - 1)) - 1.0));
        double const dx = sin(a);
        double const dy = cos(a);

        for (idx = 0; idx < p_s->cfg.rects; idx++)
        {
            double t = 0.0;
            double c = 0.0;

            if (ray_rect(sx, sy, dx, dy, &p_s->cfg.rect[idx], &t, &c) && (c >= cmin) && (t < best))
            {
                best = t;
            }
        }
    }

    if (best > SONAR_RANGE_MM)
    {
        *p_cls  = OBSTACLE_CLASS_OPEN;
        *p_dist = 0U;
    }
    else if (best < SONAR_BLIND_MM)
    {
        /* Inside the blind zone: an echo too soon to time. */
        *p_cls  = OBSTACLE_CLASS_TOO_CLOSE;
        *p_dist = 0U;
    }
    else
    {
        *p_cls  = OBSTACLE_CLASS_VALID;
        *p_dist = (uint16_t)(best + 0.5);
    }
}

static bool point_in_rect(double px, double py, rect_t const * p_r)
{
    return (px >= p_r->x0) && (px <= p_r->x1) && (py >= p_r->y0) && (py <= p_r->y1);
}

/* Distance from a world point to a rectangle, -1 inside. */
static double rect_distance(rect_t const * p_r, double wpx, double wpy)
{
    double px = 0.0;
    double py = 0.0;
    double dx = 0.0;
    double dy = 0.0;

    to_rect_frame(p_r, wpx, wpy, &px, &py);
    dx = fmax(fmax(p_r->x0 - px, 0.0), px - p_r->x1);
    dy = fmax(fmax(p_r->y0 - py, 0.0), py - p_r->y1);
    return point_in_rect(px, py, p_r) ? -1.0 : sqrt((dx * dx) + (dy * dy));
}

/* Smallest distance from any sampled outline point of the car body to any
 * rectangle (negative when inside). Coarse, 10 mm sampling. */
static double body_clearance(sim_t const * p_s)
{
    double best = 1.0e9;
    int    idx  = 0;
    int    a    = 0;
    int    b    = 0;

    for (idx = 0; idx < p_s->cfg.rects; idx++)
    {
        rect_t const * r = &p_s->cfg.rect[idx];

        for (a = -10; a <= 10; a++)
        {
            for (b = 0; b < 2; b++)
            {
                double const lat = (a * CAR_HALF_W_MM) / 10.0;
                double const fwd = (0 == b) ? CAR_FRONT_MM : -CAR_REAR_MM;
                double const px  = p_s->x + (lat * cos(p_s->th)) + (fwd * sin(p_s->th));
                double const py  = p_s->y - (lat * sin(p_s->th)) + (fwd * cos(p_s->th));
                double const d   = rect_distance(r, px, py);

                if (d < best)
                {
                    best = d;
                }
            }
        }
        for (a = 0; a <= 20; a++)
        {
            int side = 0;

            for (side = -1; side <= 1; side += 2)
            {
                double const fwd = -CAR_REAR_MM + ((a * (CAR_FRONT_MM + CAR_REAR_MM)) / 20.0);
                double const lat = side * CAR_HALF_W_MM;
                double const px  = p_s->x + (lat * cos(p_s->th)) + (fwd * sin(p_s->th));
                double const py  = p_s->y - (lat * sin(p_s->th)) + (fwd * cos(p_s->th));
                double const d   = rect_distance(r, px, py);

                if (d < best)
                {
                    best = d;
                }
            }
        }
    }

    return best;
}

static bool ir_sees_line(sim_t const * p_s)
{
    double const ix  = p_s->x + (IR_FWD_MM * sin(p_s->th));
    int          idx = 0;

    for (idx = 0; idx < p_s->cfg.lines; idx++)
    {
        if (fabs(ix - p_s->cfg.line_x[idx]) < LINE_HALF_MM)
        {
            return true;
        }
    }
    return false;
}

/* ---------------------------------------------------------------- */
/* Dead reckoning (nav_core.c) and wheels.                            */
/* ---------------------------------------------------------------- */

/* The wheel travel as the firmware reads it: whole micrometres. */
static void wheel_um(sim_t const * p_s, int64_t p_wheel[NAV_CORE_WHEELS])
{
    p_wheel[NAV_CORE_LEFT]  = (int64_t)llround(p_s->wheel_left_um);
    p_wheel[NAV_CORE_RIGHT] = (int64_t)llround(p_s->wheel_right_um);
}

static int32_t head_deci(sim_t const * p_s)
{
    int64_t wheel[NAV_CORE_WHEELS];

    wheel_um(p_s, wheel);

    return nav_core_heading_deci(&p_s->core, wheel);
}

static void update_pose(sim_t * p_s)
{
    int64_t wheel[NAV_CORE_WHEELS];

    wheel_um(p_s, wheel);
    nav_core_update_pose(&p_s->core, wheel);
}

static void drive_wheels(sim_t * p_s, int32_t left_um_s, int32_t right_um_s)
{
    p_s->cmd_left  = (double)left_um_s / 1000.0;
    p_s->cmd_right = (double)right_um_s / 1000.0;
}

/* ---------------------------------------------------------------- */
/* Scan task (mirror of scan_task.c).                                */
/* ---------------------------------------------------------------- */

enum { TASK_IDLE = 0, TASK_PREDELAY = 1 };

static bool is_car_driving(sim_t const * p_s)
{
    return (OBSTACLE_STATE_MONITOR == p_s->fsm.state) ||
           (OBSTACLE_STATE_BYPASS == p_s->fsm.state) ||
           (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state);
}

static uint32_t settle_ms(sim_t const * p_s, int16_t target)
{
    int32_t delta = p_s->b_servo_known ? (target - p_s->servo) : 1200;

    if (delta < 0)
    {
        delta = -delta;
    }
    return (uint32_t)((delta * (int32_t)OBSTACLE_SETTLE_MS_PER_DEG) / 10) + OBSTACLE_SETTLE_BASE_MS;
}

static void publish(sim_t * p_s, obstacle_statechart_out_t const * p_out)
{
    uint8_t idx = 0U;

    for (idx = 0U; idx < p_out->evt_count; idx++)
    {
        if (p_s->evt_n < EVT_QUEUE)
        {
            p_s->evt_q[p_s->evt_n] = p_out->evt[idx];
            p_s->evt_n++;
        }
    }
    if (p_out->b_call_stop)
    {
        p_s->b_hold = true;
    }
}

static double mean_odo_mm(sim_t const * p_s)
{
    return ((p_s->wheel_left_um + p_s->wheel_right_um) / 2.0) / 1000.0;
}

static void task_commands(sim_t * p_s)
{
    while (p_s->cmd_n > 0)
    {
        obstacle_statechart_out_t out;
        uint8_t const             cmd = p_s->cmd_q[0];
        int                       i   = 0;

        for (i = 1; i < p_s->cmd_n; i++)
        {
            p_s->cmd_q[i - 1] = p_s->cmd_q[i];
        }
        p_s->cmd_n--;
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_command(&p_s->fsm, cmd, p_s->now,
                                    (uint32_t)(mean_odo_mm(p_s) + 0.5), &out);
        publish(p_s, &out);
    }
}

/* Fault injection on the face pings: OPEN in a time window, or one in
 * every drop_every pings. */
static bool face_ping_open(sim_t * p_s)
{
    bool b_open = false;

    if ((p_s->cfg.open_to_ms > 0U) && (p_s->now >= p_s->cfg.open_from_ms) &&
        (p_s->now < p_s->cfg.open_to_ms))
    {
        b_open = true;
    }
    if ((p_s->cfg.drop_every > 0U) && (0U == (p_s->face_pings % p_s->cfg.drop_every)))
    {
        b_open = true;
    }
    return b_open;
}

static void task_ping(sim_t * p_s)
{
    obstacle_statechart_out_t out;
    uint8_t                   cls  = OBSTACLE_CLASS_OPEN;
    uint16_t                  dist = 0U;

    sonar(p_s, p_s->servo, &cls, &dist);

    if (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state)
    {
        p_s->reacquire_pings++;
        p_s->face_pings++;
        if (face_ping_open(p_s))
        {
            cls  = OBSTACLE_CLASS_OPEN;
            dist = 0U;
        }
    }

    p_s->last_trig = p_s->now;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_set_look(&p_s->fsm, p_s->servo);
    obstacle_statechart_ping(&p_s->fsm, cls, dist, p_s->now, &out);
    publish(p_s, &out);
}

/* A ping while the body turns: only the guards see it, as in scan_task.c. */
static void task_ping_guard(sim_t * p_s, bool b_turning)
{
    if (b_turning)
    {
        obstacle_statechart_out_t out;
        uint8_t                   cls  = OBSTACLE_CLASS_OPEN;
        uint16_t                  dist = 0U;

        sonar(p_s, p_s->servo, &cls, &dist);
        if (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state)
        {
            p_s->turn_pings++;
        }
        if (p_s->cfg.b_guard_sweep && (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state))
        {
            cls  = OBSTACLE_CLASS_VALID;
            dist = 200U;
        }
        p_s->last_trig = p_s->now;
        obstacle_statechart_out_clear(&out);
        obstacle_statechart_ping_guard(&p_s->fsm, cls, dist, p_s->now, &out);
        publish(p_s, &out);
    }
    else
    {
        task_ping(p_s);
    }
}

static void task_step(sim_t * p_s)
{
    obstacle_statechart_out_t out;

    if (p_s->now < p_s->busy_until)
    {
        return;
    }

    task_commands(p_s);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_tick(&p_s->fsm, p_s->now, (uint32_t)(mean_odo_mm(p_s) + 0.5), &out);
    publish(p_s, &out);

    {
        /* In REACQUIRE the face follow sets the servo, as the
         * firmware does. */
        int16_t target = (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state) ? p_s->follow_servo
                                                                      : obstacle_statechart_servo_deci(&p_s->fsm);

        if (TASK_PREDELAY == p_s->task_state)
        {
            p_s->task_state = TASK_IDLE;
        }
        else if (p_s->b_servo_known && (target != p_s->servo) && is_car_driving(p_s))
        {
            p_s->task_state = TASK_PREDELAY;
            p_s->busy_until = p_s->now + OBSTACLE_SERVO_PREDELAY_MS;
            return;
        }
        else
        {
            /* No pre-delay needed. */
        }

        if ((!p_s->b_servo_known) || (target != p_s->servo))
        {
            p_s->busy_until = p_s->now + settle_ms(p_s, target);
            p_s->servo         = target;
            p_s->b_servo_known = true;
        }
        else if ((p_s->now - p_s->last_trig) >= OBSTACLE_PING_PERIOD_MS)
        {
            bool const b_pivoting = (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state) && p_s->b_follow_turning;
            int32_t const body_target = (int32_t)obstacle_statechart_body_deci(&p_s->fsm);
            int32_t const body_err = head_deci(p_s) - body_target;
            bool const b_scan_pivoting = (OBSTACLE_STATE_SCANNING == p_s->fsm.state) &&
                                         (is_car_driving(p_s) || (body_err < -20) || (body_err > 20));

            /* As the firmware: the sonar pings in every state; while
             * the body turns only the health and collision checks use it. */
            task_ping_guard(p_s, b_pivoting || b_scan_pivoting);
        }
        else
        {
            /* Wait for the ping spacing. */
        }
    }
}

/* ---------------------------------------------------------------- */
/* Navigation glue (mirror of obstacle_nav.c).                       */
/* ---------------------------------------------------------------- */

static void nav_post(sim_t * p_s, uint32_t type, uint8_t id)
{
    if (p_s->cmd_n < TASK_QUEUE)
    {
        p_s->cmd_q[p_s->cmd_n] = obstacle_pack_cmd(type, id);
        p_s->cmd_n++;
    }
}

static void nav_keep_plan(sim_t * p_s, uint32_t angle, bool b_wall)
{
    p_s->start_deg = (uint8_t)angle;
    p_s->b_wall    = b_wall;
}

static void nav_event(sim_t * p_s, uint32_t evt)
{
    uint32_t const type = obstacle_pack_get(evt, OBSTACLE_EV_TYPE_SH, OBSTACLE_EV_TYPE_MK);
    uint32_t const id   = obstacle_pack_get(evt, OBSTACLE_EV_ID_SH, OBSTACLE_EV_ID_MK);

    if (OBSTACLE_EVT_OBSTACLE == type)
    {
        p_s->b_hold         = true;
        p_s->hold_ms        = p_s->now;
        p_s->req_id         = p_s->next_id;
        p_s->b_scan_pending = true;
        p_s->next_id        = (uint8_t)((p_s->next_id + 1U) & OBSTACLE_ID_MAX);
        if (OBSTACLE_STATE_WAIT_STOP == p_s->fsm.state)
        {
            p_s->rescans += (p_s->bypass_count > 0U) ? 1U : 0U;
        }
        if (NULL != getenv("SIM_DEBUG"))
        {
            printf("EVT_OBSTACLE: t=%u state=%u pos=(%.1f, %.1f) th=%.1f dist=%u\n",
                   p_s->now, p_s->fsm.state, p_s->x, p_s->y, p_s->th * 180.0 / 3.14159,
                   obstacle_pack_get(evt, OBSTACLE_EV_DIST_SH, OBSTACLE_EV_DIST_MK));
        }
    }
    else if (OBSTACLE_EVT_FAULT == type)
    {
        uint32_t const cause = obstacle_pack_get(evt, OBSTACLE_EV_CAUSE_SH, OBSTACLE_EV_CAUSE_MK);

        p_s->b_hold         = true;
        p_s->b_scan_pending = false;
        p_s->last_cause     = (uint8_t)cause;
        p_s->stop_ms        = p_s->now;
    }
    else if (OBSTACLE_EVT_SCAN_RESULT == type)
    {
        uint32_t const status = obstacle_pack_get(evt, OBSTACLE_EV_STATUS_SH, OBSTACLE_EV_STATUS_MK);

        if ((p_s->req_id == id) &&
            ((OBSTACLE_RESULT_OK == status) || (OBSTACLE_RESULT_CLEAR == status)))
        {
            p_s->side = (uint8_t)obstacle_pack_get(evt, OBSTACLE_EV_SIDE_SH, OBSTACLE_EV_SIDE_MK);
            nav_keep_plan(p_s, obstacle_pack_get(evt, OBSTACLE_EV_ANGLE_SH, OBSTACLE_EV_ANGLE_MK),
                          0U != obstacle_pack_get(evt, OBSTACLE_EV_WALL_SH, OBSTACLE_EV_WALL_MK));
            if (NULL != getenv("SIM_DEBUG"))
            {
                printf("PLAN: status=%u side=%u angle=%u wall=%d edge_state=(%u,%u) hit=(%u,%u) miss=(%u,%u) closest=%u\n",
                       status, p_s->side, p_s->start_deg, p_s->b_wall,
                       p_s->fsm.scan.edge_state[0], p_s->fsm.scan.edge_state[1],
                       p_s->fsm.scan.edge_hit[0], p_s->fsm.scan.edge_hit[1],
                       p_s->fsm.scan.edge_miss[0], p_s->fsm.scan.edge_miss[1],
                       p_s->fsm.scan.closest_mm);
            }
            nav_post(p_s, OBSTACLE_CMD_ACCEPT, p_s->req_id);
        }
        else if (p_s->req_id == id)
        {
            /* As obstacle_nav.c: no way round, answer at once. */
            nav_post(p_s, OBSTACLE_CMD_REJECT, p_s->req_id);
        }
        else
        {
            /* A result for an older request. */
        }
    }
    else
    {
        /* STEER words are not used. */
    }
}

static bool servo_moving(sim_t * p_s, uint32_t status)
{
    return nav_core_servo_moving(&p_s->core, status, p_s->now);
}

static void nav_report_state(sim_t * p_s, uint32_t status)
{
    uint32_t const state = obstacle_pack_get(status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);

    if (p_s->last_state != state)
    {
        int64_t wheel[NAV_CORE_WHEELS];

        wheel_um(p_s, wheel);
        p_s->last_state = (uint8_t)state;
        if (OBSTACLE_STATE_SCANNING == state)
        {
            /* As obstacle_nav.c: the stop point is the reference. */
            nav_core_reset_pose(&p_s->core, wheel);
            p_s->b_turned = false;
        }
        if (OBSTACLE_STATE_BYPASS == state)
        {
            p_s->bypass_count++;
        }
        if (OBSTACLE_STATE_REACQUIRE == state)
        {
            follow_in_t in;

            (void)memset(&in, 0, sizeof(in));
            nav_core_follow_begin(&p_s->core, p_s->fsm.ping, wheel, &in);
            /* As obstacle_nav.c: the stop reading is the first face point. */
            follow_start(&p_s->follow, OBSTACLE_SIDE_LEFT == p_s->side, &in,
                         (int32_t)p_s->fsm.plan.closest_mm + (int32_t)OBSTACLE_SONAR_FWD_MM);
            p_s->follow_servo     = (int16_t)(((OBSTACLE_SIDE_LEFT == p_s->side) ? -1 : 1) *
                                              (int32_t)OBSTACLE_FOLLOW_LOOK_DECI);
            p_s->b_follow_turning = false;
            p_s->reacquire_start  = p_s->now;
        }
        if ((OBSTACLE_STATE_BYPASS == state) || (OBSTACLE_STATE_REACQUIRE == state) ||
            (OBSTACLE_STATE_MONITOR == state))
        {
            nav_core_servo_wait(&p_s->core, status, p_s->now);
        }
    }
}

/* REACQUIRE: one pass of the real face follow, as obstacle_nav.c runs it. */
static void nav_follow(sim_t * p_s, uint32_t status, bool b_line)
{
    follow_in_t  in;
    follow_out_t out;
    int64_t      wheel[NAV_CORE_WHEELS];

    update_pose(p_s);
    if (servo_moving(p_s, status))
    {
        p_s->b_hold = true;
        return;
    }
    p_s->b_hold = false;
    wheel_um(p_s, wheel);
    (void)memset(&in, 0, sizeof(in));
    (void)nav_core_follow_input(&p_s->core, p_s->fsm.ping, p_s->follow_servo, wheel, b_line, &in);

    {
        unsigned const before = p_s->follow.phase;

        follow_step(&p_s->follow, &in, &out);
        if ((NULL != getenv("SIM_DEBUG")) && (before != p_s->follow.phase))
        {
            printf("FOLLOW t=%u phase %u->%u pos=(%.0f,%.0f) th=%.0f odo=(%d,%d,%d) ping=%d cls=%u d=%u face=(%d,%d,%d) pt=%u\n",
                   p_s->now, before, (unsigned)p_s->follow.phase, p_s->x, p_s->y, p_s->th / DEG2RAD,
                   in.x_mm, in.y_mm, in.head_deci, in.b_ping, in.cls, in.dist_mm,
                   p_s->follow.face_x_mm, p_s->follow.face_y_mm, p_s->follow.face_head_deci,
                   (unsigned)p_s->follow.b_point);
        }
    }
    p_s->follow_servo     = out.servo_deci;
    p_s->b_follow_turning = out.b_turning;
    if ((out.servo_deci > (int16_t)OBSTACLE_SERVO_LIMIT_DECI) ||
        (out.servo_deci < -(int16_t)OBSTACLE_SERVO_LIMIT_DECI))
    {
        p_s->servo_over_limit++;
    }
    if (FOLLOW_GAVE_UP == out.result)
    {
        /* As obstacle_nav.c give_up(): REJECT with the reason. */
        p_s->b_hold = true;
        p_s->gave_up_reason = out.reason;
        if (nav_core_repeat_due(&p_s->core, p_s->now))
        {
            nav_post(p_s, OBSTACLE_CMD_REJECT, out.reason);
        }
        return;
    }
    drive_wheels(p_s, out.left_um_s, out.right_um_s);
    if ((FOLLOW_LINE == out.result) && nav_core_repeat_due(&p_s->core, p_s->now))
    {
        nav_post(p_s, OBSTACLE_CMD_LINE_FOUND, p_s->req_id);
    }
}

static void nav_step(sim_t * p_s)
{
    uint32_t status = 0U;
    uint32_t state  = 0U;
    bool const b_line = ir_sees_line(p_s);

    while (p_s->evt_n > 0)
    {
        int i = 0;

        nav_event(p_s, p_s->evt_q[0]);
        for (i = 1; i < p_s->evt_n; i++)
        {
            p_s->evt_q[i - 1] = p_s->evt_q[i];
        }
        p_s->evt_n--;
    }

    if (p_s->b_scan_pending && ((p_s->now - p_s->hold_ms) >= OBSTACLE_NAV_BRAKE_MS))
    {
        p_s->b_scan_pending = false;
        nav_post(p_s, OBSTACLE_CMD_SCAN_REQ, p_s->req_id);
    }

    status = obstacle_statechart_status(&p_s->fsm);
    state  = obstacle_pack_get(status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);
    nav_report_state(p_s, status);

    switch (state)
    {
        case OBSTACLE_STATE_MONITOR:
        {
            uint32_t const cls = obstacle_pack_get(status, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK);
            uint32_t const d   = obstacle_pack_get(status, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK);
            bool const     b_close = (OBSTACLE_CLASS_VALID == cls) && (d <= OBSTACLE_DETECT_MM);
            bool const     b_charge = servo_moving(p_s, status) || b_close;

            p_s->b_hold = b_charge;
            if (!b_charge)
            {
                /* The line follower drives straight at the bypass speed. */
                p_s->cmd_left  = OBSTACLE_BYPASS_SPEED_MM_S;
                p_s->cmd_right = OBSTACLE_BYPASS_SPEED_MM_S;
            }
        }
        break;

        case OBSTACLE_STATE_SCANNING:
        {
            int32_t const target_head = (int32_t)obstacle_statechart_body_deci(&p_s->fsm);
            int32_t left_um_s = 0;
            int32_t right_um_s = 0;
            uint8_t steer = OBSTACLE_STEER_HOLD;
            bool b_done = false;

            obstacle_plan_wheel_pivot(head_deci(p_s), target_head, &left_um_s, &right_um_s, &steer, &b_done);
            if (b_done)
            {
                p_s->b_hold = true;
            }
            else
            {
                p_s->b_hold = false;
                drive_wheels(p_s, left_um_s, right_um_s);
            }
        }
        break;

        case OBSTACLE_STATE_BYPASS:
            update_pose(p_s);
            if (servo_moving(p_s, status))
            {
                p_s->b_hold = true;
            }
            else
            {
                int32_t left_um_s = 0;
                int32_t right_um_s = 0;
                uint8_t steer = OBSTACLE_STEER_HOLD;
                bool b_done = false;
                int32_t const target_head = (int32_t)obstacle_statechart_body_deci(&p_s->fsm);

                p_s->b_hold = false;
                obstacle_plan_wheel_pivot(head_deci(p_s), target_head, &left_um_s, &right_um_s, &steer, &b_done);
                if (b_done)
                {
                    p_s->b_turned = true;
                    if (nav_core_repeat_due(&p_s->core, p_s->now))
                    {
                        nav_post(p_s, OBSTACLE_CMD_TURNED, p_s->req_id);
                    }
                }
                drive_wheels(p_s, left_um_s, right_um_s);
            }
        break;

        case OBSTACLE_STATE_REACQUIRE:
            nav_follow(p_s, status, b_line);
        break;

        default:
            p_s->b_hold = true;
        break;
    }
}

/* ---------------------------------------------------------------- */
/* Physics and the run loop.                                         */
/* ---------------------------------------------------------------- */

static void physics_step(sim_t * p_s)
{
    double const dt     = 0.001;
    double const target_l = p_s->b_hold ? 0.0 : p_s->cmd_left;
    double const target_r = p_s->b_hold ? 0.0 : p_s->cmd_right;
    double const k      = 1.0 - exp(-1000.0 / WHEEL_LAG_MS * dt);
    double       dl     = 0.0;
    double       dr     = 0.0;

    p_s->v_left  += (target_l - p_s->v_left) * k;
    p_s->v_right += (target_r - p_s->v_right) * k;

    /* Odometry sees the nominal travel; the car moves by the true travel. */
    dl = p_s->v_left * dt;
    dr = p_s->v_right * dt;
    p_s->wheel_left_um  += dl * 1000.0;
    p_s->wheel_right_um += dr * 1000.0;
    {
        double const tl = dl * p_s->cfg.scale_left;
        double const tr = dr * p_s->cfg.scale_right;
        double const ds = (tl + tr) / 2.0;
        double const dth = (tr - tl) / CHASSIS_WHEEL_TRACK_MM;

        p_s->x  += ds * sin(p_s->th + (dth / 2.0));
        p_s->y  += ds * cos(p_s->th + (dth / 2.0));
        p_s->th += dth;
    }
}

static void observe(sim_t * p_s)
{
    double const clr = body_clearance(p_s);

    if (clr < p_s->body_min_mm)
    {
        p_s->body_min_mm = clr;
        p_s->min_x       = p_s->x;
        p_s->min_y       = p_s->y;
        p_s->min_th      = p_s->th / DEG2RAD;
        p_s->min_phase   = p_s->follow.phase;
    }
    if (p_s->x > p_s->x_max)
    {
        p_s->x_max = p_s->x;
    }
    if (p_s->y > p_s->y_max)
    {
        p_s->y_max = p_s->y;
    }
}

static void sim_init(sim_t * p_s)
{
    obstacle_plan_cfg_t cfg;
    obstacle_statechart_out_t out;

    obstacle_plan_cfg_default(&cfg);
    p_s->now            = 0U;
    p_s->x              = 0.0;
    p_s->y              = 0.0;
    p_s->th             = 0.0;
    p_s->v_left         = 0.0;
    p_s->v_right        = 0.0;
    p_s->cmd_left       = OBSTACLE_BYPASS_SPEED_MM_S;
    p_s->cmd_right      = OBSTACLE_BYPASS_SPEED_MM_S;
    p_s->wheel_left_um  = 0.0;
    p_s->wheel_right_um = 0.0;
    p_s->b_hold         = false;
    p_s->servo          = 0;
    p_s->b_servo_known  = false;
    p_s->last_trig      = 0U;
    p_s->busy_until     = 0U;
    p_s->task_state     = TASK_IDLE;
    p_s->cmd_n          = 0;
    p_s->evt_n          = 0;
    p_s->last_state     = 0xFFU;
    p_s->body_min_mm    = 1.0e9;
    p_s->x_max          = -1.0e9;
    p_s->y_max          = -1.0e9;
    p_s->last_cause     = OBSTACLE_CAUSE_NONE;
    p_s->stop_ms        = 0U;
    p_s->side           = OBSTACLE_SIDE_LEFT;
    obstacle_statechart_init(&p_s->fsm, &cfg, 0U);
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_command(&p_s->fsm, obstacle_pack_cmd(OBSTACLE_CMD_RESET, 0U), 0U, 0U, &out);
    obstacle_statechart_command(&p_s->fsm, obstacle_pack_cmd(OBSTACLE_CMD_START, 0U), 1U, 0U, &out);
    p_s->last_trig = 0U;
}

static void sim_run(sim_t * p_s)
{
    bool b_done = false;

    while ((p_s->now < p_s->cfg.max_ms) && (!b_done))
    {
        p_s->now++;
        task_step(p_s);
        if (0U == (p_s->now % NAV_PERIOD_MS))
        {
            nav_step(p_s);
        }
        physics_step(p_s);
        observe(p_s);

        if (OBSTACLE_STATE_STOPPED == p_s->fsm.state)
        {
            b_done = true;
            p_s->last_cause = p_s->fsm.cause;
            p_s->stop_ms    = p_s->now;
        }
        if (p_s->cfg.b_stop_on_monitor && (p_s->bypass_count > 0U) &&
            (OBSTACLE_STATE_MONITOR == p_s->fsm.state))
        {
            b_done = true;
            p_s->b_reached_monitor_after_bypass = true;
        }
    }
}

static sim_t * g_sim = NULL;

static void cfg_default(sim_cfg_t * p_cfg)
{
    p_cfg->rects             = 0;
    p_cfg->lines             = 1;
    p_cfg->line_x[0]         = 0.0;
    p_cfg->open_from_ms      = 0U;
    p_cfg->open_to_ms        = 0U;
    p_cfg->drop_every        = 0U;
    p_cfg->b_guard_sweep     = false;
    p_cfg->scale_left        = 1.0;
    p_cfg->scale_right       = 1.0;
    p_cfg->max_ms            = 120000U;
    p_cfg->b_stop_on_monitor = false;
}

static sim_t * sim_new(sim_cfg_t const * p_cfg)
{
    if (NULL == g_sim)
    {
        g_sim = (sim_t *)calloc(1U, sizeof(sim_t));
    }
    /* Every run starts from a clean slate: no counter survives a run. */
    (void)memset(g_sim, 0, sizeof(sim_t));
    g_sim->cfg = *p_cfg;
    sim_init(g_sim);
    return g_sim;
}

static void add_rect(sim_cfg_t * p_cfg, double x0, double y0, double x1, double y1)
{
    p_cfg->rect[p_cfg->rects].x0 = x0;
    p_cfg->rect[p_cfg->rects].y0 = y0;
    p_cfg->rect[p_cfg->rects].x1 = x1;
    p_cfg->rect[p_cfg->rects].y1 = y1;
    p_cfg->rect[p_cfg->rects].rot = 0.0;
    p_cfg->rects++;
}

static void print_run(char const * p_name, sim_t const * p_s)
{
    if (NULL != getenv("SIM_TRACE"))
    {
        printf("  [%s] t=%u ms state=%u cause=%u pos=(%.0f,%.0f) th=%.1f body_min %.0f "
               "at (%.0f,%.0f,%.0f) phase %u rescans=%u bypasses=%u pings=%u\n",
               p_name, p_s->now, p_s->fsm.state, p_s->last_cause, p_s->x, p_s->y,
               p_s->th / DEG2RAD, p_s->body_min_mm, p_s->min_x, p_s->min_y, p_s->min_th,
               p_s->min_phase, p_s->rescans, p_s->bypass_count,
               p_s->reacquire_pings);
    }
}

/* ---------------------------------------------------------------- */
/* Tests.                                                            */
/* ---------------------------------------------------------------- */

/* the bypass keeps at least this much body clearance. */
#define CLEARANCE_MIN_MM (50.0)

/* Back on the original line, past the box, facing along it, after one
 * bypass. */
static void check_rejoined(sim_t const * p_s, double rear_y)
{
    CHECK(p_s->b_reached_monitor_after_bypass);
    CHECK(p_s->y_max > rear_y);
    CHECK(p_s->bypass_count == 1U);
    CHECK(p_s->rescans == 0U);
    CHECK(fabs(p_s->x) <= (double)OBSTACLE_FOLLOW_LINE_WINDOW_MM);
    CHECK(fabs(p_s->th / DEG2RAD) <= 10.0);
    CHECK(p_s->body_min_mm >= CLEARANCE_MIN_MM);
    CHECK(p_s->servo_over_limit == 0U);
}

static sim_t * box_run(double x0, double x1, double front_y, double depth, double rot_deg,
                       char const * p_name)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    cfg_default(&cfg);
    add_rect(&cfg, x0, front_y, x1, front_y + depth);
    cfg.rect[0].rot       = rot_deg * DEG2RAD;
    cfg.b_stop_on_monitor = true;
    cfg.max_ms            = 90000U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run(p_name, s);
    return s;
}

/* Boxes on the line, 180 to 800 mm wide, two depths. */
static void test_box_widths(void)
{
    double const widths[] = {180.0, 300.0, 500.0, 800.0};
    size_t       idx      = 0U;

    for (idx = 0U; idx < (sizeof(widths) / sizeof(widths[0])); idx++)
    {
        double const half = widths[idx] / 2.0;

        check_rejoined(box_run(-half, half, 560.0, 180.0, 0.0, "box"), 740.0);
        check_rejoined(box_run(-half, half, 560.0, 300.0, 0.0, "box-deep"), 860.0);
    }
}

/* Boxes off the line pass on the side with more room. */
static void test_box_off_centre(void)
{
    sim_t * s = box_run(-150.0, 30.0, 560.0, 180.0, 0.0, "box-right-of-line");

    check_rejoined(s, 740.0);
    CHECK(OBSTACLE_SIDE_LEFT == s->side);
    s = box_run(-30.0, 150.0, 560.0, 180.0, 0.0, "box-left-of-line");
    check_rejoined(s, 740.0);
    CHECK(OBSTACLE_SIDE_RIGHT == s->side);
}

/* A box turned 30 degrees: the face points give the slant. */
static void test_slanted_box(void)
{
    check_rejoined(box_run(-150.0, 150.0, 560.0, 180.0, 30.0, "box-slant+30"), 740.0);
    check_rejoined(box_run(-150.0, 150.0, 560.0, 180.0, -30.0, "box-slant-30"), 740.0);
}

/* Wheel scale errors of +-1 %: the car still finds the tape and keeps its
 * clearance. Larger errors drift the dead-reckoned frame past the tape
 * window; the heading stays on the encoders, so the wheels must be
 * calibrated. */
static void test_wheel_scale_error(void)
{
    double const err[2] = {0.99, 1.01};
    int          i      = 0;

    for (i = 0; i < 2; i++)
    {
        sim_cfg_t cfg;
        sim_t *   s = NULL;

        cfg_default(&cfg);
        add_rect(&cfg, -150.0, 560.0, 150.0, 740.0);
        cfg.b_stop_on_monitor = true;
        cfg.max_ms            = 90000U;
        cfg.scale_left        = err[i];
        cfg.scale_right       = 2.0 - err[i];
        s = sim_new(&cfg);
        sim_run(s);
        print_run("scale", s);
        /* Without an IMU the dead-reckoned heading drifts (2 % wheel
         * difference over about 3 m), so only the return to the tape and the
         * clearance are checked; the line follower takes the heading. */
        CHECK(s->b_reached_monitor_after_bypass);
        CHECK(s->rescans == 0U);
        CHECK(s->body_min_mm >= CLEARANCE_MIN_MM);
        CHECK(fabs(s->x) <= (double)OBSTACLE_FOLLOW_LINE_WINDOW_MM);
    }
}

/* A flat wall across the path, long enough to run along for a minute. */
static void wall_world(sim_cfg_t * p_cfg, double wall_y)
{
    cfg_default(p_cfg);
    add_rect(p_cfg, -1000.0, wall_y, 20000.0, wall_y + 50.0);
}

/* A wall has no corner: the car follows it without touching it, with no
 * rescan, until it gives up on no progress. */
static void test_wall_follow_to_time_limit(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    s = sim_new(&cfg);
    sim_run(s);
    print_run("wall", s);

    CHECK(s->bypass_count == 1U);
    CHECK(OBSTACLE_SIDE_LEFT == s->side); /* a wall passes on the left */
    CHECK(s->rescans == 0U);
    CHECK(s->body_min_mm >= CLEARANCE_MIN_MM);
    CHECK(s->x_max > 2000.0);
    /* A wall never ends: the car gives up on no progress, well before
     * the time limit, and does not restart. */
    CHECK(s->fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(s->last_cause == OBSTACLE_CAUSE_BYPASS_GAVE_UP);
    CHECK(s->gave_up_reason == FOLLOW_REASON_NO_PROGRESS);
    CHECK(s->fsm.reason == FOLLOW_REASON_NO_PROGRESS);
    CHECK((s->stop_ms - s->reacquire_start) < OBSTACLE_HUG_MAX_MS);
    CHECK(s->servo_over_limit == 0U);
}

/* A tape away from the original line does not end the bypass. */
static void test_tape_off_the_line_is_ignored(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    cfg.lines             = 2;
    cfg.line_x[1]         = 1500.0;
    cfg.b_stop_on_monitor = true;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("tape", s);

    CHECK(!s->b_reached_monitor_after_bypass);
    CHECK(s->x_max > 1600.0);
    CHECK(s->body_min_mm >= CLEARANCE_MIN_MM);
}

/* A wall with a corner ahead (an inner corner): the car turns away and
 * follows the new face, and round its far end; no rescan, no collision. */
static void test_inner_corner(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    add_rect(&cfg, 1800.0, -3000.0, 1850.0, 700.0);
    cfg.max_ms = 70000U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("corner", s);

    CHECK(s->rescans == 0U);
    CHECK(s->last_cause != OBSTACLE_CAUSE_COLLISION);
    CHECK(s->body_min_mm >= CLEARANCE_MIN_MM);
    CHECK(s->x_max > 1500.0); /* it reached the second wall */
}

/* A box with no tape behind it: the car goes all the way round and gives
 * up when it is back at the hit point (Bug2). */
static void test_circled_box_gives_up(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    cfg_default(&cfg);
    add_rect(&cfg, -150.0, 560.0, 150.0, 740.0);
    cfg.lines  = 0;
    cfg.max_ms = 90000U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("circled", s);

    CHECK(s->bypass_count == 1U);
    CHECK(s->last_cause == OBSTACLE_CAUSE_BYPASS_GAVE_UP);
    CHECK((s->gave_up_reason == FOLLOW_REASON_CIRCLED) || (s->gave_up_reason == FOLLOW_REASON_NO_PROGRESS));
    CHECK(s->body_min_mm >= CLEARANCE_MIN_MM);
}

/* One face ping in three lost: the deeper look recovers the face and the
 * car still rejoins the line. */
static void test_dropped_echoes_still_rejoin(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    cfg_default(&cfg);
    add_rect(&cfg, -150.0, 560.0, 150.0, 740.0);
    cfg.b_stop_on_monitor = true;
    cfg.max_ms            = 90000U;
    cfg.drop_every        = 3U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("drop", s);
    check_rejoined(s, 740.0);
}

/* Pings taken while the follow turns sweep across the face. The follow
 * must not read them as face pings: it keeps the face and rejoins the line. */
static void test_turn_pings_do_not_lose_face(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    cfg_default(&cfg);
    add_rect(&cfg, -150.0, 560.0, 150.0, 740.0);
    cfg.b_stop_on_monitor = true;
    cfg.max_ms            = 90000U;
    cfg.b_guard_sweep     = true;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("turn pings", s);
    CHECK(s->turn_pings > 0U);
    CHECK(s->gave_up_reason != FOLLOW_REASON_ECHO_LOST);
    check_rejoined(s, 740.0);
}

int main(void)
{
    test_box_widths();
    test_box_off_centre();
    test_slanted_box();
    test_wheel_scale_error();
    test_wall_follow_to_time_limit();
    test_tape_off_the_line_is_ignored();
    test_inner_corner();
    test_circled_box_gives_up();
    test_dropped_echoes_still_rejoin();
    test_turn_pings_do_not_lose_face();

    free(g_sim);

    if (g_failures == 0)
    {
        printf("obstacle_bypass_sim: all checks passed\n");
        return 0;
    }
    printf("obstacle_bypass_sim: %d check(s) failed\n", g_failures);
    return 1;
}
