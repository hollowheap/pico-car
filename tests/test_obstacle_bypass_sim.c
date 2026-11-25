/** @file test_obstacle_bypass_sim.c
 *
 * @brief Closed-loop host simulation of the obstacle bypass (D95).
 *
 * The real statechart.c and planner.c run unchanged. Around them the test
 * models what the car adds:
 *   - the scan task (servo pre-delay and settle, 60 ms ping spacing),
 *   - the navigation glue of obstacle_nav.c (hold, handshake, dead
 *     reckoning, side filter, TURNED), written as thin as the real one:
 *     every decision is a call into planner.c,
 *   - a differential-drive car (107 mm track, 140 mm/s, wheel lag),
 *   - a sonar with a 9.5 degree half cone, and a world of flat walls,
 *     boxes and lines.
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
#define SONAR_MAX_INCIDENCE_DEG (35.0)
#define SONAR_RANGE_MM  (4030.0)
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
    uint32_t side_pings;

    /* Navigation glue (mirrors obstacle_nav.c). */
    bool     b_scan_pending;
    bool     b_turned;
    bool     b_servo_wait;
    bool     b_ir_armed;
    bool     b_ir_cleared;
    bool     b_wall;
    uint8_t  req_id;
    uint8_t  next_id;
    uint8_t  last_state;
    uint8_t  side;
    uint8_t  start_deg;
    uint8_t  phase;
    uint8_t  wait_seq;
    bool     b_have_prev_gap;
    bool     b_passed;
    uint8_t  clear_checks;
    uint8_t  side_ping_count;
    uint8_t  side_agree_count;
    uint16_t last_side_dist_mm;
    int32_t  prev_gap_mm;
    int32_t  prev_lean_deci;
    int32_t  wall_trim_deci;
    int32_t  along_head_deci;
    uint32_t look_stop_ms;
    uint32_t wait_ms;
    uint32_t hold_ms;
    uint32_t line_ms;
    int32_t  need_mm;
    double   base_left_um;
    double   base_right_um;
    double   prev_left_um;
    double   prev_right_um;
    double   leg_base_um;
    double   pose_x_um;
    double   pose_y_um;

    /* Observations. */
    uint32_t rescans;
    uint32_t forward_pings;
    uint32_t reacquire_pings;
    uint32_t reacquire_start;
    uint32_t hug_start;
    bool     b_hug;
    double   hug_head_start_deg;
    double   head_min_deg;
    double   head_max_deg;
    double   dist_min_mm;
    double   dist_max_mm;
    double   body_min_mm;
    uint32_t window_from;
    double   window_head_min_deg;
    bool     b_fault;
    uint8_t  last_cause;
    uint32_t stop_ms;
    bool     b_reached_monitor_after_bypass;
    bool     b_aligned_before_hug;
    uint32_t bypass_count;
} sim_t;

/* ---------------------------------------------------------------- */
/* World geometry.                                                   */
/* ---------------------------------------------------------------- */

static bool ray_rect(double ox, double oy, double dx, double dy, rect_t const * p_r,
                     double * p_t, double * p_cos)
{
    double tmin = 0.0;
    double tmax = 1.0e9;
    double nx   = 0.0;
    double ny   = 0.0;
    int    axis = 0;
    double lo[2];
    double hi[2];
    double org[2];
    double dir[2];

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
    double const cmin  = cos(SONAR_MAX_INCIDENCE_DEG * DEG2RAD);
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
                double const dx  = fmax(fmax(r->x0 - px, 0.0), px - r->x1);
                double const dy  = fmax(fmax(r->y0 - py, 0.0), py - r->y1);
                double       d   = sqrt((dx * dx) + (dy * dy));

                if (point_in_rect(px, py, r))
                {
                    d = -1.0;
                }
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
                double const dx  = fmax(fmax(r->x0 - px, 0.0), px - r->x1);
                double const dy  = fmax(fmax(r->y0 - py, 0.0), py - r->y1);
                double       d   = sqrt((dx * dx) + (dy * dy));

                if (point_in_rect(px, py, r))
                {
                    d = -1.0;
                }
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
/* Dead reckoning and wheels (mirror of obstacle_nav.c).             */
/* ---------------------------------------------------------------- */

static int32_t head_deci(sim_t const * p_s)
{
    double const l = p_s->wheel_left_um - p_s->base_left_um;
    double const r = p_s->wheel_right_um - p_s->base_right_um;

    return (int32_t)(((r - l) * OBSTACLE_NAV_DECI_PER_RAD) / (OBSTACLE_WHEEL_TRACK_MM * 1000.0));
}

static int32_t side_sign(sim_t const * p_s)
{
    return (OBSTACLE_SIDE_LEFT == p_s->side) ? 1 : -1;
}

static void reset_pose(sim_t * p_s)
{
    p_s->prev_left_um  = p_s->wheel_left_um;
    p_s->prev_right_um = p_s->wheel_right_um;
    p_s->pose_x_um     = 0.0;
    p_s->pose_y_um     = 0.0;
    p_s->b_turned      = false;
    p_s->b_ir_armed    = false;
    p_s->b_ir_cleared  = false;
    p_s->phase         = OBSTACLE_PHASE_TURN;
}

static void update_pose(sim_t * p_s)
{
    double const l    = p_s->wheel_left_um - p_s->prev_left_um;
    double const r    = p_s->wheel_right_um - p_s->prev_right_um;
    double const step = (l + r) / 2.0;
    double const h    = head_deci(p_s) * DECI2RAD;

    p_s->pose_x_um += step * sin(h);
    p_s->pose_y_um += step * cos(h);
    p_s->prev_left_um  = p_s->wheel_left_um;
    p_s->prev_right_um = p_s->wheel_right_um;
}

static int32_t lateral_mm(sim_t const * p_s)
{
    return side_sign(p_s) * (int32_t)(p_s->pose_x_um / 1000.0);
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

static bool side_ping_open(sim_t * p_s)
{
    bool b_open = false;

    if ((p_s->cfg.open_to_ms > 0U) && (p_s->now >= p_s->cfg.open_from_ms) &&
        (p_s->now < p_s->cfg.open_to_ms))
    {
        b_open = true;
    }
    if ((p_s->cfg.drop_every > 0U) && (0U == (p_s->side_pings % p_s->cfg.drop_every)))
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
    bool const                b_reacq = (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state);

    sonar(p_s, p_s->servo, &cls, &dist);

    if (b_reacq)
    {
        p_s->reacquire_pings++;
        if (OBSTACLE_PHASE_LOOK_PING == p_s->phase)
        {
            p_s->side_pings++;
            if (side_ping_open(p_s))
            {
                cls  = OBSTACLE_CLASS_OPEN;
                dist = 0U;
            }
            p_s->last_trig = p_s->now;
            p_s->fsm.seq   = (uint8_t)(((uint32_t)p_s->fsm.seq + 1U) & OBSTACLE_ST_SEQ_MK);
            p_s->fsm.last_class = cls;
            p_s->fsm.last_dist  = (OBSTACLE_CLASS_VALID == cls) ? dist : 0U;
            return;
        }
        else
        {
            p_s->forward_pings++;
        }
    }

    if (OBSTACLE_STATE_SCANNING == p_s->fsm.state)
    {
        if (NULL != getenv("SIM_DEBUG"))
        {
            printf("SCAN_PING: t=%u servo=%d th=%.1f slot=%u body=%d cls=%u dist=%u\n",
                   p_s->now, p_s->servo, p_s->th * 180.0 / 3.14159,
                   p_s->fsm.scan.slot, obstacle_statechart_body_deci(&p_s->fsm), cls, dist);
        }
    }

    p_s->last_trig = p_s->now;
    obstacle_statechart_out_clear(&out);
    obstacle_statechart_ping(&p_s->fsm, cls, dist, p_s->now, &out);
    publish(p_s, &out);
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
        int16_t target = obstacle_statechart_servo_deci(&p_s->fsm);

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
            bool const b_pivoting = (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state) &&
                                    ((OBSTACLE_PHASE_LOOK_STOP == p_s->phase) ||
                                     (OBSTACLE_PHASE_LOOK_IN == p_s->phase) ||
                                     (OBSTACLE_PHASE_LOOK_OUT == p_s->phase));
            int32_t const body_target = (int32_t)obstacle_statechart_body_deci(&p_s->fsm);
            int32_t const body_err = head_deci(p_s) - body_target;
            bool const b_scan_pivoting = (OBSTACLE_STATE_SCANNING == p_s->fsm.state) &&
                                         (is_car_driving(p_s) || (body_err < -20) || (body_err > 20));

            if (!b_pivoting && !b_scan_pivoting)
            {
                task_ping(p_s);
            }
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
    p_s->need_mm   = (int32_t)(((double)p_s->fsm.plan.closest_mm *
                                obstacle_plan_tan_q14((int32_t)angle * 10)) / 16384.0);
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
                printf("PLAN: status=%u side=%u angle=%u wall=%d need=%d edge_state=(%u,%u) hit=(%u,%u) miss=(%u,%u) closest=%u\n",
                       status, p_s->side, p_s->start_deg, p_s->b_wall, p_s->need_mm,
                       p_s->fsm.scan.edge_state[0], p_s->fsm.scan.edge_state[1],
                       p_s->fsm.scan.edge_hit[0], p_s->fsm.scan.edge_hit[1],
                       p_s->fsm.scan.edge_miss[0], p_s->fsm.scan.edge_miss[1],
                       p_s->fsm.scan.closest_mm);
            }
            nav_post(p_s, OBSTACLE_CMD_ACCEPT, p_s->req_id);
        }
    }
    else
    {
        /* STEER words are not used. */
    }
}

static bool servo_moving(sim_t * p_s, uint32_t status)
{
    uint32_t const seq = obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);

    if (p_s->b_servo_wait &&
        ((seq != p_s->wait_seq) || ((p_s->now - p_s->wait_ms) >= OBSTACLE_NAV_SERVO_WAIT_MS)))
    {
        p_s->b_servo_wait = false;
    }
    return p_s->b_servo_wait;
}

static void nav_report_state(sim_t * p_s, uint32_t status)
{
    uint32_t const state = obstacle_pack_get(status, OBSTACLE_ST_STATE_SH, OBSTACLE_ST_STATE_MK);

    if (p_s->last_state != state)
    {
        uint8_t const seq =
            (uint8_t)obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);

        p_s->last_state = (uint8_t)state;
        if (OBSTACLE_STATE_WAIT_STOP == state)
        {
            p_s->base_left_um  = p_s->wheel_left_um;
            p_s->base_right_um = p_s->wheel_right_um;
        }
        if (OBSTACLE_STATE_BYPASS == state)
        {
            reset_pose(p_s);
            p_s->bypass_count++;
        }
        if (OBSTACLE_STATE_REACQUIRE == state)
        {
            double const mean_um = (p_s->wheel_left_um + p_s->wheel_right_um) / 2.0;

            p_s->reacquire_start = p_s->now;
            p_s->b_hug           = false;
            p_s->clear_checks    = 0U;
            p_s->b_have_prev_gap = false;
            p_s->b_passed        = false;
            if (p_s->b_wall)
            {
                p_s->phase              = OBSTACLE_PHASE_ALONG;
                p_s->along_head_deci    = (int32_t)OBSTACLE_WALL_PIVOT_DECI;
                p_s->leg_base_um        = mean_um;
                p_s->b_ir_armed         = true;
                p_s->b_ir_cleared       = false;
                p_s->b_hug              = true;
                p_s->hug_start          = p_s->now;
                p_s->hug_head_start_deg = (double)head_deci(p_s) / 10.0;
                p_s->head_min_deg       = p_s->hug_head_start_deg;
                p_s->head_max_deg       = p_s->hug_head_start_deg;
                p_s->prev_gap_mm        = (int32_t)OBSTACLE_WALL_HOLD_MM;
                p_s->b_have_prev_gap    = true;
                p_s->prev_lean_deci     = 0;
                p_s->wall_trim_deci     = 0;
            }
            else
            {
                p_s->phase = OBSTACLE_PHASE_OUT;
            }
        }
        if ((OBSTACLE_STATE_BYPASS == state) || (OBSTACLE_STATE_REACQUIRE == state) ||
            (OBSTACLE_STATE_MONITOR == state))
        {
            p_s->b_servo_wait = true;
            p_s->wait_seq     = seq;
            p_s->wait_ms      = p_s->now;
        }
    }
}

/* head is the heading the bypass law commands. */
static void observe_hug(sim_t * p_s, double head)
{
    if (!p_s->b_hug)
    {
        p_s->b_hug              = true;
        p_s->hug_start          = p_s->now;
        p_s->hug_head_start_deg = head;
        p_s->head_min_deg       = head;
        p_s->head_max_deg       = head;
    }
    if (head < p_s->head_min_deg)
    {
        p_s->head_min_deg = head;
    }
    if (head > p_s->head_max_deg)
    {
        p_s->head_max_deg = head;
    }
}

static void nav_reacquire(sim_t * p_s, uint32_t status, bool b_line)
{
    int32_t left_um_s = 0;
    int32_t right_um_s = 0;
    uint8_t steer = OBSTACLE_STEER_HOLD;
    bool b_done = false;
    double const mean_um = (p_s->wheel_left_um + p_s->wheel_right_um) / 2.0;

    update_pose(p_s);
    if (servo_moving(p_s, status))
    {
        p_s->b_hold = true;
        return;
    }
    p_s->b_hold = false;

    if (p_s->b_ir_armed && (!p_s->b_ir_cleared) && (!b_line))
    {
        p_s->b_ir_cleared = true;
    }

    switch (p_s->phase)
    {
        case OBSTACLE_PHASE_TURN:
        {
            int32_t const target_head = side_sign(p_s) * (int32_t)p_s->start_deg * 10;

            obstacle_plan_wheel_pivot(head_deci(p_s), target_head, &left_um_s, &right_um_s, &steer, &b_done);
            if (b_done)
            {
                if (p_s->b_wall)
                {
                    p_s->phase           = OBSTACLE_PHASE_ALONG;
                    p_s->along_head_deci = (int32_t)OBSTACLE_WALL_PIVOT_DECI;
                    p_s->leg_base_um     = mean_um;
                    p_s->b_ir_armed      = true;
                    p_s->b_ir_cleared    = false;
                    observe_hug(p_s, (double)p_s->along_head_deci / 10.0);
                }
                else
                {
                    p_s->phase = OBSTACLE_PHASE_OUT;
                }
            }
        }
        break;

        case OBSTACLE_PHASE_OUT:
        {
            int32_t const target_head = side_sign(p_s) * (int32_t)p_s->start_deg * 10;

            obstacle_plan_wheel_straight(head_deci(p_s), target_head, (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                         &left_um_s, &right_um_s, &steer);
            if (lateral_mm(p_s) >= p_s->need_mm)
            {
                p_s->phase = OBSTACLE_PHASE_ALIGN;
            }
        }
        break;

        case OBSTACLE_PHASE_ALIGN:
        {
            int32_t const target_head = 0;

            obstacle_plan_wheel_pivot(head_deci(p_s), target_head, &left_um_s, &right_um_s, &steer, &b_done);
            if (b_done)
            {
                p_s->b_aligned_before_hug = true;
                p_s->phase                = OBSTACLE_PHASE_ALONG;
                p_s->along_head_deci      = 0;
                p_s->leg_base_um          = mean_um;
                p_s->b_ir_armed           = true;
                p_s->b_ir_cleared         = false;
                p_s->b_hug                = true;
                p_s->hug_head_start_deg   = (double)head_deci(p_s) / 10.0;
                observe_hug(p_s, (double)p_s->along_head_deci / 10.0);
            }
        }
        break;

        case OBSTACLE_PHASE_ALONG:
        {
            int32_t const leg_dist = (int32_t)((mean_um - p_s->leg_base_um) / 1000.0);

            obstacle_plan_wheel_straight(head_deci(p_s), p_s->along_head_deci,
                                         (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                         &left_um_s, &right_um_s, &steer);
            observe_hug(p_s, (double)(p_s->along_head_deci + p_s->wall_trim_deci) / 10.0);
            if (leg_dist >= (int32_t)OBSTACLE_SIDE_LOOK_MM)
            {
                p_s->phase        = OBSTACLE_PHASE_LOOK_STOP;
                p_s->look_stop_ms = p_s->now;
                left_um_s         = 0;
                right_um_s        = 0;
                steer             = OBSTACLE_STEER_HOLD;
            }
        }
        break;

        case OBSTACLE_PHASE_LOOK_STOP:
        {
            left_um_s = 0;
            right_um_s = 0;
            steer = OBSTACLE_STEER_HOLD;
            if ((p_s->now - p_s->look_stop_ms) >= OBSTACLE_STOP_SETTLE_MS)
            {
                p_s->phase = OBSTACLE_PHASE_LOOK_IN;
            }
        }
        break;

        case OBSTACLE_PHASE_LOOK_IN:
        {
            int32_t const look_head = p_s->along_head_deci - (side_sign(p_s) * (int32_t)OBSTACLE_WALL_PIVOT_DECI);

            obstacle_plan_wheel_pivot(head_deci(p_s), look_head, &left_um_s, &right_um_s, &steer, &b_done);
            if (b_done)
            {
                p_s->phase              = OBSTACLE_PHASE_LOOK_PING;
                p_s->side_ping_count    = 0U;
                p_s->side_agree_count   = 0U;
                p_s->last_side_dist_mm  = 0U;
                left_um_s               = 0;
                right_um_s              = 0;
                steer                   = OBSTACLE_STEER_HOLD;
            }
        }
        break;

        case OBSTACLE_PHASE_LOOK_PING:
        {
            uint32_t const cls  = obstacle_pack_get(status, OBSTACLE_ST_CLASS_SH, OBSTACLE_ST_CLASS_MK);
            uint32_t const dist = obstacle_pack_get(status, OBSTACLE_ST_DIST_SH, OBSTACLE_ST_DIST_MK);
            uint32_t const seq  = obstacle_pack_get(status, OBSTACLE_ST_SEQ_SH, OBSTACLE_ST_SEQ_MK);
            bool b_side_done    = false;

            left_um_s = 0;
            right_um_s = 0;
            steer = OBSTACLE_STEER_HOLD;

            if (seq != p_s->wait_seq)
            {
                if (NULL != getenv("LOOK_DEBUG"))
                {
                    printf("LOOK_PING: t=%u seq=%u cls=%u dist=%u agree=%u last_dist=%u\n",
                           p_s->now, seq, cls, dist, p_s->side_agree_count, p_s->last_side_dist_mm);
                }
                p_s->wait_seq = (uint8_t)seq;
                p_s->side_ping_count++;

                if ((OBSTACLE_CLASS_OPEN == cls) ||
                    ((OBSTACLE_CLASS_VALID == cls) && (dist >= OBSTACLE_CORNER_CLEAR_MM)))
                {
                    p_s->clear_checks++;
                    if (p_s->clear_checks >= OBSTACLE_PASS_CLEARS)
                    {
                        p_s->b_passed = true;
                    }
                    p_s->b_have_prev_gap = false;
                    b_side_done = true;
                }
                else if (OBSTACLE_CLASS_VALID == cls)
                {
                    if ((p_s->side_agree_count > 0U) &&
                        (geom_abs((int32_t)dist - (int32_t)p_s->last_side_dist_mm) <= (int32_t)OBSTACLE_AGREE_MM))
                    {
                        int32_t const gap_mm = (int32_t)dist + (int32_t)OBSTACLE_SONAR_FWD_MM;
                        int32_t const target_gap = p_s->b_wall ? (int32_t)OBSTACLE_WALL_HOLD_MM : (int32_t)OBSTACLE_BOX_GAP_MM;
                        int32_t const lean = obstacle_plan_side_lean_deci(gap_mm, p_s->prev_gap_mm,
                                                                          p_s->b_have_prev_gap, target_gap);

                        if (p_s->b_wall && p_s->b_have_prev_gap)
                        {
                            int32_t const trim = obstacle_plan_side_trim_deci(gap_mm, p_s->prev_gap_mm,
                                                                              p_s->prev_lean_deci);
                            p_s->wall_trim_deci += trim;
                            if (p_s->wall_trim_deci > 800)
                            {
                                p_s->wall_trim_deci = 800;
                            }
                            else if (p_s->wall_trim_deci < -800)
                            {
                                p_s->wall_trim_deci = -800;
                            }
                        }
                        int32_t const base_head = p_s->b_wall ? ((int32_t)OBSTACLE_WALL_PIVOT_DECI - p_s->wall_trim_deci) : 0;

                        p_s->along_head_deci = base_head + (side_sign(p_s) * lean);
                        p_s->prev_lean_deci  = lean;
                        p_s->prev_gap_mm     = gap_mm;
                        p_s->b_have_prev_gap = true;
                        p_s->clear_checks    = 0U;
                        b_side_done          = true;
                    }
                    else
                    {
                        p_s->last_side_dist_mm = (uint16_t)dist;
                        p_s->side_agree_count++;
                    }
                }
                else
                {
                    /* NO_ECHO, GLITCH or STUCK: wait for next ping */
                }

                if ((!b_side_done) && (p_s->side_ping_count >= OBSTACLE_SIDE_PINGS_MAX))
                {
                    b_side_done = true;
                }
            }

            if (b_side_done)
            {
                p_s->phase = OBSTACLE_PHASE_LOOK_OUT;
            }
        }
        break;

        case OBSTACLE_PHASE_LOOK_OUT:
        {
            obstacle_plan_wheel_pivot(head_deci(p_s), p_s->along_head_deci, &left_um_s, &right_um_s, &steer, &b_done);
            if (b_done)
            {
                if (p_s->b_passed && (!p_s->b_wall))
                {
                    p_s->phase       = OBSTACLE_PHASE_EXTRA;
                    p_s->leg_base_um = mean_um;
                }
                else
                {
                    p_s->phase       = OBSTACLE_PHASE_ALONG;
                    p_s->leg_base_um = mean_um;
                }
            }
        }
        break;

        case OBSTACLE_PHASE_EXTRA:
        {
            int32_t const leg_dist = (int32_t)((mean_um - p_s->leg_base_um) / 1000.0);

            obstacle_plan_wheel_straight(head_deci(p_s), p_s->along_head_deci,
                                         (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                         &left_um_s, &right_um_s, &steer);
            observe_hug(p_s, (double)p_s->along_head_deci / 10.0);
            if (leg_dist >= (int32_t)OBSTACLE_PASS_EXTRA_MM)
            {
                p_s->phase = OBSTACLE_PHASE_RETURN_PIVOT;
            }
        }
        break;

        case OBSTACLE_PHASE_RETURN_PIVOT:
        {
            int32_t const ret_head = -side_sign(p_s) * (int32_t)OBSTACLE_RETURN_DECI;

            obstacle_plan_wheel_pivot(head_deci(p_s), ret_head, &left_um_s, &right_um_s, &steer, &b_done);
            if (b_done)
            {
                p_s->phase = OBSTACLE_PHASE_RETURN_DRIVE;
            }
        }
        break;

        case OBSTACLE_PHASE_RETURN_DRIVE:
        {
            int32_t const ret_head = -side_sign(p_s) * (int32_t)OBSTACLE_RETURN_DECI;

            obstacle_plan_wheel_straight(head_deci(p_s), ret_head,
                                         (int32_t)OBSTACLE_BYPASS_SPEED_MM_S,
                                         &left_um_s, &right_um_s, &steer);
            observe_hug(p_s, (double)ret_head / 10.0);
            if (p_s->b_ir_cleared && b_line && ((p_s->now - p_s->line_ms) >= OBSTACLE_NAV_LINE_REPEAT_MS))
            {
                p_s->line_ms = p_s->now;
                nav_post(p_s, OBSTACLE_CMD_LINE_FOUND, p_s->req_id);
            }
        }
        break;

        default:
            left_um_s  = 0;
            right_um_s = 0;
            steer      = OBSTACLE_STEER_HOLD;
        break;
    }

    if (p_s->b_wall && (p_s->phase == OBSTACLE_PHASE_ALONG) && p_s->b_ir_cleared && b_line &&
        ((p_s->now - p_s->line_ms) >= OBSTACLE_NAV_LINE_REPEAT_MS))
    {
        p_s->line_ms = p_s->now;
        nav_post(p_s, OBSTACLE_CMD_LINE_FOUND, p_s->req_id);
    }

    drive_wheels(p_s, left_um_s, right_um_s);
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
                int32_t const target_head = side_sign(p_s) * (int32_t)p_s->start_deg * 10;

                p_s->b_hold = false;
                obstacle_plan_wheel_pivot(head_deci(p_s), target_head, &left_um_s, &right_um_s, &steer, &b_done);
                if (b_done)
                {
                    p_s->b_turned = true;
                    if ((p_s->now - p_s->line_ms) >= OBSTACLE_NAV_LINE_REPEAT_MS)
                    {
                        p_s->line_ms = p_s->now;
                        nav_post(p_s, OBSTACLE_CMD_TURNED, p_s->req_id);
                    }
                }
                drive_wheels(p_s, left_um_s, right_um_s);
            }
        break;

        case OBSTACLE_STATE_REACQUIRE:
            nav_reacquire(p_s, status, b_line);
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
        double const dth = (tr - tl) / OBSTACLE_WHEEL_TRACK_MM;

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
    }

    if (p_s->b_hug && (OBSTACLE_STATE_REACQUIRE == p_s->fsm.state) &&
        (OBSTACLE_PHASE_ALONG == p_s->phase) && (p_s->cfg.rects > 0))
    {
        /* Distance from the axle to the first rectangle (the wall or box),
         * measured at right angles to the face it stands on. */
        double const sy = p_s->y + (SENSOR_FWD_MM * cos(p_s->th));
        double const d  = p_s->cfg.rect[0].y0 - sy;

        if ((p_s->now - p_s->hug_start) > 1500U)
        {
            if (d < p_s->dist_min_mm)
            {
                p_s->dist_min_mm = d;
            }
            if (d > p_s->dist_max_mm)
            {
                p_s->dist_max_mm = d;
            }
        }
    }

    if ((p_s->window_from > 0U) && (p_s->now >= p_s->window_from) && (p_s->now < p_s->cfg.open_to_ms) &&
        (OBSTACLE_PHASE_ALONG == p_s->phase))
    {
        double const head = side_sign(p_s) * head_deci(p_s) / 10.0;

        if (head < p_s->window_head_min_deg)
        {
            p_s->window_head_min_deg = head;
        }
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
    p_s->dist_min_mm    = 1.0e9;
    p_s->dist_max_mm    = -1.0e9;
    p_s->body_min_mm    = 1.0e9;
    p_s->head_min_deg   = 1.0e9;
    p_s->head_max_deg   = -1.0e9;
    p_s->window_head_min_deg = 1.0e9;
    p_s->last_cause     = OBSTACLE_CAUSE_NONE;
    p_s->stop_ms        = 0U;
    p_s->side           = OBSTACLE_SIDE_LEFT;
    p_s->phase          = OBSTACLE_PHASE_TURN;
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
    p_cfg->rects++;
}

static void print_run(char const * p_name, sim_t const * p_s)
{
    if (NULL != getenv("SIM_TRACE"))
    {
        printf("  [%s] t=%u ms stop cause=%u pos=(%.0f,%.0f) th=%.1f hug_head %.1f..%.1f dist %.0f..%.0f "
               "body_min %.0f rescans=%u bypasses=%u fwd %u/%u\n",
               p_name, p_s->now, p_s->last_cause, p_s->x, p_s->y, p_s->th / DEG2RAD,
               p_s->head_min_deg, p_s->head_max_deg, p_s->dist_min_mm, p_s->dist_max_mm,
               p_s->body_min_mm, p_s->rescans, p_s->bypass_count, p_s->forward_pings,
               p_s->reacquire_pings);
    }
}

/* ---------------------------------------------------------------- */
/* Tests.                                                            */
/* ---------------------------------------------------------------- */

/* A flat wall across the path, long enough to run along for a minute. */
static void wall_world(sim_cfg_t * p_cfg, double wall_y)
{
    cfg_default(p_cfg);
    add_rect(p_cfg, -1000.0, wall_y, 20000.0, wall_y + 50.0);
}

/* The wall plan: pivot to +90, hold 300 mm +-50, heading inside +60..+120,
 * no stop on the distance budget, ends on OBSTACLE_HUG_MAX_MS. */
static void test_wall_follow_to_time_limit(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    s = sim_new(&cfg);
    sim_run(s);
    print_run("wall", s);

    CHECK(s->bypass_count == 1U);
    CHECK(s->b_wall);
    CHECK(s->b_hug);

    /* Pivoted to about +90 before the hug began. */
    CHECK(s->hug_head_start_deg >= 90.0);
    CHECK(s->hug_head_start_deg <= 100.0);

    /* Heading never leaves the +60..+120 window. */
    CHECK(s->head_min_deg >= 60.0);
    CHECK(s->head_max_deg <= 120.0);

    /* 300 mm +-50 from the wall for the whole run. */
    CHECK(s->dist_min_mm >= 250.0);
    CHECK(s->dist_max_mm <= 350.0);

    /* It never touched the wall. */
    CHECK(s->body_min_mm > 20.0);

    /* Ends on the time limit, not on the 3.8 m distance budget, and the car
     * really drove farther than the budget. */
    if (NULL != getenv("SIM_DEBUG"))
    {
        printf("WALL_CHECK: stop=%u reacq_start=%u diff=%u max=%u hug_lim=%u odo=%.1f\n",
               s->stop_ms, s->reacquire_start, s->stop_ms - s->reacquire_start,
               OBSTACLE_HUG_MAX_MS, s->fsm.hug_limit_mm, mean_odo_mm(s));
    }

    CHECK(s->fsm.state == OBSTACLE_STATE_STOPPED);
    CHECK(s->last_cause == OBSTACLE_CAUSE_LINE_TIMEOUT);
    CHECK((s->stop_ms - s->reacquire_start) >= (OBSTACLE_HUG_MAX_MS - 200U));
    CHECK((s->stop_ms - s->reacquire_start) <= (OBSTACLE_HUG_MAX_MS + 200U));
    CHECK(s->fsm.hug_limit_mm < 5000U);
    CHECK((mean_odo_mm(s) - 0.0) > (double)s->fsm.hug_limit_mm);

    /* No rescan, no fault on the way. */
    CHECK(s->rescans == 0U);
}

/* Forward-only moving sonar: while moving in REACQUIRE, pings face forward. */
static void test_forward_sonar_while_moving(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    cfg.max_ms = 30000U;
    s = sim_new(&cfg);
    sim_run(s);

    CHECK(s->reacquire_pings > 20U);
    CHECK(s->forward_pings > 0U);
    CHECK(s->side_pings > 0U);
    CHECK(s->reacquire_pings == (s->forward_pings + s->side_pings));
}

/* OPEN readings on the wall ray never turn the car towards the wall. */
static void test_wall_open_never_turns_in(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    cfg.max_ms = 40000U;
    s = sim_new(&cfg);

    /* Let it settle into the hug, then every side ping reads OPEN for 12 s. */
    s->cfg.open_from_ms = 16000U;
    s->cfg.open_to_ms   = 28000U;
    s->window_from      = 16000U;
    sim_run(s);
    print_run("open", s);

    CHECK(s->bypass_count == 1U);
    CHECK(s->window_head_min_deg >= 87.0);
    CHECK(s->head_min_deg >= 60.0);
    CHECK(s->head_max_deg <= 120.0);
    CHECK(s->body_min_mm > 20.0);
    CHECK(s->rescans == 0U);

    /* One OPEN in every three side pings all run long: still in the band. */
    wall_world(&cfg, 700.0);
    cfg.max_ms    = 40000U;
    cfg.drop_every = 3U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("drop", s);
    CHECK(s->head_min_deg >= 60.0);
    CHECK(s->head_max_deg <= 120.0);
    CHECK(s->dist_min_mm >= 250.0);
    CHECK(s->dist_max_mm <= 350.0);
    CHECK(s->body_min_mm > 20.0);
}

/* Wheel scale errors (the true travel differs from the odometry, +-1.5 % per
 * wheel): the wall trim keeps the car off the wall and inside the heading
 * window for the whole 40 s. Without the trim the car leaves the wall or
 * runs into it. */
static void test_wall_wheel_error(void)
{
    double const err[3] = {0.985, 1.0, 1.015};
    int          i      = 0;

    for (i = 0; i < 3; i++)
    {
        sim_cfg_t cfg;
        sim_t *   s = NULL;

        wall_world(&cfg, 700.0);
        cfg.max_ms      = 40000U;
        cfg.scale_left  = err[i];
        cfg.scale_right = 2.0 - err[i];
        s = sim_new(&cfg);
        sim_run(s);
        print_run("scale", s);
        CHECK(s->body_min_mm > 20.0);
        CHECK(s->head_min_deg >= 60.0);
        CHECK(s->head_max_deg <= 120.0);
        /* Nominal tolerance is 300 +-50 mm; with a 3 % wheel difference the
         * wall trim still keeps the car inside 300 +-65 mm. */
        CHECK(s->dist_min_mm >= 235.0);
        CHECK(s->dist_max_mm <= 365.0);
        CHECK(s->rescans == 0U);
    }
}

/* A line across the wall's path ends the follow early: the car stops
 * following and goes back to MONITOR. */
static void test_wall_ends_on_line(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    cfg.lines     = 2;
    cfg.line_x[1] = 2500.0;
    cfg.b_stop_on_monitor = true;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("line", s);

    CHECK(s->b_reached_monitor_after_bypass);
    CHECK(s->fsm.state == OBSTACLE_STATE_MONITOR);
    CHECK((s->now - s->reacquire_start) < OBSTACLE_HUG_MAX_MS);
    CHECK(s->x > 2400.0);
    CHECK(s->body_min_mm > 20.0);
}

/* A wall with a corner ahead: the forward look stops the car before the
 * corner and it rescans instead of driving into it. */
static void test_wall_corner_ahead(void)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    wall_world(&cfg, 700.0);
    add_rect(&cfg, 1800.0, -3000.0, 1850.0, 700.0);
    cfg.max_ms = 70000U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run("corner", s);

    CHECK(s->rescans >= 1U);
    CHECK(s->bypass_count >= 2U);
    CHECK(s->body_min_mm > 20.0);
    CHECK(s->last_cause != OBSTACLE_CAUSE_COLLISION);
    CHECK(s->x < 1800.0);
}

/* A box: out, straighten, hug, back to the line, no collision. */
static void box_run(double x0, double x1, double front_y, double depth, char const * p_name,
                    bool b_expect_left)
{
    sim_cfg_t cfg;
    sim_t *   s = NULL;

    cfg_default(&cfg);
    add_rect(&cfg, x0, front_y, x1, front_y + depth);
    cfg.b_stop_on_monitor = true;
    cfg.max_ms            = 60000U;
    s = sim_new(&cfg);
    sim_run(s);
    print_run(p_name, s);

    CHECK(s->bypass_count == 1U);
    CHECK(!s->b_wall);
    CHECK(s->b_reached_monitor_after_bypass);
    CHECK(s->body_min_mm > 15.0);
    CHECK(s->rescans == 0U);
    CHECK(s->b_aligned_before_hug);
    CHECK(fabs(s->hug_head_start_deg) <= 7.0);
    CHECK(s->head_max_deg <= 67.0);
    CHECK(s->head_min_deg >= -67.0);
    CHECK((s->side == OBSTACLE_SIDE_LEFT) == b_expect_left);
}

static void test_box_bypass(void)
{
    box_run(-90.0, 90.0, 560.0, 180.0, "box", true);
    box_run(-150.0, 30.0, 560.0, 180.0, "box-right-of-line", true);
    box_run(-30.0, 150.0, 560.0, 180.0, "box-left-of-line", false);
    box_run(-90.0, 90.0, 560.0, 300.0, "box-deep", true);
}

int main(void)
{
    test_wall_follow_to_time_limit();
    test_forward_sonar_while_moving();
    test_wall_open_never_turns_in();
    test_wall_wheel_error();
    test_wall_ends_on_line();
    test_wall_corner_ahead();
    test_box_bypass();

    free(g_sim);

    if (g_failures == 0)
    {
        printf("obstacle_bypass_sim: all checks passed\n");
        return 0;
    }
    printf("obstacle_bypass_sim: %d check(s) failed\n", g_failures);
    return 1;
}
