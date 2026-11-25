#include "pilot_sim.h"

#include <math.h>
#include <string.h>

#define PI_RAD            3.14159265358979323846
#define STEP_MM           2.0
#define DT_S              0.001
#define TICK_MS           10
#define SAMPLE_MS         5
#define START_X_MM        (-200.0)

/* Motor model per wheel. The numbers are guesses, not car measurements. */
#define DEADBAND_MM_S     30.0   /* a target below this gives no motion      */
#define STICTION_MM_S     45.0   /* a wheel at rest needs this to start      */
#define AT_REST_MM_S      2.0
#define LAG_UP_S          0.060
#define LAG_DOWN_S        0.020
#define SCRUB_YAW         0.8    /* yaw kept when the wheels run opposite    */

/* Sensors: chatter near the edge of the line, then a debounce. */
#define EDGE_ZONE_MM      1.5
#define CHATTER           0.3
#define DEBOUNCE          3

/* Start pose spread. */
#define START_OFF_MM      8.0
#define START_YAW_DEG     5.0

#define LCG_MUL           1103515245U
#define LCG_ADD           12345U
#define LCG_SHIFT         16U
#define LCG_MASK          0x7FFFU
#define LCG_RANGE         32768.0

static double rnd(pilot_sim_t * p_sim)
{
    p_sim->rng = (p_sim->rng * LCG_MUL) + LCG_ADD;
    return (double)((p_sim->rng >> LCG_SHIFT) & LCG_MASK) / LCG_RANGE;
}

static double srnd(pilot_sim_t * p_sim)
{
    p_sim->sonar_rng = (p_sim->sonar_rng * LCG_MUL) + LCG_ADD;
    return (double)((p_sim->sonar_rng >> LCG_SHIFT) & LCG_MASK) / LCG_RANGE;
}

static void line_build(pilot_sim_t * p_sim, pilot_sim_seg_t const * p_segs)
{
    double x = START_X_MM, y = 0.0, psi = 0.0;
    int i;

    p_sim->line_n = 0;
    for (i = 0; (p_segs[i].len_mm > 0.0) || (p_segs[i].turn_deg != 0.0); i++)
    {
        double const len = (p_segs[i].radius > 0.0)
                               ? (fabs(p_segs[i].turn_deg) * PI_RAD / 180.0) * p_segs[i].radius
                               : p_segs[i].len_mm;
        double const dpsi = (p_segs[i].radius > 0.0)
                                ? (p_segs[i].turn_deg * PI_RAD / 180.0) * (STEP_MM / len)
                                : 0.0;
        double done = 0.0;

        while ((done < len) && (p_sim->line_n < PILOT_SIM_MAX_PTS))
        {
            p_sim->line_x[p_sim->line_n] = x;
            p_sim->line_y[p_sim->line_n] = y;
            p_sim->line_n++;
            x += STEP_MM * cos(psi + (dpsi / 2.0));
            y += STEP_MM * sin(psi + (dpsi / 2.0));
            psi += dpsi;
            done += STEP_MM;
        }
    }
}

#define GRID_CELL_MM      20.0
#define GRID_GROW         1.5
#define GRID_SLACK        1e-6

static int iabs(int v)
{
    return (v < 0) ? -v : v;
}

static int imax(int a, int b)
{
    return (a > b) ? a : b;
}

static int grid_index(pilot_sim_t const * p_sim, double x, double y)
{
    int const cx = (int)((x - p_sim->grid_x0) / p_sim->grid_cell);
    int const cy = (int)((y - p_sim->grid_y0) / p_sim->grid_cell);

    return (cy * p_sim->grid_nx) + cx;
}

/* Bin the line points into square cells, at least GRID_CELL_MM wide and few
 * enough to fit the cell table. */
static void grid_build(pilot_sim_t * p_sim)
{
    double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
    int i, c;

    for (i = 0; i < p_sim->line_n; i++)
    {
        double const x = p_sim->line_x[i], y = p_sim->line_y[i];

        min_x = ((0 == i) || (x < min_x)) ? x : min_x;
        max_x = ((0 == i) || (x > max_x)) ? x : max_x;
        min_y = ((0 == i) || (y < min_y)) ? y : min_y;
        max_y = ((0 == i) || (y > max_y)) ? y : max_y;
    }
    p_sim->grid_x0 = min_x;
    p_sim->grid_y0 = min_y;
    p_sim->grid_cell = GRID_CELL_MM;
    do
    {
        p_sim->grid_nx = (int)((max_x - min_x) / p_sim->grid_cell) + 1;
        p_sim->grid_ny = (int)((max_y - min_y) / p_sim->grid_cell) + 1;
        if ((p_sim->grid_nx * p_sim->grid_ny) > PILOT_SIM_GRID_CELLS)
        {
            p_sim->grid_cell *= GRID_GROW;
        }
    } while ((p_sim->grid_nx * p_sim->grid_ny) > PILOT_SIM_GRID_CELLS);

    /* Counting sort: count per cell, turn the counts into end offsets, then
     * fill from the back so each cell ends at its start offset. */
    for (c = 0; c <= (p_sim->grid_nx * p_sim->grid_ny); c++)
    {
        p_sim->grid_start[c] = 0U;
    }
    for (i = 0; i < p_sim->line_n; i++)
    {
        p_sim->grid_start[grid_index(p_sim, p_sim->line_x[i], p_sim->line_y[i])]++;
    }
    for (c = 1; c < (p_sim->grid_nx * p_sim->grid_ny); c++)
    {
        p_sim->grid_start[c] = (uint16_t)(p_sim->grid_start[c] + p_sim->grid_start[c - 1]);
    }
    p_sim->grid_start[p_sim->grid_nx * p_sim->grid_ny] = (uint16_t)p_sim->line_n;
    for (i = p_sim->line_n - 1; i >= 0; i--)
    {
        int const cell = grid_index(p_sim, p_sim->line_x[i], p_sim->line_y[i]);

        p_sim->grid_pt[--p_sim->grid_start[cell]] = (uint16_t)i;
    }
}

void pilot_sim_reindex(pilot_sim_t * p_sim)
{
    grid_build(p_sim);
}

static double grid_cell_best(pilot_sim_t const * p_sim, int cell, double px, double py,
                             double best)
{
    double nearest = best;
    int k;

    for (k = p_sim->grid_start[cell]; k < p_sim->grid_start[cell + 1]; k++)
    {
        int const i = p_sim->grid_pt[k];
        double const dx = px - p_sim->line_x[i];
        double const dy = py - p_sim->line_y[i];
        double const d2 = (dx * dx) + (dy * dy);

        if (d2 < nearest)
        {
            nearest = d2;
        }
    }
    return nearest;
}

/* Distance to the nearest line point. The cells are scanned in rings around
 * the cell of the query; a point in ring r is at least (r - 1) cells away, so
 * the scan ends when the best point is nearer than that. The result is the
 * minimum over the same squared distances as a scan of all points. */
static double line_dist(pilot_sim_t const * p_sim, double px, double py)
{
    double best = 1e18;
    double const fx = floor((px - p_sim->grid_x0) / p_sim->grid_cell);
    double const fy = floor((py - p_sim->grid_y0) / p_sim->grid_cell);
    int const qx = (int)((fx < -1e6) ? -1e6 : ((fx > 1e6) ? 1e6 : fx));
    int const qy = (int)((fy < -1e6) ? -1e6 : ((fy > 1e6) ? 1e6 : fy));
    int const nx = p_sim->grid_nx, ny = p_sim->grid_ny;
    int r_max = -1, r, i, j;

    if (p_sim->line_n > 0)
    {
        /* The ring that reaches the farthest cell of the grid. */
        int const reach_x = imax(iabs(qx), iabs((nx - 1) - qx));
        int const reach_y = imax(iabs(qy), iabs((ny - 1) - qy));

        r_max = imax(reach_x, reach_y);
    }
    for (r = 0; r <= r_max; r++)
    {
        double const reach = (double)(r - 1) * p_sim->grid_cell;

        if ((r > 1) && (best < ((reach * reach) - GRID_SLACK)))
        {
            break;
        }
        for (j = qy - r; j <= (qy + r); j++)
        {
            int const step = ((j == (qy - r)) || (j == (qy + r))) ? 1 : (2 * r);

            if ((j < 0) || (j >= ny))
            {
                continue;
            }
            for (i = qx - r; i <= (qx + r); i += (step > 0) ? step : 1)
            {
                if ((i >= 0) && (i < nx))
                {
                    best = grid_cell_best(p_sim, (j * nx) + i, px, py, best);
                }
            }
        }
    }
    return sqrt(best);
}

#define SONAR_SEED_MIX    0x5EED5EEDU
#define NO_HIT            (1e18)

void pilot_sim_world_reset(pilot_sim_t * p_sim, uint32_t seed)
{
    p_sim->wall_n = 0;
    p_sim->sonar_rng = seed ^ SONAR_SEED_MIX;
    p_sim->sonar_half_deg = PILOT_SIM_SONAR_HALF_DEG;
}

bool pilot_sim_add_wall(pilot_sim_t * p_sim, double ax, double ay, double bx, double by)
{
    pilot_sim_wall_t * p_w;

    if (p_sim->wall_n >= PILOT_SIM_MAX_WALLS)
    {
        return false;
    }
    p_w = &p_sim->walls[p_sim->wall_n];
    p_w->ax = ax;
    p_w->ay = ay;
    p_w->bx = bx;
    p_w->by = by;
    p_w->b_blind = false;
    p_sim->wall_n++;
    return true;
}

bool pilot_sim_add_blind_wall(pilot_sim_t * p_sim, double ax, double ay, double bx, double by)
{
    bool const b_ok = pilot_sim_add_wall(p_sim, ax, ay, bx, by);

    if (b_ok)
    {
        p_sim->walls[p_sim->wall_n - 1].b_blind = true;
    }
    return b_ok;
}

bool pilot_sim_add_box(pilot_sim_t * p_sim, double cx, double cy, double width_mm,
                       double depth_mm, double rot_rad)
{
    double const c = cos(rot_rad), s = sin(rot_rad);
    double const hx = depth_mm / 2.0, hy = width_mm / 2.0;
    double const lx[4] = {hx, hx, -hx, -hx};
    double const ly[4] = {-hy, hy, hy, -hy};
    double px[4], py[4];
    int i;
    bool b_ok = true;

    for (i = 0; i < 4; i++)
    {
        px[i] = cx + (lx[i] * c) - (ly[i] * s);
        py[i] = cy + (lx[i] * s) + (ly[i] * c);
    }
    for (i = 0; i < 4; i++)
    {
        b_ok = pilot_sim_add_wall(p_sim, px[i], py[i], px[(i + 1) % 4], py[(i + 1) % 4]) && b_ok;
    }
    return b_ok;
}

bool pilot_sim_add_room(pilot_sim_t * p_sim, double x0, double y0, double x1, double y1)
{
    bool b_ok = pilot_sim_add_wall(p_sim, x0, y0, x1, y0);

    b_ok = pilot_sim_add_wall(p_sim, x1, y0, x1, y1) && b_ok;
    b_ok = pilot_sim_add_wall(p_sim, x1, y1, x0, y1) && b_ok;
    b_ok = pilot_sim_add_wall(p_sim, x0, y1, x0, y0) && b_ok;
    return b_ok;
}

/* Nearest distance from the origin to the part of a wall inside the beam
 * sector, or NO_HIT when the face is too oblique to echo or misses the sector. */
static double wall_hit(pilot_sim_wall_t const * p_w, double ox, double oy, double ray_rad,
                       double half_deg)
{
    double const half = half_deg * PI_RAD / 180.0;
    double const dx = p_w->bx - p_w->ax, dy = p_w->by - p_w->ay;
    double const len = sqrt((dx * dx) + (dy * dy));
    double const cx = cos(ray_rad), cy = sin(ray_rad);
    double const lox = cos(ray_rad - half), loy = sin(ray_rad - half);
    double const hix = cos(ray_rad + half), hiy = sin(ray_rad + half);
    double t0 = 0.0, t1 = 1.0, t, best;
    double const ax = p_w->ax - ox, ay = p_w->ay - oy;
    /* The wedge is cross(lo, p) >= 0 and cross(p, hi) >= 0, each linear in t:
     * f(t) = f0 + t * f1. */
    double const f0[2] = {(lox * ay) - (loy * ax), (ax * hiy) - (ay * hix)};
    double const f1[2] = {(lox * dy) - (loy * dx), (dx * hiy) - (dy * hix)};
    int i;

    if (len < 1e-9)
    {
        return NO_HIT;
    }
    /* Incidence of the beam centre: angle to the face normal. */
    if ((fabs((cx * dy) - (cy * dx)) / len) < cos(PILOT_SIM_SONAR_MAX_DEG * PI_RAD / 180.0))
    {
        return NO_HIT;
    }
    for (i = 0; i < 2; i++)
    {
        if (fabs(f1[i]) < 1e-12)
        {
            if (f0[i] < 0.0)
            {
                return NO_HIT;
            }
        }
        else
        {
            t = -f0[i] / f1[i];
            if (f1[i] > 0.0)
            {
                t0 = (t > t0) ? t : t0;
            }
            else
            {
                t1 = (t < t1) ? t : t1;
            }
        }
    }
    if (t0 > t1)
    {
        return NO_HIT;
    }
    /* Closest point of the clipped piece to the origin. */
    t = -((ax * dx) + (ay * dy)) / (len * len);
    t = (t < t0) ? t0 : ((t > t1) ? t1 : t);
    best = sqrt(((ax + (t * dx)) * (ax + (t * dx))) + ((ay + (t * dy)) * (ay + (t * dy))));
    return best;
}

unsigned pilot_sim_sonar_from(pilot_sim_t * p_sim, double ox, double oy, double ray_rad,
                              double * p_mm)
{
    double best = NO_HIT;
    double const noise = ((2.0 * srnd(p_sim)) - 1.0) * PILOT_SIM_SONAR_NOISE_MM;
    int i;

    for (i = 0; i < p_sim->wall_n; i++)
    {
        double const d = p_sim->walls[i].b_blind
                             ? NO_HIT
                             : wall_hit(&p_sim->walls[i], ox, oy, ray_rad, p_sim->sonar_half_deg);

        best = (d < best) ? d : best;
    }
    *p_mm = 0.0;
    if (best > PILOT_SIM_SONAR_RANGE_MM)
    {
        return PILOT_SIM_ECHO_OPEN;
    }
    if (best < PILOT_SIM_SONAR_MIN_MM)
    {
        *p_mm = best;
        return PILOT_SIM_ECHO_TOO_CLOSE;
    }
    if ((p_sim->dropout > 0.0) && (srnd(p_sim) < p_sim->dropout))
    {
        return PILOT_SIM_ECHO_OPEN;
    }
    if (p_sim->b_thin)
    {
        if (p_sim->thin_valid <= 0)
        {
            return PILOT_SIM_ECHO_OPEN;
        }
        p_sim->thin_valid--;
    }
    best += noise;
    *p_mm = (best < PILOT_SIM_SONAR_MIN_MM) ? PILOT_SIM_SONAR_MIN_MM : best;
    return PILOT_SIM_ECHO_VALID;
}

unsigned pilot_sim_sonar(pilot_sim_t * p_sim, double servo_rad, double * p_mm)
{
    return pilot_sim_sonar_from(p_sim, p_sim->x + (PILOT_SIM_SONAR_OFFSET_MM * cos(p_sim->psi)),
                                p_sim->y + (PILOT_SIM_SONAR_OFFSET_MM * sin(p_sim->psi)),
                                p_sim->psi + servo_rad, p_mm);
}

void pilot_sim_init(pilot_sim_t * p_sim, pilot_sim_seg_t const * p_segs,
                    uint32_t seed)
{
    (void)memset(p_sim, 0, sizeof(*p_sim));
    p_sim->rng = seed;
    pilot_sim_world_reset(p_sim, seed);
    line_build(p_sim, p_segs);
    grid_build(p_sim);
    p_sim->x = START_X_MM + 100.0;
    p_sim->y = (rnd(p_sim) * 2.0 - 1.0) * START_OFF_MM;
    p_sim->psi = (rnd(p_sim) * 2.0 - 1.0) * START_YAW_DEG * PI_RAD / 180.0;
    pilot_init(&p_sim->pilot);
    drive_init(&p_sim->drive);
    p_sim->task.echo = (uint8_t)PILOT_SIM_ECHO_OPEN;
}

static void sense(pilot_sim_t * p_sim)
{
    int i;

    for (i = 0; i < 2; i++)
    {
        double const side = (0 == i) ? 1.0 : -1.0;
        double const sx = p_sim->x + (LINE_SENSOR_LEAD_MM * cos(p_sim->psi)) -
                          (side * (LINE_SENSOR_SPACING_MM / 2.0) * sin(p_sim->psi));
        double const sy = p_sim->y + (LINE_SENSOR_LEAD_MM * sin(p_sim->psi)) +
                          (side * (LINE_SENSOR_SPACING_MM / 2.0) * cos(p_sim->psi));
        double const d = line_dist(p_sim, sx, sy);
        bool raw = (d <= (LINE_WIDTH_MM / 2.0));

        if ((fabs(d - (LINE_WIDTH_MM / 2.0)) < EDGE_ZONE_MM) && (rnd(p_sim) < CHATTER))
        {
            raw = !raw;
        }
        if (raw == p_sim->deb_level[i])
        {
            p_sim->deb_count[i] = 0;
        }
        else if (++p_sim->deb_count[i] >= DEBOUNCE)
        {
            p_sim->deb_level[i] = raw;
            p_sim->deb_count[i] = 0;
        }
    }
}

/* The sonar task, once per ms. */
static void sonar_task(pilot_sim_t * p_sim)
{
    pilot_sim_sonar_task_t * const p_t = &p_sim->task;
    pilot_cmd_t const * const p_cmd = &p_sim->cmd;
    bool const b_req = (0U != p_cmd->b_ping_req) && !p_sim->b_mute &&
                       (!p_t->b_answered || (p_t->answered_seq != p_cmd->ping_seq));
    int const want = (0U != p_cmd->b_ping_req) ? (int)p_cmd->ping_servo_deci : 0;

    if (want != p_t->servo_deci)
    {
        double const deg = fabs((double)(want - p_t->servo_deci)) / 10.0;

        p_t->servo_deci = want;
        p_t->settled_s = p_sim->t_s + ((PILOT_SIM_SERVO_BASE_MS +
                                        (PILOT_SIM_SERVO_MS_PER_DEG * deg)) / 1000.0);
    }
    if ((p_sim->t_s >= p_t->settled_s) && (p_sim->t_s >= p_t->ready_s) &&
        (b_req || (0U == p_cmd->b_ping_req)))
    {
        double mm = 0.0;

        p_t->echo = (uint8_t)pilot_sim_sonar(p_sim, ((double)p_t->servo_deci / 10.0) * PI_RAD / 180.0,
                                             &mm);
        p_t->mm = (uint16_t)(mm + 0.5);
        p_t->servo_pub_deci = (int16_t)p_t->servo_deci;
        p_t->b_reply = b_req;
        if (b_req)
        {
            p_t->seq = p_cmd->ping_seq;
            p_t->b_answered = true;
            p_t->answered_seq = p_cmd->ping_seq;
        }
        else
        {
            p_t->free_seq = (uint8_t)((p_t->free_seq + 1U) & 0x3FU);
            p_t->seq = p_t->free_seq;
        }
        p_t->ready_s = p_sim->t_s + (PILOT_SIM_PING_SPACING_MS / 1000.0);
    }
}

/* Tick of the pilot: fresh measurements unless the stale window is open. */
static void pilot_tick(pilot_sim_t * p_sim)
{
    uint32_t const now_ms = (uint32_t)((p_sim->t_s * 1000.0) + 0.5) + p_sim->clock_ms0;
    bool const b_stale = (p_sim->stale_len_s > 0.0) && (p_sim->t_s >= p_sim->stale_from_s) &&
                         (p_sim->t_s < (p_sim->stale_from_s + p_sim->stale_len_s));
    int32_t left_um_s = 0;
    int32_t right_um_s = 0;
    pilot_sim_log_t * p_log;

    p_sim->meas.now_ms = now_ms;
    if (!b_stale)
    {
        p_sim->meas.stamp_ms = now_ms;
        p_sim->meas.left_um = (int64_t)(p_sim->b_dead[0] ? p_sim->dead_um[0] : p_sim->disp_l_um);
        p_sim->meas.right_um = (int64_t)(p_sim->b_dead[1] ? p_sim->dead_um[1] : p_sim->disp_r_um);
        p_sim->meas.b_line_left = p_sim->deb_level[0];
        p_sim->meas.b_line_right = p_sim->deb_level[1];
        p_sim->meas.b_ping_reply = p_sim->task.b_reply;
        p_sim->meas.b_sonar_fault = p_sim->b_sonar_fault;
        p_sim->meas.ping_mm = p_sim->task.mm;
        p_sim->meas.ping_servo_deci = p_sim->task.servo_pub_deci;
        p_sim->meas.ping_echo = p_sim->task.echo;
        p_sim->meas.ping_seq = p_sim->task.seq;
    }
    pilot_step(&p_sim->pilot, &p_sim->meas, &p_sim->cmd);

    p_log = &p_sim->history[p_sim->history_next];
    p_log->t_s = p_sim->t_s;
    p_log->now_ms = now_ms;
    p_log->cmd = p_sim->cmd;
    p_sim->history_next = (p_sim->history_next + 1U) % PILOT_SIM_HISTORY;
    if (p_sim->history_count < PILOT_SIM_HISTORY)
    {
        p_sim->history_count++;
    }

    /* The line task sends the command through the same drive limits. */
    pilot_drive(&p_sim->drive, &p_sim->cmd, now_ms - p_sim->last_ms, &left_um_s, &right_um_s);
    p_sim->last_ms = now_ms;
    p_sim->tgt_l = (double)left_um_s / 1000.0;
    p_sim->tgt_r = (double)right_um_s / 1000.0;
}

static double motor(double v, double target)
{
    double eff = target;

    if (fabs(target) < DEADBAND_MM_S)
    {
        eff = 0.0;
    }
    else if ((fabs(v) < AT_REST_MM_S) && (fabs(target) < STICTION_MM_S))
    {
        eff = 0.0;
    }
    else
    {
        /* The wheel is driven. */
    }
    v += (eff - v) * (DT_S / ((fabs(eff) > fabs(v)) ? LAG_UP_S : LAG_DOWN_S));
    if ((0.0 == eff) && (fabs(v) < 1.0))
    {
        v = 0.0;
    }
    return v;
}

static double wall_dist_at(pilot_sim_t const * p_sim, double px, double py)
{
    double best = 1e9;
    int i;

    for (i = 0; i < p_sim->wall_n; i++)
    {
        pilot_sim_wall_t const * const p_w = &p_sim->walls[i];
        double const dx = p_w->bx - p_w->ax, dy = p_w->by - p_w->ay;
        double const len2 = (dx * dx) + (dy * dy);
        double t = (len2 > 0.0) ? ((((px - p_w->ax) * dx) + ((py - p_w->ay) * dy)) / len2) : 0.0;
        double ex, ey, d;

        t = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
        ex = p_w->ax + (t * dx) - px;
        ey = p_w->ay + (t * dy) - py;
        d = sqrt((ex * ex) + (ey * ey));
        best = (d < best) ? d : best;
    }
    return best;
}

double pilot_sim_wall_dist_mm(pilot_sim_t const * p_sim)
{
    return wall_dist_at(p_sim, p_sim->x, p_sim->y);
}

void pilot_sim_kill_encoder(pilot_sim_t * p_sim, unsigned wheel)
{
    p_sim->b_dead[wheel] = true;
    p_sim->dead_um[wheel] = (0U == wheel) ? p_sim->disp_l_um : p_sim->disp_r_um;
}

void pilot_sim_fix_encoders(pilot_sim_t * p_sim)
{
    p_sim->b_dead[0] = false;
    p_sim->b_dead[1] = false;
}

void pilot_sim_step(pilot_sim_t * p_sim)
{
    long const ms = lround(p_sim->t_s * 1000.0);
    double v, yaw, nx, ny, d_new;

    if (0 == (ms % SAMPLE_MS))
    {
        sense(p_sim);
    }
    sonar_task(p_sim);
    if (0 == (ms % TICK_MS))
    {
        pilot_tick(p_sim);
    }
    p_sim->vl = motor(p_sim->vl, p_sim->tgt_l);
    p_sim->vr = motor(p_sim->vr, p_sim->tgt_r);
    if ((p_sim->block_len_s > 0.0) && (p_sim->t_s >= p_sim->block_from_s) &&
        (p_sim->t_s < (p_sim->block_from_s + p_sim->block_len_s)))
    {
        p_sim->vl = (p_sim->vl > 0.0) ? 0.0 : p_sim->vl;
        p_sim->vr = (p_sim->vr > 0.0) ? 0.0 : p_sim->vr;
    }
    v = (p_sim->vl + p_sim->vr) / 2.0;
    yaw = (p_sim->vr - p_sim->vl) / CHASSIS_WHEEL_TRACK_MM;
    if (p_sim->vl * p_sim->vr < 0.0)
    {
        yaw *= SCRUB_YAW;
    }
    nx = p_sim->x + (v * cos(p_sim->psi) * DT_S);
    ny = p_sim->y + (v * sin(p_sim->psi) * DT_S);
    d_new = wall_dist_at(p_sim, nx, ny);
    if ((d_new < PILOT_SIM_CAR_RADIUS_MM) && (d_new < wall_dist_at(p_sim, p_sim->x, p_sim->y)))
    {
        p_sim->b_contact = true;
        p_sim->contacts++;
        nx = p_sim->x;
        ny = p_sim->y;
        if (!p_sim->b_slip)
        {
            p_sim->vl = 0.0;
            p_sim->vr = 0.0;
            yaw = 0.0;
        }
    }
    p_sim->x = nx;
    p_sim->y = ny;
    p_sim->psi += yaw * DT_S;
    p_sim->disp_l_um += p_sim->vl * 1000.0 * DT_S;
    p_sim->disp_r_um += p_sim->vr * 1000.0 * DT_S;
    p_sim->t_s = (double)(ms + 1) * DT_S;
}

double pilot_sim_off_mm(pilot_sim_t const * p_sim)
{
    return line_dist(p_sim, p_sim->x + (LINE_SENSOR_LEAD_MM * cos(p_sim->psi)),
                     p_sim->y + (LINE_SENSOR_LEAD_MM * sin(p_sim->psi)));
}

double pilot_sim_speed_mm_s(pilot_sim_t const * p_sim)
{
    return (p_sim->vl + p_sim->vr) / 2.0;
}

void pilot_sim_dump(pilot_sim_t const * p_sim, uint32_t seed, FILE * p_out)
{
    unsigned i;
    unsigned idx = (p_sim->history_next + PILOT_SIM_HISTORY - p_sim->history_count) %
                   PILOT_SIM_HISTORY;

    (void)fprintf(p_out, "  seed %u, sim time %.3f s, x %.0f y %.0f, vl %.0f vr %.0f mm/s\n",
                  (unsigned)seed, p_sim->t_s, p_sim->x, p_sim->y, p_sim->vl, p_sim->vr);
    for (i = 0U; i < p_sim->history_count; i++)
    {
        pilot_sim_log_t const * p_log = &p_sim->history[idx];

        (void)fprintf(p_out, "  t %.3f s: behaviour %u reason %u speed %d mm/s turn %d ddeg/s\n",
                      p_log->t_s, (unsigned)p_log->cmd.behaviour, (unsigned)p_log->cmd.reason,
                      (int)p_log->cmd.speed_mm_s, (int)p_log->cmd.turn_deci_s);
        idx = (idx + 1U) % PILOT_SIM_HISTORY;
    }
}
