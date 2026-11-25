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

static double line_dist(pilot_sim_t const * p_sim, double px, double py)
{
    double best = 1e18;
    int i;

    for (i = 0; i < p_sim->line_n; i++)
    {
        double const dx = px - p_sim->line_x[i];
        double const dy = py - p_sim->line_y[i];
        double const d2 = (dx * dx) + (dy * dy);

        if (d2 < best)
        {
            best = d2;
        }
    }
    return sqrt(best);
}

void pilot_sim_init(pilot_sim_t * p_sim, pilot_sim_seg_t const * p_segs,
                    uint32_t seed)
{
    (void)memset(p_sim, 0, sizeof(*p_sim));
    p_sim->rng = seed;
    line_build(p_sim, p_segs);
    p_sim->x = START_X_MM + 100.0;
    p_sim->y = (rnd(p_sim) * 2.0 - 1.0) * START_OFF_MM;
    p_sim->psi = (rnd(p_sim) * 2.0 - 1.0) * START_YAW_DEG * PI_RAD / 180.0;
    pilot_init(&p_sim->pilot);
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

/* Tick of the pilot: fresh measurements unless the stale window is open. */
static void pilot_tick(pilot_sim_t * p_sim)
{
    uint32_t const now_ms = (uint32_t)((p_sim->t_s * 1000.0) + 0.5);
    bool const b_stale = (p_sim->stale_len_s > 0.0) && (p_sim->t_s >= p_sim->stale_from_s) &&
                         (p_sim->t_s < (p_sim->stale_from_s + p_sim->stale_len_s));
    double const track = LINE_WHEEL_TRACK_MM;
    double v, w;
    pilot_sim_log_t * p_log;

    p_sim->meas.now_ms = now_ms;
    if (!b_stale)
    {
        p_sim->meas.stamp_ms = now_ms;
        p_sim->meas.left_um = (int64_t)p_sim->disp_l_um;
        p_sim->meas.right_um = (int64_t)p_sim->disp_r_um;
        p_sim->meas.b_line_left = p_sim->deb_level[0];
        p_sim->meas.b_line_right = p_sim->deb_level[1];
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

    /* Inverse of the pilot's conversion. */
    v = p_sim->cmd.speed_mm_s;
    w = (p_sim->cmd.turn_deci_s / 573.0) * track;
    p_sim->tgt_l = v - (w / 2.0);
    p_sim->tgt_r = v + (w / 2.0);
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

void pilot_sim_step(pilot_sim_t * p_sim)
{
    long const ms = lround(p_sim->t_s * 1000.0);
    double v, yaw;

    if (0 == (ms % SAMPLE_MS))
    {
        sense(p_sim);
    }
    if (0 == (ms % TICK_MS))
    {
        pilot_tick(p_sim);
    }
    p_sim->vl = motor(p_sim->vl, p_sim->tgt_l);
    p_sim->vr = motor(p_sim->vr, p_sim->tgt_r);
    v = (p_sim->vl + p_sim->vr) / 2.0;
    yaw = (p_sim->vr - p_sim->vl) / LINE_WHEEL_TRACK_MM;
    if (p_sim->vl * p_sim->vr < 0.0)
    {
        yaw *= SCRUB_YAW;
    }
    p_sim->x += v * cos(p_sim->psi) * DT_S;
    p_sim->y += v * sin(p_sim->psi) * DT_S;
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
