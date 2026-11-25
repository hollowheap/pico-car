/* Closed-loop host simulation of the car for the pilot: a differential-drive
 * car with a motor model per wheel, two line sensors, and a line built from
 * segments. The pilot runs unchanged; the simulation feeds it measurements
 * and turns its command back into wheel targets. Floating point is allowed
 * here. */

#ifndef PILOT_SIM_H
#define PILOT_SIM_H

#include <stdint.h>
#include <stdio.h>

#include "pilot/pilot.h"

#define PILOT_SIM_MAX_PTS   (7000)
#define PILOT_SIM_HISTORY   (8U)

/* One piece of the line: a straight (radius 0) or an arc. turn_deg is the arc
 * angle, positive to the left. A zero entry ends the list. */
typedef struct
{
    double len_mm;
    double radius;
    double turn_deg;
} pilot_sim_seg_t;

typedef struct
{
    double   t_s;
    uint32_t now_ms;
    pilot_cmd_t cmd;
} pilot_sim_log_t;

typedef struct
{
    /* Line, as points every 2 mm. */
    double   line_x[PILOT_SIM_MAX_PTS];
    double   line_y[PILOT_SIM_MAX_PTS];
    int      line_n;

    /* Car: axle midpoint, heading, wheel speeds in mm/s, wheel travel in um. */
    double   x, y, psi;
    double   vl, vr;
    double   disp_l_um, disp_r_um;
    double   tgt_l, tgt_r;

    /* Time and the pilot. */
    double   t_s;
    pilot_state_t pilot;
    pilot_meas_t  meas;
    pilot_cmd_t   cmd;
    bool     b_sensor_l, b_sensor_r;
    int      deb_count[2];
    bool     deb_level[2];
    uint32_t rng;

    /* Window in which no fresh measurement reaches the pilot. */
    double   stale_from_s, stale_len_s;

    pilot_sim_log_t history[PILOT_SIM_HISTORY];
    unsigned history_next;
    unsigned history_count;
} pilot_sim_t;

/* Lay out the line from (-200, 0) along +x and put the car on it, a few mm
 * off and a few degrees askew, both drawn from the seed. */
void pilot_sim_init(pilot_sim_t * p_sim, pilot_sim_seg_t const * p_segs,
                    uint32_t seed);

/* Advance 1 ms. The pilot runs every 10 ms. */
void pilot_sim_step(pilot_sim_t * p_sim);

/* Distance from the sensor midpoint to the line, mm. */
double pilot_sim_off_mm(pilot_sim_t const * p_sim);

/* Mean axle speed right now, mm/s. */
double pilot_sim_speed_mm_s(pilot_sim_t const * p_sim);

/* Print the seed context and the last pilot decisions. */
void pilot_sim_dump(pilot_sim_t const * p_sim, uint32_t seed, FILE * p_out);

#endif /* PILOT_SIM_H */
