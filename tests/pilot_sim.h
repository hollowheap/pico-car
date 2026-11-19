/* Closed-loop host simulation of the car for the pilot: a differential-drive
 * car with a motor model per wheel, two line sensors, and a line built from
 * segments. The pilot runs unchanged; the simulation feeds it measurements
 * and turns its command back into wheel targets. Floating point is allowed
 * here. */

#ifndef PILOT_SIM_H
#define PILOT_SIM_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pilot/pilot.h"

#define PILOT_SIM_MAX_PTS   (7000)
#define PILOT_SIM_GRID_CELLS (16384)
#define PILOT_SIM_HISTORY   (8U)
#define PILOT_SIM_MAX_WALLS (64)

/* Echo classes, equal to OBSTACLE_ECHO_IDX_* in navigation/obstacle.h (that
 * header needs the kernel headers, which this target does not see). */
#define PILOT_SIM_ECHO_VALID      (0U)
#define PILOT_SIM_ECHO_OPEN       (1U)
#define PILOT_SIM_ECHO_TOO_CLOSE  (2U)

#define PILOT_SIM_SONAR_OFFSET_MM  (70.0)  /* ahead of the wheel axle        */
#define PILOT_SIM_SONAR_HALF_DEG   (9.5)   /* beam half-angle                */
#define PILOT_SIM_SONAR_MAX_DEG    (30.0)  /* largest incidence that echoes  */
#define PILOT_SIM_SONAR_RANGE_MM   (4000.0)
#define PILOT_SIM_SONAR_MIN_MM     (20.0)
#define PILOT_SIM_SONAR_NOISE_MM   (3.0)   /* bound of the distance error    */

/* Sonar task, as the firmware is meant to run it: one request at a time, the
 * servo moves and settles (a guess: 3 ms per degree plus 20 ms), then a ping
 * at least 60 ms after the previous one. With no request it pings straight
 * ahead. */
#define PILOT_SIM_SERVO_MS_PER_DEG (3.0)
#define PILOT_SIM_SERVO_BASE_MS    (20.0)
#define PILOT_SIM_PING_SPACING_MS  (60.0)

/* Body of the car: a circle about the axle midpoint. */
#define PILOT_SIM_CAR_RADIUS_MM    (119.0)

/* A wall: one straight face between two world points, mm. */
typedef struct
{
    double ax, ay, bx, by;
    bool   b_blind;   /* the sonar never returns this face; the body still touches it */
} pilot_sim_wall_t;

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
    int      servo_deci;     /* where the servo is, or is moving to          */
    double   settled_s;      /* time the servo has settled                   */
    double   ready_s;        /* time of the next ping at the earliest        */
    bool     b_answered;     /* a request has been answered                  */
    uint8_t  answered_seq;
    uint8_t  free_seq;       /* sequence of the free-running pings           */
    bool     b_reply;        /* the published ping is a reply                */
    uint8_t  echo;           /* the published ping                           */
    uint16_t mm;
    int16_t  servo_pub_deci;
    uint8_t  seq;
} pilot_sim_sonar_task_t;

typedef struct
{
    /* Line, as points every 2 mm. */
    double   line_x[PILOT_SIM_MAX_PTS];
    double   line_y[PILOT_SIM_MAX_PTS];
    int      line_n;

    /* Uniform grid over the line points, for the nearest-point search. Each
     * cell lists the points inside it: grid_pt[grid_start[c] .. grid_start[c + 1]). */
    double   grid_x0, grid_y0, grid_cell;
    int      grid_nx, grid_ny;
    uint16_t grid_start[PILOT_SIM_GRID_CELLS + 1];
    uint16_t grid_pt[PILOT_SIM_MAX_PTS];

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
    drive_state_t drive;          /* the line task's slew state */
    uint32_t      last_ms;        /* the pass before, for the drive step */
    bool     b_sensor_l, b_sensor_r;
    int      deb_count[2];
    bool     deb_level[2];
    uint32_t rng;

    /* Obstacles and the sonar noise source (kept apart from rng so the line
     * runs do not change). */
    pilot_sim_wall_t walls[PILOT_SIM_MAX_WALLS];
    int      wall_n;
    uint32_t sonar_rng;
    pilot_sim_sonar_task_t task;
    bool     b_sonar_fault;
    bool     b_mute;         /* the sonar task does not answer requests      */
    double   sonar_half_deg; /* beam half-angle; init sets PILOT_SIM_SONAR_HALF_DEG */
    double   dropout;        /* chance that a VALID ray reads OPEN, 0 to 1   */
    bool     b_thin;         /* only thin_valid more rays may read VALID     */
    int      thin_valid;
    uint32_t clock_ms0;      /* added to the clock the pilot sees (may wrap) */

    /* Window in which no fresh measurement reaches the pilot. */
    double   stale_from_s, stale_len_s;

    /* Window in which an obstacle ahead blocks both wheels from going forward. */
    double   block_from_s, block_len_s;

    /* Contact: a move that would put the body circle across a face is not made.
     * The motor stalls (both wheels stop, the encoders show no travel); with
     * b_slip the wheels spin instead and the encoders count while the car stands. */
    bool     b_slip;
    bool     b_contact;      /* the body has touched a face at least once    */
    unsigned contacts;       /* ms in which a move into a face was refused   */

    /* Encoder reading frozen at a value while the wheel still turns. */
    bool     b_dead[2];
    double   dead_um[2];

    pilot_sim_log_t history[PILOT_SIM_HISTORY];
    unsigned history_next;
    unsigned history_count;
} pilot_sim_t;

/* Lay out the line from (-200, 0) along +x and put the car on it, a few mm
 * off and a few degrees askew, both drawn from the seed. */
void pilot_sim_init(pilot_sim_t * p_sim, pilot_sim_seg_t const * p_segs,
                    uint32_t seed);

/* Index the line again. Call it after the points of the line are changed by
 * hand; the distance to the line reads the index. */
void pilot_sim_reindex(pilot_sim_t * p_sim);

/* Remove all walls and reseed the sonar noise. pilot_sim_init does this. */
void pilot_sim_world_reset(pilot_sim_t * p_sim, uint32_t seed);

/* Add a wall from (ax, ay) to (bx, by). False when the table is full. */
bool pilot_sim_add_wall(pilot_sim_t * p_sim, double ax, double ay, double bx, double by);

/* Add a wall the sonar cannot see (a low face). */
bool pilot_sim_add_blind_wall(pilot_sim_t * p_sim, double ax, double ay, double bx, double by);

/* Add a box: centre, width (along its local y), depth (along its local x) and
 * rotation in radians. Four walls. */
bool pilot_sim_add_box(pilot_sim_t * p_sim, double cx, double cy, double width_mm,
                       double depth_mm, double rot_rad);

/* Add a closed room of four walls with the given corners. */
bool pilot_sim_add_room(pilot_sim_t * p_sim, double x0, double y0, double x1, double y1);

/* Sonar from an explicit origin along a world ray angle (radians). Returns a
 * PILOT_SIM_ECHO_* class and the distance in mm in *p_mm (0 unless VALID or
 * TOO_CLOSE). */
unsigned pilot_sim_sonar_from(pilot_sim_t * p_sim, double ox, double oy, double ray_rad,
                              double * p_mm);

/* Sonar on the car: origin 70 mm ahead of the axle, ray at body heading plus
 * the servo angle (radians, positive to the left). */
unsigned pilot_sim_sonar(pilot_sim_t * p_sim, double servo_rad, double * p_mm);

/* The encoder of one wheel (0 left, 1 right) stops counting: the pilot sees
 * its travel stay where it is now while the wheel goes on turning. */
void pilot_sim_kill_encoder(pilot_sim_t * p_sim, unsigned wheel);

/* The encoders count again, from the true travel. */
void pilot_sim_fix_encoders(pilot_sim_t * p_sim);

/* Advance 1 ms. The pilot runs every 10 ms. */
void pilot_sim_step(pilot_sim_t * p_sim);

/* Least distance from the axle midpoint to any wall, mm (1e9 without walls).
 * The body is a circle of PILOT_SIM_CAR_RADIUS_MM about the axle midpoint. */
double pilot_sim_wall_dist_mm(pilot_sim_t const * p_sim);

/* Distance from the sensor midpoint to the line, mm. */
double pilot_sim_off_mm(pilot_sim_t const * p_sim);

/* Mean axle speed right now, mm/s. */
double pilot_sim_speed_mm_s(pilot_sim_t const * p_sim);

/* Print the seed context and the last pilot decisions. */
void pilot_sim_dump(pilot_sim_t const * p_sim, uint32_t seed, FILE * p_out);

#endif /* PILOT_SIM_H */
