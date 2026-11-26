#ifndef PILOT_BYPASS_IMPLEMENTATION_H
#define PILOT_BYPASS_IMPLEMENTATION_H

#include "../hug_internal.h"

/* Sonar shaft ahead of the axle, from chassis.h. */
#define SHAFT_MM        ((int32_t)CHASSIS_SONAR_FORWARD_MM)

/* Owner's sonar PCB plane is 5 mm ahead of its shaft. The acoustic face
 * offset is unmeasured, so use the board plane as the near origin. */
#define SONAR_BOARD_MM  (5)
#define HALF_RIGHT_MM   ((int32_t)CHASSIS_RIGHT_REACH_MM)
#define HALF_LEFT_MM    ((int32_t)CHASSIS_LEFT_REACH_MM)
#define SWING_MM        (172)
#define SPIN_GAP_MM     (20)
/* Reserve the entire optical sweep while the axle turns at a lateral station. */
#define READER_RADIUS_MM (LINE_SENSOR_LEAD_MM + 1)
#define GUIDED_LATERAL_MM (HUG_PARALLEL_SPACING_MM - (LINE_WIDTH_MM + 1) / 2 - READER_RADIUS_MM - SPIN_GAP_MM)

/* Use the shaft as the range origin (behind the projected PCB face).
 * Reserve the part of the car ahead of it plus the stopped clearance. */
#define FRONT_RESERVE_MM (CHASSIS_FRONT_TIP_MM - CHASSIS_SONAR_FORWARD_MM + SPIN_GAP_MM)
#define MAP_LIMIT_DECI  (3600)
#define RIGHT_ANGLE     (900)
#define FULL_PAN        (600)
#define RAY_STEP        (50)
#define SPEED_MM_S      (140)
#define SLOW_MM_S       (70)
#define SPIN_SLOW_DECI_S ((2 * (SPEED_MM_S / 3) * DECI_PER_RAD) / CHASSIS_WHEEL_TRACK_MM)
#define AIM_TOL         (15)
#define MIN_MOVE_MM     (30)
#define TRACK_STEP      (100)
#define TRAVEL_LIMIT_MM (1500)
#define LEFT_BLOCKED    (1U)
#define RIGHT_BLOCKED   (2U)
#define HALF_DIVISOR    (2)
#define LOOK_VIEWS      (2U)
#define SIDE_AIM_DECI (RIGHT_ANGLE / 2)

/* bypass.c */
bool bypass_moved_usefully(hug_in_t const * p_in, uint32_t left_um, uint32_t right_um);
void bypass_enter(hug_t * p_hug, bypass_phase_t phase, int32_t aim, int32_t distance);
bool bypass_turn_within(hug_t const * p_hug, int32_t aim, int32_t tol, hug_out_t * p_out);
bool bypass_is_point(hug_in_t const * p_in);
void bypass_end_route(hug_t * p_hug, hug_out_t * p_out, bool b_rejoined);
bool bypass_moved(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out, int32_t speed);

/* geometry.c */
void bypass_reset_contact_fan(bypass_t * p_run);
bool bypass_fan_pose_moved_usefully(hug_t const * p_hug, hug_in_t const * p_in);
int32_t bypass_axis_mm(hug_t const * p_hug, int32_t angle);
bool bypass_ray(hug_t * p_hug, hug_in_t const * p_in, int32_t aim, int32_t gaze,
                hug_out_t * p_out);
void bypass_sweep(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out);
void bypass_look_back(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out);
int32_t bypass_side_reach(hug_t const * p_hug, int32_t aim);
int32_t bypass_side_aim(hug_t const * p_hug);
void bypass_start_side_stations(hug_t * p_hug, hug_out_t * p_out);
void bypass_side_fan(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out);

/* rejoin.c */
void bypass_lane_ray(hug_t * p_hug, hug_in_t const * p_in, int32_t base, bool b_track);
void bypass_move_on(hug_t * p_hug, bool b_sweep);
void bypass_track(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out);
void bypass_to_line(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out);
void bypass_map_line(hug_t * p_hug, hug_in_t const * p_in, hug_out_t * p_out);
void bypass_exit_ahead(hug_t * p_hug, look_t * p_look, hug_in_t const * p_in, hug_out_t * p_out);

#endif
