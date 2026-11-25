/** Feed measured roll and encoder feedback through the firmware line task. */
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "barcode/action.h"
#include "line_filter.h"
#include "motion/motion.h"
#include "navigation/obstacle.h"
#include "navigation/obstacle_ping.h"
#include "physics/physics.h"
#include "pilot/drive.h"
#include "platform/faults.h"
#include "platform/chassis.h"
#include "platform/liveness.h"
#include "platform/log.h"
#include "supervisor/command.h"
#include "telemetry/assemble.h"
#include "telemetry/emit.h"
#include "telemetry/queue.h"
#include "telemetry/wire_schema.h"
#include "track/line_control.h"
#include "track/line_task.h"
#include "track/track.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

#define PI_RAD 3.14159265358979323846
#define TICK_MS 10U
#define LAST_TICK 60U
#define TURN_TICK 12U
#define SONAR_FAULT_TICK 16U
#define TURN_OBSTACLE_TICK 18U
#define RETRY_TICK_LIMIT 10000U
#define RETRY_EVENT_FLAG 0x8000U
#define TERMINAL_EXTRA_TICKS 20U
#define COURSE_OBSTACLE_TICK 12U
#define COURSE_OBSTACLE_RANGE_MM 100U
#define COURSE_NEAR_CLEAR_MM 101U
#define COURSE_HOLD_TICK 18U
#define COURSE_SINGLE_OPEN_TICK 60U
#define COURSE_UNKNOWN_TICK 110U
#define COURSE_CLEAR_TICK 162U
#define COURSE_CALIBRATION_TICK 160U
#define COURSE_CALIBRATED_TICK 186U
#define COURSE_LAST_TICK 320U
#define COURSE_LOOK_REPLIES 3U
#define COURSE_MAX_LOOK_GAP_TICKS (1000U / TICK_MS)
#define COURSE_SLOW_CLEAR_PERIOD_TICKS 18U
#define COURSE_FIRST_CLEAR_TICK 180U
#define COURSE_OFF_AXIS_TICK 192U
#define COURSE_REQUESTED_TICK 198U
#define COURSE_BAD_ECHO_TICK 204U
#define COURSE_FRESH_CLEAR_TICK 210U
#define COURSE_REPEATED_CLEAR_TICK 216U
#define COURSE_RESUME_TICK 228U
#define COURSE_PING_PERIOD_TICKS 6U
#define MOTION_LOST_TICK 12U
#define MOTION_RESTORED_TICK 18U
#define MOTION_ESTOP_TICK 30U
#define IR_FROZEN_TICK 15U
#define IR_STOP_TICK 18U
#define IR_ROLLOVER_TICK 22U
#define IR_RESTORED_TICK 30U

#define JUNCTION_TURN_SHIFT 8U

#define ROUTE_SUBSTEP_S             (0.005)
#define ROUTE_MOTOR_TAU_S           (0.060)
#define ROUTE_MOTOR_FALL_TAU_S      (0.020)
#define ROUTE_TRACK_HALF_MM         (150.0)
#define ROUTE_TAPE_HALF_MM          ((double)LINE_WIDTH_MM * 0.5)
#define ROUTE_CORNER_X_MM           (400.0)
#define ROUTE_EXIT_LEN_MM           (650.0)
#define ROUTE_FINAL_WINDOW_MM       (250.0)
#define ROUTE_BARCODE_LEFT_MM       (45.0)
#define ROUTE_ROUND_BEND_X_MM       (350.0)
#define ROUTE_ROUND_RADIUS_MM       (150.0)
#define ROUTE_ROUND_EXIT_LEN_MM     (ROUTE_ROUND_RADIUS_MM + 350.0)
#define ROUTE_ROUND_BARCODE_LEFT_MM (40.0)
#define ROUTE_MAX_TICKS             (3000U)
#define ROUTE_MAX_OVERSHOOT_MM      (20.0)
#define ROUTE_MAX_EXIT_DEV_MM       (15.0)
#define ROUTE_ROUND_MAX_SENSOR_MM   ((double)LINE_SENSOR_SPACING_MM)
#define ROUTE_ROUND_MAX_EXIT_DEV_MM ((double)LINE_SENSOR_SPACING_MM)
#define ROUTE_ROUND_MIN_EXIT_SIN    (0.96)
#define ROUTE_BODY_POINTS           (5U)
#define ROUTE_THROUGH_X_MM          (ROUTE_ROUND_BEND_X_MM + ROUTE_ROUND_RADIUS_MM)
#define ROUTE_JUNCTION_Y_MM         (550.0)
#define ROUTE_U_MARKER_X_MM         (185.0)
#define ROUTE_NEXT_JUNCTION_MM      (450.0)
#define ROUTE_MOTION_FRAME_MS       (200U)
#define ROUTE_JUNCTION_MAX_TICKS    (2500U)
#define ROUTE_LATER_JUNCTION_TICKS  (4500U)
#define ROUTE_START_OFFSET_MM       (-5.5)
#define ROUTE_READER_OFFSET_MM      (30.0)
#define ROUTE_PROBE_DEG             (40.0)
#define ROUTE_LATE_PROBE_DEG        (65.0)

typedef enum
{
    ROUTE_LAYOUT_SQUARE,
    ROUTE_LAYOUT_ROUND,
    ROUTE_LAYOUT_THROUGH_BRANCH,
    ROUTE_LAYOUT_END_T,
    ROUTE_LAYOUT_OPPOSITE_BRANCH,
    ROUTE_LAYOUT_SINGLE_SIDE
} route_layout_t;

typedef enum
{
    ROUTE_SETUP_NORMAL,
    ROUTE_SETUP_RIGHT_ALTERNATIVE,
    ROUTE_SETUP_READER_BACK,
    ROUTE_SETUP_READER_FRONT,
    ROUTE_SETUP_MIRROR_START,
    ROUTE_SETUP_HOLD_BEFORE_JUNCTION,
    ROUTE_SETUP_HOLD_DURING_ALIGNMENT,
    ROUTE_SETUP_OBSTACLE_DURING_ALIGNMENT,
    ROUTE_SETUP_REPLACE_LEFT,
    ROUTE_SETUP_REPLACE_PROBE,
    ROUTE_SETUP_REPLACE_LATE_PROBE,
    ROUTE_SETUP_HOLD_ON_REPLACEMENT,
    ROUTE_SETUP_REPEAT_MANUAL_U,
    ROUTE_SETUP_INVALID_U_ORIGIN,
    ROUTE_SETUP_INVALID_U_CAPTURE
} route_setup_t;

typedef struct
{
    bool active;
    double bend_x_mm;
    double radius_mm;
    double exit_len_mm;
    double barcode_left_mm;
    bool fast_fall;
    bool ideal;
    route_layout_t layout;
    bool right_exit;
    bool straight;
    bool uturn;
    uint8_t posted;
    uint8_t slot;
    bool safety;
    bool interrupt_alignment;
    bool sonar_alignment;
    bool replace_left;
    bool hold_on_replacement;
    bool repeat_manual;
    bool invalid_origin;
    bool invalid_capture;
    bool origin_recovered;
    bool command_polled;
    double replace_probe_deg;
    double probe_origin_theta_rad;
    double replacement_theta_rad;
    unsigned replacement_tick;
    unsigned consumed;
    unsigned turns;
    unsigned done;
    unsigned aborts;
    unsigned refusals;
    int16_t event_kind;
    unsigned start_tick;
    unsigned end_tick;
    unsigned end_age_ms;
    unsigned repeat_tick;
    unsigned invalid_tick;
    unsigned blocked_tick;
    unsigned alignment_pivots;
    unsigned stopped_frames;
    bool early;
    bool early_scan;
    bool wrong_way;
    bool departed_past_end;
    bool pending_on_exit;
    double turn_start_x_mm;
    double end_x_mm;
    double end_y_mm;
    double max_axle_dev_mm;
    double x_mm;
    double y_mm;
    double theta_rad;
    double reader_dx_mm;
    double left_gain;
    double right_gain;
    double v_left_um_s;
    double v_right_um_s;
    double true_left_um;
    double true_right_um;
    int64_t enc_left_um;
    int64_t enc_right_um;
    int32_t enc_left_speed;
    int32_t enc_right_speed;
    line_debounce_t left_filter;
    line_debounce_t right_filter;
    bool left_debounced;
    bool right_debounced;
    bool barcode_raw;
    bool barcode_seen;
    uint32_t barcode_edge_us;
    unsigned barcode_edges;
    double max_overshoot_mm;
    double max_sensor_dev_mm;
    double max_track_dist_mm;
    double max_body_dist_mm;
    double max_exit_dev_mm;
    unsigned exit_samples;
    unsigned turn_events;
    bool exit_wheel_stopped;
    bool reached_exit;
} route_sim_t;

static route_sim_t g_route_sim;

static jmp_buf g_finished;
static unsigned g_tick;
static int16_t g_roll_cdeg;
static bool g_launch_glitch;
static bool g_pose_return;
static bool g_pose_obstacle;
static bool g_route_hold;
static bool g_route_pending;
static bool g_route_cancel;
static bool g_junction_pause;
static bool g_pause_right;
static bool g_side_scan;
static bool g_slow_left;
static bool g_attitude_unknown;
static bool g_invalid_speed;
static bool g_stale_last_sample;
static bool g_motion_snapshot_case;
static bool g_motion_fault_case;
static bool g_motion_approached;
static bool g_motion_resumed;
static bool g_motion_moved_without_permit;
static bool g_ir_capture_case;
static bool g_ir_approached;
static bool g_ir_moved_while_stale;
static bool g_ir_resumed;
static int32_t g_left_target;
static int32_t g_right_target;
static bool g_turn_case;
static bool g_rise_case;
static int32_t g_rise_first;
static int32_t g_rise_last;
static int32_t g_rise_max_step;
static bool g_stripe_case;
static unsigned g_stripe_reports;
static uint16_t g_stripe_bits;
static unsigned g_probe_reports;
static uint16_t g_probe_bits;
static uint8_t g_turn_opcode = VEHICLE_CMD_UTURN;
static uint8_t g_turn_source = VEHICLE_SRC_BARCODE;
static bool g_turn_obstacle;
static bool g_turn_course_moved;
static unsigned g_turn_observations;
static unsigned g_turn_look_requests;
static bool g_turning;
static bool g_began_turn;
static bool g_moved_after_fault;
static unsigned g_turn_abort_tick;
static bool g_retry_case;
static int64_t g_left_travel;
static int64_t g_right_travel;
static unsigned g_line_loss_events;
static unsigned g_retry_events;
static unsigned g_terminal_tick;
static bool g_course_case;
static bool g_course_approach;
static bool g_course_hold_moved;
static bool g_course_resumed;
static bool g_course_finite_clear;
static bool g_course_box_removed;
static bool g_course_invalid_rest;
static int g_course_probe_side;
static bool g_course_slow_clear;
static bool g_course_cutoff;
static bool g_course_missed_ping;
static bool g_course_near_approach;
static unsigned g_course_brake_tick;
static unsigned g_course_resume_tick;
static unsigned g_course_contact_tick;
static int16_t g_course_contact_mm;
static int64_t g_course_contact_um;
static unsigned g_course_look_requests;
static unsigned g_course_contacts;
static unsigned g_course_scan_completions;
static unsigned g_course_profiles;
static unsigned g_course_reply_tick;
static unsigned g_course_last_look_tick;
static unsigned g_hug_locks;
static uint8_t g_course_reply_seq;
static int16_t g_course_reply_servo;
static bool g_start_check;
static bool g_start_airborne;
static bool g_start_bad_imu;
static bool g_start_late_imu;
static bool g_start_no_wheel;
static bool g_start_noisy_baseline;
static bool g_start_no_clearance;
static bool g_start_clearance_lost;
static bool g_start_probe_seen;
static bool g_start_follow_seen;
static unsigned g_start_failures;
static int32_t g_start_speed;

ER ISpinLock(T_SPLOCK * p_lock, UINT * p_saved)
{
    g_hug_locks++;
    *p_saved = 0U;
    return SpinLock(p_lock);
}

ER tk_get_otm(SYSTIM * p_time)
{
    *p_time = (SYSTIM){0, TICK_MS * g_tick};
    return E_OK;
}

static void route_point_world(double fwd_mm, double left_mm, double * p_wx, double * p_wy)
{
    double const c = cos(g_route_sim.theta_rad);
    double const s = sin(g_route_sim.theta_rad);
    *p_wx = g_route_sim.x_mm + fwd_mm * c - left_mm * s;
    *p_wy = g_route_sim.y_mm + fwd_mm * s + left_mm * c;
}

static double route_seg_dist_mm(double px, double py, double ax, double ay, double bx, double by)
{
    double const dx = bx - ax;
    double const dy = by - ay;
    double const len2 = dx * dx + dy * dy;
    double t = (len2 > 0.0) ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return hypot(px - (ax + t * dx), py - (ay + t * dy));
}

static bool route_is_plain(void)
{
    return (ROUTE_LAYOUT_SQUARE == g_route_sim.layout) || (ROUTE_LAYOUT_ROUND == g_route_sim.layout);
}

static double route_outward_mm(double py)
{
    return g_route_sim.right_exit ? -py : py;
}

static double route_tape_dist_mm(double px, double py)
{
    double const out = route_outward_mm(py);
    if (g_route_sim.radius_mm > 0.0)
    {
        double const r = g_route_sim.radius_mm;
        double const dx = px - g_route_sim.bend_x_mm;
        double dist = fmin(hypot(fmax(dx, 0.0), out), hypot(dx - r, fmin(out - r, 0.0)));
        if ((dx >= 0.0) && (out <= r))
        {
            dist = fmin(dist, fabs(hypot(dx, out - r) - r));
        }
        return dist;
    }
    double const d_approach = route_seg_dist_mm(px, out, -200.0, 0.0, g_route_sim.bend_x_mm, 0.0);
    double const d_exit = route_seg_dist_mm(px, out, g_route_sim.bend_x_mm, 0.0,
                                            g_route_sim.bend_x_mm, g_route_sim.exit_len_mm + 250.0);
    return (d_approach < d_exit) ? d_approach : d_exit;
}

static bool route_junction_on_tape(double px, double py)
{
    double const half = ROUTE_TAPE_HALF_MM;
    if (g_route_sim.uturn && !g_route_sim.replace_left)
    {
        return fabs(py) <= half;
    }
    if ((px <= ROUTE_ROUND_BEND_X_MM) && (fabs(py) <= half))
    {
        return true;
    }
    if ((px >= ROUTE_ROUND_BEND_X_MM) && (py <= ROUTE_ROUND_RADIUS_MM) &&
        (fabs(hypot(px - ROUTE_ROUND_BEND_X_MM, py - ROUTE_ROUND_RADIUS_MM) - ROUTE_ROUND_RADIUS_MM) <= half))
    {
        return true;
    }
    route_layout_t const layout = g_route_sim.layout;
    bool const terminating = (ROUTE_LAYOUT_END_T == layout) || (ROUTE_LAYOUT_SINGLE_SIDE == layout);
    bool const branch_right = (ROUTE_LAYOUT_OPPOSITE_BRANCH == layout) ? !g_route_sim.right_exit
                                                                       : g_route_sim.right_exit;
    bool const side_tape = (fabs(py - ROUTE_JUNCTION_Y_MM) <= half) &&
        ((ROUTE_LAYOUT_END_T == layout) ||
         (branch_right ? (px >= ROUTE_THROUGH_X_MM) : (px <= ROUTE_THROUGH_X_MM)));
    bool const next_branch = (ROUTE_LAYOUT_END_T == layout) && g_route_sim.straight &&
        (fabs(fabs(px - ROUTE_THROUGH_X_MM) - ROUTE_NEXT_JUNCTION_MM) <= half) &&
        (py >= ROUTE_JUNCTION_Y_MM);
    return side_tape || next_branch ||
        ((fabs(px - ROUTE_THROUGH_X_MM) <= half) && (py >= ROUTE_ROUND_RADIUS_MM) &&
         (!terminating || (py <= ROUTE_JUNCTION_Y_MM)));
}

static bool route_on_tape(double px, double py)
{
    if (!route_is_plain())
    {
        return route_junction_on_tape(px, py);
    }
    if (g_route_sim.radius_mm > 0.0)
    {
        return route_tape_dist_mm(px, py) <= ROUTE_TAPE_HALF_MM;
    }
    double const out = route_outward_mm(py);
    bool const on_approach = (px >= -200.0) && (px <= g_route_sim.bend_x_mm + ROUTE_TAPE_HALF_MM) &&
                             (fabs(out) <= ROUTE_TAPE_HALF_MM);
    bool const on_exit = (out >= -ROUTE_TAPE_HALF_MM) && (out <= g_route_sim.exit_len_mm + 250.0) &&
                         (fabs(px - g_route_sim.bend_x_mm) <= ROUTE_TAPE_HALF_MM);
    return on_approach || on_exit;
}

static bool route_at_destination(void)
{
    double const dx = g_route_sim.x_mm - ROUTE_THROUGH_X_MM;
    double const dy = g_route_sim.y_mm - ROUTE_JUNCTION_Y_MM;
    if (route_is_plain())
    {
        return g_route_sim.reached_exit;
    }
    if (ROUTE_LAYOUT_OPPOSITE_BRANCH == g_route_sim.layout)
    {
        return dy > 240.0;
    }
    if ((ROUTE_LAYOUT_END_T == g_route_sim.layout) && g_route_sim.straight)
    {
        return (dy > -30.0) && (fabs(dx) > (ROUTE_NEXT_JUNCTION_MM + 170.0));
    }
    return (dy > -30.0) &&
           (g_route_sim.straight ? (dy > 400.0) : (g_route_sim.right_exit ? (dx > 170.0) : (dx < -170.0)));
}

static void route_record_course(void)
{
    route_sim_t * const p_sim = &g_route_sim;
    p_sim->early_scan |= g_turning && (0 != g_left_target) && (g_left_target == -g_right_target) &&
                         (p_sim->y_mm < (ROUTE_JUNCTION_Y_MM - 45.0));
    if ((ROUTE_LAYOUT_END_T == p_sim->layout) || (ROUTE_LAYOUT_SINGLE_SIDE == p_sim->layout))
    {
        double const along_exit = fabs(p_sim->x_mm - ROUTE_THROUGH_X_MM);
        p_sim->departed_past_end |= (along_exit < 60.0) && (p_sim->y_mm > (ROUTE_JUNCTION_Y_MM + 60.0));
        if ((ROUTE_LAYOUT_END_T == p_sim->layout) && p_sim->straight && !p_sim->pending_on_exit &&
            (fabs(p_sim->y_mm - ROUTE_JUNCTION_Y_MM) < 30.0) && (along_exit > 190.0) && (along_exit < 210.0))
        {
            assert((VEHICLE_CMD_STRAIGHT == p_sim->slot) && (0U == p_sim->consumed) && (0U == p_sim->turns));
            assert(fabs(cos(p_sim->theta_rad)) > 0.9);
            p_sim->pending_on_exit = true;
        }
    }
    else if (p_sim->straight)
    {
        p_sim->wrong_way |= (p_sim->y_mm > (ROUTE_JUNCTION_Y_MM - 30.0)) &&
                            (fabs(p_sim->x_mm - ROUTE_THROUGH_X_MM) > 80.0);
    }
    else
    {
        p_sim->wrong_way |= p_sim->y_mm > (ROUTE_JUNCTION_Y_MM + 100.0);
    }
}

static void route_record_geometry(void)
{
    if (!route_is_plain())
    {
        route_record_course();
        return;
    }
    double const pts[6][2] = {
        {(double)LINE_SENSOR_LEAD_MM, (double)LINE_SENSOR_SPACING_MM * 0.5},
        {(double)LINE_SENSOR_LEAD_MM, -(double)LINE_SENSOR_SPACING_MM * 0.5},
        {0.0, (double)CHASSIS_WHEEL_TRACK_MM * 0.5},
        {0.0, -(double)CHASSIS_WHEEL_TRACK_MM * 0.5},
        {-(double)CHASSIS_AXLE_TO_CASTER_MM, 0.0},
        {(double)LINE_SENSOR_LEAD_MM + g_route_sim.reader_dx_mm, g_route_sim.barcode_left_mm},
    };
    double const exit_x = g_route_sim.bend_x_mm + g_route_sim.radius_mm;
    double const overshoot = g_route_sim.x_mm - exit_x;
    if (overshoot > g_route_sim.max_overshoot_mm)
    {
        g_route_sim.max_overshoot_mm = overshoot;
    }
    double const axle_dev = route_tape_dist_mm(g_route_sim.x_mm, g_route_sim.y_mm);
    if (axle_dev > g_route_sim.max_axle_dev_mm)
    {
        g_route_sim.max_axle_dev_mm = axle_dev;
    }
    double sx = 0.0;
    double sy = 0.0;
    route_point_world((double)LINE_SENSOR_LEAD_MM, 0.0, &sx, &sy);
    double const sensor_dev = route_tape_dist_mm(sx, sy);
    if (sensor_dev > g_route_sim.max_sensor_dev_mm)
    {
        g_route_sim.max_sensor_dev_mm = sensor_dev;
    }
    for (unsigned i = 0U; i < 6U; i++)
    {
        double wx = 0.0;
        double wy = 0.0;
        route_point_world(pts[i][0], pts[i][1], &wx, &wy);
        double const dist = route_tape_dist_mm(wx, wy);
        if ((i < ROUTE_BODY_POINTS) && (dist > g_route_sim.max_body_dist_mm))
        {
            g_route_sim.max_body_dist_mm = dist;
        }
        if (dist > g_route_sim.max_track_dist_mm)
        {
            g_route_sim.max_track_dist_mm = dist;
        }
    }
    double const out_mm = route_outward_mm(g_route_sim.y_mm);
    if (out_mm >= g_route_sim.exit_len_mm - ROUTE_FINAL_WINDOW_MM)
    {
        double const dev = fabs(g_route_sim.x_mm - exit_x);
        if (dev > g_route_sim.max_exit_dev_mm)
        {
            g_route_sim.max_exit_dev_mm = dev;
        }
        if ((g_route_sim.v_left_um_s <= 0.0) || (g_route_sim.v_right_um_s <= 0.0))
        {
            g_route_sim.exit_wheel_stopped = true;
        }
        g_route_sim.exit_samples++;
    }
    if (out_mm >= g_route_sim.exit_len_mm)
    {
        g_route_sim.reached_exit = true;
    }
}

static void route_sample_sensors(uint32_t now_us, bool b_init)
{
    double lx = 0.0, ly = 0.0, rx = 0.0, ry = 0.0, bx = 0.0, by = 0.0;
    route_point_world((double)LINE_SENSOR_LEAD_MM, (double)LINE_SENSOR_SPACING_MM * 0.5, &lx, &ly);
    route_point_world((double)LINE_SENSOR_LEAD_MM, -(double)LINE_SENSOR_SPACING_MM * 0.5, &rx, &ry);
    route_point_world((double)LINE_SENSOR_LEAD_MM + g_route_sim.reader_dx_mm, g_route_sim.barcode_left_mm, &bx, &by);
    bool const raw_l = route_on_tape(lx, ly);
    bool const raw_r = route_on_tape(rx, ry);
    bool const raw_b = route_on_tape(bx, by);
    if (b_init)
    {
        line_debounce_init(&g_route_sim.left_filter, raw_l);
        line_debounce_init(&g_route_sim.right_filter, raw_r);
        g_route_sim.left_debounced = raw_l;
        g_route_sim.right_debounced = raw_r;
        g_route_sim.barcode_raw = raw_b;
    }
    else
    {
        g_route_sim.left_debounced = line_debounce_update(&g_route_sim.left_filter, raw_l);
        g_route_sim.right_debounced = line_debounce_update(&g_route_sim.right_filter, raw_r);
        if (raw_b != g_route_sim.barcode_raw)
        {
            g_route_sim.barcode_raw = raw_b;
            g_route_sim.barcode_seen = true;
            g_route_sim.barcode_edge_us = now_us;
            g_route_sim.barcode_edges++;
        }
    }
}

static double route_wheel_step_um(double * p_speed_um_s, double requested_um_s)
{
    double const before = *p_speed_um_s;
    if (g_route_sim.ideal)
    {
        *p_speed_um_s = requested_um_s;
        return requested_um_s * ROUTE_SUBSTEP_S;
    }
    if (g_route_sim.fast_fall)
    {
        double const tau = (fabs(requested_um_s) > fabs(before)) ? ROUTE_MOTOR_TAU_S : ROUTE_MOTOR_FALL_TAU_S;
        double const change = (requested_um_s - before) * -expm1(-ROUTE_SUBSTEP_S / tau);
        *p_speed_um_s = before + change;
        return requested_um_s * ROUTE_SUBSTEP_S - change * tau;
    }
    double const alpha = 1.0 - exp(-ROUTE_SUBSTEP_S / ROUTE_MOTOR_TAU_S);
    *p_speed_um_s = before + alpha * (requested_um_s - before);
    return *p_speed_um_s * ROUTE_SUBSTEP_S;
}

static void route_step_5ms(uint32_t now_us)
{
    double const dl_um = route_wheel_step_um(&g_route_sim.v_left_um_s,
                                             g_route_sim.left_gain * (double)g_left_target);
    double const dr_um = route_wheel_step_um(&g_route_sim.v_right_um_s,
                                             g_route_sim.right_gain * (double)g_right_target);
    g_route_sim.true_left_um += dl_um;
    g_route_sim.true_right_um += dr_um;

    double const dl_mm = dl_um / (double)UM_PER_MM;
    double const dr_mm = dr_um / (double)UM_PER_MM;
    double const ds = 0.5 * (dl_mm + dr_mm);
    double const dtheta = (dr_mm - dl_mm) / (double)CHASSIS_WHEEL_TRACK_MM;
    g_route_sim.x_mm += ds * cos(g_route_sim.theta_rad + 0.5 * dtheta);
    g_route_sim.y_mm += ds * sin(g_route_sim.theta_rad + 0.5 * dtheta);
    g_route_sim.theta_rad += dtheta;

    route_sample_sensors(now_us, false);
    route_record_geometry();
}

static uint32_t ir_timer_at(unsigned tick)
{
    /* Different origin from kernel time. The timer wraps between the frozen
       sample and the stop, and later returns to the frozen low word. */
    unsigned const timer_tick = (g_start_check || g_route_sim.active || tick < IR_ROLLOVER_TICK) ? tick
        : tick - IR_ROLLOVER_TICK + IR_FROZEN_TICK;
    return UINT32_MAX - 170000U + timer_tick * TICK_MS * 1000U;
}

uint32_t shim_reg_read(uint32_t address)
{
    assert(TIMER_TIMERAWL == address);
    return ir_timer_at(g_tick);
}

int motion_feedback_snapshot(motion_feedback_t * p_feedback)
{
    int32_t const left = g_slow_left ? 100000 : 125000;
    int32_t const right = g_slow_left ? 125000 : 100000;
    bool const b_invalid_rest = g_course_invalid_rest && g_tick >= COURSE_CLEAR_TICK &&
        g_tick < COURSE_CLEAR_TICK + 2U * COURSE_PING_PERIOD_TICKS;

    *p_feedback = (motion_feedback_t){
        .left_speed_um_s = (g_retry_case || g_course_case || g_turn_obstacle) ? g_left_target : left,
        .right_speed_um_s = (g_retry_case || g_course_case || g_turn_obstacle) ? g_right_target : right,
        .left_displacement_um = (g_retry_case || g_course_case) ? g_left_travel : (int64_t)left * g_tick / 100,
        .right_displacement_um = (g_retry_case || g_course_case) ? g_right_travel : (int64_t)right * g_tick / 100,
        .speed_valid = {true, !g_invalid_speed && !b_invalid_rest}, .odometry_valid = {true, true},
        .timestamp_us = TICK_MS * 1000U * g_tick,
        .fresh = !g_stale_last_sample || (g_tick + 1U != LAST_TICK),
    };
    if (g_junction_pause)
    {
        int32_t const speed = g_tick == 58U ? 0 : 96000;
        int64_t const travel = (int64_t)(g_tick + 1U) * 960;
        int64_t const inner = travel - (g_tick >= 58U ? 960 : 0);
        p_feedback->left_speed_um_s = g_pause_right ? 96000 : speed;
        p_feedback->right_speed_um_s = g_pause_right ? speed : 96000;
        p_feedback->left_displacement_um = g_pause_right ? travel : inner;
        p_feedback->right_displacement_um = g_pause_right ? inner : travel;
    }
    if (g_launch_glitch)
    {
        bool const invalid = g_tick == 24U || g_tick == 25U;
        *p_feedback = (motion_feedback_t){
            .fresh = true, .timestamp_us = TICK_MS * 1000U * g_tick,
            .left_displacement_um = g_tick >= 24U ? 2100 : 0,
            .speed_valid = {!invalid, true}, .odometry_valid = {!invalid, true},
            .invalid_samples = {(uint8_t)(g_tick >= 25U ? 2U : g_tick == 24U), 0U},
        };
    }
    if (g_pose_return && !g_turn_case)
    {
        p_feedback->left_speed_um_s = 0;
        p_feedback->right_speed_um_s = 0;
        p_feedback->left_displacement_um = 0;
        p_feedback->right_displacement_um = 0;
    }
    if (g_start_check)
    {
        p_feedback->left_speed_um_s = g_start_speed;
        p_feedback->right_speed_um_s = g_start_speed;
        p_feedback->left_displacement_um = g_left_travel;
        p_feedback->right_displacement_um = g_right_travel;
        p_feedback->timestamp_us = ir_timer_at(g_tick);
    }
    if (g_route_sim.active)
    {
        p_feedback->left_speed_um_s = g_route_sim.enc_left_speed;
        p_feedback->right_speed_um_s = g_route_sim.enc_right_speed;
        p_feedback->left_displacement_um = g_route_sim.enc_left_um;
        p_feedback->right_displacement_um = g_route_sim.enc_right_um;
        p_feedback->speed_valid[0] = true;
        p_feedback->speed_valid[1] = true;
        p_feedback->odometry_valid[0] = true;
        p_feedback->odometry_valid[1] = true;
        p_feedback->fresh = true;
        p_feedback->timestamp_us = ir_timer_at(g_tick);
        bool const b_bad_origin = g_route_sim.invalid_origin && (TURN_TICK == g_tick) &&
                                  !g_route_sim.origin_recovered;
        bool const b_bad_capture = g_route_sim.invalid_capture && g_turning &&
            (g_route_sim.theta_rad < (-150.0 * PI_RAD / 180.0)) &&
            (g_route_sim.left_debounced || g_route_sim.right_debounced);
        if (b_bad_origin || b_bad_capture)
        {
            unsigned const wheel = b_bad_origin ? 0U : 1U;
            p_feedback->speed_valid[wheel] = false;
            p_feedback->odometry_valid[wheel] = false;
            g_route_sim.invalid_tick = g_tick;
        }
    }
    return E_OK;
}

int motion_set_speed(int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    bool const b_moving = left_um_s != 0 || right_um_s != 0;
    bool const b_was_moving = g_left_target != 0 || g_right_target != 0;

    assert(ttl_us > 0U);
    if (g_route_sim.active)
    {
        if (g_route_sim.hold_on_replacement && (0U != g_route_sim.replacement_tick))
        {
            assert(!b_moving);
        }
        if (0U != g_route_sim.blocked_tick)
        {
            assert(!b_moving);
        }
        if ((1U == g_route_sim.turns) && (0U == g_route_sim.done) && (0U == g_route_sim.aborts) &&
            (g_route_sim.straight ? ((0 != left_um_s) && (right_um_s == -left_um_s)) :
             (g_route_sim.right_exit || g_route_sim.uturn) ? ((left_um_s > 0) && (right_um_s < 0)) :
                                                              ((left_um_s < 0) && (right_um_s > 0))))
        {
            g_route_sim.alignment_pivots++;
        }
    }
    if (g_start_check && b_moving)
    {
        assert(!g_start_no_clearance && g_start_failures == 0U);
        bool const probe = left_um_s == 80000 && right_um_s == 80000;
        g_start_probe_seen |= probe;
        if (!probe)
        {
            assert(g_start_probe_seen && !g_start_airborne && !g_start_bad_imu);
            g_start_follow_seen = true;
        }
    }
    if (g_pose_return && !g_turn_case)
    {
        if (g_tick < MOTION_LOST_TICK) g_motion_approached |= b_moving;
        else if (g_tick < MOTION_RESTORED_TICK || g_pose_obstacle) assert(!b_moving);
        else if (left_um_s > 0 && right_um_s > 0) g_motion_resumed = true;
    }
    if (g_launch_glitch)
    {
        assert(left_um_s >= 0 && right_um_s >= 0);
    }
    if (g_rise_case && g_turning && b_moving)
    {
        int32_t const half = (right_um_s - left_um_s) / 2;
        int32_t const size = (half < 0) ? -half : half;
        if (0 == g_rise_first)
        {
            g_rise_first = size;
        }
        else if (size - g_rise_last > g_rise_max_step)
        {
            g_rise_max_step = size - g_rise_last;
        }
        assert(left_um_s == -right_um_s);
        g_rise_last = size;
    }
    g_left_target = left_um_s;
    g_right_target = right_um_s;
    if ((g_motion_snapshot_case || g_motion_fault_case) && (left_um_s != 0 || right_um_s != 0))
    {
        g_motion_approached = g_motion_approached || g_tick < MOTION_LOST_TICK;
        g_motion_resumed = g_motion_resumed ||
            (g_tick >= MOTION_RESTORED_TICK && g_tick < MOTION_ESTOP_TICK);
        g_motion_moved_without_permit = g_motion_moved_without_permit ||
            (g_tick >= MOTION_LOST_TICK && g_tick < MOTION_RESTORED_TICK) ||
            g_tick >= MOTION_ESTOP_TICK;
    }
    if (g_ir_capture_case && (left_um_s != 0 || right_um_s != 0))
    {
        g_ir_approached = g_ir_approached || g_tick < IR_FROZEN_TICK;
        g_ir_moved_while_stale = g_ir_moved_while_stale ||
            (g_tick >= IR_STOP_TICK && g_tick < IR_RESTORED_TICK);
        g_ir_resumed = g_ir_resumed || g_tick >= IR_RESTORED_TICK;
    }
    if (g_course_cutoff)
    {
        if (g_tick >= COURSE_PING_PERIOD_TICKS && g_tick < COURSE_OBSTACLE_TICK)
        {
            g_course_near_approach = g_course_near_approach || (left_um_s > 0 && right_um_s > 0);
        }
        if (!b_moving && b_was_moving && g_course_brake_tick == 0U)
        {
            g_course_brake_tick = g_tick;
        }
        if (b_moving && g_course_brake_tick != 0U && g_course_resume_tick == 0U)
        {
            g_course_resume_tick = g_tick;
        }
    }
    if (g_course_case && b_moving)
    {
        if (g_course_box_removed && g_tick >= COURSE_OBSTACLE_TICK && g_course_resume_tick == 0U)
        {
            g_course_resume_tick = g_tick;
        }
        g_course_approach = g_course_approach || g_tick < COURSE_OBSTACLE_TICK;
        g_course_hold_moved = g_course_hold_moved ||
            (g_tick >= COURSE_HOLD_TICK && g_tick < COURSE_CALIBRATED_TICK);
        g_course_resumed = g_course_resumed ||
            (g_tick >= COURSE_CALIBRATED_TICK && left_um_s > 0 && right_um_s > 0);
    }
    if (g_turning && (left_um_s != 0 || right_um_s != 0))
    {
        g_began_turn = true;
        g_moved_after_fault = g_moved_after_fault || (g_tick >= SONAR_FAULT_TICK);
    }
    if (g_turn_obstacle && g_tick >= TURN_OBSTACLE_TICK && (left_um_s != 0 || right_um_s != 0))
    {
        g_turn_course_moved = true;
    }
    return E_OK;
}

int motion_set_scan_pivot(int32_t left_um_s, int32_t right_um_s, uint32_t ttl_us)
{
    assert(!g_course_case);
    return motion_set_speed(left_um_s, right_um_s, ttl_us);
}

int motion_status(motion_status_t * p_status)
{
    bool const b_calibrating = g_course_case && !g_course_cutoff && !g_course_box_removed &&
                               g_tick >= COURSE_CALIBRATION_TICK &&
                               g_tick < COURSE_CALIBRATED_TICK;
    bool const b_route_hold = g_route_sim.active &&
        ((g_route_sim.safety && (g_tick >= 18U) && (g_tick < 32U)) ||
         (g_route_sim.hold_on_replacement && (0U != g_route_sim.replacement_tick)) ||
         (g_route_sim.interrupt_alignment && !g_route_sim.sonar_alignment && (0U != g_route_sim.turns) &&
          (g_tick > g_route_sim.start_tick)));
    *p_status = (motion_status_t){.ready = true,
        .hold_mask = b_calibrating ? MOTION_HOLD_CALIBRATE : (b_route_hold ? MOTION_HOLD_OPERATOR : 0U)};
    if (g_pose_return && !g_turn_case && g_tick >= MOTION_LOST_TICK && g_tick < MOTION_RESTORED_TICK)
    {
        p_status->hold_mask = MOTION_HOLD_CALIBRATE;
    }
    if ((g_course_cutoff || g_route_sim.active) && p_status->hold_mask == 0U &&
        (g_left_target != 0 || g_right_target != 0))
    {
        /* This ideal drive applies pivots directly, with no PWM reversal interval. */
        p_status->state = MOTION_STATE_RUNNING;
    }
    return E_OK;
}

int motion_snapshot(motion_status_t * p_status, motion_feedback_t * p_feedback,
                    motion_diag_t * p_diag)
{
    assert(p_diag == NULL);
    if (g_motion_snapshot_case && g_tick >= MOTION_LOST_TICK && g_tick < MOTION_RESTORED_TICK)
    {
        return E_OBJ;
    }
    (void)motion_status(p_status);
    if (g_route_sim.invalid_origin && (0U != g_route_sim.invalid_tick))
    {
        g_route_sim.origin_recovered = true;
    }
    (void)motion_feedback_snapshot(p_feedback);
    if (g_motion_fault_case && g_tick >= MOTION_LOST_TICK && g_tick < MOTION_RESTORED_TICK)
    {
        p_status->state = MOTION_STATE_FAULT;
        p_status->fault = TELEMETRY_MOTION_FAULT_EXPIRED;
    }
    if ((g_motion_snapshot_case || g_motion_fault_case) && g_tick == MOTION_ESTOP_TICK)
    {
        p_status->stop_requested = true;
        p_feedback->fresh = false;
    }
    return E_OK;
}

ER physics_snapshot(physics_status_t * p_status)
{
    *p_status = (physics_status_t){
        .state = TELEMETRY_HUMP_STATE_FLAT, .ready = true, .calibrated = true,
        .roll_cdeg = g_roll_cdeg, .attitude_unknown = g_attitude_unknown,
    };
    if (g_start_check)
    {
        p_status->attitude_sample_valid = !g_start_bad_imu || g_start_speed < 12000;
        p_status->forward_sample_us = ir_timer_at(g_tick - g_tick % 2U);
        bool const response = !g_start_airborne && (g_start_speed > 0 || (g_start_no_wheel && g_left_target > 0)) &&
                              (!g_start_late_imu || g_tick >= 25U);
        p_status->forward_specific_mg = g_start_noisy_baseline ? (g_tick < 8U ? 20 : 60) :
                                       (int16_t)(20 + (response ? 40 : 0));
    }
    return E_OK;
}

ir_sensors_state_t ir_get_state(void)
{
    if (g_route_sim.active)
    {
        return (ir_sensors_state_t){
            .left = g_route_sim.left_debounced,
            .right = g_route_sim.right_debounced,
            .barcode = g_route_sim.barcode_raw,
        };
    }
    if (g_stripe_case)
    {
        bool const b_on = (g_tick >= 10U) && (g_tick < 30U);
        return (ir_sensors_state_t){.left = b_on, .right = b_on};
    }
    if (g_junction_pause)
    {
        return (ir_sensors_state_t){.left = g_tick < 30U, .right = g_tick >= 126U, .barcode = g_tick < 30U};
    }
    if (g_retry_case)
    {
        return (ir_sensors_state_t){.right = g_tick < 5U};
    }
    return (ir_sensors_state_t){.left = true, .right = !g_route_hold};
}

line_sample_t ir_get_line_sample(void)
{
    const bool frozen = g_ir_capture_case && g_tick > IR_FROZEN_TICK && g_tick < IR_RESTORED_TICK;
    const unsigned tick = frozen ? IR_FROZEN_TICK : g_tick;
    return (line_sample_t){.timestamp_us = ir_timer_at(tick), .state = ir_get_state()};
}

static void route_check_held_wheels(void)
{
    route_sim_t * const p_sim = &g_route_sim;
    if (p_sim->hold_on_replacement && (0U != p_sim->replacement_tick))
    {
        assert((0 == g_left_target) && (0 == g_right_target));
        if (g_tick > p_sim->replacement_tick)
        {
            assert((0 == p_sim->enc_left_speed) && (0 == p_sim->enc_right_speed));
            assert(p_sim->theta_rad == p_sim->replacement_theta_rad);
            if (0U == ((g_tick * TICK_MS) % ROUTE_MOTION_FRAME_MS))
            {
                p_sim->stopped_frames++;
            }
        }
    }
    if (p_sim->interrupt_alignment && (0U != p_sim->aborts))
    {
        motion_feedback_t feedback;
        assert(E_OK == motion_feedback_snapshot(&feedback));
        assert(feedback.fresh && feedback.speed_valid[0] && feedback.speed_valid[1]);
        assert((0 == g_left_target) && (0 == g_right_target));
        if (g_tick == p_sim->end_tick)
        {
            /* The feedback sampled before this tick's hold still reports the pivot. */
            if (p_sim->sonar_alignment)
            {
                assert((feedback.left_speed_um_s > 0) && (feedback.right_speed_um_s < 0));
            }
            else
            {
                assert((feedback.left_speed_um_s < 0) && (feedback.right_speed_um_s > 0));
            }
        }
        else
        {
            assert((0 == feedback.left_speed_um_s) && (0 == feedback.right_speed_um_s));
            if (0U == ((g_tick * TICK_MS) % ROUTE_MOTION_FRAME_MS))
            {
                p_sim->stopped_frames++;
            }
        }
    }
}

static void route_advance_course(void)
{
    route_sim_t * const p_sim = &g_route_sim;
    if ((SUPERVISOR_NO_TURN != p_sim->posted) && (TURN_TICK == g_tick))
    {
        p_sim->slot = p_sim->posted;
        p_sim->barcode_seen = true;
        p_sim->barcode_edge_us = ir_timer_at(g_tick);
    }
    if (p_sim->safety && (18U == g_tick))
    {
        /* The supervisor drops a pending barcode turn when it accepts the operator hold. */
        p_sim->slot = SUPERVISOR_NO_TURN;
    }
    bool const b_mid_probe = p_sim->replace_probe_deg > 0.0;
    bool const b_second_marker = b_mid_probe
        ? (g_turning && (p_sim->y_mm > (ROUTE_JUNCTION_Y_MM - 40.0)) &&
           (((p_sim->theta_rad - p_sim->probe_origin_theta_rad) * 180.0 / PI_RAD) >= p_sim->replace_probe_deg))
        : (p_sim->x_mm >= ROUTE_U_MARKER_X_MM);
    if (p_sim->replace_left && (0U == p_sim->replacement_tick) && b_second_marker)
    {
        assert((VEHICLE_CMD_LEFT == p_sim->slot) && (g_turning == b_mid_probe));
        p_sim->slot = VEHICLE_CMD_UTURN;
        p_sim->replacement_tick = g_tick;
        p_sim->replacement_theta_rad = p_sim->theta_rad;
        p_sim->barcode_seen = true;
        p_sim->barcode_edge_us = ir_timer_at(g_tick);
    }
    bool const b_plain = route_is_plain();
    unsigned const tick_limit = b_plain ? ROUTE_MAX_TICKS :
        (((ROUTE_LAYOUT_END_T == p_sim->layout) && p_sim->straight) ? ROUTE_LATER_JUNCTION_TICKS
                                                                      : ROUTE_JUNCTION_MAX_TICKS);
    if ((p_sim->interrupt_alignment && (0U != p_sim->aborts) &&
         (g_tick > (p_sim->end_tick + (2U * ROUTE_MOTION_FRAME_MS / TICK_MS)))) ||
        ((0U != p_sim->invalid_tick) && (0U != (p_sim->aborts + p_sim->done + p_sim->refusals))) ||
        (p_sim->repeat_manual && (2U == p_sim->done) && ((p_sim->x_mm - p_sim->end_x_mm) > 100.0)) ||
        (p_sim->hold_on_replacement && (p_sim->stopped_frames >= 2U)) ||
        (g_tick > tick_limit) ||
        (b_mid_probe && (0U != p_sim->done) && (p_sim->y_mm < (ROUTE_JUNCTION_Y_MM - 170.0))) ||
        (p_sim->uturn && !b_mid_probe && (p_sim->x_mm < -100.0)) ||
        (!p_sim->uturn && route_at_destination()))
    {
        longjmp(g_finished, 1);
    }
}

ER ir_wait_change(uint8_t type, TMO timeout)
{
    assert(IR_TYPE_LINE == type);
    assert((TMO)TICK_MS == timeout);
    if (g_route_sim.active)
    {
        route_check_held_wheels();
    }
    if (g_course_cutoff || g_route_sim.active)
    {
        motion_status_t status;
        assert(E_OK == motion_status(&status));
        if (status.hold_mask != 0U || (g_left_target == 0 && g_right_target == 0))
        {
            assert(status.state == MOTION_STATE_IDLE);
        }
        else
        {
            assert(status.state == MOTION_STATE_RUNNING);
        }
    }
    if (g_launch_glitch && g_side_scan) longjmp(g_finished, 1);
    if (g_route_sim.active)
    {
        uint32_t const base_us = ir_timer_at(g_tick);
        route_step_5ms(base_us + 5000U);
        route_step_5ms(base_us + 10000U);
        g_tick++;
        int64_t const edges_l = (int64_t)llround(g_route_sim.true_left_um *
            (double)CHASSIS_EDGES_PER_REV / (double)CHASSIS_WHEEL_TRAVEL_UM);
        int64_t const edges_r = (int64_t)llround(g_route_sim.true_right_um *
            (double)CHASSIS_EDGES_PER_REV / (double)CHASSIS_WHEEL_TRAVEL_UM);
        int64_t const new_l = (edges_l * (int64_t)CHASSIS_WHEEL_TRAVEL_UM) /
                              (int64_t)CHASSIS_EDGES_PER_REV;
        int64_t const new_r = (edges_r * (int64_t)CHASSIS_WHEEL_TRAVEL_UM) /
                              (int64_t)CHASSIS_EDGES_PER_REV;
        g_route_sim.enc_left_speed = (int32_t)((new_l - g_route_sim.enc_left_um) * 100);
        g_route_sim.enc_right_speed = (int32_t)((new_r - g_route_sim.enc_right_um) * 100);
        g_route_sim.enc_left_um = new_l;
        g_route_sim.enc_right_um = new_r;
        route_advance_course();
        return E_TMOUT;
    }
    g_tick++;
    if (g_start_check)
    {
        if (!g_start_no_wheel && g_left_target > 0 && g_start_speed < 80000) g_start_speed += 5000;
        if (g_left_target == 0) g_start_speed = 0;
        g_left_travel += (int64_t)g_start_speed * TICK_MS / 1000;
        g_right_travel = g_left_travel;
        if (g_tick == 520U || g_start_follow_seen) longjmp(g_finished, 1);
        return E_TMOUT;
    }
    if (g_stripe_case && g_tick == 40U) longjmp(g_finished, 1);
    if (g_junction_pause && g_tick == 175U) longjmp(g_finished, 1);
    if (g_retry_case || g_course_case)
    {
        g_left_travel += (int64_t)g_left_target * TICK_MS / 1000;
        g_right_travel += (int64_t)g_right_target * TICK_MS / 1000;
    }
    if (g_retry_case)
    {
        if ((g_tick >= RETRY_TICK_LIMIT) ||
            ((g_terminal_tick != 0U) && (g_tick - g_terminal_tick >= TERMINAL_EXTRA_TICKS)))
        {
            longjmp(g_finished, 1);
        }
    }
    else if (!g_junction_pause && g_tick == ((g_course_case || g_launch_glitch) ? COURSE_LAST_TICK : LAST_TICK))
    {
        longjmp(g_finished, 1);
    }
    return E_TMOUT;
}

uint8_t barcode_progress_count(uint8_t * p_prefix_mm)
{
    *p_prefix_mm = 0U;
    return 0U;
}

bool ir_barcode_recent(void)
{
    if (g_route_sim.active)
    {
        return g_route_sim.barcode_seen &&
               ((uint32_t)(ir_timer_at(g_tick) - g_route_sim.barcode_edge_us) < BARCODE_SUPPRESS_US);
    }
    return false;
}

uint8_t supervisor_poll_turn(uint8_t * p_source)
{
    if (g_route_sim.active)
    {
        uint8_t const opcode = g_route_sim.slot;
        if (SUPERVISOR_NO_TURN == opcode)
        {
            return SUPERVISOR_NO_TURN;
        }
        if (SUPERVISOR_NO_TURN != g_route_sim.posted && route_is_plain())
        {
            assert((g_tick >= TURN_TICK) && (opcode == g_route_sim.posted));
            g_route_sim.command_polled = true;
        }
        *p_source = (0U != g_route_sim.repeat_tick) ? VEHICLE_SRC_TELEMETRY : VEHICLE_SRC_BARCODE;
        if (VEHICLE_CMD_UTURN == opcode)
        {
            g_route_sim.slot = SUPERVISOR_NO_TURN;
        }
        return opcode;
    }
    if (g_route_hold && g_tick >= 8U && g_route_pending)
    {
        *p_source = VEHICLE_SRC_BARCODE;
        return VEHICLE_CMD_LEFT;
    }
    if (g_turn_case && g_tick == TURN_TICK)
    {
        *p_source = g_turn_source;
        return g_turn_opcode;
    }
    return SUPERVISOR_NO_TURN;
}

bool supervisor_route_pending(uint8_t expected_opcode, bool b_consume)
{
    if (g_route_sim.active)
    {
        bool const b_pending = (SUPERVISOR_NO_TURN != g_route_sim.slot) &&
                               (g_route_sim.slot == expected_opcode);
        if (b_pending && b_consume)
        {
            g_route_sim.slot = SUPERVISOR_NO_TURN;
            g_route_sim.consumed++;
        }
        return b_pending;
    }
    if (g_route_cancel && g_turning)
    {
        g_route_pending = false;
    }
    bool const present = g_route_pending && expected_opcode == VEHICLE_CMD_LEFT;
    if (present && b_consume) g_route_pending = false;
    return present;
}

void supervisor_turn_refused(uint8_t opcode)
{
    if (g_route_sim.active)
    {
        if (SUPERVISOR_NO_TURN != g_route_sim.posted && route_is_plain())
        {
            (void)fprintf(stderr, "Refused pending corner command %u at axle (%.1f, %.1f) after %u ms\n",
                          (unsigned)opcode, g_route_sim.x_mm, g_route_sim.y_mm, g_tick * TICK_MS);
        }
        assert(g_route_sim.safety ||
               ((g_route_sim.hold_on_replacement || g_route_sim.invalid_origin) &&
                (VEHICLE_CMD_UTURN == opcode)));
        g_route_sim.refusals++;
        return;
    }
    (void)opcode;
    assert(0 && "unexpected turn refusal");
}

void supervisor_set_turning(bool b_turning)
{
    if (b_turning && g_route_sim.active && (VEHICLE_CMD_LEFT == g_route_sim.slot) &&
        (0U == g_route_sim.replacement_tick))
    {
        g_route_sim.probe_origin_theta_rad = g_route_sim.theta_rad;
    }
    g_turning = b_turning;
}

bool supervisor_wheels_loaded(void)
{
    return false;
}

uint8_t supervisor_reset_count(void)
{
    return 0U;
}

uint8_t supervisor_start_snapshot(bool * floor)
{
    if (floor != NULL)
    {
        *floor = g_launch_glitch || g_pose_return || g_junction_pause;
    }
    return 1U;
}

uint8_t supervisor_armed_start(bool * p_armed)
{
    *p_armed = g_start_check;
    return 1U;
}

void supervisor_start_check_failed(uint8_t generation)
{
    assert(generation == 1U && g_start_check);
    assert(g_left_target == 0 && g_right_target == 0);
    g_start_failures++;
}

uint8_t supervisor_recovery_snapshot(bool * clear)
{
    *clear = !g_pose_obstacle;
    return (uint8_t)(g_pose_return && g_tick >= MOTION_RESTORED_TICK);
}

uint32_t obstacle_ping_read(void)
{
    if (g_course_case && g_course_look_requests != 0U && g_tick == g_course_reply_tick)
    {
        uint32_t const servo = (uint32_t)(g_course_reply_servo / OBSTACLE_PING_DECI_PER_DEG +
                                          OBSTACLE_PING_SERVO_OFFSET);
        bool const b_open = (g_course_finite_clear && g_course_look_requests == 1U) ||
            ((0 != g_course_probe_side) && g_course_reply_servo * g_course_probe_side >= 150);
        /* A perpendicular face is farther away along each angled ray. */
        uint32_t const face_range = (0 == g_course_probe_side) ? COURSE_OBSTACLE_RANGE_MM :
            (uint32_t)lround(COURSE_OBSTACLE_RANGE_MM / cos(g_course_reply_servo * PI_RAD / 1800.0));
        uint32_t const range = b_open ? 0U : face_range;
        uint32_t const cls = b_open ? OBSTACLE_CLASS_OPEN : OBSTACLE_CLASS_VALID;

        return (1U << OBSTACLE_PING_REPLY_SH) | (servo << OBSTACLE_PING_SERVO_SH) |
            (cls << OBSTACLE_PING_CLASS_SH) | (range << OBSTACLE_PING_DIST_SH) | g_course_reply_seq;
    }
    uint32_t const ping_count = g_course_slow_clear && g_tick >= COURSE_CALIBRATED_TICK
        ? (COURSE_CALIBRATED_TICK / 6U) + (g_tick - COURSE_CALIBRATED_TICK) / COURSE_SLOW_CLEAR_PERIOD_TICKS
        : g_tick / 6U;
    uint32_t sequence = ping_count & OBSTACLE_PING_SEQ_MK;
    uint32_t cls = OBSTACLE_CLASS_OPEN;
    uint32_t range = 0U;
    uint32_t servo = OBSTACLE_PING_SERVO_OFFSET;
    uint32_t reply = 0U;
    if (g_start_check && (g_start_no_clearance || (g_start_clearance_lost && g_tick >= 12U)))
    {
        cls = OBSTACLE_CLASS_VALID;
        range = 95U;
    }

    if (g_route_sim.sonar_alignment && (g_route_sim.alignment_pivots > 0U))
    {
        /* The next new forward ping sees the box while the selected straight
           route is pivoting back toward its confirmed through line. */
        if ((0U == g_route_sim.blocked_tick) && (0U == (g_tick % COURSE_PING_PERIOD_TICKS)))
        {
            g_route_sim.blocked_tick = g_tick;
        }
        if (0U != g_route_sim.blocked_tick)
        {
            cls = OBSTACLE_CLASS_VALID;
            range = COURSE_OBSTACLE_RANGE_MM;
        }
    }
    else if (g_course_cutoff)
    {
        cls = OBSTACLE_CLASS_VALID;
        range = (g_tick < COURSE_OBSTACLE_TICK || g_tick >= COURSE_FIRST_CLEAR_TICK)
            ? COURSE_NEAR_CLEAR_MM : COURSE_OBSTACLE_RANGE_MM;
        if (g_tick >= COURSE_OFF_AXIS_TICK && g_tick < COURSE_REQUESTED_TICK)
        {
            servo += (uint32_t)(OBSTACLE_SERVO_LIMIT_DECI / OBSTACLE_PING_DECI_PER_DEG);
        }
        else if (g_tick >= COURSE_REQUESTED_TICK && g_tick < COURSE_BAD_ECHO_TICK)
        {
            reply = (1U << OBSTACLE_PING_REPLY_SH);
        }
        else if (g_tick >= COURSE_BAD_ECHO_TICK && g_tick < COURSE_FRESH_CLEAR_TICK)
        {
            cls = OBSTACLE_CLASS_NO_ECHO;
            range = 0U;
        }
        else if (g_tick >= COURSE_REPEATED_CLEAR_TICK)
        {
            /* Repeat seq 35, then present seq 36 unless that ping was missed. */
            if (!g_course_missed_ping || g_tick < COURSE_REPEATED_CLEAR_TICK + COURSE_PING_PERIOD_TICKS)
            {
                sequence = (ping_count - 1U) & OBSTACLE_PING_SEQ_MK;
            }
        }
    }
    else if (g_turn_obstacle && g_tick >= TURN_TICK)
    {
        cls = OBSTACLE_CLASS_VALID;
        range = (g_tick < TURN_OBSTACLE_TICK) ? COURSE_NEAR_CLEAR_MM : COURSE_OBSTACLE_RANGE_MM;
    }
    else if (g_course_case && g_tick >= COURSE_OBSTACLE_TICK && g_tick < COURSE_CLEAR_TICK)
    {
        cls = (g_tick < COURSE_UNKNOWN_TICK) ? OBSTACLE_CLASS_VALID : OBSTACLE_CLASS_NO_ECHO;
        range = (cls == OBSTACLE_CLASS_VALID) ? COURSE_OBSTACLE_RANGE_MM : 0U;
        if (!g_course_finite_clear && !g_course_box_removed && g_tick == COURSE_SINGLE_OPEN_TICK)
        {
            cls = OBSTACLE_CLASS_OPEN;
            range = 0U;
        }
    }
    else if (g_course_case && g_course_finite_clear && g_tick >= COURSE_CLEAR_TICK)
    {
        cls = OBSTACLE_CLASS_VALID;
        range = COURSE_NEAR_CLEAR_MM;
    }

    return reply | (cls << OBSTACLE_PING_CLASS_SH) | (range << OBSTACLE_PING_DIST_SH) |
           (sequence << OBSTACLE_PING_SEQ_SH) | (servo << OBSTACLE_PING_SERVO_SH);
}

ER obstacle_ping_request(int16_t servo_deci, uint8_t seq)
{
    static const int16_t angles[2][COURSE_LOOK_REPLIES] = {
        {-OBSTACLE_SERVO_LIMIT_DECI, 0, OBSTACLE_SERVO_LIMIT_DECI},
        {OBSTACLE_SERVO_LIMIT_DECI / 2, 0, -OBSTACLE_SERVO_LIMIT_DECI / 2},
    };
    if (g_pose_obstacle)
    {
        assert(g_tick >= MOTION_RESTORED_TICK);
        assert(g_left_target == 0 && g_right_target == 0);
        g_side_scan |= servo_deci != 0;
        return E_OK;
    }
    if (g_launch_glitch)
    {
        g_side_scan = g_side_scan || servo_deci != 0;
        return E_OK;
    }
    unsigned const sweep = g_course_look_requests / COURSE_LOOK_REPLIES;
    unsigned const ray = g_course_look_requests % COURSE_LOOK_REPLIES;

    if (g_turn_obstacle || g_route_sim.sonar_alignment)
    {
        assert(g_route_sim.sonar_alignment ? (0U != g_route_sim.blocked_tick) : (g_tick >= TURN_OBSTACLE_TICK));
        assert(g_left_target == 0 && g_right_target == 0);
        g_turn_look_requests++;
        return E_OK;
    }
    assert(g_course_case);
    assert(g_left_target == 0 && g_right_target == 0);
    if (g_tick < COURSE_CALIBRATION_TICK)
    {
        assert(g_tick - g_course_last_look_tick < COURSE_MAX_LOOK_GAP_TICKS);
        int const expected = ((1U == sweep) && (0 != g_course_probe_side))
            ? g_course_probe_side * 50 * ((int)ray + 1) : angles[sweep % 2U][ray];
        assert(servo_deci == expected);
    }
    g_course_last_look_tick = g_tick;
    g_course_reply_seq = seq;
    g_course_reply_servo = servo_deci;
    g_course_reply_tick = g_tick + 6U;
    g_course_look_requests++;
    if (g_course_finite_clear && g_course_look_requests == 2U)
    {
        g_course_reply_tick = 0U;
    }
    return E_OK;
}

uint16_t obstacle_get_faults(void)
{
    /* The obstacle diagnostic snapshot has not reported the new fault yet. */
    return 0U;
}

uint32_t system_fault_get(void)
{
    return (g_turn_case && !g_pose_return && !g_turn_obstacle && !g_ir_capture_case && g_tick >= SONAR_FAULT_TICK)
        ? FAULT_SONAR_NO_ECHO_BIT : 0U;
}

void liveness_kick(uint8_t task_bit)
{
    assert(LIVE_LINE == task_bit);
}

void system_fault_assign(fault_owner_t owner, uint32_t mask, uint32_t active)
{
    (void)owner;
    (void)mask;
    assert(g_turn_case || g_retry_case ||
           (g_route_sim.active && ((active & ~FAULT_LINE_LOST_BIT) == 0U)) ||
           0U == active);
}

uint32_t telemetry_odometer_mm(const motion_feedback_t * p_feedback)
{
    (void)p_feedback;
    return 0U;
}

ER telemetry_queue_try_push(const uint8_t * p_frame, uint16_t len)
{
    bool const b_first = (0U == g_course_profiles);
    uint8_t const * const p_wire = &p_frame[1];

    assert(g_course_case && g_course_profiles < 2U);
    assert(g_left_target == 0 && g_right_target == 0);
    assert(p_frame[0] == TELEMETRY_TOPIC_PROFILE && len == TELEMETRY_PROFILE_WIRE_SIZE + 1U);
    assert(p_wire[TELEMETRY_PROFILE_OFF_STATUS] == 4U);
    assert(p_wire[TELEMETRY_PROFILE_OFF_FLAGS] == (g_course_probe_side == 0 ? 0U : 2U));
    if (0 != g_course_probe_side)
    {
        assert(p_wire[TELEMETRY_PROFILE_OFF_SIDE] == (g_course_probe_side > 0 ? 0U : 1U));
    }
    assert(p_wire[TELEMETRY_PROFILE_OFF_WIDTH_MM] == 0U &&
           p_wire[TELEMETRY_PROFILE_OFF_WIDTH_MM + 1U] == 0U);
    for (unsigned slot = 0U; slot < TELEMETRY_PROFILE_COUNT_SLOTS; slot++)
    {
        unsigned const ix = TELEMETRY_PROFILE_OFF_SLOTS + 2U * slot;
        unsigned const word = ((unsigned)p_wire[ix] << 8U) | p_wire[ix + 1U];
        unsigned expected = 0U;

        int const offset = (int)slot - 18;
        bool const fine = !b_first && ((0 == g_course_probe_side)
            ? ((-2 == offset) || (2 == offset))
            : ((offset * g_course_probe_side >= 1) && (offset * g_course_probe_side <= 3)));
        if (fine || (-4 == offset) || (4 == offset) ||
            ((0 == offset) && (!b_first || !g_course_finite_clear)))
        {
            unsigned const range = (0 == g_course_probe_side) ? COURSE_OBSTACLE_RANGE_MM :
                (unsigned)lround(COURSE_OBSTACLE_RANGE_MM / cos(offset * 5 * PI_RAD / 180.0));
            expected = 0x8000U | range;
        }
        if ((g_course_finite_clear && (-4 == offset)) ||
            ((0U != expected) && (offset * g_course_probe_side >= 3)))
        {
            expected = 0x9000U;
        }
        assert(word == expected);
    }
    g_course_profiles++;
    return b_first ? E_TMOUT : E_OK;
}

ER telemetry_emit_event(uint8_t source, uint8_t type, int16_t p1,
                        uint16_t p2, uint32_t odometer_mm)
{
    (void)source;
    (void)type;
    (void)p1;
    (void)p2;
    (void)odometer_mm;
    assert(false && "line_task must not call blocking telemetry_emit_event");
    return E_TMOUT;
}

static void route_note_turn_event(uint8_t type, int16_t p1, uint16_t p2)
{
    route_sim_t * const p_sim = &g_route_sim;
    if (TELEMETRY_EVT_TURN_START == type)
    {
        bool const b_right = VEHICLE_CMD_RIGHT == p_sim->posted;
        int16_t const kind = p_sim->uturn ? LINE_TURN_KIND_UTURN :
            (p_sim->straight ? LINE_TURN_KIND_STRAIGHT : (b_right ? LINE_TURN_KIND_RIGHT : LINE_TURN_KIND_LEFT));
        assert((p1 == kind) && (SUPERVISOR_NO_TURN == p_sim->slot));
        p_sim->event_kind = p1;
        p_sim->turns++;
        p_sim->start_tick = g_tick;
        p_sim->turn_start_x_mm = p_sim->x_mm;
        if (!p_sim->uturn && (p_sim->y_mm < (ROUTE_JUNCTION_Y_MM - 40.0)))
        {
            p_sim->early = true;
        }
        return;
    }
    assert(p1 == p_sim->event_kind);
    if (p_sim->invalid_origin || p_sim->invalid_capture || p_sim->repeat_manual)
    {
        assert((0 == g_left_target) && (0 == g_right_target));
    }
    p_sim->end_tick = g_tick;
    p_sim->end_age_ms = p2;
    if (TELEMETRY_EVT_TURN_ABORT == type)
    {
        p_sim->aborts++;
        return;
    }
    p_sim->done++;
    p_sim->end_x_mm = p_sim->x_mm;
    p_sim->end_y_mm = p_sim->y_mm;
    if (p_sim->repeat_manual && (1U == p_sim->done))
    {
        assert(!g_turning);
        p_sim->repeat_tick = g_tick;
        p_sim->slot = VEHICLE_CMD_UTURN;
    }
}

ER telemetry_try_emit_event(uint8_t source, uint8_t type, int16_t p1,
                            uint16_t p2, uint32_t odometer_mm)
{
    if (g_route_sim.active &&
        (type == TELEMETRY_EVT_TURN_START || type == TELEMETRY_EVT_TURN_DONE ||
         type == TELEMETRY_EVT_TURN_ABORT))
    {
        g_route_sim.turn_events++;
        route_note_turn_event(type, p1, p2);
    }
    if (type == TELEMETRY_EVT_TURN_ABORT)
    {
        g_turn_abort_tick = g_tick;
    }
    if ((g_turn_obstacle || g_route_sim.sonar_alignment) && type == TELEMETRY_EVT_OBSTACLE_SEEN)
    {
        assert(p1 == COURSE_OBSTACLE_RANGE_MM);
        if (g_route_sim.sonar_alignment)
        {
            assert(g_tick == g_route_sim.blocked_tick);
        }
        g_turn_observations++;
    }
    if (g_course_case && type == TELEMETRY_EVT_OBSTACLE_SEEN)
    {
        assert(source == TELEMETRY_EVENT_SOURCE_LINE);
        assert(p2 == 0U);
        if (!g_course_cutoff)
        {
            assert(p1 == COURSE_OBSTACLE_RANGE_MM);
            assert(g_tick >= COURSE_OBSTACLE_TICK && g_tick < COURSE_UNKNOWN_TICK);
        }
        g_course_contact_tick = g_tick;
        g_course_contact_mm = p1;
        g_course_contact_um = (g_left_travel + g_right_travel) / 2;
        g_course_contacts++;
    }
    if (g_course_case && type == TELEMETRY_EVT_SCAN_DONE)
    {
        assert(source == TELEMETRY_EVENT_SOURCE_LINE);
        assert(p1 == 4 && p2 == 0U);
        g_course_scan_completions++;
    }
    if (g_retry_case && type == TELEMETRY_EVT_LINE_LOST)
    {
        assert(source == TELEMETRY_EVENT_SOURCE_LINE);
        assert(p1 == LINE_SIDE_RIGHT);
        assert(g_terminal_tick == 0U);
        if (p2 == 0U)
        {
            assert(g_retry_events == 0U);
            g_line_loss_events++;
        }
        else if ((p2 & RETRY_EVENT_FLAG) != 0U)
        {
            assert(g_line_loss_events == 1U);
            g_retry_events++;
            assert(g_retry_events <= LINE_RETRY_COUNT);
            assert(p2 == (RETRY_EVENT_FLAG | g_retry_events));
        }
        else
        {
            assert(g_retry_events == LINE_RETRY_COUNT);
            assert(p2 == (LINE_SEARCH_SWEEP_DECI / 10));
            g_terminal_tick = g_tick;
        }
    }
    if (type == TELEMETRY_EVT_JUNCTION)
    {
        assert(source == TELEMETRY_EVENT_SOURCE_LINE);
        if (p1 == (int16_t)TELEMETRY_JUNCTION_KIND_STRIPE)
        {
            g_stripe_reports++;
            g_stripe_bits = p2;
        }
        else if (p1 == (int16_t)TELEMETRY_JUNCTION_KIND_PROBE)
        {
            g_probe_reports++;
            g_probe_bits = p2;
        }
    }
    (void)odometer_mm;
    return E_OK;
}

void log_write(log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    (void)sev;
    (void)task;
    (void)mod;
    (void)p_fmt;
}

static int32_t run_line(bool b_left, int16_t roll_cdeg)
{
    g_tick = 0U;
    g_slow_left = b_left;
    g_roll_cdeg = roll_cdeg;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_left_target > 0 && g_right_target > 0);
    return b_left ? (g_left_target - g_right_target) : (g_right_target - g_left_target);
}

static int turn_aborts_on_sonar(void)
{
    g_turn_case = true;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    if (!g_began_turn || g_moved_after_fault || g_turn_abort_tick != SONAR_FAULT_TICK)
    {
        (void)fprintf(stderr, "Sonar fault failed to abort the active turn at tick %u\n",
                      SONAR_FAULT_TICK);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static int stale_line_capture(bool during_turn)
{
    g_ir_capture_case = true;
    g_turn_case = during_turn;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_ir_approached);
    assert(!g_ir_moved_while_stale);
    assert(g_ir_resumed);
    if (during_turn)
    {
        assert(g_began_turn);
        assert(g_turn_abort_tick == IR_STOP_TICK);
    }
    return EXIT_SUCCESS;
}

static int turn_aborts_on_obstacle(uint8_t opcode)
{
    g_turn_case = true;
    g_turn_obstacle = true;
    g_turn_opcode = opcode;
    if (opcode == VEHICLE_CMD_STRAIGHT)
    {
        g_turn_source = VEHICLE_SRC_TELEMETRY;
    }
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_began_turn);
    assert(g_turn_abort_tick == TURN_OBSTACLE_TICK);
    assert(!g_turn_course_moved);
    assert(g_turn_observations == 1U);
    assert(g_turn_look_requests > 0U);
    return EXIT_SUCCESS;
}

static int line_retry_events(void)
{
    g_retry_case = true;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_line_loss_events == 1U);
    assert(g_retry_events == LINE_RETRY_COUNT);
    assert(g_terminal_tick != 0U);
    assert(g_left_target == 0 && g_right_target == 0);
    return EXIT_SUCCESS;
}

static int bounded_course_hold(bool b_finite_clear)
{
    g_course_case = true;
    g_course_finite_clear = b_finite_clear;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_course_approach);
    assert(!g_course_hold_moved);
    assert(g_course_look_requests >= 2U * COURSE_LOOK_REPLIES);
    assert(g_course_resumed);
    assert(g_course_contacts == 1U);
    assert(g_course_scan_completions >= 2U);
    assert(g_course_profiles == 2U && g_course_profiles == g_course_scan_completions);
    assert(g_hug_locks > 1U);
    assert(g_hug_locks < g_tick);
    return EXIT_SUCCESS;
}

static int bounded_course_cutoff(void)
{
    g_course_case = true;
    g_course_cutoff = true;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_course_near_approach);
    assert(g_course_brake_tick == COURSE_OBSTACLE_TICK);
    assert(g_course_contacts == 1U && g_course_contact_tick == COURSE_OBSTACLE_TICK);
    assert(g_course_contact_mm == COURSE_OBSTACLE_RANGE_MM);
    unsigned const expected_tick = COURSE_RESUME_TICK + (g_course_missed_ping ? COURSE_PING_PERIOD_TICKS : 0U);
    assert(g_course_resume_tick == expected_tick);
    assert(g_course_look_requests == 2U * COURSE_LOOK_REPLIES);
    assert(g_left_target > 0 && g_right_target > 0);
    return EXIT_SUCCESS;
}

static int bounded_course_box_removed(bool b_invalid_rest)
{
    g_course_case = true;
    g_course_box_removed = true;
    g_course_invalid_rest = b_invalid_rest;
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    assert(g_course_approach && g_course_contacts == 1U);
    unsigned const resume_ping = b_invalid_rest ? 4U : 2U;
    assert(g_course_resume_tick == COURSE_CLEAR_TICK + resume_ping * COURSE_PING_PERIOD_TICKS);
    int64_t const face_um = g_course_contact_um + (int64_t)g_course_contact_mm * 1000;
    assert(g_left_travel > face_um && g_right_travel > face_um);
    assert(g_left_target > 0 && g_right_target > 0);
    return EXIT_SUCCESS;
}

static bool parse_double_arg(char const * p_arg, double * p_out)
{
    char * p_end = NULL;
    double const val = strtod(p_arg, &p_end);
    if ((p_arg[0] == '\0') || (*p_end != '\0') || !isfinite(val))
    {
        return false;
    }
    *p_out = val;
    return true;
}

static bool parse_wheel_profile(char const * p_profile, double * p_left, double * p_right)
{
    if (strcmp(p_profile, "nov20") == 0)
    {
        *p_left = 120.0 / 125.0;
        *p_right = 112.0 / 125.0;
        return true;
    }
    if (strcmp(p_profile, "nov16") == 0)
    {
        *p_left = 169.0 / 163.0;
        *p_right = 161.0 / 163.0;
        return true;
    }
    if (strcmp(p_profile, "matched") == 0)
    {
        *p_left = 1.0;
        *p_right = 1.0;
        return true;
    }
    return false;
}

typedef struct
{
    char const * p_name;
    uint8_t command;
    route_layout_t layout;
    route_setup_t setup;
} route_case_t;

static route_case_t const g_route_cases[] = {
    {"route-plain-round-left", SUPERVISOR_NO_TURN, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_NORMAL},
    {"route-plain-round-right", SUPERVISOR_NO_TURN, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-plain-square-left", SUPERVISOR_NO_TURN, ROUTE_LAYOUT_SQUARE, ROUTE_SETUP_NORMAL},
    {"route-plain-square-right", SUPERVISOR_NO_TURN, ROUTE_LAYOUT_SQUARE, ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-round-left-pending-straight", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_NORMAL},
    {"route-round-right-pending-straight", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_ROUND,
     ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-round-left-pending-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_NORMAL},
    {"route-round-right-pending-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-round-left-pending-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_NORMAL},
    {"route-round-right-pending-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_ROUND, ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-square-left-pending-straight", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_SQUARE, ROUTE_SETUP_NORMAL},
    {"route-square-right-pending-straight", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_SQUARE,
     ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-square-left-pending-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_SQUARE, ROUTE_SETUP_NORMAL},
    {"route-square-right-pending-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_SQUARE, ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-square-left-pending-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_SQUARE, ROUTE_SETUP_NORMAL},
    {"route-square-right-pending-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_SQUARE,
     ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_NORMAL},
    {"route-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_NORMAL},
    {"route-straight-left", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_NORMAL},
    {"route-straight-right", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_THROUGH_BRANCH,
     ROUTE_SETUP_RIGHT_ALTERNATIVE},
    {"route-t-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_END_T, ROUTE_SETUP_NORMAL},
    {"route-t-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_END_T, ROUTE_SETUP_NORMAL},
    {"route-t-straight-later", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_END_T, ROUTE_SETUP_NORMAL},
    {"route-missing-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_OPPOSITE_BRANCH, ROUTE_SETUP_NORMAL},
    {"route-missing-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_OPPOSITE_BRANCH, ROUTE_SETUP_NORMAL},
    {"route-single-left", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_SINGLE_SIDE, ROUTE_SETUP_NORMAL},
    {"route-single-right", VEHICLE_CMD_RIGHT, ROUTE_LAYOUT_SINGLE_SIDE, ROUTE_SETUP_NORMAL},
    {"route-offset-back", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_READER_BACK},
    {"route-offset-front", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_READER_FRONT},
    {"route-left-mirror", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_MIRROR_START},
    {"route-cancel", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_HOLD_BEFORE_JUNCTION},
    {"route-event-abort", VEHICLE_CMD_LEFT, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_HOLD_DURING_ALIGNMENT},
    {"route-straight-obstacle", VEHICLE_CMD_STRAIGHT, ROUTE_LAYOUT_THROUGH_BRANCH,
     ROUTE_SETUP_OBSTACLE_DURING_ALIGNMENT},
    {"route-u", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_NORMAL},
    {"route-u-replaces-left", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_REPLACE_LEFT},
    {"route-u-during-probe", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_REPLACE_PROBE},
    {"route-u-during-late-probe", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH,
     ROUTE_SETUP_REPLACE_LATE_PROBE},
    {"route-u-hold-during-probe", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH,
     ROUTE_SETUP_HOLD_ON_REPLACEMENT},
    {"route-u-repeat-manual", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_REPEAT_MANUAL_U},
    {"route-u-invalid-origin", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH, ROUTE_SETUP_INVALID_U_ORIGIN},
    {"route-u-invalid-capture", VEHICLE_CMD_UTURN, ROUTE_LAYOUT_THROUGH_BRANCH,
     ROUTE_SETUP_INVALID_U_CAPTURE},
};

static void route_check_plain(void)
{
    route_sim_t const * const p_sim = &g_route_sim;
    bool const b_round = ROUTE_LAYOUT_ROUND == p_sim->layout;
    (void)printf("route-plain: reached=%d sensor=%.1f mm overshoot=%.1f mm axle=%.1f mm "
                 "body=%.1f mm track=%.1f mm exit_dev=%.1f mm sin=%.3f edges=%u turns=%u wheels=%ld/%ld\n",
                 p_sim->reached_exit ? 1 : 0, p_sim->max_sensor_dev_mm, p_sim->max_overshoot_mm,
                 p_sim->max_axle_dev_mm, p_sim->max_body_dist_mm, p_sim->max_track_dist_mm,
                 p_sim->max_exit_dev_mm,
                 sin(p_sim->theta_rad), p_sim->barcode_edges, p_sim->turn_events,
                 (long)g_left_target, (long)g_right_target);
    assert(p_sim->reached_exit);
    assert(p_sim->exit_samples > 0U);
    assert(p_sim->max_body_dist_mm <= ROUTE_TRACK_HALF_MM);
    if (SUPERVISOR_NO_TURN == p_sim->posted)
    {
        assert(p_sim->max_track_dist_mm <= ROUTE_TRACK_HALF_MM);
    }
    assert((p_sim->right_exit ? -1.0 : 1.0) * sin(p_sim->theta_rad) > ROUTE_ROUND_MIN_EXIT_SIN);
    assert(!p_sim->exit_wheel_stopped);
    assert(g_left_target > 0 && g_right_target > 0);
    assert(p_sim->enc_left_speed > 0 && p_sim->enc_right_speed > 0);
    assert(!g_turning && (0U == p_sim->turn_events));
    assert(p_sim->command_polled == (SUPERVISOR_NO_TURN != p_sim->posted));
    assert((p_sim->slot == p_sim->posted) && (0U == p_sim->consumed) && (0U == p_sim->turns));
    assert((0U == p_sim->done) && (0U == p_sim->aborts) && (0U == p_sim->refusals));
    if (b_round)
    {
        if (p_sim->reader_dx_mm > 0.0)
        {
            /* Only sampled leading-reader placements cross the bend tape of a left turn. */
            assert(p_sim->right_exit || (p_sim->barcode_edges > 0U));
        }
        assert(p_sim->max_sensor_dev_mm <= ROUTE_ROUND_MAX_SENSOR_MM);
        assert(p_sim->max_exit_dev_mm <= ROUTE_ROUND_MAX_EXIT_DEV_MM);
        return;
    }
    assert(p_sim->max_overshoot_mm <= ROUTE_MAX_OVERSHOOT_MM);
    assert(p_sim->max_exit_dev_mm <= ROUTE_MAX_EXIT_DEV_MM);
    assert(p_sim->max_axle_dev_mm <= ((double)CHASSIS_WHEEL_TRACK_MM / 2.0));
}

static void route_check_u_turn(void)
{
    route_sim_t const * const p_sim = &g_route_sim;
    if (p_sim->invalid_origin || p_sim->invalid_capture)
    {
        assert((0U != p_sim->invalid_tick) && (0U == p_sim->done));
        assert((1U == (p_sim->aborts + p_sim->refusals)) && (p_sim->turns == p_sim->aborts));
        assert(SUPERVISOR_NO_TURN == p_sim->slot);
        if (p_sim->invalid_origin)
        {
            assert((TURN_TICK == p_sim->invalid_tick) && (0U == p_sim->alignment_pivots));
        }
        else
        {
            assert((p_sim->alignment_pivots > 0U) && (p_sim->invalid_tick == p_sim->end_tick));
        }
        return;
    }
    if (p_sim->repeat_manual)
    {
        assert((2U == p_sim->turns) && (2U == p_sim->done) && (0U == p_sim->aborts));
        assert((0U != p_sim->repeat_tick) && (p_sim->start_tick == p_sim->repeat_tick));
        assert((p_sim->end_tick > p_sim->start_tick) && ((p_sim->x_mm - p_sim->end_x_mm) >= 100.0));
        assert((fabs(p_sim->y_mm) < (double)LINE_SENSOR_SPACING_MM) && (cos(p_sim->theta_rad) > 0.9));
        return;
    }
    if (p_sim->hold_on_replacement)
    {
        assert((0U != p_sim->replacement_tick) && (p_sim->stopped_frames >= 2U));
        assert((SUPERVISOR_NO_TURN == p_sim->slot) && (0U == p_sim->done) && (0U == p_sim->consumed));
        assert((1U == (p_sim->refusals + p_sim->aborts)) && (p_sim->turns == p_sim->aborts));
        return;
    }
    if (p_sim->replace_probe_deg > 0.0)
    {
        assert((p_sim->replacement_tick > 0U) && (p_sim->start_tick >= p_sim->replacement_tick) &&
               (p_sim->start_tick <= (p_sim->replacement_tick + 1U)));
        double const yaw_at_marker =
            (p_sim->replacement_theta_rad - p_sim->probe_origin_theta_rad) * 180.0 / PI_RAD;
        assert((yaw_at_marker >= p_sim->replace_probe_deg) &&
               (yaw_at_marker < (p_sim->replace_probe_deg + 2.0)));
        assert((fabs(p_sim->x_mm - ROUTE_THROUGH_X_MM) < (double)LINE_SENSOR_SPACING_MM) &&
               (sin(p_sim->theta_rad) < -0.9));
        assert((p_sim->y_mm < (ROUTE_JUNCTION_Y_MM - 170.0)) && ((p_sim->end_y_mm - p_sim->y_mm) >= 100.0));
    }
    else
    {
        double const marker_x = p_sim->replace_left ? ROUTE_U_MARKER_X_MM : 0.0;
        assert((fabs(p_sim->turn_start_x_mm - marker_x) < 20.0) && (p_sim->x_mm < -100.0));
        assert(p_sim->start_tick == (p_sim->replace_left ? p_sim->replacement_tick : TURN_TICK));
        assert((fabs(p_sim->y_mm) < (double)LINE_SENSOR_SPACING_MM) && (cos(p_sim->theta_rad) < -0.9));
        assert((p_sim->end_x_mm - p_sim->x_mm) >= 100.0);
    }
    assert((0U == p_sim->consumed) && (0U == p_sim->aborts) && (p_sim->alignment_pivots > 0U));
}

static void route_check_unavailable_exit(void)
{
    route_sim_t const * const p_sim = &g_route_sim;
    assert(route_at_destination() && (p_sim->slot == p_sim->posted) && (0U == p_sim->consumed));
    assert((0U == p_sim->turns) && (0U == p_sim->done) && (0U == p_sim->aborts) && (0U == p_sim->refusals));
    if (ROUTE_LAYOUT_OPPOSITE_BRANCH == p_sim->layout)
    {
        assert((fabs(p_sim->x_mm - ROUTE_THROUGH_X_MM) < 30.0) && (sin(p_sim->theta_rad) > 0.9));
    }
    else
    {
        double const direction = p_sim->right_exit ? 1.0 : -1.0;
        assert((fabs(p_sim->y_mm - ROUTE_JUNCTION_Y_MM) < 30.0) && ((cos(p_sim->theta_rad) * direction) > 0.9));
    }
    assert(p_sim->enc_left_speed > 0 && p_sim->enc_right_speed > 0);
}

static void route_check_junction(void)
{
    route_sim_t const * const p_sim = &g_route_sim;
    (void)fprintf(stderr, "route pose %.1f %.1f heading %.1f starts %u done %u early %u start x %.1f\n",
                  p_sim->x_mm, p_sim->y_mm, p_sim->theta_rad * 180.0 / PI_RAD,
                  p_sim->turns, p_sim->done, (unsigned)p_sim->early, p_sim->turn_start_x_mm);
    if (0U != p_sim->turns)
    {
        (void)fprintf(stderr, "start %u ms, end %u ms, reported duration %u ms, alignment pivots %u\n",
                      p_sim->start_tick * TICK_MS, p_sim->end_tick * TICK_MS,
                      p_sim->end_age_ms, p_sim->alignment_pivots);
    }
    if (p_sim->uturn)
    {
        route_check_u_turn();
        if (p_sim->invalid_origin || p_sim->invalid_capture || p_sim->repeat_manual ||
            p_sim->hold_on_replacement)
        {
            return;
        }
    }
    else
    {
        assert(!p_sim->early && !p_sim->early_scan);
        if (p_sim->safety)
        {
            assert((0U == p_sim->turns) && (0U == p_sim->done) && (0U == p_sim->aborts));
            return;
        }
        assert(!p_sim->departed_past_end);
        if ((ROUTE_LAYOUT_OPPOSITE_BRANCH == p_sim->layout) || (ROUTE_LAYOUT_SINGLE_SIDE == p_sim->layout))
        {
            route_check_unavailable_exit();
            return;
        }
        if (p_sim->interrupt_alignment)
        {
            assert((1U == p_sim->turns) && (0U == p_sim->done) && (1U == p_sim->aborts));
            assert((0 == g_left_target) && (0 == g_right_target) && (SUPERVISOR_NO_TURN == p_sim->slot));
            assert(p_sim->stopped_frames >= 2U);
            if (p_sim->sonar_alignment)
            {
                assert((1U == p_sim->consumed) && (p_sim->start_tick > TURN_TICK));
                assert((p_sim->blocked_tick == p_sim->end_tick) && (1U == g_turn_observations));
                assert(g_turn_look_requests > 0U);
            }
        }
        assert((p_sim->alignment_pivots > 0U) && (p_sim->end_tick > p_sim->start_tick));
        assert(p_sim->end_age_ms == ((p_sim->end_tick - p_sim->start_tick) * TICK_MS));
        if (p_sim->interrupt_alignment)
        {
            return;
        }
        assert(0U == p_sim->aborts);
        assert(!p_sim->wrong_way);
        assert(1U == p_sim->consumed);
        if ((ROUTE_LAYOUT_END_T == p_sim->layout) && p_sim->straight)
        {
            assert(p_sim->pending_on_exit && route_at_destination());
            assert(fabs(fabs(p_sim->turn_start_x_mm - ROUTE_THROUGH_X_MM) - ROUTE_NEXT_JUNCTION_MM) < 30.0);
            assert((fabs(p_sim->y_mm - ROUTE_JUNCTION_Y_MM) < 30.0) && (fabs(cos(p_sim->theta_rad)) > 0.9));
            assert(((p_sim->x_mm - p_sim->end_x_mm) * cos(p_sim->theta_rad)) > 100.0);
        }
        else if (p_sim->straight)
        {
            assert((fabs(p_sim->x_mm - ROUTE_THROUGH_X_MM) < 30.0) && (sin(p_sim->theta_rad) > 0.9));
            assert((p_sim->y_mm > (ROUTE_JUNCTION_Y_MM + 400.0)) && ((p_sim->y_mm - p_sim->end_y_mm) > 100.0));
        }
        else
        {
            double const direction = p_sim->right_exit ? 1.0 : -1.0;
            assert((fabs(p_sim->y_mm - ROUTE_JUNCTION_Y_MM) < 30.0) && ((cos(p_sim->theta_rad) * direction) > 0.9));
            assert(((p_sim->x_mm - ROUTE_THROUGH_X_MM) * direction) > 170.0);
            assert(((p_sim->x_mm - p_sim->end_x_mm) * direction) > 100.0);
        }
    }
    assert((1U == p_sim->turns) && (1U == p_sim->done));
    assert((SUPERVISOR_NO_TURN == p_sim->slot) && (p_sim->enc_left_speed > 0) && (p_sim->enc_right_speed > 0));
}

static int run_route_case(route_case_t const * p_case, int argc, char * argv[])
{
    route_setup_t const setup = p_case->setup;
    bool const b_plain = (ROUTE_LAYOUT_SQUARE == p_case->layout) || (ROUTE_LAYOUT_ROUND == p_case->layout);
    bool const b_round = ROUTE_LAYOUT_ROUND == p_case->layout;
    bool const b_model_arg = (5 == argc) && (VEHICLE_CMD_UTURN != p_case->command) &&
        ((ROUTE_SETUP_NORMAL == setup) || (ROUTE_SETUP_RIGHT_ALTERNATIVE == setup));
    bool const b_args_ok = b_plain ? (5 == argc) : ((2 == argc) || (4 == argc) || b_model_arg);
    double offset_mm = ROUTE_START_OFFSET_MM;
    double reader_dx_mm = (ROUTE_SETUP_READER_BACK == setup) ? -ROUTE_READER_OFFSET_MM :
                          ((ROUTE_SETUP_READER_FRONT == setup) ? ROUTE_READER_OFFSET_MM : 0.0);
    double left_gain = 1.0;
    double right_gain = 1.0;
    if (!b_args_ok ||
        ((argc >= 4) && (!parse_double_arg(argv[2], &offset_mm) || !parse_double_arg(argv[3], &reader_dx_mm))) ||
        ((5 == argc) && !parse_wheel_profile(argv[4], &left_gain, &right_gain)))
    {
        (void)fprintf(stderr, "Invalid arguments for %s\n", p_case->p_name);
        return EXIT_FAILURE;
    }
    if (ROUTE_SETUP_MIRROR_START == setup)
    {
        offset_mm = -offset_mm;
    }
    bool const b_ideal = !b_plain && !b_model_arg;
    double probe_deg = 0.0;
    if ((ROUTE_SETUP_REPLACE_PROBE == setup) || (ROUTE_SETUP_HOLD_ON_REPLACEMENT == setup))
    {
        probe_deg = ROUTE_PROBE_DEG;
    }
    else if (ROUTE_SETUP_REPLACE_LATE_PROBE == setup)
    {
        probe_deg = ROUTE_LATE_PROBE_DEG;
    }
    bool const b_replace_left = (ROUTE_SETUP_REPLACE_LEFT == setup) || (probe_deg > 0.0);
    g_route_sim = (route_sim_t){
        .active = true,
        .layout = p_case->layout,
        .bend_x_mm = (b_round || !b_plain) ? ROUTE_ROUND_BEND_X_MM : ROUTE_CORNER_X_MM,
        .radius_mm = (b_round || !b_plain) ? ROUTE_ROUND_RADIUS_MM : 0.0,
        .exit_len_mm = b_plain ? (b_round ? ROUTE_ROUND_EXIT_LEN_MM : ROUTE_EXIT_LEN_MM) : 0.0,
        .barcode_left_mm = (b_plain && !b_round) ? ROUTE_BARCODE_LEFT_MM : ROUTE_ROUND_BARCODE_LEFT_MM,
        .fast_fall = b_round || (!b_plain && b_model_arg),
        .ideal = b_ideal,
        .right_exit = (ROUTE_SETUP_RIGHT_ALTERNATIVE == setup) ||
                      (!b_plain && (VEHICLE_CMD_RIGHT == p_case->command)),
        .straight = VEHICLE_CMD_STRAIGHT == p_case->command,
        .uturn = VEHICLE_CMD_UTURN == p_case->command,
        .posted = b_replace_left ? VEHICLE_CMD_LEFT : p_case->command,
        .slot = SUPERVISOR_NO_TURN,
        .safety = ROUTE_SETUP_HOLD_BEFORE_JUNCTION == setup,
        .sonar_alignment = ROUTE_SETUP_OBSTACLE_DURING_ALIGNMENT == setup,
        .interrupt_alignment = (ROUTE_SETUP_HOLD_DURING_ALIGNMENT == setup) ||
                               (ROUTE_SETUP_OBSTACLE_DURING_ALIGNMENT == setup),
        .replace_left = b_replace_left,
        .hold_on_replacement = ROUTE_SETUP_HOLD_ON_REPLACEMENT == setup,
        .repeat_manual = ROUTE_SETUP_REPEAT_MANUAL_U == setup,
        .invalid_origin = ROUTE_SETUP_INVALID_U_ORIGIN == setup,
        .invalid_capture = ROUTE_SETUP_INVALID_U_CAPTURE == setup,
        .replace_probe_deg = probe_deg,
        .x_mm = 0.0,
        .y_mm = offset_mm,
        .theta_rad = 0.0,
        .reader_dx_mm = reader_dx_mm,
        .left_gain = left_gain,
        .right_gain = right_gain,
    };
    route_sample_sensors(ir_timer_at(0U), true);
    route_record_geometry();
    if (0 == setjmp(g_finished))
    {
        line_task_run();
    }
    if (b_plain)
    {
        route_check_plain();
    }
    else
    {
        route_check_junction();
    }
    return EXIT_SUCCESS;
}

int main(int argc, char * argv[])
{
    for (size_t i = 0U; (argc >= 2) && (i < (sizeof(g_route_cases) / sizeof(g_route_cases[0]))); i++)
    {
        if (0 == strcmp(argv[1], g_route_cases[i].p_name))
        {
            return run_route_case(&g_route_cases[i], argc, argv);
        }
    }
    if (argc == 2 && strncmp(argv[1], "startup-check-", 14U) == 0)
    {
        g_start_check = true;
        g_start_airborne = strcmp(argv[1], "startup-check-airborne") == 0;
        g_start_bad_imu = strcmp(argv[1], "startup-check-no-imu") == 0;
        g_start_late_imu = strcmp(argv[1], "startup-check-late-imu") == 0;
        g_start_no_wheel = strcmp(argv[1], "startup-check-no-wheel") == 0;
        g_start_noisy_baseline = strcmp(argv[1], "startup-check-noisy-baseline") == 0;
        g_start_no_clearance = strcmp(argv[1], "startup-check-blocked") == 0;
        g_start_clearance_lost = strcmp(argv[1], "startup-check-clearance-lost") == 0;
        if (setjmp(g_finished) == 0) line_task_run();
        bool const passes = !g_start_airborne && !g_start_bad_imu && !g_start_no_clearance &&
                            !g_start_late_imu && !g_start_no_wheel && !g_start_clearance_lost &&
                            !g_start_noisy_baseline;
        assert(g_start_probe_seen == !g_start_no_clearance);
        assert(g_start_follow_seen == passes && g_start_failures == (passes ? 0U : 1U));
        if (!passes) assert(g_left_target == 0 && g_right_target == 0);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && (strcmp(argv[1], "pose-clear") == 0 || strcmp(argv[1], "pose-near") == 0 ||
                      strcmp(argv[1], "pose-during-turn") == 0))
    {
        g_pose_return = true;
        g_turn_case = strcmp(argv[1], "pose-during-turn") == 0;
        g_pose_obstacle = strcmp(argv[1], "pose-near") == 0;
        if (setjmp(g_finished) == 0) line_task_run();
        if (g_turn_case)
        {
            assert(g_began_turn && g_turn_abort_tick == MOTION_RESTORED_TICK);
        }
        else
        {
            assert(g_motion_approached);
            assert(g_pose_obstacle ? g_side_scan : g_motion_resumed);
        }
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "invalid-launch") == 0)
    {
        g_launch_glitch = true;
        if (setjmp(g_finished) == 0) line_task_run();
        assert(g_side_scan);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && (!strcmp(argv[1], "junction-left-pause") || !strcmp(argv[1], "junction-right-pause")))
    {
        g_junction_pause = g_route_hold = g_route_pending = true;
        g_pause_right = !strcmp(argv[1], "junction-right-pause");
        if (setjmp(g_finished) == 0) line_task_run();
        assert(g_began_turn && g_route_pending);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && !strcmp(argv[1], "turn-rise"))
    {
        g_turn_case = g_rise_case = true;
        if (setjmp(g_finished) == 0) line_task_run();
        assert(g_began_turn);
        assert(g_rise_first == DRIVE_START_MM_S * UM_PER_MM);
        assert(g_rise_max_step <= DRIVE_ACCEL_MM_S2 * (int32_t)TICK_MS);
        assert(g_rise_last > g_rise_first);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && !strcmp(argv[1], "junction-cancel"))
    {
        g_junction_pause = g_route_hold = g_route_pending = g_route_cancel = true;
        if (setjmp(g_finished) == 0) line_task_run();
        assert(!g_route_pending && !g_began_turn);
        assert(g_probe_reports == 1U);
        assert(g_probe_bits == (((uint16_t)LINE_TURN_KIND_LEFT << JUNCTION_TURN_SHIFT) | TELEMETRY_JUNCTION_PROBE_ABORTED));
        return EXIT_SUCCESS;
    }
    if (argc == 2 && !strcmp(argv[1], "junction-stripe"))
    {
        g_stripe_case = g_junction_pause = g_route_hold = true;
        if (setjmp(g_finished) == 0) line_task_run();
        assert(g_stripe_reports == 1U);
        assert(g_stripe_bits == (LINE_STRIPE_LEFT | LINE_STRIPE_RIGHT | LINE_STRIPE_CORROBORATED));
        return EXIT_SUCCESS;
    }
    if (argc == 2 && (strcmp(argv[1], "pending-obstacle") == 0 || strcmp(argv[1], "pending-stale") == 0))
    {
        g_route_hold = g_route_pending = true;
        g_turn_obstacle = strcmp(argv[1], "pending-obstacle") == 0;
        g_ir_capture_case = !g_turn_obstacle;
        if (setjmp(g_finished) == 0) line_task_run();
        assert(g_route_pending && !g_began_turn);
        return EXIT_SUCCESS;
    }
    bool b_left;
    int32_t flat;
    int32_t flat_sum;
    int32_t climb;
    int16_t roll;

    if (argc == 2 && strcmp(argv[1], "probe-solid") == 0)
    {
        line_route_t route = {0};
        line_targets_t targets;
        assert(!line_route_observe(&route, LINE_TURN_KIND_LEFT, true, false, true, 0, 0));
        assert(!line_route_observe(&route, LINE_TURN_KIND_LEFT, false, false, false, 26000, 26000));
        assert(line_route_observe(&route, LINE_TURN_KIND_LEFT, false, true, false, 140000, 140000));
        for (int32_t travel = 0; travel <= 16000; travel += 1000)
        {
            assert(line_route_probe(&route, travel, -travel, true, &targets) == LINE_TURN_TURNING);
        }
        for (int32_t travel = -16000; travel <= 120000; travel += 1000)
        {
            assert(line_route_probe(&route, -travel, travel, true, &targets) == LINE_TURN_TURNING);
        }
        assert(targets.left_um_s > 0 && targets.right_um_s < 0);
        for (int32_t travel = 120000; travel > 0; travel -= 1000)
        {
            assert(line_route_probe(&route, -travel, travel, true, &targets) == LINE_TURN_TURNING);
        }
        assert(line_route_probe(&route, 0, 0, true, &targets) == LINE_TURN_NO_BRANCH);
        assert(targets.left_um_s == 0 && targets.right_um_s == 0);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && (strcmp(argv[1], "coherent-motion-snapshot") == 0 ||
                      strcmp(argv[1], "motion-timing-fault") == 0))
    {
        g_motion_snapshot_case = strcmp(argv[1], "coherent-motion-snapshot") == 0;
        g_motion_fault_case = !g_motion_snapshot_case;
        if (0 == setjmp(g_finished))
        {
            line_task_run();
        }
        assert(g_motion_approached);
        assert(g_motion_resumed);
        assert(!g_motion_moved_without_permit);
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "turn-sonar-fault") == 0)
    {
        return turn_aborts_on_sonar();
    }
    if (argc == 2 && strcmp(argv[1], "stale-line-capture") == 0)
    {
        return stale_line_capture(false);
    }
    if (argc == 2 && strcmp(argv[1], "turn-stale-line-capture") == 0)
    {
        return stale_line_capture(true);
    }
    if (argc == 2 && strcmp(argv[1], "turn-obstacle") == 0)
    {
        return turn_aborts_on_obstacle(VEHICLE_CMD_UTURN);
    }
    if (argc == 2 && strcmp(argv[1], "straight-obstacle") == 0)
    {
        return turn_aborts_on_obstacle(VEHICLE_CMD_STRAIGHT);
    }
    if (argc == 2 && strcmp(argv[1], "retry-events") == 0)
    {
        return line_retry_events();
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-hold") == 0)
    {
        return bounded_course_hold(false);
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-box-removed") == 0)
    {
        return bounded_course_box_removed(false);
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-box-invalid-rest") == 0)
    {
        return bounded_course_box_removed(true);
    }
    if (argc == 2 && (strcmp(argv[1], "bounded-course-probe-left") == 0 ||
                      strcmp(argv[1], "bounded-course-probe-right") == 0))
    {
        g_course_probe_side = strcmp(argv[1], "bounded-course-probe-left") == 0 ? 1 : -1;
        return bounded_course_hold(false);
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-clear-distance") == 0)
    {
        return bounded_course_hold(true);
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-cutoff") == 0)
    {
        return bounded_course_cutoff();
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-ping-gap") == 0)
    {
        g_course_missed_ping = true;
        return bounded_course_cutoff();
    }
    if (argc == 2 && strcmp(argv[1], "bounded-course-slow-clear") == 0)
    {
        g_course_slow_clear = true;
        return bounded_course_hold(false);
    }
    if (argc != 2 || (strcmp(argv[1], "left") != 0 && strcmp(argv[1], "right") != 0))
    {
        (void)fprintf(stderr, "Usage: %s left|right|turn-sonar-fault|turn-obstacle|straight-obstacle|"
                      "retry-events|bounded-course-hold|"
                      "bounded-course-clear-distance|bounded-course-cutoff|bounded-course-ping-gap|"
                      "bounded-course-slow-clear|bounded-course-box-removed|coherent-motion-snapshot|"
                      "bounded-course-box-invalid-rest|stale-line-capture|turn-stale-line-capture\n",
                      argv[0]);
        return EXIT_FAILURE;
    }
    b_left = strcmp(argv[1], "left") == 0;
    roll = b_left ? -800 : 800;
    flat = run_line(b_left, 0);
    assert(g_hug_locks == 1U);
    flat_sum = g_left_target + g_right_target;
    climb = run_line(b_left, roll);
    if (climb <= flat)
    {
        (void)fprintf(stderr, "Climbing wheel demand difference %ld vs flat %ld um/s\n",
                      (long)climb, (long)flat);
        return EXIT_FAILURE;
    }
    assert(g_left_target + g_right_target == flat_sum);
    assert(run_line(b_left, (int16_t)-roll) == flat);
    g_attitude_unknown = true;
    assert(run_line(b_left, roll) == flat);
    g_attitude_unknown = false;
    g_invalid_speed = true;
    assert(run_line(b_left, roll) == flat);
    g_invalid_speed = false;
    g_stale_last_sample = true;
    assert(run_line(b_left, roll) == flat);
    return EXIT_SUCCESS;
}
