/*
 *	assemble.c
 *	Fill telemetry packet structs from subsystem snapshots. See assemble.h.
 */

#include <stdint.h>

#include "telemetry/assemble.h"

/* No <stddef.h> here: the kernel headers define NULL and their own size_t. */

#define UM_PER_MM          1000
#define WHEEL_COUNT        2
#define HOLD_MASK_BYTE     0xFFu

/* physics_flags bit positions (protocol/telemetry.yaml). */
#define PHYSICS_FLAG_ACTIVE      (1u << TELEMETRY_PHYSICS_FLAGS_BIT_HUMP_ACTIVE)
#define PHYSICS_FLAG_FINISHED    (1u << TELEMETRY_PHYSICS_FLAGS_BIT_FINISHED)
#define PHYSICS_FLAG_HEIGHT_OK   (1u << TELEMETRY_PHYSICS_FLAGS_BIT_HEIGHT_VALID)
#define PHYSICS_FLAG_CALIBRATED  (1u << TELEMETRY_PHYSICS_FLAGS_BIT_CALIBRATED)

static int16_t
clamp_i16 (int64_t value)
{
    int64_t out = value;

    if (out > INT16_MAX)
    {
        out = INT16_MAX;
    }
    if (out < INT16_MIN)
    {
        out = INT16_MIN;
    }
    return (int16_t) out;
}

static uint16_t
clamp_u16 (uint32_t value)
{
    return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t) value;
}

/* um/s to mm/s, truncated toward zero. */
static int16_t
um_to_mm_i16 (int64_t um)
{
    return clamp_i16 (um / UM_PER_MM);
}

uint32_t
telemetry_odometer_mm (const motion_feedback_t * p_feedback)
{
    int64_t mean_um = 0;
    int64_t mm      = 0;

    if (p_feedback == 0)
    {
        return 0u;
    }
    mean_um = (p_feedback->left_displacement_um +
               p_feedback->right_displacement_um) / WHEEL_COUNT;
    mm = mean_um / UM_PER_MM;
    if (mm < 0)
    {
        return 0u;
    }
    return (mm > (int64_t) UINT32_MAX) ? UINT32_MAX : (uint32_t) mm;
}

static uint8_t
run_state_of (const motion_status_t * p_motion)
{
    uint8_t state = TELEMETRY_RUN_STATE_INIT;

    if (p_motion != 0)
    {
        if (p_motion->stop_requested ||
            ((p_motion->hold_mask & MOTION_HOLD_OPERATOR) != 0u) ||
            (p_motion->state == MOTION_STATE_STOPPED) ||
            (p_motion->state == MOTION_STATE_FAULT))
        {
            state = TELEMETRY_RUN_STATE_STOPPED;
        }
        else if ((p_motion->state == MOTION_STATE_RUNNING) ||
                 (p_motion->state == MOTION_STATE_REVERSING))
        {
            state = TELEMETRY_RUN_STATE_RUNNING;
        }
        else
        {
            state = TELEMETRY_RUN_STATE_READY;
        }
    }
    return state;
}

/* What the car is doing, from the facts the root can see today. Lifted and
   overturned come from the fault word once the attitude module sets them. */
static uint8_t
mode_of (uint8_t run_state, const obstacle_status_t * p_obstacle,
         uint16_t faults, bool b_turning, uint32_t hold_mask)
{
    uint8_t mode = TELEMETRY_VEHICLE_MODE_FOLLOW;
    uint8_t obstacle_state = 0u;

    if (p_obstacle != 0)
    {
        obstacle_state = p_obstacle->state;
    }

    if ((faults & (1u << TELEMETRY_FAULT_BIT_OVERTURNED)) != 0u)
    {
        mode = TELEMETRY_VEHICLE_MODE_OVERTURNED;
    }
    else if ((faults & (1u << TELEMETRY_FAULT_BIT_LIFTED)) != 0u)
    {
        mode = TELEMETRY_VEHICLE_MODE_LIFTED;
    }
    else if ((hold_mask & MOTION_HOLD_LIFT) != 0u)
    {
        mode = TELEMETRY_VEHICLE_MODE_LIFTED;
    }
    else if ((hold_mask & MOTION_HOLD_BARCODE) != 0u)
    {
        mode = TELEMETRY_VEHICLE_MODE_HOLD_BARCODE;
    }
    else if (run_state != TELEMETRY_RUN_STATE_RUNNING)
    {
        mode = TELEMETRY_VEHICLE_MODE_STOPPED;
    }
    else if ((obstacle_state == TELEMETRY_OBSTACLE_STATE_BYPASS) ||
             (obstacle_state == TELEMETRY_OBSTACLE_STATE_REACQUIRE))
    {
        mode = TELEMETRY_VEHICLE_MODE_BYPASS;
    }
    else if ((obstacle_state == TELEMETRY_OBSTACLE_STATE_WAIT_STOP) ||
             (obstacle_state == TELEMETRY_OBSTACLE_STATE_SCANNING) ||
             (obstacle_state == TELEMETRY_OBSTACLE_STATE_PLAN))
    {
        mode = TELEMETRY_VEHICLE_MODE_HOLD;
    }
    else if (b_turning)
    {
        mode = TELEMETRY_VEHICLE_MODE_TURN;
    }
    else if ((faults & (1u << TELEMETRY_FAULT_BIT_LINE_LOST)) != 0u)
    {
        mode = TELEMETRY_VEHICLE_MODE_SEARCH;
    }
    else
    {
        mode = TELEMETRY_VEHICLE_MODE_FOLLOW;
    }
    return mode;
}

void
telemetry_assemble_status (telemetry_status_t * p_out,
                           const motion_status_t * p_motion,
                           const motion_feedback_t * p_feedback,
                           const physics_status_t * p_physics,
                           const obstacle_status_t * p_obstacle,
                           uint16_t faults, uint16_t seq, uint8_t line_bits,
                           bool b_turning)
{
    uint16_t level = faults;

    if (p_out == 0)
    {
        return;
    }
    p_out->speed_mms    = 0;
    p_out->odometer_mm  = telemetry_odometer_mm (p_feedback);
    p_out->heading_ddeg = TELEMETRY_HEADING_INVALID;
    p_out->hold_mask    = 0u;

    if ((p_feedback != 0) && p_feedback->fresh)
    {
        p_out->speed_mms = um_to_mm_i16 (
            ((int64_t) p_feedback->left_speed_um_s +
             (int64_t) p_feedback->right_speed_um_s) /
            WHEEL_COUNT);
    }
    if ((p_physics != 0) && p_physics->heading_valid)
    {
        p_out->heading_ddeg = p_physics->heading_ddeg;
    }
    if (p_motion != 0)
    {
        p_out->hold_mask = (uint8_t) (p_motion->hold_mask & HOLD_MASK_BYTE);
        if (p_motion->stop_requested)
        {
            level = (uint16_t) (level | (1u << TELEMETRY_FAULT_BIT_ESTOP_LATCHED));
        }
    }
    p_out->state     = run_state_of (p_motion);
    p_out->mode      = mode_of (p_out->state, p_obstacle, level, b_turning, p_motion != 0 ? p_motion->hold_mask : 0u);
    p_out->faults    = level;
    p_out->seq       = seq;
    p_out->line_bits = line_bits;
}

void
telemetry_assemble_motion (telemetry_motion_t * p_out,
                           const motion_status_t * p_motion,
                           const motion_feedback_t * p_feedback,
                           const motion_diag_t * p_diag)
{
    if (p_out == 0)
    {
        return;
    }
    p_out->left_mms         = 0;
    p_out->right_mms        = 0;
    p_out->left_target_mms  = 0;
    p_out->right_target_mms = 0;
    p_out->left_duty_pm     = 0;
    p_out->right_duty_pm    = 0;
    p_out->overruns         = 0u;
    p_out->motion_state     = 0u;
    p_out->motion_fault     = 0u;

    if ((p_feedback != 0) && p_feedback->fresh)
    {
        p_out->left_mms  = um_to_mm_i16 (p_feedback->left_speed_um_s);
        p_out->right_mms = um_to_mm_i16 (p_feedback->right_speed_um_s);
    }
    if (p_diag != 0)
    {
        p_out->left_target_mms  = um_to_mm_i16 (p_diag->left_target_um_s);
        p_out->right_target_mms = um_to_mm_i16 (p_diag->right_target_um_s);
        p_out->left_duty_pm     = p_diag->left_duty_permille;
        p_out->right_duty_pm    = p_diag->right_duty_permille;
        p_out->overruns         = clamp_u16 (p_diag->overruns);
    }
    if (p_motion != 0)
    {
        p_out->motion_state = p_motion->state;
        p_out->motion_fault = p_motion->fault;
    }
}

void
telemetry_assemble_physics (telemetry_physics_t * p_out,
                            const physics_status_t * p_physics)
{
    if (p_out == 0)
    {
        return;
    }
    p_out->pitch_cdeg     = 0;
    p_out->roll_cdeg      = 0;
    p_out->az_mg          = 0;
    p_out->a_fwd_mg       = 0;
    p_out->yaw_rate_cdps  = 0;
    p_out->amag_mg        = 0u;
    p_out->heading_ddeg   = TELEMETRY_HEADING_INVALID;
    p_out->hump_peak_mm   = 0u;
    p_out->run_peak_mm    = 0u;
    p_out->hump_state     = 0u;
    p_out->motion_class   = 0u;
    p_out->physics_faults = 0u;
    p_out->flags          = 0u;
    p_out->attitude_state = 0u;

    if ((p_physics == 0) || !p_physics->ready)
    {
        return;
    }
    p_out->pitch_cdeg     = p_physics->pitch_cdeg;
    p_out->a_fwd_mg       = p_physics->a_fwd_mg;
    p_out->yaw_rate_cdps  = p_physics->yaw_rate_cdps;
    p_out->hump_peak_mm   = p_physics->hump_peak_mm;
    p_out->run_peak_mm    = p_physics->run_peak_mm;
    p_out->hump_state     = p_physics->state;
    p_out->motion_class   = p_physics->motion;
    p_out->physics_faults = p_physics->faults;
    if (p_physics->heading_valid)
    {
        p_out->heading_ddeg = p_physics->heading_ddeg;
    }
    if (p_physics->hump_active)
    {
        p_out->flags |= PHYSICS_FLAG_ACTIVE;
    }
    if (p_physics->finished)
    {
        p_out->flags |= PHYSICS_FLAG_FINISHED;
    }
    if (p_physics->height_valid)
    {
        p_out->flags |= PHYSICS_FLAG_HEIGHT_OK;
    }
    if (p_physics->calibrated)
    {
        p_out->flags |= PHYSICS_FLAG_CALIBRATED;
    }
}

void
telemetry_assemble_obstacle (telemetry_obstacle_t * p_out,
                             const obstacle_status_t * p_obstacle,
                             const obstacle_plan_summary_t * p_plan)
{
    if (p_out == 0)
    {
        return;
    }
    p_out->faults         = 0u;
    p_out->dist_mm        = 0u;
    p_out->closest_mm     = 0u;
    p_out->plan_width_mm  = 0u;
    p_out->plan_need_mm   = 0u;
    p_out->servo_deci     = 0;
    p_out->body_deci      = 0;
    p_out->state          = 0u;
    p_out->cause          = 0u;
    p_out->echo_class     = 0u;
    p_out->flags          = 0u;
    p_out->seq            = 0u;
    p_out->req_id         = 0u;
    p_out->plan_status    = 0u;
    p_out->plan_side      = 0u;
    p_out->plan_start_deg = 0u;
    p_out->rescans        = 0u;

    if (p_obstacle == 0)
    {
        return;
    }
    p_out->state      = p_obstacle->state;
    p_out->cause      = p_obstacle->cause;
    p_out->faults     = p_obstacle->faults;
    p_out->echo_class = p_obstacle->echo_class;
    p_out->dist_mm    = p_obstacle->distance_mm;
    p_out->seq        = p_obstacle->seq;
    p_out->req_id     = p_obstacle->req_id;
    p_out->servo_deci = p_obstacle->servo_deci;
    p_out->body_deci  = p_obstacle->body_deci;
    p_out->flags      = p_obstacle->flags;

    if (p_plan != 0)
    {
        p_out->closest_mm     = p_plan->closest_mm;
        p_out->plan_width_mm  = p_plan->width_mm;
        p_out->plan_need_mm   = p_plan->need_mm;
        p_out->plan_status    = p_plan->status;
        p_out->plan_side      = p_plan->side;
        p_out->plan_start_deg = p_plan->start_deg;
    }
}
