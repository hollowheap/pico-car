/*
 *	assemble.c
 *	Fill telemetry packet structs from subsystem snapshots. See assemble.h.
 */

#include <stdint.h>

#include "comms/telemetry/assemble.h"

#include "obstacle/obstacle.h"
#include "platform/units.h"
#include "comms/telemetry/hug_word.h"

/* No <stddef.h> here: the kernel headers define NULL and their own size_t. */

#define WHEEL_COUNT        2
#define HOLD_MASK_BYTE     0xFFu
#define US_PER_MS          (1000U)
#define SAMPLE_BIT(name)   TELEMETRY_STATUS_SAMPLE_FLAGS_BIT_##name

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

static int32_t travel_mm (int64_t displacement_um)
{
    int64_t const mm = displacement_um / UM_PER_MM;

    return (mm > INT32_MAX)   ? INT32_MAX
           : (mm < INT32_MIN) ? INT32_MIN
                              : (int32_t)mm;
}

uint32_t
telemetry_odometer_mm (const motion_feedback_t * p_feedback)
{
    if (p_feedback == 0)
    {
        return 0u;
    }
    return wheel_odometer_mm (p_feedback->left_displacement_um, p_feedback->right_displacement_um);
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
         uint16_t faults, bool b_turning, uint32_t hold_mask, uint8_t hug_state)
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
    else if ((hug_state == TELEMETRY_HUG_STATE_CONTACT) || (hug_state == TELEMETRY_HUG_STATE_LOOK))
    {
        mode = TELEMETRY_VEHICLE_MODE_HOLD;
    }
    else if ((hug_state != TELEMETRY_HUG_STATE_IDLE) || (obstacle_state == TELEMETRY_OBSTACLE_STATE_BYPASS) ||
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
                           bool b_turning, const uint32_t * p_hug)
{
    uint8_t const hug_state = ((p_hug != 0) && hug_word_is_valid(p_hug))
        ? (uint8_t)hug_word_get(p_hug[0], HUG_WORD_STATE_SH, HUG_WORD_STATE_MK) : TELEMETRY_HUG_STATE_IDLE;
    uint16_t level = faults;

    if (p_out == 0)
    {
        return;
    }
    p_out->speed_mms    = 0;
    p_out->odometer_mm  = telemetry_odometer_mm (p_feedback);
    p_out->heading_ddeg = TELEMETRY_HEADING_INVALID;
    p_out->hold_mask    = 0u;
    p_out->pivot_scale_pct = ((p_physics != 0) && p_physics->ready) ? p_physics->pivot_scale_pct : 0u;

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
            /* The drive inhibit is latched while the motion task handles the brake. */
            level = (uint16_t) (level | (1u << TELEMETRY_FAULT_BIT_ESTOP_LATCHED));
        }
    }
    p_out->state     = run_state_of (p_motion);
    p_out->mode      = mode_of (p_out->state, p_obstacle, level, b_turning,
                                p_motion != 0 ? p_motion->hold_mask : 0u, hug_state);
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
    p_out->roll_cdeg      = p_physics->report_roll_cdeg;
    p_out->az_mg          = p_physics->az_mg;
    p_out->amag_mg        = p_physics->amag_mg;
    p_out->attitude_state = p_physics->attitude_state;
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
    if (p_physics->attitude_sample_valid)
    {
        p_out->flags |= PHYSICS_FLAG_CALIBRATED;
    }
}

void telemetry_assemble_bundle_details (
    telemetry_status_bundle_t * p_out, uint32_t sample_us,
    const motion_feedback_t * p_feedback, const physics_status_t * p_physics,
    const obstacle_forward_sample_t * p_forward)
{
    p_out->device_sample_us   = sample_us;
    p_out->left_travel_mm     = 0;
    p_out->right_travel_mm    = 0;
    p_out->wheel_age_us       = UINT16_MAX;
    p_out->sample_flags       = 0U;
    p_out->forward_dist_mm    = UINT16_MAX;
    p_out->forward_echo_class = UINT8_MAX;
    p_out->forward_age_ms     = UINT8_MAX;

    if (p_feedback != NULL)
    {
        uint32_t const age_us = sample_us - p_feedback->timestamp_us;
        uint32_t const flags  = ((uint32_t)p_feedback->speed_valid[0]
                                << SAMPLE_BIT(LEFT_SPEED_VALID)) |
                               ((uint32_t)p_feedback->speed_valid[1]
                                << SAMPLE_BIT(RIGHT_SPEED_VALID)) |
                               ((uint32_t)p_feedback->odometry_valid[0]
                                << SAMPLE_BIT(LEFT_TRAVEL_VALID)) |
                               ((uint32_t)p_feedback->odometry_valid[1]
                                << SAMPLE_BIT(RIGHT_TRAVEL_VALID));
        bool const b_fresh =
            p_feedback->fresh && (age_us <= MOTION_FEEDBACK_MAX_AGE_US);

        p_out->left_travel_mm  = travel_mm(p_feedback->left_displacement_um);
        p_out->right_travel_mm = travel_mm(p_feedback->right_displacement_um);
        p_out->sample_flags =
            (uint8_t)(flags | ((uint32_t)b_fresh << SAMPLE_BIT(WHEEL_FRESH)));
        if ((p_feedback->fresh || (flags != 0U)) && (age_us < UINT16_MAX))
        {
            p_out->wheel_age_us = (uint16_t)age_us;
        }
    }
    if ((p_physics != NULL) && p_physics->attitude_sample_valid)
    {
        p_out->sample_flags |= (1U << SAMPLE_BIT(ATTITUDE_VALID));
    }
    if (p_forward != NULL)
    {
        uint32_t const age_us = sample_us - p_forward->trigger_us;

        if (age_us < OBSTACLE_FORWARD_MAX_AGE_US)
        {
            p_out->forward_age_ms     = (uint8_t)(age_us / US_PER_MS);
            p_out->forward_echo_class = p_forward->echo_class;
            if ((p_forward->echo_class == OBSTACLE_CLASS_VALID) ||
                (p_forward->echo_class == OBSTACLE_CLASS_TOO_CLOSE))
            {
                p_out->forward_dist_mm = p_forward->distance_mm;
            }
        }
    }
}

static void
fill_hug (telemetry_obstacle_t * p_out, const uint32_t * p_hug)
{
    if ((p_hug != 0) && hug_word_is_valid(p_hug))
    {
        uint32_t const state = hug_word_get(p_hug[0], HUG_WORD_STATE_SH, HUG_WORD_STATE_MK);

        p_out->flags |= (uint8_t)(hug_word_get(p_hug[1], HUG_WORD_WALL_SH, 1U) << TELEMETRY_OBSTACLE_FLAGS_BIT_WALL);
        p_out->hug_state  = (uint8_t)state;
        p_out->hug_seq    = (uint8_t)hug_word_get(p_hug[0], HUG_WORD_SEQ_SH, HUG_WORD_SEQ_MK);
        p_out->hug_side   = (state == TELEMETRY_HUG_STATE_IDLE)
                                ? (uint8_t)TELEMETRY_HUG_SIDE_NONE
                                : (uint8_t)hug_word_get(p_hug[0], HUG_WORD_SIDE_SH,
                                                        HUG_WORD_SIDE_MK);
        p_out->width_mm   = (uint16_t)hug_word_get(p_hug[0], HUG_WORD_WIDTH_SH, HUG_WORD_WIDTH_MK);
        p_out->breadth_mm = (uint16_t)hug_word_get(p_hug[0], HUG_WORD_BREADTH_SH,
                                                   HUG_WORD_BREADTH_MK);
        p_out->face_mm    = (uint16_t)hug_word_get(p_hug[1], HUG_WORD_FACE_SH, HUG_WORD_FACE_MK);
        p_out->body_deci  = hug_word_heading_deci(p_hug);
    }
}

void
telemetry_assemble_obstacle (telemetry_obstacle_t * p_out,
                             const obstacle_status_t * p_obstacle,
                             const obstacle_plan_summary_t * p_plan,
                             uint32_t ping,
                             const uint32_t * p_hug)
{
    if (p_out == 0)
    {
        return;
    }
    p_out->faults         = 0u;
    p_out->dist_mm        = 0u;
    p_out->closest_mm     = 0u;
    p_out->width_mm       = 0u;
    p_out->breadth_mm     = 0u;
    p_out->face_mm        = 0u;
    p_out->servo_deci     = 0;
    p_out->body_deci      = 0;
    p_out->state          = 0u;
    p_out->cause          = 0u;
    p_out->echo_class     = TELEMETRY_ECHO_CLASS_NO_ECHO;
    p_out->flags          = 0u;
    p_out->seq            = 0u;
    p_out->hug_state      = TELEMETRY_HUG_STATE_IDLE;
    p_out->hug_side       = TELEMETRY_HUG_SIDE_NONE;
    p_out->hug_seq        = 0u;

    if (ping != 0u)
    {
        p_out->echo_class = (uint8_t)((ping >> OBSTACLE_PING_CLASS_SH) & OBSTACLE_PING_CLASS_MK);
        p_out->dist_mm    = obstacle_pack_ping_mm(ping);
        p_out->seq        = obstacle_pack_ping_seq(ping);
        p_out->servo_deci = (int16_t)obstacle_pack_ping_servo(ping);
    }

    if (p_obstacle == 0)
    {
        fill_hug(p_out, p_hug);
        return;
    }
    p_out->state      = p_obstacle->state;
    p_out->cause      = p_obstacle->cause;
    p_out->faults     = p_obstacle->faults;
    p_out->body_deci  = p_obstacle->body_deci;
    p_out->flags      = p_obstacle->flags;

    if (p_plan != 0)
    {
        p_out->closest_mm = p_plan->closest_mm;
    }
    fill_hug(p_out, p_hug);
}

void
telemetry_assemble_profile (telemetry_profile_t * p_out,
                            const obstacle_plan_summary_t * p_plan)
{
    if ((p_out == 0) || (p_plan == 0))
    {
        return;
    }
    p_out->req_id     = p_plan->req_id;
    p_out->status     = p_plan->status;
    p_out->side       = p_plan->side;
    p_out->flags      = p_plan->b_wall ? (uint8_t)(1u << TELEMETRY_PROFILE_FLAGS_BIT_WALL) : 0u;
    p_out->closest_mm = p_plan->closest_mm;
    p_out->width_mm   = p_plan->width_mm;
}
