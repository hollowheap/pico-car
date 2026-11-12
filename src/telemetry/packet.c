/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	packet.c
 *	Big-endian wire serializers. See packet.h. Every offset and size comes
 *	from the generated telemetry/wire_schema.h, so the layout in
 *	protocol/telemetry.yaml is the only place it is written down.
 */

#include <stddef.h>
#include <stdint.h>

#include "telemetry/packet.h"

#define WIRE_BYTE_BITS 8u
#define WIRE_BYTE_MASK 0xFFu
#define WIRE_U16_BYTES 2u
#define WIRE_U32_BYTES 4u

/* Write `value` big-endian, `width` bytes wide, at dst[off]. */
static void
put_be (uint8_t * dst, uint32_t off, uint32_t value, uint32_t width)
{
    for (uint32_t i = 0u; i < width; i++)
    {
        const uint32_t shift = (width - 1u - i) * WIRE_BYTE_BITS;

        dst[off + i] = (uint8_t) ((value >> shift) & WIRE_BYTE_MASK);
    }
}

static void
put_u8 (uint8_t * dst, uint32_t off, uint8_t value)
{
    dst[off] = value;
}

static void
put_u16 (uint8_t * dst, uint32_t off, uint16_t value)
{
    put_be (dst, off, value, WIRE_U16_BYTES);
}

static void
put_i16 (uint8_t * dst, uint32_t off, int16_t value)
{
    put_be (dst, off, (uint16_t) value, WIRE_U16_BYTES);
}

static void
put_u32 (uint8_t * dst, uint32_t off, uint32_t value)
{
    put_be (dst, off, value, WIRE_U32_BYTES);
}

uint16_t
telemetry_pack_status (uint8_t * dst, const telemetry_status_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_i16 (dst, TELEMETRY_STATUS_OFF_SPEED_MMS, p_src->speed_mms);
    put_u32 (dst, TELEMETRY_STATUS_OFF_ODOMETER_MM, p_src->odometer_mm);
    put_u16 (dst, TELEMETRY_STATUS_OFF_HEADING_DDEG, p_src->heading_ddeg);
    put_u8 (dst, TELEMETRY_STATUS_OFF_STATE, p_src->state);
    put_u8 (dst, TELEMETRY_STATUS_OFF_MODE, p_src->mode);
    put_u16 (dst, TELEMETRY_STATUS_OFF_FAULTS, p_src->faults);
    put_u16 (dst, TELEMETRY_STATUS_OFF_SEQ, p_src->seq);
    put_u8 (dst, TELEMETRY_STATUS_OFF_LINE_BITS, p_src->line_bits);
    put_u8 (dst, TELEMETRY_STATUS_OFF_HOLD_MASK, p_src->hold_mask);

    return TELEMETRY_STATUS_WIRE_SIZE;
}

uint16_t
telemetry_pack_motion (uint8_t * dst, const telemetry_motion_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_i16 (dst, TELEMETRY_MOTION_OFF_LEFT_MMS, p_src->left_mms);
    put_i16 (dst, TELEMETRY_MOTION_OFF_RIGHT_MMS, p_src->right_mms);
    put_i16 (dst, TELEMETRY_MOTION_OFF_LEFT_TARGET_MMS, p_src->left_target_mms);
    put_i16 (dst, TELEMETRY_MOTION_OFF_RIGHT_TARGET_MMS,
             p_src->right_target_mms);
    put_i16 (dst, TELEMETRY_MOTION_OFF_LEFT_DUTY_PM, p_src->left_duty_pm);
    put_i16 (dst, TELEMETRY_MOTION_OFF_RIGHT_DUTY_PM, p_src->right_duty_pm);
    put_u8 (dst, TELEMETRY_MOTION_OFF_MOTION_STATE, p_src->motion_state);
    put_u8 (dst, TELEMETRY_MOTION_OFF_MOTION_FAULT, p_src->motion_fault);
    put_u16 (dst, TELEMETRY_MOTION_OFF_OVERRUNS, p_src->overruns);

    return TELEMETRY_MOTION_WIRE_SIZE;
}

uint16_t
telemetry_pack_physics (uint8_t * dst, const telemetry_physics_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_i16 (dst, TELEMETRY_PHYSICS_OFF_PITCH_CDEG, p_src->pitch_cdeg);
    put_i16 (dst, TELEMETRY_PHYSICS_OFF_ROLL_CDEG, p_src->roll_cdeg);
    put_i16 (dst, TELEMETRY_PHYSICS_OFF_AZ_MG, p_src->az_mg);
    put_u16 (dst, TELEMETRY_PHYSICS_OFF_AMAG_MG, p_src->amag_mg);
    put_i16 (dst, TELEMETRY_PHYSICS_OFF_A_FWD_MG, p_src->a_fwd_mg);
    put_i16 (dst, TELEMETRY_PHYSICS_OFF_YAW_RATE_CDPS, p_src->yaw_rate_cdps);
    put_u16 (dst, TELEMETRY_PHYSICS_OFF_HEADING_DDEG, p_src->heading_ddeg);
    put_u16 (dst, TELEMETRY_PHYSICS_OFF_HUMP_PEAK_MM, p_src->hump_peak_mm);
    put_u16 (dst, TELEMETRY_PHYSICS_OFF_RUN_PEAK_MM, p_src->run_peak_mm);
    put_u8 (dst, TELEMETRY_PHYSICS_OFF_HUMP_STATE, p_src->hump_state);
    put_u8 (dst, TELEMETRY_PHYSICS_OFF_MOTION_CLASS, p_src->motion_class);
    put_u8 (dst, TELEMETRY_PHYSICS_OFF_PHYSICS_FAULTS, p_src->physics_faults);
    put_u8 (dst, TELEMETRY_PHYSICS_OFF_FLAGS, p_src->flags);
    put_u8 (dst, TELEMETRY_PHYSICS_OFF_ATTITUDE_STATE, p_src->attitude_state);
    put_u8 (dst, TELEMETRY_PHYSICS_OFF_RESERVED, 0u);

    return TELEMETRY_PHYSICS_WIRE_SIZE;
}

uint16_t
telemetry_pack_obstacle (uint8_t * dst, const telemetry_obstacle_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_STATE, p_src->state);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_CAUSE, p_src->cause);
    put_u16 (dst, TELEMETRY_OBSTACLE_OFF_FAULTS, p_src->faults);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_ECHO_CLASS, p_src->echo_class);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_FLAGS, p_src->flags);
    put_u16 (dst, TELEMETRY_OBSTACLE_OFF_DIST_MM, p_src->dist_mm);
    put_u16 (dst, TELEMETRY_OBSTACLE_OFF_CLOSEST_MM, p_src->closest_mm);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_SEQ, p_src->seq);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_REQ_ID, p_src->req_id);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_PLAN_STATUS, p_src->plan_status);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_PLAN_SIDE, p_src->plan_side);
    put_u16 (dst, TELEMETRY_OBSTACLE_OFF_PLAN_WIDTH_MM, p_src->plan_width_mm);
    put_u16 (dst, TELEMETRY_OBSTACLE_OFF_PLAN_NEED_MM, p_src->plan_need_mm);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_PLAN_START_DEG, p_src->plan_start_deg);
    put_u8 (dst, TELEMETRY_OBSTACLE_OFF_RESCANS, p_src->rescans);
    put_i16 (dst, TELEMETRY_OBSTACLE_OFF_SERVO_DECI, p_src->servo_deci);
    put_i16 (dst, TELEMETRY_OBSTACLE_OFF_BODY_DECI, p_src->body_deci);

    return TELEMETRY_OBSTACLE_WIRE_SIZE;
}

uint16_t
telemetry_pack_diag (uint8_t * dst, const telemetry_diag_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_u16 (dst, TELEMETRY_DIAG_OFF_STICKY_FAULTS, p_src->sticky_faults);
    put_u16 (dst, TELEMETRY_DIAG_OFF_TASK_ALIVE_MASK, p_src->task_alive_mask);
    put_u16 (dst, TELEMETRY_DIAG_OFF_LINE_LOST_COUNT, p_src->line_lost_count);
    put_u16 (dst, TELEMETRY_DIAG_OFF_LINE_SEARCH_MM, p_src->line_search_mm);
    put_u16 (dst, TELEMETRY_DIAG_OFF_BARCODE_OK, p_src->barcode_ok);
    put_u16 (dst, TELEMETRY_DIAG_OFF_BARCODE_BAD, p_src->barcode_bad);
    put_u16 (dst, TELEMETRY_DIAG_OFF_BARCODE_EDGE_DROPS,
             p_src->barcode_edge_drops);
    put_u8 (dst, TELEMETRY_DIAG_OFF_LAST_BARCODE, p_src->last_barcode);
    put_u8 (dst, TELEMETRY_DIAG_OFF_LAST_BARCODE_ACTION,
            p_src->last_barcode_action);
    put_u16 (dst, TELEMETRY_DIAG_OFF_TELEM_QUEUE_DROPS, p_src->telem_queue_drops);
    put_u16 (dst, TELEMETRY_DIAG_OFF_NET_QUEUE_DROPS, p_src->net_queue_drops);
    put_u16 (dst, TELEMETRY_DIAG_OFF_PHYSICS_OVERRUNS, p_src->physics_overruns);
    put_u16 (dst, TELEMETRY_DIAG_OFF_TELEMETRY_OVERRUNS,
             p_src->telemetry_overruns);
    put_u8 (dst, TELEMETRY_DIAG_OFF_ECHO_VALID, p_src->echo_valid);
    put_u8 (dst, TELEMETRY_DIAG_OFF_ECHO_OPEN, p_src->echo_open);
    put_u8 (dst, TELEMETRY_DIAG_OFF_ECHO_TOO_CLOSE, p_src->echo_too_close);
    put_u8 (dst, TELEMETRY_DIAG_OFF_ECHO_NO_ECHO, p_src->echo_no_echo);
    put_u8 (dst, TELEMETRY_DIAG_OFF_ECHO_GLITCH, p_src->echo_glitch);
    put_u8 (dst, TELEMETRY_DIAG_OFF_ECHO_STUCK, p_src->echo_stuck);
    put_u8 (dst, TELEMETRY_DIAG_OFF_LINE_EDGES_PER_S, p_src->line_edges_per_s);
    put_u8 (dst, TELEMETRY_DIAG_OFF_LINE_RAW_EDGES_PER_S,
            p_src->line_raw_edges_per_s);
    put_u16 (dst, TELEMETRY_DIAG_OFF_SUPERVISOR_PASSES, p_src->supervisor_passes);

    return TELEMETRY_DIAG_WIRE_SIZE;
}

uint16_t
telemetry_pack_heartbeat (uint8_t * dst, const telemetry_heartbeat_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_u32 (dst, TELEMETRY_HEARTBEAT_OFF_UPTIME_S, p_src->uptime_s);
    put_u8 (dst, TELEMETRY_HEARTBEAT_OFF_DROPS, p_src->drops);
    put_u8 (dst, TELEMETRY_HEARTBEAT_OFF_WIFI_RSSI, (uint8_t) p_src->wifi_rssi);
    put_u8 (dst, TELEMETRY_HEARTBEAT_OFF_RESET_REASON, p_src->reset_reason);
    put_u8 (dst, TELEMETRY_HEARTBEAT_OFF_LINK_STATE, p_src->link_state);

    return TELEMETRY_HEARTBEAT_WIRE_SIZE;
}

uint16_t
telemetry_pack_event (uint8_t * dst, const telemetry_event_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_u16 (dst, TELEMETRY_EVENT_OFF_EVENT_ID, p_src->event_id);
    put_u8 (dst, TELEMETRY_EVENT_OFF_EVENT_TYPE, p_src->event_type);
    put_u8 (dst, TELEMETRY_EVENT_OFF_SOURCE, p_src->source);
    put_i16 (dst, TELEMETRY_EVENT_OFF_P1, p_src->p1);
    put_u16 (dst, TELEMETRY_EVENT_OFF_P2, p_src->p2);
    put_u32 (dst, TELEMETRY_EVENT_OFF_ODOMETER_MM, p_src->odometer_mm);

    return TELEMETRY_EVENT_WIRE_SIZE;
}

uint16_t
telemetry_pack_profile (uint8_t * dst, const telemetry_profile_t * p_src)
{
    if ((dst == NULL) || (p_src == NULL))
    {
        return 0u;
    }
    put_u8 (dst, TELEMETRY_PROFILE_OFF_REQ_ID, p_src->req_id);
    put_u8 (dst, TELEMETRY_PROFILE_OFF_STATUS, p_src->status);
    put_u8 (dst, TELEMETRY_PROFILE_OFF_SIDE, p_src->side);
    put_u8 (dst, TELEMETRY_PROFILE_OFF_FLAGS, p_src->flags);
    put_u16 (dst, TELEMETRY_PROFILE_OFF_CLOSEST_MM, p_src->closest_mm);
    put_u16 (dst, TELEMETRY_PROFILE_OFF_WIDTH_MM, p_src->width_mm);
    for (uint32_t i = 0u; i < TELEMETRY_PROFILE_COUNT_SLOTS; i++)
    {
        put_u16 (dst, TELEMETRY_PROFILE_OFF_SLOTS + (i * WIRE_U16_BYTES),
                 p_src->slots[i]);
    }

    return TELEMETRY_PROFILE_WIRE_SIZE;
}
