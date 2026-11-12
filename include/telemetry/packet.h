/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	packet.h
 *	Big-endian wire serializers for the telemetry packets (schema 2, D110).
 *
 *	The layout lives in protocol/telemetry.yaml. The generated header
 *	telemetry/wire_schema.h holds every offset, size, topic and code, and
 *	packet.c writes with those names only. Each packer takes a struct of
 *	engineering values and returns the wire size, or 0 for a NULL pointer
 *	(nothing is written in that case). dst has no alignment requirement and
 *	needs the packet's TELEMETRY_<NAME>_WIRE_SIZE bytes.
 *
 *	The topic is not part of the packet. The queue carries it as the first
 *	byte of a frame; see telemetry/queue.h.
 */

#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <stdint.h>

#include "platform/static_assert.h"
#include "telemetry/wire_schema.h"

/* Largest wire packet, and the largest queued frame (topic byte + packet). */
#define TELEMETRY_WIRE_MAX  TELEMETRY_PROFILE_WIRE_SIZE
#define TELEMETRY_FRAME_MAX (TELEMETRY_WIRE_MAX + 1u)

/* 5 Hz. The vehicle in one frame. */
typedef struct
{
    uint32_t odometer_mm;
    int16_t  speed_mms;
    uint16_t heading_ddeg; /* 0..3599, 0xFFFF = invalid */
    uint16_t faults;
    uint16_t seq;
    uint8_t  state;
    uint8_t  mode;
    uint8_t  line_bits;
    uint8_t  hold_mask;
} telemetry_status_t;

STATIC_ASSERT (sizeof(telemetry_status_t) == 16U, "telemetry_status_t size");

/* 5 Hz. Wheels and the motion controller. */
typedef struct
{
    int16_t  left_mms;
    int16_t  right_mms;
    int16_t  left_target_mms;
    int16_t  right_target_mms;
    int16_t  left_duty_pm;
    int16_t  right_duty_pm;
    uint16_t overruns;
    uint8_t  motion_state;
    uint8_t  motion_fault;
} telemetry_motion_t;

STATIC_ASSERT (sizeof(telemetry_motion_t) == 16U, "telemetry_motion_t size");

/* 5 Hz. IMU, hump and attitude. The wire reserved byte is written as 0, so
   the struct has one byte of padding. */
typedef struct
{
    int16_t  pitch_cdeg;
    int16_t  roll_cdeg;
    int16_t  az_mg;
    int16_t  a_fwd_mg;
    int16_t  yaw_rate_cdps;
    uint16_t amag_mg;
    uint16_t heading_ddeg;
    uint16_t hump_peak_mm;
    uint16_t run_peak_mm;
    uint8_t  hump_state;
    uint8_t  motion_class;
    uint8_t  physics_faults;
    uint8_t  flags;
    uint8_t  attitude_state;
} telemetry_physics_t;

STATIC_ASSERT (sizeof(telemetry_physics_t) == 24U, "telemetry_physics_t size");

/* 2 Hz and on change. Sonar, scan and plan. */
typedef struct
{
    uint16_t faults;
    uint16_t dist_mm;
    uint16_t closest_mm;
    uint16_t plan_width_mm;
    uint16_t plan_need_mm;
    int16_t  servo_deci;
    int16_t  body_deci;
    uint8_t  state;
    uint8_t  cause;
    uint8_t  echo_class;
    uint8_t  flags;
    uint8_t  seq;
    uint8_t  req_id;
    uint8_t  plan_status;
    uint8_t  plan_side;
    uint8_t  plan_start_deg;
    uint8_t  rescans;
} telemetry_obstacle_t;

STATIC_ASSERT (sizeof(telemetry_obstacle_t) == 24U, "telemetry_obstacle_t size");

/* 1 Hz. Counters and health. */
typedef struct
{
    uint16_t sticky_faults;
    uint16_t task_alive_mask;
    uint16_t line_lost_count;
    uint16_t line_search_mm;
    uint16_t barcode_ok;
    uint16_t barcode_bad;
    uint16_t barcode_edge_drops;
    uint16_t telem_queue_drops;
    uint16_t net_queue_drops;
    uint16_t physics_overruns;
    uint16_t telemetry_overruns;
    uint8_t  last_barcode;
    uint8_t  last_barcode_action;
    uint8_t  echo_valid;
    uint8_t  echo_open;
    uint8_t  echo_too_close;
    uint8_t  echo_no_echo;
    uint8_t  echo_glitch;
    uint8_t  echo_stuck;
    uint8_t  line_edges_per_s;
    uint8_t  line_raw_edges_per_s;
    uint16_t supervisor_passes; /* a frozen value shows a hung supervisor */
} telemetry_diag_t;

STATIC_ASSERT (sizeof(telemetry_diag_t) == 34U, "telemetry_diag_t size");

/* 1 Hz. Sent by the connection task. */
typedef struct
{
    uint32_t uptime_s;
    uint8_t  drops;
    int8_t   wifi_rssi;
    uint8_t  reset_reason;
    uint8_t  link_state;
} telemetry_heartbeat_t;

STATIC_ASSERT (sizeof(telemetry_heartbeat_t) == 8U, "telemetry_heartbeat_t size");

/* On event. */
typedef struct
{
    uint32_t odometer_mm;
    uint16_t event_id;
    int16_t  p1;
    uint16_t p2;
    uint8_t  event_type;
    uint8_t  source;
} telemetry_event_t;

STATIC_ASSERT (sizeof(telemetry_event_t) == 12U, "telemetry_event_t size");

/* Once per finished scan. */
typedef struct
{
    uint16_t slots[TELEMETRY_PROFILE_COUNT_SLOTS];
    uint16_t closest_mm;
    uint16_t width_mm;
    uint8_t  req_id;
    uint8_t  status;
    uint8_t  side;
    uint8_t  flags;
} telemetry_profile_t;

STATIC_ASSERT (sizeof(telemetry_profile_t) == 82U, "telemetry_profile_t size");

uint16_t telemetry_pack_status (uint8_t * dst, const telemetry_status_t * p_src);
uint16_t telemetry_pack_motion (uint8_t * dst, const telemetry_motion_t * p_src);
uint16_t telemetry_pack_physics (uint8_t * dst,
                                 const telemetry_physics_t * p_src);
uint16_t telemetry_pack_obstacle (uint8_t * dst,
                                  const telemetry_obstacle_t * p_src);
uint16_t telemetry_pack_diag (uint8_t * dst, const telemetry_diag_t * p_src);
uint16_t telemetry_pack_heartbeat (uint8_t * dst,
                                   const telemetry_heartbeat_t * p_src);
uint16_t telemetry_pack_event (uint8_t * dst, const telemetry_event_t * p_src);
uint16_t telemetry_pack_profile (uint8_t * dst,
                                 const telemetry_profile_t * p_src);

#endif /* TELEMETRY_PACKET_H */
