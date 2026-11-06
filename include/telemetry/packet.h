/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	packet.h
 *	Big-endian wire serializers for telemetry status, event and heartbeat
 *	packets.  Pure C, depends only on stdint.h.
 *
 *	dst must point to at least TELEMETRY_STATUS_WIRE_SIZE (8),
 *	TELEMETRY_EVENT_WIRE_SIZE (8) or TELEMETRY_HEARTBEAT_WIRE_SIZE (4)
 *	bytes, depending on the packer.  dst has no alignment requirement.
 *
 *	Each serializer returns the number of bytes written (8, 8 or 4).
 *	It returns 0 when dst is NULL, and writes nothing in that case.
 *
 *	The topic is not a pack argument.  The transport adapter selects the
 *	topic for each packet type using the TELEMETRY_TOPIC_* constants below.
 */

#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <stdint.h>

#define TELEMETRY_STATUS_WIRE_SIZE    8u
#define TELEMETRY_EVENT_WIRE_SIZE     8u
#define TELEMETRY_HEARTBEAT_WIRE_SIZE 4u
#define TELEMETRY_SYSTEM_WIRE_SIZE    16u

/* Single-character MQTT topic for each packet type. */
#define TELEMETRY_TOPIC_STATUS     's'
#define TELEMETRY_TOPIC_EVENT      'e'
#define TELEMETRY_TOPIC_HEARTBEAT  'h'
#define TELEMETRY_TOPIC_COMMAND    'c'
#define TELEMETRY_TOPIC_SYSTEM     't'

/* Status packet: speed_mms i16, dist_mm u16, heading_deg u8, state u8,
   faults u16. */
uint16_t telemetry_pack_status (uint8_t * dst, int16_t speed_mms,
                                uint16_t dist_mm, uint8_t heading,
                                uint8_t state, uint16_t faults);

/* Event packet: event_id u16, event_type u8, param1 i16, param2 u16.
   The reserved trailing byte is written as zero. */
uint16_t telemetry_pack_event (uint8_t * dst, uint16_t event_id,
                               uint8_t type, int16_t p1, uint16_t p2);

/* Heartbeat packet: uptime_s u16, drop_count u8, wifi_rssi i8. */
uint16_t telemetry_pack_heartbeat (uint8_t * dst, uint16_t uptime_s,
                                   uint8_t drops, int8_t rssi);

/* Buddy 4 and Buddy 5 state. Offsets match protocol/telemetry.yaml. */
typedef struct
{
    uint8_t  obstacle_state;
    uint8_t  obstacle_cause;
    uint8_t  obstacle_faults;
    uint8_t  echo_class;
    uint16_t obstacle_dist_mm;
    uint8_t  obstacle_seq;
    uint8_t  obstacle_req_id;
    uint8_t  physics_state;
    uint8_t  physics_motion;
    uint8_t  physics_faults;
    uint8_t  physics_flags; /* bit0 active, bit1 finished, bit2 height_valid */
    int16_t  pitch_cdeg;
    uint16_t hump_peak_mm;
} telemetry_system_t;

/* System packet: obstacle state/cause/faults/class/dist/seq/req_id,
   physics state/motion/faults/flags, pitch and hump peak. */
uint16_t telemetry_pack_system (uint8_t * dst,
                                const telemetry_system_t * p_system);

#endif /* TELEMETRY_PACKET_H */
