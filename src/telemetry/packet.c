/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *	packet.c
 *	Big-endian wire serializers.  See packet.h.
 */

#include <stddef.h>
#include <stdint.h>

#include "platform/static_assert.h"
#include "telemetry/packet.h"

/* Byte offsets within each wire packet.  Each serializer indexes dst with
   these macros, so the wire layout is fixed here and needs no struct
   overlay or alignment. */

#define STATUS_OFF_SPEED_HI   0u
#define STATUS_OFF_SPEED_LO   1u
#define STATUS_OFF_DIST_HI    2u
#define STATUS_OFF_DIST_LO    3u
#define STATUS_OFF_HEADING    4u
#define STATUS_OFF_STATE      5u
#define STATUS_OFF_FAULTS_HI  6u
#define STATUS_OFF_FAULTS_LO  7u

#define EVENT_OFF_ID_HI       0u
#define EVENT_OFF_ID_LO       1u
#define EVENT_OFF_TYPE        2u
#define EVENT_OFF_P1_HI       3u
#define EVENT_OFF_P1_LO       4u
#define EVENT_OFF_P2_HI       5u
#define EVENT_OFF_P2_LO       6u
#define EVENT_OFF_RESERVED    7u

#define HEARTBEAT_OFF_UPTIME_HI 0u
#define HEARTBEAT_OFF_UPTIME_LO 1u
#define HEARTBEAT_OFF_DROPS     2u
#define HEARTBEAT_OFF_RSSI      3u

#define SYSTEM_OFF_OBSTACLE_STATE  0u
#define SYSTEM_OFF_OBSTACLE_CAUSE  1u
#define SYSTEM_OFF_OBSTACLE_FAULTS 2u
#define SYSTEM_OFF_ECHO_CLASS      3u
#define SYSTEM_OFF_OBSTACLE_DIST_HI 4u
#define SYSTEM_OFF_OBSTACLE_DIST_LO 5u
#define SYSTEM_OFF_OBSTACLE_SEQ    6u
#define SYSTEM_OFF_OBSTACLE_REQ_ID 7u
#define SYSTEM_OFF_PHYSICS_STATE   8u
#define SYSTEM_OFF_PHYSICS_MOTION  9u
#define SYSTEM_OFF_PHYSICS_FAULTS  10u
#define SYSTEM_OFF_PHYSICS_FLAGS   11u
#define SYSTEM_OFF_PITCH_HI        12u
#define SYSTEM_OFF_PITCH_LO        13u
#define SYSTEM_OFF_HUMP_HI         14u
#define SYSTEM_OFF_HUMP_LO         15u

#define WIRE_BYTE_BITS 8u

/* Each offset must be contiguous, and the last offset must be the final
   byte of its packet. */
STATIC_ASSERT (STATUS_OFF_SPEED_HI == 0u, "status offsets");
STATIC_ASSERT (STATUS_OFF_SPEED_LO == STATUS_OFF_SPEED_HI + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_DIST_HI == STATUS_OFF_SPEED_LO + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_DIST_LO == STATUS_OFF_DIST_HI + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_HEADING == STATUS_OFF_DIST_LO + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_STATE == STATUS_OFF_HEADING + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_FAULTS_HI == STATUS_OFF_STATE + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_FAULTS_LO == STATUS_OFF_FAULTS_HI + 1u,
                "status offsets");
STATIC_ASSERT (STATUS_OFF_FAULTS_LO + 1u == TELEMETRY_STATUS_WIRE_SIZE,
                "status size must match last offset");

STATIC_ASSERT (EVENT_OFF_ID_HI == 0u, "event offsets");
STATIC_ASSERT (EVENT_OFF_ID_LO == EVENT_OFF_ID_HI + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_TYPE == EVENT_OFF_ID_LO + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_P1_HI == EVENT_OFF_TYPE + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_P1_LO == EVENT_OFF_P1_HI + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_P2_HI == EVENT_OFF_P1_LO + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_P2_LO == EVENT_OFF_P2_HI + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_RESERVED == EVENT_OFF_P2_LO + 1u, "event offsets");
STATIC_ASSERT (EVENT_OFF_RESERVED + 1u == TELEMETRY_EVENT_WIRE_SIZE,
                "event size must match last offset");

STATIC_ASSERT (HEARTBEAT_OFF_UPTIME_HI == 0u, "heartbeat offsets");
STATIC_ASSERT (HEARTBEAT_OFF_UPTIME_LO == HEARTBEAT_OFF_UPTIME_HI + 1u,
                "heartbeat offsets");
STATIC_ASSERT (HEARTBEAT_OFF_DROPS == HEARTBEAT_OFF_UPTIME_LO + 1u,
                "heartbeat offsets");
STATIC_ASSERT (HEARTBEAT_OFF_RSSI == HEARTBEAT_OFF_DROPS + 1u,
                "heartbeat offsets");
STATIC_ASSERT (HEARTBEAT_OFF_RSSI + 1u == TELEMETRY_HEARTBEAT_WIRE_SIZE,
                "heartbeat size must match last offset");

STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_STATE == 0u, "system offsets");
STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_CAUSE == SYSTEM_OFF_OBSTACLE_STATE + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_FAULTS == SYSTEM_OFF_OBSTACLE_CAUSE + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_ECHO_CLASS == SYSTEM_OFF_OBSTACLE_FAULTS + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_DIST_HI == SYSTEM_OFF_ECHO_CLASS + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_DIST_LO == SYSTEM_OFF_OBSTACLE_DIST_HI + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_SEQ == SYSTEM_OFF_OBSTACLE_DIST_LO + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_OBSTACLE_REQ_ID == SYSTEM_OFF_OBSTACLE_SEQ + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_PHYSICS_STATE == SYSTEM_OFF_OBSTACLE_REQ_ID + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_PHYSICS_MOTION == SYSTEM_OFF_PHYSICS_STATE + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_PHYSICS_FAULTS == SYSTEM_OFF_PHYSICS_MOTION + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_PHYSICS_FLAGS == SYSTEM_OFF_PHYSICS_FAULTS + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_PITCH_HI == SYSTEM_OFF_PHYSICS_FLAGS + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_PITCH_LO == SYSTEM_OFF_PITCH_HI + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_HUMP_HI == SYSTEM_OFF_PITCH_LO + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_HUMP_LO == SYSTEM_OFF_HUMP_HI + 1u,
                "system offsets");
STATIC_ASSERT (SYSTEM_OFF_HUMP_LO + 1u == TELEMETRY_SYSTEM_WIRE_SIZE,
                "system size must match last offset");

uint16_t
telemetry_pack_status (uint8_t * dst, int16_t speed_mms, uint16_t dist_mm,
                       uint8_t heading, uint8_t state, uint16_t faults)
{
    if (dst == NULL)
    {
        return 0u;
    }

    uint16_t speed = (uint16_t) speed_mms;
    dst[STATUS_OFF_SPEED_HI] = (uint8_t) (speed >> WIRE_BYTE_BITS);
    dst[STATUS_OFF_SPEED_LO] = (uint8_t) speed;
    dst[STATUS_OFF_DIST_HI] = (uint8_t) (dist_mm >> WIRE_BYTE_BITS);
    dst[STATUS_OFF_DIST_LO] = (uint8_t) dist_mm;
    dst[STATUS_OFF_HEADING] = heading;
    dst[STATUS_OFF_STATE] = state;
    dst[STATUS_OFF_FAULTS_HI] = (uint8_t) (faults >> WIRE_BYTE_BITS);
    dst[STATUS_OFF_FAULTS_LO] = (uint8_t) faults;

    return TELEMETRY_STATUS_WIRE_SIZE;
}

uint16_t
telemetry_pack_event (uint8_t * dst, uint16_t event_id, uint8_t type,
                      int16_t p1, uint16_t p2)
{
    if (dst == NULL)
    {
        return 0u;
    }

    uint16_t p1_bits = (uint16_t) p1;
    dst[EVENT_OFF_ID_HI] = (uint8_t) (event_id >> WIRE_BYTE_BITS);
    dst[EVENT_OFF_ID_LO] = (uint8_t) event_id;
    dst[EVENT_OFF_TYPE] = type;
    dst[EVENT_OFF_P1_HI] = (uint8_t) (p1_bits >> WIRE_BYTE_BITS);
    dst[EVENT_OFF_P1_LO] = (uint8_t) p1_bits;
    dst[EVENT_OFF_P2_HI] = (uint8_t) (p2 >> WIRE_BYTE_BITS);
    dst[EVENT_OFF_P2_LO] = (uint8_t) p2;
    dst[EVENT_OFF_RESERVED] = 0u;

    return TELEMETRY_EVENT_WIRE_SIZE;
}

uint16_t
telemetry_pack_heartbeat (uint8_t * dst, uint16_t uptime_s, uint8_t drops,
                          int8_t rssi)
{
    if (dst == NULL)
    {
        return 0u;
    }

    dst[HEARTBEAT_OFF_UPTIME_HI] = (uint8_t) (uptime_s >> WIRE_BYTE_BITS);
    dst[HEARTBEAT_OFF_UPTIME_LO] = (uint8_t) uptime_s;
    dst[HEARTBEAT_OFF_DROPS] = drops;
    dst[HEARTBEAT_OFF_RSSI] = (uint8_t) rssi;

    return TELEMETRY_HEARTBEAT_WIRE_SIZE;
}

uint16_t
telemetry_pack_system (uint8_t * dst, const telemetry_system_t * p_system)
{
    if ((dst == NULL) || (p_system == NULL))
    {
        return 0u;
    }

    uint16_t pitch = (uint16_t) p_system->pitch_cdeg;
    dst[SYSTEM_OFF_OBSTACLE_STATE] = p_system->obstacle_state;
    dst[SYSTEM_OFF_OBSTACLE_CAUSE] = p_system->obstacle_cause;
    dst[SYSTEM_OFF_OBSTACLE_FAULTS] = p_system->obstacle_faults;
    dst[SYSTEM_OFF_ECHO_CLASS] = p_system->echo_class;
    dst[SYSTEM_OFF_OBSTACLE_DIST_HI] =
        (uint8_t) (p_system->obstacle_dist_mm >> WIRE_BYTE_BITS);
    dst[SYSTEM_OFF_OBSTACLE_DIST_LO] = (uint8_t) p_system->obstacle_dist_mm;
    dst[SYSTEM_OFF_OBSTACLE_SEQ] = p_system->obstacle_seq;
    dst[SYSTEM_OFF_OBSTACLE_REQ_ID] = p_system->obstacle_req_id;
    dst[SYSTEM_OFF_PHYSICS_STATE] = p_system->physics_state;
    dst[SYSTEM_OFF_PHYSICS_MOTION] = p_system->physics_motion;
    dst[SYSTEM_OFF_PHYSICS_FAULTS] = p_system->physics_faults;
    dst[SYSTEM_OFF_PHYSICS_FLAGS] = p_system->physics_flags;
    dst[SYSTEM_OFF_PITCH_HI] = (uint8_t) (pitch >> WIRE_BYTE_BITS);
    dst[SYSTEM_OFF_PITCH_LO] = (uint8_t) pitch;
    dst[SYSTEM_OFF_HUMP_HI] =
        (uint8_t) (p_system->hump_peak_mm >> WIRE_BYTE_BITS);
    dst[SYSTEM_OFF_HUMP_LO] = (uint8_t) p_system->hump_peak_mm;

    return TELEMETRY_SYSTEM_WIRE_SIZE;
}
