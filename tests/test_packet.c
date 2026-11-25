/*
 * Host unit test for src/telemetry/packet.c (spec S01, S46).
 *
 * Every packer is checked against the known-good bytes that
 * protocol/generate.ts derives from protocol/telemetry.yaml
 * (tests/telemetry_vectors.h). A wrong offset, width or sign shows up as a
 * byte difference. The test also checks NULL handling, that a packer needs no
 * alignment, and that it never writes outside its packet.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry/packet.h"
#include "telemetry_vectors.h"

#define GUARD_BYTE 0xA5u
#define GUARD_LEN  4u
#define BUF_LEN    (GUARD_LEN + TELEMETRY_WIRE_MAX + GUARD_LEN)
#define ODD_OFFSET 1u

static unsigned g_failures = 0u;

#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            (void) printf ("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

/* A scratch area with guard bytes around the packet. */
typedef struct
{
    uint8_t   raw[BUF_LEN + ODD_OFFSET];
    uint8_t * pkt;
    uint16_t  size;
} scratch_t;

static void
scratch_init (scratch_t * s, uint16_t size, uint16_t lead)
{
    (void) memset (s->raw, GUARD_BYTE, sizeof (s->raw));
    s->pkt  = &s->raw[lead];
    s->size = size;
}

static int
guards_intact (const scratch_t * s, uint16_t lead)
{
    for (uint16_t i = 0u; i < lead; i++)
    {
        if (s->raw[i] != GUARD_BYTE)
        {
            return 0;
        }
    }
    for (uint16_t i = (uint16_t) (lead + s->size); i < sizeof (s->raw); i++)
    {
        if (s->raw[i] != GUARD_BYTE)
        {
            return 0;
        }
    }
    return 1;
}

/* Pack at `lead` bytes into the scratch and compare with the vector. */
#define CHECK_PACK(NAME, name, src, lead)                                    \
    do                                                                       \
    {                                                                        \
        scratch_t s;                                                         \
        scratch_init (&s, TELEMETRY_##NAME##_WIRE_SIZE, (lead));             \
        CHECK (telemetry_pack_##name (s.pkt, &(src)) ==                      \
               TELEMETRY_##NAME##_WIRE_SIZE);                                \
        CHECK (memcmp (s.pkt, TELEMETRY_VECTOR_##NAME,                       \
                       TELEMETRY_##NAME##_WIRE_SIZE) == 0);                  \
        CHECK (guards_intact (&s, (lead)));                                  \
    } while (0)

/* NULL destination or source: returns 0 and writes nothing. */
#define CHECK_NULLS(name, src)                                               \
    do                                                                       \
    {                                                                        \
        uint8_t guard[TELEMETRY_WIRE_MAX];                                   \
        (void) memset (guard, GUARD_BYTE, sizeof (guard));                   \
        CHECK (telemetry_pack_##name (NULL, &(src)) == 0u);                  \
        CHECK (telemetry_pack_##name (guard, NULL) == 0u);                   \
        for (unsigned i = 0u; i < sizeof (guard); i++)                       \
        {                                                                    \
            CHECK (guard[i] == GUARD_BYTE);                                  \
        }                                                                    \
    } while (0)

static void
test_status (void)
{
    telemetry_status_t src = {
        .odometer_mm  = TELEMETRY_VEC_STATUS_ODOMETER_MM,
        .speed_mms    = TELEMETRY_VEC_STATUS_SPEED_MMS,
        .heading_ddeg = TELEMETRY_VEC_STATUS_HEADING_DDEG,
        .faults       = TELEMETRY_VEC_STATUS_FAULTS,
        .seq          = TELEMETRY_VEC_STATUS_SEQ,
        .state        = TELEMETRY_VEC_STATUS_STATE,
        .mode         = TELEMETRY_VEC_STATUS_MODE,
        .line_bits    = TELEMETRY_VEC_STATUS_LINE_BITS,
        .hold_mask    = TELEMETRY_VEC_STATUS_HOLD_MASK,
    };

    CHECK_PACK (STATUS, status, src, 0u);
    CHECK_PACK (STATUS, status, src, ODD_OFFSET);
    CHECK_NULLS (status, src);
}

static void
test_motion (void)
{
    telemetry_motion_t src = {
        .left_mms         = TELEMETRY_VEC_MOTION_LEFT_MMS,
        .right_mms        = TELEMETRY_VEC_MOTION_RIGHT_MMS,
        .left_target_mms  = TELEMETRY_VEC_MOTION_LEFT_TARGET_MMS,
        .right_target_mms = TELEMETRY_VEC_MOTION_RIGHT_TARGET_MMS,
        .left_duty_pm     = TELEMETRY_VEC_MOTION_LEFT_DUTY_PM,
        .right_duty_pm    = TELEMETRY_VEC_MOTION_RIGHT_DUTY_PM,
        .overruns         = TELEMETRY_VEC_MOTION_OVERRUNS,
        .motion_state     = TELEMETRY_VEC_MOTION_MOTION_STATE,
        .motion_fault     = TELEMETRY_VEC_MOTION_MOTION_FAULT,
    };

    CHECK_PACK (MOTION, motion, src, 0u);
    CHECK_PACK (MOTION, motion, src, ODD_OFFSET);
    CHECK_NULLS (motion, src);
}

static void
test_physics (void)
{
    telemetry_physics_t src = {
        .pitch_cdeg     = TELEMETRY_VEC_PHYSICS_PITCH_CDEG,
        .roll_cdeg      = TELEMETRY_VEC_PHYSICS_ROLL_CDEG,
        .az_mg          = TELEMETRY_VEC_PHYSICS_AZ_MG,
        .a_fwd_mg       = TELEMETRY_VEC_PHYSICS_A_FWD_MG,
        .yaw_rate_cdps  = TELEMETRY_VEC_PHYSICS_YAW_RATE_CDPS,
        .amag_mg        = TELEMETRY_VEC_PHYSICS_AMAG_MG,
        .heading_ddeg   = TELEMETRY_VEC_PHYSICS_HEADING_DDEG,
        .hump_peak_mm   = TELEMETRY_VEC_PHYSICS_HUMP_PEAK_MM,
        .run_peak_mm    = TELEMETRY_VEC_PHYSICS_RUN_PEAK_MM,
        .hump_state     = TELEMETRY_VEC_PHYSICS_HUMP_STATE,
        .motion_class   = TELEMETRY_VEC_PHYSICS_MOTION_CLASS,
        .physics_faults = TELEMETRY_VEC_PHYSICS_PHYSICS_FAULTS,
        .flags          = TELEMETRY_VEC_PHYSICS_FLAGS,
        .attitude_state = TELEMETRY_VEC_PHYSICS_ATTITUDE_STATE,
    };

    CHECK_PACK (PHYSICS, physics, src, 0u);
    CHECK_PACK (PHYSICS, physics, src, ODD_OFFSET);
    CHECK_NULLS (physics, src);
    /* The reserved byte is always zero. */
    CHECK (TELEMETRY_VECTOR_PHYSICS[TELEMETRY_PHYSICS_OFF_RESERVED] == 0u);
}

static void
test_obstacle (void)
{
    telemetry_obstacle_t src = {
        .faults         = TELEMETRY_VEC_OBSTACLE_FAULTS,
        .dist_mm        = TELEMETRY_VEC_OBSTACLE_DIST_MM,
        .closest_mm     = TELEMETRY_VEC_OBSTACLE_CLOSEST_MM,
        .plan_width_mm  = TELEMETRY_VEC_OBSTACLE_PLAN_WIDTH_MM,
        .plan_need_mm   = TELEMETRY_VEC_OBSTACLE_PLAN_NEED_MM,
        .servo_deci     = TELEMETRY_VEC_OBSTACLE_SERVO_DECI,
        .body_deci      = TELEMETRY_VEC_OBSTACLE_BODY_DECI,
        .state          = TELEMETRY_VEC_OBSTACLE_STATE,
        .cause          = TELEMETRY_VEC_OBSTACLE_CAUSE,
        .echo_class     = TELEMETRY_VEC_OBSTACLE_ECHO_CLASS,
        .flags          = TELEMETRY_VEC_OBSTACLE_FLAGS,
        .seq            = TELEMETRY_VEC_OBSTACLE_SEQ,
        .req_id         = TELEMETRY_VEC_OBSTACLE_REQ_ID,
        .plan_status    = TELEMETRY_VEC_OBSTACLE_PLAN_STATUS,
        .plan_side      = TELEMETRY_VEC_OBSTACLE_PLAN_SIDE,
        .plan_start_deg = TELEMETRY_VEC_OBSTACLE_PLAN_START_DEG,
        .rescans        = TELEMETRY_VEC_OBSTACLE_RESCANS,
    };

    CHECK_PACK (OBSTACLE, obstacle, src, 0u);
    CHECK_PACK (OBSTACLE, obstacle, src, ODD_OFFSET);
    CHECK_NULLS (obstacle, src);
}

static void
test_diag (void)
{
    telemetry_diag_t src = {
        .sticky_faults        = TELEMETRY_VEC_DIAG_STICKY_FAULTS,
        .task_alive_mask      = TELEMETRY_VEC_DIAG_TASK_ALIVE_MASK,
        .line_lost_count      = TELEMETRY_VEC_DIAG_LINE_LOST_COUNT,
        .line_search_mm       = TELEMETRY_VEC_DIAG_LINE_SEARCH_MM,
        .barcode_ok           = TELEMETRY_VEC_DIAG_BARCODE_OK,
        .barcode_bad          = TELEMETRY_VEC_DIAG_BARCODE_BAD,
        .barcode_edge_drops   = TELEMETRY_VEC_DIAG_BARCODE_EDGE_DROPS,
        .telem_queue_drops    = TELEMETRY_VEC_DIAG_TELEM_QUEUE_DROPS,
        .net_queue_drops      = TELEMETRY_VEC_DIAG_NET_QUEUE_DROPS,
        .physics_overruns     = TELEMETRY_VEC_DIAG_PHYSICS_OVERRUNS,
        .telemetry_overruns   = TELEMETRY_VEC_DIAG_TELEMETRY_OVERRUNS,
        .last_barcode         = TELEMETRY_VEC_DIAG_LAST_BARCODE,
        .last_barcode_action  = TELEMETRY_VEC_DIAG_LAST_BARCODE_ACTION,
        .echo_valid           = TELEMETRY_VEC_DIAG_ECHO_VALID,
        .echo_open            = TELEMETRY_VEC_DIAG_ECHO_OPEN,
        .echo_too_close       = TELEMETRY_VEC_DIAG_ECHO_TOO_CLOSE,
        .echo_no_echo         = TELEMETRY_VEC_DIAG_ECHO_NO_ECHO,
        .echo_glitch          = TELEMETRY_VEC_DIAG_ECHO_GLITCH,
        .echo_stuck           = TELEMETRY_VEC_DIAG_ECHO_STUCK,
        .line_edges_per_s     = TELEMETRY_VEC_DIAG_LINE_EDGES_PER_S,
        .line_raw_edges_per_s = TELEMETRY_VEC_DIAG_LINE_RAW_EDGES_PER_S,
        .supervisor_passes = TELEMETRY_VEC_DIAG_SUPERVISOR_PASSES,
    };

    CHECK_PACK (DIAG, diag, src, 0u);
    CHECK_PACK (DIAG, diag, src, ODD_OFFSET);
    CHECK_NULLS (diag, src);
}

static void
test_heartbeat (void)
{
    telemetry_heartbeat_t src = {
        .uptime_s     = TELEMETRY_VEC_HEARTBEAT_UPTIME_S,
        .drops        = TELEMETRY_VEC_HEARTBEAT_DROPS,
        .wifi_rssi    = TELEMETRY_VEC_HEARTBEAT_WIFI_RSSI,
        .reset_reason = TELEMETRY_VEC_HEARTBEAT_RESET_REASON,
        .link_state   = TELEMETRY_VEC_HEARTBEAT_LINK_STATE,
    };

    CHECK_PACK (HEARTBEAT, heartbeat, src, 0u);
    CHECK_PACK (HEARTBEAT, heartbeat, src, ODD_OFFSET);
    CHECK_NULLS (heartbeat, src);
}

static void
test_event (void)
{
    telemetry_event_t src = {
        .odometer_mm = TELEMETRY_VEC_EVENT_ODOMETER_MM,
        .event_id    = TELEMETRY_VEC_EVENT_EVENT_ID,
        .p1          = TELEMETRY_VEC_EVENT_P1,
        .p2          = TELEMETRY_VEC_EVENT_P2,
        .event_type  = TELEMETRY_VEC_EVENT_EVENT_TYPE,
        .source      = TELEMETRY_VEC_EVENT_SOURCE,
    };

    CHECK_PACK (EVENT, event, src, 0u);
    CHECK_PACK (EVENT, event, src, ODD_OFFSET);
    CHECK_NULLS (event, src);
}

static void
test_profile (void)
{
    telemetry_profile_t src;

    (void) memset (&src, 0, sizeof (src));
    for (unsigned i = 0u; i < TELEMETRY_PROFILE_COUNT_SLOTS; i++)
    {
        src.slots[i] = TELEMETRY_VEC_PROFILE_SLOTS[i];
    }
    src.closest_mm = TELEMETRY_VEC_PROFILE_CLOSEST_MM;
    src.width_mm   = TELEMETRY_VEC_PROFILE_WIDTH_MM;
    src.req_id     = TELEMETRY_VEC_PROFILE_REQ_ID;
    src.status     = TELEMETRY_VEC_PROFILE_STATUS;
    src.side       = TELEMETRY_VEC_PROFILE_SIDE;
    src.flags      = TELEMETRY_VEC_PROFILE_FLAGS;

    CHECK_PACK (PROFILE, profile, src, 0u);
    CHECK_PACK (PROFILE, profile, src, ODD_OFFSET);
    CHECK_NULLS (profile, src);
}

/* The sizes the queue and the host decoders rely on. */
static void
test_sizes_and_topics (void)
{
    CHECK (TELEMETRY_SCHEMA_VERSION == 2u);
    CHECK (TELEMETRY_STATUS_WIRE_SIZE == 16u);
    CHECK (TELEMETRY_EVENT_WIRE_SIZE == 12u);
    CHECK (TELEMETRY_HEARTBEAT_WIRE_SIZE == 8u);
    CHECK (TELEMETRY_PROFILE_WIRE_SIZE == 82u);
    CHECK (TELEMETRY_WIRE_MAX == 82u);
    CHECK (TELEMETRY_FRAME_MAX == 83u);
    CHECK (TELEMETRY_TOPIC_STATUS == 's');
    CHECK (TELEMETRY_TOPIC_EVENT == 'e');
    CHECK (TELEMETRY_TOPIC_HEARTBEAT == 'h');
    CHECK (TELEMETRY_TOPIC_COMMAND == 'c');
    CHECK (TELEMETRY_FAULT_BIT_LINE_LOST == 4u);
    CHECK (TELEMETRY_FAULT_BIT_ESTOP_LATCHED == 9u);
}

int
main (void)
{
    test_status ();
    test_motion ();
    test_physics ();
    test_obstacle ();
    test_diag ();
    test_heartbeat ();
    test_event ();
    test_profile ();
    test_sizes_and_topics ();

    if (g_failures != 0u)
    {
        (void) printf ("%u check(s) failed\n", g_failures);
        return 1;
    }
    (void) printf ("test_packet: all checks passed\n");
    return 0;
}
