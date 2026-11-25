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

#include "comms/telemetry/packet.h"
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
        .odometer_mm  = TELEMETRY_VEC_STATUS_PIVOT_ODOMETER_MM,
        .speed_mms    = TELEMETRY_VEC_STATUS_PIVOT_SPEED_MMS,
        .heading_ddeg = TELEMETRY_VEC_STATUS_PIVOT_HEADING_DDEG,
        .faults       = TELEMETRY_VEC_STATUS_PIVOT_FAULTS,
        .seq          = TELEMETRY_VEC_STATUS_PIVOT_SEQ,
        .state        = TELEMETRY_VEC_STATUS_PIVOT_STATE,
        .mode         = TELEMETRY_VEC_STATUS_PIVOT_MODE,
        .line_bits    = TELEMETRY_VEC_STATUS_PIVOT_LINE_BITS,
        .hold_mask    = TELEMETRY_VEC_STATUS_PIVOT_HOLD_MASK,
        .pivot_scale_pct = TELEMETRY_VEC_STATUS_PIVOT_PIVOT_SCALE_PCT,
    };

    CHECK_PACK (STATUS_PIVOT, status, src, 0u);
    CHECK_PACK (STATUS_PIVOT, status, src, ODD_OFFSET);
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
        .seq              = TELEMETRY_VEC_MOTION_SEQ,
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
        .seq            = TELEMETRY_VEC_PHYSICS_SEQ,
    };

    CHECK_PACK (PHYSICS, physics, src, 0u);
    CHECK_PACK (PHYSICS, physics, src, ODD_OFFSET);
    CHECK_NULLS (physics, src);
}

static void
test_obstacle (void)
{
    telemetry_obstacle_t src = {
        .faults         = TELEMETRY_VEC_OBSTACLE_FAULTS,
        .dist_mm        = TELEMETRY_VEC_OBSTACLE_DIST_MM,
        .closest_mm     = TELEMETRY_VEC_OBSTACLE_CLOSEST_MM,
        .width_mm       = TELEMETRY_VEC_OBSTACLE_WIDTH_MM,
        .breadth_mm     = TELEMETRY_VEC_OBSTACLE_BREADTH_MM,
        .face_mm        = TELEMETRY_VEC_OBSTACLE_FACE_MM,
        .servo_deci     = TELEMETRY_VEC_OBSTACLE_SERVO_DECI,
        .body_deci      = TELEMETRY_VEC_OBSTACLE_BODY_DECI,
        .state          = TELEMETRY_VEC_OBSTACLE_STATE,
        .cause          = TELEMETRY_VEC_OBSTACLE_CAUSE,
        .echo_class     = TELEMETRY_VEC_OBSTACLE_ECHO_CLASS,
        .flags          = TELEMETRY_VEC_OBSTACLE_FLAGS,
        .seq            = TELEMETRY_VEC_OBSTACLE_SEQ,
        .hug_state      = TELEMETRY_VEC_OBSTACLE_HUG_STATE,
        .hug_side       = TELEMETRY_VEC_OBSTACLE_HUG_SIDE,
        .hug_seq        = TELEMETRY_VEC_OBSTACLE_HUG_SEQ,
    };

    CHECK_PACK (OBSTACLE, obstacle, src, 0u);
    CHECK_PACK (OBSTACLE, obstacle, src, ODD_OFFSET);
    CHECK_NULLS (obstacle, src);
}

static void
test_diag (void)
{
    telemetry_diag_t src = {
        .sticky_faults        = TELEMETRY_VEC_DIAG_COUNTS_STICKY_FAULTS,
        .task_alive_mask      = TELEMETRY_VEC_DIAG_COUNTS_TASK_ALIVE_MASK,
        .line_lost_count      = TELEMETRY_VEC_DIAG_COUNTS_LINE_LOST_COUNT,
        .barcode_ok           = TELEMETRY_VEC_DIAG_COUNTS_BARCODE_OK,
        .barcode_bad          = TELEMETRY_VEC_DIAG_COUNTS_BARCODE_BAD,
        .barcode_edge_drops   = TELEMETRY_VEC_DIAG_COUNTS_BARCODE_EDGE_DROPS,
        .telem_queue_drops    = TELEMETRY_VEC_DIAG_COUNTS_TELEM_QUEUE_DROPS,
        .net_queue_drops      = TELEMETRY_VEC_DIAG_COUNTS_NET_QUEUE_DROPS,
        .physics_overruns     = TELEMETRY_VEC_DIAG_COUNTS_PHYSICS_OVERRUNS,
        .telemetry_overruns   = TELEMETRY_VEC_DIAG_COUNTS_TELEMETRY_OVERRUNS,
        .last_barcode         = TELEMETRY_VEC_DIAG_COUNTS_LAST_BARCODE,
        .last_barcode_action  = TELEMETRY_VEC_DIAG_COUNTS_LAST_BARCODE_ACTION,
        .line_edges_total     = TELEMETRY_VEC_DIAG_COUNTS_LINE_EDGES_TOTAL,
        .line_raw_edges_total = TELEMETRY_VEC_DIAG_COUNTS_LINE_RAW_EDGES_TOTAL,
        .sensor_sample_us     = TELEMETRY_VEC_DIAG_COUNTS_SENSOR_SAMPLE_US,
        .supervisor_passes = TELEMETRY_VEC_DIAG_COUNTS_SUPERVISOR_PASSES,
        .stack_min_free    = TELEMETRY_VEC_DIAG_COUNTS_STACK_MIN_FREE,
        .stack_min_task    = TELEMETRY_VEC_DIAG_COUNTS_STACK_MIN_TASK,
        .cmd_ring_drops    = TELEMETRY_VEC_DIAG_COUNTS_CMD_RING_DROPS,
        .cmd_last_id       = TELEMETRY_VEC_DIAG_COUNTS_CMD_LAST_ID,
        .cmd_rej_length    = TELEMETRY_VEC_DIAG_COUNTS_CMD_REJ_LENGTH,
        .cmd_rej_opcode    = TELEMETRY_VEC_DIAG_COUNTS_CMD_REJ_OPCODE,
        .cmd_rej_run       = TELEMETRY_VEC_DIAG_COUNTS_CMD_REJ_RUN,
        .cmd_rej_old_id    = TELEMETRY_VEC_DIAG_COUNTS_CMD_REJ_OLD_ID,
        .cmd_rej_age       = TELEMETRY_VEC_DIAG_COUNTS_CMD_REJ_AGE,
    };

    for (unsigned i = 0u; i < TELEMETRY_DIAG_COUNTS_COUNT_ECHO_TOTAL; i++)
    {
        src.echo_total[i] = TELEMETRY_VEC_DIAG_COUNTS_ECHO_TOTAL[i];
    }

    CHECK_PACK (DIAG_COUNTS, diag, src, 0u);
    CHECK_PACK (DIAG_COUNTS, diag, src, ODD_OFFSET);
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
        .run_id       = TELEMETRY_VEC_HEARTBEAT_RUN_ID,
        .car_id       = TELEMETRY_VEC_HEARTBEAT_CAR_ID,
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
    CHECK (TELEMETRY_SCHEMA_VERSION == 3u);
    CHECK (TELEMETRY_STATUS_WIRE_SIZE == 16u);
    CHECK (TELEMETRY_STATUS_PIVOT_WIRE_SIZE == 17u);
    CHECK (TELEMETRY_EVENT_WIRE_SIZE == 12u);
    CHECK (TELEMETRY_DIAG_WIRE_SIZE == 45u);
    CHECK (TELEMETRY_DIAG_COUNTS_WIRE_SIZE == 73u);
    CHECK (TELEMETRY_HEARTBEAT_WIRE_SIZE == 14u);
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
