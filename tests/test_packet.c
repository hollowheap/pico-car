/*
 * Host unit test for src/telemetry/packet.c (spec S01).
 *
 * Checks the big-endian wire layout of the status, event, heartbeat and
 * system packets against protocol/telemetry.yaml, the NULL handling, and
 * that the serializers neither need alignment nor write outside the packet.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry/packet.h"

#define GUARD_BYTE   0xA5u
#define GUARD_LEN    4u
#define BUF_LEN      (GUARD_LEN + TELEMETRY_SYSTEM_WIRE_SIZE + GUARD_LEN)
#define ODD_OFFSET   1u

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

/* A scratch area with guard bytes around the packet.  The packet starts
   at an odd address when the caller passes an odd lead. */
typedef struct
{
    uint8_t raw[BUF_LEN + ODD_OFFSET];
    uint8_t * pkt;
    uint16_t  size;
} scratch_t;

static void
scratch_init (scratch_t * s, uint16_t size, uint16_t lead)
{
    (void) memset (s->raw, GUARD_BYTE, sizeof (s->raw));
    s->pkt = &s->raw[lead];
    s->size = size;
}

static int
guards_intact (const scratch_t * s, uint16_t lead)
{
    uint16_t i;
    for (i = 0u; i < lead; i++)
    {
        if (s->raw[i] != GUARD_BYTE)
        {
            return 0;
        }
    }
    for (i = (uint16_t) (lead + s->size); i < sizeof (s->raw); i++)
    {
        if (s->raw[i] != GUARD_BYTE)
        {
            return 0;
        }
    }
    return 1;
}

static int
bytes_equal (const uint8_t * got, const uint8_t * want, uint16_t len)
{
    return memcmp (got, want, len) == 0;
}

static void
test_wire_sizes (void)
{
    CHECK (TELEMETRY_STATUS_WIRE_SIZE == 8u);
    CHECK (TELEMETRY_EVENT_WIRE_SIZE == 8u);
    CHECK (TELEMETRY_HEARTBEAT_WIRE_SIZE == 4u);
    CHECK (TELEMETRY_SYSTEM_WIRE_SIZE == 16u);
}

static void
test_topics (void)
{
    CHECK (TELEMETRY_TOPIC_STATUS == 's');
    CHECK (TELEMETRY_TOPIC_EVENT == 'e');
    CHECK (TELEMETRY_TOPIC_HEARTBEAT == 'h');
    CHECK (TELEMETRY_TOPIC_COMMAND == 'c');
    CHECK (TELEMETRY_TOPIC_SYSTEM == 't');
}

static void
test_status_layout (void)
{
    scratch_t s;
    /* speed 0x0102, dist 0x0304, heading 0x05, state 0x06, faults 0x0708 */
    static const uint8_t want[8] = { 0x01, 0x02, 0x03, 0x04,
                                     0x05, 0x06, 0x07, 0x08 };
    uint16_t n;

    scratch_init (&s, TELEMETRY_STATUS_WIRE_SIZE, ODD_OFFSET);
    n = telemetry_pack_status (s.pkt, 0x0102, 0x0304u, 0x05u, 0x06u, 0x0708u);
    CHECK (n == 8u);
    CHECK (bytes_equal (s.pkt, want, 8u));
    CHECK (guards_intact (&s, ODD_OFFSET));
}

static void
test_status_signed_and_extremes (void)
{
    scratch_t s;
    static const uint8_t neg_one[8] = { 0xFF, 0xFF, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00 };
    static const uint8_t min_max[8] = { 0x80, 0x00, 0xFF, 0xFF,
                                        0xFF, 0xFF, 0xFF, 0xFF };
    static const uint8_t max_speed[8] = { 0x7F, 0xFF, 0x80, 0x00,
                                          0x01, 0x02, 0x00, 0x01 };
    static const uint8_t neg_mid[8] = { 0xFE, 0xD4, 0x12, 0x34,
                                        0x00, 0x00, 0x00, 0x00 };

    scratch_init (&s, TELEMETRY_STATUS_WIRE_SIZE, 0u);
    CHECK (telemetry_pack_status (s.pkt, -1, 0u, 0u, 0u, 0u) == 8u);
    CHECK (bytes_equal (s.pkt, neg_one, 8u));

    scratch_init (&s, TELEMETRY_STATUS_WIRE_SIZE, 0u);
    CHECK (telemetry_pack_status (s.pkt, INT16_MIN, UINT16_MAX, UINT8_MAX,
                                  UINT8_MAX, UINT16_MAX) == 8u);
    CHECK (bytes_equal (s.pkt, min_max, 8u));

    scratch_init (&s, TELEMETRY_STATUS_WIRE_SIZE, 0u);
    CHECK (telemetry_pack_status (s.pkt, INT16_MAX, 0x8000u, 1u, 2u, 1u)
           == 8u);
    CHECK (bytes_equal (s.pkt, max_speed, 8u));

    /* -300 mm/s is 0xFED4 in two's complement. */
    scratch_init (&s, TELEMETRY_STATUS_WIRE_SIZE, 0u);
    CHECK (telemetry_pack_status (s.pkt, -300, 0x1234u, 0u, 0u, 0u) == 8u);
    CHECK (bytes_equal (s.pkt, neg_mid, 8u));
}

static void
test_status_null (void)
{
    CHECK (telemetry_pack_status (NULL, 1, 2u, 3u, 4u, 5u) == 0u);
}

static void
test_event_layout (void)
{
    scratch_t s;
    /* id 0x1122, type 0x33, p1 0x4455, p2 0x6677, reserved 0 */
    static const uint8_t want[8] = { 0x11, 0x22, 0x33, 0x44,
                                     0x55, 0x66, 0x77, 0x00 };
    static const uint8_t neg[8] = { 0xFF, 0xFF, 0xFF, 0xFF,
                                    0x38, 0xFF, 0xFF, 0x00 };
    uint16_t n;

    scratch_init (&s, TELEMETRY_EVENT_WIRE_SIZE, ODD_OFFSET);
    /* Pre-dirty the reserved byte to prove it is written, not left alone. */
    s.pkt[7] = 0xEEu;
    n = telemetry_pack_event (s.pkt, 0x1122u, 0x33u, 0x4455, 0x6677u);
    CHECK (n == 8u);
    CHECK (bytes_equal (s.pkt, want, 8u));
    CHECK (s.pkt[7] == 0u);
    CHECK (guards_intact (&s, ODD_OFFSET));

    /* p1 = -200 is 0xFF38. */
    scratch_init (&s, TELEMETRY_EVENT_WIRE_SIZE, 0u);
    s.pkt[7] = 0xEEu;
    n = telemetry_pack_event (s.pkt, UINT16_MAX, UINT8_MAX, -200, UINT16_MAX);
    CHECK (n == 8u);
    CHECK (bytes_equal (s.pkt, neg, 8u));
    CHECK (guards_intact (&s, 0u));
}

static void
test_event_null (void)
{
    CHECK (telemetry_pack_event (NULL, 1u, 2u, 3, 4u) == 0u);
}

static void
test_heartbeat_layout (void)
{
    scratch_t s;
    static const uint8_t want[4] = { 0x12, 0x34, 0x56, 0x78 };
    static const uint8_t neg[4] = { 0xFF, 0xFF, 0xFF, 0x80 };
    static const uint8_t rssi_m60[4] = { 0x00, 0x01, 0x00, 0xC4 };
    uint16_t n;

    scratch_init (&s, TELEMETRY_HEARTBEAT_WIRE_SIZE, ODD_OFFSET);
    n = telemetry_pack_heartbeat (s.pkt, 0x1234u, 0x56u, 0x78);
    CHECK (n == 4u);
    CHECK (bytes_equal (s.pkt, want, 4u));
    CHECK (guards_intact (&s, ODD_OFFSET));

    scratch_init (&s, TELEMETRY_HEARTBEAT_WIRE_SIZE, 0u);
    n = telemetry_pack_heartbeat (s.pkt, UINT16_MAX, UINT8_MAX, INT8_MIN);
    CHECK (n == 4u);
    CHECK (bytes_equal (s.pkt, neg, 4u));
    CHECK (guards_intact (&s, 0u));

    /* -60 dBm is 0xC4. */
    scratch_init (&s, TELEMETRY_HEARTBEAT_WIRE_SIZE, 0u);
    n = telemetry_pack_heartbeat (s.pkt, 1u, 0u, -60);
    CHECK (n == 4u);
    CHECK (bytes_equal (s.pkt, rssi_m60, 4u));
}

static void
test_heartbeat_null (void)
{
    CHECK (telemetry_pack_heartbeat (NULL, 1u, 2u, 3) == 0u);
}

static void
test_system_layout (void)
{
    scratch_t s;
    telemetry_system_t sys;
    /* Every field distinct so a swapped offset shows up. */
    static const uint8_t want[16] = { 0x01, 0x02, 0x03, 0x04,
                                      0x05, 0x06, 0x07, 0x08,
                                      0x09, 0x0A, 0x0B, 0x0C,
                                      0x0D, 0x0E, 0x0F, 0x10 };
    uint16_t n;

    (void) memset (&sys, 0, sizeof (sys));
    sys.obstacle_state = 0x01u;
    sys.obstacle_cause = 0x02u;
    sys.obstacle_faults = 0x03u;
    sys.echo_class = 0x04u;
    sys.obstacle_dist_mm = 0x0506u;
    sys.obstacle_seq = 0x07u;
    sys.obstacle_req_id = 0x08u;
    sys.physics_state = 0x09u;
    sys.physics_motion = 0x0Au;
    sys.physics_faults = 0x0Bu;
    sys.physics_flags = 0x0Cu;
    sys.pitch_cdeg = 0x0D0E;
    sys.hump_peak_mm = 0x0F10u;

    scratch_init (&s, TELEMETRY_SYSTEM_WIRE_SIZE, ODD_OFFSET);
    n = telemetry_pack_system (s.pkt, &sys);
    CHECK (n == 16u);
    CHECK (bytes_equal (s.pkt, want, 16u));
    CHECK (guards_intact (&s, ODD_OFFSET));
}

static void
test_system_signed_pitch (void)
{
    scratch_t s;
    telemetry_system_t sys;

    (void) memset (&sys, 0, sizeof (sys));
    sys.pitch_cdeg = -1550; /* 0xF9F2 */
    sys.obstacle_dist_mm = UINT16_MAX;
    sys.hump_peak_mm = 0x8001u;

    scratch_init (&s, TELEMETRY_SYSTEM_WIRE_SIZE, 0u);
    CHECK (telemetry_pack_system (s.pkt, &sys) == 16u);
    CHECK (s.pkt[12] == 0xF9u);
    CHECK (s.pkt[13] == 0xF2u);
    CHECK (s.pkt[4] == 0xFFu);
    CHECK (s.pkt[5] == 0xFFu);
    CHECK (s.pkt[14] == 0x80u);
    CHECK (s.pkt[15] == 0x01u);
    CHECK (guards_intact (&s, 0u));

    sys.pitch_cdeg = INT16_MIN;
    scratch_init (&s, TELEMETRY_SYSTEM_WIRE_SIZE, 0u);
    CHECK (telemetry_pack_system (s.pkt, &sys) == 16u);
    CHECK (s.pkt[12] == 0x80u);
    CHECK (s.pkt[13] == 0x00u);
}

static void
test_system_null (void)
{
    telemetry_system_t sys;
    uint8_t buf[TELEMETRY_SYSTEM_WIRE_SIZE];

    (void) memset (&sys, 0, sizeof (sys));
    (void) memset (buf, GUARD_BYTE, sizeof (buf));

    CHECK (telemetry_pack_system (NULL, &sys) == 0u);
    CHECK (telemetry_pack_system (buf, NULL) == 0u);
    CHECK (telemetry_pack_system (NULL, NULL) == 0u);
    /* Nothing written when either pointer is NULL. */
    {
        uint8_t want[TELEMETRY_SYSTEM_WIRE_SIZE];

        (void) memset (want, GUARD_BYTE, sizeof (want));
        CHECK (bytes_equal (buf, want, TELEMETRY_SYSTEM_WIRE_SIZE));
    }
}

static void
test_odd_alignment_all_offsets (void)
{
    /* Pack at every start offset 0..3 to cover each alignment. */
    uint8_t raw[TELEMETRY_SYSTEM_WIRE_SIZE + 4u];
    telemetry_system_t sys;
    uint16_t lead;

    (void) memset (&sys, 0, sizeof (sys));
    sys.obstacle_dist_mm = 0xBEEFu;
    for (lead = 0u; lead < 4u; lead++)
    {
        (void) memset (raw, GUARD_BYTE, sizeof (raw));
        CHECK (telemetry_pack_status (&raw[lead], -2, 0xBEEFu, 1u, 2u, 3u)
               == 8u);
        CHECK (raw[lead + 2u] == 0xBEu);
        CHECK (raw[lead + 3u] == 0xEFu);
        CHECK (raw[lead + 8u] == GUARD_BYTE);

        (void) memset (raw, GUARD_BYTE, sizeof (raw));
        CHECK (telemetry_pack_system (&raw[lead], &sys) == 16u);
        CHECK (raw[lead + 4u] == 0xBEu);
        CHECK (raw[lead + 5u] == 0xEFu);
        if ((lead + 16u) < sizeof (raw))
        {
            CHECK (raw[lead + 16u] == GUARD_BYTE);
        }
    }
}

int
main (void)
{
    test_wire_sizes ();
    test_topics ();
    test_status_layout ();
    test_status_signed_and_extremes ();
    test_status_null ();
    test_event_layout ();
    test_event_null ();
    test_heartbeat_layout ();
    test_heartbeat_null ();
    test_system_layout ();
    test_system_signed_pitch ();
    test_system_null ();
    test_odd_alignment_all_offsets ();

    if (g_failures != 0u)
    {
        (void) printf ("test_packet: %u check(s) FAILED\n", g_failures);
        return 1;
    }
    (void) printf ("test_packet: all tests passed\n");
    return 0;
}
