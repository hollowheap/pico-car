/*
 * Host unit test for src/telemetry/cmd.c.
 *
 * Covers the 8-byte command frame parser (length, reserved byte, opcode
 * range, big-endian fields) and the accept policy (run ID, 2 s TTL with
 * u16 wrap, monotonic cmd_id with u16 wrap, dispatch exactly once).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry/cmd.h"

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

#define RUN_ID 0x1234u

typedef struct
{
    unsigned        calls;
    telemetry_cmd_t last;
    const telemetry_cmd_t * last_ptr;
    void * last_ctx;
} dispatch_probe_t;

static void
probe_dispatch (const telemetry_cmd_t * cmd, void * context)
{
    dispatch_probe_t * probe = (dispatch_probe_t *) context;

    probe->calls++;
    probe->last = *cmd;
    probe->last_ptr = cmd;
    probe->last_ctx = context;
}

static telemetry_cmd_t
make_cmd (uint16_t id, uint16_t run, uint16_t sent, uint8_t opcode)
{
    telemetry_cmd_t c;

    c.cmd_id = id;
    c.run_id = run;
    c.sent_ms = sent;
    c.opcode = opcode;
    return c;
}

static telemetry_cmd_result_t
accept (telemetry_cmd_state_t * st, const telemetry_cmd_t * c, uint16_t now,
        dispatch_probe_t * probe)
{
    return telemetry_cmd_accept (st, c, now, probe_dispatch, probe);
}

static void
test_constants (void)
{
    CHECK (TELEMETRY_CMD_WIRE_SIZE == 8u);
    CHECK (TELEMETRY_CMD_MAX_AGE_MS == 2000u);
    /* Opcode values come from protocol/telemetry.yaml. */
    CHECK ((unsigned) TELEMETRY_CMD_STOP == 0x00u);
    CHECK ((unsigned) TELEMETRY_CMD_START == 0x01u);
    CHECK ((unsigned) TELEMETRY_CMD_RESET == 0x02u);
    CHECK ((unsigned) TELEMETRY_CMD_LEFT == 0x03u);
    CHECK ((unsigned) TELEMETRY_CMD_RIGHT == 0x04u);
    CHECK ((unsigned) TELEMETRY_CMD_UTURN == 0x05u);
    CHECK ((unsigned) TELEMETRY_CMD_ESTOP == 0x06u);
}

static void
test_state_init (void)
{
    telemetry_cmd_state_t st;

    (void) memset (&st, 0xFF, sizeof (st));
    telemetry_cmd_state_init (&st, RUN_ID);
    CHECK (st.last_cmd_id == 0u);
    CHECK (st.expected_run_id == RUN_ID);
    CHECK (st.has_last == false);

    telemetry_cmd_state_init (NULL, RUN_ID); /* must not crash */
}

static void
test_parse_layout (void)
{
    /* cmd_id 0x0102, run_id 0x0304, sent_ms 0x0506, opcode 3, reserved 0 */
    static const uint8_t frame[8] = { 0x01, 0x02, 0x03, 0x04,
                                      0x05, 0x06, 0x03, 0x00 };
    telemetry_cmd_t c;

    (void) memset (&c, 0, sizeof (c));
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_OK);
    CHECK (c.cmd_id == 0x0102u);
    CHECK (c.run_id == 0x0304u);
    CHECK (c.sent_ms == 0x0506u);
    CHECK (c.opcode == 0x03u);
}

static void
test_parse_odd_buffer (void)
{
    /* No alignment requirement on the source buffer. */
    uint8_t raw[9] = { 0xA5, 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0x06, 0x00 };
    telemetry_cmd_t c;

    CHECK (telemetry_cmd_parse (&raw[1], 8u, &c) == TELEMETRY_CMD_OK);
    CHECK (c.cmd_id == 0xDEADu);
    CHECK (c.run_id == 0xBEEFu);
    CHECK (c.sent_ms == 0xCAFEu);
    CHECK (c.opcode == (uint8_t) TELEMETRY_CMD_ESTOP);
}

static void
test_parse_all_opcodes (void)
{
    uint8_t frame[8] = { 0, 1, 0, 2, 0, 3, 0, 0 };
    telemetry_cmd_t c;
    uint8_t op;

    for (op = 0u; op <= (uint8_t) TELEMETRY_CMD_ESTOP; op++)
    {
        frame[6] = op;
        CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_OK);
        CHECK (c.opcode == op);
    }
    /* First value past ESTOP, and the extremes.  The parser fills cmd_id,
       run_id and sent_ms before it checks the opcode, so *cmd is partly
       written on BAD_OPCODE; callers must not use it after a reject. */
    frame[6] = 0x07u;
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_OPCODE);
    frame[6] = 0x7Fu;
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_OPCODE);
    frame[6] = 0xFFu;
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_OPCODE);
}

static void
test_parse_rejects (void)
{
    uint8_t frame[10] = { 0, 1, 0, 2, 0, 3, 1, 0, 0, 0 };
    telemetry_cmd_t c;
    telemetry_cmd_t sentinel = make_cmd (0x1111u, 0x2222u, 0x3333u, 0x44u);

    CHECK (telemetry_cmd_parse (NULL, 8u, &c) == TELEMETRY_CMD_BAD_ARG);
    CHECK (telemetry_cmd_parse (frame, 8u, NULL) == TELEMETRY_CMD_BAD_ARG);

    c = sentinel;
    CHECK (telemetry_cmd_parse (frame, 0u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_parse (frame, 7u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_parse (frame, 9u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_parse (frame, 10u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_parse (frame, UINT16_MAX, &c)
           == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (memcmp (&c, &sentinel, sizeof (c)) == 0); /* output untouched */

    /* Any non-zero reserved byte is rejected. */
    frame[7] = 0x01u;
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_RESERVED);
    frame[7] = 0x80u;
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_RESERVED);
    frame[7] = 0xFFu;
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_RESERVED);
    CHECK (memcmp (&c, &sentinel, sizeof (c)) == 0);

    /* Length is checked before the reserved byte and the opcode. */
    frame[6] = 0xEEu;
    CHECK (telemetry_cmd_parse (frame, 7u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    /* Reserved is checked before the opcode. */
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_RESERVED);
}

static void
test_accept_bad_args (void)
{
    telemetry_cmd_state_t st;
    telemetry_cmd_t c = make_cmd (1u, RUN_ID, 100u, 0u);
    dispatch_probe_t probe;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    CHECK (telemetry_cmd_accept (NULL, &c, 100u, probe_dispatch, &probe)
           == TELEMETRY_CMD_BAD_ARG);
    CHECK (telemetry_cmd_accept (&st, NULL, 100u, probe_dispatch, &probe)
           == TELEMETRY_CMD_BAD_ARG);
    CHECK (probe.calls == 0u);
    CHECK (st.has_last == false);
}

static void
test_accept_dispatch_once (void)
{
    telemetry_cmd_state_t st;
    telemetry_cmd_t c = make_cmd (7u, RUN_ID, 1000u, TELEMETRY_CMD_UTURN);
    dispatch_probe_t probe;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    CHECK (accept (&st, &c, 1010u, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 1u);
    CHECK (probe.last_ptr == &c);
    CHECK (probe.last_ctx == (void *) &probe);
    CHECK (probe.last.cmd_id == 7u);
    CHECK (probe.last.opcode == (uint8_t) TELEMETRY_CMD_UTURN);
    CHECK (st.has_last == true);
    CHECK (st.last_cmd_id == 7u);

    /* The same command again is a duplicate and is not dispatched. */
    CHECK (accept (&st, &c, 1011u, &probe) == TELEMETRY_CMD_DUPLICATE);
    CHECK (probe.calls == 1u);
}

static void
test_accept_null_dispatch (void)
{
    telemetry_cmd_state_t st;
    telemetry_cmd_t c1 = make_cmd (1u, RUN_ID, 10u, TELEMETRY_CMD_STOP);
    telemetry_cmd_t c2 = make_cmd (1u, RUN_ID, 10u, TELEMETRY_CMD_STOP);

    telemetry_cmd_state_init (&st, RUN_ID);
    /* A NULL dispatch is allowed: the command is still recorded. */
    CHECK (telemetry_cmd_accept (&st, &c1, 20u, NULL, NULL)
           == TELEMETRY_CMD_OK);
    CHECK (st.has_last == true);
    CHECK (telemetry_cmd_accept (&st, &c2, 20u, NULL, NULL)
           == TELEMETRY_CMD_DUPLICATE);
}

static void
test_accept_run_id (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t wrong = make_cmd (1u, RUN_ID + 1u, 100u, 1u);
    telemetry_cmd_t zero = make_cmd (1u, 0u, 100u, 1u);
    telemetry_cmd_t right = make_cmd (1u, RUN_ID, 100u, 1u);
    telemetry_cmd_t stale_wrong = make_cmd (2u, 0x9999u, 0u, 1u);

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    CHECK (accept (&st, &wrong, 100u, &probe) == TELEMETRY_CMD_RUN_MISMATCH);
    CHECK (accept (&st, &zero, 100u, &probe) == TELEMETRY_CMD_RUN_MISMATCH);
    CHECK (probe.calls == 0u);
    CHECK (st.has_last == false);

    /* Run mismatch wins over staleness. */
    CHECK (accept (&st, &stale_wrong, 60000u, &probe)
           == TELEMETRY_CMD_RUN_MISMATCH);

    /* Rejections must not advance the monotonic counter. */
    CHECK (accept (&st, &right, 100u, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 1u);

    /* expected_run_id 0 accepts any run ID. */
    telemetry_cmd_state_init (&st, 0u);
    CHECK (accept (&st, &wrong, 100u, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 2u);
}

static void
test_accept_ttl (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t c;

    (void) memset (&probe, 0, sizeof (probe));

    /* Age 0, 1999, 2000 pass; 2001 is stale (limit is inclusive). */
    telemetry_cmd_state_init (&st, RUN_ID);
    c = make_cmd (1u, RUN_ID, 5000u, 1u);
    CHECK (accept (&st, &c, 5000u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (2u, RUN_ID, 5000u, 1u);
    CHECK (accept (&st, &c, 6999u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (3u, RUN_ID, 5000u, 1u);
    CHECK (accept (&st, &c, 7000u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (4u, RUN_ID, 5000u, 1u);
    CHECK (accept (&st, &c, 7001u, &probe) == TELEMETRY_CMD_STALE);
    CHECK (probe.calls == 3u);

    /* A stale command does not consume its cmd_id. */
    CHECK (accept (&st, &c, 7000u, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 4u);
}

static void
test_accept_ttl_wrap (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    /* sent 0xFFF0, now 0x0010: age is 32 across the u16 wrap. */
    c = make_cmd (1u, RUN_ID, 0xFFF0u, 1u);
    CHECK (accept (&st, &c, 0x0010u, &probe) == TELEMETRY_CMD_OK);

    /* sent 0xFFFF: now 1999 is age 2000 (ok), now 2000 is age 2001. */
    c = make_cmd (2u, RUN_ID, 0xFFFFu, 1u);
    CHECK (accept (&st, &c, 1999u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (3u, RUN_ID, 0xFFFFu, 1u);
    CHECK (accept (&st, &c, 2000u, &probe) == TELEMETRY_CMD_STALE);
    CHECK (probe.calls == 2u);
}

static void
test_accept_future_timestamp (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t c = make_cmd (1u, RUN_ID, 5100u, 1u);

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    /* sent_ms ahead of now wraps to a huge age and is treated as stale. */
    CHECK (accept (&st, &c, 5000u, &probe) == TELEMETRY_CMD_STALE);
    CHECK (probe.calls == 0u);
}

static void
test_accept_monotonic (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    /* The first command is accepted whatever its ID, including 0. */
    c = make_cmd (0u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);

    c = make_cmd (0u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_DUPLICATE);

    c = make_cmd (5u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 2u);

    /* Duplicate and older IDs are rejected and leave the state alone. */
    c = make_cmd (5u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_DUPLICATE);
    c = make_cmd (4u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_DUPLICATE);
    c = make_cmd (0u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_DUPLICATE);
    CHECK (st.last_cmd_id == 5u);
    CHECK (probe.calls == 2u);

    /* Gaps are fine. */
    c = make_cmd (105u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
    CHECK (st.last_cmd_id == 105u);
    CHECK (probe.calls == 3u);
}

static void
test_accept_cmd_id_wrap (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    c = make_cmd (0xFFFEu, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0xFFFFu, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);

    /* 0xFFFF -> 0x0000 is a forward step of 1 (serial-number arithmetic). */
    c = make_cmd (0x0000u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0x0001u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 4u);

    /* After the wrap, the pre-wrap IDs count as older. */
    c = make_cmd (0xFFFFu, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_DUPLICATE);
    CHECK (probe.calls == 4u);

    /* Half-range boundary: +0x7FFF is ahead, +0x8000 is behind. */
    telemetry_cmd_state_init (&st, RUN_ID);
    c = make_cmd (0x0000u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0x8000u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_DUPLICATE);
    c = make_cmd (0x7FFFu, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);
}

static void
test_accept_check_order (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    telemetry_cmd_t c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);
    c = make_cmd (10u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 100u, &probe) == TELEMETRY_CMD_OK);

    /* An old, stale command reports STALE (TTL is checked before the ID). */
    c = make_cmd (9u, RUN_ID, 100u, 1u);
    CHECK (accept (&st, &c, 9000u, &probe) == TELEMETRY_CMD_STALE);
    CHECK (probe.calls == 1u);
}

static void
test_parse_then_accept_each_opcode (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t probe;
    uint8_t frame[8];
    telemetry_cmd_t c;
    uint8_t op;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    for (op = 0u; op <= (uint8_t) TELEMETRY_CMD_ESTOP; op++)
    {
        frame[0] = 0x00u;
        frame[1] = (uint8_t) (op + 1u);
        frame[2] = (uint8_t) (RUN_ID >> 8);
        frame[3] = (uint8_t) RUN_ID;
        frame[4] = 0x03u;
        frame[5] = 0xE8u; /* sent_ms 1000 */
        frame[6] = op;
        frame[7] = 0x00u;
        CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_OK);
        CHECK (accept (&st, &c, 1500u, &probe) == TELEMETRY_CMD_OK);
        CHECK (probe.last.opcode == op);
        CHECK (probe.calls == (unsigned) op + 1u);
    }
}

int
main (void)
{
    test_constants ();
    test_state_init ();
    test_parse_layout ();
    test_parse_odd_buffer ();
    test_parse_all_opcodes ();
    test_parse_rejects ();
    test_accept_bad_args ();
    test_accept_dispatch_once ();
    test_accept_null_dispatch ();
    test_accept_run_id ();
    test_accept_ttl ();
    test_accept_ttl_wrap ();
    test_accept_future_timestamp ();
    test_accept_monotonic ();
    test_accept_cmd_id_wrap ();
    test_accept_check_order ();
    test_parse_then_accept_each_opcode ();

    if (g_failures != 0u)
    {
        (void) printf ("test_cmd: %u check(s) FAILED\n", g_failures);
        return 1;
    }
    (void) printf ("test_cmd: all tests passed\n");
    return 0;
}
