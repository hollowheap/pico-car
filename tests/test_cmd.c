/*
 * Host unit test for src/telemetry/cmd.c.
 *
 * Covers the 9-byte command frame parser (length, opcode range, big-endian
 * fields) and the accept policy: the run ID the car announced at boot, a
 * cmd_id newer than the last accepted one inside the 16-bit sequence, and an
 * echoed heartbeat uptime at most 3 s old. Every refusal is counted per
 * cause. The car clock is a parameter, so no real clock is involved.
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
#define NOW_S  100u

typedef struct
{
    unsigned        calls;
    telemetry_cmd_t last;
    void *          last_ctx;
} dispatch_probe_t;

static void
probe_dispatch (const telemetry_cmd_t * cmd, void * context)
{
    dispatch_probe_t * probe = (dispatch_probe_t *) context;

    probe->calls++;
    probe->last     = *cmd;
    probe->last_ctx = context;
}

/* A command that echoes the car uptime NOW_S. */
static telemetry_cmd_t
make_cmd (uint16_t id, uint16_t run, uint8_t opcode)
{
    telemetry_cmd_t c;

    (void) memset (&c, 0, sizeof (c));
    c.cmd_id   = id;
    c.run_id   = run;
    c.opcode   = opcode;
    c.uptime_s = NOW_S;
    return c;
}

static telemetry_cmd_result_t
accept (telemetry_cmd_state_t * st, const telemetry_cmd_t * c,
        dispatch_probe_t * probe)
{
    return telemetry_cmd_accept (st, c, NOW_S, probe_dispatch, probe);
}

/* The wire frame for a command, big-endian. */
static void
frame_of (const telemetry_cmd_t * c, uint8_t out[TELEMETRY_CMD_WIRE_SIZE])
{
    out[0] = (uint8_t) (c->cmd_id >> 8);
    out[1] = (uint8_t) c->cmd_id;
    out[2] = (uint8_t) (c->run_id >> 8);
    out[3] = (uint8_t) c->run_id;
    out[4] = c->opcode;
    out[5] = (uint8_t) (c->uptime_s >> 24);
    out[6] = (uint8_t) (c->uptime_s >> 16);
    out[7] = (uint8_t) (c->uptime_s >> 8);
    out[8] = (uint8_t) c->uptime_s;
}

static telemetry_cmd_result_t
receive (telemetry_cmd_state_t * st, const telemetry_cmd_t * c,
         dispatch_probe_t * probe)
{
    uint8_t frame[TELEMETRY_CMD_WIRE_SIZE];

    frame_of (c, frame);
    return telemetry_cmd_receive (st, frame, TELEMETRY_CMD_WIRE_SIZE, NOW_S,
                                  probe_dispatch, probe);
}

static void
test_constants (void)
{
    CHECK (TELEMETRY_CMD_WIRE_SIZE == 9u);
    CHECK (TELEMETRY_CMD_MAX_AGE_S == 3u);
    /* Opcode values come from protocol/telemetry.yaml. */
    CHECK ((unsigned) TELEMETRY_CMD_STOP == 0x00u);
    CHECK ((unsigned) TELEMETRY_CMD_START == 0x01u);
    CHECK ((unsigned) TELEMETRY_CMD_RESET == 0x02u);
    CHECK ((unsigned) TELEMETRY_CMD_LEFT == 0x03u);
    CHECK ((unsigned) TELEMETRY_CMD_RIGHT == 0x04u);
    CHECK ((unsigned) TELEMETRY_CMD_UTURN == 0x05u);
    CHECK ((unsigned) TELEMETRY_CMD_ESTOP == 0x06u);
    CHECK ((unsigned) TELEMETRY_CMD_STRAIGHT == 0x07u);
    CHECK ((unsigned) TELEMETRY_CMD_RECALIBRATE == 0x08u);
    CHECK ((unsigned) TELEMETRY_CMD_START_CONTINUOUS_FLOOR == 0x09u);
    CHECK ((unsigned) TELEMETRY_CMD_START_STATIONARY == 0x0Au);
}

static void
test_state_init (void)
{
    telemetry_cmd_state_t st;

    (void) memset (&st, 0xFF, sizeof (st));
    telemetry_cmd_state_init (&st, RUN_ID);
    CHECK (st.expected_run_id == RUN_ID);
    CHECK (st.has_last == false);
    CHECK (st.last_cmd_id == 0u);
    CHECK (st.rejects.length == 0u);
    CHECK (st.rejects.opcode == 0u);
    CHECK (st.rejects.run == 0u);
    CHECK (st.rejects.old_id == 0u);
    CHECK (st.rejects.age == 0u);

    /* Run ID 0 would accept a host that never read a heartbeat. */
    telemetry_cmd_state_init (&st, 0u);
    CHECK (st.expected_run_id != 0u);

    telemetry_cmd_state_init (NULL, RUN_ID); /* must not crash */
}

static void
test_parse_layout (void)
{
    /* cmd_id 0x0102, run_id 0x0304, opcode 3, uptime 0x11223344 s; the
     * source needs no alignment. */
    static const uint8_t raw[10] = { 0xA5, 0x01, 0x02, 0x03, 0x04, 0x03,
                                     0x11, 0x22, 0x33, 0x44 };
    telemetry_cmd_t      c;

    (void) memset (&c, 0, sizeof (c));
    CHECK (telemetry_cmd_parse (&raw[1], 9u, &c) == TELEMETRY_CMD_OK);
    CHECK (c.cmd_id == 0x0102u);
    CHECK (c.run_id == 0x0304u);
    CHECK (c.opcode == 0x03u);
    CHECK (c.uptime_s == 0x11223344u);
}

static void
test_parse_rejects (void)
{
    uint8_t         frame[10] = { 0, 1, 0, 2, 0, 0, 0, 0, 0, 0 };
    telemetry_cmd_t c;
    telemetry_cmd_t sentinel = make_cmd (0x1111u, 0x2222u, 0x44u);
    uint8_t         op;

    CHECK (telemetry_cmd_parse (NULL, 9u, &c) == TELEMETRY_CMD_BAD_ARG);
    CHECK (telemetry_cmd_parse (frame, 9u, NULL) == TELEMETRY_CMD_BAD_ARG);

    c = sentinel;
    CHECK (telemetry_cmd_parse (frame, 0u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    /* The 5-byte frame without the uptime echo is refused. */
    CHECK (telemetry_cmd_parse (frame, 5u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_parse (frame, 8u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_parse (frame, 10u, &c) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (memcmp (&c, &sentinel, sizeof (c)) == 0); /* output untouched */

    for (op = 0u; op <= (uint8_t) TELEMETRY_CMD_START_STATIONARY; op++)
    {
        frame[4] = op;
        CHECK (telemetry_cmd_parse (frame, 9u, &c) == TELEMETRY_CMD_OK);
        CHECK (c.opcode == op);
    }
    frame[4] = 0x0Bu;
    CHECK (telemetry_cmd_parse (frame, 9u, &c) == TELEMETRY_CMD_BAD_OPCODE);
    frame[4] = 0xFFu;
    CHECK (telemetry_cmd_parse (frame, 9u, &c) == TELEMETRY_CMD_BAD_OPCODE);
}

static void
test_accept_bad_args (void)
{
    telemetry_cmd_state_t st;
    telemetry_cmd_t       c = make_cmd (1u, RUN_ID, 0u);
    dispatch_probe_t      probe;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);
    CHECK (telemetry_cmd_accept (NULL, &c, NOW_S, probe_dispatch, &probe)
           == TELEMETRY_CMD_BAD_ARG);
    CHECK (telemetry_cmd_accept (&st, NULL, NOW_S, probe_dispatch, &probe)
           == TELEMETRY_CMD_BAD_ARG);
    CHECK (probe.calls == 0u);

    /* A NULL dispatch still accepts and records the id. */
    CHECK (telemetry_cmd_accept (&st, &c, NOW_S, NULL, NULL)
           == TELEMETRY_CMD_OK);
    CHECK (st.last_cmd_id == 1u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
}

static void
test_accept_run_id (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    /* A host that never read the heartbeat sends 0; an old run is wrong. */
    c = make_cmd (1u, 0u, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_RUN_MISMATCH);
    c = make_cmd (1u, (uint16_t) (RUN_ID + 1u), 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_RUN_MISMATCH);
    CHECK (probe.calls == 0u);
    CHECK (st.has_last == false);

    c = make_cmd (1u, RUN_ID, (uint8_t) TELEMETRY_CMD_START);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 1u);
    CHECK (probe.last.opcode == (uint8_t) TELEMETRY_CMD_START);
    CHECK (probe.last.cmd_id == 1u);
    CHECK (probe.last.run_id == RUN_ID);
    CHECK (probe.last_ctx == &probe);
}

/* After a boot the first command of the run may carry any id. */
static void
test_window_first_command (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);
    c = make_cmd (0x9000u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    CHECK (st.has_last && (st.last_cmd_id == 0x9000u));

    telemetry_cmd_state_init (&st, RUN_ID);
    c = make_cmd (0u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (probe.calls == 2u);
}

static void
test_window_old_and_duplicate (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    c = make_cmd (100u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    /* A QoS 1 redelivery repeats the id. */
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    /* A late command older than the last accepted one. */
    c = make_cmd (99u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    c = make_cmd (1u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (probe.calls == 1u);
    CHECK (st.last_cmd_id == 100u);

    /* A gap is fine: lost commands do not block the next one. */
    c = make_cmd (105u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (104u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (probe.calls == 2u);
}

static void
test_window_wraps (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    c = make_cmd (0xFFFEu, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0xFFFFu, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0x0000u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0x0001u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    /* 0xFFFF is now behind 0x0001. */
    c = make_cmd (0xFFFFu, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (probe.calls == 4u);
}

/* Half the sequence or more ahead reads as behind. */
static void
test_window_far_future (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    c = make_cmd (0x0010u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0x8010u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    c = make_cmd (0xF000u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (st.last_cmd_id == 0x0010u);

    c = make_cmd (0x800Fu, RUN_ID, 0u); /* 32767 ahead: the edge */
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    c = make_cmd (0x0010u, RUN_ID, 0u);
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (probe.calls == 2u);
}

static void
test_age (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    c = make_cmd (1u, RUN_ID, 0u);
    c.uptime_s = NOW_S - TELEMETRY_CMD_MAX_AGE_S;
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);

    /* One second more is too old. The clock counts whole seconds, so this
     * heartbeat was built between 3 s and 5 s before the command arrived. */
    c = make_cmd (2u, RUN_ID, 0u);
    c.uptime_s = NOW_S - TELEMETRY_CMD_MAX_AGE_S - 1u;
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_BAD_AGE);
    c.uptime_s = 0u;
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_BAD_AGE);

    /* An echo ahead of the car never came from its heartbeat. */
    c.uptime_s = NOW_S + 1u;
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_BAD_AGE);
    c.uptime_s = 0xFFFFFFFFu;
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_BAD_AGE);
    CHECK (probe.calls == 1u);

    /* A refused command does not move the window: the same id with a
     * fresh echo is accepted. */
    CHECK (st.last_cmd_id == 1u);
    c.uptime_s = NOW_S;
    CHECK (accept (&st, &c, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 2u);

    /* At boot the car and the echo are both at 0 s. */
    telemetry_cmd_state_init (&st, RUN_ID);
    c.uptime_s = 0u;
    CHECK (telemetry_cmd_accept (&st, &c, 0u, probe_dispatch, &probe)
           == TELEMETRY_CMD_OK);
}

static void
test_receive_counts_each_cause (void)
{
    telemetry_cmd_state_t st;
    dispatch_probe_t      probe;
    telemetry_cmd_t       c;
    uint8_t               frame[TELEMETRY_CMD_WIRE_SIZE];
    unsigned              idx;

    (void) memset (&probe, 0, sizeof (probe));
    telemetry_cmd_state_init (&st, RUN_ID);

    c = make_cmd (7u, RUN_ID, (uint8_t) TELEMETRY_CMD_LEFT);
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_OK);
    CHECK (probe.calls == 1u);
    CHECK (probe.last.cmd_id == 7u);
    CHECK (probe.last.run_id == RUN_ID);
    CHECK (probe.last.opcode == (uint8_t) TELEMETRY_CMD_LEFT);
    CHECK (probe.last.uptime_s == NOW_S);

    frame_of (&c, frame);
    CHECK (telemetry_cmd_receive (&st, frame, 5u, NOW_S, probe_dispatch,
                                  &probe) == TELEMETRY_CMD_BAD_LENGTH);
    CHECK (telemetry_cmd_receive (&st, NULL, 9u, NOW_S, probe_dispatch,
                                  &probe) == TELEMETRY_CMD_BAD_ARG);
    CHECK (st.rejects.length == 2u);

    c = make_cmd (8u, RUN_ID, 0x0Bu);
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_BAD_OPCODE);
    CHECK (st.rejects.opcode == 1u);

    c = make_cmd (8u, (uint16_t) (RUN_ID + 1u), 0u);
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_RUN_MISMATCH);
    CHECK (st.rejects.run == 1u);

    c = make_cmd (7u, RUN_ID, 0u);
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    c = make_cmd (6u, RUN_ID, 0u);
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_OLD_ID);
    CHECK (st.rejects.old_id == 2u);

    c = make_cmd (8u, RUN_ID, 0u);
    c.uptime_s = NOW_S + 1u;
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_BAD_AGE);
    c.uptime_s = NOW_S - 4u;
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_BAD_AGE);
    CHECK (st.rejects.age == 2u);

    /* Nothing else moved. */
    CHECK (st.rejects.length == 2u);
    CHECK (st.rejects.opcode == 1u);
    CHECK (st.rejects.run == 1u);
    CHECK (st.rejects.old_id == 2u);
    CHECK (probe.calls == 1u);

    /* A count is the low byte of a count that wraps. */
    for (idx = 0u; idx < 255u; idx++)
    {
        (void) telemetry_cmd_receive (&st, frame, 0u, NOW_S, NULL, NULL);
    }
    CHECK (st.rejects.length == 1u);

    /* An accepted command counts nothing. */
    c = make_cmd (8u, RUN_ID, 0u);
    CHECK (receive (&st, &c, &probe) == TELEMETRY_CMD_OK);
    CHECK (st.rejects.age == 2u);
    CHECK (st.rejects.old_id == 2u);

    CHECK (telemetry_cmd_receive (NULL, frame, 9u, NOW_S, NULL, NULL)
           == TELEMETRY_CMD_BAD_ARG); /* must not crash */
}

int
main (int argc, char * argv[])
{
    if (argc == 2 && strcmp(argv[1], "reserved-stationary-opcode") == 0)
    {
        uint8_t const raw[TELEMETRY_CMD_WIRE_SIZE] = {0U, 1U, 0x12U, 0x34U, 10U, 0U, 0U, 0U, 100U};
        telemetry_cmd_t command;
        CHECK (telemetry_cmd_parse (raw, sizeof(raw), &command) == TELEMETRY_CMD_BAD_OPCODE);
        return (g_failures == 0U) ? 0 : 1;
    }
    if (argc != 1)
    {
        return 2;
    }
    test_constants ();
    test_state_init ();
    test_parse_layout ();
    test_parse_rejects ();
    test_accept_bad_args ();
    test_accept_run_id ();
    test_window_first_command ();
    test_window_old_and_duplicate ();
    test_window_wraps ();
    test_window_far_future ();
    test_age ();
    test_receive_counts_each_cause ();

    if (g_failures != 0u)
    {
        (void) printf ("test_cmd: %u failure(s)\n", g_failures);
        return 1;
    }
    (void) printf ("test_cmd: all checks passed\n");
    return 0;
}
