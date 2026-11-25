/*
 * Host unit test for src/telemetry/tcp_transport.c against the lwIP stubs in
 * shim_tcp/.
 */

#include <stdint.h>
#include <stdio.h>

/* Included, not linked: the test reads the static lost-stream count, TX ring
   and stream flag without a test-only API in the firmware. */
#include "tcp_transport.c"
#include "tcp_stubs.h"

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

#define CONNECT_BYTES 18u
#define LATE_BYTES    4u

static struct pbuf g_pbuf;

static void
push (uint16_t len)
{
    uint8_t bytes[64] = {0};

    CHECK (tcp_transport_send (0, bytes, len) == (int) len);
}

static void
push_late (void)
{
    g_stubs.abort_hook = 0;
    g_stubs.sys_now_hook = 0;
    g_stubs.log_hook = 0;
    push (LATE_BYTES);
}

static void
reset (void)
{
    g_stubs.abort_hook = 0;
    g_stubs.sys_now_hook = 0;
    g_stubs.log_hook = 0;
    tcp_transport_close ();
    tcp_transport_poll ();
    tcp_stubs_reset ();
}

static void
open_attempt (void)
{
    CHECK (tcp_transport_open ("10.0.0.1", 1883u));
    tcp_transport_poll ();
}

static void
open_connected (void)
{
    open_attempt ();
    CHECK (tcp_transport_state () == TCP_TRANSPORT_CONNECTING);
    CHECK (g_stubs.on_connected (0, &g_stubs.pcb, ERR_OK) == ERR_OK);
    CHECK (tcp_transport_state () == TCP_TRANSPORT_CONNECTED);
}

static void
write_connect (void)
{
    push (CONNECT_BYTES);
    tcp_transport_poll ();
    CHECK (g_stream_tx);
    CHECK (ring_used (&g_tx) == 0u);
}

/* The error path aborts the pcb and then fails the attempt; one stream end
   counts one lost stream, even when the client pushes while it runs. lwIP
   passes ERR_OK to the recv callback today, so this branch is not taken on the
   car; the timeout test below covers the same rule on a live path. */
static void
test_recv_error_ends_the_stream_once (void)
{
    uint8_t base = 0u;

    reset ();
    open_connected ();
    write_connect ();
    base = g_lost_streams;
    g_stubs.sys_now_hook = push_late;

    CHECK (g_stubs.on_recv (0, &g_stubs.pcb, &g_pbuf, ERR_ABRT) == ERR_ABRT);

    CHECK ((uint8_t) (g_lost_streams - base) == 1u);
    CHECK (g_stubs.aborts == 1u);
    CHECK (tcp_transport_state () == TCP_TRANSPORT_FAILED);
    CHECK (!g_stream_tx);
}

/* A CONNECT the client queues after the stream ended stays for the next. */
static void
test_push_after_stream_end_stays_queued (void)
{
    uint8_t base = 0u;

    reset ();
    open_connected ();
    write_connect ();
    base = g_lost_streams;
    g_stubs.log_hook = push_late;

    CHECK (g_stubs.on_recv (0, &g_stubs.pcb, 0, ERR_OK) == ERR_OK);

    CHECK ((uint8_t) (g_lost_streams - base) == 1u);
    CHECK (g_stubs.closes == 1u);
    CHECK (ring_used (&g_tx) == LATE_BYTES);
}

static void
test_connect_timeout_counts_the_queued_connect_once (void)
{
    uint8_t base = 0u;

    reset ();
    open_attempt ();
    CHECK (tcp_transport_state () == TCP_TRANSPORT_CONNECTING);
    push (CONNECT_BYTES);
    base = g_lost_streams;
    g_stubs.now_ms += TCP_TRANSPORT_CONNECT_TIMEOUT_MS;
    g_stubs.abort_hook = push_late;
    g_stubs.log_calls = 0u;

    tcp_transport_poll ();

    CHECK ((uint8_t) (g_lost_streams - base) == 1u);
    CHECK (g_stubs.aborts == 1u);
    CHECK (g_stubs.log_calls == 1u);
    CHECK (ring_used (&g_tx) == 0u);
    CHECK (tcp_transport_state () == TCP_TRANSPORT_FAILED);
}

static void
test_close_ends_the_stream_once (void)
{
    uint8_t base = 0u;

    reset ();
    open_connected ();
    write_connect ();
    base = g_lost_streams;
    g_stubs.log_calls = 0u;

    tcp_transport_close ();
    tcp_transport_poll ();
    tcp_transport_poll ();

    CHECK ((uint8_t) (g_lost_streams - base) == 1u);
    CHECK (g_stubs.aborts == 1u);
    CHECK (g_stubs.log_calls == 0u);
    CHECK (tcp_transport_state () == TCP_TRANSPORT_CLOSED);
    CHECK (!tcp_transport_close_pending ());
}

static void
test_link_loss_ends_the_stream_once (void)
{
    uint8_t base = 0u;

    reset ();
    open_connected ();
    write_connect ();
    base = g_lost_streams;
    g_stubs.link_up = 0u;
    g_stubs.log_calls = 0u;

    tcp_transport_poll ();
    push (CONNECT_BYTES);
    tcp_transport_poll ();

    CHECK ((uint8_t) (g_lost_streams - base) == 1u);
    CHECK (g_stubs.aborts == 1u);
    CHECK (g_stubs.log_calls == 0u);
    CHECK (ring_used (&g_tx) == CONNECT_BYTES);
    CHECK (tcp_transport_state () == TCP_TRANSPORT_WAITING);
}

/* A close the stack refuses falls back to an abort. */
static void
test_failed_close_falls_back_to_abort (void)
{
    uint8_t base = 0u;

    reset ();
    open_connected ();
    write_connect ();
    base = g_lost_streams;
    g_stubs.close_result = ERR_MEM;

    CHECK (g_stubs.on_recv (0, &g_stubs.pcb, 0, ERR_OK) == ERR_OK);

    CHECK (g_stubs.closes == 1u);
    CHECK (g_stubs.aborts == 1u);
    CHECK ((uint8_t) (g_lost_streams - base) == 1u);
}

/* A write the stack refuses is not on the wire: the stream carries nothing. */
static void
test_stream_flag_follows_the_write (void)
{
    reset ();
    open_connected ();
    g_stubs.write_result = ERR_MEM;
    push (CONNECT_BYTES);

    tcp_transport_poll ();

    CHECK (!g_stream_tx);
    CHECK (ring_used (&g_tx) == CONNECT_BYTES);

    g_stubs.write_result = ERR_OK;
    tcp_transport_poll ();

    CHECK (g_stream_tx);
    CHECK (ring_used (&g_tx) == 0u);
    CHECK (g_stubs.written_bytes == CONNECT_BYTES);
}

/* A failed attempt that carried nothing reads as an empty stream. */
static void
test_recv_is_empty_after_a_quiet_failed_attempt (void)
{
    uint8_t buffer[8];
    uint8_t base = 0u;

    reset ();
    g_stubs.connect_result = ERR_RTE;
    base = g_lost_streams;

    open_attempt ();

    CHECK (tcp_transport_state () == TCP_TRANSPORT_FAILED);
    CHECK (g_lost_streams == base);
    CHECK (tcp_transport_recv (0, buffer, (uint16_t) sizeof(buffer)) == 0);
}

/* Each lost stream reports exactly one -1 to the reader. */
static void
test_recv_reports_each_lost_stream_once (void)
{
    uint8_t buffer[8];
    unsigned round;

    reset ();
    open_connected ();
    write_connect ();

    for (round = 0u; round < 2u; round++)
    {
        g_stubs.on_error (0, ERR_RST);

        CHECK (tcp_transport_recv (0, buffer, (uint16_t) sizeof(buffer)) ==
               -1);
        CHECK (tcp_transport_recv (0, buffer, (uint16_t) sizeof(buffer)) ==
               0);

        g_stubs.now_ms += TCP_TRANSPORT_RETRY_MS;
        tcp_transport_poll ();
        CHECK (g_stubs.on_connected (0, &g_stubs.pcb, ERR_OK) == ERR_OK);
        write_connect ();
    }
}

int
main (void)
{
    test_recv_error_ends_the_stream_once ();
    test_push_after_stream_end_stays_queued ();
    test_connect_timeout_counts_the_queued_connect_once ();
    test_close_ends_the_stream_once ();
    test_link_loss_ends_the_stream_once ();
    test_failed_close_falls_back_to_abort ();
    test_stream_flag_follows_the_write ();
    test_recv_is_empty_after_a_quiet_failed_attempt ();
    test_recv_reports_each_lost_stream_once ();

    if (g_failures != 0u)
    {
        (void) printf ("test_tcp_transport: %u check(s) FAILED\n", g_failures);
        return 1;
    }
    (void) printf ("test_tcp_transport: all tests passed\n");
    return 0;
}
