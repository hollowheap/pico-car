/*
 * Host unit test for the lost-stream count rule of src/telemetry/stream_loss.h.
 *
 * tcp_transport_recv reports one -1 whenever the count differs from the value
 * it last saw. The client does not read while it backs off, so a count that
 * moved during the back-off would drop the first good stream after an outage.
 */

#include <stdint.h>
#include <stdio.h>

#include "stream_loss.h"

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

/* Failed connect attempts while the client backs off carry no bytes, and
   neither does a connected stream that ends before the client wrote a byte. */
static void
test_idle_streams_in_backoff_do_not_count (void)
{
    uint8_t lost = 7u;
    const uint8_t seen = lost;
    unsigned attempt;

    for (attempt = 0u; attempt < 5u; attempt++)
    {
        lost = stream_loss_note (lost, false, 0u);
    }
    /* The healthy CONNECT that follows reads on an unchanged count. */
    CHECK (lost == seen);
}

static void
test_lost_bytes_count (void)
{
    /* A stream the client wrote to, with nothing left queued. */
    CHECK (stream_loss_note (7u, true, 0u) == 8u);
    CHECK (stream_loss_note (7u, true, 40u) == 8u);

    /* A CONNECT queued for a stream that never came up. */
    CHECK (stream_loss_note (7u, false, 18u) == 8u);
    CHECK (stream_loss_note (7u, false, 1u) == 8u);

    /* The 8-bit count wraps. */
    CHECK (stream_loss_note (255u, true, 0u) == 0u);
}

int
main (void)
{
    test_idle_streams_in_backoff_do_not_count ();
    test_lost_bytes_count ();

    if (g_failures != 0u)
    {
        (void) printf ("test_telemetry_stream_loss: %u check(s) FAILED\n",
                       g_failures);
        return 1;
    }
    (void) printf ("test_telemetry_stream_loss: all tests passed\n");
    return 0;
}
