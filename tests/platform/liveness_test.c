#include <assert.h>
#include <stdbool.h>
#include <tk/smp.h>

#include "platform/liveness.h"
#include "supervisor/health_ids.h"

#ifdef NDEBUG
#error "tests need assert()"
#endif

static bool g_kick_after_sample;

static void kick_after_sample (void)
{
    if (g_kick_after_sample)
    {
        g_kick_after_sample = false;
        liveness_kick(LIVE_CONNECTION);
    }
}

UW atomic_bitset (UW * addr, UW bits)
{
    const UW old = *addr;

    *addr = old | bits;
    if (0U == bits)
    {
        kick_after_sample();
    }
    return old;
}

UW atomic_xchg (UW * addr, UW value)
{
    const UW old = *addr;

    *addr = value;
    kick_after_sample();
    return old;
}

int main (void)
{
    const uint16_t connection = 1U << LIVE_CONNECTION;

    liveness_kick(LIVE_CONNECTION);
    /* The connection core kicks again after the supervisor samples the word. */
    g_kick_after_sample = true;
    assert(connection == liveness_take());
    assert(connection == liveness_take());
    assert(0U == liveness_take());
    return 0;
}
