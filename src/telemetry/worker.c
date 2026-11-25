#include "telemetry/packet.h"
#include "telemetry/worker.h"

#define WORKER_MAX_FRAMES_PER_PASS (16U)

void telemetry_worker_run (telemetry_pop_t pop, telemetry_sink_t sink, TMO timeout)
{
    uint8_t frame[TELEMETRY_FRAME_MAX];

    /* Return to the periodic scheduler even during a continuous edge burst. */
    for (unsigned n = 0U; n < WORKER_MAX_FRAMES_PER_PASS; ++n)
    {
        const INT len = pop(frame, (0U == n) ? timeout : TMO_POL);
        if ((len <= 0) || (len > (INT)sizeof(frame)))
        {
            return;
        }
        if ((sink != 0) && (sink(frame, (uint16_t)len) != 0))
        {
            return;
        }
    }
}
