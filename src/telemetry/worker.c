#include "telemetry/worker.h"

#define TELEMETRY_WORKER_MAX_FRAME 17U

void telemetry_worker_run (telemetry_pop_t pop, telemetry_sink_t sink, TMO timeout)
{
    uint8_t frame[TELEMETRY_WORKER_MAX_FRAME];

    for (;;)
    {
        const INT len = pop(frame, timeout);
        if (len <= 0)
        {
            return;
        }
        if ((sink != 0) && (sink(frame, (uint16_t)len) != 0))
        {
            return;
        }
    }
}
