#include "telemetry/packet.h"
#include "telemetry/publisher.h"

#define TELEMETRY_PUBLISHER_MAX_FRAME TELEMETRY_FRAME_MAX
#define PUBLISH_MAX_FRAMES_PER_PASS (16U)

void telemetry_publisher_run (telemetry_pull_t pull, telemetry_publish_t publish)
{
    uint8_t  frame[TELEMETRY_PUBLISHER_MAX_FRAME];
    uint16_t len = 0U;

    /* Service incoming commands between bursts even if the producer keeps up. */
    for (unsigned n = 0U; n < PUBLISH_MAX_FRAMES_PER_PASS; ++n)
    {
        if ((pull(frame, &len) <= 0) || (len == 0U) || (len > TELEMETRY_PUBLISHER_MAX_FRAME))
        {
            return;
        }
        if ((publish != 0) && (publish(frame, len) != 0))
        {
            return;
        }
    }
}
