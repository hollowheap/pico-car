#include "telemetry/publisher.h"

#define TELEMETRY_PUBLISHER_MAX_FRAME 17U

void telemetry_publisher_run (telemetry_pull_t pull, telemetry_publish_t publish)
{
    uint8_t  frame[TELEMETRY_PUBLISHER_MAX_FRAME];
    uint16_t len = 0U;

    while (pull(frame, &len) > 0)
    {
        if ((len == 0U) || (len > TELEMETRY_PUBLISHER_MAX_FRAME))
        {
            return;
        }
        if ((publish != 0) && (publish(frame, len) != 0))
        {
            return;
        }
    }
}
