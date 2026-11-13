#include "telemetry/publisher.h"

void telemetry_publisher_run (telemetry_publisher_t * publisher,
                              telemetry_pull_t pull, telemetry_publish_t publish)
{
    for (;;)
    {
        int result = 0;

        if ((publisher->len == 0u) &&
            ((pull(publisher->frame, &publisher->len) <= 0) ||
             (publisher->len == 0u) || (publisher->len > TELEMETRY_FRAME_MAX)))
        {
            publisher->len = 0u;
            return;
        }
        if (publish != 0)
        {
            result = publish(publisher->frame, publisher->len);
        }
        if (result == TELEMETRY_PUBLISH_RETRY)
        {
            return;
        }
        publisher->len = 0u;
        if (result != 0)
        {
            return;
        }
    }
}
