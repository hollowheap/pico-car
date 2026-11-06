#ifndef TELEMETRY_PUBLISHER_H
#define TELEMETRY_PUBLISHER_H

#include <stdint.h>

typedef int (*telemetry_pull_t)(uint8_t *frame, uint16_t *len);
typedef int (*telemetry_publish_t)(const uint8_t *frame, uint16_t len);

/** Pull frames and publish them through the injected transport. */
void telemetry_publisher_run(telemetry_pull_t pull, telemetry_publish_t publish);

#endif /* TELEMETRY_PUBLISHER_H */
