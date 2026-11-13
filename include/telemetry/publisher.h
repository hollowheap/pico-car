#ifndef TELEMETRY_PUBLISHER_H
#define TELEMETRY_PUBLISHER_H

#include <stdint.h>

#include "telemetry/packet.h"

typedef int (*telemetry_pull_t)(uint8_t *frame, uint16_t *len);
typedef int (*telemetry_publish_t)(const uint8_t *frame, uint16_t len);

typedef struct
{
    uint8_t  frame[TELEMETRY_FRAME_MAX];
    uint16_t len;
} telemetry_publisher_t;

#define TELEMETRY_PUBLISH_RETRY 1

/* Start with zeroed state. Keep a frame for the next call only on RETRY. */
void telemetry_publisher_run(telemetry_publisher_t * publisher,
                             telemetry_pull_t pull, telemetry_publish_t publish);

#endif /* TELEMETRY_PUBLISHER_H */
