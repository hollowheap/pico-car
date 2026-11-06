#ifndef TELEMETRY_WORKER_H
#define TELEMETRY_WORKER_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef INT (*telemetry_pop_t)(uint8_t *frame, TMO tmout);
typedef int (*telemetry_sink_t)(const uint8_t *frame, uint16_t len);

/** Drain frames through `pop` and forward them to `sink`. */
void telemetry_worker_run(telemetry_pop_t pop, telemetry_sink_t sink, TMO timeout);

#endif /* TELEMETRY_WORKER_H */
