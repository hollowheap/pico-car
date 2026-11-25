#ifndef TELEMETRY_MQTT_SESSION_H
#define TELEMETRY_MQTT_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include "comms/telemetry/cmd.h"
#include "comms/telemetry/packet.h"

/* The connection task starts and uses these handoffs. */
int32_t  mqtt_session_init (void);
void     mqtt_session_start (int32_t task);
uint32_t mqtt_session_now_ms (void);
bool     mqtt_session_connected (void);
bool     mqtt_session_heartbeat (
        const uint8_t payload[TELEMETRY_HEARTBEAT_WIRE_SIZE]);
bool     mqtt_session_command (uint8_t    payload[TELEMETRY_CMD_WIRE_SIZE],
                               uint16_t * len);
uint32_t mqtt_session_drops (void);

/* The lwIP owner calls this after servicing the Wi-Fi lease. */
void mqtt_session_poll (void);

#endif /* TELEMETRY_MQTT_SESSION_H */
