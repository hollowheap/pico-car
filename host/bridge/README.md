# LAN Bridge

Node/TypeScript backend for MQTT ingestion and dashboard state.

## Run

From the repo root, `podman-compose -f deploy/compose.yaml up -d --build` starts
the broker, this bridge and the dashboard in containers. To run the bridge alone, start a broker on `localhost:1883` and
run `npm start` in this directory.

Environment:

- `MQTT_URL` — default `mqtt://localhost:1883`.
- `MQTT_USERNAME`, `MQTT_PASSWORD` — optional broker credentials.
- `MQTT_TOPIC_PREFIX` — default `car/PICO-CAR`.
- `BRIDGE_HTTP_PORT` — default `8080`.
- `PROTOCOL_SCHEMA` — default `../../protocol/telemetry.yaml`.

`GET /health` returns MQTT connection state and the last decoded packet.

`POST /command` takes `{"action": "<name>"}` with a command name from the
protocol schema. The bridge numbers each command and adds the run ID and
the uptime from the latest heartbeat. The car picks a new run ID at each boot
and refuses a command with another one, a command id that is not newer than
the last one it accepted, or an uptime echo more than 3 s old. The bridge
moves its counter past the `cmd_last_id` of the diag frame when the car would
refuse it. The bridge refuses a command when no heartbeat arrived in the last
3 s or no diag frame arrived yet.

`POST /command` returns 202 only after the broker acknowledges the QoS 1
publish. It returns 502 when the publish fails, 503 when MQTT is
disconnected, no heartbeat is fresh or no diag frame arrived, and 400 for an invalid body. A 202
means the bridge sent the frame; it does not confirm that the car applied
the command.

History storage and HTTPS/WSS transport are follow-up tasks.
