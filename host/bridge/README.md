# LAN Bridge

Node/TypeScript backend for MQTT ingestion and dashboard state.

## Run

From the repo root, `./dev.sh` starts the broker, this bridge, and the
dashboard. To run the bridge alone, start a broker on `localhost:1883` and
run `npm start` in this directory.

Environment:

- `MQTT_URL` — default `mqtt://localhost:1883`.
- `MQTT_USERNAME`, `MQTT_PASSWORD` — optional broker credentials.
- `MQTT_TOPIC_PREFIX` — default `car/PICO-CAR`.
- `BRIDGE_HTTP_PORT` — default `8080`.
- `PROTOCOL_SCHEMA` — default `../../protocol/telemetry.yaml`.

`GET /health` returns MQTT connection state and the last decoded packet.

`POST /command` returns 202 only after the broker acknowledges the QoS 1
publish. It returns 502 when the publish fails, 503 when MQTT is
disconnected, and 400 for an invalid body. A 202 means the bridge sent the
frame; it does not confirm that the car applied the command.

History storage and HTTPS/WSS transport are follow-up tasks.
