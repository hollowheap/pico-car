# LAN Bridge

Node/TypeScript backend for MQTT ingestion and dashboard state.

## Run

```sh
MQTT_URL=mqtt://localhost:1883 PROTOCOL_SCHEMA=../../protocol/telemetry.yaml npm start
```

Environment:

- `MQTT_URL` — required broker URL.
- `MQTT_USERNAME`, `MQTT_PASSWORD` — optional broker credentials.
- `MQTT_TOPIC_PREFIX` — default `car/SE01`.
- `BRIDGE_HTTP_PORT` — default `8080`.
- `PROTOCOL_SCHEMA` — default `../../protocol/telemetry.yaml`.

`GET /health` returns MQTT connection state and the last decoded packet.

History storage and HTTPS/WSS transport are follow-up tasks.
