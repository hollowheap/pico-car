# Ground-station server

The same Node server serves the exported dashboard, ingests MQTT, records
telemetry, streams WebSocket updates and accepts dashboard commands.

## Run

Run `npm run setup` once, then `npm start` from the repository root.
`npm run serve` reuses the current web export. The station starts a local
Mosquitto process when needed. Providing `MQTT_URL` selects an external
broker. For backend development, `npm start` in this directory connects
to an existing broker and can run before the first dashboard export.

Environment:

- `MQTT_URL` — default `mqtt://localhost:1883`.
- `MQTT_USERNAME`, `MQTT_PASSWORD` — broker login. When unset, the bridge reads
  `bridge.env` from `host/credentials` (see `PICO_CAR_CREDENTIALS_DIR`).
- `CAR_ID` — expected car ID, eight hex digits. When unset, the bridge reads
  `car-id` from the credentials directory (`CAR_ID_PATH` moves it) and writes
  the first car it hears there.
- `MQTT_TOPIC_PREFIX` — default `car/PICO-CAR`.
- `BRIDGE_HTTP_HOST`, `BRIDGE_HTTP_PORT` — default `127.0.0.1:5281`.
- `PROTOCOL_SCHEMA` — defaults to the repository's `protocol/telemetry.yaml`.
- `BRIDGE_HISTORY_PATH` — defaults to `host/bridge/history.jsonl`.

`GET /health` returns MQTT connection state and the last decoded packet. Its
`car` object holds `expected` (the pinned ID or null), `reporting` (the ID in
the newest heartbeat), `mismatch` and `rejected`.

The heartbeat `car_id` is the last four bytes of the car's radio MAC, and the
car's MQTT client ID ends with it. The first heartbeat pins the car when no ID
is configured. After a heartbeat with another ID, the bridge drops every packet
and refuses commands with 409 until that heartbeat has stopped for 3 s. Delete
`car-id` or change `CAR_ID` to pair a different car.

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
disconnected, no heartbeat is fresh or no diag frame arrived, 409 when a car with
another ID is reporting, and 400 for an invalid body. A 202
means the bridge sent the frame; it does not confirm that the car applied
the command.

`GET /history?limit=100` returns recorded telemetry. `GET /view` returns the
current console snapshot, including the derived course, events, run summary
and recent frames. A WebSocket client can set `?console=1` to receive a full
`console` snapshot on connect and `consoleUpdate` messages with changes.
The raw packet stream uses the same WebSocket without that query parameter.
An HTTPS reverse proxy must forward WebSocket upgrades for WSS clients.
