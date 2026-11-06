# Ground Station CLI

Direct-MQTT ground-station CLI. It consumes `protocol/telemetry.yaml`, decodes
status/event/heartbeat frames, and sends validated commands without going
through the dashboard backend (D41).

## Usage

```sh
python3 host/cli/cli.py --help
python3 host/cli/cli.py monitor --host 192.168.1.10
python3 host/cli/cli.py command stop --host 192.168.1.10 --run-id 1
```

Defaults: broker `localhost:1883`, topic prefix `car/SE01`, keepalive 30 s.
Override `--username` / `--password` for authenticated brokers.

`monitor` reconnects every second after a loss and prints decoded frames:

```text
12:00:01 status    speed_mms=123 dist_mm=456 heading_deg=90 state=2 faults=0
12:00:01 heartbeat uptime_s=42 drops=0 wifi_rssi=-55
```

`command` subscribes to the 1 Hz heartbeat to synchronize `sent_ms` with the
device clock, then publishes QoS 1 and waits for PUBACK. The command is an
8-byte frame: `cmd_id u16, run_id u16, sent_ms u16, opcode u8, reserved u8`.
Override the synchronized value with `--sent-ms` for controlled tests.

## Outage check

`artifacts/scripts/test_wifi_outage.py` starts a loopback broker, publishes
telemetry, drops the link for 30 seconds, and checks reconnect (NFR6), live
decode, and the CLI command path including heartbeat synchronization. Set `PICO_CAR_CLI_DIR` and
`PICO_CAR_SCHEMA` when running it against a worktree.
