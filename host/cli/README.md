# Ground Station CLI

Direct-MQTT ground-station CLI. It consumes `protocol/telemetry.yaml`, decodes
the schema 3 packets (status, motion, physics, obstacle, diag, event,
heartbeat, profile and barcode sample), and sends validated commands without going
through the dashboard backend (D41).

## Usage

```sh
python3 host/cli/cli.py --help
python3 host/cli/cli.py monitor
python3 host/cli/cli.py command start --run-id 0
```

The default broker host is this host's default-route IPv4. Use
`--host <broker-ip>` when the broker runs elsewhere.

`--host` and the other broker options are global, so they go before the
subcommand. The firmware starts with run ID 0; the CLI default is already 0.

Dependencies:

```sh
python3 -m venv .venv
.venv/bin/pip install paho-mqtt pyyaml
.venv/bin/python host/cli/cli.py monitor
```

Defaults: broker `<this host>:1883`, topic prefix `car/PICO-CAR`, keepalive
30 s. Override `--username` / `--password` for authenticated brokers.

`monitor` reconnects every second after a loss and prints decoded frames:

```text
12:00:01 status    speed_mms=123 odometer_mm=456 heading_ddeg=900 state=2 mode=1 faults=0 seq=7 line_bits=0 hold_mask=0
12:00:01 heartbeat uptime_s=42 drops=0 wifi_rssi=-55 reset_reason=0 link_state=3 run_id=4660
```

`command` reads the run ID and the uptime from the 1 Hz heartbeat and
`cmd_last_id` from the 1 Hz diag frame, then publishes QoS 1 and waits for
PUBACK. The command is a 9-byte frame: `cmd_id u16, run_id u16, opcode u8,
uptime_s u32`. The car picks a new run ID at each boot and refuses a command
with another one. It also refuses a cmd_id that is not newer than the last
one it accepted, so the CLI sends `cmd_last_id + 1`, and an uptime echo more
than 3 s old. Override the ids with `--run-id` and `--cmd-id` for controlled
tests; the uptime always comes from the heartbeat.

Diagnostics use `/d` in both layouts. The current 73-byte frame sends
cumulative line and sonar counters with the device timer. The CLI and bridge
also read the 45-byte legacy frame. Update the CLI and bridge before flashing
firmware that sends the current layout so command correlation continues to work.

## Outage check

The outage check script `test_wifi_outage.py` (kept outside the repository) starts a loopback broker, publishes
telemetry, drops the link for 30 seconds, and checks reconnect (NFR6), live
decode, and the CLI command path including the heartbeat run ID. Set `PICO_CAR_CLI_DIR` and
`PICO_CAR_SCHEMA` when running it against a worktree.

## Floor precondition

Autonomous runs assume a verified continuous supporting floor without drop-offs.
Verify the course surface before sending `command start`. Both `command start`
and the compatibility alias `command start_continuous_floor` arm continuous-floor
obstacle hugging. Stationary start (`start_stationary`, wire opcode 10) has been
withdrawn and is refused by the CLI, bridge, and firmware.
