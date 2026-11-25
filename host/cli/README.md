# Ground Station CLI

Ground-station CLI for the bridge HTTP API. It prints the decoded packets the
bridge keeps in its history and sends commands through the bridge's `POST
/command`. The bridge decodes the schema, numbers the commands and publishes
them, so the CLI needs neither a broker connection nor the protocol schema.

## Usage

```sh
python3 host/cli/cli.py --help
python3 host/cli/cli.py monitor
python3 host/cli/cli.py command start
```

The CLI uses the Python standard library only. It talks to
`http://127.0.0.1:5281` by default. Use `--url <bridge-url>` or set `BRIDGE_URL`
when the bridge runs elsewhere. `--url` is global, so it goes before the
subcommand. The bridge listens on `127.0.0.1` unless `BRIDGE_HTTP_HOST` says
otherwise.

`monitor` polls `GET /history` twice a second and prints each frame whose
`receiptOrder` is newer than the last one it printed. It starts from the newest
frame, so it shows no earlier history. `--limit` sets how many frames each poll
fetches (default 200); raise it if the car sends more than that between polls.
It keeps polling through a lost connection and prints the error:

```text
12:00:01 status    speed_mms=123 odometer_mm=456 heading_ddeg=900 state=2 mode=1 faults=0 seq=7 line_bits=0 hold_mask=0
12:00:01 heartbeat uptime_s=42 drops=0 wifi_rssi=-55 reset_reason=0 link_state=3 run_id=4660 car_id=3238001904
```

The time is the bridge's receipt time.

Current firmware sends one 5 Hz status frame on `/s` with vehicle state,
motion, attitude and forward sonar. The CLI prints the bundle on one line
with its shared sequence and device timestamp. The bridge also reads separate
legacy status, motion and physics frames. Update the bridge before flashing
firmware that sends the bundle.

`command` posts `{"action": "<name>"}` and prints `sent <name> cmd_id=<id>` on
the bridge's 202 reply. The bridge adds the run ID, the command ID and the
heartbeat uptime, and refuses with 503 when it has no fresh heartbeat and diag
frame. The CLI exits 1 on any HTTP or connection error and prints the status and
the bridge's message. The command is a 9-byte frame: `cmd_id u16, run_id u16,
opcode u8, uptime_s u32`. The car refuses another run ID, a cmd_id that is not
newer than the last one it accepted, and an uptime echo more than 3 s old.

Diagnostics use `/d` in both layouts. The current 73-byte frame sends
cumulative line and sonar counters with the device timer. The bridge
also reads the 45-byte legacy frame. Update the bridge before flashing
firmware that sends the current layout so command correlation continues to work.

## Floor precondition

Autonomous runs assume a verified continuous supporting floor without drop-offs.
Verify the course surface before sending `command start`. Both `command start`
and the compatibility alias `command start_continuous_floor` arm continuous-floor
obstacle hugging. Stationary start (`start_stationary`, wire opcode 10) has been
withdrawn and is refused by the bridge and firmware.
