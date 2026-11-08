# Smoke test plan

Run the stages in order. Stages 1 and 2 are optional bring-up checks; the
normal smoke test is Stage 3 onward with `main.uf2`. Wheels stay off the floor
through Stage 5.

## Safety

- Use a current-limited supply or a fused battery lead.
- Keep the STOP control reachable. STOP is the reboot-only motion latch;
  power-cycle to clear it.
- Keep hands and cables clear of the wheels and the servo.
- Do not hot-plug the sonar or servo while the board is powered.

## Images

| Image | Purpose | Build | Flash |
|---|---|---|---|
| `main.uf2` | full stack | `./build.sh` | `./flash.sh` |
| `physics_check.uf2` | IMU/hump only (optional) | `make physics_check` | `./flash.sh build/physics_check.uf2` |
| `obstacle_bench.uf2` | obstacle only (optional) | `make obstacle_bench` | `./flash.sh build/obstacle_bench.uf2` |

The bring-up targets build inside `build/`:

```sh
nix develop -c ./configure.sh
nix develop -c ./build.sh
nix develop -c bash -lc 'cd build && make physics_check obstacle_bench'
```

Open the console with `picocom -b 115200 /dev/ttyACM0` (or `screen`). The
kernel USB CDC ignores the baud rate. If your user is not in `dialout`, run it
through sudo with the current PATH:

```sh
sudo env "PATH=$PATH" picocom -b 115200 /dev/ttyACM0
```

## Stage 0 — host checks

`configure.sh` detects the active Wi-Fi, its auth type, and the host's private
IPv4. It writes the credentials and broker host into `build/generated/`:

```sh
nix develop -c ./configure.sh
nix develop -c ./build.sh
```

Expect output like:

```text
configure: Wi-Fi '<ssid>' (WPA2-PSK)
configure: MQTT broker host default <host-ip>
```

`wpa-eap` and `sae` networks are not supported by the CYW43. The script warns
and leaves the Wi-Fi join off; use a WPA2-PSK hotspot or pass the `-D` flags.

Check the generated values:

```sh
rg 'UTK_WIFI_SSID|UTK_WIFI_AUTH' build/generated/wifi_credentials.h
rg 'TELEMETRY_BROKER_HOST|TELEMETRY_TOPIC_PREFIX' build/generated/telemetry_config.h
```

The broker host must be the host's current Wi-Fi IPv4. If the address changes,
re-run `./configure.sh`, rebuild, and reflash.

Run the host tests:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build
npm --prefix host/dashboard test
npm --prefix host/bridge test
npm --prefix host/dashboard run typecheck
npm --prefix host/bridge run typecheck
python3 -m venv .venv
.venv/bin/pip install paho-mqtt pyyaml
```

Pass: firmware builds, firmware host tests, dashboard tests, bridge tests, and
both typechecks pass.

## Stage 1 — IMU/hump bring-up (`physics_check`, optional)

Wheels up. Flash `build/physics_check.uf2`.

- Expect the `PHYS_H` init line and `PHYS_S` counters.
- The IMU initialises (`lsm303_init=0`); the CSV rows show pitch and word.
- Keep the board still for the first second (calibration).
- Tilt the board: pitch changes with the correct sign.
- Disconnect the IMU: the I2C fault appears in the word and counters.

Pass: no fault at rest, pitch tracks the board, I2C fault is reported.

## Stage 2 — obstacle bench (`obstacle_bench`, optional)

Wheels up. Flash `build/obstacle_bench.uf2`.

- Put a 100 mm box at 200 mm ahead, centred.
- Expect a `profile` line, the region around slots 9–15, fine slots 7/8/16/17,
  width 75–125 mm, and `SCAN_RESULT` CLEAR/OK.
- Move the box off-centre: the chosen side follows it.
- Remove the box: CLEAR.
- Cover the sonar: `NO_ECHO` after five pings.
- Hold ECHO high: `ECHO_STUCK`.
- Watch the servo: 60° sweep, no bind or buzz.

Pass: profile matches the box, no scan timeout, no fault.

## Stage 3 — full boot (`main`)

Wheels up. Flash `main.uf2`.

- Expect `[init]` lines without `startup failed`.
- `physics_init -> 0` and `obstacle_line_init -> 0`.
- The blink task runs.
- With the auto-detected Wi-Fi, `[conn]` reports `join=0 dhcp=0` and reaches
  ready.
- No `MOTION FAULT`.

Pass: every task starts, no fault.

## Stage 4 — sensors on the full stack

- Line sensors: pass a black card under the left, then the right sensor; the
  line bits change.
- Barcode: present a known Code 39 marker; expect `[barcode] X` and a telemetry
  event on topic `e`.
- System packet: the dashboard shows the physics and obstacle values from
  topic `t`.

Pass: line bits, barcode character and system fields are correct.

## Stage 5 — motion, wheels up

- With no line under the sensors, the line task drives straight; both wheels
  turn forward at about 250 mm/s.
- Check the encoder speeds on the dashboard or console; no `NO_EDGES` or
  `FEEDBACK` fault.
- Send STOP from the dashboard or CLI: the wheels stop, fault bit 9
  (`estop_latched`) is set, and Reset is disabled.
- Power-cycle to clear the latch; START runs again.

Pass: direction, speed, encoder validity and the emergency latch behave.

## Stage 6 — line following, floor

- Straight line at 250 mm/s: no weave and no lost line.
- Curve: the car follows without overshoot.
- Both sensors on the line (junction): the current code drives straight; note
  the behaviour.
- Neither sensor (gap): the current code drives straight; note the risk.

Pass: the car follows a straight line and a curve at cruise speed.

## Stage 7 — obstacle, floor

Set a straight line into a box, then a dead-end line with a box at the end.

- The car stops about 250 mm from the box and prints `OBSTACLE`.
- The scan prints a profile; the plan prints side, width and start angle.
- BYPASS turns away, HUG circles the box, and the car reacquires the line.
- Check the 60° reading holds near 300 mm; it must not chatter between the
  250/350 thresholds.
- No `SCAN_TIMEOUT` on a normal scan.
- The U-turn completes within the plan budget (2.5–8 m).
- A curved or oblique box: the reported width must not be cut short.
- A box too wide for the plan: `NO_SAFE_PATH`, then the 3 s retry.
- STOP during HUG: the car stops and bit 9 is set.

Pass: uniform clearance, complete U-turn, line reacquisition, safe stop.

## Stage 8 — telemetry and dashboard

Configure, build, and flash with the detected host:

```sh
./configure.sh
./build.sh
./flash.sh
```

`configure.sh` detects the active Wi-Fi and the host's private IPv4. Check the
generated broker host and topic before flashing:

```sh
rg 'TELEMETRY_BROKER_HOST|TELEMETRY_TOPIC_PREFIX' build/generated/telemetry_config.h
```

If the host IP changes, rebuild and reflash; the firmware cannot follow a new
DHCP address on its own.

Start the host stack:

```sh
./dev.sh
```

It starts the broker, the bridge, and the dashboard, prints their addresses,
and restarts a child that exits. Logs are under `logs/`:
`broker.log`, `bridge.log`, `expo.log`, `supervisor.log`, and
`state/history.jsonl`. Use `EXPO_CLEAR=1 ./dev.sh` for a clean Metro cache.

The current host address can also be read directly:

```sh
HOST_IP=$(ip -4 route get 1.1.1.1 2>/dev/null |
  awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')
```

Or copy the broker line printed by `./dev.sh`.

- Broker check: `mosquitto_sub -h "$HOST_IP" -t 'car/PICO-CAR/#' -v`.
  Expect `s` at 5 Hz, `t` at 2 Hz, `h` at 1 Hz, `e` for events.
- Open the printed dashboard URL (`http://$HOST_IP:8083`) on the host or
  phone. The script sets `EXPO_PUBLIC_ALLOW_INSECURE=1` for the LAN.
- Check live state, the onboard system panel, and the history page.
- Send STOP/START/RESET from the dashboard; STOP sets bit 9 and disables
  Reset.
- CLI: `.venv/bin/python host/cli/cli.py --host "$HOST_IP" monitor`.
- Allow TCP `1883`, `8090`, and `8083` on `wlan0` in the host firewall. The
  `dev.sh` start prints the same note.

### Telemetry reconnect

- The bridge is the WebSocket; `LINK OPEN` means the dashboard reached it.
- `BRIDGE UP` means the bridge reached the broker.
- `HEALTH STALE` means no fresh car status; the car link is down.
- The broker log is `logs/broker.log`; the bridge log is `logs/bridge.log`;
  `curl http://127.0.0.1:8090/health` shows the bridge MQTT state and packet
  count.
- The car prints `[conn] state=...` on state changes and
  `[conn] transport open failed` once. The TCP retry loop is silent, so watch
  the serial and the broker log for `pico-car`.
- On the serial, expect the Wi-Fi join, DHCP, and `[conn]` ready state.
- Outage: stop the broker or AP for 30 s. The car retries the TCP/MQTT session;
  the broker log shows a new `pico-car` client when it recovers. The dashboard
  reconnects its WebSocket and shows the bridge state.

Pass: packets, dashboard, commands, serial reconnect and broker reconnect work.

## Stage 9 — failure modes

- Sonar covered: `NO_ECHO`; ECHO held high: `ECHO_STUCK`.
- IMU unplugged: I2C fault in the physics word.
- Scan blocked: no settled slot for 3 s → `SCAN_TIMEOUT`.
- Full event queue: check the drop counters.
- Barcode edge overflow: check `ir_barcode_drops`.

Pass: each fault is reported and the car stops safely.

## Sign-off

Save the serial log, MQTT capture and dashboard screenshots under
`artifacts/logs/smoke-YYYYMMDD/`. Keep the `logs/` dev-stack logs alongside
the capture. Record the pass/fail per stage and file any failure with the file
and line.
