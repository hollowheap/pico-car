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

`wpa-eap` and `sae` networks are not supported by the CYW43. The CYW43439
is also 2.4 GHz only. When the active BSS is 5/6 GHz and the same SSID is not
visible on 2.4 GHz, the script warns, lists the 2.4 GHz SSIDs in range, and
leaves the Wi-Fi join off. Connect the host to the router's 2.4 GHz SSID, or
pass the `-D` flags with a 2.4 GHz SSID and password.

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

- Expect no `[ERROR](boot)` line. A line such as `[ERROR](boot) startup failed, er=-33` means a task did not start.
- The blink task runs.
- With the auto-detected Wi-Fi, `[INFO](connection) state=...` reports `join=0 dhcp=0` and reaches
  ready.
- No `[ERROR](motion) fault=...` line.
- The console format is in [`docs/debugging.md`](debugging.md).

Pass: every task starts, no fault.

## Stage 4 — sensors on the full stack

- Line sensors: pass a black card under the left, then the right sensor; the
  line bits change.
- Barcode: present a known Code 39 marker; expect `[INFO](barcode) read X, action N`
  and a `barcode` event on topic `e` (p1 is the ASCII letter, p2 the action).
- Physics and obstacle packets: the dashboard shows the physics values from
  topic `p` and the obstacle values from topic `o`.

Pass: line bits, barcode character, and the physics and obstacle fields are correct.

## Stage 4b — supervisor and barcode actions

Wheels up. Run the tests S1 to S7 in [`docs/supervisor.md`](supervisor.md#test-on-the-car).

- A barcode `S` (straight), `L`, `R`, `U` or `H` (halt) gives the `barcode` event on topic `e` and the matching `cmd_accept` event.
- Any other letter gives the `barcode` event with action 5 (rejected) and no motion.
- Lift the car by hand: fault bit 11 shows and the wheels stop.
- Leave the watchdog off (`-DPICO_CAR_WATCHDOG` not set) until test W1 passes.

Pass: every command is accepted or rejected with an event, and no command moves the wheels while the car is held.

## Stage 5 — motion, wheels up

- With no line under the sensors, the line task holds its heading for 100 mm,
  then weaves at reduced speed (about 100 mm/s). It does not stop (D180). Both
  wheels turn forward at cruise (163 mm/s) while it holds the heading.
- Check the encoder speeds on the dashboard or console; no `NO_EDGES` or
  `FEEDBACK` fault.
- Send E-stop from the dashboard or CLI: the wheels stop, fault bit 9
  (`estop_latched`) is set, and Clear faults is disabled.
- Power-cycle to clear the latch; START runs again.

Pass: direction, speed, encoder validity and the emergency latch behave.

## Stage 6 — line following, floor

- Straight line at cruise (163 mm/s): a small weave is expected with the sensors
  35 mm apart. Count the sensor changes per second on the Line panel; run
  `car_bench` LINEA and LINEB first for the baseline (`docs/bench.md`).
- Curve of 400 mm and 200 mm radius: the car follows without leaving the line.
- Gap: cut a 60 mm gap in the line. The car crosses it on its heading.
- Longer gap (300 mm): the Line panel shows `line_lost`, the car weaves, and it
  follows again at the next sensor hit.
- Junction with a barcode `S`: the car drives 250 mm straight and follows the
  line again. With `L`, `R` or `U` it turns.

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
  Expect `s`, `m` and `p` at 5 Hz, `o` at 2 Hz, `d` and `h` at 1 Hz, `e` for events,
  and `f` once per obstacle scan.
- Open the printed dashboard URL (`http://$HOST_IP:8083`) on the host or
  phone. The script sets `EXPO_PUBLIC_ALLOW_INSECURE=1` for the LAN.
- Check live state, the onboard state panel, and the history page.
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
- The car prints `[INFO](connection) state=...` on state changes and
  `[WARN](connection:tcp)` lines when a connection attempt fails. Watch
  the serial and the broker log for `pico-car`.
- On the serial, expect the Wi-Fi join, DHCP, and the `[INFO](connection)` ready state.
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

## Stage 10 — what only you can check

The code cannot see these. Do them with the benches in [`docs/bench.md`](bench.md). Write each result in the question named in the last column of `artifacts/questions.md`.

| # | Check | How | Pass when | Question |
|---|---|---|---|---|
| 10.1 | Line sensor lead and spacing | Done: 130 mm lead, 25 mm spacing. Measure again only if you move the sensors | The model is re-run with new numbers | Q71 |
| 10.2 | Sensor comparator trimpots | `car_bench` SENS: slide the car over the line by hand | Each sensor switches at the middle of the edge, with few raw edges beside the filtered ones | Q71 |
| 10.3 | Wheel direction and encoder plugs | `car_bench` ENC with the wheels off the floor | Both wheels turn forward, both verdicts `OK` | none |
| 10.4 | Wheel lag | `car_bench` STEP | 63% time near 60 ms, or the model is re-run with the real value | Q72 |
| 10.5 | Old and new line law | `car_bench` LINEA then LINEB on the same 1.5 m straight | Cascade changes per second under 0.65 of the legacy value | Q72 |
| 10.6 | Watchdog and H-bridge | `wdt_check` (test W1) | The wheels stop at the reset and stay stopped | Q61 |
| 10.7 | Lift cue | `car_bench` LIFT | The cue sets in the air and clears on the floor | none |
| 10.8 | Hump FLAT checks | `physics_check`, then `physics_log_summary.py --checks` | Checks 1 to 3 pass; check 4 by hand (see `docs/bench.md`) | none |
| 10.9 | Real ramp | Done: 10 mm high over 100 mm, about 6 degrees at most, less across the wheelbase | The hump detector enters at 4 degrees, so it can see this ramp; the 6 degree limit could not | none |
| 10.10 | Servo travel | `obstacle_bench` SERVO CHECK on a square wall | Verdict OK, or tune `OBSTACLE_SERVO_US_PER_10DEG` and the centre | none |
| 10.11 | Sonar beam edge and incidence | `obstacle_bench` scan of a board at 230 mm; slide it across | The angle where the face stops returning is written down | Q56 |
| 10.12 | TRIG readback | Obstacle checklist H7e | Good pings read VALID or OPEN | Q62 |
| 10.13 | Battery during braking | Multimeter on the battery rail while the car brakes hard from cruise | The rail stays above the Pico brown-out level and the powerbank does not sleep | Q12 |
| 10.14 | Barcode reads at speed | Run over each code at cruise and while the car slows | Every pass gives a `barcode` event; a `barcode_bad` reason 1 (element) means a bar fell outside the width bands | Q23 |
| 10.15 | Crossing length | Only if the track has crossings: the length from the barcode to the far side of the crossing | Under 250 mm (`LINE_STRAIGHT_MM`), or change the constant | none |
| 10.16 | Wi-Fi coverage | Carry the car (powered, wheels held) to each corner of the course | RSSI above -75 dBm on the Link panel | none |
| 10.17 | Stack headroom | Run 30 s with the serial monitor open | Every `stack free` line shows at least 512 bytes | Q25 |

Pass: every line has a written result.

## Sign-off

Save the serial log, MQTT capture and dashboard screenshots under
`artifacts/logs/smoke-YYYYMMDD/`. Keep the `logs/` dev-stack logs alongside
the capture. Record the pass/fail per stage and file any failure with the file
and line.
