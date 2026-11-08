# Smoke test plan

Run the stages in order. Stop at the first failed stage and record the serial
log. Wheels stay off the floor through Stage 5.

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
| `physics_check.uf2` | Buddy 4 only | `make physics_check` | `./flash.sh build/physics_check.uf2` |
| `obstacle_bench.uf2` | Buddy 5 only | `make obstacle_bench` | `./flash.sh build/obstacle_bench.uf2` |

The bring-up targets build inside `build/`:

```sh
nix develop -c ./configure.sh
nix develop -c ./build.sh
nix develop -c bash -lc 'cd build && make physics_check obstacle_bench'
```

Open the console with `picocom -b 115200 /dev/ttyACM0` (or `screen`). The
kernel USB CDC ignores the baud rate.

## Stage 0 — host checks

```sh
nix develop -c ./configure.sh
nix develop -c ./build.sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build
npm --prefix host/dashboard run typecheck
npm --prefix host/bridge run typecheck
```

Pass: firmware builds, 8 host tests pass, both typechecks pass.

## Stage 1 — Buddy 4 bring-up (`physics_check`)

Wheels up. Flash `build/physics_check.uf2`.

- Expect `PHYS_H,b4_task_init=0` and `PHYS_S` counters.
- The IMU initialises (`lsm303_init=0`); the CSV rows show pitch and word.
- Keep the board still for the first second (calibration).
- Tilt the board: pitch changes with the correct sign.
- Disconnect the IMU: the I2C fault appears in the word and counters.

Pass: no fault at rest, pitch tracks the board, I2C fault is reported.

## Stage 2 — Buddy 5 bench (`obstacle_bench`)

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
- With Wi-Fi configured, `[conn]` reports `join=0 dhcp=0` and reaches ready.
- No `MOTION FAULT`.

Pass: every task starts, no fault.

## Stage 4 — sensors on the full stack

- Line sensors: pass a black card under the left, then the right sensor; the
  line bits change.
- Barcode: present a known Code 39 marker; expect `[barcode] X` and a
  telemetry event on topic `e`.
- System packet: the dashboard shows the B4 and B5 values from topic `t`.

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

Build with credentials and a reachable broker:

```sh
./configure.sh \
  -DPICO_CAR_MQTT_HOST=192.168.1.10 \
  -DPICO_CAR_WIFI_SSID=your-ssid \
  -DPICO_CAR_WIFI_PASSWORD=your-password
./build.sh
```

- Broker: `mosquitto_sub -h 192.168.1.10 -t 'car/SE01/#' -v`.
  Expect `s` at 5 Hz, `t` at 2 Hz, `h` at 1 Hz, `e` for events.
- Bridge: `MQTT_URL=mqtt://192.168.1.10:1883 npm --prefix host/bridge start`.
- Dashboard: `npm --prefix host/dashboard run web` with
  `EXPO_PUBLIC_LINK_URL=ws://localhost:8080`,
  `EXPO_PUBLIC_SERVICE_URL=http://localhost:8080`.
- Check live state, the onboard system panel, and the history page.
- Send STOP/START/RESET from the dashboard; STOP sets bit 9 and disables
  Reset.
- CLI: `python3 host/cli/cli.py --host 192.168.1.10 monitor`.
- Outage: stop the broker or AP for 30 s; the connection reconnects.

Pass: packets, dashboard, commands and reconnect work.

## Stage 9 — failure modes

- Sonar covered: `NO_ECHO`; ECHO held high: `ECHO_STUCK`.
- IMU unplugged: I2C fault in the B4 word.
- Scan blocked: no settled slot for 3 s → `SCAN_TIMEOUT`.
- Full event queue: check the drop counters.
- Barcode edge overflow: check `ir_barcode_drops`.

Pass: each fault is reported and the car stops safely.

## Sign-off

Save the serial log, MQTT capture and dashboard screenshots under
`artifacts/logs/smoke-YYYYMMDD/`. Record the pass/fail per stage and file any
failure with the file and line.
