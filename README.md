# Pico Car

Pico W firmware with the shared line follower, barcode routing, and the
obstacle-extras sonar scan, bypass, and rejoin. The firmware sends telemetry
and receives commands over Wi-Fi and MQTT.

## Build and flash

From the repository root:

```sh
nix develop
./configure.sh
./build.sh
./flash.sh
```

`configure.sh` initializes the pinned Pico SDK and upstream micro T-Kernel.
Kernel limits such as `CNF_MAX_TSKID` are set in `config/config.h`, which
replaces the kernel's own copy. The telemetry and Wi-Fi header templates in
`config/` are filled from the CMake settings below.
Reconfigure when changing build settings or refreshing the embedded commit ID.
`build.sh` writes `build/main.uf2` and links it as `main.uf2` in the repository root.
The course template's entry point also works: run `make -j8` in
`build_make/` inside `nix develop`. It builds the same CMake target and copies
it to `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`. Only the defaults
`SMP=1 CONSOLE=usb_cdc WIFI=cyw43` are accepted. `CRASH_REPORT=1` is refused
because the sonar uses GP0 and GP1.
Hold BOOTSEL while connecting USB to mount the Pico W as `RPI-RP2`. The flash
script also supports `picotool` and accepts an image path.

The Pico W uses 2.4 GHz Wi-Fi with WPA2-PSK or an open network. Provide the LAN
broker address and car login through the `PICO_CAR_MQTT_HOST`,
`PICO_CAR_MQTT_USERNAME`, and `PICO_CAR_MQTT_PASSWORD` environment variables or
local CMake cache. The same broker settings can be used for dev-stack and
obstacle-extras.

## Ground station

The dashboard, bridge, and CLI receive live readings and send driving
commands. The course plots the estimated axle path from recorded odometry and
heading, with located event markers. Tape signals, sonar readings and attitude
stay in their own panels. Read the extras flight log through the USB serial
console while the car is parked.

The Nix shell provides Node.js 24, npm, and Mosquitto.

```sh
npm run setup
npm start
```

Open <http://127.0.0.1:5281>. The bridge serves the dashboard, HTTP API, and
WebSocket on the same port. Use `npm run serve` to reuse the current web export.

- **Broker.** The station starts Mosquitto on TCP 1883 when needed. Set
  `MQTT_URL` to use a different broker.
- **Telemetry.** The 5 Hz `/s` packet carries status, wheel motion, attitude
  and the age and class of the current centered sonar shot. Obstacle scans,
  barcodes and hump completion retain their event streams. The Obstacle and
  Dynamics cards retain each reading across START and STOP until another
  obstacle or hump is detected. Dynamics marks rejected heights as invalid.
- **Recordings.** The bridge records to `testing/logs/bridge/history.jsonl` and saves
  course geometry in a `.course` sidecar. Set `BRIDGE_HISTORY_PATH` to select
  another file. Use one writer per recording.
- **CLI.** Run `python3 host/cli/cli.py monitor`. See the
  [CLI command reference](host/cli/README.md) and [bridge API](host/bridge/README.md).
- **Remote access.** Use HTTPS with WebSocket upgrades at the proxy. For a
  private HTTP station on a non-loopback address, set
  `EXPO_PUBLIC_ALLOW_INSECURE=1` when exporting the dashboard.

To run the station in containers:

```sh
podman-compose -f deploy/compose.yaml up -d --build
```

Set `BROKER_PORT` (1883), `DASHBOARD_LOCAL_PORT` (5281), or the private HTTP
build option in `deploy/.env`. The dashboard binds to loopback and the
`bridge-data` volume keeps its recordings.

## Run the car

- Use dashboard START or press GP20 to begin after the car reports ready.
- Dashboard STOP parks the car. Use START to continue.
- GP21 or dashboard E-stop latches the emergency stop until a reboot.
- Dashboard turn commands and barcodes L, R, S, and U wait for the next
  confirmed junction. U returns along the approach tape. H parks the car at
  the marker.
- A confirmed pickup holds the wheels. Put the car down level, wait for a
  second of stillness, then use START. Wheel movement restarts the settling window.
- Sonar faults remain visible in diagnostics. The car can start and follow
  tape when readings are missing, stale, or faulty. Recent close echoes still
  engage obstacle handling.
- The scanner uses a ±48° servo sweep around the 1450 μs center pulse.
  Verify clearance across that range before running this branch on the car.

See [TEAM.md](TEAM.md) for wiring and
[car dimensions](TEAM.md#car-dimensions).

## Development and testing

| Path | Purpose |
| --- | --- |
| [`app_program/`](app_program/) | Firmware and subsystem APIs |
| [`bench/`](bench/) | Bench check images |
| [`tests/`](tests/) | CTest suites and hardware fakes |
| [`host/`](host/) | Dashboard, bridge, and CLI |
| [`protocol/telemetry.yaml`](protocol/telemetry.yaml) | MQTT topics, packet layouts, and events |
| [`testing/`](testing/README.md) | Test procedures, run records, and local captures |

```sh
cmake -S tests -B build-tests
cmake --build build-tests -j8
ctest --test-dir build-tests --no-tests=error
bash lint.sh
```

Use the [testing guide](testing/README.md) for host and bench checks.
Save captures under `testing/logs/`. See the
[course calibration guide](testing/course-calibration.md) for measured runs
and trace replay.

The default barcode bindings are H and Z for STOP, L for left, R for right,
S for straight, and U for a U-turn. Open **Edit barcode mapping** in the
dashboard to save different letters. The bridge sends the saved map to the
car, which executes it locally. A car that boots offline uses the defaults
until it can receive the saved map.
[Barcode bindings](app_program/ir/barcode/action_config.h)

The shared barcode decoder qualifies a closing bar across the same optical
glitch window used to merge split elements. Capture loss during confirmation
aborts the read. The `ir.barcode_task_reject` host cases exercise decoding and
command output.

After changing the wire schema, run
`npm --prefix host/dashboard run generate`. Firmware C follows
[BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf)
with warnings treated as errors.
