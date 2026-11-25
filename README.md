# Pico Car

Pico W firmware for the obstacle-extras line follower, sonar scan, bypass,
and rejoin. This branch reports through the USB console.

## Build and flash

From the repository root:

```sh
nix develop
./configure.sh
./build.sh
./flash.sh
```

`configure.sh` initializes the pinned Pico SDK and upstream micro T-Kernel.
Reconfigure when changing build settings or refreshing the embedded commit ID.
`build.sh` writes `build/main.uf2` and links it as `main.uf2` in the repository root.
Hold BOOTSEL while connecting USB to mount the Pico W as `RPI-RP2`. The flash
script also supports `picotool` and accepts an image path.

## Ground station

The dashboard, bridge, and CLI are available for working with dev-stack
recordings. This branch's firmware uses USB for diagnostics.

The Nix shell provides Node.js 24, npm, and Mosquitto.

```sh
npm run setup
npm start
```

Open <http://127.0.0.1:5281>. The bridge serves the dashboard, HTTP API, and
WebSocket on the same port. Use `npm run serve` to reuse the current web export.

- **Broker.** The station starts Mosquitto on TCP 1883 when needed. Set
  `MQTT_URL` to use a different broker.
- **Recordings.** The bridge records to `host/bridge/history.jsonl` and saves
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

- The car begins following the line after a three-second boot delay.
- GP21 latches the emergency stop until a reboot.
- The scanner uses a ±48° servo sweep around the 1450 μs center pulse.
  Verify clearance across that range before running this branch on the car.

See [TEAM.md](TEAM.md) for wiring and
[car dimensions](TEAM.md#car-dimensions).

## Development and testing

| Path | Purpose |
| --- | --- |
| [`app_program/`](app_program/) | Firmware, subsystem APIs, and bench images |
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

After changing the wire schema, run
`npm --prefix host/dashboard run generate`. Firmware C follows
[BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf)
with warnings treated as errors.
