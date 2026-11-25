# Pico Car

Pico W firmware for a line-following car with barcode routing, sonar obstacle
navigation, and a LAN ground station.

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

The car uses 2.4 GHz Wi-Fi with WPA2-PSK or an open network. The configure script
can detect the network and broker address through NetworkManager. Override them
with `PICO_CAR_WIFI_SSID`, `PICO_CAR_WIFI_PASSWORD`, and `PICO_CAR_MQTT_HOST`
as local CMake settings. The car needs the broker's LAN address. Keep
credentials in the environment or local build cache.

## Broker credentials

The broker refuses anonymous clients. `configure.sh` runs `host/credentials.sh`,
which creates random 192-bit passwords for two broker users, `car` and
`bridge`, when they are missing. Run it by hand before the first
`npm start` or compose start:

```sh
host/credentials.sh
```

The files live in `host/credentials/`, which Git ignores:

| File | Holds | Read by |
| --- | --- | --- |
| `car.env` | the car's login in plaintext | `configure.sh`, for the CMake cache |
| `bridge.env` | the bridge's login in plaintext | the bridge and the compose bridge |
| `passwd` | salted PBKDF2 hashes for both users | the broker |
| `acl` | topic permissions | the broker |
| `car-id` | the pinned car ID | the bridge |

The car may publish only its telemetry topics and subscribe only to its
command topic. The bridge may subscribe to every topic under the prefix and
publish only the command topic. Set `PICO_CAR_TOPIC_PREFIX` when the prefix is
not `car/PICO-CAR`. The script rewrites `acl` on every run.

Existing files stay as they are. To replace a password, delete `car.env` or
`bridge.env`, rerun `host/credentials.sh`, then run `./configure.sh` and flash
the car again. The firmware must hold its own password in plaintext in flash
and in the build cache, while the broker stores only hashes. Anyone who can read
the car's flash or `host/credentials/` can publish as the car.

### Pair a car

Each car reports a fingerprint: the last four bytes of its radio MAC, in the
heartbeat `car_id` field and at the end of its MQTT client ID
(`pico-car-<eight hex digits>`). The bridge pins the first car it hears, or the
ID in `CAR_ID`, and saves it in `host/credentials/car-id`. While a heartbeat
with another ID is arriving, the bridge drops all telemetry and refuses
commands with 409. `GET /health` shows `car.expected`, `car.reporting` and
`car.mismatch`. To pair a different car, power off the old one, delete
`host/credentials/car-id` (or change `CAR_ID`), restart the bridge, and power on
the new car. The ID identifies a car; the password authenticates it.

## Ground station

The Nix shell provides Node.js 24, npm, and Mosquitto.

```sh
npm run setup
npm start
```

Open <http://127.0.0.1:5281>. The bridge serves the dashboard, HTTP API, and
WebSocket on the same port. Use `npm run serve` to reuse the current web export.

- **Broker.** The station starts Mosquitto on TCP 1883 when needed, with the
  files in `host/credentials/`, and logs in as `bridge`. Set `MQTT_URL` to use a
  different broker, and `MQTT_USERNAME` and `MQTT_PASSWORD` for its login.
  Match the firmware's `PICO_CAR_TOPIC_PREFIX` to the bridge's
  `MQTT_TOPIC_PREFIX`. Both default to `car/PICO-CAR`.
- **Recordings.** The bridge records to `host/bridge/history.jsonl` and saves
  course geometry in a `.course` sidecar. Set `BRIDGE_HISTORY_PATH` to select
  another file. Use one writer per recording.
- **CLI.** Run `python3 host/cli/cli.py monitor`. See the
  [CLI command reference](host/cli/README.md) and [bridge API](host/bridge/README.md).
- **Remote access.** Use HTTPS with WebSocket upgrades at the proxy. For a
  private HTTP station on a non-loopback address, set
  `EXPO_PUBLIC_ALLOW_INSECURE=1` when exporting the dashboard.

Firmware publishes one [75-byte telemetry bundle](protocol/telemetry.yaml)
on `/s` every 200 ms. It carries status, wheel motion, attitude, and the
latest qualified forward sonar shot when available. The bundle has one
sequence and a device timestamp for the telemetry pass. Encoder and sonar
ages identify their respective acquisition times. The bridge still decodes
legacy status, motion, and physics packets from older firmware and recordings.

To run the station in containers:

```sh
host/credentials.sh
podman-compose -f deploy/compose.yaml up -d --build
```

The broker container mounts only `passwd` and `acl` from `host/credentials/`
read-only. The bridge container takes its login from `bridge.env` and keeps its
pinned car ID in the `bridge-data` volume. Run the broker rootless, because the
credential files are mode 0600. Set `BROKER_PORT` (1883), `DASHBOARD_LOCAL_PORT` (5281), or the private HTTP
build option in `deploy/.env`. The dashboard binds to loopback and the
`bridge-data` volume keeps its recordings.

## Run the car

- **START.** Use GP20, the dashboard, or the CLI. Set the car upright and still
  for IMU calibration. With no MQTT session, the car can start automatically
  after a power-up wait of at least 10 seconds once its checks pass. An armed
  run can also resume after a watchdog reset when the forward path is clear.
- **Course.** START defaults to continuous supporting floor. Use **Stop at
  obstacles** in the dashboard for courses with possible drop-offs.
- **E-STOP.** GP21 or an accepted E-STOP command latches the drive inhibit.
  Reboot to clear it. Use the bench procedure to measure electrical cutoff and
  stopping distance.
- **Barcodes.** L, R, and S request the left, right, or through exit at the
  next junction. The car moves the axle over the measured stripe before
  pivoting. It keeps the instruction if the requested exit is unavailable.
  U turns back at the marker.

See [TEAM.md](TEAM.md) for wiring, pin ownership, and task priorities. The
[car dimensions](TEAM.md#car-dimensions) use values from
[chassis.h](app_program/platform/chassis.h).

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
