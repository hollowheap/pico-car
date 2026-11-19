## Building

Run these commands:

```sh
nix develop     # Enter devshell with build tools and pico SDK installed
./configure.sh  # Init submodules + configure CMake
./build.sh      # Compile and link
./flash.sh      # Load main.uf2 over USB
```

The final binary is hardlinked inside the base directory as `main.uf2`.
Disassembly and other files can be found in the `build/` folder.

Hold BOOTSEL while plugging the Pico W. If it mounts as `RPI-RP2`, `flash.sh`
copies the UF2 directly, with no sudo. Otherwise it falls back to `picotool`.
Pass an optional image for the bring-up builds:

```sh
./flash.sh build/physics_check.uf2
./flash.sh build/obstacle_bench.uf2
./flash.sh build/car_bench.uf2
./flash.sh build/wdt_check.uf2
```

When developing, `configure.sh` should be run before the first build or whenever
`CMakeLists.txt` changes. It is unnecessary to run it otherwise; Running
`build.sh` is enough.

Build notes:

- `configure.sh` initializes submodules before configuring CMake.
- `CMakeLists.txt` pins `PICO_SDK_PATH` to the vendored `lib/pico-sdk` so a
  devshell environment variable cannot override it.
- Kernel translation units build with `-std=gnu11`; only `src/` is held to
  BARR-C C99 with `-Werror`.
- The kernel linker script asserts `.boot2` is exactly 256 bytes. Link with
  `-Wl,--undefined=boot2` so the member is not dropped from the archive.
- Hand-rolled firmware targets need `-ffunction-sections` and `-fdata-sections`;
  otherwise `--gc-sections` cannot drop unused SDK code.

## Developing

All C and C++ code in this repository must comply with the
[BARR-C:2018 standard](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).

When creating a new file, use the templates in `templates/` for consistency.

## Layout

Public subsystem headers live in `include/<subsystem>/`; implementations live in
`src/<subsystem>/`. `src/main.c` is the composition root: it creates every task
and starts one startup task that initialises the shared hardware.

| Path                                        | Contents                                                             |
| ------------------------------------------- | -------------------------------------------------------------------- |
| `include/navigation/obstacle.h`             | Buddy 5 facade: commands, events, telemetry snapshot, line-task glue |
| `include/physics/physics.h`                 | Buddy 4 facade: telemetry snapshot                                   |
| `src/navigation/obstacle/`                  | Buddy 5 scan, planner, statechart, comms, flight log                 |
| `src/physics/`                              | Buddy 4 IMU, hump detection, pipeline, task                          |
| `src/motion/`                               | Motor owner, PIO quadrature encoders, PID                            |
| `src/track/`, `src/platform/`               | Line sensors and the shared GPIO IRQ dispatcher                      |
| `src/telemetry/`                            | lwIP transport, MQTT client, queues, command ingress                 |
| `host/dashboard`, `host/cli`, `host/bridge` | Ground station UI, CLI and JSON bridge                               |
| `protocol/telemetry.yaml`                   | Wire schema shared by firmware and host                              |

The opt-in bring-up targets build with `make physics_check` or
`make obstacle_bench` inside `build/`.

## Ground station

`podman-compose -f deploy/compose.yaml up -d --build` builds and starts the MQTT
broker, the bridge and the dashboard as containers. The dashboard is at
`http://127.0.0.1:5281` and the bridge at `http://127.0.0.1:5280`.
`podman-compose -f deploy/compose.yaml down` stops them. The broker config is
`host/broker/mosquitto.conf`.

The broker publishes TCP `1883` on every interface so the car reaches it on the
Wi-Fi; allow it in the host firewall. The bridge and the dashboard listen on the
loopback only. The dashboard build holds the bridge address, so rebuild it with
`--build` after you change `BRIDGE_LOCAL_PORT`.

## Telemetry

The firmware publishes the status, motion, physics, obstacle, diag, heartbeat,
event and profile packets on MQTT (topics `s m p o d h e f`) and accepts
STOP/START/RESET commands. Broker settings are CMake cache values:
`PICO_CAR_MQTT_HOST`, `PICO_CAR_MQTT_PORT`, `PICO_CAR_MQTT_USERNAME`,
`PICO_CAR_MQTT_PASSWORD`, `PICO_CAR_TOPIC_PREFIX` and `PICO_CAR_CLIENT_ID`.

The run state never starts the car by itself, with one exception: a pilot stop
for a short sonar or stale-feedback fault resumes on its own when the fault
clears within PILOT_SONAR_RESUME_MS (1 s). After a boot, a watchdog reset, a
lift, an overturn or a lapsed motion permit (a stall or a long gap) it holds the
wheels until an operator START from the dashboard or the CLI. A barcode turn is
refused until then. START is refused while the attitude reads LIFTED or
OVERTURNED, while the permit has lapsed, and while the lean is unknown. Once the
attitude reads OK again (50 level samples, tilt under 5 deg), START is accepted,
even while the car is held level in the air, because the operator chose to send
it. RECAL is refused while the lean is unknown, and a running car keeps driving
then. The car counts as upright when its lean is known, it is calibrated and its
roll is within 60 deg. RESET is refused while the car is overturned and not upright, and is
accepted once it is upright.

A link outage never stops the car. The `wifi_disconn` fault bit is set after 3 s
without an MQTT session, for telemetry only.

The status, motion and physics packets go out at 5 Hz. The obstacle packet goes
out at 2 Hz and on change, and the diag and heartbeat packets at 1 Hz. Events go
out when they happen, and the scan profile once per scan. The layout of every
packet, the fault bits and the event types are in `protocol/telemetry.yaml`
(schema 2). After you edit that file, run
`npm --prefix host/dashboard run generate`.

`lib/lwip` is a submodule used by the telemetry build; `configure.sh`
initialises it. Set the Wi-Fi credentials on the configure line:

```sh
./configure.sh \
  -DPICO_CAR_WIFI_SSID=your-ssid \
  -DPICO_CAR_WIFI_PASSWORD=your-password
```

`configure.sh` detects the active Wi-Fi and the host's private address. It fills
the credentials and broker host automatically for WPA2-PSK and open networks.
Enterprise (`wpa-eap`) networks are not supported by the CYW43, so the join
stays off and the script warns. Override with `-DPICO_CAR_WIFI_SSID`,
`-DPICO_CAR_WIFI_PASSWORD` and `-DPICO_CAR_MQTT_HOST`.

The generated credentials header stays in `build/` and is never committed. An
empty SSID keeps `TM_WIFI_JOIN=0`.

## Calibration and attitude

The rest reference is set from 50 samples (1 s) taken with the car at rest. Each
sample needs the total acceleration within 0.05 g of 1 g, the pitch varying by
less than 2 deg across the window, the up axis (z) reading positive and the tilt
from vertical within 10 deg. A sample that breaks any of these starts the count
again, so an upside down car or one tilted past 10 deg never sets the reference.
Calibration runs by itself while the car is uncalibrated: the supervisor's
calibration hold releases as soon as `physics.calibrated` turns true, so no
START is needed to finish it.

The attitude reads unknown (`attitude_unknown`) until a rest reference is set:
at boot, after a new run (RECAL) and after the IMU is lost. The supervisor
refuses START and RECAL while it is unknown, and the attitude fields keep their
last known values. A single failed accelerometer read does not make it unknown;
the IMU counts as lost after 25 failed reads in a row (0.5 s), and then the
reference is dropped.

Every 2 s the accelerometer configuration is read back. A mismatch means the
sensor has reset, so it is initialised again. The readback is skipped on the
cycle after a failed read and uses two attempts per transaction. The
magnetometer configuration is checked only at init, so a magnetometer reset is
not caught by the readback and affects the heading only.

## Tests

The measurement benches (`car_bench`, `wdt_check`, `physics_check`,
`obstacle_bench`) build to `build/*.uf2`; `tools/bench_summary.py` reads their
logs.

Host tests build out of tree:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build
```

## Table of Abbreviations

| Abbreviation | Meaning                    |
| ------------ | -------------------------- |
| lcg          | linear congruent generator |
