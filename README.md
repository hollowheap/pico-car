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

Hold BOOTSEL while plugging the Pico W. If it mounts as `RPI-RP2`,
`flash.sh` copies the UF2 directly, with no sudo. Otherwise it falls back to
`picotool`. Pass an optional image for the bring-up builds:

```sh
./flash.sh build/physics_check.uf2
./flash.sh build/obstacle_bench.uf2
./flash.sh build/car_bench.uf2
./flash.sh build/wdt_check.uf2
```

When developing, `configure.sh` should be run before the first build or
whenever `CMakeLists.txt` changes. It is unnecessary to run it otherwise;
Running `build.sh` is enough.

Build notes:

- `configure.sh` initializes submodules before configuring CMake.
- `CMakeLists.txt` pins `PICO_SDK_PATH` to the vendored `lib/pico-sdk` so a devshell environment variable cannot override it.
- Kernel translation units build with `-std=gnu11`; only `src/` is held to BARR-C C99 with `-Werror`.
- The kernel linker script asserts `.boot2` is exactly 256 bytes. Link with `-Wl,--undefined=boot2` so the member is not dropped from the archive.
- Hand-rolled firmware targets need `-ffunction-sections` and `-fdata-sections`; otherwise `--gc-sections` cannot drop unused SDK code.

## Developing

All C and C++ code in this repository must comply with the
[BARR-C:2018 standard](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).

When creating a new file, use the templates in `templates/` for consistency.

## Layout

Public subsystem headers live in `include/<subsystem>/`; implementations live
in `src/<subsystem>/`. `src/main.c` is the composition root: it creates every
task and starts one startup task that initialises the shared hardware.

| Path | Contents |
|---|---|
| `include/navigation/obstacle.h` | Buddy 5 facade: commands, events, telemetry snapshot, line-task glue |
| `include/physics/physics.h` | Buddy 4 facade: telemetry snapshot |
| `src/navigation/obstacle/` | Buddy 5 scan, planner, statechart, comms, flight log |
| `src/physics/` | Buddy 4 IMU, hump detection, pipeline, task |
| `src/motion/` | Motor owner, encoders, PID |
| `src/track/`, `src/platform/` | Line sensors and the shared GPIO IRQ dispatcher |
| `src/telemetry/` | lwIP transport, MQTT client, queues, command ingress |
| `host/dashboard`, `host/cli`, `host/bridge` | Ground station UI, CLI and JSON bridge |
| `protocol/telemetry.yaml` | Wire schema shared by firmware and host |

The opt-in bring-up targets build with `make physics_check` or
`make obstacle_bench` inside `build/`.

## Ground station

`./dev.sh` starts the local MQTT broker, the bridge, and the dashboard. It
prints the broker, bridge, and dashboard addresses. The broker config is
`host/broker/mosquitto.conf`. Logs are under `logs/`. The supervisor
restarts the broker, bridge, or dashboard if one exits. Set
`EXPO_CLEAR=1` to clear Metro's cache on start.

The services listen on the host's private address so the car and phone can
reach them on the hotspot. Allow TCP `1883`, `8090`, and `8083` on the Wi-Fi
interface in the host firewall.

## Telemetry

The firmware publishes the status, motion, physics, obstacle, diag, heartbeat,
event and profile packets on MQTT (topics `s m p o d h e f`) and accepts
STOP/START/RESET commands. Broker settings are CMake cache
values: `PICO_CAR_MQTT_HOST`, `PICO_CAR_MQTT_PORT`,
`PICO_CAR_MQTT_USERNAME`, `PICO_CAR_MQTT_PASSWORD`,
`PICO_CAR_TOPIC_PREFIX` and `PICO_CAR_CLIENT_ID`.

The status, motion and physics packets go out at 5 Hz. The obstacle packet
goes out at 2 Hz and on change, and the diag and heartbeat packets at 1 Hz.
Events go out when they happen, and the scan profile once per scan. The layout
of every packet, the fault bits and the event types are in
`protocol/telemetry.yaml` (schema 2). After you edit that file, run
`npm --prefix host/dashboard run generate`.

`lib/lwip` is a submodule used by the telemetry build; `configure.sh`
initialises it. Set the Wi-Fi credentials on the configure line:

```sh
./configure.sh \
  -DPICO_CAR_WIFI_SSID=your-ssid \
  -DPICO_CAR_WIFI_PASSWORD=your-password
```

`configure.sh` detects the active Wi-Fi and the host's private address. It
fills the credentials and broker host automatically for WPA2-PSK and open
networks. Enterprise (`wpa-eap`) networks are not supported by the CYW43, so
the join stays off and the script warns. Override with
`-DPICO_CAR_WIFI_SSID`, `-DPICO_CAR_WIFI_PASSWORD` and
`-DPICO_CAR_MQTT_HOST`.

The generated credentials header stays in `build/` and is never committed.
An empty SSID keeps `TM_WIFI_JOIN=0`.

## Tests

The hardware smoke test plan is in [`docs/smoke-test.md`](docs/smoke-test.md). The measurement benches (`car_bench`, `wdt_check`, `physics_check`, `obstacle_bench`) are in [`docs/bench.md`](docs/bench.md); `tools/bench_summary.py` reads their logs.
The supervisor, its commands and its health checks are in [`docs/supervisor.md`](docs/supervisor.md).

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
