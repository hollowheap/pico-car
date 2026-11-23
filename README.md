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
- The firmware uses the kernel object limits in `cmake/mtk3smp_config.h.in`.
  CMake applies those limits to the pinned kernel and firmware targets.
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
STOP/START/RESET and E-STOP commands. Broker settings are CMake cache values:
`PICO_CAR_MQTT_HOST`, `PICO_CAR_MQTT_PORT`, `PICO_CAR_MQTT_USERNAME`,
`PICO_CAR_MQTT_PASSWORD`, `PICO_CAR_TOPIC_PREFIX` and `PICO_CAR_CLIENT_ID`.

A valid MQTT E-STOP latches the drive inhibit before command diagnostics or
supervisor reporting. The motion task applies the brake to the outputs. The
supervisor handles one pending E-STOP before queued commands, even if eight STOPs
are waiting. The status packet reports the drive inhibit and uses zero as the
speed fallback when wheel feedback is unavailable. Confirm brake application and
vehicle rest separately.

A falling edge on GP21 or an accepted firmware E-STOP request wakes the motion
task immediately. The task alone controls the motor outputs and still samples
wheel motion on a 10 ms schedule during normal operation. The electrical cutoff
target is 5 ms from GP21 assertion or firmware acceptance. Verify cutoff at the
motor pins and measure stopping time and distance on the car. Sonar detection
and MQTT delivery add their own latency before firmware accepts a stop request.

In `main.uf2`, wire a normally open momentary START switch between GP20 and
GND. The input has an internal pull-up and Schmitt trigger. The supervisor
requires a stable release and then a stable press, each debounced for 40 ms.
A button held at boot issues no command. Each press submits one operator START
through the same safety checks as dashboard and CLI START. After a refusal,
release the button and press it again once the fault clears.

The supervisor applies a STOP hold before acknowledging it. When START requires
calibration, the calibration hold is in place before the existing holds are
released. A motion timing fault stops permit renewal and watchdog feeding until
reboot.

On a power-up with no MQTT session, the car waits at least 10 seconds and
starts once its calibration, motion feedback, and forward sonar are ready.
A broker session during the wait leaves START with GP20, the dashboard or CLI.
The offline start happens only once per boot and requires power-up evidence
from the RP2040, which includes brownouts. A watchdog, RUN-pin, debugger,
or unknown reset requires operator START. A lift, overturn, deliberate stop,
or lapsed motion permit also requires operator recovery.

On an unmarked bend, a single front line detector on tape steers the car even
when the left barcode reader crosses the same tape. A confirmed change of side
changes the steering direction on that control pass. The inside wheel can reach
zero motor duty on a tight bend. With both readers clear, side-reader pulses
alone can hold the heading for up to 100 mm from the last front sighting.
Measured elements of a tentative barcode start extend that allowance by their
width until the decoder accepts or rejects them. After a verified start, each
accepted element renews the distance.

At an unmarked square
corner, the follower remembers where the front pair crossed the outgoing strip.
If the line disappears as the axle approaches that crossing, the car searches
from the corner for the outgoing tape.

During a run, a barcode L, R or S requests the left, right or through exit
at the next junction. The car follows ordinary bends with the instruction
pending through an obstacle bypass or a temporary sensor hold. The fixed
barcode reader is left of the line. The car uses that
reader and the line sensors to locate a possible junction. A brief pause of
the inside wheel preserves the approach while the outside wheel advances.
The car then checks the outgoing tape with an encoder-guided pivot. Distinct tape directions with
clear floor between them establish a choice. A T can offer left and right
with no forward tape. When the requested exit is present the car aligns
with it. When it is missing the car follows an observed exit and keeps the
instruction for another junction. It prefers through tape when available.
For S at a terminating T, it takes the side it verifies first during the
return sweep. When the car has verified the only available exit, it can finish
turning back to that tape with the instruction still pending.
The car reports `turn_start` at selection and `turn_done` after alignment.
An interrupted alignment reports `turn_abort` with elapsed time.

A fully decoded U cancels a waiting barcode direction and turns back at
the marker. If U arrives during a junction scan, the car uses the heading
it had on approach to reacquire the same tape heading back. A committed
maneuver stays busy until it finishes or is interrupted. Motion holds
take priority. Operator turn buttons request immediate maneuvers.
Reader placement, branch geometry and traction still need a physical run.

After a pickup or overturn, set the car down upright and still. Once the
IMU confirms recovery, one operator START clears resolved fault history,
retries recoverable pilot faults and starts calibration on level ground.
Active fault bits remain visible. A restart on a known hump retains the
previous IMU reference. A latched E-stop requires a reboot.

Normal START selects continuous floor and moving obstacle navigation. This also
applies to the GP20 button and offline startup. On a course with possible
drop-offs, send the stationary-course START command (opcode 10). The
three-second limit on a planned obstacle pause uses wheel feedback. The pilot
takes another observation from a new pose or sets up its next leg. The next
physical run must establish the timing and rejoin. [Obstacle behavior](docs/obstacle.md#hug-pilot-path)

START checks the motion permit, attitude and IMU and sonar connections.
The wheels stay held if these checks fail. A timed-out calibration can be
retried with START. Once it completes, press START again to authorize motion.
A stationary car held level in the air can resemble a car on the ground to
the IMU, so set it down before pressing START. The CLI also accepts RESET
to acknowledge faults and keep the wheels held.
During a run, an isolated missed accelerometer read preserves the attitude
reference. After 25 consecutive failed cycles (about 500 ms), or when the last
physics publication expires at 500 ms, the supervisor holds the wheels and
keeps the run armed. It resumes automatically once the IMU reports calibrated,
upright and still, both fresh wheel speeds are valid and zero, and a new
centered forward ping arrives. An open path resumes line control. A nearby
return on the normal build starts a stationary obstacle survey. An interrupted
turn or bypass needs a fresh plan. A held-high sonar pulse at the echo deadline
has no measured range and cannot clear the hold.

A missed sample restarts the IMU's set-down confirmation. A pickup also cancels the active obstacle maneuver.
If both tilt and acceleration evidence lapse, the next disturbance starts
a fresh pickup count. The final sample must still show qualifying tilt or
acceleration. Sustained tilt still counts over a hump, and a confirmed lift
or overturn holds the wheels until operator recovery.

A link outage during a run leaves local navigation active. With Wi-Fi configured,
the car retries five seconds after a failed association or a lost link. The USB
console reports a connection fault during this wait and joining during the next
attempt. After association the car waits for a current DHCP lease and queues a
heartbeat as soon as MQTT reconnects. The
`wifi_disconn` fault bit is set after 3 s without a session, for telemetry only.
Link events mark MQTT session changes. A link-down event can be delivered after
reconnection if the telemetry queues retain it.
The pilot allows 250 ms for the first centered sonar sample once the other
drive inputs are ready. After a long sonar or stale-feedback dropout it
requires three consecutive usable forward pings with fresh wheel feedback
and the motion holds released. During parked course recovery, any observed
movement restarts the clearance count. If a line sensor
sample is more than 25 ms old, the line task stops the wheels and aborts any
active turn. Fresh line samples let normal navigation resume.

The status, motion and physics packets go out at 5 Hz, obstacle packets at 2 Hz,
and diag and heartbeat packets at 1 Hz. Sonar aim events record the initial
center and subsequent commanded angles before settling. Completed pilot sweeps
publish sparse sonar samples for display. The legacy scanner reports a full
profile from the completed scan before navigation can advance the state.
Status carries the raw line and barcode sensor levels, the last measured
barcode element (space, bar or block), and the latest decoder phase. The
phase starts after the opening sentinel, advances on the first accepted
payload element, and ends when the read completes or aborts. Best-effort
`barcode_phase` events carry those transitions and the known scan direction.
The latest phase remains in status if an event is dropped.
An ambiguous IR pulse or a full capture buffer interrupts the current read.
The next edge starts a fresh width measurement. A measured dark block also
ends a partial read. Only a fully framed A–Z payload counts as a decoded
letter and starts the 100 mm duplicate-suppression distance.
The scan and connection tasks count each telemetry drop when the queue is
busy or full. Packet layouts,
fault bits and event types are in `protocol/telemetry.yaml` (schema 3).
After you edit that file, run
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

The IMU acquires its rest reference from 50 consecutive samples (1 s). Each
sample must have positive board Z, tilt within 10° of vertical and acceleration
magnitude within 0.10 g of 1 g. Across the window, pitch may vary by up to 2°
and acceleration magnitude by up to 0.05 g. The provisional mean tolerance is
±0.075 g around 1 g. This accommodates the installed sensor's stationary readings
around 1.04 g. Wheel speed above the calibration threshold or a break in
sampling restarts acquisition. After calibration the set-down check compares
acceleration with that measured reference. When START requests a new
calibration, physics tags the first sample from that request. Motion waits
for a calibrated upright sample carrying the matching request.

At boot, after RECAL and after IMU loss, attitude reads unknown until the rest
reference is set. A fresh acceleration measurement still reports board Z and
total magnitude during this time. A parked car can receive START during gravity
calibration or during the subsequent level/still check. One press waits up to
five seconds while upright rest samples continue. The operator hold stays on
until both checks pass and wheel feedback is fresh and stationary. STOP, loss of
supporting sensor evidence (including a break between supervisor polls), a
hardware fault or expiry cancels the request. A subsequent START is required
after cancellation. A calibrated car retains its reference
across a single failed read and drops it after 25 consecutive failures (0.5 s).
An interval longer than 50 ms between usable accelerometer readings clears
partial pickup and overturn evidence. An already confirmed operator hold
continues through the gap.

Every 2 s the accelerometer configuration is read back. A mismatch means the
sensor has reset, so it is initialised again. The readback is skipped on the
cycle after a failed read and uses two attempts per transaction. On sustained
data loss, the driver retries initialization immediately and then every 50
physics cycles (nominally 1 s) until accelerometer data returns. If the physics
task itself stops running, the supervisor withholds the hardware watchdog feed.
Resumed task heartbeats before the watchdog expires allow local recovery. The
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
