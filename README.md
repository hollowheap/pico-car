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
- The kernel and firmware use the object limits in the pinned port's
  `lib/mtk3smp-rp2040/config/config.h`.
- Application C, the PIO adapter, and the compiled SDK, lwIP and TinyUSB sources
  build with `-std=c99` using the pinned `hollowheap/pico-sdk` C99 fork.
  The kernel port builds with `-std=gnu11`. Application warnings are errors.
- The kernel linker script asserts `.boot2` is exactly 256 bytes. Link with
  `-Wl,--undefined=boot2` so the member is not dropped from the archive.
- Hand-rolled firmware targets need `-ffunction-sections` and `-fdata-sections`;
  otherwise `--gc-sections` cannot drop unused SDK code.

## Developing

All C and C++ code in this repository must comply with the
[BARR-C:2018 standard](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).

## Layout

Public subsystem headers live in `include/<subsystem>/`; implementations live in
`src/<subsystem>/`. `src/main.c` is the composition root: it calls every `<subsystem>_init()`
and then every `<subsystem>_start()`, which creates that subsystem's tasks. One
startup task initialises the shared hardware.

| Path                                        | Contents                                                             |
| ------------------------------------------- | -------------------------------------------------------------------- |
| `include/navigation/obstacle.h`             | Obstacle API: commands, events, telemetry snapshot, line-task glue    |
| `include/physics/physics.h`                 | IMU API: telemetry snapshot                                          |
| `src/navigation/obstacle/`                  | Obstacle scan, planner, statechart, comms, flight log                |
| `src/physics/`                              | IMU sensor, hump detection, pipeline, task                           |
| `src/motion/`                               | Motor owner, PIO quadrature encoders, PID                            |
| `src/track/`, `src/platform/`               | Line sensors and the shared GPIO IRQ dispatcher                      |
| `src/telemetry/`                            | lwIP transport, MQTT client, queues, command ingress                 |
| `host/dashboard`, `host/cli`, `host/bridge` | Ground station UI, CLI and JSON bridge                               |
| `protocol/telemetry.yaml`                   | Wire schema shared by firmware and host                              |

Build the optional IMU bring-up image with `make physics_check` inside
`build/`.

## Ground station

From the repository root, use Node.js 24, npm and Mosquitto. The Nix
development shell provides all three.

```sh
npm run setup   # install the locked dependencies once
npm start
```

Open **http://127.0.0.1:5281**. One server hosts the dashboard, WebSocket,
history and command API. Logs continue while the browser is closed.
When the dashboard reconnects, it uses retained bridge history to fill gaps
in the course trace, charts and frame log. If that request fails, the dashboard
keeps showing live data and retries while connected. It then rebuilds those
views with the recovered samples. History and the live stream share a stable
ID and arrival order for each receipt. This keeps receipts distinct within one
millisecond, preserves the exact Reset boundary on reload, and lets reconnect
replay follow arrival order after a backward clock adjustment. The frame log
and event labels show the bridge’s wall-clock receipt times.
If the history file becomes unwritable during a run, live telemetry continues
and recent packets stay available in memory. The dashboard shows the recording
failure and the bridge saves the retained packets when writes resume. A gap
remains if a frame never reached the bridge or aged out of its history.
The Course trace draws measured odometer distance in millimetres and fits the
map to the available space. Use the scale bar when comparing runs. When the
firmware measures pivot slip, the trace applies that correction to wheel yaw.
Straight-line distance still comes from the encoders.
The bridge accepts both the original status frame and the frame with pivot
evidence. Update the ground station before flashing firmware that sends it.
`npm start` exports the current dashboard before launching. For subsequent
starts with the same export, use `npm run serve`. Ctrl+C stops the station.

The browser uses the page's hostname and port for history, commands and live
telemetry. HTTPS selects WSS automatically. If a private station proxy serves
HTTP on a non-loopback hostname, build with `EXPO_PUBLIC_ALLOW_INSECURE=1 npm start`.
For Compose, put `EXPO_PUBLIC_ALLOW_INSECURE=1` in `deploy/.env` and rebuild
the `bridge` service. Expo embeds this setting when it exports the dashboard.

The station uses the local broker on TCP 1883, starting Mosquitto with
`host/broker/mosquitto.conf` when the port is free. The car reaches that
listener over Wi-Fi. Set `MQTT_URL` to use another broker. Only a broker
started by the station is stopped on exit.

Native history is stored in `host/bridge/history.jsonl`. Set
`BRIDGE_HISTORY_PATH` to use another file. Keep a single writer for each
history file.

An optional Compose deployment runs the station and broker with the
`bridge-data` volume for history:

```sh
podman-compose -f deploy/compose.yaml up -d --build
```

The broker publishes `BROKER_PORT` (default 1883) on every interface and the
dashboard listens on `127.0.0.1:${DASHBOARD_LOCAL_PORT:-5281}`. Set either in
`deploy/.env`. The bridge reconnects to the broker, so start order does not
matter.

## Telemetry

The firmware publishes status, motion, physics, obstacle, diag, heartbeat,
event, profile and barcode sample packets on MQTT (topics `s m p o d h e f b`) and accepts
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
released. A motion timing fault (an expired drive lease or a late control
period) brakes the wheels and clears in place after 500 ms, and the next drive
command moves the car again. A timing fault that lasts over 1 s, or a fourth one
since boot, stops permit renewal and watchdog feeding until reboot.

Each accepted STOP or E-stop sends a `stop_timing` event. p1 is the time in ms
from acceptance until both motor duties read zero. p2 is the time until both
wheel speeds also read zero, and the event carries the odometer at rest. The
supervisor samples both at its control pass. A stop that never reaches rest
reports p2 as 65535.

On a power-up with no MQTT session, the car waits at least 10 seconds and
starts once its calibration, motion feedback, and forward sonar are ready.
A broker session during the wait leaves START with GP20, the dashboard or CLI.
The offline start happens only once per boot and requires power-up evidence
from the RP2040, which includes brownouts. A RUN-pin, debugger or unknown
reset requires operator START. A lift, overturn, deliberate stop, or lapsed
motion permit also requires operator recovery.

The watchdog scratch registers keep a run record across a watchdog reset.
The record holds whether a run was armed, its floor mode and its resume count.
After a watchdog reset during an armed run, the car resumes without operator
START. It first waits until it is parked with a fresh clear forward ping.
The resume keeps the floor mode of the run: continuous floor or **Stop at
obstacles**. Each resume sends a supervisor `recovery` event with the
watchdog reset reason and the resume count. After two resumes without an
operator START, a further watchdog reset requires operator START. An E-stop,
pickup, lift, overturn or lapsed permit cancels a pending resume. A watchdog
reset with no armed run requires operator START.

On continuous floor, START permits forward travel with both front readers on
bare floor. The pilot holds its encoder heading until tape guides a correction
or another behavior takes control. Missing tape alone has no distance limit.
The **Stop at obstacles** setting retains the bounded line search for courses
where floor support is uncertain.

On an unmarked bend, a single front line detector on tape steers the car even
when the left barcode reader crosses the same tape. A confirmed change of side
changes the steering direction on that control pass. The inside wheel can reach
zero motor duty on a tight bend. On continuous floor, after one reader stays
on tape for 25 mm, a gap of up to 25 mm retains steering into that bend.
If the tape remains missing for 130 mm from the last contact, the axle starts
a bounded optical sweep toward the last-seen side. An opposite reader hit takes
effect immediately. Outside a sustained bend, with both readers clear, side-reader pulses
alone can hold the heading for up to 100 mm from the last front sighting.
Measured elements of a tentative barcode start extend that allowance by their
width until the decoder accepts or rejects them. After a verified start, each
accepted element renews the distance.

At an unmarked square
corner, the follower remembers where the front pair crossed the outgoing strip.
If the line disappears as the axle approaches that crossing, the car searches
from the corner for the outgoing tape. On continuous floor, one unsuccessful
sweep returns to its approach heading and it continues forward.

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
A probe that ends without a turn reports `junction` with the outcome: no
branch, a fallback exit, or aborted. The follower also reports `junction`
for every transverse tape stripe it measures, with or without an
instruction. The event names the line detectors that measured the stripe
and whether another reader saw the same tape.

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
drop-offs, select **Stop at obstacles** in the dashboard Start popup. The
three-second limit on a planned obstacle pause uses wheel feedback. The pilot
takes another observation from a new pose or sets up its next leg. The next
physical run must establish the timing and rejoin.

After an obstacle bypass rejoins the line, the follower drives 100 mm on the
line. The pilot then sends a `rejoin` event with the heading against the entry
heading and the lateral offset from the entry line. A heading more than a
quarter turn from the entry heading means the car follows the line the wrong
way, so the pilot starts the local line search. The 100 mm hold is a guess
until it is measured on the car.

START checks the motion permit, attitude and IMU and sonar connections.
The wheels stay held if these checks fail. A timed-out calibration can be
retried with START. Once it completes, press START again to authorize motion.
On the pilot build, START also checks that the chassis moves when the wheels
accelerate. Set the car down with clear space ahead before pressing START.
The check waits up to five seconds for fresh centered sonar clearance and
usable wheel and IMU data. It then commands both wheels forward at 80 mm/s
for at most 500 ms and ends at 35 mm of indicated wheel travel. It compares
the wheel speed rise with the IMU's forward response relative to its stable
reading at rest. If the check fails, the car stays stopped until a fresh START and the dashboard
reports **Startup motion unverified**. The probe thresholds need grounded
and suspended runs on the car.

Stop holds the wheels and cancels the current maneuver. Reset in the dashboard
or CLI also clears the route and acknowledges eligible faults. The dashboard clears its trace,
graphs and activity when the supervisor accepts Reset. The car keeps its boot
identity and uptime. Start is required to move again.
During a run, an isolated missed accelerometer read preserves the attitude
reference. After 25 consecutive failed cycles (about 500 ms), or when the last
physics publication expires at 500 ms, the supervisor holds the wheels and
keeps the run armed. It resumes automatically once the IMU reports calibrated,
upright and still, both fresh wheel speeds are valid and zero, and a new
centered forward ping arrives. An open path resumes line control. A nearby
return on the normal build starts a stationary obstacle survey. An interrupted
turn or bypass needs a fresh plan. A held-high sonar pulse at the echo deadline
has no measured range and cannot clear the hold.

The accelerometer reads about 1 g on level ground. Pickup detection subtracts
the calibrated rest load along the measured vertical. With two fresh stationary
wheel readings, five consecutive samples more than 0.05 g upward confirm a
pickup (about 100 ms at 50 Hz). During travel a pickup needs 15 consecutive upward samples. The first three
must exceed 0.12 g and the rest must stay above 0.05 g. A supported tilt above 20° for 15 samples independently confirms a
pickup. A moving hump masks the upward cue. After a downward unload, ten
samples back within 0.05 g of the rest magnitude rearm the pickup check.
Missing samples interrupt confirmation. A confirmed pickup cancels the active
maneuver and puts the car in Stopped. Setting the car down or using Clear
Faults leaves the pickup fault active and the wheels held. An accepted
operator START after a level, still set-down clears the pickup stop and starts
calibration. The thresholds need a labelled hand-pickup and landing run.

Physics telemetry sends the filtered pitch in centidegrees and smooths roll
over about 150 ms. A failed accelerometer read clears the frame’s calibrated
attitude flag until a usable sample arrives. The safety detector uses each
sensor sample directly. The logged board Z and acceleration magnitude include
gravity and retain impact peaks at the telemetry sampling instants.

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

The barcode reader sends best-effort `barcode_sample` frames on topic `b`.
Each queued edge wakes the telemetry worker and forwarding wakes the MQTT task.
Use edge timestamps for stripe timing. Task scheduling and network delivery
can group several edges into one burst.
The installed-reader setting interprets GP26 LOW as light floor and HIGH as
a dark bar. Status and edge frames carry the raw electrical level.
They record the GP26 level after each edge, its microsecond timestamp,
interval, sequence, and the estimated width when wheel feedback permits.
An unmeasured interval starts a new span. Capture loss and unavailable
speed are explicit. A complete framed Z is reported as a letter with no
driving action. The decoder learns each marker's narrow width and wide-to-narrow
ratio, updating them as unambiguous elements arrive. After the start pattern,
an element narrower than 30 % of the learnt narrow width counts as a glitch.
The decoder joins the glitch and the element after it to the element before
it, so a bar, a glitch and a bar read as one bar. The 30 % share is a guess
until it is measured on barcode paper.

The vehicle IMU uses board −Y for forward and +Z for up. Raising the nose
produces positive pitch. Raising the left side produces negative roll under
the dashboard and wheel controller's left-down-positive convention.

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
After the boot event, the supervisor sends one `build` event. It carries the
28-bit git short hash and flags for a dirty tree, the pilot build, the watchdog
and an unknown hash. CMake records the hash at configure time, so rerun
`configure.sh` after a commit to refresh it.
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

The magnetometer heading starts without a hard-iron offset, so it is only a
rough monitor. When the car pivots on the spot on level ground through at least
240 degrees of encoder yaw, as a line search does, the physics task fits a
circle to the horizontal field. It then uses the circle centre as the offset of
the two horizontal axes. The offset includes the motor field at pivot duty, so
the heading is approximate while the car drives at other duty.

With the offset learnt, the magnetometer also witnesses each pivot on the
spot. After 300 ms of steady turning in one direction, the physics task reports
the share of the encoder yaw that the field shows, from 50 to 100 %. A share
under 90 % counts as wheel slip. The pilot then advances the follower's heading
by that share of the encoder yaw, so a search sweep keeps turning until the
body has covered it. The obstacle hug and the contour route turn their pose by
the reported share during a pivot. Without a report they assume 80 %. The
300 ms, 50 % and 90 % limits are guesses until they are measured on the car.

Pivots on the spot rise to their speed instead of stepping to it, so the wheels
do not spin on the floor or on the barcode paper. One-shot turns, junction
probes and search sweeps raise their turn speed at 500 mm/s² from the speed
the car had, after a 45 mm/s start. They slow at once, so a pivot still stops
on the tape. The rate is a guess until it is measured on the car.

## Tests

The measurement benches (`car_bench`, `wdt_check`, `physics_check`) build to
`build/*.uf2`. The [course calibration guide](testing/course-calibration.md)
covers ODO and PIVOT logs, measured fixtures and dashboard trace replay.

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
