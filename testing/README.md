# Testing

This folder records how each subsystem was tested and what happened. Each test is one of three kinds:

| Kind | Where it runs | Evidence |
|---|---|---|
| Host tests | Your computer, against shims | CTest output |
| Bench checks | The car, with a check image | USB console log |
| Behaviour runs | The car on the course, with `main.uf2` | Bridge recording, notes and video |

## Subsystems

| Subsystem | Covers |
|---|---|
| comms | Wi-Fi, MQTT telemetry and commands |
| motion | Motors, encoders, wheel control and the pilot |
| ir | Line sensors and barcode reading |
| imu | Attitude, humps and lift detection |
| obstacle | Sonar, servo scan and obstacle bypass |
| shared | Platform, supervisor, watchdog and start-up |

## Host tests

Run these from the Nix shell (`nix develop`):

```
cmake -S tests -B build-tests
cmake --build build-tests -j8
ctest --test-dir build-tests --no-tests=error
```

Tests are named `<subsystem>.<unit>.<case>`. Run a subsystem with
`ctest --test-dir build-tests -L '^motion$' --output-on-failure` or one case with
`ctest --test-dir build-tests -R '^motion[.]quadrature[.]default$' --output-on-failure`.

Run the ground-station tests with:

```
npm --prefix host/bridge test
npm --prefix host/dashboard run typecheck
npm --prefix host/dashboard test
```

## Bench checks

Build the images with `./build.sh`. Build the optional physics image with `cmake --build build --target physics_check`. To flash, hold BOOTSEL, plug in the USB cable and copy the `.uf2` file to the drive. Save the whole USB console under `testing/logs/`.

| Image | Subsystem | Checks | Summary |
|---|---|---|---|
| `car_bench.uf2` | motion, ir, imu | Encoders, wheel step, odometry, pivot, line sensors, both line laws, lift cue, timing | `python3 testing/bench_summary.py LOG` |
| `wdt_check.uf2` | shared | Motor pins across a watchdog reset | Read the `W1_PINS` lines |
| `physics_check.uf2` | imu | Guided still, nose-lift and side-lift checks | `python3 testing/physics_log_summary.py LOG --checks` |

On `car_bench`, a short press on GP20 selects the next item. Hold GP20 for 1 s to run it. Do not press GP21: it latches the emergency stop until a power cycle.

## Obstacle extras

Build the two optional obstacle-extras images in the configured firmware build.

```sh
cmake --build build --target obstacle_extras obstacle_extras_bench
ctest --test-dir build-tests --no-tests=error -R '^obstacle[.]extras_' --output-on-failure
```

- `obstacle_extras.uf2` runs the original line-following, sweep, bypass and
  rejoin controller on the shared motion and GPIO drivers with the original
  200 mm/s wheel target limit and 25% duty cap. GP21 is the emergency stop.
  Diagnostics use USB. GP7 stays available for I²C.
- `obstacle_extras_bench.uf2` drives the sonar and simulates navigation in
  software. GP20 selects a menu item and GP21 sends it. The motor task is
  absent and the AUTO-RUN simulation starts at boot.

Use these images with a mount that clears the original physical sweep
from −48° to +48° around the 1450 μs center pulse. The dev-stack mount was
qualified only from −20° to +20°. The normal build still produces `main.uf2`
for that narrower range.

The host tests replay the recorded scan profiles and navigation events from
`obstacle-extras` at `eeb8177`. Three recorded flat-face profiles return
`NO_SAFE_PATH` at that revision and retain that expected outcome here.

## Behaviour runs

1. Flash `main.uf2` and start the ground station (`npm start`).
2. The bridge records every packet to `host/bridge/history.jsonl` while it runs.
3. Note the setup, start the run and note the wall-clock time.
4. Write the record below, including the time, so the run can be found in the recording.

## Run logs

Keep captures for a run together in `testing/logs/<date>-<subsystem>-<run>/`.
Save the USB console log and any exported bridge recording there. Put the
capture path and the run's wall-clock time in the test record. Git ignores
the capture files.

## Record template

Copy this block for each test and fill in every field. Write "n/a" rather than leaving a field empty.

```
### <date> — <subsystem> — <short test name>

- Build: <git commit hash, plus "+dirty" if uncommitted>
- Kind: host test | bench check | behaviour run
- Setup: <course layout, obstacles, start pose, battery state>
- Steps: <what you did>
- Expected: <what should happen>
- Observed: <what happened; rough numbers welcome>
- Result: pass | fail | partial
- Evidence: <log file, recording time window or video link>
- Follow-up: <fix, retest or n/a>
```

Keep one file per subsystem, for example `testing/motion.md`, with the newest record first.
