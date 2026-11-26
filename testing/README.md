# Testing

This folder records how each subsystem was tested and what happened. Each test is one of three kinds:

| Kind | Where it runs | Evidence |
|---|---|---|
| Host tests | Your computer, against shims | CTest output |
| Bench checks | The car, with a check image | USB console log |
| Behaviour runs | The car on the course, with `main.uf2` | USB console log, notes and video |

## Subsystems

| Subsystem | Covers | Records |
|---|---|---|
| comms | Wi-Fi, MQTT telemetry and commands | [comms.md](comms.md) |
| motion | Motors, encoders, wheel control and the pilot | [motion.md](motion.md) |
| ir | Line sensors and barcode reading | [ir.md](ir.md) |
| imu | Attitude, humps and lift detection | [imu.md](imu.md) |
| obstacle | Sonar, servo scan and obstacle bypass | [obstacle.md](obstacle.md) |
| shared | Platform, supervisor, watchdog and start-up | [shared.md](shared.md) |

The CTest labels for shared code are `platform` and `supervisor`.

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

Run the firmware lint gate (clang-tidy and clang-format) with `bash lint.sh`.
It must report `0 finding(s)`.

Run the ground-station tests with:

```
npm --prefix host/bridge test
npm --prefix host/dashboard run typecheck
npm --prefix host/dashboard test
```

## Bench checks

Build the images with `./build.sh`; the bench sources live in `bench/`. Build the optional physics image with `cmake --build build --target physics_check`. To flash, hold BOOTSEL, plug in the USB cable and copy the `.uf2` file to the drive. Save the whole USB console under `testing/logs/`.

| Image | Subsystem | Checks | Summary |
|---|---|---|---|
| `car_bench.uf2` | motion, ir, imu | Encoders, wheel step, odometry, pivot, line sensors, both line laws, lift cue, timing | `python3 testing/bench_summary.py LOG` |
| `wdt_check.uf2` | shared | Motor pins across a watchdog reset | Read the `W1_PINS` lines |
| `physics_check.uf2` | imu | Guided still, nose-lift and side-lift checks | `python3 testing/physics_log_summary.py LOG --checks` |

On `car_bench`, a short press on GP20 selects the next item. Hold GP20 for 1 s to run it. Do not press GP21: it latches the emergency stop until a power cycle.

## Obstacle extras

The standard build's `main.uf2` runs the obstacle-extras controller.
It uses the original 200 mm/s wheel target limit and 25% duty cap.
Bypass spins run at 140 mm/s and slow to 46.7 mm/s for the last 20 degrees.
Spin headings use the 80.25 mm pivot track from `app_program/platform/chassis.h`.
Neither value has been measured on the car yet.
Verify that the mount clears the physical sweep from −48° to +48° around
the 1450 μs center pulse. The dev-stack mount was qualified from −20° to +20°.

For the motor-free scanner bench, hold **GP20 and GP21** while booting
`car_bench.uf2`, then release both. GP20 selects a menu item and GP21 sends
it. AUTO-RUN starts at boot and simulates navigation in software. Boot
without the two-button hold to return to the standard measurement menu.

Run the recorded scan and navigation regressions with:

```sh
ctest --test-dir build-tests --no-tests=error -R '^obstacle[.]extras_' --output-on-failure
```

Three flat-face recordings return `NO_SAFE_PATH` at `eeb8177` and preserve
that outcome here.

## Behaviour runs

1. Flash `main.uf2` and open the USB console.
2. Note the setup and the wall-clock time before powering the car.
3. Save the USB log and fill out the record below.

## Run logs

Keep captures for a run together in `testing/logs/<date>-<subsystem>-<run>/`.
Save the USB console log and any exported bridge recording there. Put the
capture path and the run's wall-clock time in the test record. Git ignores
the capture files.

The bridge records every live session to `testing/logs/bridge/history.jsonl`.
Its 5 Hz status packets carry wheel travel, wheel speed and target, mode and
state. They are enough to rebuild the axle path, stretch lengths, bend radius
and spin angles after a run.

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
