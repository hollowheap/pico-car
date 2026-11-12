# Supervisor

One task, `ctsk_supervisor`, owns the run state of the car and watches the other tasks. This page tells what it does and what you can tune. The artifacts are the source of truth: D120, D121, D160, S51, S52 and S71 in `artifacts/`.

The supervisor never drives a wheel. The line task drives. The supervisor decides what is allowed.

| Item | Value |
|---|---|
| Task | `ctsk_supervisor`, priority 2, Core 0, 4096 B stack |
| Wake | A cyclic handler every 20 ms (`SUPERVISOR_PASS_MS`) |
| Code | `src/supervisor/`, public header `include/supervisor/supervisor.h` |
| Pure files (host tests) | `command_ring.c`, `run_state.c`, `health_core.c` |

## Mission control

Every command goes into one ring. The dashboard, a barcode and a button post with `vehicle_command_post(opcode, source)`. The supervisor reads the ring once per pass and decides.

| Opcode | Source allowed | Result |
|---|---|---|
| `STOP` | telemetry, button | Operator hold. State `STOPPED`. |
| `STOP` | barcode | Barcode hold. State `HELD`. |
| `ESTOP` | telemetry, button | Motion emergency latch. State `LATCHED`. The stop also acts at once in the connection task. |
| `START` | telemetry, button | Releases the operator, barcode and lift holds, and sends RESET and START to the obstacle task. Refused while latched or lifted. |
| `RESET` | telemetry, button | Called "Clear faults" on the dashboard. Clears the obstacle faults (after an obstacle stop, a sonar fault or a bypass failure) and an overturn latch, and starts the IMU calibration again. It never releases a hold and never moves the car: send START after it. Refused after an emergency stop. |
| `LEFT`, `RIGHT`, `UTURN`, `STRAIGHT` | telemetry, barcode, button | `STRAIGHT` drives `LINE_STRAIGHT_MM` (250 mm) on the heading it had, with the line sensors ignored: the car crosses a junction. Accepted only when the state is `RUNNING`, the line task follows the line, the obstacle task monitors and no turn is pending. |

A barcode can only send `STRAIGHT`, `LEFT`, `RIGHT`, `UTURN` and `STOP`. Any other opcode from a barcode is refused.

The barcode letters (one place to change them: `barcode_action_of()` in `src/barcode/action.c`):

| Letter | Command |
|---|---|
| `S` | Straight |
| `L` | Left 90° |
| `R` | Right 90° |
| `U` | U-turn |
| `H` | Halt (barcode hold; START releases it). A fifth, optional: delete its row to have four. |

Every other letter is reported as a refused barcode.

Each decision is an event: `cmd_accept`, or `cmd_reject` with a reason (see `protocol/telemetry.yaml`).

A full ring drops the oldest command that is not a stop. A stop is never lost.

## Run state

The state follows from five flags: operator hold, barcode hold, lift hold, overturn latch, emergency latch.

| State | When | Mode byte |
|---|---|---|
| `BOOT` | Before the first pass | `stopped` |
| `RUNNING` | No flag set | `follow`, `turn`, `search`, `hold`, `bypass` |
| `STOPPED` | Operator hold | `stopped` |
| `HELD` | Barcode hold or lift hold | `hold_barcode`, `lifted` |
| `LATCHED` | Emergency latch or overturn | `overturned`, `stopped` |

The car starts to run after boot without a START, as before. There is no `ready` state.

A lift or an overturn sets `MOTION_HOLD_LIFT` at once. The hold stays until START, and START is refused while the car is still lifted. An overturn needs RESET after the car is upright. RESET starts the IMU calibration again.

## Health

Every 100 ms the supervisor runs these checks.

| Check | Trip | Reaction |
|---|---|---|
| Liveness | A required task misses its bit for 5 checks (500 ms) | Fault bit 14 `task_stalled`, event `task_stall`, an ERROR line |
| Progress | The wheels are told to move, no hold is set, and the travel stays under 2 mm for 3 s | Fault bit 14, event `task_stall` with p1 `0x80` |
| Stuck line sensor | The line sensors do not change over 1500 mm of driving | Event `task_stall` with p1 `0x81`, a WARN line |

Required tasks: motion, line, obstacle, physics, and telemetry, connection and barcode when they started. A task is watched after its first kick. A required task that never kicks counts as stalled after 5 s.

The tasks kick their bit once per pass with `liveness_kick(LIVE_*)` from `platform/liveness.h`.

### Run permit

The supervisor renews a 200 ms run permit every pass (`motion_renew_permit`). When the permit lapses, the motion task sets `MOTION_HOLD_SUPERVISOR` by itself and the car brakes. The supervisor stops renewing the permit while a critical task (motion, line, obstacle) is stalled. The permit is not enforced before the first renewal.

### Hardware watchdog

The watchdog is off by default. It stays off until you pass the check below:

```
./configure.sh -DPICO_CAR_WATCHDOG=ON
```

With it on, the supervisor starts the RP2040 watchdog with a 1 s timeout and feeds it at the end of every pass, unless a critical task is stalled. A hung supervisor or a stalled critical task resets the board.

**Test W1 (do this before you turn it on).** The RP2040 resets all pins. The H-bridge inputs must stay low across the reset, or the motors can run.

1. Wheels off the floor. Build with `-DPICO_CAR_WATCHDOG=ON`.
2. Let the car run so the wheels turn. Stop the supervisor on purpose (for example, loop forever in `supervisor_run` for a test build).
3. Watch the wheels for the 1 s until the reset and for the first 2 s after it.
4. Pass when the wheels brake at the reset and do not move until the car runs again. Write the result in `artifacts/questions.md`, Q61.

After a watchdog reset, fault bit 15 `abnormal_reset` is set, the heartbeat shows `reset_reason` `watchdog`, and the `boot` event carries the reason. The RP2040 cannot tell a brown-out from a power-on, so a brown-out reads as power.

### Boot self-test

Two seconds after boot the supervisor sends the `boot` event. `p1` is the reset reason. `p2` is the set of failed checks: 1 IMU missing or I2C fault, 2 no sonar echo, 4 something in the beam (P3). Encoder polarity and servo travel are not checked: both need the wheels or the servo to move.

### Stack marks

All task stacks are filled with `0xA5` before the tasks exist. Every 30 s the supervisor logs the untouched bytes of each stack and warns when one is under 512 bytes. This answers Q25.

## What you can tune

| Setting | File | Default |
|---|---|---|
| Liveness limit | `LIVENESS_MISS_LIMIT` in `health_core.h` | 5 checks |
| Progress stall time | `PROGRESS_STALL_MS` | 3000 ms |
| Stuck sensor travel | `STUCK_TRAVEL_MM` | 1500 mm |
| Run permit lease | `PERMIT_LEASE_MS` in `supervisor.c` | 200 ms |
| Watchdog timeout | `WATCHDOG_TIMEOUT_MS` in `supervisor.c` | 1000 ms |
| Barcode turn delay | `BARCODE_TURN_DELAY_MM` in `main.c` | 0 mm (turn at once) |
| Barcode re-arm distance | `BARCODE_REARM_MM` in `barcode/action.h` | 100 mm |
| Line-lost suppression | `BARCODE_SUPPRESS_US` in `barcode/action.h` | 250 ms |
| Lift and overturn limits | `attitude.c` defaults | tilt 20°, lift 0.12 g, overturn 100° or roll 60° |

All limits are placeholders. Tune them on logs from the car.

## Test on the car

| # | Test | Pass when |
|---|---|---|
| S1 | Send STOP, then START, from the dashboard | The wheels stop and start. `cmd_accept` events show. |
| S2 | Send LEFT while the obstacle task scans | `cmd_reject` with reason 2 (busy). No turn. |
| S3 | Show a barcode `H` | The car stops. Mode is `hold_barcode`. START releases it. |
| S4 | Show a barcode `L`, `R`, `U` | The car turns left, right, U-turn. |
| S4b | Show a barcode `S` at a crossing | The car drives straight across it for about 250 mm, then follows the line again. `turn_start` and `turn_done` show kind 4. |
| S5 | Show the same barcode twice within 100 mm | The second one is `barcode_bad` reason 5. |
| S6 | Lift the car by hand | `lift_start`, fault bit 11, wheels hold. Set down and send START. |
| S7 | Tip the car on its side | `overturn`, fault bits 11 and 12. RESET after upright, then START. |
| S8 | Run the car for 30 s with a serial monitor open and read the `stack free` lines | No stack under 512 bytes free. |
| W1 | See the watchdog test above | Wheels brake at the reset. |
