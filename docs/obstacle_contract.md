# Obstacle contract

This page states what the obstacle subsystem promises to the rest of the car, and what it needs in return. The artifacts are the source of truth: `artifacts/specs.md` (S12, S13, S39, S40) and `artifacts/decisions.md` (D95 to D99, D110, D111). If this page and an artifact disagree, the artifact wins. Fix this page.

The code is in `src/navigation/obstacle/`. The public header is `include/navigation/obstacle.h`. Constants are in `obstacle_config.h`. Codes and packed layouts are in `obstacle_pack.h`.

## Ownership

| Item | Owner |
|---|---|
| Sonar, servo, echo ISR, statechart, planner | The obstacle task (`ctsk_obstacle`, priority 4, Core 0). Nobody else touches GP0, GP1 or GP12. |
| Motors during an obstacle | The line task, through `obstacle_line_step`. It holds the car, then drives the bypass from the STEER words. |
| Fault register bits 8, 10 and 13 | The obstacle publish step (`comms.c`, `share_faults`). |
| Start and stop of the whole run | The operator command today. The supervisor takes it later (WP3). |

The obstacle task never drives a wheel. Its only direct action on the car is the emergency stop link (`obstacle_stop_fn_t`) when a fault latches.

## Commands in

Post a command with `obstacle_post_cmd(cmd)`. A command is one byte: type (3 bits) and request id (5 bits). The command ring holds 4 commands. A full ring returns an error and drops nothing that is queued.

| Type | Code | Sent by | Effect |
|---|---|---|---|
| `SCAN_REQ` | 1 | line task, once the car is still | `WAIT_STOP` to `SCANNING` for this request id. |
| `ACCEPT` | 2 | line task | `PLAN` to `BYPASS`, or to `MONITOR` when the plan is CLEAR. |
| `REJECT` | 3 | line task | `PLAN` to `STOPPED` with cause `PLAN_REJECTED`. |
| `LINE_FOUND` | 4 | line task | `BYPASS` or `REACQUIRE` to `MONITOR`. |
| `STOP` | 5 | operator, supervisor | Any state to `STOPPED`, cause `EXTERNAL`. |
| `RESET` | 6 | operator, supervisor, line task | Clears the latched faults when the cause allows it. |
| `START` | 7 | operator, supervisor, line task | `STOPPED` to `MONITOR`, only after a `RESET`. |

## Events out

Read events with `obstacle_get_event`. The event ring holds 4 events. A full ring drops the new event and counts it (`dropped_events` in the snapshot). A STEER word replaces the previous one.

| Event | Meaning |
|---|---|
| `OBSTACLE` | A hit was confirmed. Payload: distance in mm. The line task must hold the car and later send `SCAN_REQ`. |
| `SCAN_RESULT` | The scan is done. Payload: plan status, side, width, start angle, wall flag. |
| `FAULT` | The statechart is `STOPPED` with a cause. Payload: cause and the fault word. |
| `STEER` | The drive shape for the bypass: straight, left, right or hold. |

## States

| State | Servo | Leaves when |
|---|---|---|
| `STOPPED` | 0 degrees | `RESET`, then `START`. |
| `MONITOR` | 0 degrees | 2 hits among the last 3 pings: `OBSTACLE` event, then `WAIT_STOP`. |
| `WAIT_STOP` | 0 degrees | `SCAN_REQ`. After `OBSTACLE_WAIT_STOP_MS` (500 ms) with no `SCAN_REQ`: `STOPPED`, cause `HANDOVER`. |
| `SCANNING` | sweeps, at most +-30 degrees | Scan done: `SCAN_RESULT`, then `PLAN`. No progress for `OBSTACLE_STEP_STALL_MS` (3 s): `STOPPED`, cause `SCAN_TIMEOUT`. |
| `PLAN` | 0 degrees | `ACCEPT` or `REJECT`. After `OBSTACLE_PLAN_ACCEPT_MS` (200 ms): `STOPPED`, cause `PLAN_TIMEOUT`. |
| `BYPASS` | 0 degrees | Forward beam clear for 2 pings and the planned angle reached: `REACQUIRE`. After `OBSTACLE_ESCAPE_MAX_MS` (4 s): `STOPPED`, cause `BYPASS_LIMIT`. |
| `REACQUIRE` | 0 degrees | `LINE_FOUND`. Time or distance limits (`OBSTACLE_HUG_MAX_MS`, the plan hug limit) end it: `STOPPED`, cause `LINE_TIMEOUT`. |

While the car moves in `BYPASS` or `REACQUIRE`, any forward reading under `OBSTACLE_FWD_GUARD_MM` (300 mm) stops the car and rescans.

## The hit rule

One rule is used everywhere. A ping is a hit when it is VALID at or below `OBSTACLE_DETECT_MM` (250 mm), or TOO_CLOSE. The statechart keeps the last 3 pings.

| Rule | Window | Needed | Result |
|---|---|---|---|
| Detect | last 3 pings (`OBSTACLE_HIT_WINDOW`) | 2 hits (`OBSTACLE_DETECT_HITS`) | `OBSTACLE` event. |
| Collision guard | last 3 pings | 2 pings at or below `OBSTACLE_COLLISION_MM` (80 mm), or TOO_CLOSE | `STOPPED`, cause `COLLISION`. |

The navigation side holds the car as soon as the latest ping is a hit (`is_obstacle_close`). The statechart confirms with 2 of 3. A single outlier holds the car for one ping and then lets it go.

## Echo classes

| Code | Class | Meaning | In a scan | In `MONITOR` |
|---|---|---|---|---|
| 0 | `VALID` | Clean echo, 116 to 23500 us (about 20 mm to 4 m). | Measured range. | Hit when 250 mm or less. |
| 1 | `OPEN` | ECHO high beyond 23.5 ms: nothing reflects. | Miss. | Clear. |
| 2 | `TOO_CLOSE` | Clean echo from `OBSTACLE_GLITCH_MAX_US` (30 us) up to 116 us. | A hit at 20 mm. | A hit, and a collision ping. |
| 3 | `NO_ECHO` | No edge at all within the 40 ms wait. | Proves nothing. Retried. | Counts toward a fault. |
| 4 | `GLITCH` | One edge missing, or an echo shorter than 30 us. | Proves nothing. Retried. | Counts toward a fault. |
| 5 | `STUCK` | ECHO was already high before the trigger. | Proves nothing. | Counts toward a fault. |

A missing echo is never OPEN. A slot that stays unresolved after 4 pings is stored as GLITCH. The edge walk assumes the obstacle goes on past it, and the planner blocks that side.

## Fault causes and bits

Each failing class has its own limit. A good ping (VALID, OPEN or TOO_CLOSE) ends the run.

| Cause | Code | Trigger | Obstacle fault bit | Shared bit |
|---|---|---|---|---|
| `NO_ECHO` | 3 | 3 `NO_ECHO` pings in a row (`OBSTACLE_NO_ECHO_COUNT`) | 0 | 8 `sonar_no_echo` |
| `ECHO_STUCK` | 4 | 3 `STUCK` pings in a row | 1 | 10 `echo_line` |
| `ECHO_NOISE` | 14 | 5 `GLITCH` pings in a row, or 5 failures of any mix | 8 | 10 `echo_line` |
| `SCAN_TIMEOUT` | 5 | No scan progress for 3 s | 2 | 13 `obstacle_stopped` |
| `NO_SAFE_PATH` | 6 | The plan has no usable side | 3 | 13 |
| `COLLISION` | 7 | Collision guard | 4 | 13 |
| `HANDOVER` | 8 | No `SCAN_REQ` in 500 ms | 5 | 13 |
| `PLAN_TIMEOUT` | 9 | No `ACCEPT` or `REJECT` in 200 ms | none | none |
| `PLAN_REJECTED` | 10 | `REJECT` | none | none |
| `BYPASS_LIMIT` | 11 | Time or distance limit | 6 | 13 |
| `LINE_TIMEOUT` | 12 | The line was not found in time | 7 | 13 |
| `INIT_FAIL` | 13 | Start-up failed. Only a reboot clears it. | none | none |

There is no separate "timeout" fault for the echo. A timeout is how the task sees `NO_ECHO`.

## Warning flags

Plausibility checks raise flags in the `flags` byte of the `o` packet. They never stop the car.

| Bit | Flag | Check |
|---|---|---|
| 0 | `wall` | The plan follows a wall. |
| 3 | `echo_noisy` | P1: 3 of the last 8 pings of a still scan jump by more than 40 mm. |
| 4 | `echo_inconsistent` | P2: after 100 mm of travel between two VALID readings under 600 mm, the range did not change by the travel within 40 mm. |
| 5 | `beam_blocked` | P3: 6 of the first 8 pings after boot were hits. |
| 6 | `scan_noisy` | P5: more than 30 percent of the slots in a finished scan needed 3 or 4 pings. |

P4 is a check on the trigger, not a flag. The TRIG pad is read back at the end of the 12 us pulse. A pad that does not read high makes the ping `NO_ECHO` and skips the 40 ms wait. `OBSTACLE_TRIG_READBACK` set to 0 turns it off. Bits 1 and 2 are reserved for servo moving and body pivoting.

## Retry policy

The line task sends `RESET` and `START` by itself every `OBSTACLE_NAV_RETRY_MS` (3 s) while the statechart is `STOPPED`, but only for these causes:

- `BOOT` (the first start)
- `SCAN_TIMEOUT`, `HANDOVER`, `PLAN_TIMEOUT`, `PLAN_REJECTED`, `BYPASS_LIMIT`, `LINE_TIMEOUT`

For every other cause the car stays held until an explicit `RESET` and `START`: `COLLISION`, `NO_SAFE_PATH`, `NO_ECHO`, `ECHO_STUCK`, `ECHO_NOISE`, `EXTERNAL` and `INIT_FAIL`. A collision means the car touches something. A sensor fault means the car would drive blind. Both need a person or the supervisor to decide. The rule is `obstacle_pack_cause_may_auto_restart`.

## Snapshot for other tasks

All readers copy and never wait.

| Call | Gives |
|---|---|
| `obstacle_snapshot` | State, cause, fault word, latest echo class and range, request id, ping sequence (wraps at 32), servo and body angle, flags. |
| `obstacle_copy_profile` | The 37 scan slots (distance 12 bits, class 3 bits, measured 1 bit). Slot 18 is straight ahead. A slot is 5 degrees, from -90 to +90 degrees. Valid after the first finished scan. |
| `obstacle_copy_plan` | Closest range, width, side room, status, side, start angle and wall flag of the last finished scan. |
| `obstacle_echo_counts` | Pings per echo class since boot. The telemetry task subtracts two reads for the per-second counts in the `d` packet. |

## Telemetry

| Packet | Obstacle data |
|---|---|
| `o` (2 Hz and on change) | State, cause, fault word, echo class, range, closest, plan fields, servo and body angle, flags, sequence, request id. |
| `f` (once per scan) | The 37 slots and the plan summary. |
| `d` (1 Hz) | Pings per class in the last second. |
| `e` (events) | `obstacle_seen`, `scan_done`, `bypass_start`, `bypass_end`, `obstacle_stop` (planned, WP3 and WP8). Fault bits also send `fault_set` and `fault_clear`. |

The wire layout is in `protocol/telemetry.yaml`. See [telemetry notes](../artifacts/notes/telemetry-schema-v2-and-fault-bits.md).

## Timing budget

| Item | Value |
|---|---|
| Ping period | 60 ms |
| Echo wait | 40 ms (the sensor holds ECHO high about 38 ms when nothing reflects) |
| Servo settle | 3 ms per degree plus 20 ms |
| Brake before scan | `OBSTACLE_NAV_BRAKE_MS`, 100 ms |
| Stop to scan | at most 500 ms (`WAIT_STOP`) |
| Plan to accept | at most 200 ms |
| Escape pivot | at most 4 s |

At 60 ms per ping, 2 hits among 3 pings take 120 to 180 ms. At 150 mm/s the car moves about 18 to 27 mm in that time. The 250 mm detect distance leaves about 200 mm for braking and the sensor offset.

## What the subsystem does not promise

- It cannot tell an angled or soft object from an empty space. Both give OPEN.
- A single wrong VALID reading that passes the checks is a normal reading.
- It does not restart after a collision, a sensor fault or a STOP.
- A side strip about 250 mm ahead is a blind strip: the beam is about 84 to 108 mm wide there and the car is 126 mm wide. An object only in that strip is seen when it enters the 250 mm forward beam. See [geometry](obstacle.md#geometry-and-blind-strip).
