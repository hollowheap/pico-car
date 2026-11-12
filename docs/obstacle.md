# Buddy 5: obstacle detection and avoidance

Ultrasonic sensor (HC-SR04+), scan servo (SG90), coarse/fine scan, bypass
planning and the obstacle statechart. The code is in
`src/navigation/obstacle/`. Buddy 5 never writes the motors: it only sends events and
steering words to the line task.

The artifacts are the source of truth: `artifacts/specs.md` (S12, S13, S39,
S40) and `artifacts/decisions.md` (D95 to D99, D150, D151). This page
describes the code that implements them.

Other Buddy 5 docs:

- [Obstacle contract: commands, events, states, faults, retry policy](obstacle_contract.md)
- [File guide: which file does what](obstacle_file_guide.md)
- [Hardware test checklist](obstacle_hardware_checklist.md)

## Status

Last updated 12 November 2026.

| Step | Work | Current position |
|---|---|---|
| 1 | Design (pins, statechart, scan, bypass) | Done; see the artifacts above. |
| 2 | Code in `src/navigation/obstacle/` | Done: six echo classes, the 2-of-3 hit rule, the retry policy, and the plan, flags and class counts in telemetry. |
| 3 | Host tests | Done: `ctest -R obstacle_` (scan, plan, statechart, bypass simulation). |
| 4 | BARR-C:2018 pass | Done; `bash tools/lint.sh src/navigation/obstacle include/navigation` reports 0 findings. |
| 5 | Bench build in the normal team build | Done; `./build.sh` makes `build/obstacle_bench.uf2`. |
| 6 | Car measurements (**FILL** values) | Pending; see [Numbers to fill in](#numbers-to-fill-in). The sonar edge and incidence measurement (Q56) comes first. |
| 7 | Hardware checklist H1 to H9 on the car | Pending; see the [checklist](obstacle_hardware_checklist.md). |
| 8 | Floor test | Pending. Boxes 240 to 340 mm wide are planned too narrow; see [wide-box findings](../artifacts/notes/obstacle-wide-box-findings.md). |

## Wiring

**Ultrasonic sensor (HC-SR04+) → Grove 1**

| Sensor pin | Grove 1 wire | Pico pin |
|---|---|---|
| VCC | Red | 3.3 V |
| GND | Black | GND |
| TRIG | Yellow | GP0 |
| ECHO | White | GP1 |

If the cable colours differ, follow the pin labels: TRIG to GP0, ECHO to
GP1. Mount the sensor on the servo horn, facing straight ahead.

**Servo (SG90) → Servo port 1**

| Servo wire | Board marking |
|---|---|
| Orange (signal) | S (GP12) |
| Red (power) | + (V+) |
| Brown (ground) | − (GND) |

Only port 1 is driven (GP12, PWM slice 6). V+ comes from the battery or
USB, so a low LiPo makes the servo slower.

**Why the sensor must run on 3.3 V.** The HC-SR04+ works from 3.3 V to
5 V, and its ECHO high level equals its supply. Powered from Grove 1, ECHO
is 3.3 V, which is safe. Powered from 5 V, ECHO would reach 5 V, but the
Pico's GPIO is rated to 3.6 V at most. Current would then flow through
GP1's protection diodes into the 3.3 V rail, which can destroy GP1 or the
RP2040. The sensor itself would be unharmed. Check ECHO with a meter
before connecting it (checklist H1). If it ever reads about 5 V, fit a
divider: 1 kΩ from ECHO to GP1 and 2 kΩ from GP1 to GND.

**Do not use** GP8–GP11 (motors), GP4/5/16/17 (encoders), GP26–28 (IR)
or GP20/21 (buttons).

## Build and run

**Bench firmware** (the normal team build, no extra flags):

```sh
nix develop
./configure.sh
./build.sh                  # makes main.uf2 as before, plus:
# build/obstacle_bench.uf2  -> hold BOOTSEL, plug in, copy it to the Pico
```

Open the USB serial port to see the output. The Buddy 5 targets are the
"Buddy 5" block at the end of the team `CMakeLists.txt`. `main.uf2` does not
link Buddy 5 yet, so it is byte-for-byte the same as without that block.

**Bench buttons.** USB input isn't implemented in `libtm`, so there is no
serial menu.

- **GP20** goes to the next menu item, which is printed.
- **GP21** sends the selected item.
- **AUTO-RUN** is on at boot. A simulated Navigation then answers by itself:
  - OBSTACLE → SCAN_REQ
  - OK or CLEAR result → ACCEPT
  - after 4 s of hugging → LINE_FOUND
  - FAULT → RESET + START after 3 s

## Files

| File (in `src/navigation/obstacle/`) | Job | Pure (no kernel calls) |
|---|---|---|
| `obstacle_config.h` | Pins, timings, tunables, **FILL** placeholders, compile-time range checks | yes |
| `obstacle_pack.h` | Codes and packed-word layouts (masks and shifts) | yes |
| `scan_algo.c/.h` | Echo classification, ping agreement, coarse → fine scan engine | yes |
| `planner.c/.h` | Beam-corrected width, side choice, start angle, hug rules, integer trig | yes |
| `statechart.c/.h` | Statechart (no kernel calls; the task carries out its requests) | yes |
| `comms.c/.h` | Command queue, event outbox, published copies, links to Buddy 2 (private) | no |
| `scan_hw.c/.h` | GP0/GP1/GP12 set-up, servo PWM, TIMER check, echo ISR, trigger | no |
| `scan_task.c/.h` | The task: commands → time limits → servo/settle → 60 ms ping | no |
| `include/navigation/obstacle.h` | Public facade: links, commands, event wait, telemetry snapshot, line glue | — |
| `obstacle.c/.h` | Start-up order, then forwards to `obstacle_comms`; `obstacle.h` is private (flags and packed accessors) | no |
| `obstacle_nav.c/.h` | Phase 2 glue: line task ↔ Buddy 5 ↔ motion service (hold, STEER → speed, odometer) | no |
| `bench_main.c` | Standalone `usermain` with a simulated Navigation | no |

All implementation files above are in `src/navigation/obstacle/`; the only public
header is `include/navigation/obstacle.h`. The build rules are the "Buddy 5"
block at the end of the team `CMakeLists.txt`. The
[file guide](obstacle_file_guide.md) explains each file in more detail.

## How it works

### Tasks and kernel objects

| Item | Value |
|---|---|
| Obstacle task (`ctsk_obstacle`) | priority 4, `TP_PRC1` (Core 0), 4096 B stack |
| Echo ISR | 1 slot in the shared `src/platform/isr` dispatcher |
| Kernel objects created | **1 event flag**; no message buffer, cyclic handler or mutex |
| PWM | slice 6 channel A (GP12), 125 MHz / 125 = 1 MHz, TOP 19999 = 50 Hz |

### One ping (60 ms)

1. Empty the echo result with interrupts paused. If ECHO is already high,
   the ping is **STUCK** and no trigger is sent.
2. Send a 12 µs TRIG pulse.
3. The ISR records the rise time. On the fall it writes
   `width | RISE | DONE` in one store and sets the flag bit.
4. The task waits at most 40 ms, then `scan_algo_classify` gives the class:
   VALID, OPEN, TOO_CLOSE, NO_ECHO, GLITCH or STUCK. The table, and what
   each class does, is in the [contract](obstacle_contract.md#echo-classes).

A missing echo is **never** OPEN. There is no UNKNOWN class (D97).

### Statechart

The states, the time limits and the fault causes are in the
[contract](obstacle_contract.md#states). Rules that apply in every state:

- **STOP command or any fault** gives STOPPED, plus a FAULT event.
- **A fault with a bit** also calls the emergency-stop link.
- **Detect** is 2 hits among the last 3 pings. The **collision guard** is
  2 close pings (at most 80 mm, or TOO_CLOSE) among the last 3. Both use the
  same hit definition (D150).

### Scan

- **Grid:** 37 slots, -90 to +90 degrees in 5 degree steps, positive = left.
  Slot 18 is straight ahead.
- **Servo:** never beyond +-30 degrees. The scan reaches the rest of the grid
  by turning the car body +-60 degrees (S40).
- **Coarse:** every 2nd slot (10 degrees) in the centre window.
- **Region:** the closest hit within 600 mm, plus coarse neighbours within
  closest + 250 mm. The closest reading is looked for inside the car's path
  (+-30 degrees) first.
- **Fine:** walk outward from each edge, one slot at a time, up to the first
  real miss (OPEN, or VALID beyond the last hit on that edge + 250 mm).
  - A NO_ECHO, GLITCH or STUCK slot proves nothing, so the walk assumes the
    obstacle goes on and steps past it.
  - TOO_CLOSE is a hit at 20 mm and never a miss.
  - An edge with no miss inside the window gets one more body-pivot sweep.
    An edge still unbounded after that makes the side unsafe.
- **Per slot:** settle for 3 ms per degree plus 20 ms. Ping until two pings
  agree (same class, within 15 mm for VALID) and keep the closer distance.
  After 4 pings without agreement the slot is stored as GLITCH.
- **Warning flags:** P1 (`echo_noisy`) and P5 (`scan_noisy`) are computed
  while the scan runs. They never stop the car.

### Plan and bypass

**Beam-cone correction.** With h = last hit angle, m = first miss angle and
β = half-cone (9.5 degrees), the planner places an edge between h - β and
m - β. This assumes the sonar returns an echo when only the edge of its beam
is on the obstacle. That assumption is not measured (Q56), and the
simulation does not follow it. See the
[wide-box findings](../artifacts/notes/obstacle-wide-box-findings.md).

**Choosing a side.** A side is usable when the sideways offset it needs (edge
+ `OBSTACLE_BOX_GAP_MM`) fits on the free floor, and no other reading on that
side falls in the swept band. The smaller offset wins; on a tie, left. An
edge that is not bounded, or lies beyond `OBSTACLE_EDGE_TRUST_DECI` (30
degrees), plans a wall pass on the left. With no usable side the plan is
NO_SAFE_PATH and the car holds.

**Out pivot (D98).** One pivot away from the obstacle at the smallest of 45,
60, 75 or 90 degrees that clears the front corner by half the car width plus
`OBSTACLE_PLAN_MARGIN_MM`. The car never reverses.

**Along the obstacle.** Every `OBSTACLE_SIDE_LOOK_MM` (250 mm) the car stops,
pivots 60 degrees toward the obstacle, aims the servo at 30 degrees so the
ray is normal to the face, takes a side reading, and pivots back. The reading
sets the lean that holds `OBSTACLE_BOX_GAP_MM` (190 mm) from the axle line.
For a wall the gap is `OBSTACLE_WALL_HOLD_MM` (300 mm).

**Return.** After the box the car turns 45 degrees toward the line and
drives until the IR sensors capture it, then sends LINE_FOUND.

**While moving** the sonar faces forward. A VALID or TOO_CLOSE reading under
300 mm stops the car and rescans.

### Geometry and blind strip

The car is 126 mm wide. The beam is about 84 to 108 mm wide at 250 mm. An
object that lies only in the outer 20 to 40 mm of the car width is outside
the beam until the car comes closer. The line follower keeps the car on its
lane, so the strip is accepted and not covered by extra pings.
`OBSTACLE_DETECT_MM` can be raised if the course needs it.

### Interfaces

The commands, events, snapshots and their limits are in the
[contract](obstacle_contract.md). The calls are in
`include/navigation/obstacle.h`.

```c
obstacle_links_t links = { h_nav_mbf, p_stop, p_get_distance_mm };  /* 0/NULL allowed */
obstacle_init(&links);              /* after gpio_irq_init() */
obstacle_post_cmd(obstacle_pack_cmd(OBSTACLE_CMD_SCAN_REQ, req_id));
obstacle_get_event(&evt, tmout);    /* own 4-slot outbox while h_nav_mbf == 0 */
obstacle_snapshot(&status);         /* state, cause, faults, echo class, flags */
obstacle_copy_profile(profile);     /* 37 slots: E_BUSY, never waits */
obstacle_copy_plan(&plan);          /* plan of the last finished scan */
obstacle_echo_counts(counts);       /* pings per echo class since boot */
```

## Bit table

| Word | Field | Valid range | Bits | Representation |
|---|---|---|---|---|
| Echo result `u32` | width_us | 0-40 000 µs (sat. 65 535) | 16 | bits 0-15, written whole by the ISR |
| | RISE, DONE | flags | 1 + 1 | bits 16, 17 |
| Scan slot `u16` x 37 | dist_mm | 0-4095 (VALID <= 4030) | 12 | bits 0-11 |
| | class | 6 classes | 3 | bits 12-14 |
| | measured | flag | 1 | bit 15 → **74 bytes** |
| Status `u32` (one store) | state | 7 states | 3 | bits 0-2 |
| | stop cause | 15 causes | 4 | bits 3-6 |
| | last class | 6 | 3 | bits 7-9 |
| | last dist_mm | 0-4095 | 12 | bits 10-21 |
| | scan_req_id | 0-31 | 5 | bits 22-26 |
| | seq (liveness) | 0-31 wrap | 5 | bits 27-31 |
| Fault word `u16` | 9 fault flags | - | 9 | single writer (obstacle task) |
| Command `u8` | type, id | 8 types, 0-31 | 3 + 5 | one byte |
| Event `u32` | type, id | 4 types, 0-31 | 2 + 5 | bits 0-6 |
| | OBSTACLE: dist | 0-4095 | 12 | bits 7-18 |
| | SCAN_RESULT: status, side, width, angle, wall | 4, 2, 0-1023 mm, 0-127 deg, flag | 2 + 1 + 10 + 7 + 1 | bits 7-27 |
| | FAULT: cause, faults | 15, 9 bits | 4 + 9 | bits 7-19 |
| | STEER: action, seq | 4, 0-63 | 2 + 6 | bits 7-14 |
| Ping `u32` (one store) | seq, class, dist_mm, servo | 0-63, 6, 0-4095, -30..+30 deg | 6 + 3 + 12 + 6 | bits 0-26 |
| Counters (task-private) | `obstacle_counters_t`: seven `uint8_t` | runs and 3-bit histories | 56 | no packed word |

Compile-time checks in `obstacle_config.h` stop the build if a field can't hold
its range. They use a negative array size, because C99 has no
`_Static_assert`.

## Coding standard (BARR-C:2018)

Every file in `src/navigation/obstacle/` was checked against BARR-C:2018 with:

- the team `.clang-format`, then a small fix-up for the two rules it cannot
  express (see the deviations table);
- the team `.clang-tidy` (warnings are errors), plus `readability-identifier-length`
  (minimum 3, rule 7.1.e) and `bugprone-narrowing-conversions`;
- gcc `-Werror -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion` for
  the firmware files (arm-none-eabi);
- a line scan for the rules no tool covers: 80 columns, tabs, CR-LF,
  end-of-file marker, banned keywords, constant on the left of `==`,
  `char` only for strings, file section order, single exit, if-nesting ≤ 2,
  parentheses around `&&`/`||` operands.

### Naming used here

| Prefix / suffix | Meaning | Rule |
|---|---|---|
| `obstacle_`, `scan_algo_`, `scan_hw_`, `scan_task_`, `flight_log_` | module prefix on every public function and type | 5.1.c, 6.1.i |
| `g_` | file-global variable | 7.1.j |
| `p_` | pointer (variable, parameter or member) | 7.1.k |
| `b_` | Boolean, phrased as a question | 7.1.m |
| `h_` | kernel object handle (an `ID`) | 7.1.n |
| `_t` | typedef | 5.1.a |
| `_task`, `_isr` | task body, interrupt handler | 6.4.a, 6.5.b |
| `_main.c` | file that holds `main()` or `usermain()` | 4.1.d |

A global Boolean or handle would be `gb_` / `gh_` under rule 7.1.o, but the
team `.clang-tidy` only accepts `g_` and `gp_` for globals. Those globals are
therefore `b_` / `h_` members of a `g_` struct (for example
`g_links.h_flag`, `g_obstacle.b_ready`). That satisfies both.

### Project abbreviations (rule 1.5.b)

Appendix A abbreviations (`cfg`, `init`, `isr`, `max`, `min`, `msg`,
`num`, `pwm`, `reg`, …) are used as listed there. Extra ones:

| Abbreviation | Meaning |
|---|---|
| cls | echo class (VALID / OPEN / TOO_CLOSE / NO_ECHO / GLITCH / STUCK) |
| cmd | command |
| ct | compile-time (in `OBSTACLE_CT_ASSERT`) |
| deci | deci-degrees, 0.1° (positive = left) |
| dist | distance |
| evt | event |
| fsm | finite-state machine (the statechart) |
| mbf | kernel message buffer |
| echo_result | echo result word written by the ISR |
| mm, ms, us | millimetres, milliseconds, microseconds (as unit suffixes) |
| odo | odometer (distance travelled) |
| ptn | event-flag bit pattern (kernel term) |
| q14 | fixed point with 14 fraction bits (16384 = 1.0) |
| req | request |
| seq | sequence number |
| sh, mk | shift and mask of a packed field (`OBSTACLE_*_SH`, `OBSTACLE_*_MK`) |
| st, ev | status and event word fields (`OBSTACLE_ST_*`, `OBSTACLE_EV_*`) |
| stacd, exinf, intsts, tmout | kernel parameter names, as in the T-Kernel spec |
| um | micrometres (bench odometer only) |

### Deviations

BARR-C allows documented deviations. Each one is also noted in a `NOTE:`
comment next to the code it affects.

| Rule | Deviation | Why |
|---|---|---|
| 5.2.a | Kernel types `ID`, `ER`, `INT`, `UINT`, `UW`, `TMO`, `BOOL`, `SYSTIM`, `T_CTSK`, `T_CFLG` are used at kernel and BSP calls | The kernel API is written in those types. All Buddy 5 data is `stdint.h` types. |
| 1.6.a | In `bench_main.c`, `(UB const *)` on string literals and `(INT)` / `(UINT)` on printf arguments are explained once, at the top of the file | Dozens of identical, value-safe casts; one note is clearer than one per line. Every other cast in the folder has its own comment. |
| 4.2.a | `bench_main.c` has no header | Nothing calls into a main module. `obstacle_config.h` and `obstacle_pack.h` have no `.c` because they hold only constants and inline helpers. |
| 6.1.i | `usermain()` has no module prefix | The kernel calls it by that name. |
| 6.3.a | `OBSTACLE_CT_ASSERT` is a function-like macro | A compile-time check needs the preprocessor. Each parameter is used once and parenthesised. |
| 6.5.a | `echo_isr()` has no interrupt `#pragma` | The kernel's interrupt entry runs the shared dispatcher, which calls it as an ordinary C function. It is still `static`, named `_isr` and placed last in `scan_hw.c` (6.5.b, 6.5.c). |
| 8.3.a, 3.3.c | The team `.clang-format` puts `break;` inside the case body and removes the blank line after `/*** end of file ***/` | After running the team `.clang-format` on a Buddy 5 file, move each `break;` back to line up with its `case`, and make sure the file ends with the end-of-file comment followed by one blank line. |
| 1.8.c | The spinlock words and `g_fsm` are not declared `volatile` by Buddy 5 | `T_SPLOCK` is already `volatile` in the kernel's own typedef; `g_fsm` is touched by the Buddy 5 task only. |

## Numbers to fill in

Search `obstacle_config.h` for **FILL**. Status:

| Setting | Status |
|---|---|
| Car half-width `OBSTACLE_CAR_HALF_W_MM` | **Done:** 63 mm (car is 126 mm wide) |
| Free floor `OBSTACLE_FREE_FLOOR_MM` | **Done:** 500 mm cap; the course has no side boundary, and the planner always uses the smaller offset |
| Servo direction `OBSTACLE_SERVO_DIR` | **Confirmed** on the bench (+1) |
| Bypass speed `OBSTACLE_BYPASS_SPEED_MM_S` | 40 mm/s, matches `main.c` (40.8 mm/s) |
| Beam half-angle `OBSTACLE_BEAM_HALF_DECI` | **Done:** 9.5° (95). Power bank at 300 mm dropped out with its near edge 50 mm left / 65 mm right of centre (9.5° / 12.2°); the smaller value gives more room. The left/right difference comes from the two transducers sitting either side of the centre. |
| Wheel diameter | **Done:** 65 mm (already in the motion service scale) |
| Wheel track `OBSTACLE_WHEEL_TRACK_MM` | **Done:** 107 mm (measured) |
| Servo→tail `OBSTACLE_SERVO_TO_REAR_MM` | Pending (170 mm); sets when the car turns back |
| Servo→front | Pending (optional) |
| Servo centre / scale | Looks right by eye; protractor check optional |
| Power source | Pending (settle time on a low LiPo, H2d) |

## Phase 2 (integration) — in main.uf2

Buddy 5 now runs inside the normal `main.uf2`, driven by the existing line
task (`line_task` in `src/main.c`) through the glue module
`src/navigation/obstacle/obstacle_nav.c`.

**Start-up** (in the startup task, processor 1): `gpio_irq_init()` →
`motion_service_init()` → `physics_init()` → `ir_capture_init()` →
**`obstacle_line_init()`**. That calls `obstacle_init()` with these links,
then sends RESET and START. `src/main.c` creates every task; the subsystems
only prepare resources and expose their task body.

| Link | Wired to | Why |
|---|---|---|
| `p_stop` | `motion_service_set_hold(MOTION_HOLD_OBSTACLE, true)` | Stops the car without the reboot latch of `motion_service_request_stop()`, so RESET + START can recover. |
| `p_get_distance_mm` | Mean forward `displacement_um` of both wheels from `motion_service_read_status()`, in mm | Real odometer for the plan-derived bypass limit. It keeps the last good value if a sample is invalid. |
| `h_nav_mbf` | 0 | Events go to Buddy 5's own outbox; the line task polls it. |

If `obstacle_line_init()` fails, it prints the error and line following runs
without Buddy 5.

**Every loop pass** (every ≤ 10 ms) the line task calls
`obstacle_line_step(left, right)` before its own steering. It returns true when
Buddy 5 is in charge, and the line follower then sends nothing.

| Buddy 5 state | What the glue does | Line follower |
|---|---|---|
| MONITOR | releases the obstacle hold, unless the latest ping is a hit: VALID within 250 mm, or TOO_CLOSE (so a retry never creeps into the obstacle) | drives as before |
| WAIT_STOP, SCANNING, PLAN | holds the car; 300 ms after OBSTACLE sends SCAN_REQ(id); on SCAN_RESULT OK/CLEAR sends ACCEPT(id) | paused |
| BYPASS | releases the hold; pivots away (STEER word) until Buddy 5 sees the way ahead is clear | paused |
| REACQUIRE | TURN (pivot to the start angle) and OUT (straight to the side offset) from the wheel odometry, then HUG: a proportional controller holds a 300 mm look-ray target by slowing the wheel next to the obstacle by up to 80% of the outer wheel. One ping under 250 mm forces a turn away, clamped at 60° from the line. OUT drives straight. IR gate: armed when the offset is reached, opens once both IR sensors have been off the line; the next line contact sends LINE_FOUND(id) (at most every 100 ms) | paused |
| STOPPED (fault) | holds the car. Sends RESET + START every 3 s only after BOOT or a transient cause (`obstacle_pack_cause_may_auto_restart`). After a collision, NO_SAFE_PATH, a sensor fault or a STOP it waits for an explicit RESET and START | paused |

**Speeds** (`motion_service_set_speed()`, lease 100 ms). The car stands
still while the servo turns (entering BYPASS, REACQUIRE or MONITOR): a
servo swing while a motor drives can disturb the wheel encoders.

| Phase | Straight | Turning (inside / outside wheel) |
|---|---|---|
| Escape spin, TURN | — | −40 000 / +40 000 µm/s |
| OUT, ALONG, BACK | 40 000 / 40 000 µm/s | 40 000 / 60 000 µm/s |

NOTE: The inside wheel never runs between 0 and 40 mm/s. The motors only
get up to 25 % duty, and a driven wheel that makes no encoder edges for 2 s
latches the motion service's NO_EDGES fault, which only a reboot clears.
If the motors cannot reach 60 mm/s, the outside wheel runs at its top
speed, which still curves the car.

**Dead reckoning.** Heading = `(right − left wheel distance) /
OBSTACLE_WHEEL_TRACK_MM` (from the totals, so it does not drift); position is
summed along that heading every pass. `OBSTACLE_WHEEL_TRACK_MM` is 107 mm
(measured). The wheel diameter
(65 mm) is already in the motion service's scale (204.2 mm per turn).

**Simulation** (host, the real Buddy 5 code in a closed loop with a
kinematic car), from detection to rejoining the line: obstacles 60/105/160 mm wide,
real beam 9.5°/15°/25°, true track 90/100/115 mm, obstacle centred or
35 mm off-centre: 81 of 81 runs pass without contact (closest gap
57–150 mm), rejoin the line at about 30° and resume line following. A
bypass takes about 20–30 s at these speeds.

The glue logs `[INFO](line:nav) state N cause N`, `plan: ...`, `phase N: ...`,
`steer N: ...`, `escape turned N deg`, motion changes and every `SCAN_RESULT`
(see [debugging](debugging.md) for the format), and writes all of it to the
flight recorder.

### Flight recorder (`flight_log.c`)

The car has no wireless link, so `main.uf2` keeps a log in RAM
(1024 records, about 2 minutes of driving round obstacles) and prints it
over USB when a serial monitor is opened.

**How to read it after a run:**

1. Do **not** switch the car off (RAM is lost). Pick it up so the wheels
   are off the floor.
2. Plug the USB cable into the laptop and open the serial monitor
   (VS Code Serial Monitor or `screen`, any baud).
3. The log prints once, oldest first. Close and reopen the monitor to
   print it again.

**What is in it:** every Buddy 5 state change with its cause, OBSTACLE,
FAULT, every SCAN_RESULT with the full 37-slot profile, every steering
change with how far the car has turned, every motion-service state/fault
change, every command sent, and a trace line (state, echo, IR sensors,
wheel distances) every 100 ms while Buddy 5 drives and every 500 ms
otherwise.

**Tethered alternative:** run the car on a long USB cable with the
monitor open; the live `(line:nav)` lines show the same events as they
happen.

The speeds, lease, brake wait, retry time and LINE_FOUND rate are the
`OBSTACLE_NAV_*` settings in `obstacle_config.h`.

**Changes outside `src/navigation/obstacle/` for Phase 2:**

| File | Change |
|---|---|
| `src/main.c` | +1 include (`obstacle/obstacle_nav.h`); +4 lines calling `obstacle_nav_init()` after `ir_capture_init()`; the steering `if` gets a new first branch `if (obstacle_nav_step(state.left, state.right)) { }` and the old first `if` becomes `else if`. Nothing else changed. |
| `CMakeLists.txt` | Buddy 5 block: comment updated; new `obstacle_nav` library (`obstacle_nav.c`, `flight_log.c`); `target_link_libraries(main PRIVATE obstacle_nav)`. |
| `src/motion/controller.h`, `src/motion/controller.c` (Buddy 2's motion code, applied with the user's approval) | Glitch tolerance: a wheel may have up to 3 consecutive invalid speed samples (30 ms) before FEEDBACK is latched; during a bad sample the wheel keeps the duty already applied and the controller update is skipped. New field `bad_samples[]`, new `MAX_BAD_SAMPLES 3U`, counter reset when idle. Reason: single encoder spikes when motors start, stop or reverse latched FEEDBACK (reboot only) in 5 car runs, on both wheels, with 1-2 bad steps each. Other checks (NO_EDGES, GAP, missing sample, expiry, STOP pin) unchanged. Needs Buddy 2's review. |

No IR or `sys` file was changed. The only motion change is the glitch tolerance above.

**Before driving on the floor:**

1. Run the [hardware checklist](obstacle_hardware_checklist.md) H1–H8 with
   `obstacle_bench.uf2`, especially the servo direction (H2b). A wrong
   direction makes the car pass on the wrong side.
2. Fill in the **FILL** values in `obstacle_config.h`.
3. Flash `main.uf2` with the wheels off the floor and put a hand in front
   of the sensor: the wheels must stop.
4. Then H9 on the floor.

Known points for the team:

- After a **cold power-on** the kernel leaves the RP2040 TIMER in reset with
  no 1 µs tick (`hw_setting.c` and `cpu_clock.c` never start it); only
  drag-and-drop flashing leaves it running. Buddy 5 now starts it in
  `obstacle_init()` (release-only, never resets it). The motion service
  also reads this TIMER and starts before Buddy 5 in `main.c`, so the
  proper team fix is to start the TIMER at the top of `usermain()`.

- `motion_service_request_stop()` latches until reboot, so Buddy 5 uses the
  obstacle hold instead (see Phase 2).
- **Open, line follower:** `line_task` sends 20 ms speed leases on a loop
  that waits up to 20 ms, so a lease can expire before the next one and
  latch `MOTION_FAULT_EXPIRED` until reboot (`motion state=4 fault=2` in the
  log). It was hidden while the TIMER was frozen after power-on; Buddy 5
  now starts the TIMER (TIMERAWL rule), so it can show up. Suggested fix
  for the owner: a lease of 40–200 ms, for example `5 * poll_ms * 1000`.
- `obstacle_nav.c` logs `(line:nav) motion state=.. fault=.. hold=..` whenever
  the motion service changes state, so silent motor faults show up.
- If the ultrasonic sensor is unplugged, Buddy 5 faults with NO_ECHO after 3 pings and
  holds the car, so line following stops too. This is deliberate, and the car
  does not restart by itself (D151).
- GP21 is the motion stop button in `main.uf2` and the "send" button in
  `obstacle_bench.uf2`.
- `gpio_irq_init()` enables the IRQ only on the core that calls it.
- The kernel's `BOARD_LED_PIN` is GP16, which is Buddy 2's encoder pin.
- At ~41 mm/s the hug may need more than 5 s to reach the line.
