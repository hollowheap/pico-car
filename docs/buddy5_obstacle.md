# Buddy 5: obstacle detection and avoidance

Ultrasonic sensor (HC-SR04+), scan servo (SG90), coarse/fine scan, bypass
planning and the obstacle statechart (design review 6.2). The code is in
`src/obstacle/`. Buddy 5 never writes the motors: it only sends events and
steering words to Navigation.

Other Buddy 5 docs:

- [File guide: which file does what](buddy5_file_guide.md)
- [Hardware test checklist](buddy5_hardware_checklist.md)

## Status

Last updated 30 October 2026.

| Step | Work | Current position |
|---|---|---|
| 1 | Design (pins, statechart, scan, bypass) | Done; agreed with design review sections 2, 4.1 and 6.2. |
| 2 | Phase 1 code in `src/obstacle/` | Done. |
| 3 | Laptop tests | Done: 194 checks passed. The test folder was then taken out of the repo and kept as a backup zip (`buddy5_tests_backup.zip`). |
| 4 | BARR-C:2018 pass | Done; see [Coding standard](#coding-standard-barr-c2018). |
| 5 | Bench build in the normal team build | Done; plain `./configure.sh` + `./build.sh` makes `build/obstacle_bench.uf2`. `main.uf2` is unchanged. |
| 6 | Car measurements (**FILL** values) | Pending; see [Numbers to fill in](#numbers-to-fill-in). |
| 7 | Hardware checklist H1–H8 on the car | Pending; see the [checklist](buddy5_hardware_checklist.md). |
| 8 | Phase 2 integration with the line task | Code done and builds into `main.uf2`; floor test (H9) pending. See [Phase 2](#phase-2-integration--in-mainuf2). |

Outside `src/obstacle/`, Buddy 5 changes the "Buddy 5" block at the end of
the team `CMakeLists.txt`, five lines of `src/main.c` (Phase 2), and adds
the three `docs/buddy5_*.md` files.

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

| File (in `src/obstacle/`) | Job | Pure (no kernel calls) |
|---|---|---|
| `b5_cfg.h` | Pins, timings, tunables, **FILL** placeholders, compile-time range checks | yes |
| `b5_pack.h` | Codes and packed-word layouts (masks and shifts) | yes |
| `scan_algo.c/.h` | Echo classification, ping agreement, coarse → fine scan engine | yes |
| `b5_plan.c/.h` | Beam-corrected width, side choice, start angle, hug rules, integer trig | yes |
| `b5_fsm.c/.h` | Statechart (no kernel calls; the task carries out its requests) | yes |
| `b5_comms.c/.h` | Command queue, event outbox, published copies, links to Buddy 2 (private) | no |
| `scan_hw.c/.h` | GP0/GP1/GP12 set-up, servo PWM, TIMER check, echo ISR, trigger | no |
| `scan_task.c/.h` | The task: commands → time limits → servo/settle → 60 ms ping | no |
| `obstacle.c/.h` | Public API: start-up order, then forwards to `b5_comms` | no |
| `b5_nav.c/.h` | Phase 2 glue: line task ↔ Buddy 5 ↔ motion service (hold, STEER → speed, odometer) | no |
| `b5_bench_main.c` | Standalone `usermain` with a simulated Navigation | no |

All files above are in `src/obstacle/`. The build rules are the "Buddy 5"
block at the end of the team `CMakeLists.txt`. The
[file guide](buddy5_file_guide.md) explains each file in more detail.

## How it works

### Tasks and kernel objects

| Item | Value |
|---|---|
| Buddy 5 task | priority 6, processor 1 (core 0), 4096 B stack |
| Echo ISR | 1 slot in the shared `src/sys/gpio_irq` dispatcher, processor 1 |
| Kernel objects created | **1 event flag** (`flg_obs_id`); no message buffer, cyclic handler or mutex |
| PWM | slice 6 channel A (GP12), 125 MHz / 125 = 1 MHz, TOP 19999 = 50 Hz |

### One ping (60 ms)

1. Empty the mailbox with interrupts paused (three statements, about 0.2 µs).
   If ECHO is already high, the ping is **STUCK** and no trigger is sent.
2. Send a 12 µs TRIG pulse.
3. The ISR records the rise time, then on the fall writes `width | RISE | DONE`
   in one store and sets the flag bit.
4. The task waits at most 40 ms, then classifies the ping:

| Class | Meaning |
|---|---|
| VALID | 116–23 500 µs |
| OPEN | rise seen, pulse beyond 4 m |
| UNKNOWN | no rise, a glitch, or a missed fall |
| STUCK | ECHO high before the trigger |

A missing echo is **never** OPEN.

Fault counters:

- 5 UNKNOWN in a row → NO_ECHO. This works with ECHO unplugged, because the
  task raises it on timeout.
- 3 STUCK in a row → ECHO_STUCK.

### Statechart

| State | Servo | Leaves when |
|---|---|---|
| STOPPED (power-on, latched) | 0° | RESET (only if the cause is cleared), then START |
| MONITOR | 0° | 2 VALID ≤ 250 mm in a row → OBSTACLE event |
| WAIT_STOP | 0° | SCAN_REQ(id); 1 s limit |
| SCANNING | sweeps | profile done → SCAN_RESULT event; 8 s limit |
| PLAN | 0° | ACCEPT(id) → BYPASS (or MONITOR if CLEAR); REJECT → STOPPED; 1 s limit |
| BYPASS (escape) | 0° | forward beam clear for 2 pings in a row; 10 s limit |
| REACQUIRE (out, along, back) | ±45° toward obstacle | LINE_FOUND(id); 60 s / 2.5 m limit |

These rules apply in every state:

- **STOP command or any fault** → STOPPED, plus a FAULT event.
- **Fault causes** also call the emergency-stop link.
- **Collision guard** (WAIT_STOP / BYPASS / REACQUIRE): 2 readings under 80 mm
  → emergency stop.

### Scan

- **Grid:** 25 slots, −60° to +60° in 5° steps, positive = left.
- **Coarse:** every 3rd slot, swept right to left.
- **Region:** the closest VALID ≤ 600 mm, plus coarse neighbours within
  closest + 250 mm. The closest reading is looked for inside the car's path
  (±30°) first, and only then over the whole sweep, so clutter off to one
  side cannot take the place of the obstacle that stopped the car.
- **Fine:** walk outward from each edge, one slot at a time, up to the first
  real miss (OPEN, or VALID beyond closest + 250 mm).
  - An UNKNOWN or STUCK slot proves nothing, so the walk assumes the obstacle
    goes on and steps past it (the car gets more room, never less).
  - An edge that reaches ±60° without a miss is **unbounded**, and that side
    is treated as unsafe.
- **Per slot:**
  - settle for 3 ms/° + 20 ms;
  - ping until two pings agree (same class, within 15 mm), keeping the closer
    distance;
  - after 4 pings without agreement the slot is UNKNOWN.

### Plan and bypass

**Beam-cone correction** (Tribunal decision). With h = last hit angle,
m = first miss angle and β = half-cone, an edge lies between h − β and m − β.

- **Reported width:** the middle estimates of the two edges, joined by the law
  of cosines.
- **Side clearance:** uses the safe outer bound, `x = r · tan(m − β)`.

**Choosing a side.** A side is usable when:

- the sideways offset it needs (edge + half-width + margin) still fits on the
  free floor, and
- no other reading on that side falls in the swept band.

The usable side with the smaller offset wins; on a tie, left. If neither side
is usable → NO_SAFE_PATH.

**Start angle:** `atan(needed offset / closest distance)`.

**Bypass [Chat].** The car never reverses; it turns by slowing one wheel.

1. **Escape (BYPASS):** spin away on the spot (inside wheel −40 mm/s,
   outside wheel +40 mm/s) until the forward beam has been clear for 2
   pings in a row. NOTE: a pivot with the inside wheel held dragged that
   wheel backwards on the car, the encoder reported invalid steps and the
   motion service latched FEEDBACK; the spin drives both wheels.
2. **TURN:** if the beam cleared before the car had turned the planned start
   angle, keep pivoting until it has (the beam is on the car's centre line,
   so "beam clear" alone would let the car's side clip the corner).
3. **OUT:** straight on at that angle until the car is the planned offset
   to the side of the line (from the wheel odometry).
4. **ALONG (zig-zag):** hold that offset (−10 / +30 mm) and a heading within
   ±30° of the line, by curving gently (inside wheel 40 mm/s, outside
   60 mm/s), until the tail has passed the far end of the obstacle
   (closest + assumed depth 180 mm + servo-to-tail 170 mm).
5. **BACK:** curve towards the line until heading 30° across it, then
   straight on. The IR sensors count from ALONG on (the car is then beside
   the obstacle, never over the line it left), and the first line contact
   hands back to line following at once.

Throughout 3–5 the servo looks 45° towards the obstacle; a reading under
150 mm bends the car away. The statechart still publishes its own hug
STEER words (band below), but Navigation drives from the odometry.

| Look-ray reading | STEER word published |
|---|---|
| < 150 mm | away |
| 150–250 mm | straight |
| > 250 mm or OPEN | towards |
| UNKNOWN | keep the last choice |

NOTE: An earlier version steered only from the 45° look-ray and accepted
the line once the ray had seen and then lost the obstacle. A simulation
showed why that fails: while the car is still angled out, the 45° ray
points almost straight ahead, loses the obstacle early and the car curves
back into its side. Dead reckoning fixes this.

STEER words use the order of Buddy 2's `motion_drive_action_t`:
STRAIGHT / LEFT / RIGHT / HOLD. In Phase 2, Navigation turns them into
`motion_service_submit_request()` calls.

### Interfaces (Design review 4.1)

```c
obstacle_links_t links = { h_nav_mbf, p_stop, p_get_distance_mm };  /* 0/NULL allowed */
obstacle_init(&links);              /* processor 1, after gpio_irq_init() */
obstacle_post_cmd(b5_pack_cmd(B5_CMD_SCAN_REQ, req_id));
obstacle_get_event(&evt, tmout);    /* own 4-slot outbox while h_nav_mbf == 0 */
obstacle_get_steer();  obstacle_get_status();  obstacle_get_faults();
obstacle_copy_profile(profile);     /* telemetry copy: E_BUSY, never waits */
```

| Commands in (3 bits) | Events out (2 bits) |
|---|---|
| SCAN_REQ, ACCEPT, REJECT, LINE_FOUND, STOP, RESET, START | OBSTACLE, SCAN_RESULT, FAULT, STEER |

## Bit table (Change Requirement 1)

| Word | Field | Valid range | Bits | Representation |
|---|---|---|---|---|
| Echo mailbox `u32` | width_us | 0–40 000 µs (sat. 65 535) | 16 | bits 0–15, written whole by the ISR |
| | RISE, DONE | flags | 1 + 1 | bits 16, 17 |
| Scan slot `u16` × 25 | dist_mm | 0–4095 (VALID ≤ 4030) | 12 | bits 0–11 |
| | class | 4 classes | 2 | bits 12–13 |
| | measured | flag | 1 | bit 14 (bit 15 spare) → **50 bytes** |
| Status `u32` (one store) | state | 7 states | 3 | bits 0–2 |
| | stop cause | 14 causes | 4 | bits 3–6 |
| | last class | 4 | 2 | bits 7–8 |
| | last dist_mm | 0–4095 | 12 | bits 9–20 |
| | scan_req_id | 0–31 | 5 | bits 21–25 |
| | seq (liveness) | 0–63 wrap | 6 | bits 26–31 |
| Fault byte `u8` | 8 fault flags | – | 8 | single writer (Buddy 5 task) |
| Command `u8` | type, id | 8 types, 0–31 | 3 + 5 | one byte |
| Event `u32` | type, id | 4 types, 0–31 | 2 + 5 | bits 0–6 |
| | OBSTACLE: dist | 0–4095 | 12 | bits 7–18 |
| | SCAN_RESULT: status, side, width, angle | 4, 2, 0–1023 mm, 0–60° | 2 + 1 + 10 + 6 | bits 7–25 |
| | FAULT: cause, faults | 14, byte | 4 + 8 | bits 7–18 |
| | STEER: action, seq | 4, 0–63 | 2 + 6 | bits 7–14 |
| Counters `u16` (task-private) | unknown, stuck, hits, collision, clear | 0–5, 0–3, 0–2, 0–2, 0–4 | 3+2+3+2+3 | bits 0–12 |

Compile-time checks in `b5_cfg.h` stop the build if a field can't hold
its range. They use a negative array size, because C99 has no
`_Static_assert`.

## Coding standard (BARR-C:2018)

Every file in `src/obstacle/` was checked against BARR-C:2018 with:

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
| `b5_`, `scan_algo_`, `scan_hw_`, `scan_task_`, `obstacle_` | module prefix on every public function and type | 5.1.c, 6.1.i |
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
| b5 | Buddy 5 (this subsystem) |
| cls | echo class (VALID / OPEN / UNKNOWN / STUCK) |
| cmd | command |
| ct | compile-time (in `B5_CT_ASSERT`) |
| deci | deci-degrees, 0.1° (positive = left) |
| dist | distance |
| evt | event |
| fsm | finite-state machine (the statechart) |
| mbf | kernel message buffer |
| mbx | echo mailbox word |
| mm, ms, us | millimetres, milliseconds, microseconds (as unit suffixes) |
| odo | odometer (distance travelled) |
| ptn | event-flag bit pattern (kernel term) |
| q14 | fixed point with 14 fraction bits (16384 = 1.0) |
| req | request |
| seq | sequence number |
| sh, mk | shift and mask of a packed field (`B5_*_SH`, `B5_*_MK`) |
| st, ev, cnt | status, event and counter word fields (`B5_ST_*`, `B5_EV_*`, `B5_CNT_*`) |
| stacd, exinf, intsts, tmout | kernel parameter names, as in the T-Kernel spec |
| um | micrometres (bench odometer only) |

### Deviations

BARR-C allows documented deviations. Each one is also noted in a `NOTE:`
comment next to the code it affects.

| Rule | Deviation | Why |
|---|---|---|
| 5.2.a | Kernel types `ID`, `ER`, `INT`, `UINT`, `UW`, `TMO`, `BOOL`, `SYSTIM`, `T_CTSK`, `T_CFLG` are used at kernel and BSP calls | The kernel API is written in those types. All Buddy 5 data is `stdint.h` types. |
| 1.6.a | In `b5_bench_main.c`, `(UB const *)` on string literals and `(INT)` / `(UINT)` on printf arguments are explained once, at the top of the file | Dozens of identical, value-safe casts; one note is clearer than one per line. Every other cast in the folder has its own comment. |
| 4.2.a | `b5_bench_main.c` has no header | Nothing calls into a main module. `b5_cfg.h` and `b5_pack.h` have no `.c` because they hold only constants and inline helpers. |
| 6.1.i | `usermain()` has no module prefix | The kernel calls it by that name. |
| 6.3.a | `B5_CT_ASSERT` is a function-like macro | A compile-time check needs the preprocessor. Each parameter is used once and parenthesised. |
| 6.5.a | `echo_isr()` has no interrupt `#pragma` | The kernel's interrupt entry runs the shared dispatcher, which calls it as an ordinary C function. It is still `static`, named `_isr` and placed last in `scan_hw.c` (6.5.b, 6.5.c). |
| 8.3.a, 3.3.c | The team `.clang-format` puts `break;` inside the case body and removes the blank line after `/*** end of file ***/` | After running the team `.clang-format` on a Buddy 5 file, move each `break;` back to line up with its `case`, and make sure the file ends with the end-of-file comment followed by one blank line. |
| 1.8.c | The spinlock words and `g_fsm` are not declared `volatile` by Buddy 5 | `T_SPLOCK` is already `volatile` in the kernel's own typedef; `g_fsm` is touched by the Buddy 5 task only. |

## Numbers to fill in

Search `b5_cfg.h` for **FILL**. Status:

| Setting | Status |
|---|---|
| Car half-width `B5_CAR_HALF_W_MM` | **Done:** 63 mm (car is 126 mm wide) |
| Free floor `B5_FREE_FLOOR_MM` | **Done:** 500 mm cap; the course has no side boundary, and the planner always uses the smaller offset |
| Servo direction `B5_SERVO_DIR` | **Confirmed** on the bench (+1) |
| Bypass speed `B5_BYPASS_SPEED_MM_S` | 40 mm/s, matches `main.c` (40.8 mm/s) |
| Beam half-angle `B5_BEAM_HALF_DECI` | **Done:** 9.5° (95). Power bank at 300 mm dropped out with its near edge 50 mm left / 65 mm right of centre (9.5° / 12.2°); the smaller value gives more room. The left/right difference comes from the two transducers sitting either side of the centre. |
| Wheel diameter | **Done:** 65 mm (already in the motion service scale) |
| Wheel track `B5_WHEEL_TRACK_MM` | **Done:** 107 mm (measured) |
| Servo→tail `B5_SERVO_TO_REAR_MM` | Pending (170 mm); sets when the car turns back |
| Servo→front | Pending (optional) |
| Servo centre / scale | Looks right by eye; protractor check optional |
| Power source | Pending (settle time on a low LiPo, H2d) |

## Phase 2 (integration) — in main.uf2

Buddy 5 now runs inside the normal `main.uf2`, driven by the existing line
task (`line_task` in `src/main.c`) through the glue module
`src/obstacle/b5_nav.c`.

**Start-up** (in `line_task`, processor 1): `gpio_irq_init()` →
`motion_service_start()` → `ir_capture_init()` → **`b5_nav_init()`**. That
calls `obstacle_init()` with these links, then sends RESET and START:

| Link | Wired to | Why |
|---|---|---|
| `p_stop` | `motion_service_set_hold(MOTION_HOLD_OBSTACLE, true)` | Stops the car without the reboot latch of `motion_service_request_stop()`, so RESET + START can recover. |
| `p_get_distance_mm` | Mean forward `displacement_um` of both wheels from `motion_service_read_status()`, in mm | Real odometer for the 2.5 m bypass limit. It keeps the last good value if a sample is invalid. |
| `h_nav_mbf` | 0 | Events go to Buddy 5's own outbox; the line task polls it. |

If `b5_nav_init()` fails, it prints the error and line following runs
without Buddy 5.

**Every loop pass** (every ≤ 20 ms) the line task calls
`b5_nav_step(left, right)` before its own steering. It returns true when
Buddy 5 is in charge, and the line follower then sends nothing.

| Buddy 5 state | What the glue does | Line follower |
|---|---|---|
| MONITOR | releases the obstacle hold, unless the latest reading is a VALID echo within 250 mm (so a retry never creeps into the obstacle) | drives as before |
| WAIT_STOP, SCANNING, PLAN | holds the car; 300 ms after OBSTACLE sends SCAN_REQ(id); on SCAN_RESULT OK/CLEAR sends ACCEPT(id) | paused |
| BYPASS | releases the hold; pivots away (STEER word) until Buddy 5 sees the way ahead is clear | paused |
| REACQUIRE | TURN / OUT / ALONG / BACK from the wheel odometry (see Bypass above); from ALONG on, sends LINE_FOUND(id) as soon as either IR sensor sees the line (at most every 100 ms) | paused |
| STOPPED (fault) | holds the car; sends RESET + START every 3 s until Buddy 5 is back in MONITOR | paused |

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
B5_WHEEL_TRACK_MM` (from the totals, so it does not drift); position is
summed along that heading every pass. `B5_WHEEL_TRACK_MM` is 107 mm
(measured). The wheel diameter
(65 mm) is already in the motion service's scale (204.2 mm per turn).

**Simulation** (host, the real Buddy 5 code in a closed loop with a
kinematic car), from detection to rejoining the line: obstacles 60/105/160 mm wide,
real beam 9.5°/15°/25°, true track 90/100/115 mm, obstacle centred or
35 mm off-centre: 81 of 81 runs pass without contact (closest gap
57–150 mm), rejoin the line at about 30° and resume line following. A
bypass takes about 20–30 s at these speeds.

The glue prints `[b5-nav] state N cause N`, `[b5-nav] plan: ...`,
`[b5-nav] phase N: side .. ahead .. heading ..`, `[b5-nav] steer N: left ..
right ..`, `[b5-nav] escape turned N deg`, motion changes and every
`SCAN_RESULT`, and writes all of it to the flight recorder.

### Flight recorder (`b5_log.c`)

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
FAULT, every SCAN_RESULT with the full 25-slot profile, every steering
change with how far the car has turned, every motion-service state/fault
change, every command sent, and a trace line (state, echo, IR sensors,
wheel distances) every 100 ms while Buddy 5 drives and every 500 ms
otherwise.

**Tethered alternative:** run the car on a long USB cable with the
monitor open; the live `[b5-nav]` lines show the same events as they
happen.

The speeds, lease, brake wait, retry time and LINE_FOUND rate are the
`B5_NAV_*` settings in `b5_cfg.h`.

**Changes outside `src/obstacle/` for Phase 2:**

| File | Change |
|---|---|
| `src/main.c` | +1 include (`obstacle/b5_nav.h`); +4 lines calling `b5_nav_init()` after `ir_capture_init()`; the steering `if` gets a new first branch `if (b5_nav_step(state.left, state.right)) { }` and the old first `if` becomes `else if`. Nothing else changed. |
| `CMakeLists.txt` | Buddy 5 block: comment updated; new `obstacle_nav` library (`b5_nav.c`, `b5_log.c`); `target_link_libraries(main PRIVATE obstacle_nav)`. |
| `src/motion/motion_control.h`, `src/motion/motion_control.c` (Buddy 2's motion code, applied with the user's approval) | Glitch tolerance: a wheel may have up to 3 consecutive invalid speed samples (30 ms) before FEEDBACK is latched; during a bad sample the wheel keeps the duty already applied and the controller update is skipped. New field `bad_samples[]`, new `MAX_BAD_SAMPLES 3U`, counter reset when idle. Reason: single encoder spikes when motors start, stop or reverse latched FEEDBACK (reboot only) in 5 car runs, on both wheels, with 1-2 bad steps each. Other checks (NO_EDGES, GAP, missing sample, expiry, STOP pin) unchanged. Needs Buddy 2's review. |

No IR or `sys` file was changed. The only motion change is the glitch tolerance above.

**Before driving on the floor:**

1. Run the [hardware checklist](buddy5_hardware_checklist.md) H1–H8 with
   `obstacle_bench.uf2`, especially the servo direction (H2b). A wrong
   direction makes the car pass on the wrong side.
2. Fill in the **FILL** values in `b5_cfg.h`.
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
- `b5_nav.c` prints `[b5-nav] motion state=.. fault=.. hold=..` whenever
  the motion service changes state, so silent motor faults show up.
- If the ultrasonic sensor is unplugged, Buddy 5 faults with NO_ECHO and
  holds the car, so line following stops too. This is deliberate.
- GP21 is the motion stop button in `main.uf2` and the "send" button in
  `obstacle_bench.uf2`.
- `gpio_irq_init()` enables the IRQ only on the core that calls it.
- The kernel's `BOARD_LED_PIN` is GP16, which is Buddy 2's encoder pin.
- At ~41 mm/s the hug may need more than 5 s to reach the line.
