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
  - after 4 s in REACQUIRE → LINE_FOUND
  - FAULT → RESET + START after 3 s

## Files

| File (in `src/obstacle/`) | Job | Pure (no kernel calls) |
|---|---|---|
| `b5_cfg.h` | Pins, timings, tunables, **FILL** placeholders, compile-time range checks | yes |
| `b5_pack.h` | Codes and packed-word layouts (masks and shifts) | yes |
| `scan_algo.c/.h` | Echo classification, ping agreement, coarse → fine scan engine | yes |
| `b5_plan.c/.h` | Beam-corrected width, side choice, start angle, bypass geometry and side-scan judgement, integer trig | yes |
| `b5_fsm.c/.h` | Statechart (no kernel calls; the task carries out its requests) | yes |
| `b5_comms.c/.h` | Command queue, event outbox, published copies, links to Buddy 2 (private) | no |
| `scan_hw.c/.h` | GP0/GP1/GP12 set-up, servo PWM, TIMER check, echo ISR, trigger | no |
| `scan_task.c/.h` | The task: commands → time limits → servo/settle → 60 ms ping | no |
| `obstacle.c/.h` | Public API: start-up order, then forwards to `b5_comms` | no |
| `b5_nav.c/.h` | Phase 2 glue: line task ↔ Buddy 5 ↔ motion service (hold, bypass, virtual bumper, odometer) | no |
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
| REACQUIRE (bypass) | set by Navigation (`obstacle_set_look()`): 0° while turning and moving, a full −60°..+60° sweep at every side scan | LINE_FOUND(id); 120 s / 4 m limit |

These rules apply in every state:

- **STOP command or any fault** → STOPPED, plus a FAULT event.
- **Fault causes** also call the emergency-stop link.
- **Collision guard** (REACQUIRE only): 2 readings under 80 mm → emergency
  stop. WAIT_STOP (already braking) and BYPASS (Navigation backs away from a
  close obstacle first) are not guarded; Navigation's virtual bumper covers
  contact in every bypass phase.

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

**Revised bypass [Chat]** (`b5_nav.c`, geometry in `b5_plan.c`). Every
position is the middle of the wheel axle; the origin is where the car
stopped, "ahead" is along the line and "side" is towards the side being
passed. Two kinds of rotation:

- **Turn:** a 90° pivot on one stopped wheel (moves the axle half a track,
  53 mm, sideways and forwards).
- **Spin:** both wheels at the same speed in opposite directions, so the
  car rotates on the middle of its axle (no shift).

Headings are absolute (90° away, 0°, −90° = facing the line, 0°), so errors
do not add up.

| Phase | What the car does | Ends when |
|---|---|---|
| BACKUP | reverse at 70 mm/s (only if needed) | backed up `177 + 50 − 69 − B` mm (the far front corner swings 177 mm round the stopped wheel) |
| TURN_AWAY | turn away (escape, then REACQUIRE); the direction is remembered | heading 90° away |
| OUT | straight, heading held | moved E |
| TURN_BACK | turn back | heading 0° |
| LEVEL | straight, heading held | moved B + 10 mm (level with the near face) |
| SPIN_FACE | spin to face the line (and the obstacle) | heading −90° |
| SWEEP | stand still; servo −60°..+60° in 5° steps, 2 pings each | judged (below) |
| SPIN_BACK | spin back | heading 0° |
| MOVE_ON | straight, heading held | moved D + C, then SPIN_FACE and SWEEP again |
| TO_LINE | slowly (70 mm/s) at the line, heading held | IR black for ≥ 8 mm of travel, no more than 80 mm short of the line; stop 300 mm past it |
| CREEP | straight | 70 mm more |
| ALIGN | turn the same way as TURN_AWAY | heading 0° → LINE_FOUND |

The numbers (measured: car length 127 mm, half width 63 mm, axle → sonar
68.5 mm, axle → IR 123.5 mm, tape 17.5 mm, track 107 mm):

- **B** = the closest reading of the first scan.
- **A** = `B · tan(cleared angle)`, the cleared angle being the first 5°
  slot past the edge on the chosen side (raw, no beam correction), capped
  at 150 mm (`B5_ROOM_MAX_MM`).
- **E** = `A + 127 + 20`.
- **C** = `63 + 30` = 93 mm (half the car plus a buffer).
- **Side scan judgement** (`b5_plan_side_scan()`), with the car facing the
  line. Each reading becomes a point `f = r·cos α` towards the line and
  `s = r·sin α` along it (α positive = forward along the line). Points past
  the line + 100 mm are ignored.
  - **Lane clear** (no point with |s| ≤ C) → TO_LINE.
  - Otherwise **d** = the closest point, the **forward cleared angle** is
    one slot past the last forward hit (0° with none), and the car moves on
    **D + C** with `D = d · tan(forward cleared angle)`. With no forward
    cleared angle (hits up to +60°), 30° is used.
  - Still blocked after 5 side scans → stop.
- **Creep** = `124 − 53 − (9 − 8)` = 70 mm, so the last turn ends with the
  axle on the middle of the tape.

Safety: the **virtual bumper** stops the car if a wheel asked for
≥ 70 mm/s measures under 40 % of it for 150 ms; the collision guard (REACQUIRE)
stops it for 2 readings under 80 mm; a collision, a bump, a box still
blocking after 5 scans or a missed line stops the car until it is reset (no
automatic retry, which would drive it back into the obstacle). Before any
wheel reverses (every spin and its correction) both wheels stop for 300 ms.

Known behaviour: the beam is about 19° wide, so right after the car has
moved past the far corner the next sweep can still place that corner just
inside the lane; the car then makes one more short loop (C = 93 mm).

Worked example (150 mm box, closest 233 mm, cleared 30°): A = 134, E = 281,
LEVEL 243 mm; side scan 1 blocked (forward cleared 35°, d 250 → move on
268 mm); side scan 2 clear → TO_LINE. Host simulation of the real
`b5_nav.c` (kinematic car, cone sonar), all without contact: 150 mm box
both sides, boxes 80–300 mm wide, 400 and 700 mm deep (3 and 4 side
scans), rotated 30° and 45°, obstacles 70–100 mm away (car backs up first),
a floor-mat gap (ignored, too short). A 1500 mm deep box stops after 5 side
scans. A whole run takes about 30–55 s.

STEER words use the order of Buddy 2's `motion_drive_action_t`:
STRAIGHT / LEFT / RIGHT / HOLD. The statechart publishes "away" during the
escape and "straight" in REACQUIRE; Navigation drives from its own odometry
and only logs them.

### Interfaces (Design review 4.1)

```c
obstacle_links_t links = { h_nav_mbf, p_stop, p_get_distance_mm };  /* 0/NULL allowed */
obstacle_init(&links);              /* processor 1, after gpio_irq_init() */
obstacle_post_cmd(b5_pack_cmd(B5_CMD_SCAN_REQ, req_id));
obstacle_get_event(&evt, tmout);    /* own 4-slot outbox while h_nav_mbf == 0 */
obstacle_get_steer();  obstacle_get_status();  obstacle_get_faults();
obstacle_set_look(deci);            /* servo angle in REACQUIRE (Navigation) */
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
| Bypass speed `B5_BYPASS_SPEED_MM_S` | 140 mm/s (the 25 % duty cap tops out near 142 mm/s) |
| Beam half-angle `B5_BEAM_HALF_DECI` | **Done:** 9.5° (95). Power bank at 300 mm dropped out with its near edge 50 mm left / 65 mm right of centre (9.5° / 12.2°); the smaller value gives more room. The left/right difference comes from the two transducers sitting either side of the centre. |
| Wheel diameter | **Done:** 65 mm (already in the motion service scale) |
| Wheel track `B5_WHEEL_TRACK_MM` | **Done:** 107 mm (measured) |
| Axle → sonar / IR / front `B5_AXLE_TO_*_MM` | **Done:** 68.5 / 123.5 / 133 mm (rounded up to 69 / 124 / 133) |
| Tape width `B5_TAPE_W_MM` | **Done:** 17.5 mm (18) |
| Assumed depth `B5_DEPTH_ASSUMED_MM` | 220 mm (covers a 150 mm box corner-first); set to the deepest real obstacle |
| Servo→tail `B5_SERVO_TO_REAR_MM` | Pending (170 mm); only the planner's corridor check uses it |
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
| `p_get_distance_mm` | Mean forward `displacement_um` of both wheels from `motion_service_read_status()`, in mm | Real odometer for the 4 m bypass limit. It keeps the last good value if a sample is invalid. |
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
| BYPASS | releases the hold; starts the bypass (BACKUP if needed, then TURN_AWAY); Buddy 5 moves on to REACQUIRE once the way ahead is clear | paused |
| REACQUIRE | the rest of the bypass (see Plan and bypass); sets the servo angle with `obstacle_set_look()`; ends with LINE_FOUND(id) (at most every 100 ms) after ALIGN | paused |
| STOPPED (fault) | holds the car; sends RESET + START every 3 s until Buddy 5 is back in MONITOR, except after a collision or a bypass Navigation gave up on (bump, still blocked after 5 side scans, line missed): those stay stopped until the car is reset | paused |

**Speeds** (`motion_service_set_speed()`, lease 100 ms). The car stands
still whenever the servo turns (entering BYPASS or MONITOR, and every look
angle change in REACQUIRE) until 2 new pings have arrived: a servo swing
while a motor drives can disturb the wheel encoders.

| Phase | Left / right wheel (passing right) |
|---|---|
| BACKUP | −70 / −70 mm/s |
| Turns (pivots) | 140 / 0 (away) or 0 / 140 (towards); the driven wheel slows to 70 for the last 10° |
| Spins | +70 / −70 or −70 / +70; both slow to 46 for the last 10° |
| Straight legs | 140 / 140 (TO_LINE and CREEP 70 / 70); a heading correction runs one wheel 14 mm/s slower |

NOTE: The stopped wheel of a pivot is at 0 (the motion service does not
check a wheel told to stand still). A wheel changing direction (after
BACKUP, at the start of a spin from a straight leg, or a spin correcting an
overshoot) first stops both wheels for 300 ms.

**Dead reckoning.** Heading = `(right − left wheel distance) /
B5_WHEEL_TRACK_MM` (from the totals, so it does not drift); position is
summed along that heading every pass. `B5_WHEEL_TRACK_MM` is 107 mm
(measured). The wheel diameter
(65 mm) is already in the motion service's scale (204.2 mm per turn).

**Simulation:** see the worked example under Plan and bypass.

The glue prints `[b5-nav] state N cause N`, `[b5-nav] plan: ...`,
`[b5-nav] phase N: side .. ahead .. heading ..`, `[b5-nav] steer N: left ..
right ..`, `[b5-nav] turned to N deg`, `spun to N deg`, `side scan N: ...`,
`line ...`, `HALT: ...`, motion changes and every `SCAN_RESULT`, and writes
all of it to the flight recorder.

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
change with how far the car has turned, every bypass phase, the PLAN (B,
cleared angle, A, E), every side scan (judgement, d, move on, and its 25
readings), every line contact
(found, or ignored and why), a BUMP or HALT, every motion-service
state/fault change, every command sent, and a trace line (state, echo, IR sensors,
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
| `src/main.c` | +1 include (`obstacle/b5_nav.h`); +4 lines calling `b5_nav_init()` after `ir_capture_init()`; the steering `if` gets a new first branch `if (b5_nav_step(state.left, state.right)) { }` and the old first `if` becomes `else if`. |
| `src/main.c` (TEMP, needs the line follower's owner) | Speed lease `3 * poll_ms` (was `poll_ms`, which expired between passes and latched EXPIRED); at a junction (both IR black) drive straight instead of stopping. |
| `src/main.c` (Buddy 2's settings, applied with the user's approval) | `kd_q16` 16 → 2 (derivative noise made the wheels jitter); `max_target_um_s` 100 000 → 200 000; line speed 40 841 → 163 364 µm/s (4×). NOTE: 163 mm/s is above the ~142 mm/s the wheels reach at the 25 % duty cap. |
| `src/motion/motion_pins.h` | Encoder pins follow the Week 6 design-review table (left GP4/5, right GP16/17); the plugs were moved to match, so `B5_ENCODERS_CROSSED` is 0. |
| `CMakeLists.txt` | Buddy 5 block: comment updated; new `obstacle_nav` library (`b5_nav.c`, `b5_log.c`); `target_link_libraries(main PRIVATE obstacle_nav)`. |
| `src/motion/motion_control.h`, `src/motion/motion_control.c` (Buddy 2's motion code, applied with the user's approval) | Glitch tolerance: a wheel may have up to 3 consecutive invalid speed samples (30 ms) before FEEDBACK is latched; during a bad sample the wheel keeps the duty already applied and the controller update is skipped. New field `bad_samples[]`, new `MAX_BAD_SAMPLES 3U`, counter reset when idle. Reason: single encoder spikes when motors start, stop or reverse latched FEEDBACK (reboot only) in 5 car runs, on both wheels, with 1-2 bad steps each. Other checks (NO_EDGES, GAP, missing sample, expiry, STOP pin) unchanged. Needs Buddy 2's review. |

No IR or `sys` file was changed. All `main.c`, `motion_pins.h` and motion
changes need their owners' review.

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
- **Line follower lease (TEMP fix applied):** `line_task` sent 20 ms speed
  leases on a loop that waits up to 20 ms, so a lease could expire before
  the next one and latch `MOTION_FAULT_EXPIRED` until reboot. It was hidden
  while the TIMER was frozen after power-on; Buddy 5 starts the TIMER, so it
  showed up. `main.c` now uses `3 * poll_ms`; the owner should confirm.
  Printing the flight log while the car drives can still delay the loop
  enough to expire a lease.
- `b5_nav.c` prints `[b5-nav] motion state=.. fault=.. hold=..` whenever
  the motion service changes state, so silent motor faults show up.
- If the ultrasonic sensor is unplugged, Buddy 5 faults with NO_ECHO and
  holds the car, so line following stops too. This is deliberate.
- GP21 is the motion stop button in `main.uf2` and the "send" button in
  `obstacle_bench.uf2`.
- `gpio_irq_init()` enables the IRQ only on the core that calls it.
- The kernel's `BOARD_LED_PIN` is GP16, which is Buddy 2's encoder pin.
- A bypass takes about 30–55 s: each side scan loop (spin, 25-angle sweep,
  spin back, move on) takes about 11 s.
