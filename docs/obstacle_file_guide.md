# Buddy 5 file guide

A map of every Buddy 5 file: what each one does, how they fit together, and
which one to open for a given job. For the design itself (statechart, scan,
bypass, bit tables, BARR-C notes) see
[the Buddy 5 design doc](obstacle.md).

Last updated 30 October 2026.

## 1. Folder map

```
docs/
├── obstacle.md           design, interfaces, BARR-C notes, Phase 2 plan
├── obstacle_file_guide.md         this guide
└── obstacle_hardware_checklist.md tests to run on the real car

CMakeLists.txt                   team file; its last block builds Buddy 5
include/navigation/obstacle.h    public facade (the only public header)
src/main.c                       team file; line_task calls obstacle_line_*()

src/navigation/obstacle/
├── obstacle_config.h                     settings: pins, timings, FILL values
├── obstacle_pack.h                    codes and packed-word layouts
│
├── scan_algo.c / .h             echo classing + scan engine        (pure)
├── planner.c / .h               width, side, angle, hug rules      (pure)
├── statechart.c / .h                the statechart                     (pure)
│
├── scan_hw.c / .h               sensor pins, servo PWM, echo ISR   (hardware)
├── scan_task.c / .h             the Buddy 5 task loop              (kernel)
├── comms.c / .h              queues, outbox, shared copies      (kernel)
├── obstacle.c / .h              implementation; obstacle.h is private (kernel)
│
├── obstacle_nav.c / .h                Phase 2 glue used by main.c       (kernel)
├── flight_log.c / .h                flight recorder, printed over USB  (kernel)
└── bench_main.c              bench program with a pretend Navigation
```

**Pure** means the file is plain logic with no hardware or kernel calls.
**Hardware/kernel** files talk to the pins, timer or kernel directly.

## 2. How the pieces connect

```
 Other buddies (Navigation / Buddy 2)
          │  commands in, events + steering out
          ▼
 ┌────────────────┐
 │  obstacle.c    │  public API: start-up order, then forwards calls
 └───────┬────────┘
         ▼
 ┌────────────────┐        ┌────────────────┐
 │  comms.c    │◄──────►│  scan_task.c   │  the task: one loop pass =
 │ queues, outbox │ pop    │                │  commands → time limits →
 │ shared copies  │ publish│                │  move servo → ping
 └────────────────┘        └──┬─────────┬───┘
                              │         │
                    feeds     ▼         ▼   drives
                  ┌────────────────┐  ┌────────────────┐
                  │  statechart.c      │  │  scan_hw.c     │
                  │  statechart    │  │ TRIG, ECHO ISR,│
                  └──┬─────────┬───┘  │ servo PWM      │
                     ▼         ▼      └────────────────┘
          ┌──────────────┐ ┌──────────────┐
          │ scan_algo.c  │ │  planner.c   │
          │ scan engine  │ │ plan + hug   │
          └──────────────┘ └──────────────┘

 Every file uses obstacle_config.h (settings) and obstacle_pack.h (codes and bit layouts).
```

**One ping, start to finish:**

1. `scan_task.c` asks `statechart.c` where the servo should point, and moves it
   with `scan_hw.c`.
2. `scan_hw.c` sends the 12 µs trigger. Its ISR times the echo and writes
   the result into a mailbox word.
3. `scan_task.c` hands the mailbox to `scan_algo.c`, which classes the
   reading (VALID / OPEN / UNKNOWN / STUCK) and turns it into millimetres.
4. `statechart.c` decides what the reading means (obstacle? scan step? steer?),
   using `scan_algo.c` and `planner.c`.
5. `comms.c` publishes what changed (events, steering, status) for
   Navigation to read.

## 3. File by file

Paths in this section are inside `src/navigation/obstacle/` unless they say otherwise.

Each entry answers the same four questions: what it does, its main
functions, who uses it, and when you would edit it.

### 3.1 Settings and formats

#### `obstacle_config.h`
- **What it does:** holds every number Buddy 5 depends on: pins, ping
  timing, detection distance, scan grid, servo pulse widths, time limits and
  car size. Each value says where it came from (`[Spec]`, `[DR x]`,
  `[Tribunal]`, `[Chat]`, `[Default]`). Compile-time checks stop the build
  if a value can't fit in its packed field.
- **Main items:** `OBSTACLE_TRIG_PIN`, `OBSTACLE_ECHO_PIN`, `OBSTACLE_SERVO_PIN`,
  `OBSTACLE_DETECT_MM`, `OBSTACLE_SERVO_*`, `OBSTACLE_CAR_HALF_W_MM`, `OBSTACLE_CT_ASSERT`.
- **Used by:** every other file.
- **Edit it when:** you have measurements for the **FILL** values, or you
  are tuning (servo centre, settle time, beam angle).

#### `obstacle_pack.h`
- **What it does:** defines the codes (states, causes, classes, commands,
  events, steering) and where each field sits inside the packed 8, 16 and
  32-bit words. Small inline helpers read and write those fields.
- **Main functions:** `obstacle_pack_get`, `obstacle_pack_put`, `obstacle_pack_slot`,
  `obstacle_pack_slot_dist`, `obstacle_pack_slot_class`, `obstacle_pack_slot_is_measured`,
  `obstacle_pack_cmd`.
- **Used by:** every other file, and by Navigation to build commands and
  read events.
- **Edit it when:** a new code or field is needed. Then update the bit table
  in the design doc.

### 3.2 Logic (pure: no hardware or kernel calls)

#### `scan_algo.c / .h`
- **What it does:** turns one echo into a class and a distance, makes two
  pings agree before a slot is trusted, and runs the scan: a coarse sweep
  every 15°, then fine steps outward from each edge of the obstacle.
- **Main functions:** `scan_algo_classify`, `scan_algo_dist_mm`,
  `scan_algo_agree_add`, `scan_algo_start`, `scan_algo_target`,
  `scan_algo_feed`, `scan_algo_has_region`.
- **Used by:** `statechart.c`, `planner.c`, `scan_task.c`.
- **Edit it when:** changing how echoes are classed or how the scan picks
  its next slot.

#### `planner.c / .h`
- **What it does:** after a scan, works out the obstacle width (corrected
  for the sensor's 15° beam), which side to pass on, and the start angle.
  During the bypass it gives the proportional hug command and the bypass
  distance budget. All maths is whole numbers: no floating point, no libm.
- **Main functions:** `obstacle_plan_cfg_default`, `obstacle_plan_build`,
  `obstacle_plan_hug_ctrl`, `obstacle_plan_hug_limit_mm`,
  `obstacle_plan_hug_steer`, `obstacle_plan_is_escape_clear`,
  `obstacle_plan_steer_away`,
  `obstacle_plan_steer_toward`, plus integer trig (`obstacle_plan_sin_q14`,
  `obstacle_plan_atan_deci`, `obstacle_plan_isqrt`, …).
- **Used by:** `statechart.c`, `scan_task.c` (for the default settings),
  `obstacle_nav.c` (hug controller).
- **Edit it when:** changing how a side is chosen or how the hug steers.

#### `statechart.c / .h`
- **What it does:** the statechart from design review 6.2: STOPPED →
  MONITOR → WAIT_STOP → SCANNING → PLAN → BYPASS → REACQUIRE → MONITOR, with
  every fault and time limit. It never calls the kernel. It only says what
  should happen (send this event, steer this way, call emergency stop), and
  the task carries that out.
- **Main functions:** `obstacle_statechart_init`, `obstacle_statechart_command`, `obstacle_statechart_ping`,
  `obstacle_statechart_tick`, `obstacle_statechart_servo_deci`, `obstacle_statechart_status`, `obstacle_statechart_out_clear`.
- **Used by:** `scan_task.c`, `comms.c` (to publish its status).
- **Edit it when:** changing a transition, a fault rule or a time limit.

### 3.3 Hardware and kernel (run on the Pico only)

#### `scan_hw.c / .h`
- **What it does:** sets up GP0 (TRIG), GP1 (ECHO) and GP12 (servo PWM, 50
  Hz), checks that the 1 MHz timer is ticking, sends the trigger pulse, and
  holds the ECHO interrupt handler. The ISR times the pulse and writes it
  into a mailbox word in one store.
- **Main functions:** `scan_hw_init`, `scan_hw_servo_set`,
  `scan_hw_trigger`, `scan_hw_rearm`, `scan_hw_mailbox`,
  `scan_hw_is_echo_high`, `scan_hw_now_us`; ISR `echo_isr` (private).
- **Used by:** `obstacle.c` (start-up), `scan_task.c` (every ping).
- **Edit it when:** pins, PWM settings or the ISR change.

#### `scan_task.c / .h`
- **What it does:** the Buddy 5 task (processor 1, priority 6). Each loop
  pass handles commands and time limits, then does one of: move the servo
  and wait for it to settle, wait for the 60 ms ping spacing, or take one
  ping and feed it to the statechart.
- **Main function:** `scan_task_obstacle_task` (the task body).
- **Used by:** `obstacle.c`, which creates the task.
- **Edit it when:** changing the loop order, settle timing or ping timing.

#### `comms.c / .h`
- **What it does:** everything shared between Buddy 5 and other tasks: the
  4-slot command queue, the 4-slot event outbox, the status and steering
  words, the fault byte, the profile copy, and the links to Buddy 2
  (emergency stop, odometer). Other tasks can read these without ever
  making Buddy 5 wait. Private: only `obstacle.c` and `scan_task.c`
  include it.
- **Main functions:** `obstacle_comms_init`, `obstacle_comms_post_cmd`,
  `obstacle_comms_pop_cmd`, `obstacle_comms_publish`, `obstacle_comms_get_event`,
  `obstacle_comms_odometer_mm`, `obstacle_comms_copy_profile`.
- **Used by:** `obstacle.c`, `scan_task.c`.
- **Edit it when:** changing queue sizes, delivery rules or the links.

#### `obstacle.c / .h`
- **What it does:** the only header other buddies include. It runs the
  start-up order (links → event flag → hardware → task) and forwards every
  other call to `comms.c`.
- **Main functions:** `obstacle_init`, `obstacle_post_cmd`,
  `obstacle_get_event`, `obstacle_get_steer`, `obstacle_get_status`,
  `obstacle_get_faults`, `obstacle_copy_profile`, `obstacle_flag_id`,
  `obstacle_dropped_events`.
- **Used by:** `obstacle_nav.c` (in `main.uf2`) and `bench_main.c`.
- **Edit it when:** the interface agreed with other buddies changes.

#### `obstacle_nav.c / .h`
- **What it does:** the Phase 2 glue. It starts Buddy 5 with links to the
  motion service, and on every line-task pass turns Buddy 5's events into
  motion commands: hold the car, send SCAN_REQ, ACCEPT and LINE_FOUND,
  retry after a fault, and drive the bypass (escape pivot, then TURN / OUT
  / ALONG zig-zag / BACK from wheel-odometry dead reckoning).
- **Main functions:** `obstacle_nav_init`, `obstacle_nav_step`; private
  `drive_reacquire`, `along_intent`, `back_intent`, `update_pose`,
  `keep_plan`, `handle_event`, `nav_stop` (obstacle hold),
  `nav_distance_mm` (odometer).
- **Used by:** `line_task` in `src/main.c`.
- **Edit it when:** the bypass or the handshake with the line follower
  changes (the numbers are `OBSTACLE_NAV_*` in `obstacle_config.h`).

#### `flight_log.c / .h`
- **What it does:** the flight recorder. Keeps the last 1024 records
  (states, causes, scan results with profiles, steering, motion faults,
  commands and a trace of the echo, IR sensors and wheel distances) in RAM
  and prints them when a USB serial monitor is opened.
- **Main functions:** `flight_log_add`, `flight_log_profile`, `flight_log_service`.
- **Used by:** `obstacle_nav.c` only (one task owns the log, so no locking).
- **Edit it when:** you want more or different things recorded (sizes are
  `OBSTACLE_LOG_*` in `obstacle_config.h`).

### 3.4 Bench (real car, no other buddy's code)

#### `bench_main.c`
- **What it does:** a standalone `usermain` that starts Buddy 5 and a
  pretend Navigation. It prints everything over USB, reads the GP20 (next
  item) and GP21 (send) buttons, and AUTO-RUN answers each event the way
  Navigation would. The motors are never touched.
- **Main functions:** `usermain`, `sim_task`, `sim_nav_loop`, `send_item`,
  `auto_on_event`, `print_status`, `print_profile`.
- **Edit it when:** you want different bench output or menu items.

#### The "Buddy 5" block in the team `CMakeLists.txt` (repo root)
- **What it does:** the last block of the repo's `CMakeLists.txt` builds the
  `obstacle` and `obstacle_nav` libraries and `obstacle_bench.uf2` with the
  team's warning flags, and links `obstacle_nav` into `main`. It runs on
  every normal `./configure.sh` + `./build.sh`; no extra flags.
- **Build it:** `./configure.sh` then `./build.sh`; flash
  `build/obstacle_bench.uf2`.
- **Edit it when:** a Buddy 5 `.c` file is added, renamed or removed. It is a
  team file, so tell the team first.

### 3.5 Documents

| File | What it is for |
|---|---|
| `docs/obstacle_file_guide.md` | This guide: which file does what. |
| `docs/obstacle.md` | Status, design, interfaces, bit table, BARR-C naming, abbreviations and deviations, Phase 2 plan. |
| `docs/obstacle_hardware_checklist.md` | Steps H1–H9 to run on the car, with a pass mark and a result column for tribunal evidence. |

## 4. Where do I look if…

| I want to… | Open |
|---|---|
| wire the sensor or the servo | the design doc → Wiring |
| fill in my car measurements | `obstacle_config.h` (search for **FILL**) |
| change the 250 mm detection distance | `obstacle_config.h` → `OBSTACLE_DETECT_MM` |
| fix the servo pointing slightly off centre | `obstacle_config.h` → `OBSTACLE_SERVO_CENTRE_US` |
| change how a side is chosen | `planner.c` → `obstacle_plan_build` |
| change the hug distances (15 / 25 cm) | `obstacle_config.h` → `OBSTACLE_HUG_NEAR_MM`, `OBSTACLE_HUG_FAR_MM` |
| change a state change or time limit | `statechart.c` (limits are in `obstacle_config.h`) |
| see how Navigation should call Buddy 5 | `obstacle.h`, then the design doc → Interfaces |
| change bypass speed or steering strength | `obstacle_config.h` → `OBSTACLE_NAV_SPEED_UM_S`, `OBSTACLE_NAV_INSIDE_PCT` |
| see how the line task uses Buddy 5 | `obstacle_nav.c`, and the design doc → Phase 2 |
| understand a bench log line | `bench_main.c` (`print_*` functions) |
| add a Buddy 5 source file | the file, plus the "Buddy 5" block in the team `CMakeLists.txt` |
| check my change didn't break anything | `./build.sh` (no warnings), then the hardware checklist |

## 5. Reading the names

| Part | Meaning | Example |
|---|---|---|
| file name first | which file a public function lives in | `obstacle_plan_build` is in `planner.c` |
| `g_` | global variable inside one file | `g_links` |
| `p_` | pointer | `p_out` |
| `b_` | true/false, phrased as a question | `b_ready` |
| `h_` | kernel object ID (handle) | `h_flag` |
| `_t` | a type | `obstacle_statechart_t` |
| `_mm`, `_ms`, `_us`, `_deci` | units: millimetres, milliseconds, microseconds, 0.1° | `dist_mm` |
| `_task`, `_isr` | task body, interrupt handler | `scan_task_obstacle_task`, `echo_isr` |
| `_main.c` | file that has `main()` or `usermain()` | `bench_main.c` |

The full abbreviation table is in the design doc → Coding standard.
