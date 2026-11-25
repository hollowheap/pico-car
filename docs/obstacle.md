# Buddy 5: obstacle detection and avoidance

Ultrasonic sensor (HC-SR04+), scan servo (SG90), coarse/fine scan, bypass
planning and the obstacle statechart. The code is in
`src/navigation/obstacle/`. Buddy 5 never writes the motors: it only sends events and
steering words to the line task.

The default firmware runs the mobile pilot on a continuous floor. The Start
popup also offers a stationary obstacle survey for courses with possible
drop-offs. See [Hug (pilot path)](#hug-pilot-path). The Buddy 5 bench and
legacy navigation sections describe the separate statechart configuration.

Other Buddy 5 docs:

- [File guide: which file does what](obstacle_file_guide.md)
- [Hardware test checklist](obstacle_hardware_checklist.md)

## Status

Buddy 5 integration status recorded on 12 November 2026.

| Step | Work | Current position |
|---|---|---|
| 1 | Design (pins, statechart, scan, bypass) | Done. |
| 2 | Code in `src/navigation/obstacle/` | Done: six echo classes, the 2-of-3 hit rule, the retry policy, and the plan, flags and class counts in telemetry. |
| 3 | Host tests | Done: `ctest -R obstacle_` (scan, plan, statechart and face following). |
| 4 | BARR-C:2018 pass | Done; `bash tools/lint.sh src/navigation/obstacle include/navigation` reports 0 findings. |
| 5 | Bench build in the normal team build | Done; `./build.sh` makes `build/obstacle_bench.uf2`. |
| 6 | Car measurements (**FILL** values) | Pending; see [Numbers to fill in](#numbers-to-fill-in). The sonar edge and incidence measurement (Q56) comes first. |
| 7 | Hardware checklist H1 to H9 on the car | Pending; see the [checklist](obstacle_hardware_checklist.md). |
| 8 | Floor test | Pending. Boxes 240 to 340 mm wide are planned too narrow. |

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

With **Stop at obstacles** selected, a fresh centered valid range of 100 mm or less
from the sonar face or a fresh centered TOO_CLOSE return starts an obstacle
hold. The line task publishes `obstacle_seen` with the measured range for a
valid contact. Once stationary, the pilot samples the front of the obstacle,
then uses a second three-angle sweep to examine a visible edge. With no
edge identified, the second sweep samples both sides. Each completed sweep
publishes `scan_done` with clearance unverified and tries to queue the
accumulated readings from that stationary pose for display. Once the sweep budget is spent, the
servo stays centered. See [hold and release behavior](#hug-pilot-path).

**Bench firmware** (the normal team build, no extra flags):

```sh
nix develop
./configure.sh
./build.sh                  # makes main.uf2 as before, plus:
# build/obstacle_bench.uf2  -> hold BOOTSEL, plug in, copy it to the Pico
```

Open the USB serial port to see the bench output. The default `main.uf2`
uses the obstacle sonar server and gives the pilot ownership of route decisions.
The legacy navigation configuration selects `PICO_CAR_PILOT=OFF`.

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
| `scan_task.c/.h` | Commands, time limits, servo settling, and ping scheduling | no |
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
| Obstacle task (`g_ctsk_obstacle` in `src/main.c`) | priority 4, `TP_PRC1` (Core 0), 2048 B stack |
| Echo ISR | 1 slot in the shared `src/platform/isr` dispatcher |
| Kernel objects created | **1 event flag**; no message buffer, cyclic handler or mutex |
| PWM | slice 6 channel A (GP12), 125 MHz / 125 = 1 MHz, TOP 19999 = 50 Hz |

### Ping timing

The pilot schedules pings at least 50 ms apart. The legacy navigation build
uses 60 ms. The same spacing applies to scheduled and requested pings within
each build, and both use the 40 ms echo wait.

1. Empty the echo result with interrupts paused. If ECHO is already high,
   the ping is **STUCK** and no trigger is sent.
2. Send a 12 µs TRIG pulse.
3. The ISR records the rise time. On the fall it writes
   `width | RISE | DONE` in one store and sets the flag bit.
4. The task waits at most 40 ms, then `scan_algo_classify` gives the class:
   VALID, OPEN, TOO_CLOSE, NO_ECHO, GLITCH or STUCK. The table, and what
   each class does, is in `src/navigation/obstacle/scan_algo.c`.

A completed pulse beyond the valid range gives OPEN. A pulse still high at
the 40 ms deadline gives NO_ECHO and the scanner retries at the next ping.
Such a pulse could have risen just before the deadline.

### Statechart

The states, the time limits and the fault causes are in
`src/navigation/obstacle/statechart.c` and `obstacle_config.h`. Rules that
apply in every state:

- **STOP command or any fault** gives STOPPED, plus a FAULT event.
- **A fault with a bit** also calls the emergency-stop link.
- **Detect** is 2 hits among the last 3 pings. The **collision guard** is
  2 close pings (at most 80 mm, or TOO_CLOSE) among the last 3. Both use the
  same hit definition.
- **Guard pings.** A ping is a guard ping when the body turns at the start
  or at the end of its echo wait. Only the health checks and the collision
  guard use it. It does not change the class, distance or seq of the status
  word, or the ping word. So the face follow never reads a guard ping as a
  face ping.
- **Ping word.** Every face ping updates the ping word: class, distance, a
  6-bit seq and the servo angle the ping was taken at
  (`obstacle_comms_ping()`). The face follow takes a ping only when its seq is
  new and its servo angle is the look angle that the follow asked for.

### Scan

- **Grid:** 37 slots, -90 to +90 degrees in 5 degree steps, positive = left.
  Slot 18 is straight ahead.
- **Servo:** the pilot and legacy scan share a physical limit of ±20 degrees.
  The pilot's side looks use the whole servo range. The legacy scan turns the
  body to 40 degrees and then 70 degrees toward a side as needed to reach the
  rest of the grid.
- **Coarse:** every slot (5 degrees) in the centre window.
- **Region:** the closest hit within 600 mm, plus coarse neighbours within
  closest + 250 mm. The closest reading is looked for inside the car's path
  (±30 degrees) first.
- **Fine:** walk outward from each edge, one slot at a time, up to the first
  real miss (OPEN, or VALID beyond the last hit on that edge + 250 mm).
  - A NO_ECHO, GLITCH or STUCK slot proves nothing, so the walk assumes the
    obstacle goes on and steps past it.
  - TOO_CLOSE is a hit at 20 mm and never a miss.
  - An edge with no miss inside the current window continues through the
    shifted windows. An edge still unbounded at the end of the grid makes
    the side unsafe.
- **Per slot:** settle for 3 ms per degree plus 20 ms. Ping until two pings
  agree (same class, within 15 mm for VALID) and keep the closer distance.
  After 4 pings without agreement the slot is stored as GLITCH.
- **Warning flags:** P1 (`echo_noisy`) and P5 (`scan_noisy`) are computed
  while the scan runs. They never stop the car.

### Hug (pilot path)

The track is 300 mm wide. A bypass may use adjacent floor and must rejoin
the original tape beyond the obstacle in the original direction. The full
vehicle outline must clear the box, and the drive wheels and caster must
have continuous support through the maneuver. Normal Start, the GP20 button
and offline power-up use the continuous-floor policy in this build. The
pilot chooses a side, turns along the obstacle, measures travel with the
wheel encoders and scans the face again from the new position.

For a course with possible drop-offs, select **Stop at obstacles** in the
Start popup. The car holds at the obstacle and surveys with the servo. The
installed downward IR sensors still lack floor-edge measurements for the
wheel and caster paths. The selection applies to one run. Stop the current
run before changing it.

A planned stop immediately sets both wheel targets to zero. The scanner waits
for fresh feedback showing both wheels at rest before requesting a ray.
During an active hug, three seconds is the maximum intended interval between
measured wheel movements. Settling, servo requests and retries share that
interval. After 1.8 s at one pose the pilot can finish an optional close-up
from its previously measured edges, or use the chassis to aim the remaining
rays of a longer sweep. A wheel command alone cannot confirm movement.
If the encoders stay fixed through the deadline the pilot reports a hug
timeout. An operator stop, pickup, or sensor fault keeps its own hold.
Thirty seconds for the entire bypass remains a cue to inspect the run.

On the physical log, correlate the accepted command with `obstacle_seen`,
the obstacle frame's hug state and side, and the wheel targets and speeds.
The pilot emits `bypass_start` when it first commands forward travel along
the chosen face. Check the wheel feedback for actual travel and
`bypass_end` for optical rejoin.

With **Stop at obstacles** selected, the pilot holds both wheel targets at zero and records each requested sonar
return in a local frame anchored to both stationary wheel encoders. The first
three bearings are −20°, 0° and +20°, with left positive. The pilot fits a
near face from at least two distinct return points, including one within the
100 mm contact distance. An OPEN ray or a sufficiently distant return just
past the fitted face can locate its edge within the sonar incidence limit.
If both ends are visible, the nearer lateral edge determines the provisional
side, with left winning an exact tie. The second sweep probes 5°, 10° and 15°
on that side. With an unresolved edge it samples +10°, 0° and −10°.
This discovery also works with both line sensors clear.

A reply must match the requested sequence and angle and arrive at the
pilot before the 300 ms request deadline. The sonar mailbox has no capture
timestamp. A timeout skips the sample, including a matching late reply. One completed sweep retains the preceding
sweep’s observations while both encoder totals stay fixed. A change to either
total, loss of stationary feedback, or an independent safety hold discards
the anchor and its provisional side. An interrupted sweep counts toward the
same two-attempt budget. Between sweeps and after the budget is spent, the regular
centered stream can release the hold with three consecutive fresh OPEN returns
or valid ranges greater than 100 mm from the sonar face. A blocked or invalid
forward reading restarts the count. If a regular ping is missed, the next
clear reading starts a new count at one. A normal sequence wrap from 63 to 0
preserves the count. Requested and off-center rays and cached
readings cannot advance it. The car must be stationary, the run must remain
armed, and other motion holds must clear under their own rules. After release,
the next obstruction gets its own sweep budget. An apparent stall or slip
has a separate hold in this stationary mode. The pilot brakes before reversing and refuses
a pivot. Clear the obstruction and send START to retry the original route.
Three fresh centered clear pings with the car stationary are still required.

An encoder feedback fault also brakes both wheels. After 500 ms of braking
the motor controller requires two fresh captures forming a valid measurement
interval on both wheels before it reports ready. It discards commands sent
during the fault and waits for a fresh command. Counts accumulated while
braked do not change the reported odometer.

The dashboard's polar scan plot decodes the 37 range and echo-class slots in
the `profile` packet. A bounded pilot profile fills the measured slots from its
current stationary pose. The provisional side locates a sector worth probing. The unseen rear edge, box depth and supporting floor still require evidence. Legacy profiles can show supported surfaces
and measured endpoints. The course view draws a separate estimate from
the hug's reported face distance, width, and breadth. The host anchors that
estimate to the contact heading and uses dashed edges for missing sizes.
The Edges instrument plots the axle as a dot in that coordinate frame. A
reported width places the near edge, and a reported breadth completes the
rectangular estimate. The hug packet carries no lateral center offset or
face normal, so the estimate is centered across the approach heading. The
connector shows estimated axle-to-edge distance. A survey without a known
starting pose shows the recorded scan and its own axle origin.

A host-computed detour needs pose-linked sonar returns and a surveyed area of supporting floor. The host should fit the obstacle and evaluate candidate routes.
The pilot retains wheel control, clearance and freshness checks, and local
recovery when telemetry is unavailable. Display placeholders cannot supply
clearance evidence.

The host planner supports a surveyed area with one straight, tape-aligned
rectangle. It fits all four faces from calibrated sonar returns linked to
capture-time poses. It checks the full vehicle outline, tracking uncertainty,
obstacle clearance and independent supporting-floor bounds throughout the rounded route. A
front-only scan, missing or incompatible face evidence, or an unsafe sweep
leaves the planner without a route. The survey must establish clear space
around the box independently of sonar OPEN readings.

The experimental executor accepts the host's integer arc and straight
segments from a stationary pre-surveyed start. The planner applies the pilot's
limits of seven segments, 1,500 mm per segment, 3,500 mm overall, and 50°/s at
100 mm/s. Its rounded return can align one sensor with the surveyed tape,
keeping the car parallel to the tape. The experimental pilot entry point
binds the route to the local odometry at that start. It waits for fresh
forward clearance before each segment and checks position and heading while
braking between segments. It brakes if the route is withdrawn, feedback
leaves the checked envelope, or local safety takes control. A stopped route
requires a fresh survey and an explicitly reset executor. At the endpoint,
the pilot stays stopped until it sees tape and new centered sonar clearance
after settling. Direct host tests exercise this executor through the pilot interface. The installed pilot requires a verified route admission path before
it can use these segments, and the present planner needs an approach anchor
with enough room for its lead-in and the executor's forward clearance checks.

The four-face survey happens before the route benchmark. An initial
encounter still requires its own acquisition and timing evidence.

At a 100 mm centered sonar return, a flat face is about 170 mm ahead of the
axle in the planning model. A centered 60 mm box leaves 120 mm on either
side of a hypothetical 300 mm-wide area of supporting floor. The 126 mm-wide car exceeds either passage
before adding the 50 mm obstacle clearance. In the surveyed planner tests,
the same close anchor also holds in a wider area. The rounded route needs
more room ahead of the face before it can turn out. A separate synthetic
300 mm-wide patch of surveyed floor does admit a route when the tape leaves a 210 mm passing side,
the box is on the other side, and the surveyed face is 850 mm ahead of the
start axle. That fit uses a 5 mm tracking bound and still needs vehicle
evidence before use on the installed course.

The earlier Buddy 5 reference (`9654e50`) backs up 58 mm from a 100 mm sonar
return, pivots 90° on a stopped wheel, then moves at least 147 mm sideways.
With the 107 mm track and the current 130 mm forward footprint, the pivot and
sideways leg move the outer edge to 330.5 mm from the original tape. Including
the 63 mm on the other side of the initial footprint, the maneuver needs at
least 393.5 mm of strip width before adding clearance. That reference
assumes up to 500 mm of lateral floor and a ±60° servo. The installed servo
reaches ±20°. [Surveyed planning tests](../host/bridge/src/detour.test.ts)

Non-scan pivots with equal and opposite wheel targets can request up to 500
permille with valid encoder progress. Scan-aiming pivots and straight driving
keep the 450 permille ceiling. The 400 permille base still applies when the
wheel stalls or its feedback is invalid. Confirm the trial limit on the car.

#### Continuous-floor hug

Normal Start selects these maneuvers.
If the IMU loses its reference during a run, the wheels wait for upright
attitude, fresh stationary wheel feedback and a new centered sonar return.
A nearby return or an interrupted hug starts a stationary survey from the
current position. If two surveys miss a required ray, a new usable forward
ping after the 320 ms retry interval allows another survey. The wheels stay
at zero while the scanner reacquires both side bearings and the center.

The plant uses a 119 mm radius around the axle for collision checks. The
installed front sensors extend 130 mm ahead of the axle and the caster
contacts the floor 120 mm behind it. Qualifying a physical route requires
the installed outline, wheel paths and braking distance.

- **Obstacle priority on a tape bend.** Two of the last three forward returns
  at or below 250 mm normally start a survey. On a sustained left or right
  bend the car can keep tracing if the IR, wheel targets and measured wheel
  speeds agree on the turn. Each pair of wheel speeds gives an axle-midpoint
  radius of `107 mm × (left + right) / (2 × |right − left|)`. The pilot uses
  the wider of the requested and measured arcs. The collision check extends
  63 mm on each side of the axle midpoint using the measured 126 mm outside
  tire width. It includes the front sensor and rear caster reach, a ±13°
  sonar beam, 15 mm of range uncertainty and a 25 mm clearance margin.
  For example, at inner and outer wheel speeds of 86 and 100 mm/s, a return
  225 mm directly ahead of the sonar lies 58.8 mm outside the axle midpoint's
  arc. The outer tire would cross that point if the turn continued.
  Returns at or below 180 mm from the sonar go through the ordinary trigger.
  For a return outside the turning path, the car allows up to 60 mm of outer
  wheel travel or 800 ms from the first hit, whichever comes first. Two fresh
  clear readings end the check. A persistent return, an echo in the turning
  path or loss of evidence for the turn restores the ordinary trigger.
  A single forward beam can miss an obstacle at the side of the car.
  These limits await a physical bend run.
- **Looks.** The car stops and looks with the servo and with body pivots. The
  first look of a hit is coarse: 19 rays, 5 degrees apart within 35 degrees of
  the heading of the hit and farther apart beyond it (`HUG_CONTACT_RAYS`).
  The rays beyond 35 degrees only measure the room beside the face. The pivot
  before a look goes straight to the first body step. When the last fit
  showed the measured face extending ahead of the new pose despite the
  allowed aiming error, the pilot checks three bearings from one stationary
  heading, starting on the side it is travelling toward. Matching ranges
  confirm the existing face for another short leg. A fresh centred echo must
  clear the forward guard after the car turns back to the travel heading. For a
  measured, aligned side leg, the guard accounts for the remaining drive
  distance, the 145 mm turning radius, a 120 mm forward reserve and a settling
  allowance. A fresh TOO_CLOSE reply always interrupts the leg.
  If the prior fit lacks coverage or the check cannot confirm the face, a
  fresh full side look covers all 15 bearings from two stationary headings,
  15 degrees either side of the expected face normal. The full sweep ends
  toward the next leg.
- **Measured corners.** After the axle passes a measured endpoint by the
  planned gap, the pilot checks three bearings along its next heading. A
  close return triggers a wider survey. A missing, late or unusable reply
  starts one fresh three-bearing check from the same pose. A second
  inconclusive check holds both wheels. With clearance the car drives to a
  side-looking position alongside the saved endpoint. At least four fresh
  returns must fit a surface that meets that endpoint within 45 mm. An
  independent wheel takeover during this side survey discards its readings
  and holds both wheels when control returns. Where
  the chosen side has no measured far edge yet, the car advances in short
  steps that keep successive looks overlapping.
- **Oblique corners.** An observed side surface may miss the saved endpoint.
  The pilot can then check the return corridor and turn the car to drive
  forward to its midpoint, where the sonar has a better view of a possible
  connecting face. A measured oblique face that passes through the endpoint
  permits another step. The car measures each following endpoint and seeks
  a connecting surface. A confirmed rear plane can then support the return
  to the original lateral coordinate. If the rear cannot be qualified, the
  wheels stop. A mirrored eight-face host case exercises both a visible and
  an acoustically invisible rear. Measured paths and limits
- **Face fit and edge test.** Echoes on a face are fitted to a line. A hit is
  trusted up to 25 degrees off the face normal. The edge test applies the
  9.5 degree beam correction described under the planner.
- **Narrow obstacles on continuous floor.** After an inconclusive front survey,
  the chassis returns to the approach heading and waits for a new centered
  reading. A recurring close range that matches the survey can start a 60 mm
  sideways inspection leg. The pilot attempts all 19 bearings. The center and
  the chosen side must have usable readings and room for the turning body
  and the leg. A missing return on the opposite side rules out that side. A close obstacle
  inside the turning clearance first triggers the existing measured back-off.
  After the pivot the wheels must settle and sonar must confirm clearance
  straight down the leg. If the chassis coasted outside the 2° tolerance,
  the pilot corrects the heading once and checks a new reading. It measures
  the leg with both encoders and looks back toward the contact from the new
  position. A usable face continues the hug. If the first look misses the
  face, the pilot looks once more from that station. Two inconclusive looks
  end the probe with a hug timeout. Width and depth stay unknown until measured.
- **Repeated contact after a clear handoff.** If a new contact matches the
  recent inconclusive survey, the pilot can recheck nine bearings across
  ±20° with the chassis stationary. Both wheel encoders must stay within 10 mm of their original
  readings and their differential within the 2° aiming tolerance. The
  handback may be at most 800 ms old and the full survey at most 15 s old.
  A new return inside the 300 mm forward guard, a widened close face or
  an incomplete check starts a fresh full survey. Safety ownership discards
  a partial contact survey. A successful recheck waits for a fresh,
  centered OPEN or VALID over 300 mm after settling before handing the
  wheels back. These limits await car measurement.
- **Back-off.** A measured close face can require room to turn. The hug
  aligns with the contact heading and reverses by the difference between
  the sonar range and its 250 mm standoff. It looks again before choosing a
  side. A wheel stall during an active obstacle maneuver also permits a
  bounded 40 mm recovery reverse. The normal line follower uses the
  stationary survey below when wheel progress fails.
- **Side.** The pilot chooses the nearer visible edge and checks the space
  beside it. With sparse returns, it fits the nearby face separately from a
  deeper corner when at least four close returns agree. If the face is narrow
  and only one edge is visible, the car can hold the body still and sweep the
  servo in 2° steps from −20° to +20°. A complete sweep can distinguish the
  nearer side from the distribution of face returns. Side-wall clearance from
  the wide scan retains priority. Equal edge estimates on a broad face
  (within 30 mm) favor the left. With no edge the side with more open room in the wide rays wins by 500
  mm of reading, else left. A side with less than 340 mm between the edge and
  a wall beside it is passed over. A box 300 mm wide and about 35 mm off the
  line shows neither edge from the hit, so it can go the farther way round.
- **Next observation.** On an unbounded measured face the pilot adapts Buddy 5’s
  side-scan rule. Each valid echo is projected onto and along the measured
  face from its sonar pose. The furthest on-face return toward travel guides
  the next encoder-measured leg. The extra advance is limited by both the half
  lane and the observed coverage behind the axle. When searching for the third
  vertex the pilot omits that extra advance. Minimum drive increments still
  apply. A measured endpoint uses the corner routine. [Buddy 5 side-scan planner](https://github.com/hollowheap/pico-car/blob/9654e50b5c202c00ef23f34fa75e68a773eab37b/src/obstacle/b5_plan.c#L466)
- **Gap around a small obstacle.** If two observed front endpoints span at
  most 63 mm and the front is within 15° of square to the approach, the
  pilot can pass the nearer corner on a straight 25° diagonal. The axle
  path runs 103 mm perpendicular to the measured corner. Subtracting the
  63 mm body half-width gives a nominal 40 mm gap. For a centered 40–50 mm
  round post whose measured edges match its extents, the calculated closest
  body clearance is about 47–49 mm. These are geometric targets awaiting a
  physical run.
- The car pivots upstream, checks three bearings along the diagonal, and
  holds that heading past the obstacle. It brakes once odometry puts the
  axle at least 170 mm beyond the measured side plane. That station allows
  the 145 mm chassis sweep plus a 25 mm margin for the next sonar look.
  A blocked diagonal triggers a new look before departure. An obstruction
  during the close pass stops the wheels.
- **Other faces.** Wider fronts, incomplete edges and stall recovery use
  the ordinary wrap. Its 195 mm axle target includes the full turning
  radius and a 50 mm margin. Subsequent observations guide each leg and
  allow up to 30° of steering correction.
- **Stall during a hug.** When an obstacle maneuver stalls, the pilot can
  back off up to 40 mm, with a two-second limit. The hug then rechecks from
  the backed-off pose. Its repeat-stall memory can retain the location for
  15 s and limit the next approach to 70 mm/s. This recovery requires an
  active obstacle encounter.
- **Possible slip on approach.** When both wheels advance at least 100 mm
  while following straight, a centered range between 250 and 600 mm that
  stays within 15 mm of the first reading can indicate slip. Encoder heading
  must agree within 2°. The pilot stops and surveys from rest. A near
  reading in a completed survey starts the hug.
- **Legs.** It drives legs round the faces of the obstacle, looking at each
  corner. Every corner is turned, as many as there are: 90, 180 and 270
  degrees round, or a full turn and more. A look that loses the face first
  drives on a body radius and the gap, then turns, since the corner may be
  just behind the car.
- **Rejoin.** Outside the approach zone (150 mm along the hit heading and
  250 mm to either side), the unrestricted pilot records where a line sensor
  crosses tape. This also covers tape seen while the body turns during a
  scan. Parallel tape can run 300 mm to either side of the entry line.
  Within 80 mm of those positions, a lone hit is ambiguous and the hug
  continues. Both readers may identify a transverse exit there if their
  lateral separation exceeds the 15 mm tape width. This check uses the
  projected IR position. The car may use adjacent clear floor for the bypass.
  The 80 mm tolerance is an initial allowance for odometry error.
  On the next drive it projects the observed point onto the heading
  and advances up to the 130 mm sensor lead. A stall or a fresh obstacle
  inside the forward guard interrupts that drive.
  The chassis then sweeps away from the obstacle through up to 135 degrees
  at its low pivot rate. The leading detector must enter and leave a stripe
  after both line detectors read clear in that sweep. Wheel travel must be consistent
  with crossing a stripe between half and twice the configured tape width.
  After braking, the pilot checks that the axle and heading stayed near the
  crossing before handing control to the follower. A safety interruption
  requires a fresh crossing. Exhausting the sweep without one stops the
  wheels with `HUG_TIMEOUT`.
- **Walls.** A wall is followed for ever. The hug stops only for a sensor fault,
  STOP or a move timeout.
- **Move timeout.** Each hug pivot or drive must end within three times its
  planned time, and never sooner than the 1 s stuck window. A move that takes
  longer ends the hug and stops the car with reason HUG_TIMEOUT, which latches
  until an accepted START or RESET.

#### Returning to the line behind a measured box

On continuous supporting floor, the pilot follows the selected side of a box
until sonar observations locate its far vertex. It checks that the next
measured face is the rear and that the vehicle can traverse it with room to
pivot. The clearance calculation includes the 130 mm forward sensor reach,
120 mm rear caster reach and 126 mm vehicle width. With no incoming tape
observation, a stripe encountered beside the box cannot trigger a handoff
before the vehicle envelope has cleared the measured side endpoint.

Wheel odometry places the axle back at its original lateral coordinate behind
the box. In the pilot's coordinate frame the contact heading is forward and
the entry lateral coordinate is zero. The pilot restores that heading and
checks both wheels are stationary, the estimated lateral error is within
half a stripe and centered sonar is fresh and clear. Before the rear traverse
and again before the tape search, a missed echo leaves both wheels at zero
while the 800 ms reply window runs. A fresh clear ping inside that window
permits the next movement. A measured obstruction or an expired window ends
the maneuver. The local tape search pivots up to 30° each way. An observed stripe can return
control to line following. The dashboard records one bypass completion on
that optical transfer.

An empty local sweep returns to the entry heading. If the pilot saw incoming
tape shortly before contact, it may follow the remaining measured edges once
to find an exit that bends under the box. The car must still be stationary
at the return pose with fresh forward clearance. The normal contact back-off
preserves the incoming tape observation. A safety interruption, E-stop or
new START invalidates the offer to continue. Finding the entry face again
without an exit ends the attempt. With no incoming tape, the first empty
sweep ends it. Both outcomes hold the motor targets at zero.

Qualify the selected side, nearest gap and optical handoff using the car’s
wheel, sonar and tape readings alongside an observation of the physical course.
The bounded-floor option holds at the obstacle on courses with possible drop-offs.

#### Shared pilot and line-task behavior

- **Failed forward progress on continuous floor.** After one second, each
  wheel commanded above its deadband must have advanced at least 2 mm.
  A tight turn may intentionally stop the inner wheel. Displacement across
  an invalid encoder interval is discarded. At startup or later in the
  run, a failed progress check brakes both wheels. Once stationary, the
  car scans left, center and right. A completed scan with a near return
  starts the obstacle survey. A clear scan and three fresh centered clear
  readings permit a forward retry. Two incomplete attempts hold the wheels
  for a new START. A commanded wheel that stays silent for a second window
  while its peer moves can latch the encoder fault without a diagnostic reverse.
- **Safety stop.** The pilot sets both wheel targets to zero immediately and
  bypasses the normal acceleration limit. Confirm rest from fresh, valid wheel
  feedback and measure the braking distance on the car.
- **Pings.** Each 10 ms tick the line task reads the latest ping word with
  `obstacle_ping_read()` and gives the pilot its distance, servo angle, echo
  class, sequence and reply flag. NO_ECHO, GLITCH and STUCK give no reading
  (`LOOK_ECHO_NONE`); they are never OPEN. A STUCK, NO_ECHO or ECHO_NOISE
  fault of the sonar task stops the pilot with reason SONAR. Once a forward
  ping has been seen, 250 ms without a new forward sequence also stops it
  with SONAR. The forward timer waits while a hug request uses the servo.
  A different failing echo class keeps the fault active. A usable ping clears
  it, and an echo-induced stop returns to MONITOR. If the stop lasts no longer
  than PILOT_SONAR_RESUME_MS (1 s, a guess to be measured on the car), the pilot
  drives on by itself. A longer stop latches: the reason then reads
  SONAR_LATCHED (wire 9) for sonar or STALE_LATCHED (wire 10) for wheel
  feedback. Three consecutive usable forward pings with fresh wheel
  feedback and released motion holds clear the latch. START or RESET
  also clears it. RESET leaves the wheels held, so START must follow. The
  same rule applies to stale feedback, which means the wheel feedback is older
  than PILOT_STALE_MS (100 ms) and says nothing about the sonar. While a hold
  is on, a STALE_LATCHED shows as HELD; a SONAR_LATCHED does not. An operator
  STOP holds the car until START; RESET does not release it.
  A sonar that leaves two requests of the hug in a row without a reply
  (`HUG_LOST_REPLIES`, 800 ms each) also stops the pilot with reason SONAR. The
  hug keeps asking and goes on at the next reply; that stop is not a latch.
- **Ping requests.** When the pilot asks for a ping, the line task calls
  `obstacle_ping_request()` once for each new (sequence, angle) pair, not
  every tick. A refused request is logged once with `LOG_WARN` and tried again.
- **Corner speed.** The straight request is 163.364 mm/s. On a sustained
  single-reader hit the heading target slews by one radian per 107 mm of
  requested outer-wheel travel, the curvature of Buddy 5's one-wheel turn.
  The outer target drops to 100 mm/s at 2° of heading error. The inner target
  falls progressively to zero if the heading continues to lag. From 100 mm/s
  the 80 mm/s² recovery takes about 0.79 s of clear straight running to reach
  cruise. Matching front readings during a barcode hold the current outer
  target and anchor the next correction. These are command targets. Wheel
  speed and tape retention through successive 270° bends need a physical run.
- **Line events.** The line task raises `line_lost`, `line_found` and the
  line-lost fault only while the follower has the wheels. While the hug drives,
  the line is off the sensors and that is not a lost line; a lost state that
  is open when the hug takes over ends without a `line_found`. For 250 ms after
  a qualified barcode edge the follower can drive straight under the sonar
  speed cap when both front detectors agree. A single detector on tape steers
  through the bend. The follower records tape crossings during either action.
  An ambiguous edge or ring overflow
  restores line tracking immediately. A new edge qualifies after 250 ms of
  quiet. On a gap, the follower relaxes its prior correction and holds the
  settled heading over the first part of blind travel. It then returns to
  relaxed steering. If both sensors still read clear after 100 mm of flat
  travel, the car steers about 4.4° while driving forward to bring one
  sensor across the blind strip between the detectors. A tape hit resumes
  ordinary steering. At an unmarked square corner the front detectors can
  cross the outgoing strip while the axle is still approaching it. If the
  tape stays clear, forward travel to that remembered crossing triggers a pivot.
  Gaps without this evidence retain the 150 mm search limit. The heading hold,
  search position and probe need checking on the car.
- **Commanded maneuvers.** The pilot checks sonar and wheel feedback before
  each wheel command during a pivot or straight junction crossing. A close
  centered return ends the maneuver at the same standoff used by the hug.
  The line task brakes, reports the abort and lets the pilot check course
  clearance. A sonar fault or stale feedback can also abort the maneuver.
- **Slope compensation.** On a straight forward line-following pass, the
  pilot compares both fresh wheel speeds with the calibrated roll. When
  the uphill wheel is slower, it adds demand to that wheel and subtracts the
  same amount from the other. The trim has a 2° roll deadband and a 12 mm/s
  limit per wheel. The roll sign and trim values still need a car run.
- **Lost line.** With no sensor hit the follower bridges up to
  `LINE_GAP_MM` (150 mm). On a hump it skips the flat-ground probe and
  allows `LINE_HUMP_GAP_MM` (400 mm) for the sensors to clear the crest. Then it turns
  on the spot toward the side that saw the line last, and back the other way.
  A failed search is retried up to `LINE_RETRY_COUNT` (3) times: the car turns
  back to the heading it had when the search began, reverses `LINE_REVERSE_MM`
  (150 mm) straight and searches again. A reverse in which the slower wheel
  backs up less than `LINE_REVERSE_STEP_MM` (10 mm) in `LINE_REVERSE_STALL_MS`
  (1.5 s) is blocked (a jam of one wheel counts): it ends as a failed retry,
  and the car searches again. The pilot's stuck check does not back off after
  a reverse (the blockage is behind the car) and does not tell the hug of a
  contact ahead. A sensor hit at any time ends the recovery. After the last
  retry fails the car stops until a sensor sees the line. Each retry sends a
  `line_lost` event with p2 = `0x8000 | retry`. The retry count clears once a
  sensor hit comes past the point where the line was lost. The constants are
  guesses, not measured on the car.
- **Hug fields of the frame.** The line task publishes hug phase and geometry
  under one spinlock (`telemetry/hug_word.h`). The telemetry task reads the
  hug fields together. Idle shows side none. With the pilot on, `body_deci`
  is the body heading from the heading at the hit, in
  deci-degrees, left positive; it is packed into word 1 with an offset of 1800.
  `face_mm` is the last measured axle-to-face distance while the hug is
  active, not the drive distance the hug plans from it. An idle hug shows 0
  for both `face_mm` and `body_deci`.

The telemetry obstacle frame is schema 3, 24 bytes, big-endian: state, cause,
faults (u16), echo class, flags, dist_mm (u16), closest_mm (u16), seq,
hug_state (idle, contact, backoff, align, leg, look, turn, inner, exit),
hug_side (left, right, none), hug_seq (the low two bits of the hug publish count; a changed value is a new hug sample), width_mm,
breadth_mm, face_mm (all u16, 0 until seen), servo_deci and body_deci (i16).
Faults bits 12 to 14 are not set on the wire. The `bypass_start` event gives
the side in p1; `bypass_end` gives the width in p1 and the breadth in p2.

Both firmware modes read `echo_class`, `dist_mm`, `servo_deci` and `seq`
from one published sonar sample. The angle is the servo bearing relative to
the chassis during that ping, reported in deci-degrees at whole-degree
resolution. A requested ping carries its request sequence. Before the first
sample the frame reports NO_ECHO and zero distance, angle and sequence. Hug
phase, geometry and body heading come from the separate hug publication when
available.

### Plan and bypass

This is the old path, used when `PICO_CAR_PILOT=OFF`. The telemetry frame no longer carries its plan fields.

**Beam-cone correction.** With h = last hit angle, m = first miss angle and
β = half-cone (9.5 degrees), the planner places an edge between h - β and
m - β. This assumes the sonar returns an echo when only the edge of its beam
is on the obstacle. That assumption is not measured (Q56), and the
simulation does not follow it.

**Choosing a side.** The legacy planner accepts a sideways offset (edge
+ `OBSTACLE_BOX_GAP_MM`) within its assumed 500 mm lateral allowance when
no other reading on that side falls in the swept band. The smaller offset
wins, with ties going left. An
edge that is not bounded, or lies beyond `OBSTACLE_EDGE_TRUST_DECI` (30
degrees), plans a wall pass on the left. With no usable side the plan is
NO_SAFE_PATH and the car holds. The navigation answers a NO_SAFE_PATH or
FAILED plan with REJECT. Buddy 5 then stops with cause NO_SAFE_PATH and its
fault bit, and waits for RESET and START. It does not rescan the same
obstacle.

**Out pivot.** One pivot away from the obstacle at the smallest of 45,
60, 75 or 90 degrees that clears the front corner by half the car width plus
`OBSTACLE_PLAN_MARGIN_MM`. The car never reverses.

**Along the obstacle.** Every `OBSTACLE_SIDE_LOOK_MM` (250 mm) the car stops,
pivots 75 degrees toward the obstacle, aims the servo at 15 degrees so the
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

The commands, events, snapshots and their limits are in
`include/navigation/obstacle_cmd.h`, `include/navigation/obstacle_flags.h` and
`src/navigation/obstacle/obstacle_config.h`. The calls are in
`include/navigation/obstacle.h`.

```c
obstacle_links_t links = { h_nav_mbf, p_stop, p_get_distance_mm };  /* 0/NULL allowed */
obstacle_init(&links);              /* after gpio_irq_init() */
obstacle_post_cmd(obstacle_pack_cmd(OBSTACLE_CMD_SCAN_REQ, req_id));
obstacle_snapshot(&status);         /* state, cause, faults, stop reason, echo class, flags */
obstacle_copy_profile(profile);     /* 37 slots: E_BUSY, never waits */
obstacle_copy_plan(&plan);          /* plan of the last finished scan, with its req_id */
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
| | seq (liveness) | 0-31 wrap, face pings only | 5 | bits 27-31 |
| Fault word `u16` | 9 fault flags | - | 9 | single writer (obstacle task) |
| | bypass give-up reason | 0-7 | 3 | bits 12-14; the snapshot gives it as `stop_reason`; not sent on the wire |
| Command `u8` | type, id | 8 types, 0-31 | 3 + 5 | one byte |
| Event `u32` | type, id | 4 types, 0-31 | 2 + 5 | bits 0-6 |
| | OBSTACLE: dist | 0-4095 | 12 | bits 7-18 |
| | SCAN_RESULT: status, side, width, angle, wall | 4, 2, 0-1023 mm, 0-127 deg, flag | 2 + 1 + 10 + 7 + 1 | bits 7-27 |
| | FAULT: cause, faults | 15, 9 bits | 4 + 9 | bits 7-19 |
| | STEER: action, seq | 4, 0-63 | 2 + 6 | bits 7-14 |
| Ping `u32` (one store) | seq, class, dist_mm, servo | 0-63, 6, 0-4095, -20..+20 deg | 6 + 3 + 12 + 6 | bits 0-26; face pings only; servo stored as whole degrees plus 30 |
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
| Lateral planning limit `OBSTACLE_FREE_FLOOR_MM` | The legacy planner caps the offset at 500 mm. Actual supporting floor beside the track must be established from onboard evidence. |
| Servo direction `OBSTACLE_SERVO_DIR` | Configured for the car (−1) |
| Bypass speed `OBSTACLE_BYPASS_SPEED_MM_S` | 40 mm/s, matches `main.c` (40.8 mm/s) |
| Beam half-angle `OBSTACLE_BEAM_HALF_DECI` | **Done:** 9.5° (95). Power bank at 300 mm dropped out with its near edge 50 mm left / 65 mm right of centre (9.5° / 12.2°); the smaller value gives more room. The left/right difference comes from the two transducers sitting either side of the centre. |
| Wheel diameter | **Done:** 65 mm (already in the motion service scale) |
| Wheel track `CHASSIS_WHEEL_TRACK_MM` | **Done:** 107 mm (measured) |
| Servo→tail `OBSTACLE_SERVO_TO_REAR_MM` | Pending (170 mm); sets when the car turns back |
| Servo→front | Pending (optional) |
| Servo centre / scale | 1494 µs nominal center compensates the observed −1° boot aim using the configured 5.6 µs/° scale. Check straight-ahead aim on cold boot and clearance at both ±20° limits on the car. |
| Power source | Pending (settle time on a low LiPo, H2d) |

## Phase 2 integration in the legacy firmware

With `PICO_CAR_PILOT=OFF`, the line task uses the Buddy 5 statechart through
`src/navigation/obstacle/obstacle_nav.c`.

**Start-up** (in the startup task, processor 1): `gpio_irq_init()` →
`motion_service_init()` → `physics_init()` → `ir_capture_init()` →
**`obstacle_line_init()`**. That calls `obstacle_init()` with these links and
leaves the car parked. The scan task keeps sampling sonar while parked. The
supervisor posts RESET and START to the obstacle statechart when it accepts
operator START. `src/main.c` creates every task.

| Link | Wired to | Why |
|---|---|---|
| `p_stop` | `motion_service_set_hold(MOTION_HOLD_OBSTACLE, true)` | Stops the car without the reboot latch of `motion_service_request_stop()`, so RESET + START can recover. |
| `p_get_distance_mm` | Mean wheel travel from `motion_feedback_snapshot()`, as forward and backward mm since boot (`obstacle_travel_t`) | The bypass limit counts all travel. The P2 range check uses the net forward travel. The link returns E_OBJ when the feedback is stale. Buddy 5 then keeps the last good travel and skips the P2 check. It never estimates travel from time. |
| `h_nav_mbf` | 0 | Events go to Buddy 5's own outbox; the line task polls it. |

If `obstacle_line_init()` fails, it prints the error and line following runs
without Buddy 5.

**Every loop pass** (every ≤ 10 ms) the line task calls
`obstacle_line_step(left, right)` before its own steering. It returns true when
Buddy 5 is in charge, and the line follower then sends nothing.

| Buddy 5 state | What the glue does | Line follower |
|---|---|---|
| MONITOR | releases the obstacle hold, unless the latest ping is a hit: VALID within 250 mm, or TOO_CLOSE (so a retry never creeps into the obstacle) | drives as before |
| WAIT_STOP, SCANNING, PLAN | holds the car; 300 ms after OBSTACLE sends SCAN_REQ(id); on SCAN_RESULT OK/CLEAR sends ACCEPT(id); on NO_SAFE_PATH/FAILED sends REJECT(id) | paused |
| BYPASS | releases the hold; pivots away (STEER word) until Buddy 5 sees the way ahead is clear | paused |
| REACQUIRE | TURN (pivot to the start angle) and OUT (straight to the side offset) from the wheel odometry, then HUG: a proportional controller holds a 300 mm look-ray target by slowing the wheel next to the obstacle by up to 80% of the outer wheel. One ping under 250 mm forces a turn away, clamped at 60° from the line. OUT drives straight. IR gate: armed when the offset is reached, opens once both IR sensors have been off the line; the next line contact sends LINE_FOUND(id) (at most every 100 ms) | paused |
| STOPPED (boot or fault) | holds the car until the supervisor accepts operator START | paused |

**Speeds** (`motion_service_set_speed()`, lease 100 ms). The car stands
still while the servo turns (entering BYPASS, REACQUIRE or MONITOR): a
servo swing while a motor drives can disturb the wheel encoders.

| Phase | Straight | Turning (inside / outside wheel) |
|---|---|---|
| Escape spin, TURN | — | −40 000 / +40 000 µm/s |
| OUT, ALONG, BACK | 40 000 / 40 000 µm/s | 40 000 / 60 000 µm/s |

NOTE: The inside wheel never runs between 0 and 40 mm/s. Each wheel's speed
controller has a 40% base duty limit. Fresh feedback showing a wheel moving
slower than requested in the commanded direction raises that wheel's limit
in proportion to the speed shortfall, up to a 45% (450 permille) trial
ceiling. A stationary wheel or an invalid encoder sample stays within the
base limit. The trial ceiling has host coverage and needs vehicle measurement
under load. A driven wheel that
makes no encoder edges no longer faults the motion service. The pilot backs
off and tries again if both wheels stay still for 1 s. A wheel that stays
still while the other moves stops the car with reason ENCODER. Once the
operator checks the wheels and clears the path, one accepted START clears
recoverable pilot latches, including ENCODER and HUG_TIMEOUT. Emergency
stop stays latched until reboot. The hug's own lost-replies SONAR stop
ends at the next reply. A CLI RESET acknowledges faults and leaves the
wheels held. After a pickup or overturn, set the car down level and still
before START. The supervisor holds the wheels during level-ground calibration.
If the motors cannot reach 60 mm/s, the outside wheel runs at its top
speed, which still curves the car.

**Dead reckoning.** Heading = `(right − left wheel distance) /
CHASSIS_WHEEL_TRACK_MM` (from the totals, so it does not drift); position is
summed along that heading every pass. `CHASSIS_WHEEL_TRACK_MM` is 107 mm
(measured). The wheel diameter
(65 mm) is already in the motion service's scale (204.2 mm per turn).

**Simulation** (host, the real Buddy 5 code in a closed loop with a
kinematic car), from detection to rejoining the line: obstacles 60/105/160 mm wide,
real beam 9.5°/15°/25°, true track 90/100/115 mm, obstacle centred or
35 mm off-centre: 81 of 81 runs pass without contact (closest gap
57–150 mm), rejoin the line at about 30° and resume line following. A
bypass took about 20–30 s in this legacy fixture.

The glue logs `[INFO](line:nav) state N cause N`, `plan: ...`, `phase N: ...`,
`steer N: ...`, `escape turned N deg`, motion changes and every `SCAN_RESULT`
and writes all of it to the
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

The hardware checklist distinguishes the supported-chassis bench, installed
obstacle hold and future bypass trial. [Hardware checklist](obstacle_hardware_checklist.md)

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
  does not restart by itself.
- GP21 is the motion stop button in `main.uf2` and the "send" button in
  `obstacle_bench.uf2`.
- `gpio_irq_init()` enables the IRQ only on the core that calls it.
- The kernel's `BOARD_LED_PIN` is GP16, which is Buddy 2's encoder pin.
- At ~41 mm/s the hug may need more than 5 s to reach the line.
