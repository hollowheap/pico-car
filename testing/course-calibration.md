# Bench route and course trace calibration

Use independent measurements of the axle midpoint and chassis heading. Tape gives a repeatable route. It does not show where the car actually drove through a bend or after a slip.

## Fixtures

| Fixture | What it measures |
| --- | --- |
| Stationary mount and flat sonar target | Center, backlash, rear clearance, echo timing and range at known distances. Use the [servo procedure](#qualify-sonar-center-and-travel). |
| Stationary timing fixture | Sensor capture and task latency under load, including GP21-to-output cutoff. Physical braking distance requires a moving run. |
| Straight line with start and finish 1 m apart | Left and right wheel scale, signed wheel travel, odometer change, lateral drift and rest after braking. Use `car_bench` ODO for a straight push with the motors off, then validate on driven forward and reverse trials. |
| Taped right-angle cross | Independent pivot yaw and repeatability. The current PIVOT bench runs one direction. Measure the opposite direction in a separately recorded, externally observed turn. |
| Mirrored L routes or a small measured loop | Turning accuracy and accumulated course trace error. Include it as a held-out run after the straight and pivot fit. |

The straight and cross can share the same tape setup on level continuous floor. Start with that layout. Add a known obstacle and tape rejoin after the unobstructed trace has a baseline.

## Collect the reference

1. Mark the axle midpoint and chassis centerline on the car. Measure the start, straight checkpoints and turn witness marks in millimetres. Use overhead footage with fixed scale marks if checkpoints occur during motion. Use the observed axle position and heading even when they differ from the tape.
2. Save the bench or bridge log with build identity, surface, speed, direction and trial ID. Record how each position was measured and its uncertainty. For the first bridge replay, use stationary checkpoints with a visible cue so they can be matched to a specific 5 Hz status sample. Keep one short trial in the JSONL capture and preserve receipt order.
3. Compare the measured straight distance with each signed ODO wheel distance. The scale candidate for each wheel is measured travel divided by the magnitude of its indicated travel. Compare repeated forward and reverse trials before changing either wheel's scale. Use measured yaw versus encoder yaw to diagnose pivot error after the straight fit. Keep a slip correction distinct from physical axle track width.
4. Score the original trace and any calibration candidate on separate trials over mirrored turns or the measured loop. Compare error at checkpoints, heading error, end position and any data gaps. R06 provides existing vehicle targets of 50 mm distance error over 1 m and 10° turn error. Report actual trace error and measurement uncertainty independently.

## Replay a captured trial

Summarize the USB log to identify ODO endpoints and PIVOT results:

```sh
python3 testing/bench_summary.py bench.log
```

Copy the trial's bridge JSONL receipts. Create `checkpoints.json` with the start as the first entry. Coordinates are in millimetres with +x to the right, +y forward at 0° and headings increasing clockwise. Use the observed pose and the corresponding status `seq` for each checkpoint. Heading is required for the start and optional thereafter.

```json
[
  {"label":"Start","seq":120,"xMm":0,"yMm":0,"headingDeg":0},
  {"label":"After the straight","seq":154,"xMm":12,"yMm":1003,"headingDeg":3}
]
```

```sh
(cd host/dashboard && node --import tsx tools/score-course.ts ../../trial.jsonl ../../checkpoints.json)
```

The scorer uses the dashboard's course reconstruction before visual smoothing. It aligns to the measured start pose once, reports per-checkpoint position and heading errors, and exits with a failure status when a checkpoint is missing, ambiguous or beyond an unsupported interval. An endpoint alone cannot validate an entire trajectory. The current car still needs a surveyed run to establish its baseline.

## Bench timing capture

Select TIME in `car_bench` to record a stationary trial. Keep the serial capture open and save the JSONL lines from `header` through `end` in a separate file. The preceding countdown and following status messages are console logs. The bench captures elapsed device microseconds in RAM across up to 64 passes and starts USB export after capture ends. Repeat with the serial reader keeping up and with a slow reader to check that USB drain does not change pass timing.

```jsonl
{"kind":"header","version":1,"build":"abc1234+dirty","fixture":"bench-timing","trial":"timing-000b7e3c"}
{"kind":"scheduler","seq":0,"timeUs":29,"marker":"release"}
{"kind":"scheduler","seq":1,"timeUs":35,"marker":"capture"}
{"kind":"scheduler","seq":2,"timeUs":36,"marker":"complete"}
{"kind":"end","dropped":0}
```

`release` marks the start of the bench pass. `capture` marks its successful motion snapshot read. `complete` marks the end of the measured work before sleep. Trial and build identify the capture. A nonzero `dropped` means the RAM buffer filled. A file without its `end` row is an interrupted export. Use a board trace of the actual task and pins for control-task and GPIO latency qualification.

The device trace format also supports `scheduler` records with a `contention` marker, `servo` and `trigger` records, and `reply` records with the echo class and range in millimetres. Device rows share `seq`, `timeUs` and optional `cue`. Servo, trigger and reply bearings use deci-degrees in `requestedDeci` and `effectiveDeci`. Servo rows include `pwmUs`. Reply rows include `echo` (`valid`, `open`, `too_close`, `no_echo`, `glitch` or `stuck`) and `rangeMm`. Use the [servo fixture](../tests/fixtures/bench-trace.jsonl) for the `reference` row that joins independent measurement by trial and cue.

## Qualify sonar center and travel

The current firmware cap is ±20°. Qualify ±30° first on the installed mount. With the servo axis 10 mm from the rear wall, the board rear plane 5 mm forward of the axis and 22.5 mm to either board edge, near-corner clearance is `10 + 5 cos(θ) − 22.5 sin(|θ|)` mm. Check those dimensions and anything protruding behind the board.

| Command magnitude | Nominal wall gap |
| --- | ---: |
| 20° | 7.00 mm |
| 30° | 3.08 mm |
| 35° | 1.19 mm |
| 37° | 0.45 mm |
| 38° | 0.09 mm |

With propulsion disabled, mark the chassis forward centerline using the axle midpoint and a second chassis reference. Mark the board normal by placing a square against the face. Command 0° and compare the lines with an overhead camera or angle gauge. Record signed yaw error at cold boot and after returning from each wing. Aim for a repeatable error within 1°. Adjust the horn or center pulse if zero is consistently offset, then repeat. The current nominal center pulse is 1494 µs. A difference between return directions exposes backlash or compliance. A 1° centering error consumes about 0.37 mm of one side's nominal clearance near 35°.

Use a bench-only servo command to increase angle in small increments on both sides and watch the minimum rear gap. Include powered reversals and settling overshoot, with the cable routed as it will be on the course. Record the smallest gap and the measurement uncertainty. Stop increasing travel if any part touches or the measurement cannot distinguish the gap from zero. Keep the course firmware at ±20° pending qualification of both ±30° endpoints. Consider ±35° after that pass. Enabling the wider candidate in the course image also needs one spare bit in the internal ping reply.

A bench trace can capture timestamped PWM, requested and effective commanded angle, trigger, reply, range and echo class. Join camera or gauge measurements by trial and cue and label physical angle and gap separately. The servo provides no physical-angle feedback. Capture to RAM and export after the fixture stops so console output does not alter timing. The standalone bench currently writes USB logs. A separate read-only Bench replay is planned.
