# Buddy 2 speed and displacement sampling

`motion_sampler` converts coherent encoder snapshots in task context. It
normalises the verified wiring signs (left quadrature negative is forward;
right positive is forward), calculates signed interval-average wheel speeds,
and retains signed total encoder displacement. It does not drive motors,
follow the IR line, perform PID, or calculate heading/turn angle.

The pure library accepts configuration rather than assuming a motor model.
The optional `buddy2_motion_check` firmware uses these provisional values:

| Wheel | x4 edges/revolution | Wheel travel/revolution | Forward sign |
| --- | --- | --- | --- |
| Left | 2541 | 210487 um | -1 |
| Right | 2530 | 210487 um | +1 |

These are rounded means of hand-turn estimates: left 2542.8 and 2539.2;
right 2539.2 and 2520.8. Nominal travel is pi times the owner's 67 mm tyre
diameter, rounded to whole micrometres. Loaded rolling travel has not been
measured. Both `scale_verified` flags remain false. Earlier left trials
registered some right-wheel counts, whose cause is unresolved. The latest
left report had a truncated header, although all six beginning/ending
counters and the printed deltas matched. These measurements do not establish
exact encoder CPR or physical distance accuracy.

## Interface and quality

Initialise one single-owner `motion_sampler_t` with a baseline and immutable
configuration. Supply coherent increasing positive/negative/invalid event
counters and their microsecond timestamp to `motion_sampler_update()`.
The capture driver's snapshot can be copied to `motion_input_t` after its
lock has been released. Conversions and printing must not occur in its ISR.

The output contains timestamp, actual elapsed microseconds, signed net edges,
signed displacement in micrometres, and signed speed in micrometres/second.
1000 um = 1 mm; 1000 um/s = 1 mm/s. Displacement is measured from the baseline;
forward then backward can return it to zero. It is not total path length,
car-centre travel, or a measurement of tyre slip.

Total signed edges are retained before converting displacement, so repeatedly
rounding short interval distances does not accumulate numerical drift. Speed
uses the measured interval, not a presumed task period. Conversion truncates
toward zero. Internal displacement and edges use signed 64-bit storage; speed
uses signed 32-bit storage with checked overflow. These are internal arithmetic
types with magnitude limits INT64_MAX and INT32_MAX respectively, rather than
special handling for the most-negative signed value. They are
not the project's final packed telemetry/interface bit allocation.

| Condition | Behaviour |
| --- | --- |
| Capture invalids on one wheel | Its interval speed is flagged invalid; its odometry validity stays false until an explicit new baseline |
| Interval exceeds configured gap bound | Both interval speeds flagged invalid and both odometries latched invalid |
| Clean interval after a fault | Speed validity may recover; lost odometry is not reconstructed |
| Zero elapsed time, bad configuration, numeric overflow | Return false without changing state or output; caller must not publish cached output as a new sample |
| Cached sample | Check `motion_sample_fresh()` plus the wheel validity and scale flags before use |

Reinitialisation intentionally starts a new displacement segment. Reinitialise
after capture-counter resets or calibration changes. Unsigned counter/time
rollover is handled only when each counter increments less than one full
32-bit cycle between samples and the timestamp interval is shorter than one
full cycle. Multiple wraps and resets cannot be detected from these values
alone. A zero, fresh, numerically valid speed does not prove physical rest
or healthy wiring. No stationary encoder-failure detector is implemented.

The sampler is not a shared SMP mailbox. A future motion task owns it and must
publish snapshots through agreed kernel synchronisation. Invalid output must
not silently become valid motion feedback; fault response and stop ownership
remain integration decisions.

## Optional hardware probe

Build `buddy2_motion_check` explicitly. It reuses the quiet encoder-calibration
harness and holds both motor bridges in coast, with no PWM. It neither starts
the team demo nor energises a motor. Physical carrier M1/M2 buttons bypass
software: do not press them during this test.

1. Support both wheels clear of the table. Align the selected wheel mark.
2. Hold the GP20 user button for one second, release, then wait one second.
   Confirm the host received `START accepted` before moving. That message is
   emitted before the baseline, followed by silence while recording. A failed
   start or continued idle messages needs investigation before rolling.
3. Turn one wheel slowly; the task samples without serial printing. For a
   five-turn comparison, count five full returns to the mark from zero.
4. Leave both wheels still. Hold GP21 for one second and release.
5. Read the latest complete `TRIAL` block after draining old USB output.
   Result output is frozen. Reboot restarts trial numbering; do not compare
   trial numbers across separate boots. Partial/stale reports need scrutiny.

The frozen result includes encoder counters and `MOTION` lines with signed
displacement, the last sampled velocity, peak absolute valid interval-average
speed, actual maximum sample gap, failure count, and quality flags. The last
velocity can be zero because the wheel is stopped before finishing. The peak
is an observed interval average, not a guaranteed physical instantaneous peak.
The diagnostic uses 200000 us as its provisional gap bound, not as a validated
control deadline or a module requirement. Requested kernel delay is not proof
of a sampling period. Check failures, all quality fields and frozen timestamps.

T-Monitor lacks a 64-bit output format, so the probe refuses to display
displacements outside the signed 32-bit micrometre display range; the library
still retains a checked 64-bit displacement. Host tests cover that larger
domain. No print occurs during the measured interval; other kernel/USB
interrupt latency still exists.

## Verification and next steps

Host tests cover forward/reverse signs, zero/stale/wrapping time, wrapping
counters, invalid capture and long-gap quality, fractional distance retention,
large signed displacement, arithmetic overflow and unchanged-state failure.
They do not verify physical capture accuracy, scheduling deadlines or braking.

Before using these provisional scales in closed-loop motion, compare encoder
distance with measured travel under the actual car load. Investigate unintended
other-wheel counts and incomplete USB reports rather than hiding them. Next
stages are a shared motion-task interface, simultaneous two-wheel actuation,
speed control, distance commands and stop handling. Angle-based turns also
need wheel-track geometry and loaded turn calibration.

## Observed hand-turn checks, 22 October 2026

Pico W / ROBO-PICO ran `buddy2_motion_check` built from `fa72711`. The owner
turned one marked wheel revolution by hand, with wheels lifted. Each interval
started with GP20 and ended with GP21 after the wheel stopped. The table
records separate intervals; an accidental empty interval was excluded.

| Wheel and direction | Forward-normalised net edges | Displacement (mm) | Capture invalids |
| --- | ---: | ---: | ---: |
| Left forward | 2517 | +208.498 | 0 |
| Right forward | 2527 | +210.237 | 0 |
| Left backward | -2443 | -202.369 | 0 |
| Right backward | -2462 | -204.829 | 0 |

The other wheel's counters stayed unchanged in each listed interval. The
owner clarified that the initially requested two-wheel reverse check actually
turned only the left wheel; right reverse was tested separately afterward.
All four reports had zero numeric failures, matching encoder/motion endpoint
timestamps and positive speed/odometry validity flags. Scale verification
remained false. The largest observed sampling gap across these tests was
20025 us; this is not a worst-case scheduling bound or proof of a 10 ms loop.

Distances independently matched the configured integer conversion. Last
sampled velocity was zero after stopping. Reported peak absolute interval
speeds had no external physical-speed reference. Correct signs and clean
reported invalid counters do not establish exact counts/revolution or prove
that no full quadrature cycles were missed. The measured mark precision and
gearbox lost motion were not independently established; no cause is assigned
to differences from the nominal 210.487 mm circumference. Loaded measured
rolling distance remains the next validation step. Raw serial reports and
qualified arithmetic checks are retained in the owner's Codex artifacts,
outside the repository.
