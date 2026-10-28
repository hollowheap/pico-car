# Buddy 2 synthetic guidance bench check

Build the opt-in `buddy2_guidance_check` target. This reuses the once-only
GP20 start / GP21 terminal-stop speed-check harness and real motor service.
It supplies **simulated decoded decisions**, not real IR or sonar readings.
The default team `main` does not include this test.

| Time after start | Synthetic input | Requested left/right speed |
| --- | --- | --- |
| 0–0.6 s | Straight | 50 / 50 mm/s |
| 0.6–1.2 s | Steer left | 25 / 50 mm/s |
| 1.2–1.8 s | Steer right | 50 / 25 mm/s |
| 1.8–2.0 s | Obstacle hold | 0 / 0 |
| 2.0–2.2 s | Obstacle + line-lost holds | 0 / 0 |
| 2.2–2.4 s | Clear obstacle, retain line-lost | 0 / 0 |
| 2.4–3.0 s | Release line hold, new straight decision | 50 / 50 mm/s |
| At 3 s | Terminal stop | 0 / 0, reboot required |

50/25 mm/s are experimental test inputs, not measured maximum speeds or
final line-following settings. The initial PI gains, 20% duty cap, unverified
wheel scales, 200 ms request lease and existing fault checks are retained.
Decisions refresh every nominal 10 ms with a new synthetic timestamp. This
does not verify real sensor freshness, polarity, barcode logic, a steering
angle, ground trajectory or a fully autonomous car.

New holds are asserted before old reasons are released. Motion submissions
during any hold must return `E_BUSY`; unexpected API results end the run.
The final release uses a new timestamp and retains the ordinary braked
iteration before fresh motion. Either GP21, an observed service fault or a
producer gap over 100 ms ends the run. These are polled software mechanisms,
not independent hardware cutoffs or a proven physical stop-time bound.

For a physical check, lift both wheels clear, leave carrier M1/M2 buttons
released, flash this specific UF2, wait for `READY guidance trial service=1`,
then press GP20 once. Start serial recording first. No active-motion printing
occurs. After the owner acknowledges terminal stop, the frozen report repeats.
The firmware cannot start a second run without reboot.

`DATA` adds owner-reported left/right targets and the hold mask to the speed
trace's velocity, requested PWM, quality and invalid-transition fields.
Hold bits are obstacle=1, line-lost=2. A newly posted hold can appear before
the owner's next zero-target update; allow for this transition when reading
the trace. The frozen trace is decimated, not an exhaustive timing record.
Check target progression, rejection during holds, zero requested duties during
established holds, fresh resume and terminal stop. This short test does not
require each speed transition to settle or retune the PI gains.

The host replay test traverses the real planner, inbox, sampler and controller
with fake output and artificial encoder progress. It checks all seven stages,
overlapping holds, brake-before-resume, forward-only capped output, 32-bit
clock wrap, deadline and no automatic restart. Existing tests separately
exercise emergency priority, stale requests and producer-gap termination.
Passing those tests is software evidence; hardware results must be recorded
separately after the actual run.

## First wheels-up result, 22 October 2026

Firmware source `2ad2993`, UF2 SHA256
`C1B3139EEA6547C67A8FBC2CE0D6C5741823AFC047A61502A654E34E5050FC15`.
One GP20 start completed all seven stages. The owner confirmed both wheels
stopped. The complete capture contained 105 rows, no dropped rows and no
reported init/API/controller/producer-gap/brake failures; final state was
terminal STOPPED. All recorded speeds had valid interval flags, recorded
invalid-transition deltas were zero, and requested duties stayed within cap.

Using 50 ms margins around each stage transition, all seven stages had
matching owner-reported targets/hold masks. Both requested duties were zero
through the established obstacle, overlapping and line-only holds. Clearing
obstacle alone retained the brake; fresh straight guidance resumed after the
final release. Observed mean encoder speeds during left/right stages were
30.08/46.19 and 43.61/31.07 mm/s respectively, consistent with slowing the
inside wheel. These short transitions are not settled-speed acceptance tests.

Largest observed owner interval was 20.177 ms and request-to-observed-stop
acknowledgement was 19.949 ms. Neither is a guaranteed deadline or physical
stop latency. Scale verification and distance-history quality remain false;
this result does not close the separate odometry issue. Real sensor mapping,
sensor supervision and loaded line following remain to be integrated.
