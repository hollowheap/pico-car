# Buddy 2: encoder bring-up and remaining commits

## What is implemented

The local `feature/buddy2-encoder` branch has checked integer speed/distance
calculations, a tested quadrature decoder, and an opt-in encoder capture
diagnostic. The normal `main` target remains the leads' RTOS demo.

`buddy2_encoder_check` is a separate firmware entry point. It sets GP8/9/10/11
to low SIO outputs (brake in the Cytron ROBO-PICO truth table), never enables
motor PWM, and prints counts while a wheel is turned by hand. This is not
autonomous motion firmware. Servo, IR, sonar and IMU pins are not configured by
this application. The kernel's existing USB and radio services still start.

| Signal | GPIO |
| --- | --- |
| Left motor A / B | 8 / 9 |
| Right motor A / B | 10 / 11 |
| Left encoder A / B | 16 / 17 |
| Right encoder A / B | 4 / 5 |

The motor pins match the submitted Week 6 table. The encoder left/right labels
above instead follow isolated physical-wheel tests on 21 October 2026: M1
rotates the left wheel and produces encoder activity only on GP16/17; M2
rotates the right wheel and produces activity only on GP4/5. The submitted
table assigned those encoder pairs to the opposite wheels. Reconcile that
documentation with the tested car before team integration; the harness has
not been rearranged to match the old labels.

The owner identifies the installed motors as Yahboom TT six-pin motors,
SKU RM-YAHB-064. After initial zero-count tests, the owner found reversed
encoder VCC/GND connections, corrected them and reported changing raw inputs.
Encoder VCC is now reported connected to 3.3 V; voltage has not been measured.
The capture diagnostic then counted real motor-driven rotation in both
directions on both wheels. This establishes feedback activity and wheel-pair
mapping, not calibrated distance, speed or loss-free capture.
Use this target for two-channel quadrature encoders with signals suitable for
3.3 V GPIO, not a single-channel HC020K connected as if it supplied A and B.
Inputs use no internal pull resistors. An open-collector encoder may need
external pull-ups to 3.3 V; establish its output type before changing pulls.

### Isolated carrier-button results

Each row is the counter delta between stable readings before and after a
single button press. The owner confirmed the physical wheel and direction
reversal. Button hold times were approximate, not measured timing evidence.

| Button | Physical wheel | Encoder GPIO pair | Positive delta | Negative delta | Invalid delta |
| --- | --- | --- | --- | --- | --- |
| M1A | Left | 16 / 17 | 0 | 16732 | 4 |
| M1B | Left, opposite to M1A | 16 / 17 | 9566 | 0 | 2 |
| M2A | Right | 4 / 5 | 10416 | 0 | 3 |
| M2B | Right, opposite to M2A | 4 / 5 | 0 | 10434 | 3 |

The other wheel's positive, negative and invalid counters stayed unchanged
in every isolated test. Logs are `encoder-isolated-baseline.log` and
`encoder-isolated-m1a.log` through `encoder-isolated-m2b.log` in the owner's
Codex artifact folder. Those logs were captured before the mapping correction:
their printed `L` means GP4/5 and printed `R` means GP16/17, so interpret the
historic labels using the physical GPIO pairs in this table.

The positive/negative signs describe phase order, not established car-forward
direction. Invalid transitions occurred in every powered test. Their cause
and the total number of missed edges remain unknown; precise distance/speed
and full-speed capture are not accepted on the basis of these tests. Next,
compare slow known-turn counts and investigate ambiguous capture before using
the counters for closed-loop motion.

## Build

Configure using the team's normal environment (`nix develop`, `./configure.sh`)
or the already configured local WSL build directory. Then explicitly build:

```sh
cmake --build <configured-build-directory> --target buddy2_encoder_check
```

The output is `buddy2_encoder_check.uf2`. `main.uf2` is the existing RTOS demo
and does not run the encoder check. Build outputs belong outside the checkout
when using a personal build helper. Compilation is not hardware verification.

### Internal input-path self-test

`buddy2_encoder_selftest` is a separate optional target. After USB startup it
uses INOVER to supply synthetic input levels internally, and PROC0_INTF to
force matching GPIO interrupt events. The pins remain inputs: it checks pad
output-disable and SIO direction first, and never drives the encoder wires or
changes motor output commands. Physical GPIO edge enables are masked during
injection so the attached encoder cannot contaminate this test. The test-only
ISR clears INTF as well as acknowledging physical edge latches.

Each wheel receives five cycles in each direction: the expected result is 20
positive and 20 negative transitions per wheel. A forced simultaneous two-bit
jump on the left must increment its invalid count once, with no fabricated
distance. Test output says PASS only if these counts and final states match.
The GPIO controls/enables are restored and all synthetic counts are discarded
before normal external-input logging begins. Do not move wheels or press
motor-test buttons during the internal test.

This checks the software input-read, forced-interrupt delivery and decoder
path. It bypasses the external electrical signal and physical edge detector,
so PASS does not validate the encoder, wiring, calibration or maximum rate.
Host arithmetic/decoder tests and successful compilation are separate evidence.

Before the encoder power correction, the owner requested a four-button repeat
test on the real inputs before this
self-test was added. The 90-second serial recording contained 220 complete
samples, all with live pairs 11/11, zero cumulative polled changes/decoded
edges/invalid transitions, pending mask 0 and enable mask 3. That observation
was recorded before the owner identified reversed encoder VCC/GND. It is
historical diagnostic evidence, not the current feedback status; see the
post-correction isolated results above.

## First board test

1. Confirm the board is the Pico W / RP2040 expected by this build, the carrier
   is ROBO-PICO, both encoder channels are present and signals are GPIO-safe.
   BOOTSEL's `RPI-RP2` ID identifies the RP2040 bootloader, not Pico vs Pico W.
2. Support the car so wheels are off the table. Keep the motor battery/drive
   supply disconnected for the initial test. USB alone can power ROBO-PICO's
   motor rail, so unplugging only the battery is not a motor isolation switch.
   Do not assume all encoder modules receive power in that configuration.
3. Enter BOOTSEL and copy **only** `buddy2_encoder_check.uf2` to `RPI-RP2`.
   This replaces the existing application on the board. The drive disappears
   when it reboots. This step is performed deliberately by the owner.
4. Open the new serial port at 115200 baud, 8N1. USB CDC does not use baud to
   time the encoder. The capture task begins before its bounded USB wait.
5. Check the banner says `motor drive DISABLED`. Turn the left wheel slowly
   by hand, then the right. Do not force the gearbox if it cannot be back-driven;
   use an appropriate accessible encoder shaft or a later controlled motor test.
6. Record baseline/end counters for a known number of complete wheel turns,
   in each direction, separately for each wheel. Use wheel revolutions, not
   motor-shaft revolutions. Repeat to establish reproducibility.
7. Send the log and results before adding powered movement. Rising `bad` counts,
   the other wheel's counts changing, or both positive/negative counts rising
   during steady one-direction motion require investigation first.

Example log format (values are not expected calibration numbers):

```text
t=<microseconds> L_ab=<0..3> live=<0..3> poll=<count> +=<count> -=<count> bad=<count> R_ab=... pend=<mask> en=<mask>
```

`+` and `-` describe A/B phase order, not calibrated car-forward or reverse.
The decoder counts all valid A and B transitions (x4). The counts per wheel
revolution must be measured in this same mode. `bad` counts ambiguous capture
observations, not the exact number of missed physical edges. If a channel is
disconnected, a one-bit signal can bounce between positive and negative steps
without raising `bad`; zero bad counts alone cannot validate quadrature.

`L_ab` / `R_ab` are the last ISR-decoded states. `live` reads the actual GPIO
input at snapshot time. `poll` counts task-observed A/B state changes, with a
requested 1 ms task delay; actual sampling depends on kernel timing. It can
miss transitions and must not be used as odometry or a rate reference. `pend`
latches pending left/right edge observations during each reporting interval;
`en` reports GPIO edge enables AND the calling core's NVIC enable. Both masks
use bit 0 for left and bit 1 for right. `en=3` is expected in this diagnostic,
whose reader is pinned to core 0. Live changes without ISR counts require
interrupt-path investigation; no live changes during confirmed rotation require
checking signal wiring/power and GPIO configuration. Neither observation alone
proves the encoder module is defective.

## Capture design and limits

The capture task is pinned to `TP_PRC1` (RP2040 core 0). It installs a high-level
handler through `tk_def_int()` and enables IO_IRQ_BANK0 only on that processor.
It refuses to start if GPIO IRQ enables are already in use on either core.
The isolated diagnostic owns the bank IRQ; it must not be dropped into the
combined car alongside an independent sonar IRQ handler. Before integrating
Buddy 5, agree a single GPIO-bank dispatcher or a different capture peripheral.
Do not overwrite the other's handler. The shared vector table is kernel-owned.

The ISR acknowledges only its observed encoder edge flags, samples A/B once,
and updates counters under `ISpinLock`. Reading copies both wheels and a
microsecond timestamp under that same interrupt-preserving cross-core lock.
No printf, division, allocation, blocking calls or motor writes occur while
holding it. The timestamp is the software snapshot time, not an edge timestamp;
interrupt service latency contributes to interval measurements.

Two-bit jumps, both edge polarities pending on one pin, or pending edges on
both channels are treated as ambiguous: increment `bad`, resynchronise A/B,
and do not fabricate distance. Hardware edge flags coalesce, so even this
cannot detect every missed cycle. Maximum-rate counting, latency, electrical
noise and scheduling performance need tests on the car. Consider PIO capture
if measured edge rates/interrupt masking make GPIO capture inadequate.

No-edge observations are not proof of physical rest or a healthy encoder.
The log runs approximately every 500 ms; this is a diagnostic output rate, not the proposed
10 ms motion-control loop. The diagnostic does not implement a stall detector,
watchdog, physical-stop acknowledgement or an autonomous safety guarantee.

## Data representation status

| Field | Valid range / logical bits | Implementation / status |
| --- | --- | --- |
| One encoder A/B sample | 0..3 / 2 | Explicit masks and shifts |
| Both wheel A/B samples | 0..15 / 4 | One byte, upper four bits zero |
| Live states + pending and enable masks | 4 + 2 + 2 = 8 bits | Explicit shifts in one diagnostic byte |
| Positive, negative and invalid counters per wheel | 0..2^32-1 / 32 | Provisional modulo diagnostic counters |
| Snapshot timestamp in microseconds | 0..2^32-1 / 32 | Matches low hardware timer; wraps in about 71.6 min |
| Speed/distance calculation result | 0..2^32-1 / 32 | Checked arithmetic API, not final shared message |

The 32-bit diagnostic counter domain is an explicit temporary implementation
choice, not evidence that 32 bits is the minimum physical requirement.
Final counter windows, distance/speed ranges and resolutions need calibration,
an agreed maximum rate, sampling deadline and overflow policy. Do not simply
reuse this diagnostic struct for queues or MQTT. A C struct has padding and
is not a portable binary protocol. `encoder_quad_t` is 28 bytes and the capture
sample is 32 bytes with the current RP2040 ABI; verify actual emitted debug type
information/build sizes before citing these as memory evidence. This phase
does not claim final minimum-bit compliance or before/after RAM savings.

Each counter is modulo 2^32; compute an unsigned interval delta before
converting. More than one full wrap between reads cannot be recovered.
Positive and negative deltas must be interpreted separately: their sum gives
counted path travel, their difference gives net signed displacement. Do not
pass a decreasing signed position to `encoder_math_count_delta()`.

## Remaining sequence

The next opt-in target is `buddy2_motor_check`, a button-triggered short-pulse
PWM diagnostic with a single output owner and a latched software stop. See
[its procedure and limitations](buddy2_motor_check.md). It does not establish
capture accuracy, calibrated speed/distance or autonomous motion readiness.

| Next local commit / checkpoint | What is needed first | Evidence |
| --- | --- | --- |
| Motor PWM driver, one motor owner, brake/stop latch | Correct carrier, signal voltage and polarity confirmed; test area prepared | Low-duty wheel-up test, stop-output response |
| Calibrated motion sampling and shared snapshot | Measured x4 counts/revolution, wheel travel, direction signs | Repeated turns; straight measured distance; timestamped log |
| Speed controller and straight/turn/distance execution | Loaded speed measurements, track width, chosen operating limits | PID tuning traces, 1 m and 90/180 degree trials |
| Minimum-bit shared motion/command representation | B1/B3/B4/B5 agree range, resolution, validity and maximum age | Field-width derivation, compiled RAM/message sizes, boundary tests |
| Integration with navigation and other buddies | Agreed command/run IDs, stop causes/reset, IRQ ownership | Stale-command, sensor fault, stop, concurrency and timing tests |

No invented wheel diameter, encoder resolution, maximum speed, PID gain or
track width is compiled into this diagnostic. Minimum-bit packing and the
10 ms/physical-stop/accuracy requirements remain verification work; passing
host tests does not establish them. Keep commits local until the owner chooses
to push and create a PR for the lead to review.
