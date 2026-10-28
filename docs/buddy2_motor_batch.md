# Buddy 2 two-wheel output and one-start bench test

The optional `buddy2_motor_batch_check` target checks the next output stage.
It does not implement PID, line following, distance commands or angle turns.
The team `main` target is unchanged. Build explicitly:

```sh
cmake --build <configured-build-directory> --target buddy2_motor_batch_check
```

## Physical workflow

Support both wheels clear of the surface before loading or starting this
motor-enabled diagnostic. USB is the present bench supply; simultaneous
motor operation from USB has not yet been verified. Do not press the four
M1/M2 carrier motor-test buttons, which bypass firmware control.

The firmware boots braked. After READY, release GP20, then press it once.
That single start runs the following six steps automatically at 20% duty:

1. Left forward.
2. Right forward.
3. Both forward.
4. Left reverse.
5. Right reverse.
6. Both reverse.

Each step requests a nominal 300 ms pulse followed by at least 400 ms with
brake commanded. These are conservative diagnostic settings, not calibrated
operating speed/duty limits. Actual pulse lengths include scheduling delay.
There is no automatic increase if a motor does not start. Pressing GP21 stops
and latches the sequence until reboot; it cannot resume on releasing GP21.
Completion also latches the output off. A held GP20 at boot cannot start it.

There is no printing between first energising the motors and the terminal
brake. Results are stored in RAM, then repeated every two seconds, allowing
the host to recover a complete report. The console uses interrupt masking,
so it is intentionally kept out of driven intervals.

## Feedback and report

Each pulse reports both wheels' positive, negative and invalid event deltas,
baseline-to-brake observation time and feedback code:

| Code | Meaning |
| --- | --- |
| 0 | Every commanded wheel has nonzero net movement in its expected direction; no reported invalid transitions. |
| 1 | Invalid transition or unavailable capture snapshot. |
| 2 | A commanded wheel has zero net feedback. |
| 3 | A commanded wheel has net feedback in the opposite direction. |

The verified signs are negative raw counts for left forward and positive for
right forward. An invalid transition during a pulse aborts it. Endpoint
missing or reversed feedback prevents the next pulse. Zero feedback cannot
distinguish no movement, inadequate starting torque, a supply issue or failed
capture. Do not infer exact edge accuracy from zero invalid transitions.

An observed active polling gap over 100 ms stops the sequence at its next
update. This chosen diagnostic limit is not a physical stop deadline or a
hardware watchdog. GP21 response, mechanical rest and stopping distance have
not been instrumented. A task/CPU failure and carrier-button override are
outside this software latch's protection. Reports cover the brake command,
not all travel until physical rest. A stop-interrupted pulse may have code 2
simply because it ended before movement; check `button_stop` too.

## Pair output contract

`motor_pwm_set_pair(left, right)` takes signed permille, -1000 to +1000.
Positive is the physically verified forward direction. GP8/9 are left motor
A/B; GP10/11 are right A/B. Only the initialising core-0 task can call the
driver after startup. Other buddies must submit requests to that owner; they
must not write the pins or reserve PWM slices 4/5 independently.

Same-direction magnitudes can change while running. A nonzero sign reversal
brakes both wheels and returns `E_BUSY`; the owner must leave a control
interval braked before retrying. An explicit zero similarly needs a braked
interval before reversing. This API does not enforce a minimum elapsed
dead-time or synchronise the two per-slice compare writes. It prevents both
bridge inputs being commanded nonzero together. Braking switches pins to
known-low SIO before disabling PWM. A stopped wheel's slice is disabled and
reseeded before restart. Enabling uses the atomic SET alias to preserve other
slices. A legacy single-wheel drive and a pair drive cannot run concurrently.

## Verification status

Host checks exercise all signed duty values, channel exclusivity, reversal
detection, invalid inputs, six-step order, start gating, terminal states,
active-gap failure, timer/counter wrap and missing/reversed/invalid feedback.
The original and batch diagnostic targets compile against the pinned team
kernel with application warnings treated as errors. This does not establish
electrical PWM correctness or supply capability. The first batch run below
verifies basic wheel-up actuation, not loaded control. The earlier single-wheel
PWM observations are documented in `buddy2_motor_check.md`.

The measured nominal tyre diameter remains 67 mm. Track width is still
unknown: measure across the car between the midpoints of the two drive
tyres' widths. Angle commands require that measurement and subsequent
physical validation; they are not enabled by this diagnostic.

## Wheel-up batch observation, 22 October 2026

Source `0d8619e`, USB power, 20% duty. After one GP20 start, the owner confirmed
the sequence completed and both wheels stopped. The complete report recorded
state DONE, no gap fault, driver/capture error zero and no GP21 stop request.
Counts are signed raw quadrature edges (left forward is negative):

| Step | Left net edges | Right net edges | Invalid L/R | Observation interval |
| --- | ---: | ---: | --- | ---: |
| Left forward | -453 | 0 | 0 / 0 | 319221 us |
| Right forward | 0 | +464 | 0 / 0 | 300029 us |
| Both forward | -425 | +472 | 0 / 0 | 300020 us |
| Left reverse | +414 | 0 | 0 / 0 | 300024 us |
| Right reverse | 0 | -439 | 0 / 0 | 300025 us |
| Both reverse | +380 | -404 | 0 / 0 | 300025 us |

Every pulse returned feedback code 0. Largest observed active polling gap
was 20012 us. Different left/right counts at equal duty demonstrate why equal
PWM is not a guarantee of equal wheel speed; causes were not isolated. Do not
tune a PID or rescale wheel calibration from these short startup pulses.
There was no independent per-phase physical direction confirmation, but
encoder signs matched the earlier independently observed motor directions.
This run did not test GP21 during simultaneous drive, loaded travel, worst-case
response time or physical stopping distance. The earlier single-wheel GP21
test remains separate evidence. The terminal state latched the software
output off; startup/restart/feedback-fault policy tests also run on the host.

Raw evidence: `motor-batch-check/physical-batch-20261021T173550Z.log` and
`latest-complete-report.txt` in the owner's Codex artifact folder. START
acknowledgement and the entire report were captured continuously; the reader
closed normally after the final report.
