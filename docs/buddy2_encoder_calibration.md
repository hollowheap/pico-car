# Encoder counts-per-revolution calibration

The owner measured both project wheels at 67 mm outer diameter. The calculated
nominal circumference is about 210.49 mm; actual loaded rolling travel has not
been measured. No encoder counts-per-revolution constant has been calibrated.

## Firmware

Build the opt-in `buddy2_encoder_calibrate` target and deliberately load its
UF2. It reuses the normal kernel encoder capture and decoder. Its startup
banner identifies CALIBRATION, motors COAST and no PWM. The main demo,
encoder-only brake diagnostic and motor-pulse diagnostic remain separate.

Before the task starts, motor pins are low. After successful capture init,
one SIO write sets GP8/9/10/11 high together. On ROBO-PICO, both inputs high
selects coast (high-impedance motor outputs), reducing electrical resistance
to hand turning. It supplies no PWM torque. Gear friction remains; do not
force a stiff wheel. Capture-init failure restores the low/brake state and
does not continue the measurement task.

Support both wheels clear of the table. Do not press the carrier's M1/M2
motor-test buttons during this calibration; they bypass firmware control.
GP20 and GP21 have no calibration action in this target.

## Procedure

1. Put a visible mark on the left tyre and choose a fixed reference on the
   chassis. Align them while leaving the right wheel still.
2. Stop touching the wheels. Record a stable serial baseline after alignment.
   Counts accumulated while aligning are excluded by this baseline.
3. Rotate only the left wheel slowly through exactly five complete turns in
   one direction, ending with the same mark at the same reference. Count
   each return to the mark; avoid back-and-forth rocking.
4. Leave the wheel still. Record stable end counters. There is no timed
   recording window: firmware counters accumulate while USB stays powered.
5. Compute positive, negative and invalid deltas. Check the other wheel's
   deltas too. Repeat the trial, then do the same for the right wheel.

For each five-turn trial, net counted edges are the absolute difference
between positive and negative deltas. Divide net edges by five to obtain a
candidate x4 count per wheel revolution. Do not use the task's `poll` field.

Any invalid/ambiguous delta means the trial does not establish an accurate
edge count. Unexpected movement in the other channel, unstable end counts,
uncertain number of turns or disagreement between repeated trials also need
investigation before assigning a calibration constant. No guessed edges or
error suppression is applied. Zero invalid counts alone is not proof of
correct calibration; the measured turns and repeatability matter too.

The manufacturer's previously identified 2340-edge expectation is a cross-check,
not a replacement for the installed project-motor measurement.

After repeatable counts are established, nominal distance per counted edge
is calculated wheel circumference divided by measured x4 edges/revolution.
Verify commanded travel against a measured floor distance under the car's
normal load before claiming distance accuracy.
