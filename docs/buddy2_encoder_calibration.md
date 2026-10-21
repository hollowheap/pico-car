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
The GP20 user button starts a measurement and GP21 finishes it. Neither
button commands motor movement in this target. Both require a release
before a debounced press is accepted; held GP20 does not restart a trial.

The pinned kernel's `tm_printf` masks local interrupts while formatting and
emitting output. Earlier live-print trials recorded invalid transitions,
so this variant prints only before or after a measurement. This isolates
one plausible source of capture loss; it does not prove the loss was caused
by printing, or remove other kernel/USB interrupt latency.

## Procedure

1. Put a visible mark on the left tyre and choose a fixed reference on the
   chassis. Align them while leaving the right wheel still.
2. Stop touching the wheels. Press and release GP20, then wait one second.
   The output becomes silent: firmware has saved the baseline and started
   the interval. Do not reposition the wheel after pressing GP20. Counts
   accumulated before the baseline, including mark alignment, are excluded.
3. Rotate only the left wheel slowly through exactly five complete turns in
   one direction, ending with the same mark at the same reference. Count
   each return to the mark; avoid back-and-forth rocking.
4. Leave both wheels still. Press and release GP21. Firmware freezes the
   ending snapshot before printing. There is no automatic time limit; keep
   USB connected. If interrupted or unsure of the turns, discard the trial.
5. Read the repeated `TRIAL ... COMPLETE`, `BEGIN`, `END`, `LEFT delta`
   and `RIGHT delta` lines. These are frozen results, not live counters.
   Starting another trial with GP20 replaces the previous result. Compute
   positive, negative and invalid deltas; check the other wheel too. Repeat
   the trial, then do the same for the right wheel.

For each five-turn trial, net counted edges are the absolute difference
between positive and negative deltas. Divide net edges by five to obtain a
candidate x4 count per wheel revolution. This target does not count how many
physical revolutions were made; the owner must count them. The separate
live-pin diagnostic's `poll` field is not an encoder distance measurement.

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
