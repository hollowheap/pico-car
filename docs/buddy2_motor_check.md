# Buddy 2 motor PWM bring-up

`buddy2_motor_check` is an opt-in diagnostic using the pinned team kernel/SDK.
The normal `main` demo and encoder-only targets remain separate. Build with:

```sh
cmake --build <configured-build-directory> --target buddy2_motor_check
```

## Before loading

Support the car with both wheels clear of the table and fingers clear. This
firmware can power a wheel after the GP20 user button is deliberately pressed.
Do not operate the four M1/M2 motor-test buttons during this test: those carrier
buttons bypass firmware control and use full motor power. USB can supply the
carrier motor rail. Physical button override is not prevented by the software
stop latch.

## Operation

- At boot, all GP8/9/10/11 outputs are held low (ROBO-PICO brake).
- After the READY banner, release GP20, then press it for at least 20 ms.
- One new press triggers one nominal 750 ms pulse at 20% duty and 10 kHz.
- Separate presses select M1A, M1B, M2A and M2B, then the sequence is complete.
- GP20 held at boot, during motion or after timeout does not trigger another
  pulse. A new release of at least 50 ms and a new press are required.
- GP21 requests brake and latches STOP until reboot. It has priority over GP20.
- The owner task polls the buttons and deadline with `tk_dly_tsk(1)` between
  iterations. Actual response includes kernel scheduling and interrupt delay.
  The 750 ms value is not a measured physical-stop deadline or hardware cutoff;
  a task/CPU failure is not covered. Reset/power removal remains available.

Read the reported encoder deltas after each pulse, and record which physical
wheel moved and whether its rotation would propel the car forward or backward.
M1 refers to the left motor, M2 to the right. A/B labels describe the electrical
drive inputs; car-forward has not been calibrated. The assembled encoder
mapping is left GP16/17, right GP4/5.

Counter deltas cover the sampled interval up to the software brake command.
They do not establish full stopping travel or physical rest. A motor that does
not start at 20% duty may be below its starting duty; do not automatically
increase duty or diagnose an encoder fault from zero movement alone.

## Output implementation

`motor_pwm.c` initialises only PWM slices 4 and 5, their GP8/9 and GP10/11
outputs, and their compare/counter/divider registers. No global PWM reset or
shared PWM interrupt handler is installed. Initialisation refuses slices with
an active counter, wrap interrupt enable or forced interrupt. The motor owner
task must exclusively reserve these slices; do not also allocate them through
the kernel physical-timer API. The known 125 MHz system clock, divider 1 and
TOP 12499 give a nominal 10 kHz PWM frequency, below the carrier's 20 kHz limit.

Only the selected motor input is switched to PWM; the opposite input and
other wheel remain low. A drive request while already active is rejected:
brake must intervene. Brake preloads low SIO values and switches all four pins
to SIO before disabling PWM. This avoids relying on buffered compare updates
or stopping a PWM counter while its output is HIGH. Driver calls after init
are accepted only from the initialising core-0 task. The software stop latch
rejects subsequent nonzero drive requests until reboot.

## Verification and unresolved work

Host tests exercise startup with start held, release/press gating, pulse
timeout, stop priority, restart prevention, ignored presses during motion,
sequence completion and unsigned timer wrap. These test the pure pulse policy,
not electrical outputs, PWM frequency or kernel scheduling on the board.

The new motor driver and firmware need a wheel-up hardware test. The current
GPIO encoder capture showed invalid transitions during earlier manual motor
button and hand-turn trials. A code review has not established their cause.
Invalid counts remain visible; no debounce, guessed edges or error suppression
has been added. Compare errors under this controlled duty before choosing a
capture change. Distance/speed calibration and closed-loop control remain open.

Carrier reference: Cytron ROBO-PICO Board Datasheet, sections 3 and 4
(specifications and motor-driver truth table), available from the manufacturer's
[resource page](https://www.cytron.io/c-raspberry-pi-4/p-robo-pico).
