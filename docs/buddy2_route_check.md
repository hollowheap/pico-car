# Optional50cm / right3wheel-revolutions /10cm demonstration

`buddy2_route_check` is a separate, opt-in firmware target. It does not change
team `main`, line-following behaviour or the motor-service API. One GP20
press starts a two-second braked countdown followed by:

1. Drive both wheels forward approximately500mm, each at50mm/s.
2. Brake for400ms.
3. Drive the left wheel forward three encoder-estimated revolutions while
   braking the right wheel. This is a rightward arc, not a specified90degree turn.
4. Brake for400ms.
5. Drive both wheels forward approximately100mm, then latch the output stopped.

The route starts once per reboot. GP21 stops it before start, during countdown
or while running. Leave the carrier's full-power M1/M2 buttons released.
This demo reads no IR/sonar and performs no obstacle avoidance. Run on a clear
floor with enough USB cable slack for the initial50cm and the rotation.

USB powerbank operation is also supported: flash from the PC first, then
switch the USB lead to the powerbank. Wait25seconds after power-on before
the released/pressed GP20 start, because this diagnostic initially waits up
to about20seconds for a USB host. There is no automatic boot movement.
Without a data connection the PC cannot capture live serial evidence; RAM
logs are lost on power interruption, so a powerbank-only observation is not
a captured encoder/timing validation.

## Endpoint and quality rules

The provisional67mm circumference210487um and x4counts2541left/2530right
are retained. Each straight-wheel goal is ceil(distance*counts/circumference).
The turning goal is7623left edges and zero right travel. The straight legs
stop each wheel independently at its count goal; a reached wheel does not
restart if its count subsequently rolls back. Physical momentum, slip and
approximate calibration can make the ground distance differ. Equal targets
and speed feedback are not independent heading feedback.

The single core-0 owner uses the existing capture, sampling, controller, PI
and PWM modules, with the same experimental50mm/s limit and20% PWM cap.
There is no automatic duty/gain increase for loaded motors. An insufficient
starting torque or stalled motor can therefore abort instead of completing.

After the button acknowledgement/countdown, while outputs are braked, the
demo explicitly establishes a new coherent capture baseline. The route
sampler and controller sampler begin there; pre-start invalid-transition
counters are retained in the report. No running service is reset, no lost
distance before start is reconstructed, and no scale-verification flag is
changed. Every subsequent captured interval is checked; quality is never
reseeded during a leg or pause. This demonstrates an explicit new distance
segment without claiming to fix the earlier service's unlogged idle history.

Invalid/stale/duplicate feedback, a task/sample gap over100ms, controller
failure, GP21 or a time limit terminates the route. Per-leg limits are25s,
30s and10s; overall limit65s. Software checks act when the task next runs,
not through an independent watchdog. A completed route has phase5/error0
and controller state3/fault0. Phase6 indicates an abort; inspect the report.

The frozen `ROUTE TRACE` prints only after braking/latching. It includes
transitions and nominal once-per-second samples, forward-normalised counts,
speed, applied targets, requested PWM and quality bits (speed0/1, distance2/3,
scale4/5). Sample quality is about capture continuity, not accepted scale
accuracy or proof of ground travel. Counts at a transition precede any
subsequent physical braking drift. No active-motion console printing occurs.

## Validation

All13 host CTest groups pass, including endpoint ceilings, independent wheel
completion, no rollback restart, pause baselines, left-only three-revolution
turn,32-bit clock wrap, quality failures during motion/pause, stale and
duplicate samples, loop/sample gaps, timeout, stop priority, arithmetic range
checks and no automatic restart. Pico route firmware builds with application
warnings as errors. This is software evidence; the first physical route
result is pending.
