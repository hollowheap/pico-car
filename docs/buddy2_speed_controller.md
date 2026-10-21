# Buddy 2 speed-controller core

`wheel_pid` provides a task-owned integer PID calculation for one wheel.
There are no default car gains, speed limits or autonomous startup. This
module is now used by the optional `motion_control` / `motion_service` owner
libraries (see `buddy2_motion_service.md`). The running motor-batch firmware
does not use it. Build the portable Pico library explicitly:

```sh
cmake --build <configured-build-directory> --target buddy2_motion_core
```

## Units and configuration

Target and measured speed use signed micrometres per second: 1000 um/s is
1 mm/s. Output is signed permille: 200 is 20% duty, not 200 mm/s. Positive
means car-forward after the motion sampler's encoder sign normalisation.

Each gain uses Q16 fixed point: the stored integer divided by 65536 is its
coefficient. P acts on speed error, I on accumulated speed error over seconds,
and D on measured speed change per second. Gain units are documented in the
header. A zero coefficient disables that term, allowing PI tuning first.
The unit-test gains are synthetic arithmetic examples, not suggested car
values. Gains for the left and right wheels can differ.

The caller must supply a target-speed limit, duty cap, maximum sample age and
maximum update interval. These limits are application decisions requiring
bench/loaded validation. Invalid or zero limits make initialisation fail.
Unverified distance/speed scales are rejected unless the caller explicitly
sets `allow_provisional_scale` for bench work. That option does not make the
calibration verified.

## Intended owner workflow

1. One motor-owner task obtains a coherent capture snapshot and updates the
   existing motion sampler, outside the encoder ISR/lock.
2. Give a new sample and each requested wheel speed to its own `wheel_pid_t`.
   Consume `speed_valid` and freshness; distance/turn completion additionally
   requires valid odometry, checked separately by the future command layer.
3. If either update fails, brake both and latch the command-layer fault. A
   failure returns zero duty and leaves PID state unchanged. It never clears
   the driver's stop latch or authorises automatic recovery.
4. Otherwise apply both calculated duties through `motor_pwm_set_pair()`.
   Respect a braked control interval after reversal/E_BUSY. The PID can ask
   for negative duty when a positive-speed wheel overshoots; the owner must
   handle or constrain that according to the chosen braking policy.
5. Reset both controllers when stopping, changing calibration or switching
   between incompatible commands. A valid zero-speed target returns zero
   duty, clears integral/history, and requests brake rather than position hold.

The calculation uses measured timestamps, rejects duplicates and excessive
gaps, and handles a single timer rollover. D acts on measurement to avoid a
setpoint derivative kick. Integral is bounded by the configured output cap;
integration that pushes further into saturation is rejected. Checked signed
64-bit intermediates refuse unrepresentable arithmetic rather than wrapping.
Per-update fixed-point integration/division truncates fractional remainders;
this is not an exact real-valued PID model. Some extreme intermediates are
rejected even when a differently ordered rational calculation could fit.

## Evidence and remaining integration

Host tests cover exact P/I responses, signed output, saturation and anti-windup,
first-sample/no-setpoint-kick D behavior, stale/invalid/unverified data,
duplicate timestamps, timestamp wrap, target limits and arithmetic overflow.
The library compiles for the pinned Pico toolchain. These are software checks;
no physical closed-loop stability, tracking accuracy or speed-limit validation
is claimed. The derivative path is unfiltered; keep D disabled until its
noise behavior has been assessed. No hardware controller settings are supplied.

The optional RTOS adapter now provides stop priority and command timeouts;
it still needs physical validation and controlled speed tuning, followed by
distance completion, turn geometry and Buddy3/obstacle integration.
Do not replace the line-following IR decision
with this controller: Buddy 3 proposes wheel-speed/steering requests; Buddy 2
owns feedback, output and stop handling. The exact shared API still needs team
agreement. Track width is unknown; no angle-turn command is enabled.
