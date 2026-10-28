# Buddy 2 interface for sensor-led driving

For the exact header, startup settings and calls to use, begin with
[Buddy3's quickstart](buddy3_motion_quickstart.md).

Buddy3 owns IR/barcode decoding. Buddy2 accepts decoded motion decisions and
owns encoder feedback/PWM/stop. This code reads no IR GPIO and assumes no
black/white polarity or two-bit truth table. No Buddy3 IR implementation was
found on the fetched main/available branches. The API is provisional and can
be adapted to their actual header.

| Decision | Left speed target | Right speed target |
| --- | --- | --- |
| `MOTION_DRIVE_STRAIGHT` | requested speed | same requested speed |
| `MOTION_STEER_LEFT` | inside-wheel speed | requested speed |
| `MOTION_STEER_RIGHT` | requested speed | inside-wheel speed |
| `MOTION_DRIVE_HOLD` | zero | zero |

Inside-wheel speed is0..requested forward speed. The navigator chooses speed
and steering strength; no ratio or final operating speed is guessed. This
interface handles gradual steering, not a calibrated angle/pivot command.
1000um/s=1mm/s. Equal straight targets can require different motor PWM.
No distance/turn measurement is needed to choose these actions. Wheel-speed
feedback retains the provisional scale; the separate odometry-quality issue
is not cleared and remains open for later distance commands.

## Buddy3 task example

After decoding the real sensors and checking their quality:

```c
motion_drive_request_t request = {
    .action = decoded_action,
    .speed_um_s = configured_forward_speed_um_s,
    .inside_um_s = configured_inside_speed_um_s,
    .sensed_us = ir_sample_timestamp_us,
    .valid_for_us = configured_ir_lease_us
};
ER result = motion_service_submit_request(&request);
```

Use the same RP2040 low32-bit microsecond clock as capture/service. Preserve
when the source sensor data was read; do not stamp cached data as new.
The original timestamp/lease survives planning and mailbox admission, so
resending cached guidance cannot extend its life. Straight requests are
rejected if either configured wheel limit cannot accept that same speed.

`E_OK` means queued, not applied. `E_BUSY` means a normal hold blocks it.
Invalid guidance returns `E_PAR`; expired/old guidance can return `E_TMOUT`.
Bad guidance cancels pending movement and queues an ordinary brake. If an
active command has already expired, its terminal timeout fault still wins.
No request clears a terminal fault/emergency stop.

The existing `motion_service_set_speed()` is still available for trusted
direct wheel commands and bench code. It stamps a new request at admission
and cannot know whether its caller reused an old sensor decision. Use the
timestamped request API for IR decisions.

## Independent normal holds

One navigation authority should issue ordinary straight/steering decisions.
Separate sensor holds override them without racing a zero-speed request
against fresh line-following requests:

```c
/* Buddy5's obstacle reason; assert for detected obstacle/known invalid data. */
motion_service_set_hold(MOTION_HOLD_OBSTACLE, true);
/* Release after a valid current clear decision: */
motion_service_set_hold(MOTION_HOLD_OBSTACLE, false);

/* Buddy3's line-lost reason: */
motion_service_set_hold(MOTION_HOLD_LINE_LOST, true);
motion_service_set_hold(MOTION_HOLD_LINE_LOST, false);
```

`MOTION_HOLD_OPERATOR` is a third reason. Each call accepts one known bit;
clearing one leaves the others set. The team must agree which task owns each
reason; this is not caller authentication. While any reason is held, motion
posts return `E_BUSY` and the owner requests zero/zero. `status.hold_mask`
reports reasons independently of hardware `ready` and controller state.

A hold set AND cleared between owner polls still delivers one braked
iteration. Old pending motion is discarded. After the final release,
timestamped guidance sensed before release cannot restart movement. Take a
new reading; the next fresh frame may resume behind the brake event.
Repeatedly clearing an already-clear bit does not keep resetting this barrier.

Assert unknown sensor conditions before launching drive producers and release
each only after valid data. Hold bits are interlock states, not heartbeat
monitors: stale/error detection and supervision of a sensor task that stops
running still need integration. A last obstacle-clear state does not prove
Buddy5 is still healthy. Check those freshness paths and physical stopping
before calling the full car autonomous.

Emergency `motion_service_request_stop()` remains latched until reboot and
wins over normal hold/motion. Owner polling services these requests; they
are not instantaneous asynchronous hardware cutoffs.

## Checks and remaining work

All13 host CTest groups pass. Tests cover equal straight targets, left/right
mapping, bad ranges/expiry/wrap, unchanged source leases, latest replacement,
all combinations of hold reasons, set/clear between polls, old-frame
rejection, brake-before-resume, long-uptime freshness barriers, emergency
precedence and guidance through inbox/sampler/PIDs/fake PWM. No real IR sensor
values were invented for these tests.

The updated service links into the Pico speed-check and guidance-check targets
with application warnings as errors. The opt-in `buddy2_guidance_check` replays
straight/left/right, overlapping holds and fresh resume through the real service.
See [the synthetic check instructions and physical result](buddy2_guidance_check.md).
The 2ad2993 guidance firmware was flashed and completed all seven wheels-up
stages: 105 trace rows, no drops or reported software faults, expected
steering/hold/resume targets and owner-confirmed final stop. The robot now
runs that completed, terminally stopped test; a reboot permits another
intentional GP20-triggered run. Actual IR/sonar inputs remain untested.
Main, dependency pins, PID gains, calibration flags and other buddies' code
are unchanged.

Next: Buddy3 maps its actual sensor decisions/timestamps to this API; Buddy5
connects obstacle holds plus stale/error supervision. Check real sensor-to-
action mapping, then run a low-speed combined test. Distance/angle calibration
is deferred until a distance/angle command actually needs it.
