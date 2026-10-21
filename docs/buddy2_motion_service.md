# Buddy 2 motion owner and provisional teammate API

`motion_control` connects coherent encoder input, the motion sampler, two
wheel PIDs and one pair-output interface. `motion_service` supplies the
micro T-Kernel task, hardware adapter and protected cross-core command/status
copies. They are opt-in libraries. The team `main` and currently flashed
motor-batch firmware do not start this service.

```sh
cmake --build <configured-build-directory> --target buddy2_motion_service
```

## Interface for the team

After the startup task calls `motion_service_start(&config)` successfully,
other tasks may call these task-context APIs on either core:

| API | Meaning |
| --- | --- |
| `motion_service_set_speed(left_um_s, right_um_s, ttl_us)` | Submit signed wheel speed targets; positive means car-forward. |
| `motion_service_request_stop()` | Emergency stop request, latched until reboot. |
| `motion_service_read_status(&status)` | Coherent copy of state, faults, last command ID and timestamped measurements. |

1000 um/s = 1 mm/s. Duty is internal signed permille, not physical speed.
`E_OK` from the speed API means the request was accepted into a one-entry
latest-command slot, not that it has already reached the motors. A newer
pending request replaces an older pending request. There is no accumulating
motion queue. TTL is stamped when the service accepts a request; it does not
prove the upstream IR, network or obstacle information was fresh. Producers
must validate that information themselves. IDs are nonzero uint32 counters
for correlation, wrapping back to 1; they do not provide global ordering.

Agree one navigation/command authority for ordinary speeds. Several buddies
must not race independent speed decisions into this slot: latest arrival wins,
with no navigation priority arbitration. Buddy3 can supply line-following
wheel targets through that authority; obstacle decisions must inhibit/alter
them there. A normal pause is a zero/zero speed request, coordinated so later
line commands cannot immediately overwrite the pause. Emergency stop has its
own latched slot and always wins over pending motion. Obstacle recovery and
navigation state machines are not implemented by this service.

This is a proposed source API, not the final minimum-bit shared/MQTT schema.
Never send these compiler-padded structures directly as a wire packet.

## Startup and configuration

Call `motion_service_start()` once from a core-0 startup task, before launching
command producers or status readers. The configuration is copied and immutable
after startup. `E_OK` means the task was created; wait for `status.ready` and
check `status.init_error`. Zeroed/incomplete configuration returns `E_PAR`.
There is no public restart/clear-fault API. Failed startup requires correction
and reboot, rather than repeatedly allocating tasks/IRQs.

Explicit settings are required for:

- Both wheel scales/signs and the sampler's maximum interval.
- Per-wheel PID gains, target-speed limit, duty cap, maximum sample age and
  maximum PID interval. P or I must be nonzero for each wheel; D-only cannot
  provide startup speed control.
- Maximum command TTL, active control gap, no-edge timeout and reversal-brake
  interval. No-edge timeout must exceed the allowed control gap.
- A nominal task polling delay in milliseconds. It must fit within the
  configured control gap; actual scheduling can be slower and must be measured.

No motor operating speed, PID gains or stop deadline is guessed. The present
wheel scales remain provisional. Bench use requires explicitly permitting
provisional scale in each affected PID; status continues to report it as
unverified. The synthetic unit-test settings are not car tuning values.

The core-0 owner initialises PWM slices4/5 and encoder capture, converts
snapshots outside the capture lock, then computes both wheel duties before
calling the pair driver. GP21 is polled as an active-low emergency stop;
GP20 is not used. There is no motion at boot without a valid producer command.
No console printing occurs in the owner task. Other tasks' interrupt-masking
console activity can still affect capture and timing and needs integration
measurement.

This capture driver still owns IO_BANK0 IRQ exclusively. A sonar implementation
using that bank needs a shared dispatcher before coexistence. Reserve motor
pins/PWM slices for this owner. The library is not an automatic drop-in for
another buddy's already-initialised GPIO/PWM driver.

## Control and stop behavior

Stop requests take priority over invalid feedback, command expiry and motion
requests. They are checked before capture and again after capture. A stop
arriving after the last check is processed on a later owner iteration; this
is polling, not an instantaneous asynchronous cutoff. Neither stop nor a
fault can be cleared by a new command. Releasing GP21 cannot restart motion.

Command expiry, excessive active task gap, invalid/stale/unverified speed,
sampler/PID arithmetic failure, missing encoder movement while PWM is nonzero,
or output failure causes a terminal fault and a latched brake request. A
freshly arrived command cannot conceal expiry of the active command. Repeated
command refreshes do not reset the no-edge timer. No-edge failure is not a
diagnosis of wiring versus supply/torque: investigate instead of raising duty
automatically. It does not detect every noise, wrong-direction or slip fault.

Same-direction speed changes retain PID history so line steering does not
erase integral action every cycle. Direction/zero transitions reset the
affected PID. If a PID asks for output opposing its target (overspeed), the
owner uses brake rather than active counter-drive. Before a requested drive
direction reversal, both wheels stay braked for the configured interval.
An intervening zero command cannot bypass this interval. Unexpected driver
`E_BUSY` also brakes and starts a full reversal interval before retrying.

Published duty values describe requested output, not measured electrical
voltage or proof of rest. `brake_failed` signals a rejected brake call; a
software request cannot guarantee a functioning power stage. `sample_available`
means the current iteration produced a sampler result, not that all quality
flags are good. Consumers must inspect the sample timestamp and per-wheel
speed/odometry/scale flags. Historical target values may remain after a stop;
the state and requested duty show what the controller is doing.

## Checks and limits

Host CTest groups pass, including an integrated fake-output test from
encoder counters through sampling and both PIDs. It exercises healthy refresh
and movement, stop priority, command expiry, no-edge detection despite command
refreshes, invalid/stale/unverified feedback, whole-pair fault handling,
reversal dwell (including through zero), driver retry/failure, overspeed brake,
clock rollover and preserved integral state across steering changes.

The kernel adapter and dependencies compile for Pico with application warnings
treated as errors. A first3second wheels-up service/PI trial from107f5e1 now
ran without reported software faults; observed last-second encoder-speed
means were50.443/49.655mm/s for a50mm/s request. See `buddy2_speed_check.md` for
the full settings, limits and open pre-run odometry-quality issue. This does
not characterise worst-case cross-core scheduling, stack margin, lock timing,
loaded stability or physical distance accuracy. The standalone speed-check
was flashed; this service is still not enabled in team main.
No 10ms control or physical-stop guarantee is claimed. Task/CPU failure,
carrier motor-button override, motor supply quality and physical stopping
distance remain outside these software checks.

The opt-in `buddy2_speed_check` now supplies explicit experimental settings
and a bounded wheels-up PI trial (see `buddy2_speed_check.md`). Next: run it,
then loaded speed/distance tests. Distance completion and angle-turn APIs are
not provided yet; turns also require measured wheel track. Buddy3 IR/barcode,
obstacle/navigation arbitration and final packed telemetry remain integration
work.
