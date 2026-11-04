# Buddy3: start here for motor control

Use `src/motion_service.h`. Do not read every Buddy2 file first.
Your IR task chooses an action; the existing motor task handles capture,
speed feedback, PI and PWM. No shared motor variable needs an `extern`.

```
your IR task -> motion_service_submit_request() -> motor-owner task -> motors
```

## Files and calls you actually use

| File / call | Use |
| --- | --- |
| `src/motion_service.h` | Include this in your task. It includes request/status types. |
| `motion_service_start(&config)` | Application startup calls ONCE from a core0 task, before producers. Creates the private motor task automatically. |
| `motion_service_submit_request(&request)` | Your normal IR/barcode navigation decision. |
| `motion_service_set_hold(MOTION_HOLD_LINE_LOST, true/false)` | Your independent line-lost pause. |
| `motion_service_read_status(&status)` | Read motor state, errors and measured speeds/counts. |
| `motion_service_request_stop()` | Emergency terminal stop, requiring reboot. Not a normal pause. |

The private `owner_task` in `motion_service.c` is priority5 on processor1
(RP2040 core0). Do not call it or create another motor-owner task. Producers
may call the API from tasks on either core, not directly from interrupt handlers.
Use one ordinary navigation producer; several tasks racing speed requests
would simply replace one another's latest request.

## 1. Link it into the real application

For the existing team executable, the integration change is:

```cmake
target_link_libraries(main PRIVATE buddy2_motion_service)
```

The library brings its source dependencies and include paths automatically.
Call startup from an existing/new core0 startup task and then start the IR
producer. Do not add a second `usermain()` to `main`. Its current entry point
is the kernel demonstration; linking the library alone does not start motors.
This document does not change the application's main/IR task for you.

## 2. Initialise once, using explicit settings

The following is a copyable EXAMPLE startup helper, not an existing exported
function. It uses the provisional bench settings already exercised on the
car.50mm/s and20percent duty are experimental starting settings, not loaded
tuning or motor ratings. The helper leaves LINE_LOST asserted: no drive
until your task has valid data and explicitly releases its reason.

```c
#include "motion_service.h"
#include <bsp/libbsp.h>

ER buddy3_example_start_motion(void)
{
    motion_service_config_t config = {0};
    motion_service_status_t status;
    unsigned int wheel;
    uint32_t started;
    ER result;

    config.control.sampling.wheel[0] =
        (motion_scale_t){210487U, 2541U, -1, false};
    config.control.sampling.wheel[1] =
        (motion_scale_t){210487U, 2530U, 1, false};
    config.control.sampling.max_interval_us = 100000U;
    for (wheel = 0U; wheel < 2U; wheel++)
    {
        config.control.speed[wheel] = (wheel_pid_config_t){
            .kp_q16 = 64U, .ki_q16 = 256U, .kd_q16 = 0U,
            .max_target_um_s = 50000U,
            .max_interval_us = 100000U, .max_sample_age_us = 60000U,
            .max_duty_permille = 200U, .allow_provisional_scale = true
        };
    }
    config.control.max_command_ttl_us = 200000U;
    config.control.max_control_gap_us = 100000U;
    config.control.feedback_timeout_us = 2000000U;
    config.control.reversal_brake_us = 100000U;
    config.poll_delay_ms = 10U;

    result = motion_service_start(&config); /* Core0 task, once only. */
    if (result != E_OK) { return result; }
    result = motion_service_set_hold(MOTION_HOLD_LINE_LOST, true);
    if (result != E_OK)
    {
        (void)motion_service_request_stop();
        return result;
    }
    started = in_w(TIMER_BASE + 0x28U);
    do
    {
        result = motion_service_read_status(&status);
        if (result != E_OK || status.init_error != E_OK ||
            status.control.state == MOTION_FAULT ||
            status.control.state == MOTION_STOPPED)
        {
            (void)motion_service_request_stop();
            return result != E_OK ? result :
                (status.init_error != E_OK ? status.init_error : E_OBJ);
        }
        if (status.ready) { return E_OK; }
        (void)tk_dly_tsk(10);
    } while ((uint32_t)(in_w(TIMER_BASE + 0x28U) - started) < 2000000U);
    (void)motion_service_request_stop();
    return E_TMOUT;
}
```

The service configuration is copied at startup. To tune later, edit the
application's configuration and reboot; do not change private service globals.
GP21 is already the polled terminal stop. GP20 has no automatic start role in
the service; decide application arming with the team before sending commands.

## 3. Send your decoded IR decision

From your task, using a NEW valid sensor reading after all required releases:

```c
motion_drive_request_t request = {
    .action = MOTION_DRIVE_STRAIGHT,
    .speed_um_s = 50000,       /* 50mm/s, same target for both wheels */
    .inside_um_s = 25000,      /* Steering example: inside wheel25mm/s */
    .sensed_us = ir_sample_us, /* Your variable: timestamp of this reading */
    .valid_for_us = 200000U    /* Expires200ms after that reading */
};
ER result = motion_service_submit_request(&request);
```

`ir_sample_us` is supplied by your code, not an existing Buddy2 global. Record
it when actually sampling the IR, with `in_w(TIMER_BASE + 0x28U)` (same Pico
microsecond clock). Do not stamp cached data as fresh. Renew decisions from
new readings well within200ms; a nominal20ms task delay is an example, not
a guaranteed scheduling period. Changing a desired direction means changing
`request.action`, not editing PWM:

| `request.action` | Left target | Right target |
| --- | --- | --- |
| `MOTION_DRIVE_STRAIGHT` | 50000 | 50000 |
| `MOTION_STEER_LEFT` | 25000 | 50000 |
| `MOTION_STEER_RIGHT` | 50000 | 25000 |
| `MOTION_DRIVE_HOLD` | 0 | 0 |

These are um/s:1000um/s =1mm/s. Steering is a speed difference, not a90degree
turn. Both inside/outside speeds must fit the startup limits and
0 <= inside <= outside. Buddy3 defines the real IR polarity/truth table and
barcode command meanings; the motor code does not assume which bit means black.

`E_OK` means queued, not already applied. `E_BUSY` means a normal hold blocks
the request; do not clear somebody else's hold. `E_PAR` means invalid input;
`E_TMOUT` means stale/old input. Those bad guidance paths queue a normal
brake: inspect status and use a new valid reading, not cached retries. Active
lease expiry, feedback faults and emergency stop can latch a terminal fault;
new commands cannot clear it. Never ignore errors and assume movement.

## 4. Line lost, obstacle and emergency

```c
/* Buddy3: no valid line / known stale or invalid IR data */
ER result = motion_service_set_hold(MOTION_HOLD_LINE_LOST, true);

/* Buddy3: valid line recovered */
result = motion_service_set_hold(MOTION_HOLD_LINE_LOST, false);
/* Check result, then take a NEW reading before submitting motion. */

/* Buddy5 uses its own independent reason */
result = motion_service_set_hold(MOTION_HOLD_OBSTACLE, true);
/* Release only that reason after a valid clear decision: */
result = motion_service_set_hold(MOTION_HOLD_OBSTACLE, false);

/* Terminal emergency stop, until reboot */
result = motion_service_request_stop();
```

The final hold release rejects frames sampled before the release. On line
recovery, release LINE_LOST, discard the recovery frame, then use the next
fresh reading. A remaining obstacle/operator hold still blocks motion.
`MOTION_DRIVE_HOLD` is an ordinary zero-speed request; a persistent safety
condition belongs in a hold reason. Hold bits themselves do not supervise
a dead sensor task or stale obstacle-clear state; that supervision is team
integration work. No controller/sample fault should be reset to resume.

For trusted manual wheel commands, `motion_service_set_speed(L, R, ttl_us)`
also exists. It stamps a new admission time and cannot detect an old IR
decision, so use the timestamped API above for sensor guidance.

## 5. Read speeds/status; leave internals alone

```c
motion_service_status_t status;
ER result = motion_service_read_status(&status);
/* Check result before reading the copy. */
```

- `status.ready`, `status.init_error`: initialisation. Ready can be true while held.
- `status.hold_mask`: obstacle1, line-lost2, operator4; bits can coexist.
- `status.control.state`, `.fault`, `.brake_failed`: running/idle or terminal state.
- `status.control.sample.timestamp_us`: timestamp; check freshness too.
- `status.control.sample.wheel[0]`: LEFT; `[1]`: RIGHT.
- Per wheel: `.velocity_um_s`, `.net_edges`, `.displacement_um`, `.speed_valid`,
  `.odometry_valid`, `.scale_verified`. Counts/displacement do not prove valid
  odometry; inspect flags. Scale is still provisional. The service's earlier
  idle-history issue remains open for production distance commands.
- `status.control.target_um_s[0/1]` and `.duty[0/1]`: requested target/output,
  not independent physical measurements. Historical targets may remain after stop.

No direct calls to `motor_pwm_*`, `encoder_capture_*`, `wheel_pid_*` or
`motion_control_tick()` from your IR task. No writes to private variables.
Leave those implementation files in the repository; the library needs them.
The service owns motorGP8/9/10/11, PWM slices4/5, encoder LEFT GP16/17 and
RIGHT GP4/5. This verified encoder mapping differs from Week6 labels.
Capture currently owns IO_BANK0 IRQ; another GPIO interrupt owner, such as
sonar, needs an agreed common dispatcher. Polling non-conflicting IR pins
does not require a second GPIO interrupt handler.

Ignore `buddy2_*check.c`, `route_demo.*`, `guidance_replay.*`, `speed_probe.*`,
`motor_batch.*` and `motor_pulse.*` for normal integration. Those are separate
diagnostic apps/policies, some with their own `usermain()`. Do not copy their
entry points into your application or use GP20 route-demo behaviour for IR.
The currently flashed robot still runs the completed route demo: build and
flash the integrated application before expecting real IR following.

More detail: [sensor semantics](buddy2_sensor_motion.md),
[service ownership/faults](buddy2_motion_service.md),
[roadmap](buddy2_roadmap.md). No main/IR code was changed by this handoff.
