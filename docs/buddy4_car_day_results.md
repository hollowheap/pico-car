# Buddy 4: Car Day Results

**Module:** Buddy 4 (terrain detection: IMU, hump detection and height, motion classes)
**Hardware:** Pico W on Cytron Robo Pico, GY-511 (LSM303DLHC) on Grove 2, I2C1 (`GP2` SDA, `GP3` SCL), 400 kHz
**RTOS:** micro T-Kernel 3.0 SMP (`mtk3smp-rp2040`), 50 Hz task
**Logs:** [`logs/`](logs/) (day 1 and day 2 S6) and `pico-car/logs/` (all, plus `lift_left.log`)
**Plan:** [`car_day_plan.md`](car_day_plan.md)

Car day 1 was 2026-11-04 and car day 2 was 2026-11-05. An earlier version of
this file overstated several day-1 results. Section 4 lists what changed and
why; section 1 is the current status.

---

## 1. Status

| Item | Result | Status | Evidence |
|---|---|---|---|
| I2C init, bus (S2) | `init=0`, `nacks=0`, `timeouts=0`, `fails=0` | PASS | `s3_flat.log` |
| Up axis (S3) | Z, +1. Rest \|a\| 1.031-1.037 g (day 1), 1.054-1.057 g (day 2) | PASS | `s3_flat.log`, `s6_static.log` |
| Rest noise (S3) | sigma 3.5 / 4.8 / 4.4 mg on X / Y / Z | PASS | `s3_flat.log` |
| Forward axis (S4) | Y, sign -1 (nose up makes ay negative) | PASS | `s4_axes.log` |
| Side axis (S4, rechecked day 2) | X. **Left side up makes ax positive** (+375 mg, 22 deg). The day-1 "left side up" step was really the right side. | PASS | `lift_left.log` |
| Task timer | Day 1 from S5 on: timer ran 42.6x slow. Day 2: `timer_5s` 5,019,947-5,020,051 (target 5,000,000) | FIXED | `s6_static.log`, `s6_rolling.log` |
| Tilt accuracy (S5) | Caster raised on 7 / 14 / 21 mm stacks, reference asin(h / 82 mm) = 4.90 / 9.83 / 14.84 deg. Measured 4.74 / 10.03 / 14.17 deg (errors -0.16 / +0.20 / -0.67 deg). Back to flat within 0.4 deg. The v2 run (hand-set "5 / 10 / 15 deg") had no reliable reference and is not used. | PASS | `s5_tilt_v3.log` |
| Calibration | Day 2 rest \|a\| 1.054-1.057 g failed the 0.05 g gate, so `cal` stayed 0. Gate widened to 0.10 g; replay calibrates in 1 s. | FIXED | `s6_static.log` |
| Hump, 10 mm under right wheel (S6) | On the car, tilt mode, pushed by hand: 6 passes, 11 / 12 / 11 / 11 / 10 / 11 mm (mean 11.0 mm), `height_valid=1`, no false humps. Replay of `s6_rolling.log`: 11.5 / 9.1 / 8.7 mm. Powered run aborted (section 3). | PASS (hand-pushed) | `hump_test_v1.log`, `s6_rolling.log` |
| Magnetometer field (S7) | \|m\| 0.341 G, sigma 0.0014 G | PASS | `s7_mag_still.log` |
| Yaw sign (S7) | `mag_yaw_sign = 1` (left turn positive). Day-1 yaw rate values were 42.6x too large (timer). | PASS (sign only) | `s7_mag_left.log` |
| Hard iron (S7) | X +0.227 G, Y -0.008 G. Z not measurable from a flat circle; firmware uses 0. | PASS (X, Y) | `s7_mag_circle.log` |
| Impact (S8) | Day 2: 3 moderate taps. The hard one (632 mg on the forward axis) gave `IMPACT` for 7 rows through the 0.5 g forward-residual check; the two lighter ones (about 220-250 mg) were ignored, as designed. No false humps. A real collision at 0.15 m/s stopping in 20 ms is about 0.78 g; braking is about 0.15 g, so 0.5 g is kept. | PASS | `s8_impact_v2.log` |
| Unplug (S9) | Retest with the fix: unplugged while on the hump. State went to NO_DATA, I2C fault set, `height_valid=0`. Data returned 5 s after replug without a reboot; `inits` 1 -> 6, `init=0`. The interrupted hump closed with `height_valid=0`. | PASS | `unplug_test_v1.log` (day-1 bug: `s9_unplug.log`) |
| Soak (S10) | `overruns=0`, `fails=0` over 30,310 samples (10.1 min). Real `max_work_us` about 16,900 (logged 396 x 42.6). | PASS, tight | `s10_soak.log` |
| Work time with logging | Day 2 `max_work_us` 17,225-18,532 of 20,000 with `log_every=1` | WATCH | day 2 logs |

---

## 2. Settings in the firmware

These are built into `src/buddy4_imu_check.c` (the day-1 file listed them,
but the day-1 firmware did not contain them):

```c
cfg.hump.fwd_axis      = HUMP_AXIS_Y;   /* S4 */
cfg.hump.fwd_sign      = -1;
cfg.hump.mode          = HUMP_MODE_TILT; /* course humps are under one wheel */
cfg.hump.enter_deg     = 3.0f;           /* 10 mm gives 6.2 deg */
cfg.hump.exit_band_deg = 1.5f;
cfg.hump.enter_samples = 10U;            /* 0.2 s */
cfg.motion.mag_yaw_sign    = 1;          /* S7 */
cfg.motion.mag_offset_g[0] = 0.227f;
cfg.motion.mag_offset_g[1] = -0.008f;
cfg.motion.mag_offset_g[2] = 0.0f;
```

Library defaults changed in `hump_detect.c`: `cal_max_dev_g` 0.10 g,
`lever_m` 0.0925 m, `tilt_avg_len` 10, `entry_max_dev_g` 0.05 g,
`require_motion` false (set it to true in the combined Buddy 2 + Buddy 4 app).

The timer fix is `b4_timer_tick_init()` in `src/b4_i2c_rp2040.c`: it sets
`WATCHDOG_TICK` (0x4005802C) to enable with 12 cycles, because clk_ref is the
12 MHz crystal. Other buddies reading the RP2040 timer need the same fix.

---

## 3. Car day 2: single-wheel hump

### Geometry

Rear-wheel drive with a front caster: track W = 112 mm, rear axle to caster
L = 82 mm. With the right rear wheel on the hump the car pivots about the
line from the left wheel to the caster. The right wheel is
W·L / sqrt(L² + (W/2)²) = **92.5 mm** from that line, so a 10 mm hump tilts
the car asin(10 / 92.5) = **6.2 deg**, split 1.46 : 1 between roll and pitch
(pitch alone is about 3.5 deg, which the 6 deg pitch threshold never sees).

Measured in `s6_static.log`: change from flat of ax -86 to -106 mg and ay +61
to +76 mg (ratio 1.39-1.42), total tilt 5.8-7.0 deg in the steady holds.
The geometry matches.

Height = 92.5 mm x sin(peak tilt). Tilt is the angle between the current
gravity vector and the one stored at calibration, so it does not depend on
the mounting angle.

### Logs (`timer_5s`, `overruns=0`, `fails=0` in all three)

| Log | What was done | Replay with firmware settings (`tools/b4_replay.c`) |
|---|---|---|
| `s6_static.log` | Wheel placed on the hump by hand and held, 4 times | Detected 4 times (the 4th is still open when the log ends). Peaks of the 3 finished ones 16.5-19.5 mm because placing the wheel by hand gives 0.3 s spikes of 10-12 deg. Steady holds 5.8-7.0 deg = 9.3-11.3 mm. |
| `s6_rolling.log` | Car pushed over the hump by hand, 3 passes of about 5 s | 3 humps: **11.5, 9.1, 8.7 mm**. No other detections. |
| `lift_left.log` | Left side lifted by hand (axis check) | 1 hump, 37.9 mm. Rejected once `require_motion` is on with Buddy 2 running. |

Without the 0.2 s moving average and 0.2 s entry debounce, the first rolling
pass read 12.2 mm and three extra humps of about 6 mm appeared from pushing
the car.

### Car day 2, on-device run (`hump_test_v1.log`)

Firmware `buddy4_imu_check` with the section 2 settings, car pushed over the
hump by hand 6 times. `overruns=0`, `max_work_us` 17,590.

| Pass | Time (s) | Duration (s) | `hump_peak` (mm) | `height_valid` |
|---|---|---|---|---|
| 1 | 79.7-86.0 | 6.3 | 11 | 1 |
| 2 | 101.7-106.8 | 5.1 | 12 | 1 |
| 3 | 119.7-123.4 | 3.7 | 11 | 1 |
| 4 | 131.5-135.5 | 4.0 | 11 | 1 |
| 5 | 144.0-147.3 | 3.3 | 10 | 1 |
| 6 | 154.6-157.8 | 3.2 | 11 | 1 |

`run_peak` = 13 mm in the first visible end message means one more pass
happened in a 73 s stretch the serial capture lost (the firmware counters
show no gap on the Pico).

### S6 conclusion

**S6 (hump, 10 mm under right wheel): PASS (hand-pushed).** 6 passes in
`hump_test_v1.log` read 10-12 mm (mean 11.0 mm); replay of `s6_rolling.log`
gave 11.5 / 9.1 / 8.7 mm; no false humps. Height is derived from tilt
(92.5 mm lever), so it does not depend on how the car is propelled, and tilt
accuracy was verified separately in S5 v3 (within 0.7 deg).

A powered run was attempted but aborted. The test app
`src/buddy4/buddy4_drive_check.c` drives straight through Buddy 2's motion
controller and feeds its wheel feedback to Buddy 4 (`require_motion` on).
The controller latched a fault and stopped the wheels before 5 passes; the
cause is not yet identified. This is a Buddy 2 + Buddy 4 integration issue,
not a Buddy 4 detection failure.

Not yet verified:

- Accelerometer noise from motor vibration. The 0.2 s moving average and
  0.2 s entry debounce are meant to absorb it, but it has not been measured
  with the motors on.
- The `require_motion` speed gate with live wheel data.

Both move to the integrated Buddy 2 + Buddy 4 run.

---

## 4. Corrections to the day-1 version of this file

| Day-1 claim | Correction |
|---|---|
| "All ten scenarios successfully executed and validated" | S5 not measured, S6 and S8 partial, S9 found a replug bug. |
| Settings "applied" for a final rebuild | They were not in the day-1 firmware. Now built in (section 2). |
| S5: 5 / 10 / 15 deg blocks read 6.15 / 10.96 / 22.65 deg | The angles were hand estimates, not blocks. Redone on day 2 with measured stacks (`s5_tilt_v3.log`): within 0.7 deg. |
| S6: `enter_deg = 2.5` captures every pass | Not adopted: 2.5 deg on pitch triggers on handling and taps. Replaced by tilt mode (section 3). |
| S6: L about 130 mm, roll about 4.5 deg | Measured L = 82 mm, W = 112 mm. Tilt 6.2 deg. |
| S4: left side up gives ax -480 mg | That was the right side. Left up gives ax +375 mg (`lift_left.log`). |
| S7: Z hard-iron offset +0.047 G | A flat circle cannot measure Z. Firmware uses 0. |
| S8: PASS | Drop detected (`IMPACT` for 15 samples); taps (about 160 mg) were not. PARTIAL. Replay in tilt mode: the drop gives a false 43.8 mm hump with the shock gate off and none with it on. |
| S9: PASS, re-init only a recommendation | After replug the chip read zero and the task never re-initialised it. Fixed: all-zero reads are rejected, and after 5 bad reads in a row the driver clears `ready` so the task re-initialises every 1 s. I2C counters now accumulate across re-inits. Retested on day 2: PASS (`unplug_test_v1.log`). |
| S10: `max_work_us = 396`, CPU load under 2 % | The timer ran 42.6x slow, so the real figure is about 16.9 ms of 20 ms. Most of it is probably the CSV logging; retest with `log_every=10`. |
| Durations from S5 on (for example S10 14.2 s by `t_us`) | Sample counts are right (50 Hz from the kernel tick); `t_us` and `max_work_us` were 42.6x small. |

---

## 5. Next tests

Done on car day 2: `cal` = 1 at boot (0.98 s), S5 with measured stacks, S6
hand-pushed, S8 taps, S9 retest. Standalone Buddy 4 car testing is closed.

Not run, by decision:

- S7 yaw-rate magnitude (sign only is verified).
- S10 with `log_every=10` (work time stays at about 17.6 ms of 20 ms with
  full logging).

Moved to the end-of-project integration with Buddy 2:

- S6 under motor power with `require_motion` on. In `buddy4_drive_check`,
  the `B4H,motion_fault=<n>` line names the motion fault that stopped the
  wheels.

Serial capture gaps on day 2 (73 s, 52 s, 188 s, 700 s, with no gap in the
Pico counters) are most likely the laptop USB adapter.
