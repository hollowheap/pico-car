# Obstacle hardware tests

H1–H8 use the Buddy 5 bench firmware, `build/obstacle_bench.uf2`, with USB
serial open. For an already configured checkout, `./build.sh` produces the
bench and installed images. Record the image, result and date for each trial.

- Support the chassis with the wheels clear of the floor for the bench tests.
- **Tool:** a ruler is enough for most steps; a multimeter is needed for H1.
- **Values:** the numbers in this list come from `obstacle_config.h`. The
  `statechart.c` lists the states and the faults.

| # | Test | How | Pass when | Result |
|---|---|---|---|---|
| H1 | ECHO voltage is safe | Meter the Grove 1 VCC pin. Then power the sensor from Grove 1 with the ECHO wire **not** connected to GP1, and meter ECHO against GND while waving a hand in front. | VCC ≈ 3.3 V and ECHO high ≤ 3.4 V (HC-SR04+ or HC-SR04P). Only then connect ECHO to GP1. If ECHO reads about 5 V, do not connect it; fit the divider in the [wiring section](obstacle.md#wiring). | |
| H2a | Servo centre | After boot the servo should point straight ahead. If not, adjust `OBSTACLE_SERVO_CENTRE_US` in steps of 10 µs. | Sensor faces straight ahead | |
| H2b | Servo direction | Watch the start of the centered legacy scan after SCAN_REQ. Its first slot is 14 of 0–36, at −20°. | The first move goes right. | |
| H2c | Servo scale | Use a protractor at the ±15° test points and check clearance at the ±20° travel limits. | The ±15° positions are within ±3° and the sensor clears the mount at both limits. | |
| H2d | Settle on a low battery | Repeat a scan on a LiPo at ~3.5 V. Then double `OBSTACLE_SETTLE_MS_PER_DEG`, rebuild and scan again. | The two profiles match within 15 mm | |
| H3 | Detection distance | Status lines show distance and class. Slide a flat box toward the sensor. | OBSTACLE event at 250 ± 15 mm, after 2 hits among 3 pings | |
| H3b | One outlier | Hold a box at 200 mm. Pass a hand through the beam for one ping only (about 60 ms). | The OBSTACLE event still comes from the box. A single hand ping alone does not raise it. | |
| H4 | 100 mm box at 200 mm | Box centred 200 mm ahead, AUTO-RUN on. Keep everything else (wires, hands, walls) more than 600 mm away. Read the printed profile and SCAN_RESULT. | Hits are slots 16 to 20 (about ±10°); slots 15 and 21 are the first misses; width 75–125 mm; scan < 8 s | |
| H4b | Beam half-angle | Servo straight ahead. Power bank 300 mm ahead, flat face to the sensor; slide it sideways until the status line jumps to the background or OPEN. Measure centre line → near edge, both sides. | Done: 50 mm left, 65 mm right → 9.5° (`OBSTACLE_BEAM_HALF_DECI` 95). Re-run H4. | 50 / 65 mm |
| H4c | Edge and incidence | Board 230 mm ahead, face square to the sensor. Turn the servo in 5° steps and note where the reading is lost. Then slide the board edge across the beam centre. | The angle where the face stops returning (the simulation uses 35°), and whether the cone sees an edge that is off the beam centre. Write both numbers here. | |
| H5 | Full handshake | AUTO-RUN: box ahead → watch the log. | STATE goes MONITOR → WAIT_STOP → SCANNING → PLAN → BYPASS → REACQUIRE → MONITOR, with STEER lines | |
| H5b | Time limits | AUTO-RUN off (GP20 to "AUTO-RUN on/off"). Trigger an OBSTACLE and don't send SCAN_REQ. | STOPPED cause HANDOVER after 0.5 s; FAULT event; "EMERGENCY STOP called" | |
| H5c | Retry policy | After H5b, wait 10 s. Then trigger a COLLISION (H6) and wait 10 s. | After HANDOVER the bench sends RESET + START by itself after 3 s. After COLLISION it does not: STATE stays STOPPED until you send RESET and START. | |
| H6 | Emergency stop during bypass | During BYPASS/REACQUIRE, put a hand within 80 mm of the sensor. | STOPPED cause COLLISION within 2 close pings among 3 (about 120–180 ms); STEER HOLD; stop link called | |
| H7a | ECHO unplugged | In MONITOR, unplug the ECHO wire. | STOPPED cause NO_ECHO after 3 pings (about 180 ms). Fault bit 8 (`sonar_no_echo`) shows in telemetry. RESET + START works only after you replug, because the fault returns after 3 pings. | |
| H7b | ECHO stuck high | Tie ECHO to 3V3 through 1 kΩ. | STOPPED cause ECHO_STUCK after 3 pings. Fault bit 10 (`echo_line`) shows in telemetry. | |
| H7c | ECHO noise | Tap the ECHO wire against GND for a few ms, about once per ping, for 10 pings. | GLITCH counts rise in the `d` packet. 5 GLITCH pings in a row stop the car with cause ECHO_NOISE and fault bit 10. If this is hard to do by hand, skip it: the host test `obstacle_statechart` covers the counting. | |
| H7d | Blind zone | Hold a flat card 30–60 mm in front of the sensor. | Class TOO_CLOSE (`echo_too_close` in the `d` packet). 2 of 3 pings stop the car with cause COLLISION. If the sensor reports something else under 60 mm, note the class and the distance. | |
| H7e | TRIG readback (P4) | In MONITOR, watch the class counts in the `d` packet. Then pull the TRIG wire from GP0. | With TRIG connected the pings are VALID or OPEN. With TRIG unplugged or shorted to ground they become `NO_ECHO` within 3 pings and the car stops with cause `NO_ECHO`. If good pings read `NO_ECHO` at once, the pad does not read back: set `OBSTACLE_TRIG_READBACK` to 0. | |
| H8 | Open room | Point the sensor at open space for 30 s. | Only OPEN or far VALID. No STUCK, NO_ECHO or GLITCH in the `d` packet. If STUCK appears, the module holds ECHO high longer than 60 ms on no-echo: report it. | |
| H9a | Fixed IR floor qualification | With the chassis supported at its installed ride height, pass course floor, tape and an exposed edge under each digital IR sensor. Record its bit, surface and edge position across repeated passes in course lighting. | Record which observations reliably identify support, including any ambiguity. For each proposed wheel or caster path, measure detection lead and compare it with sensing delay and braking travel. | |
| H9 | Installed obstacle hold | On supported track with braking room, select **Stop at obstacles** in Start and approach a stationary box. Leave it in place through the survey, then lift the box away from the line. Record sonar class and range, both wheel targets, valid wheel speeds, and any independent holds. | A fresh centered VALID at ≤100 mm from the sonar face or a centered TOO_CLOSE commands both targets to zero. The wheels come to rest before contact and stay at zero during at most two stationary servo sweeps. An armed run can continue along the original tape after three consecutive fresh centered clear returns, with motion ready and all independent holds released. | |

For the continuous-floor trial, use normal Start and leave the box in place.
Record both encoder speeds, wheel targets, obstacle state and odometer from
contact through the first side leg, subsequent scans and optical rejoin.
During a planned pause, measured wheel motion should resume within three
seconds. Identify a safety hold or a gap in telemetry separately. A total
duration over 30 s calls for inspection of the path. [Pilot behavior](obstacle.md#hug-pilot-path)

Settings named in this list (`OBSTACLE_SERVO_CENTRE_US`, `OBSTACLE_SERVO_DIR`,
`OBSTACLE_SERVO_US_PER_10DEG`, `OBSTACLE_SETTLE_MS_PER_DEG`, `OBSTACLE_BEAM_HALF_DECI`) are in
`src/navigation/obstacle/obstacle_config.h`. After changing one, run `./build.sh` and flash
again. See the [design doc](obstacle.md) for what each state and cause
means.
