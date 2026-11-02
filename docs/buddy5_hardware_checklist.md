# Buddy 5 hardware test checklist

Build with the normal `./configure.sh` and `./build.sh`, then flash
`build/obstacle_bench.uf2` (hold BOOTSEL, plug in, copy the file). Run these
in order with the USB serial monitor open. Write the result and date next to
each step: these are your evidence for the tribunal.

- **Wheels:** keep them off the floor (on a block) until H6.
- **Tool:** a ruler is enough for most steps; a multimeter is needed for H1.

| # | Test | How | Pass when | Result |
|---|---|---|---|---|
| H1 | ECHO voltage is safe | Meter the Grove 1 VCC pin. Then power the sensor from Grove 1 with the ECHO wire **not** connected to GP1, and meter ECHO against GND while waving a hand in front. | VCC ≈ 3.3 V and ECHO high ≤ 3.4 V (HC-SR04+ or HC-SR04P). Only then connect ECHO to GP1. If ECHO reads about 5 V, do not connect it; fit the divider in the design doc → Wiring. | |
| H2a | Servo centre | After boot the servo should point straight ahead. If not, adjust `B5_SERVO_CENTRE_US` in steps of 10 µs. | Sensor faces straight ahead | |
| H2b | Servo direction | Watch the start of a scan (GP20 to SCAN_REQ after an OBSTACLE). Slot 0 is −60° = **right**. | First move goes right. If it goes left, set `B5_SERVO_DIR` to −1. | |
| H2c | Servo scale | Protractor under the sensor. Compare the ±60° slots with the real angle and adjust `B5_SERVO_US_PER_10DEG`. | ±60° within ±3° | |
| H2d | Settle on a low battery | Repeat a scan on a LiPo at ~3.5 V. Then double `B5_SETTLE_MS_PER_DEG`, rebuild and scan again. | The two profiles match within 15 mm | |
| H3 | Detection distance | Status lines show distance. Slide a flat box toward the sensor. | OBSTACLE event at 250 ± 15 mm | |
| H4 | 100 mm box at 200 mm | Box centred 200 mm ahead, AUTO-RUN on. Keep everything else (wires, hands, walls) more than 600 mm away. Read the printed profile and SCAN_RESULT. | Region is slots 9–15; fine slots 7, 8, 16, 17 measured; width 75–125 mm; scan < 8 s | |
| H4b | Beam half-angle | Servo straight ahead. Power bank 300 mm ahead, flat face to the sensor; slide it sideways until the status line jumps to the background or OPEN. Measure centre line → near edge, both sides. | Done: 50 mm left, 65 mm right → 9.5° (`B5_BEAM_HALF_DECI` 95). Re-run H4. | 50 / 65 mm |
| H5 | Full handshake | AUTO-RUN: box ahead → watch the log. | STATE goes MONITOR → WAIT_STOP → SCANNING → PLAN → BYPASS → REACQUIRE → MONITOR, with STEER lines | |
| H5b | Time limits | AUTO-RUN off (GP20 to "AUTO-RUN on/off"). Trigger an OBSTACLE and don't send SCAN_REQ. | STOPPED cause HANDOVER after 1 s; FAULT event; "EMERGENCY STOP called" | |
| H6 | Emergency stop during bypass | During BYPASS/REACQUIRE, put a hand within 80 mm of the sensor. | STOPPED cause COLLISION within 2 pings (~120 ms); STEER HOLD; stop link called | |
| H7a | ECHO unplugged | In MONITOR, unplug the ECHO wire. | STOPPED cause NO_ECHO after 5 pings (~300 ms). RESET is refused until replugged; RESET + START then works | |
| H7b | ECHO stuck high | Tie ECHO to 3V3 through 1 kΩ. | STOPPED cause ECHO_STUCK after 3 pings | |
| H8 | Open room | Point the sensor at open space for 30 s. | Only OPEN or far VALID, no false STUCK. If STUCK appears, the module holds ECHO high longer than 60 ms on no-echo: report it. | |
| H9a | Phase 2, wheels up | Flash `main.uf2`, wheels off the floor. Let the wheels spin, then put a hand 20 cm in front of the sensor. | Wheels stop; `[b5-nav] OBSTACLE`; servo sweeps; after you remove your hand the wheels turn again (SCAN_RESULT CLEAR → ACCEPT) | |
| H9 | Hug on the car (Phase 2) | Flash `main.uf2`. Box on the line, car following the line towards it. Nothing else within 600 mm of the box. After the run keep the car on, plug in USB and open the serial monitor to print the flight log. | Car stops, scans, pivots until the way is clear and the planned angle is reached, drives out, zig-zags along the obstacle, turns back at about 30°, line following resumes at the first line contact. Takes about 20–30 s. Flight log: no `MOTION ... fault=` other than NONE, no FAULT lines. | |

Settings named in this list (`B5_SERVO_CENTRE_US`, `B5_SERVO_DIR`,
`B5_SERVO_US_PER_10DEG`, `B5_SETTLE_MS_PER_DEG`, `B5_BEAM_HALF_DECI`) are in
`src/obstacle/b5_cfg.h`. After changing one, run `./build.sh` and flash
again. See the [design doc](buddy5_obstacle.md) for what each state and cause
means.
