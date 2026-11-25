# Measurement benches

Some numbers in the design are guesses until the car gives them. A bench is a small firmware you flash, run, and copy the console log from. Each bench prints lines that start with a tag in capitals. `tools/bench_summary.py` reads them.

Build all of them with `nix develop -c ./build.sh`. Flash a file by holding BOOTSEL, plugging in the USB cable, and copying the `.uf2` file to the drive.

| File | What it measures |
|---|---|
| `build/car_bench.uf2` | Encoders, wheel response, line sensors, both line laws, the lift cue |
| `build/wdt_check.uf2` | Test W1: the H-bridge inputs across a watchdog reset |
| `build/physics_check.uf2` | The four checks of the FLAT hump state (guided) |
| `build/obstacle_bench.uf2` | The sonar and the servo (see `docs/obstacle_hardware_checklist.md`), plus SERVO CHECK |

Open the USB serial monitor, copy the whole console into a file, and give the file to the matching tool:

```
python3 tools/bench_summary.py car_bench.log
python3 tools/physics_log_summary.py physics.log --checks
```

## car_bench

Buttons: GP20 chooses the next item (the console prints it). GP21 runs it after a 3 s countdown. GP20 during a run stops it. The bench starts only the motors and the line sensors. The obstacle task and the supervisor do not run, so nothing else holds the car.

| Item | Setup | What it prints | What it answers |
|---|---|---|---|
| ENC | Wheels off the floor | `ENC,left,...` and `ENC,right,...` with a verdict | Are the encoder plugs crossed? Does an encoder count backward? Is one dead? |
| STEP | Car on the floor, 25 cm free | 200 rows `STEP,t_ms,left,right,left_duty,right_duty` and `STEP_RESULT` | The wheel lag the line-follow model uses (Q72) |
| SENS | Car on the line | `SENS,t_ms,left,right,code` on every change | Where each sensor switches while you slide the car by hand. Measure the offset with a ruler. |
| LINEA | Car at the start of 1.5 m of straight line | `LINE_RESULT,law=legacy,...` | The baseline: sensor changes per second and inner-wheel stops per second of the old law |
| LINEB | Car back at the same start | `LINE_RESULT,law=cascade,...` | The same numbers for the new law |
| LIFT | 1.5 m free. Lift the car when the console says so | `DRV,...` rows and `DRV_RESULT` | The duty on the floor and in the air, and when the wheels-run-free cue sets |

Run LINEA and LINEB on the same straight line with the sensors in their final place. Measure the distance from the axle to the sensors with a ruler and put it in Q71.

## wdt_check (test W1)

1. Wheels off the floor.
2. Flash `wdt_check.uf2`. The console prints `W1_PINS,GP8=..` with the level of the four motor pins at start.
3. Press GP21. Both wheels run for 2 s. The bench starts the 1 s watchdog and hangs.
4. Watch the wheels. The board resets after 1 s. After the reset the console says that the boot follows a watchdog reset, and the motor pins are held low.

Pass: the wheels stop at the reset and stay stopped. Write the result in Q61. If they keep turning, leave `PICO_CAR_WATCHDOG` off.

## physics_check (the FLAT checks)

Flash it and keep the console open. It prints `PHYS_G` lines that tell you what to do and when: keep still, lift the nose to about 10 degrees, set level, lift the left side, set level, push the car over a ramp. Keep the car still for the first 4 s. Save the console and run `tools/physics_log_summary.py FILE --checks`. It says which of checks 1 to 3 fails and which board axis points forward.

Check 4 (pitch while driving against pitch standing still) needs the wheels. Run the car with `main.uf2` on flat ground, watch `pitch` on the Live screen at rest and while driving a straight line. They must match within 1 degree.

## obstacle_bench: SERVO CHECK

Put a flat, square wall 150 to 800 mm in front of the sonar. Run a scan (AUTO-RUN, or SCAN_REQ), then choose SERVO CHECK. It prints the angle the servo really reached at the commanded +30 and -30 degree slots, and says OK, FLAT, SCALE or ASYMMETRIC. The scan profile at 5 degree steps also answers the H4c measurement of Q56: the angle where a flat face stops returning.
