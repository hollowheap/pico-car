## Building

Run these commands:

```sh
nix develop     # Enter devshell with build tools and pico SDK installed
./configure.sh  # Init submodules + configure CMake
./build.sh      # Compile and link
```

The final binary is hardlinked inside the base directory as `main.uf2`.
Disassembly and other files can be found in the `build/` folder.

When developing, `configure.sh` should be run before the first build or
whenever `CMakeLists.txt` changes. It is unnecessary to run it otherwise;
Running `build.sh` is enough.

## Developing

All C and C++ code in this repository must comply with the
[BARR-C:2018 standard](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).

When creating a new file, use the templates in `templates/` for consistency.

## Buddy 2 encoder calculations

`src/encoder_math.c` provides checked distance and speed conversions and a
wrapping-counter difference. It does not configure hardware or replace the
current RTOS demo. See [encoder calibration and tests](docs/encoder_math.md)
for units, numerical limits, remaining integration work and host-test commands.

The opt-in `buddy2_encoder_check` target reads quadrature encoders on GP4/5
and GP16/17 while holding the motor inputs low. See the
[bring-up steps and remaining motion work](docs/buddy2_bringup.md) before using
it on hardware. It is separate from `main.uf2` and does not drive the car.

The optional `buddy2_motion_check` target adds quiet hand-turn speed and
signed-displacement sampling with explicit provisional calibration and validity
flags. See [motion sampling and its limits](docs/buddy2_motion_sampling.md).
It does not drive the motors or replace the team demo.

## Buddy 2 motor control and integration

**Buddy3 taking over: [read this quickstart first](docs/buddy3_motion_quickstart.md).**
It lists the one public header, startup, copyable settings, calls from your IR
task and the diagnostic files you can ignore.

The optional motor service owns encoder feedback, paired PWM and wheel-speed
PI control. Standalone wheels-up motor, speed and synthetic steering/hold
checks have passed. Real IR/obstacle integration and loaded driving remain.
The team `main` is not automatically switched to these diagnostics.

- [Current roadmap and remaining work](docs/buddy2_roadmap.md)
- [Sensor-to-motion API and hold rules](docs/buddy2_sensor_motion.md)
- [Motor-owner service and fault handling](docs/buddy2_motion_service.md)
- [First PI speed trial and limits](docs/buddy2_speed_check.md)
- [Steering/overlapping-hold test and physical result](docs/buddy2_guidance_check.md)
- [Optional50cm / right3wheel-revolutions /10cm demo](docs/buddy2_route_check.md)
- [Verified hardware mapping and bring-up history](docs/buddy2_bringup.md)

## Table of Abbreviations

| Abbreviation | Meaning                    |
| ------------ | -------------------------- |
| lcg          | linear congruent generator |
