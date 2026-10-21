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

## Table of Abbreviations

| Abbreviation | Meaning                    |
| ------------ | -------------------------- |
| lcg          | linear congruent generator |
