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

Build notes:

- `configure.sh` initializes submodules before configuring CMake.
- `CMakeLists.txt` pins `PICO_SDK_PATH` to the vendored `lib/pico-sdk` so a devshell environment variable cannot override it.
- Kernel translation units build with `-std=gnu11`; only `src/` is held to BARR-C C99 with `-Werror`.
- The kernel linker script asserts `.boot2` is exactly 256 bytes. Link with `-Wl,--undefined=boot2` so the member is not dropped from the archive.
- Hand-rolled firmware targets need `-ffunction-sections` and `-fdata-sections`; otherwise `--gc-sections` cannot drop unused SDK code.

## Developing

All C and C++ code in this repository must comply with the
[BARR-C:2018 standard](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf).

When creating a new file, use the templates in `templates/` for consistency.

## Layout

Public subsystem headers live in `include/<subsystem>/`; implementations live
in `src/<subsystem>/`. `src/main.c` is the composition root: it creates every
task and starts one startup task that initialises the shared hardware.

| Path | Contents |
|---|---|
| `include/navigation/obstacle.h` | Buddy 5 facade: commands, events, telemetry snapshot, line-task glue |
| `include/physics/physics.h` | Buddy 4 facade: telemetry snapshot |
| `src/navigation/obstacle/` | Buddy 5 scan, planner, statechart, comms, flight log |
| `src/physics/` | Buddy 4 IMU, hump detection, pipeline, task |
| `src/motion/` | Motor owner, encoders, PID |
| `src/ir/`, `src/sys/` | Line sensors and the shared GPIO IRQ dispatcher |

The opt-in bring-up targets build with `make buddy4_imu_check` or
`make obstacle_bench` inside `build/`.

## Tests

Host tests build out of tree:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build
```

## Table of Abbreviations

| Abbreviation | Meaning                    |
| ------------ | -------------------------- |
| lcg          | linear congruent generator |
