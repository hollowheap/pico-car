## Building

Run these commands:

```sh
nix develop     # Enter devshell with build tools and pico SDK installed
./configure.sh  # Configure CMake
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

Unfortunately, all versions of the pico SDK require ISO C11 to build due to
their use of `static_assert`. To automatically check for C99 compliance of
our own code, a patched version of the pico SDK is available under
`lib/pico-sdk/` and the `check.sh` script will attempt to build our code with a
C99 compiler.

When creating a new file, use the templates in `templates/` for consistency.

## Table of Abbreviations

| Abbreviation | Meaning                    |
| ------------ | -------------------------- |
| lcg          | linear congruent generator |
