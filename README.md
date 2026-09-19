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

When creating a new file, use the templates in `templates/` for consistency.

## Table of Abbreviations

| Abbreviation | Meaning                    |
| ------------ | -------------------------- |
| lcg          | linear congruent generator |
