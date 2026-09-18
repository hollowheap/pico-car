cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
if (DEFINED ENV{CHECK_ONLY})
    set(CHECK_ONLY TRUE)
else()
    set(CHECK_ONLY FALSE)
endif()

project(pico-blink C CXX ASM)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()


# Create compile_commands.json for Clangd to analyze the project and report errors in the editor
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Check for compliance with C99
if (CHECK_ONLY)
    set(CMAKE_C_STANDARD 99)
else()
    # Barr standard enforces C99, but pico SDK requires C11
    set(CMAKE_C_STANDARD 11)
endif()
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# This must come before we compile stuff
pico_sdk_init()

# Compile main.c into an executable
add_executable(main src/main.c)
target_sources(main PRIVATE
    src/lcg32.c
)
# Link stdlib to our exe
target_link_libraries(main pico_stdlib)
# Enable USB console. Access with `sudo screen /dev/ttyACM0` (or ttyACM1, ttyACM2, etc.)
pico_enable_stdio_usb(main 1)
# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)
# Extra compiler flags
target_compile_options(main PRIVATE
    -Werror
    -Wall
    -Wextra

    -Wparentheses           # 8.6 suspicious assignment
    -Wfloat-equal           # 5.4b.iv no float equality tests
    -Wswitch-default        # 8.3b default required
    -Wimplicit-fallthrough  # 8.3c commented fall-throughs
    -Wshadow
    -Wvla
)
