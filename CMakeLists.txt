cmake_minimum_required(VERSION 3.12)

set(REQUIRED_PICO_SDK_VERSION "2.3.1")
set(REQUIRED_MTK3_VERSION "1.00.00.B5-pico_rp2040")

if (NOT DEFINED ENV{PICO_SDK_PATH} AND NOT DEFINED PICO_SDK_PATH)
    set(ENV{PICO_SDK_PATH} "${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk")
endif()

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

if (DEFINED ENV{CHECK_ONLY})
    set(CHECK_ONLY TRUE)
else()
    set(CHECK_ONLY FALSE)
endif()

project(pico-blink C CXX ASM)

# pico-sdk version check
if (NOT PICO_SDK_VERSION_STRING VERSION_EQUAL "${REQUIRED_PICO_SDK_VERSION}")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version mismatch: required ${REQUIRED_PICO_SDK_VERSION}, found ${PICO_SDK_VERSION_STRING}")
endif()

# mtk3-bsp version check

execute_process(
    COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR}/lib/mtk3-bsp describe --tags --always
    OUTPUT_VARIABLE MTK3_GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

string(REGEX  REPLACE "^v" "" MTK3_VERSION_STRING "${MTK3_GIT_TAG}")

if (NOT MTK3_VERSION_STRING)
    set (MTK3_SYSDEF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/mtk3-bsp/include/tk/sys/profile.h")
    if (EXISTS "${MTK3_SYSDEF_PATH}")
        file(STRINGS "${MTK3_SYSDEF_PATH}" MTK3_SPEC_LINE REGEX "^#define[ \t]+TK_SPECVER")
        string(REGEX MATCH "0x[0-9a-fA-F]+" MTK3_SPEC_HEX "${MTK3_SPEC_LINE}")
        set(MTK3_VERSION_STRING "${MTK3_SPEC_HEX}")
    else()
        message(FATAL_ERROR "mtk3-bsp missing. Run 'git submodule update --init --recursive'.")
    endif()
endif()

if (NOT MTK3_VERSION_STRING VERSION_EQUAL "${REQUIRED_MTK3_VERSION}" AND NOT MTK3_VERSION_STRING STREQUAL "${REQUIRED_MTK3_VERSION}")
    message(FATAL_ERROR "micro T-Kernel version mismatch: required ${REQUIRED_MTK3_VERSION}, found: ${MTK3_VERSION_STRING}")
endif()

# Create compile_commands.json for Clangd to analyze the project and report errors in the editor
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Barr standard enforces C99
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# mtk3-bsp init
add_library(mtk3 STATIC)

target_compile_definitions(mtk3 PUBLIC _PICO_RP2040_)

target_include_directories(mtk3 PUBLIC
    lib/mtk3-bsp/include
    lib/mtk3-bsp/config
  PRIVATE
    lib/mtk3-bsp/kernel/knlinc
    lib/mtk3-bsp/kernel/tkernel
    lib/mtk3-bsp/kernel/sysdepend/cpu/rp2040
    lib/mtk3-bsp/kernel/sysdepend/pico_rp2040
    lib/mtk3-bsp/device
)

target_sources(mtk3 PRIVATE
    lib/mtk3-bsp/kernel/tkernel/sysmgr.h
    lib/mtk3-bsp/kernel/tkernel/task.c
    lib/mtk3-bsp/kernel/tkernel/timer.c
    lib/mtk3-bsp/kernel/tkernel/task_sync.c
    lib/mtk3-bsp/kernel/tkernel/eventflag.c
    lib/mtk3-bsp/include/sys/sysdepend/pico_rp2040/machine.h
    lib/mtk3-bsp/kernel/sysdepend/pico_rp2040/sys_timer.h
    lib/mtk3-bsp/device/common/drvif/msdrvif.c
)

target_compile_options(mtk3 PRIVATE -w)
target_link_libraries(mtk3 PUBLIC pico_stdlib hardware_irq)

# This must come before we compile stuff
pico_sdk_init()

# Compile main.c into an executable
add_executable(main src/main.c)
target_sources(main PRIVATE
    src/lcg32.c
)
# Link stdlib to our exe
target_link_libraries(main
    pico_stdlib
    mtk3
)
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
