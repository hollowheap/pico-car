# Buddy 4 targets for pico-car/CMakeLists.txt. NOT yet added to the team repo.
#
# To try it: copy the sources into pico-car/src/physics/ and this file into
# pico-car/cmake/physics.cmake, then add this line at the end of
# pico-car/CMakeLists.txt:
#
#     include(${CMAKE_CURRENT_LIST_DIR}/cmake/physics.cmake)
#
# Not built yet: no ARM toolchain on the Buddy 4 laptop. The sources pass a
# clang Cortex-M0+ syntax check against the kernel headers only.
# Build after configure:  cd build && make physics_check

# PROJECT_SOURCE_DIR, not CMAKE_CURRENT_LIST_DIR: inside an included file
# the latter is this cmake/ directory.
set(PHYSICS_SRC ${PROJECT_SOURCE_DIR}/src/physics)

# Portable logic and drivers. Uses <math.h>, so it links libm ("m"):
# arm-none-eabi-gcc does not add -lm by itself, and the team build does not
# link pico_float.
add_library(physics_core STATIC EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/attitude.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/lsm303.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c)
target_include_directories(physics_core PUBLIC ${PHYSICS_SRC})
target_compile_options(physics_core PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})
target_link_libraries(physics_core PUBLIC m)

# Task, RP2040 I2C1 driver. Kernel-dependent.
add_library(physics_task STATIC EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/i2c_rp2040.c
    ${PHYSICS_SRC}/physics_task.c)
target_link_libraries(physics_task PUBLIC physics_core platform mtk3smp)
target_compile_options(physics_task PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})

# Optional: Buddy 2 motion-service reader. Link this only into an
# application that also links motion_service and starts it.
add_library(physics_glue STATIC EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/motion_glue.c)
target_include_directories(physics_glue PRIVATE
    ${PROJECT_SOURCE_DIR}/src/motion ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(physics_glue PUBLIC physics_core motion_service)
target_compile_options(physics_glue PRIVATE ${FIRMWARE_WARNINGS})

# Public facade: physics_init()/physics_snapshot(). Keeps the packed hump
# word and the LSM303 driver private.
add_library(physics STATIC
    ${PHYSICS_SRC}/physics.c)
target_include_directories(physics PRIVATE
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(physics PUBLIC physics_task physics_glue)
target_compile_options(physics PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})

# Stage 1/2 bring-up and CSV logger. Motors held low; Buddy 2 not started.
add_executable(physics_check EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/physics_check.c)
target_link_libraries(physics_check PRIVATE physics_task mtk3smp)
target_compile_options(physics_check PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections
    -Werror -Wall -Wextra -Wparentheses -Wfloat-equal -Wswitch-default
    -Wimplicit-fallthrough -Wshadow -Wvla)
target_link_options(physics_check PRIVATE
    -nostartfiles
    -Wl,--gc-sections
    -Wl,--undefined=boot2
    -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld")
pico_add_extra_outputs(physics_check)
