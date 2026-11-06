# Buddy 4 targets for pico-car/CMakeLists.txt. NOT yet added to the team repo.
#
# To try it: copy the sources into pico-car/src/physics/ and this file into
# pico-car/cmake/buddy4.cmake, then add this line at the end of
# pico-car/CMakeLists.txt:
#
#     include(${CMAKE_CURRENT_LIST_DIR}/cmake/buddy4.cmake)
#
# Not built yet: no ARM toolchain on the Buddy 4 laptop. The sources pass a
# clang Cortex-M0+ syntax check against the kernel headers only.
# Build after configure:  cd build && make buddy4_imu_check

# PROJECT_SOURCE_DIR, not CMAKE_CURRENT_LIST_DIR: inside an included file
# the latter is this cmake/ directory.
set(BUDDY4_SRC ${PROJECT_SOURCE_DIR}/src/physics)

# Portable logic and drivers. Uses <math.h>, so it links libm ("m"):
# arm-none-eabi-gcc does not add -lm by itself, and the team build does not
# link pico_float.
add_library(buddy4_core STATIC EXCLUDE_FROM_ALL
    ${BUDDY4_SRC}/hump_detect.c
    ${BUDDY4_SRC}/imu_motion.c
    ${BUDDY4_SRC}/hump_msg.c
    ${BUDDY4_SRC}/lsm303.c
    ${BUDDY4_SRC}/b4_b2_adapter.c
    ${BUDDY4_SRC}/b4_pipeline.c)
target_include_directories(buddy4_core PUBLIC ${BUDDY4_SRC})
target_compile_options(buddy4_core PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal)
target_link_libraries(buddy4_core PUBLIC m)

# Task, RP2040 I2C1 driver. Kernel-dependent.
add_library(buddy4_task STATIC EXCLUDE_FROM_ALL
    ${BUDDY4_SRC}/b4_i2c_rp2040.c
    ${BUDDY4_SRC}/b4_task.c)
target_link_libraries(buddy4_task PUBLIC buddy4_core mtk3smp)
target_compile_options(buddy4_task PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal)

# Optional: Buddy 2 motion-service reader. Link this only into an
# application that also links motion_service and starts it.
add_library(buddy4_b2_glue STATIC EXCLUDE_FROM_ALL
    ${BUDDY4_SRC}/b4_b2_glue.c)
target_include_directories(buddy4_b2_glue PRIVATE
    ${PROJECT_SOURCE_DIR}/src/motion ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(buddy4_b2_glue PUBLIC buddy4_core motion_service)

# Public facade: physics_init()/physics_snapshot(). Keeps the packed hump
# word and the LSM303 driver private.
add_library(buddy4_physics STATIC
    ${BUDDY4_SRC}/physics.c)
target_include_directories(buddy4_physics PRIVATE
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(buddy4_physics PUBLIC buddy4_task buddy4_b2_glue)
target_compile_options(buddy4_physics PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal)

# Stage 1/2 bring-up and CSV logger. Motors held low; Buddy 2 not started.
add_executable(buddy4_imu_check EXCLUDE_FROM_ALL
    ${BUDDY4_SRC}/buddy4_imu_check.c)
target_link_libraries(buddy4_imu_check PRIVATE buddy4_task mtk3smp)
target_compile_options(buddy4_imu_check PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections
    -Werror -Wall -Wextra -Wparentheses -Wfloat-equal -Wswitch-default
    -Wimplicit-fallthrough -Wshadow -Wvla)
target_link_options(buddy4_imu_check PRIVATE
    -nostartfiles
    -Wl,--gc-sections
    -Wl,--undefined=boot2
    -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld")
pico_add_extra_outputs(buddy4_imu_check)
