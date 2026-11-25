set(PHYSICS_SRC ${PROJECT_SOURCE_DIR}/app_program/imu)

# Portable logic and drivers. Uses <math.h>, so it links libm ("m"):
# arm-none-eabi-gcc does not add -lm by itself, and the team build does not
# link pico_float.
add_library(physics_core STATIC EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/attitude.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/drv_lsm303.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c)
target_include_directories(physics_core PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_compile_options(physics_core PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})
target_link_libraries(physics_core PUBLIC m)

# Task, RP2040 I2C1 driver. Kernel-dependent.
add_library(physics_task STATIC EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/drv_i2c.c
    ${PHYSICS_SRC}/physics_task.c)
target_link_libraries(physics_task PUBLIC physics_core platform mtk3smp)
target_compile_options(physics_task PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})

# Optional motion-service reader. Link this only into an
# application that also links motion_service and starts it.
add_library(physics_glue STATIC EXCLUDE_FROM_ALL
    ${PHYSICS_SRC}/motion_glue.c)
target_include_directories(physics_glue PRIVATE
    ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(physics_glue PUBLIC physics_core motion_service)
target_compile_options(physics_glue PRIVATE ${FIRMWARE_WARNINGS})

# Public facade: physics_setup()/physics_snapshot(). Keeps the packed hump
# word and the LSM303 driver private.
add_library(physics STATIC
    ${PHYSICS_SRC}/physics.c)
target_include_directories(physics PRIVATE
    ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(physics PUBLIC physics_task physics_glue PRIVATE telemetry)
target_compile_options(physics PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})
