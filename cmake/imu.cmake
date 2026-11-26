set(IMU_SRC ${PROJECT_SOURCE_DIR}/app_program/imu)

# Portable logic and drivers. Uses <math.h>, so it links libm ("m"):
# arm-none-eabi-gcc does not add -lm by itself, and the team build does not
# link pico_float.
add_library(imu_core STATIC EXCLUDE_FROM_ALL
    ${IMU_SRC}/attitude.c
    ${IMU_SRC}/hump_detect.c
    ${IMU_SRC}/imu_motion.c
    ${IMU_SRC}/hump_msg.c
    ${IMU_SRC}/drv_lsm303.c
    ${IMU_SRC}/motion_adapter.c
    ${IMU_SRC}/pipeline.c)
target_include_directories(imu_core PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_compile_options(imu_core PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})
target_link_libraries(imu_core PUBLIC m)

# Task, RP2040 I2C1 driver. Kernel-dependent.
add_library(imu_task STATIC EXCLUDE_FROM_ALL
    ${IMU_SRC}/drv_i2c.c
    ${IMU_SRC}/physics_task.c)
target_link_libraries(imu_task PUBLIC imu_core platform mtk3smp)
target_compile_options(imu_task PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})

# Supplies wheel feedback to the IMU task. The bench can link the task alone.
add_library(imu_motion_input STATIC EXCLUDE_FROM_ALL
    ${IMU_SRC}/motion_glue.c)
target_include_directories(imu_motion_input PRIVATE
    ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(imu_motion_input PUBLIC imu_core motion_service)
target_compile_options(imu_motion_input PRIVATE ${FIRMWARE_WARNINGS})

# Public facade: physics_setup()/physics_snapshot(). Keeps the packed hump
# word and the LSM303 driver private.
add_library(imu STATIC
    ${IMU_SRC}/physics.c)
target_include_directories(imu PRIVATE
    ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(imu PUBLIC imu_task imu_motion_input PRIVATE comms)
target_compile_options(imu PRIVATE
    -ffreestanding -Wall -Wextra -Werror -Wshadow -Wfloat-equal
    ${FIRMWARE_WARNINGS})
