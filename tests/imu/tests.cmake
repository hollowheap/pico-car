# test_hump_detect
add_executable(test_hump_detect imu/hump_detect_test.c ${PHYSICS_SRC}/hump_detect.c)
target_include_directories(test_hump_detect PRIVATE ${PHYSICS_SRC})
target_compile_options(test_hump_detect PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_hump_detect PRIVATE m)
add_test(NAME imu.hump_detect.default COMMAND test_hump_detect)

# test_hump_msg
add_executable(test_hump_msg imu/hump_msg_test.c ${PHYSICS_SRC}/hump_msg.c ${PHYSICS_SRC}/hump_detect.c)
target_include_directories(test_hump_msg PRIVATE ${PHYSICS_SRC})
target_compile_options(test_hump_msg PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_hump_msg PRIVATE m)
add_test(NAME imu.hump_msg.default COMMAND test_hump_msg)

# test_imu_motion
add_executable(test_imu_motion imu/imu_motion_test.c ${PHYSICS_SRC}/imu_motion.c ${PHYSICS_SRC}/hump_detect.c)
target_include_directories(test_imu_motion PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_imu_motion PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_imu_motion PRIVATE m)
add_test(NAME imu.imu_motion.default COMMAND test_imu_motion)

# test_drivers
add_executable(test_drivers imu/drivers_test.c
    ${PHYSICS_SRC}/attitude.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/drv_lsm303.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c)
target_include_directories(test_drivers PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_drivers PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_drivers PRIVATE m)
add_test(NAME imu.drivers.default COMMAND test_drivers)
add_test(NAME imu.drivers.physics_lift_settled_gap COMMAND test_drivers pipeline_lift_settled_gap)
add_test(NAME imu.drivers.physics_sustained_lift_cues COMMAND test_drivers pipeline_sustained_lift_cues)

add_executable(test_physics_connection imu/physics_connection_test.c
    ${PHYSICS_SRC}/physics.c
    ${PHYSICS_SRC}/physics_task.c
    ${PHYSICS_SRC}/attitude.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/liveness.c)
target_include_directories(test_physics_connection PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_physics_connection PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/imu/fakes/physics_task_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_physics_connection PRIVATE m)
add_test(NAME imu.physics_connection.default COMMAND test_physics_connection)
add_test(NAME imu.physics_connection.physics_snapshot_expiry COMMAND test_physics_connection snapshot-expiry)
add_test(NAME imu.physics_connection.physics_acceleration_acquisition_time COMMAND test_physics_connection acceleration-acquisition-time)
add_test(NAME imu.physics_connection.physics_attitude_events COMMAND test_physics_connection attitude-events)
add_test(NAME imu.physics_connection.physics_hump_events COMMAND test_physics_connection hump-events)
foreach(height IN ITEMS 10 30)
    foreach(speed IN ITEMS 120 180)
        add_test(NAME imu.physics_connection.hump_${height}mm_at_${speed}mms
            COMMAND test_physics_connection hump-acceptance ${height} ${speed})
    endforeach()
endforeach()
add_test(NAME imu.physics_connection.physics_stationary_start COMMAND test_physics_connection stationary-start)
add_test(NAME imu.physics_connection.physics_stationary_setdown COMMAND test_physics_connection stationary-setdown)
add_test(NAME imu.physics_connection.physics_calibration_gap COMMAND test_physics_connection calibration-gap)
add_test(NAME imu.physics_connection.physics_repeated_read_loss COMMAND test_physics_connection repeated-read-loss)
add_test(NAME imu.physics_connection.physics_start_witness COMMAND test_physics_connection start-witness)

# RP2040 I2C1 interrupt-driven driver against register and timer models.
add_executable(test_i2c_rp2040 imu/i2c_rp2040_test.c ${PHYSICS_SRC}/drv_i2c.c)
target_include_directories(test_i2c_rp2040 PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_i2c_rp2040 PRIVATE -include stddef.h
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes/tk/flg_fake.h"
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/imu/fakes/i2c_rp2040_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME imu.i2c_rp2040.default COMMAND test_i2c_rp2040)
add_test(NAME imu.i2c_rp2040.i2c_completion_interrupt_armed COMMAND test_i2c_rp2040 completion-interrupt-armed)

# Lift and overturn from the gravity direction.
add_executable(test_attitude imu/attitude_test.c ${PHYSICS_SRC}/attitude.c)
target_include_directories(test_attitude PRIVATE ${PHYSICS_SRC})
target_compile_options(test_attitude PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_attitude PRIVATE m)
add_test(NAME imu.attitude.default COMMAND test_attitude)
