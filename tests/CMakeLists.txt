cmake_minimum_required(VERSION 3.16)
project(pico_car_host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

set(BUDDY4_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/physics)

# test_hump_detect
add_executable(test_hump_detect test_hump_detect.c ${BUDDY4_SRC}/hump_detect.c)
target_include_directories(test_hump_detect PRIVATE ${BUDDY4_SRC})
target_compile_options(test_hump_detect PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_hump_detect PRIVATE m)
add_test(NAME hump_detect COMMAND test_hump_detect)

# test_hump_msg
add_executable(test_hump_msg test_hump_msg.c ${BUDDY4_SRC}/hump_msg.c ${BUDDY4_SRC}/hump_detect.c)
target_include_directories(test_hump_msg PRIVATE ${BUDDY4_SRC})
target_compile_options(test_hump_msg PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_hump_msg PRIVATE m)
add_test(NAME hump_msg COMMAND test_hump_msg)

# test_imu_motion
add_executable(test_imu_motion test_imu_motion.c ${BUDDY4_SRC}/imu_motion.c ${BUDDY4_SRC}/hump_detect.c)
target_include_directories(test_imu_motion PRIVATE ${BUDDY4_SRC})
target_compile_options(test_imu_motion PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_imu_motion PRIVATE m)
add_test(NAME imu_motion COMMAND test_imu_motion)

# test_drivers
add_executable(test_drivers test_drivers.c
    ${BUDDY4_SRC}/hump_detect.c
    ${BUDDY4_SRC}/imu_motion.c
    ${BUDDY4_SRC}/hump_msg.c
    ${BUDDY4_SRC}/lsm303.c
    ${BUDDY4_SRC}/b4_b2_adapter.c
    ${BUDDY4_SRC}/b4_pipeline.c)
target_include_directories(test_drivers PRIVATE ${BUDDY4_SRC})
target_compile_options(test_drivers PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_drivers PRIVATE m)
add_test(NAME drivers COMMAND test_drivers)

# Obstacle avoidance pure modules
set(OBSTACLE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/obstacle)
set(APP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(test_obstacle_scan test_obstacle_scan.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_scan PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_scan PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_scan COMMAND test_obstacle_scan)

add_executable(test_obstacle_plan test_obstacle_plan.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_plan PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_plan PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_plan COMMAND test_obstacle_plan)

add_executable(test_obstacle_statechart test_obstacle_statechart.c
    ${OBSTACLE_SRC}/statechart.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_statechart PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_statechart PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_statechart COMMAND test_obstacle_statechart)
