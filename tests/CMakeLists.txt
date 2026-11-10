cmake_minimum_required(VERSION 3.16)
project(pico_car_host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

set(PHYSICS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/physics)

# test_hump_detect
add_executable(test_hump_detect test_hump_detect.c ${PHYSICS_SRC}/hump_detect.c)
target_include_directories(test_hump_detect PRIVATE ${PHYSICS_SRC})
target_compile_options(test_hump_detect PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_hump_detect PRIVATE m)
add_test(NAME hump_detect COMMAND test_hump_detect)

# test_hump_msg
add_executable(test_hump_msg test_hump_msg.c ${PHYSICS_SRC}/hump_msg.c ${PHYSICS_SRC}/hump_detect.c)
target_include_directories(test_hump_msg PRIVATE ${PHYSICS_SRC})
target_compile_options(test_hump_msg PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_hump_msg PRIVATE m)
add_test(NAME hump_msg COMMAND test_hump_msg)

# test_imu_motion
add_executable(test_imu_motion test_imu_motion.c ${PHYSICS_SRC}/imu_motion.c ${PHYSICS_SRC}/hump_detect.c)
target_include_directories(test_imu_motion PRIVATE ${PHYSICS_SRC})
target_compile_options(test_imu_motion PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_imu_motion PRIVATE m)
add_test(NAME imu_motion COMMAND test_imu_motion)

# test_drivers
add_executable(test_drivers test_drivers.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/lsm303.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c)
target_include_directories(test_drivers PRIVATE ${PHYSICS_SRC})
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
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_plan PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_plan PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_plan COMMAND test_obstacle_plan)

add_executable(test_obstacle_statechart test_obstacle_statechart.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/statechart.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_statechart PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_statechart PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_statechart COMMAND test_obstacle_statechart)

add_executable(test_obstacle_bypass_sim test_obstacle_bypass_sim.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/statechart.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_bypass_sim PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_bypass_sim PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_obstacle_bypass_sim PRIVATE m)
add_test(NAME obstacle_bypass_sim COMMAND test_obstacle_bypass_sim)

# D95: the STEER-word hug law and everything only it used are gone.
add_test(NAME obstacle_hug_law_removed
    COMMAND ${CMAKE_COMMAND}
        -DROOT=${CMAKE_CURRENT_SOURCE_DIR}/..
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_removed_symbols.cmake)

add_executable(test_barcode_parser test_barcode_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/barcode/parser.c)
target_include_directories(test_barcode_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_barcode_parser PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME barcode_parser COMMAND test_barcode_parser)

# GPIO dispatcher: src/platform/isr.c against a kernel shim (D67).
add_executable(test_isr_dispatch test_isr_dispatch.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/isr.c)
target_include_directories(test_isr_dispatch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_isr_dispatch PRIVATE -Wall -Wextra -Werror)
add_test(NAME isr_dispatch COMMAND test_isr_dispatch)

# Telemetry pure modules: packet, command validation and MQTT client
# (the MQTT client runs against a fake transport port).
foreach(tname packet cmd mqtt)
    if(tname STREQUAL "mqtt")
        set(tsrc mqtt_client.c)
    else()
        set(tsrc ${tname}.c)
    endif()
    add_executable(test_${tname} test_${tname}.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry/${tsrc})
    target_include_directories(test_${tname} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    target_compile_options(test_${tname} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME telemetry_${tname} COMMAND test_${tname})
endforeach()

# Telemetry queue, net queue, worker and publisher (S03, S24). The queues run
# unchanged against the kernel shim and the message buffer fake in
# shim/tk/mbf_fake.h.
set(TELEMETRY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry)
set(TELEMETRY_TEST_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
set(TELEMETRY_TEST_FLAGS -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)

foreach(name IN ITEMS queue net_queue worker publisher)
    add_executable(test_telemetry_${name} test_telemetry_${name}.c
        ${TELEMETRY_SRC}/${name}.c)
    target_include_directories(test_telemetry_${name} PRIVATE
        ${TELEMETRY_TEST_INCLUDES})
    target_compile_options(test_telemetry_${name} PRIVATE ${TELEMETRY_TEST_FLAGS})
    add_test(NAME telemetry_${name} COMMAND test_telemetry_${name})
endforeach()

add_executable(test_line_control test_line_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_line_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_line_control PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_control PRIVATE m)
add_test(NAME line_control COMMAND test_line_control)
