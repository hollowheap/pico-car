cmake_minimum_required(VERSION 3.16)
project(pico_car_host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()
include_directories(AFTER "${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes")

set(PHYSICS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/imu)
set(OBSTACLE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/obstacle)
set(APP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
set(PILOT_OBSTACLE_SOURCES
    ../app_program/motion/pilot/hug/hug.c
    ../app_program/motion/pilot/hug/motion.c
    ../app_program/motion/pilot/hug/scan.c
    ../app_program/motion/pilot/bypass/bypass.c
    ../app_program/motion/pilot/bypass/geometry.c
    ../app_program/motion/pilot/bypass/rejoin.c)

set(LINE_CONTROL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line/line_control/follow.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line/line_control/junction.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line/line_control/recovery.c)
set(TELEMETRY_SRC ${APP_INCLUDE}/comms/telemetry)

include(imu/tests.cmake)
include(obstacle/tests.cmake)
include(platform/tests.cmake)
include(ir/tests.cmake)
include(comms/tests.cmake)
include(motion/tests.cmake)
include(supervisor/tests.cmake)

# Keep existing case-specific labels, such as the two corner replay families.
get_property(host_tests DIRECTORY PROPERTY TESTS)
foreach(test IN LISTS host_tests)
    string(REGEX REPLACE "[.].*$" "" subsystem "${test}")
    set_property(TEST "${test}" APPEND PROPERTY LABELS "${subsystem}")
endforeach()
