cmake_minimum_required(VERSION 3.16)
project(pico_car_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(MOTION_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion)
set(IR_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/track)
set(INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(TEST_FLAGS -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow -UNDEBUG)

add_library(encoder_math_reference SHARED ${MOTION_SRC}/encoder_math.c)
target_include_directories(encoder_math_reference PUBLIC ${INCLUDE} PRIVATE ${MOTION_SRC})
target_compile_options(encoder_math_reference PRIVATE ${TEST_FLAGS})

add_executable(test_encoder_math test_encoder_math.c ${MOTION_SRC}/encoder_math.c)
target_include_directories(test_encoder_math PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_encoder_math PRIVATE ${TEST_FLAGS})
add_test(NAME encoder_math_cases COMMAND test_encoder_math)
add_test(NAME encoder_math_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_encoder_math.py
            $<TARGET_FILE:encoder_math_reference>)

add_executable(test_quadrature test_quadrature.c ${MOTION_SRC}/quadrature.c)
target_include_directories(test_quadrature PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_quadrature PRIVATE ${TEST_FLAGS})
add_test(NAME quadrature COMMAND test_quadrature)

add_executable(test_odometry test_odometry.c
    ${MOTION_SRC}/odometry.c ${MOTION_SRC}/encoder_math.c)
target_include_directories(test_odometry PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_odometry PRIVATE ${TEST_FLAGS})
add_test(NAME odometry COMMAND test_odometry)

add_executable(test_bridge test_bridge.c ${MOTION_SRC}/bridge.c)
target_include_directories(test_bridge PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_bridge PRIVATE ${TEST_FLAGS})
add_test(NAME bridge_planning COMMAND test_bridge)

add_executable(test_pid test_pid.c ${MOTION_SRC}/pid.c
    ${MOTION_SRC}/odometry.c ${MOTION_SRC}/encoder_math.c)
target_include_directories(test_pid PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_pid PRIVATE ${TEST_FLAGS})
add_test(NAME pid COMMAND test_pid)

add_executable(test_guidance test_guidance.c
    ${MOTION_SRC}/guidance.c ${MOTION_SRC}/command_slot.c)
target_include_directories(test_guidance PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_guidance PRIVATE ${TEST_FLAGS})
add_test(NAME guidance_policy COMMAND test_guidance)

add_executable(test_controller test_controller.c
    ${MOTION_SRC}/controller.c ${MOTION_SRC}/pid.c
    ${MOTION_SRC}/odometry.c ${MOTION_SRC}/encoder_math.c
    ${MOTION_SRC}/guidance.c ${MOTION_SRC}/command_slot.c)
target_include_directories(test_controller PRIVATE
    ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_controller PRIVATE ${TEST_FLAGS})
add_test(NAME controller_integration COMMAND test_controller)

add_executable(test_line_filter test_line_filter.c ${IR_SRC}/line_filter.c)
target_include_directories(test_line_filter PRIVATE ${IR_SRC} ${INCLUDE})
target_compile_options(test_line_filter PRIVATE ${TEST_FLAGS})
add_test(NAME line_filter_debounce COMMAND test_line_filter)

add_executable(test_motion_feedback test_motion_feedback.c
    ${MOTION_SRC}/feedback.c)
target_include_directories(test_motion_feedback PRIVATE ${INCLUDE} ${MOTION_SRC})
target_compile_options(test_motion_feedback PRIVATE ${TEST_FLAGS})
add_test(NAME motion_feedback COMMAND test_motion_feedback)

add_executable(test_faults test_faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/faults.c)
target_include_directories(test_faults PRIVATE ${INCLUDE}
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_faults PRIVATE ${TEST_FLAGS})
target_link_libraries(test_faults PRIVATE pthread)
add_test(NAME fault_register COMMAND test_faults)

add_executable(test_line_recovery test_line_recovery.c
    ${IR_SRC}/line_recovery.c)
target_include_directories(test_line_recovery PRIVATE ${INCLUDE} ${IR_SRC})
target_compile_options(test_line_recovery PRIVATE ${TEST_FLAGS})
add_test(NAME line_recovery COMMAND test_line_recovery)

add_executable(test_mission_events test_mission_events.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/mission_events.c)
target_include_directories(test_mission_events PRIVATE ${INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_mission_events PRIVATE ${TEST_FLAGS})
add_test(NAME mission_events COMMAND test_mission_events)

add_executable(test_isr_select test_isr_select.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/isr_select.c)
target_include_directories(test_isr_select PRIVATE ${INCLUDE}
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_isr_select PRIVATE ${TEST_FLAGS})
add_test(NAME isr_select COMMAND test_isr_select)

add_executable(test_sonar test_sonar.c ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/sonar.c)
target_include_directories(test_sonar PRIVATE ${INCLUDE})
target_compile_options(test_sonar PRIVATE ${TEST_FLAGS})
add_test(NAME sonar_classifier COMMAND test_sonar)

add_executable(test_stop_confirm test_stop_confirm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/stop_confirm.c)
target_include_directories(test_stop_confirm PRIVATE ${INCLUDE})
target_compile_options(test_stop_confirm PRIVATE ${TEST_FLAGS})
add_test(NAME stop_confirm COMMAND test_stop_confirm)
