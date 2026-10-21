cmake_minimum_required(VERSION 3.16)
project(pico_car_host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_library(encoder_math_reference SHARED ../src/encoder_math.c)
target_include_directories(encoder_math_reference PUBLIC ../src)
target_compile_options(encoder_math_reference PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)

add_executable(test_encoder_math test_encoder_math.c ../src/encoder_math.c)
target_include_directories(test_encoder_math PRIVATE ../src)
target_compile_options(test_encoder_math PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)

add_test(NAME encoder_math_cases COMMAND test_encoder_math)
add_test(NAME encoder_math_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_encoder_math.py
            $<TARGET_FILE:encoder_math_reference>)

add_executable(test_encoder_quad test_encoder_quad.c ../src/encoder_quad.c)
target_include_directories(test_encoder_quad PRIVATE ../src)
target_compile_options(test_encoder_quad PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME encoder_quadrature COMMAND test_encoder_quad)

add_executable(test_motor_pulse test_motor_pulse.c ../src/motor_pulse.c)
target_include_directories(test_motor_pulse PRIVATE ../src)
target_compile_options(test_motor_pulse PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motor_pulse_policy COMMAND test_motor_pulse)

add_executable(test_motion_sampler test_motion_sampler.c
    ../src/motion_sampler.c ../src/encoder_math.c)
target_include_directories(test_motion_sampler PRIVATE ../src)
target_compile_options(test_motion_sampler PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion_sampler COMMAND test_motion_sampler)

add_executable(test_motor_pair test_motor_pair.c ../src/motor_pair.c)
add_executable(test_motor_batch test_motor_batch.c
    ../src/motor_batch.c ../src/motor_pulse.c)
foreach(test_target IN ITEMS test_motor_pair test_motor_batch)
    target_include_directories(${test_target} PRIVATE ../src)
    target_compile_options(${test_target} PRIVATE
        -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow -UNDEBUG)
endforeach()
add_test(NAME motor_pair_planning COMMAND test_motor_pair)
add_test(NAME motor_batch_policy COMMAND test_motor_batch)
