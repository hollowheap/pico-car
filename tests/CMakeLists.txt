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
