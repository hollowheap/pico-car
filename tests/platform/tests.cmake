# GPIO dispatcher: app_program/platform/drv_gpio_irq.c against a kernel platform/fakes.
add_executable(test_isr_dispatch platform/isr_dispatch_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/drv_gpio_irq.c)
target_include_directories(test_isr_dispatch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_isr_dispatch PRIVATE -Wall -Wextra -Werror)
add_test(NAME platform.isr_dispatch.default COMMAND test_isr_dispatch)

add_executable(test_log_core platform/log_core_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/log_core.c)
target_include_directories(test_log_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform)
target_compile_options(test_log_core PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME platform.log_core.default COMMAND test_log_core)

add_executable(test_faults platform/faults_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/faults.c)
target_include_directories(test_faults PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_definitions(test_faults PRIVATE PICO_TEST_FAULT_CONTEXT=1)
target_compile_options(test_faults PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME platform.faults.default COMMAND test_faults)

# RP2040 reset reasons from the application watchdog adapter.
add_executable(test_watchdog_reset platform/watchdog_reset_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/drv_watchdog.c)
target_include_directories(test_watchdog_reset PRIVATE
    ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes ${CMAKE_CURRENT_SOURCE_DIR}/motion/fakes)
target_compile_options(test_watchdog_reset PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
foreach(reset_case IN ITEMS timer forced both_watchdog power run debug unknown mixed
        token token-corrupt token-magic)
    add_test(NAME platform.watchdog_reset.${reset_case} COMMAND test_watchdog_reset ${reset_case})
endforeach()

add_executable(test_liveness platform/liveness_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/liveness.c)
target_include_directories(test_liveness PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_definitions(test_liveness PRIVATE PICO_TEST_LIVENESS_ATOMICS)
target_compile_options(test_liveness PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME platform.liveness.default COMMAND test_liveness)
