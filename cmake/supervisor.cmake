# Supervisor state, admission and telemetry handoff.
add_library(supervisor STATIC
    app_program/supervisor/command_ring.c
    app_program/supervisor/health_core.c
    app_program/supervisor/edge_events.c
    app_program/supervisor/run_state.c
    app_program/supervisor/supervisor/loop.c
    app_program/supervisor/supervisor/admission.c
    app_program/supervisor/supervisor/telem.c
)
target_include_directories(supervisor PUBLIC app_program PRIVATE app_program/supervisor)
target_link_libraries(supervisor PUBLIC platform telemetry motion_service obstacle physics mtk3smp)
target_compile_options(supervisor PRIVATE ${FIRMWARE_WARNINGS})

# The hardware watchdog is on by default. A watchdog reset returns the
# motor pins to inputs with pull-downs, so the H-bridge inputs read low. Turn it
# off for a debug build: ./configure.sh -DPICO_CAR_WATCHDOG=OFF
option(PICO_CAR_WATCHDOG "Start the RP2040 watchdog in the supervisor" ON)
if(PICO_CAR_WATCHDOG)
    target_compile_definitions(supervisor PRIVATE PICO_CAR_WATCHDOG)
endif()

# The boot events name the build: a 28-bit git short hash and whether tracked
# files differ from it. Without git the firmware reports an unknown build.
execute_process(COMMAND git rev-parse --short=7 HEAD
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE PICO_CAR_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE PICO_CAR_GIT_HASH_RESULT)
if(PICO_CAR_GIT_HASH_RESULT EQUAL 0)
    string(SUBSTRING "${PICO_CAR_GIT_HASH}" 0 7 PICO_CAR_GIT_HASH)
    execute_process(COMMAND git diff --quiet HEAD
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        RESULT_VARIABLE PICO_CAR_GIT_CLEAN_RESULT
        ERROR_QUIET OUTPUT_QUIET)
    if(PICO_CAR_GIT_CLEAN_RESULT EQUAL 0)
        set(PICO_CAR_BUILD_DIRTY 0)
    else()
        set(PICO_CAR_BUILD_DIRTY 1)
    endif()
    target_compile_definitions(supervisor PRIVATE
        PICO_CAR_BUILD_HASH=0x${PICO_CAR_GIT_HASH}U PICO_CAR_BUILD_DIRTY=${PICO_CAR_BUILD_DIRTY})
endif()
