add_executable(test_obstacle_servo obstacle/obstacle_servo_test.c ${OBSTACLE_SRC}/drv_sonar.c)
target_include_directories(test_obstacle_servo PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} platform/fakes obstacle/fakes)
target_compile_options(test_obstacle_servo PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME obstacle.obstacle_servo.boot_center COMMAND test_obstacle_servo)
add_test(NAME obstacle.obstacle_servo.obstacle_trigger_readback COMMAND test_obstacle_servo trigger-readback)

add_executable(test_obstacle_aim obstacle/obstacle_aim_test.c
    ${OBSTACLE_SRC}/scan_task.c ${OBSTACLE_SRC}/sonar_health.c
    ${OBSTACLE_SRC}/geometry.c ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_aim PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE} platform/fakes)
target_compile_options(test_obstacle_aim PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/obstacle/fakes/sonar_task_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.obstacle_aim.default COMMAND test_obstacle_aim)
add_test(NAME obstacle.obstacle_aim.obstacle_echo_timing COMMAND test_obstacle_aim echo-timing)
add_test(NAME obstacle.obstacle_aim.obstacle_trigger_low_no_echo COMMAND test_obstacle_aim trigger-low)
add_test(NAME obstacle.obstacle_aim.obstacle_aimed_echo_fault COMMAND test_obstacle_aim aimed-echo-fault)
add_test(NAME obstacle.obstacle_aim.obstacle_aimed_forward_stream COMMAND test_obstacle_aim aimed-forward)
add_test(NAME obstacle.obstacle_aim.pilot_mirrored COMMAND test_obstacle_aim mirror)
add_test(NAME obstacle.obstacle_aim.stop_settle COMMAND test_obstacle_aim stop-settle)
add_test(NAME obstacle.obstacle_aim.stop_spacing COMMAND test_obstacle_aim stop-spacing)
add_test(NAME obstacle.obstacle_aim.stop_echo COMMAND test_obstacle_aim stop-echo)
add_test(NAME obstacle.obstacle_aim.stop_late_echo COMMAND test_obstacle_aim stop-late-echo)
add_test(NAME obstacle.obstacle_aim.stop_pending COMMAND test_obstacle_aim stop-pending)

add_executable(test_obstacle_scan obstacle/obstacle_scan_test.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_scan PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_link_libraries(test_obstacle_scan PRIVATE m)
target_compile_options(test_obstacle_scan PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.obstacle_scan.default COMMAND test_obstacle_scan)

add_executable(test_obstacle_ping obstacle/obstacle_ping_test.c)
target_include_directories(test_obstacle_ping PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_ping PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.obstacle_ping.default COMMAND test_obstacle_ping)

# Obstacle published copies: comms.c and obstacle.c against the kernel platform/fakes,
# with the event flag API force-included from platform/fakes/tk/flg_fake.h.
add_executable(test_obstacle_comms obstacle/obstacle_comms_test.c
    ${OBSTACLE_SRC}/comms.c
    ${OBSTACLE_SRC}/obstacle.c
    ${OBSTACLE_SRC}/sonar_health.c)
target_include_directories(test_obstacle_comms PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_obstacle_comms PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes/tk/flg_fake.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.obstacle_comms.default COMMAND test_obstacle_comms)

add_executable(test_obstacle_start obstacle/pilot_sonar_start_test.c ${OBSTACLE_SRC}/motion_link.c)
target_include_directories(test_obstacle_start PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_obstacle_start PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/ir/fakes/line_task_ports.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.obstacle_start.default COMMAND test_obstacle_start)

set(OBSTACLE_EXTRAS_SRC ${OBSTACLE_SRC}/extras)
add_executable(test_obstacle_extras_adapter obstacle/extras_adapter_test.c
    obstacle/fakes/extras_adapter_kernel.c
    ${OBSTACLE_EXTRAS_SRC}/adapter.c ${OBSTACLE_EXTRAS_SRC}/comms.c
    ${OBSTACLE_EXTRAS_SRC}/service.c ${OBSTACLE_EXTRAS_SRC}/statechart.c
    ${OBSTACLE_EXTRAS_SRC}/planner.c ${OBSTACLE_EXTRAS_SRC}/scan.c)
target_include_directories(test_obstacle_extras_adapter PRIVATE ${APP_INCLUDE} platform/fakes)
target_compile_definitions(test_obstacle_extras_adapter PRIVATE E_NOSPT=-9)
target_compile_options(test_obstacle_extras_adapter PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes/tk/flg_fake.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.extras_adapter.stop_and_observations COMMAND test_obstacle_extras_adapter)

add_executable(test_obstacle_extras_scan obstacle/extras_scan_test.c
    ${OBSTACLE_EXTRAS_SRC}/scan.c ${OBSTACLE_EXTRAS_SRC}/planner.c)
target_include_directories(test_obstacle_extras_scan PRIVATE ${OBSTACLE_EXTRAS_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_extras_scan PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle.extras_scan.original_replays COMMAND test_obstacle_extras_scan)

add_executable(test_obstacle_extras_navigation obstacle/extras_navigation_test.c
    ${OBSTACLE_EXTRAS_SRC}/scan.c ${OBSTACLE_EXTRAS_SRC}/planner.c
    ${OBSTACLE_EXTRAS_SRC}/statechart.c)
target_include_directories(test_obstacle_extras_navigation PRIVATE
    ${OBSTACLE_EXTRAS_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_extras_navigation PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
foreach(scenario IN ITEMS no_path_twice supervisor_handoff advisory_echo_health stale_close_is_advisory
                         half_speed_bumper matching_ping rejoin_speed reversal_pause close_echoes)
    add_test(NAME obstacle.extras_navigation.${scenario}
        COMMAND test_obstacle_extras_navigation ${scenario})
endforeach()
