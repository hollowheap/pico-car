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
target_include_directories(test_imu_motion PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_imu_motion PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_imu_motion PRIVATE m)
add_test(NAME imu_motion COMMAND test_imu_motion)

# test_drivers
add_executable(test_drivers test_drivers.c
    ${PHYSICS_SRC}/attitude.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/lsm303.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c)
target_include_directories(test_drivers PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_drivers PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_drivers PRIVATE m)
add_test(NAME drivers COMMAND test_drivers)
add_test(NAME physics_lift_settled_gap COMMAND test_drivers pipeline_lift_settled_gap)
add_test(NAME physics_sustained_lift_cues COMMAND test_drivers pipeline_sustained_lift_cues)

add_executable(test_physics_connection test_physics_connection.c
    ${PHYSICS_SRC}/physics.c
    ${PHYSICS_SRC}/physics_task.c
    ${PHYSICS_SRC}/attitude.c
    ${PHYSICS_SRC}/hump_detect.c
    ${PHYSICS_SRC}/imu_motion.c
    ${PHYSICS_SRC}/hump_msg.c
    ${PHYSICS_SRC}/motion_adapter.c
    ${PHYSICS_SRC}/pipeline.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/liveness.c)
target_include_directories(test_physics_connection PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_physics_connection PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/physics_task_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_physics_connection PRIVATE m)
add_test(NAME physics_connection COMMAND test_physics_connection)
add_test(NAME physics_snapshot_expiry COMMAND test_physics_connection snapshot-expiry)
add_test(NAME physics_attitude_events COMMAND test_physics_connection attitude-events)
add_test(NAME physics_hump_events COMMAND test_physics_connection hump-events)
add_test(NAME physics_stationary_start COMMAND test_physics_connection stationary-start)
add_test(NAME physics_stationary_setdown COMMAND test_physics_connection stationary-setdown)
add_test(NAME physics_calibration_gap COMMAND test_physics_connection calibration-gap)
add_test(NAME physics_repeated_read_loss COMMAND test_physics_connection repeated-read-loss)

# RP2040 I2C1 polling driver against a register model in the test.
add_executable(test_i2c_rp2040 test_i2c_rp2040.c ${PHYSICS_SRC}/i2c_rp2040.c)
target_include_directories(test_i2c_rp2040 PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_i2c_rp2040 PRIVATE -include stddef.h
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME i2c_rp2040 COMMAND test_i2c_rp2040)

# Obstacle avoidance pure modules
set(OBSTACLE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/obstacle)
set(APP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_executable(test_obstacle_servo test_obstacle_servo.c ${OBSTACLE_SRC}/scan_hw.c)
target_include_directories(test_obstacle_servo PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} shim shim_obstacle_servo)
target_compile_options(test_obstacle_servo PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME obstacle_servo_boot_center COMMAND test_obstacle_servo)

foreach(pilot RANGE 0 1)
    add_executable(test_obstacle_aim_${pilot} test_obstacle_aim.c
        ${OBSTACLE_SRC}/scan_task.c ${OBSTACLE_SRC}/statechart.c
        ${OBSTACLE_SRC}/geometry.c ${OBSTACLE_SRC}/planner.c ${OBSTACLE_SRC}/scan_algo.c)
    target_include_directories(test_obstacle_aim_${pilot} PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE} shim)
    target_compile_definitions(test_obstacle_aim_${pilot} PRIVATE PICO_CAR_PILOT=${pilot})
    target_compile_options(test_obstacle_aim_${pilot} PRIVATE
        "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/sonar_task_kernel.h"
        -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
    add_test(NAME obstacle_aim_${pilot} COMMAND test_obstacle_aim_${pilot})
    add_test(NAME obstacle_echo_timing_${pilot} COMMAND test_obstacle_aim_${pilot} echo-timing)
endforeach()

add_test(NAME obstacle_aim_pilot_mirrored COMMAND test_obstacle_aim_1 mirror)
add_test(NAME obstacle_aim_stop_settle COMMAND test_obstacle_aim_1 stop-settle)
add_test(NAME obstacle_aim_stop_spacing COMMAND test_obstacle_aim_1 stop-spacing)
add_test(NAME obstacle_aim_stop_echo COMMAND test_obstacle_aim_1 stop-echo)
add_test(NAME obstacle_aim_stop_late_echo COMMAND test_obstacle_aim_1 stop-late-echo)
add_test(NAME obstacle_aim_stop_pending COMMAND test_obstacle_aim_1 stop-pending)
add_test(NAME obstacle_aim_legacy_stop_settle COMMAND test_obstacle_aim_0 stop-settle)

add_executable(test_obstacle_scan test_obstacle_scan.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_scan PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_link_libraries(test_obstacle_scan PRIVATE m)
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

add_executable(test_obstacle_nav_core test_obstacle_nav_core.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c
    ${OBSTACLE_SRC}/nav_core.c)
target_include_directories(test_obstacle_nav_core PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_nav_core PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_nav_core COMMAND test_obstacle_nav_core)

add_executable(test_obstacle_follow test_obstacle_follow.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c
    ${OBSTACLE_SRC}/follow.c)
target_include_directories(test_obstacle_follow PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_follow PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_follow COMMAND test_obstacle_follow)

add_executable(test_obstacle_ping test_obstacle_ping.c)
target_include_directories(test_obstacle_ping PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_ping PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_ping COMMAND test_obstacle_ping)

# Obstacle published copies: comms.c and obstacle.c against the kernel shim,
# with the event flag API force-included from shim/tk/flg_fake.h.
add_executable(test_obstacle_comms test_obstacle_comms.c
    ${OBSTACLE_SRC}/comms.c
    ${OBSTACLE_SRC}/obstacle.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/statechart.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_comms PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_definitions(test_obstacle_comms PRIVATE PICO_CAR_PILOT=0)
target_compile_options(test_obstacle_comms PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/tk/flg_fake.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_comms COMMAND test_obstacle_comms)
add_test(NAME obstacle_legacy_plan_source COMMAND test_obstacle_comms legacy-plan-source)

# Exercise the real startup facade and stopped-state navigation adapter in both modes.
foreach(pilot IN ITEMS 0 1)
    set(target test_obstacle_start_${pilot})
    if(pilot)
        set(start_sources test_pilot_sonar_start.c)
    else()
        set(start_sources test_obstacle_start_contract.c ${OBSTACLE_SRC}/obstacle_nav.c)
    endif()
    add_executable(${target} ${start_sources} ${OBSTACLE_SRC}/motion_link.c
        ${OBSTACLE_SRC}/nav_core.c ${OBSTACLE_SRC}/follow.c
        ${OBSTACLE_SRC}/geometry.c ${OBSTACLE_SRC}/planner.c ${OBSTACLE_SRC}/scan_algo.c)
    target_include_directories(${target} PRIVATE
        ${OBSTACLE_SRC} ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim)
    target_compile_definitions(${target} PRIVATE PICO_CAR_PILOT=${pilot})
    target_compile_options(${target} PRIVATE
        "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_line/line_task_ports.h"
        -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
    add_test(NAME obstacle_start_${pilot} COMMAND ${target})
endforeach()

add_executable(test_barcode_parser test_barcode_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/barcode/parser.c)
target_include_directories(test_barcode_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_barcode_parser PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME barcode_parser COMMAND test_barcode_parser)

add_executable(test_barcode_width test_barcode_width.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/barcode_width.c)
target_include_directories(test_barcode_width PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_barcode_width PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME barcode_width COMMAND test_barcode_width)

# GPIO dispatcher: src/platform/isr.c against a kernel shim.
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
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_${tname} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME telemetry_${tname} COMMAND test_${tname})
endforeach()

# Telemetry queue, net queue, worker and publisher. The queues run
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

add_executable(test_telemetry_connection_cmd test_telemetry_connection_cmd.c
    ${TELEMETRY_SRC}/cmd.c ${TELEMETRY_SRC}/connection_task.c
    ${TELEMETRY_SRC}/packet.c ${TELEMETRY_SRC}/publisher.c)
target_include_directories(test_telemetry_connection_cmd PRIVATE
    ${TELEMETRY_TEST_INCLUDES} ${CMAKE_CURRENT_SOURCE_DIR}/shim_connection
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
target_compile_options(test_telemetry_connection_cmd PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_connection/kernel.h"
    -ffunction-sections -fdata-sections ${TELEMETRY_TEST_FLAGS})
target_link_options(test_telemetry_connection_cmd PRIVATE
    -Wl,--gc-sections -Wl,--wrap=telemetry_cmd_receive)
add_test(NAME telemetry_connection_cmd COMMAND test_telemetry_connection_cmd)
add_test(NAME telemetry_connection_publish_failure COMMAND test_telemetry_connection_cmd publish-failure)
add_test(NAME telemetry_connection_data_publish_failure COMMAND test_telemetry_connection_cmd data-publish-failure)
add_test(NAME telemetry_connection_join_failure COMMAND test_telemetry_connection_cmd join-failure)

add_executable(test_line_control test_line_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_line_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_line_control PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_control PRIVATE m)
add_test(NAME line_control COMMAND test_line_control)
add_test(NAME line_controller_unit COMMAND test_line_control unit)
add_test(NAME line_corner_wheel_demands COMMAND test_line_control corner-wheel-demands)

add_executable(test_line_task_contract test_line_slope_contract.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/bypass.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/scan_algo.c
    ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_line_task_contract PRIVATE
    ${APP_INCLUDE} ${OBSTACLE_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/shim ../src/motion)
target_compile_definitions(test_line_task_contract PRIVATE PICO_CAR_PILOT=1)
target_compile_options(test_line_task_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_line/line_task_ports.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_task_contract PRIVATE m)
add_test(NAME line_task_slope_left COMMAND test_line_task_contract left)
add_test(NAME line_task_slope_right COMMAND test_line_task_contract right)
add_test(NAME line_task_retry_events COMMAND test_line_task_contract retry-events)
add_test(NAME bounded_course_hold COMMAND test_line_task_contract bounded-course-hold)
add_test(NAME bounded_course_probe_left COMMAND test_line_task_contract bounded-course-probe-left)
add_test(NAME bounded_course_probe_right COMMAND test_line_task_contract bounded-course-probe-right)
add_test(NAME bounded_course_clear_distance COMMAND test_line_task_contract bounded-course-clear-distance)
add_test(NAME bounded_course_cutoff COMMAND test_line_task_contract bounded-course-cutoff)
add_test(NAME bounded_course_ping_gap COMMAND test_line_task_contract bounded-course-ping-gap)
add_test(NAME bounded_course_slow_clear COMMAND test_line_task_contract bounded-course-slow-clear)
add_test(NAME pilot_coherent_motion_snapshot COMMAND test_line_task_contract coherent-motion-snapshot)
add_test(NAME line_resume_clear COMMAND test_line_task_contract pose-clear)
add_test(NAME line_resume_near COMMAND test_line_task_contract pose-near)
add_test(NAME line_cancel_turn_after_pose_loss COMMAND test_line_task_contract pose-during-turn)
add_test(NAME barcode_route_ambiguous COMMAND test_line_task_contract probe-solid)
add_test(NAME junction_left_wheel_pause COMMAND test_line_task_contract junction-left-pause)
add_test(NAME junction_right_wheel_pause COMMAND test_line_task_contract junction-right-pause)
add_test(NAME pilot_turn_obstacle COMMAND test_line_task_contract turn-obstacle)
add_test(NAME pilot_turn_sonar_fault COMMAND test_line_task_contract turn-sonar-fault)
add_test(NAME pilot_straight_obstacle COMMAND test_line_task_contract straight-obstacle)
add_test(NAME line_task_contract_invalid_launch COMMAND test_line_task_contract invalid-launch)
add_test(NAME line_task_contract_pending_obstacle COMMAND test_line_task_contract pending-obstacle)
add_test(NAME line_task_contract_pending_stale COMMAND test_line_task_contract pending-stale)
add_test(NAME pilot_stale_line_capture COMMAND test_line_task_contract stale-line-capture)
add_test(NAME pilot_turn_stale_line_capture COMMAND test_line_task_contract turn-stale-line-capture)

add_executable(test_pilot_observations test_pilot_observations.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/bypass.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_pilot_observations PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(test_pilot_observations PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_observations PRIVATE m)
add_test(NAME pilot_observations COMMAND test_pilot_observations)

add_executable(test_pilot_line test_pilot_line.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/bypass.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_pilot_line PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(test_pilot_line PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_line PRIVATE m)
add_test(NAME pilot_line COMMAND test_pilot_line)
add_test(NAME pilot_course_reply_deadline COMMAND test_pilot_line course-reply-deadline)
add_test(NAME pilot_stationary_course_edge COMMAND test_pilot_line stationary-course-edge)
add_test(NAME pilot_bounded_course_contact_boundary COMMAND test_pilot_line bounded-course-contact-boundary)
add_test(NAME pilot_bounded_course_scan_budget COMMAND test_pilot_line bounded-course-scan-budget)
add_test(NAME pilot_interrupted_course_scan_budget COMMAND test_pilot_line interrupted-course-scan-budget)

add_executable(test_pilot_drive test_pilot_drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/bypass.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_pilot_drive PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_drive PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME pilot_drive COMMAND test_pilot_drive)

add_executable(test_pilot_log test_pilot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot_log.c)
target_include_directories(test_pilot_log PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_pilot_log PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME pilot_log COMMAND test_pilot_log)

add_executable(test_pilot_look test_pilot_look.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_pilot_look PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include ${OBSTACLE_SRC})
target_compile_options(test_pilot_look PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_look PRIVATE m)
add_test(NAME pilot_look COMMAND test_pilot_look)

add_executable(test_line_filter test_line_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_filter.c)
target_include_directories(test_line_filter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track)
target_compile_options(test_line_filter PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME line_filter COMMAND test_line_filter)

add_executable(test_log_core test_log_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/log_core.c)
target_include_directories(test_log_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform)
target_compile_options(test_log_core PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME log_core COMMAND test_log_core)

add_executable(test_faults test_faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/faults.c)
target_include_directories(test_faults PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_definitions(test_faults PRIVATE PICO_TEST_FAULT_CONTEXT=1)
target_compile_options(test_faults PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME faults COMMAND test_faults)

# The quadrature state machine program and its word decoder.
add_executable(test_quadrature test_quadrature.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/quadrature.c)
target_include_directories(test_quadrature PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/generated)
target_compile_definitions(test_quadrature PRIVATE PICO_NO_HARDWARE=1)
target_compile_options(test_quadrature PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME quadrature COMMAND test_quadrature)

add_executable(test_assemble test_assemble.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry/assemble.c)
target_include_directories(test_assemble PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_assemble PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME telemetry_assemble COMMAND test_assemble)

add_executable(test_telemetry_emit test_telemetry_emit.c
    ${TELEMETRY_SRC}/emit.c ${TELEMETRY_SRC}/queue.c ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_telemetry_emit PRIVATE ${TELEMETRY_TEST_INCLUDES})
target_compile_options(test_telemetry_emit PRIVATE ${TELEMETRY_TEST_FLAGS})
add_test(NAME telemetry_emit COMMAND test_telemetry_emit)

add_executable(test_telemetry_stream_loss test_telemetry_stream_loss.c)
target_include_directories(test_telemetry_stream_loss PRIVATE
    ${TELEMETRY_SRC} ${TELEMETRY_TEST_INCLUDES})
target_compile_options(test_telemetry_stream_loss PRIVATE ${TELEMETRY_TEST_FLAGS})
add_test(NAME telemetry_stream_loss COMMAND test_telemetry_stream_loss)

# tcp_transport.c runs against the lwIP and platform stubs in shim_tcp/, which
# comes before include/ for arch/cc.h and lwipopts.h; shim/ supplies tk/smp.h.
set(LWIP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk/lib/lwip/src/include)
add_executable(test_tcp_transport test_tcp_transport.c shim_tcp/tcp_stubs.c)
target_include_directories(test_tcp_transport BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/shim_tcp)
target_include_directories(test_tcp_transport PRIVATE
    ${TELEMETRY_SRC} ${TELEMETRY_TEST_INCLUDES})
target_include_directories(test_tcp_transport SYSTEM PRIVATE
    ${LWIP_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
target_compile_options(test_tcp_transport PRIVATE ${TELEMETRY_TEST_FLAGS})
add_test(NAME tcp_transport COMMAND test_tcp_transport)

# Run the production radio owner loop with simulated CYW43 events and leases.
set(RADIO_PORT ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
add_executable(test_wifi_recovery test_wifi_recovery.c ${RADIO_PORT}/cyw43_utk.c)
set_property(TARGET test_wifi_recovery PROPERTY C_STANDARD 11)
target_include_directories(test_wifi_recovery PRIVATE shim_wifi shim_tcp ${RADIO_PORT}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk/lib/cyw43-driver/src
    ${LWIP_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libnet/lwip/include)
target_compile_definitions(test_wifi_recovery PRIVATE TK_SUPPORT_SMP=1 TM_WIFI_JOIN=1
    TM_WIFI_NETIF=1 TM_WIFI_DHCP=1 TM_WIFI_DNS=0 TM_WIFI_UDP=0 TM_WIFI_TCP=0
    TM_WIFI_TCPBULK=0 TM_WIFI_STATIC=0 CYW43_LWIP=1 LWIP_DHCP=1
    CYW43_CONFIG_FILE="radio_port.h")
target_compile_options(test_wifi_recovery PRIVATE -Wall -Wextra -Werror)
foreach(reason async request timeout late-link)
    add_test(NAME wifi_join_${reason} COMMAND test_wifi_recovery ${reason})
endforeach()

# The command ring, the run state and the health checks.
set(SUPERVISOR_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/supervisor)
set(SUPERVISOR_TEST_CORE
    ${SUPERVISOR_SRC}/command_ring.c
    ${SUPERVISOR_SRC}/run_state.c
    ${SUPERVISOR_SRC}/health_core.c
    ${SUPERVISOR_SRC}/edge_events.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/liveness.c)
set(SUPERVISOR_TEST_INCLUDES
    ${SUPERVISOR_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
add_executable(test_supervisor test_supervisor.c ${SUPERVISOR_TEST_CORE})
target_include_directories(test_supervisor PRIVATE ${SUPERVISOR_TEST_INCLUDES})
target_compile_options(test_supervisor PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME supervisor COMMAND test_supervisor)

add_executable(test_supervisor_start_contract test_supervisor_start_contract.c
    ${SUPERVISOR_TEST_CORE}
    ${SUPERVISOR_SRC}/supervisor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry/assemble.c)
target_include_directories(test_supervisor_start_contract PRIVATE ${SUPERVISOR_TEST_INCLUDES})
target_compile_definitions(test_supervisor_start_contract PRIVATE PICO_CAR_PILOT=1)
target_compile_options(test_supervisor_start_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/supervisor_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME barcode_route_supervisor_slot COMMAND test_supervisor_start_contract route-slot)
add_test(NAME supervisor_estop_before_report COMMAND test_supervisor_start_contract estop-before-report)
add_test(NAME supervisor_estop_full_stop_inbox COMMAND test_supervisor_start_contract estop-full-stop-inbox)
add_test(NAME supervisor_stop_before_report COMMAND test_supervisor_start_contract stop-before-report)
add_test(NAME supervisor_calibrating_floor COMMAND test_supervisor_start_contract calibrating-floor)
add_test(NAME supervisor_continuous_floor COMMAND test_supervisor_start_contract continuous-floor)
add_test(NAME supervisor_start_calibration_handoff COMMAND test_supervisor_start_contract start-calibration-handoff)
add_test(NAME supervisor_delayed_upright_start COMMAND test_supervisor_start_contract delayed-upright-start)
foreach(pending_case stationary start stop timeout loss wheel fault settling tilted stale interrupted)
    add_test(NAME supervisor_calibrating_${pending_case} COMMAND test_supervisor_start_contract calibrating-${pending_case})
endforeach()
add_test(NAME supervisor_settling_pickup COMMAND test_supervisor_start_contract settling-pickup)
add_test(NAME supervisor_settling_tilted COMMAND test_supervisor_start_contract settling-tilted)
add_test(NAME supervisor_pickup_during_run COMMAND test_supervisor_start_contract running-pickup)
add_test(NAME supervisor_watchdog_gap COMMAND test_supervisor_start_contract watchdog-gap)
add_test(NAME supervisor_watchdog_physics COMMAND test_supervisor_start_contract watchdog-physics)
add_test(NAME supervisor_drive_feedback_validity COMMAND test_supervisor_start_contract drive-feedback-validity)
add_test(NAME supervisor_lost_reference COMMAND test_supervisor_start_contract lost-reference)
add_test(NAME supervisor_lost_reference_stop COMMAND test_supervisor_start_contract lost-reference-stop)
add_test(NAME supervisor_lost_reference_near COMMAND test_supervisor_start_contract lost-reference-near)
add_test(NAME supervisor_lost_reference_pickup COMMAND test_supervisor_start_contract lost-reference-pickup)
add_test(NAME supervisor_lost_reference_overturn COMMAND test_supervisor_start_contract lost-reference-overturn)
add_test(NAME supervisor_lost_reference_estop COMMAND test_supervisor_start_contract lost-reference-estop)
add_test(NAME supervisor_watchdog_expired COMMAND test_supervisor_start_contract watchdog-expired)
add_test(NAME supervisor_watchdog_gap_after_estop COMMAND test_supervisor_start_contract watchdog-gap-after-estop)
add_test(NAME supervisor_watchdog_expired_after_estop COMMAND test_supervisor_start_contract watchdog-expired-after-estop)
add_test(NAME supervisor_offline_power_up COMMAND test_supervisor_start_contract offline-start)
add_test(NAME supervisor_offline_watchdog_reset COMMAND test_supervisor_start_contract watchdog-offline)
add_test(NAME supervisor_offline_other_reset COMMAND test_supervisor_start_contract other-reset-offline)
add_test(NAME supervisor_start_button COMMAND test_supervisor_start_contract start-button)
add_test(NAME supervisor_start_button_gates COMMAND test_supervisor_start_contract start-button-gates)

# RP2040 reset reasons from the application watchdog adapter.
add_executable(test_watchdog_reset test_watchdog_reset.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/watchdog.c)
target_include_directories(test_watchdog_reset PRIVATE
    ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim ${CMAKE_CURRENT_SOURCE_DIR}/shim_motion)
target_compile_options(test_watchdog_reset PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
foreach(reset_case IN ITEMS timer forced both_watchdog power run debug unknown mixed)
    add_test(NAME watchdog_reset_${reset_case} COMMAND test_watchdog_reset ${reset_case})
endforeach()

# Lift and overturn from the gravity direction.
add_executable(test_attitude test_attitude.c ${PHYSICS_SRC}/attitude.c)
target_include_directories(test_attitude PRIVATE ${PHYSICS_SRC})
target_compile_options(test_attitude PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_attitude PRIVATE m)
add_test(NAME attitude COMMAND test_attitude)

# Barcode actions, the re-arm gate and the parser information.
add_executable(test_barcode_action test_barcode_action.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/barcode/action.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/barcode/parser.c)
target_include_directories(test_barcode_action PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_barcode_action PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME barcode_action COMMAND test_barcode_action)

add_executable(test_barcode_task_reject test_barcode_task_reject.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/barcode_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/barcode_width.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/barcode/action.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/barcode/parser.c)
target_include_directories(test_barcode_task_reject PRIVATE ${APP_INCLUDE} shim)
target_compile_options(test_barcode_task_reject PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/supervisor_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME barcode_task_reject COMMAND test_barcode_task_reject)

add_executable(test_motion_controller test_motion_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/odometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/encoder_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/motion_inbox.c)
target_include_directories(test_motion_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion)
target_compile_options(test_motion_controller PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion_controller COMMAND test_motion_controller)

add_executable(test_motion_feedback test_motion_feedback.c
    ../src/motion/controller.c ../src/motion/pid.c ../src/motion/odometry.c
    ../src/motion/encoder_math.c ../src/motion/motion_inbox.c
    ../src/platform/faults.c ../src/platform/liveness.c
    ../src/telemetry/assemble.c ../src/telemetry/packet.c)
target_include_directories(test_motion_feedback PRIVATE
    ${APP_INCLUDE} ../src/motion shim shim_motion)
target_compile_options(test_motion_feedback PRIVATE
    -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
target_link_options(test_motion_feedback PRIVATE -Wl,--gc-sections)
foreach(case signed-snapshot feedback-age-boundary reject-delayed-feedback
             estop-brake-failure boot-estop-brake-failure estop-remote-wake
             estop-gpio-wake estop-gpio-queued estop-final-admission
             estop-unavailable-irq normal-output-admission state-events)
    string(REPLACE "-" "_" test_name "${case}")
    add_test(NAME motion_${test_name} COMMAND test_motion_feedback ${case})
endforeach()
add_test(NAME telemetry_estop_request_status COMMAND test_motion_feedback estop-request-status)

# The bounded wait on the encoder RX FIFO (header-only, no PIO driver).
add_executable(test_encoder_pio_wait test_encoder_pio_wait.c)
target_include_directories(test_encoder_pio_wait PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion)
target_compile_options(test_encoder_pio_wait PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME encoder_pio_wait COMMAND test_encoder_pio_wait)

# IR capture pure helpers: barcode edge decision and alarm catch-up.
add_executable(test_ir_edge test_ir_edge.c)
target_include_directories(test_ir_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_ir_edge PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir_edge COMMAND test_ir_edge)

add_executable(test_ir_capture_contract test_ir_capture_startup_contract.c
    ../src/track/ir_capture.c ../src/track/line_filter.c ../src/track/line_control.c
    ../src/barcode/action.c ../src/pilot/pilot.c ../src/pilot/avoidance.c ../src/pilot/contour.c ../src/pilot/drive.c
    ../src/pilot/hug.c ../src/pilot/bypass.c ../src/pilot/look.c ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_ir_capture_contract PRIVATE
    ${APP_INCLUDE} ${OBSTACLE_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/shim_ir_capture ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_definitions(test_ir_capture_contract PRIVATE TEST_CAPTURE_PILOT)
target_compile_options(test_ir_capture_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/ir_capture_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_ir_capture_contract PRIVATE m)
add_test(NAME barcode_route_capture COMMAND test_ir_capture_contract route-capture)
add_test(NAME ir_capture_contract_late_arm COMMAND test_ir_capture_contract late-arm)
add_test(NAME ir_capture_line_pins COMMAND test_ir_capture_contract line-capture)
add_test(NAME ir_capture_barcode_pulse_noise COMMAND test_ir_capture_contract barcode-pulse-noise)
add_test(NAME ir_capture_barcode_overflow_noise COMMAND test_ir_capture_contract barcode-overflow-noise)
add_test(NAME ir_capture_barcode_timer_wrap COMMAND test_ir_capture_contract barcode-timer-wrap)
add_test(NAME pilot_barcode_noise COMMAND test_ir_capture_contract pilot-barcode-noise)

add_executable(test_pilot_latches test_pilot_latches.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/bypass.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_pilot_latches PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_latches PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_latches PRIVATE m)
add_test(NAME pilot_latches COMMAND test_pilot_latches)
add_test(NAME pilot_latches_steering_handoff COMMAND test_pilot_latches steering-handoff)
add_test(NAME pilot_latches_inner_wheel_stop COMMAND test_pilot_latches inner-wheel-stop)
add_test(NAME pilot_latches_unconfirmed_marker COMMAND test_pilot_latches unconfirmed-marker)
add_test(NAME pilot_marker_between_bends COMMAND test_pilot_latches marker-between-bends)
foreach(hug_case short-leg slow-scan frozen-pivot recorded-face adjacent-tape front-edge line-bend)
    add_test(NAME pilot_hug_${hug_case} COMMAND test_pilot_latches hug-${hug_case})
endforeach()
add_test(NAME pilot_startup_stall_scan COMMAND test_pilot_latches startup-stall-scan)
add_test(NAME pilot_course_stall_floor COMMAND test_pilot_latches course-stall-floor)
add_test(NAME pilot_course_held_contact COMMAND test_pilot_latches course-held-contact)
add_test(NAME pilot_course_slip_floor COMMAND test_pilot_latches course-slip-floor)
add_test(NAME pilot_course_movement_breaks_clear_streak COMMAND test_pilot_latches course-movement-breaks-clear-streak)
add_test(NAME pilot_forward_startup_deadline COMMAND test_pilot_latches never-seen-forward-startup-deadline)

add_executable(test_liveness test_liveness.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/liveness.c)
target_include_directories(test_liveness PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_definitions(test_liveness PRIVATE PICO_TEST_LIVENESS_ATOMICS)
target_compile_options(test_liveness PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME liveness COMMAND test_liveness)

add_executable(test_bypass_legs test_bypass_legs.c ../src/pilot/hug.c ../src/pilot/bypass.c
    ../src/pilot/look.c ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_bypass_legs PRIVATE ${APP_INCLUDE} ${OBSTACLE_SRC})
target_compile_options(test_bypass_legs PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_bypass_measured_legs COMMAND test_bypass_legs)
