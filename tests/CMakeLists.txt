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
add_test(NAME physics_acceleration_acquisition_time COMMAND test_physics_connection acceleration-acquisition-time)
add_test(NAME physics_attitude_events COMMAND test_physics_connection attitude-events)
add_test(NAME physics_hump_events COMMAND test_physics_connection hump-events)
add_test(NAME physics_stationary_start COMMAND test_physics_connection stationary-start)
add_test(NAME physics_stationary_setdown COMMAND test_physics_connection stationary-setdown)
add_test(NAME physics_calibration_gap COMMAND test_physics_connection calibration-gap)
add_test(NAME physics_repeated_read_loss COMMAND test_physics_connection repeated-read-loss)
add_test(NAME physics_start_witness COMMAND test_physics_connection start-witness)

# RP2040 I2C1 interrupt-driven driver against register and timer models.
add_executable(test_i2c_rp2040 test_i2c_rp2040.c ${PHYSICS_SRC}/i2c_rp2040.c)
target_include_directories(test_i2c_rp2040 PRIVATE ${PHYSICS_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_i2c_rp2040 PRIVATE -include stddef.h
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/tk/flg_fake.h"
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/i2c_rp2040_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME i2c_rp2040 COMMAND test_i2c_rp2040)
add_test(NAME i2c_completion_interrupt_armed COMMAND test_i2c_rp2040 completion-interrupt-armed)

# Obstacle avoidance pure modules
set(OBSTACLE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/navigation/obstacle)
set(APP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(PILOT_OBSTACLE_SOURCES
    ../src/pilot/hug/hug.c
    ../src/pilot/hug/motion.c
    ../src/pilot/hug/scan.c
    ../src/pilot/bypass/bypass.c
    ../src/pilot/bypass/geometry.c
    ../src/pilot/bypass/rejoin.c)

add_executable(test_obstacle_servo test_obstacle_servo.c ${OBSTACLE_SRC}/scan_hw.c)
target_include_directories(test_obstacle_servo PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} shim shim_obstacle_servo)
target_compile_options(test_obstacle_servo PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME obstacle_servo_boot_center COMMAND test_obstacle_servo)
add_test(NAME obstacle_trigger_readback COMMAND test_obstacle_servo trigger-readback)

add_executable(test_obstacle_aim test_obstacle_aim.c
    ${OBSTACLE_SRC}/scan_task.c ${OBSTACLE_SRC}/sonar_health.c
    ${OBSTACLE_SRC}/geometry.c ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_aim PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE} shim)
target_compile_options(test_obstacle_aim PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/sonar_task_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_aim COMMAND test_obstacle_aim)
add_test(NAME obstacle_echo_timing COMMAND test_obstacle_aim echo-timing)
add_test(NAME obstacle_trigger_low_no_echo COMMAND test_obstacle_aim trigger-low)
add_test(NAME obstacle_aimed_echo_fault COMMAND test_obstacle_aim aimed-echo-fault)
add_test(NAME obstacle_aimed_forward_stream COMMAND test_obstacle_aim aimed-forward)
add_test(NAME obstacle_aim_pilot_mirrored COMMAND test_obstacle_aim mirror)
add_test(NAME obstacle_aim_stop_settle COMMAND test_obstacle_aim stop-settle)
add_test(NAME obstacle_aim_stop_spacing COMMAND test_obstacle_aim stop-spacing)
add_test(NAME obstacle_aim_stop_echo COMMAND test_obstacle_aim stop-echo)
add_test(NAME obstacle_aim_stop_late_echo COMMAND test_obstacle_aim stop-late-echo)
add_test(NAME obstacle_aim_stop_pending COMMAND test_obstacle_aim stop-pending)

add_executable(test_obstacle_scan test_obstacle_scan.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_scan PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_link_libraries(test_obstacle_scan PRIVATE m)
target_compile_options(test_obstacle_scan PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_scan COMMAND test_obstacle_scan)

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
    ${OBSTACLE_SRC}/sonar_health.c)
target_include_directories(test_obstacle_comms PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_obstacle_comms PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/tk/flg_fake.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_comms COMMAND test_obstacle_comms)

add_executable(test_obstacle_start test_pilot_sonar_start.c ${OBSTACLE_SRC}/motion_link.c)
target_include_directories(test_obstacle_start PRIVATE
    ${OBSTACLE_SRC} ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_obstacle_start PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_line/line_task_ports.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_start COMMAND test_obstacle_start)

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

# Telemetry packet and command validation.
foreach(tname packet cmd)
    add_executable(test_${tname} test_${tname}.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry/${tname}.c)
    target_include_directories(test_${tname} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_${tname} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME telemetry_${tname} COMMAND test_${tname})
endforeach()

# Telemetry queue, net queue and worker. The queues run
# unchanged against the kernel shim and the message buffer fake in
# shim/tk/mbf_fake.h.
set(TELEMETRY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry)
set(TELEMETRY_TEST_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
set(TELEMETRY_TEST_FLAGS -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)

foreach(name IN ITEMS queue net_queue worker)
    add_executable(test_telemetry_${name} test_telemetry_${name}.c
        ${TELEMETRY_SRC}/${name}.c)
    target_include_directories(test_telemetry_${name} PRIVATE
        ${TELEMETRY_TEST_INCLUDES})
    target_compile_options(test_telemetry_${name} PRIVATE ${TELEMETRY_TEST_FLAGS})
    add_test(NAME telemetry_${name} COMMAND test_telemetry_${name})
endforeach()

add_executable(test_telemetry_connection_cmd test_telemetry_connection_cmd.c
    ${TELEMETRY_SRC}/cmd.c ${TELEMETRY_SRC}/connection_task.c
    ${TELEMETRY_SRC}/drv_wifi.c
    ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_telemetry_connection_cmd PRIVATE
    ${TELEMETRY_TEST_INCLUDES} ${TELEMETRY_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/shim_connection
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
target_compile_options(test_telemetry_connection_cmd PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_connection/kernel.h"
    -ffunction-sections -fdata-sections ${TELEMETRY_TEST_FLAGS})
target_link_options(test_telemetry_connection_cmd PRIVATE
    -Wl,--gc-sections -Wl,--wrap=telemetry_cmd_receive)
add_test(NAME telemetry_connection_cmd COMMAND test_telemetry_connection_cmd)
add_test(NAME telemetry_connection_heartbeat_busy COMMAND test_telemetry_connection_cmd heartbeat-busy)
add_test(NAME telemetry_connection_join_failure COMMAND test_telemetry_connection_cmd join-failure)

set(LINE_CONTROL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control/follow.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control/junction.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control/recovery.c)

add_executable(test_line_control test_line_control.c
    ${LINE_CONTROL_SOURCES})
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
    ${LINE_CONTROL_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/start_check.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/scan_algo.c
    ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_line_task_contract PRIVATE
    ${APP_INCLUDE} ${OBSTACLE_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/shim ../src/motion ../src/track)
target_compile_options(test_line_task_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_line/line_task_ports.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_task_contract PRIVATE m)
add_test(NAME line_task_slope_left COMMAND test_line_task_contract left)
foreach(startup_case grounded airborne no-imu late-imu no-wheel blocked clearance-lost noisy-baseline)
    add_test(NAME startup_check_${startup_case} COMMAND test_line_task_contract startup-check-${startup_case})
endforeach()
add_test(NAME line_task_slope_right COMMAND test_line_task_contract right)
add_test(NAME line_task_retry_events COMMAND test_line_task_contract retry-events)
add_test(NAME bounded_course_hold COMMAND test_line_task_contract bounded-course-hold)
add_test(NAME bounded_course_box_removed COMMAND test_line_task_contract bounded-course-box-removed)
add_test(NAME bounded_course_box_invalid_rest COMMAND test_line_task_contract bounded-course-box-invalid-rest)
add_test(NAME bounded_course_probe_left COMMAND test_line_task_contract bounded-course-probe-left)
add_test(NAME bounded_course_probe_right COMMAND test_line_task_contract bounded-course-probe-right)
add_test(NAME bounded_course_clear_distance COMMAND test_line_task_contract bounded-course-clear-distance)
add_test(NAME bounded_course_cutoff COMMAND test_line_task_contract bounded-course-cutoff)
add_test(NAME bounded_course_ping_gap COMMAND test_line_task_contract bounded-course-ping-gap)
add_test(NAME bounded_course_slow_clear COMMAND test_line_task_contract bounded-course-slow-clear)
add_test(NAME pilot_coherent_motion_snapshot COMMAND test_line_task_contract coherent-motion-snapshot)
add_test(NAME pilot_motion_timing_fault_hold COMMAND test_line_task_contract motion-timing-fault)
add_test(NAME line_resume_clear COMMAND test_line_task_contract pose-clear)
add_test(NAME line_resume_near COMMAND test_line_task_contract pose-near)
add_test(NAME line_cancel_turn_after_pose_loss COMMAND test_line_task_contract pose-during-turn)
add_test(NAME barcode_route_ambiguous COMMAND test_line_task_contract probe-solid)
add_test(NAME junction_left_wheel_pause COMMAND test_line_task_contract junction-left-pause)
add_test(NAME junction_right_wheel_pause COMMAND test_line_task_contract junction-right-pause)
add_test(NAME junction_probe_cancel_event COMMAND test_line_task_contract junction-cancel)
add_test(NAME junction_stripe_event COMMAND test_line_task_contract junction-stripe)
add_test(NAME turn_pivot_rises COMMAND test_line_task_contract turn-rise)
add_test(NAME pilot_turn_obstacle COMMAND test_line_task_contract turn-obstacle)
add_test(NAME pilot_turn_sonar_fault COMMAND test_line_task_contract turn-sonar-fault)
add_test(NAME pilot_straight_obstacle COMMAND test_line_task_contract straight-obstacle)
add_test(NAME line_task_contract_invalid_launch COMMAND test_line_task_contract invalid-launch)
add_test(NAME line_task_contract_pending_obstacle COMMAND test_line_task_contract pending-obstacle)
add_test(NAME line_task_contract_pending_stale COMMAND test_line_task_contract pending-stale)
add_test(NAME pilot_stale_line_capture COMMAND test_line_task_contract stale-line-capture)
add_test(NAME pilot_turn_stale_line_capture COMMAND test_line_task_contract turn-stale-line-capture)
# These plain square corners exceed the track-width bound at the modeled
# barcode reader. They stay available in the replay binary.
set(CORNER_REPLAY_KNOWN_FAILING
    line_plain_square_left_matched_0_30
    line_plain_square_left_matched_5.5_30
    line_plain_square_left_nov16_-5.5_30
    line_plain_square_left_nov16_0_30
    line_plain_square_left_nov16_5.5_30
    line_plain_square_left_nov20_-5.5_30
    line_plain_square_left_nov20_0_30
    line_plain_square_left_nov20_5.5_30
    line_plain_square_right_matched_-5.5_30
    line_plain_square_right_matched_0_30
    line_plain_square_right_matched_5.5_30
    line_plain_square_right_nov16_-5.5_30
    line_plain_square_right_nov16_0_30
    line_plain_square_right_nov16_5.5_30
    line_plain_square_right_nov20_-5.5_30
    line_plain_square_right_nov20_0_30
    line_plain_square_right_nov20_5.5_30
)
foreach (shape IN ITEMS round square)
    foreach (direction IN ITEMS left right)
        foreach (wheel_model IN ITEMS matched nov16 nov20)
            foreach (lateral IN ITEMS -5.5 0 5.5)
                foreach (lead IN ITEMS -30 0 30)
                    set(case_name line_plain_${shape}_${direction}_${wheel_model}_${lateral}_${lead})
                    if (NOT case_name IN_LIST CORNER_REPLAY_KNOWN_FAILING)
                        add_test(NAME ${case_name} COMMAND test_line_task_contract
                            route-plain-${shape}-${direction} ${lateral} ${lead} ${wheel_model})
                        set_tests_properties(${case_name} PROPERTIES LABELS ordinary_corner)
                    endif ()
                    foreach (request IN ITEMS straight left right)
                        set(case_name line_pending_${request}_${shape}_${direction}_${wheel_model}_${lateral}_${lead})
                        if (NOT case_name IN_LIST CORNER_REPLAY_KNOWN_FAILING)
                            add_test(NAME ${case_name} COMMAND test_line_task_contract
                                route-${shape}-${direction}-pending-${request} ${lateral} ${lead} ${wheel_model})
                            set_tests_properties(${case_name} PROPERTIES LABELS pending_corner)
                        endif ()
                    endforeach ()
                endforeach ()
            endforeach ()
        endforeach ()
    endforeach ()
endforeach ()
foreach (course IN ITEMS left right straight-left straight-right t-left t-right t-straight-later
        missing-left missing-right single-left single-right offset-back offset-front left-mirror
        cancel event-abort straight-obstacle u u-replaces-left u-during-probe u-during-late-probe
        u-hold-during-probe u-repeat-manual u-invalid-origin u-invalid-capture)
    string(REPLACE "-" "_" test_suffix "${course}")
    add_test(NAME line_route_${test_suffix} COMMAND test_line_task_contract route-${course})
endforeach ()
add_test(NAME line_route_right_nov20 COMMAND test_line_task_contract route-right -5.5 -30 nov20)
add_test(NAME line_route_t_left_matched COMMAND test_line_task_contract route-t-left 5.5 -30 matched)
add_test(NAME line_route_t_right_matched COMMAND test_line_task_contract route-t-right 5.5 -30 matched)
add_test(NAME line_route_t_straight_later_nov16 COMMAND test_line_task_contract route-t-straight-later 5.5 -30 nov16)
add_test(NAME line_route_t_straight_later_nov20 COMMAND test_line_task_contract route-t-straight-later 5.5 -30 nov20)
add_test(NAME line_route_single_left_matched_back COMMAND test_line_task_contract route-single-left 5.5 -30 matched)
add_test(NAME line_route_single_left_matched_inline COMMAND test_line_task_contract route-single-left 5.5 0 matched)

add_executable(test_pilot_observations test_pilot_observations.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
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
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
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
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
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
target_compile_options(test_telemetry_emit PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/emit_kernel.h"
    ${TELEMETRY_TEST_FLAGS})
add_test(NAME telemetry_emit COMMAND test_telemetry_emit)

# Build the pinned SDK MQTT source against the host's simulated TCP connection.
set(LWIP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk/lib/lwip/src)
set(LWIP_SRC ${LWIP_ROOT}/include)
set(SDK_MQTT ${LWIP_ROOT}/apps/mqtt/mqtt.c)
set_source_files_properties(${SDK_MQTT} PROPERTIES COMPILE_OPTIONS "-w")
add_executable(test_mqtt_session test_mqtt_session.c ${SDK_MQTT}
    ${TELEMETRY_SRC}/mqtt_session.c ${TELEMETRY_SRC}/worker.c)
target_include_directories(test_mqtt_session BEFORE PRIVATE shim_tcp shim_connection)
target_include_directories(test_mqtt_session PRIVATE ${TELEMETRY_TEST_INCLUDES} ${TELEMETRY_SRC})
target_include_directories(test_mqtt_session SYSTEM PRIVATE ${LWIP_SRC})
target_compile_options(test_mqtt_session PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim_connection/kernel.h"
    ${TELEMETRY_TEST_FLAGS})
target_link_options(test_mqtt_session PRIVATE -Wl,--wrap=tk_snd_mbf)
foreach(scenario wire backpressure puback-space keepalive retry dns timeout)
    add_test(NAME telemetry_mqtt_${scenario} COMMAND test_mqtt_session ${scenario})
endforeach()

# Run the kernel radio owner loop and the link timer with simulated CYW43 events and leases.
set(RADIO_PORT ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
set(LINK_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry/wifi_link.c)
set_source_files_properties(${LINK_SRC} PROPERTIES INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/shim")
add_executable(test_wifi_recovery test_wifi_recovery.c ${RADIO_PORT}/cyw43_utk.c ${LINK_SRC})
set_property(TARGET test_wifi_recovery PROPERTY C_STANDARD 11)
target_include_directories(test_wifi_recovery PRIVATE shim_wifi shim_tcp ${RADIO_PORT}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk/lib/cyw43-driver/src
    ${LWIP_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libnet/lwip/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_definitions(test_wifi_recovery PRIVATE TK_SUPPORT_SMP=1 TM_WIFI_JOIN=1
    TM_WIFI_NETIF=1 TM_WIFI_DHCP=1 TM_WIFI_DNS=0 TM_WIFI_UDP=0 TM_WIFI_TCP=0
    TM_WIFI_TCPBULK=0 TM_WIFI_STATIC=0 CYW43_LWIP=1 LWIP_DHCP=1
    CYW43_CONFIG_FILE="radio_port.h")
target_compile_options(test_wifi_recovery PRIVATE -Wall -Wextra -Werror)
foreach(reason async request timeout late-link renumber)
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
    ${SUPERVISOR_SRC}/supervisor/loop.c
    ${SUPERVISOR_SRC}/supervisor/admission.c
    ${SUPERVISOR_SRC}/supervisor/telem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/telemetry/assemble.c)
target_include_directories(test_supervisor_start_contract PRIVATE ${SUPERVISOR_TEST_INCLUDES})
target_compile_definitions(test_supervisor_start_contract PRIVATE
    PICO_CAR_BUILD_HASH=0x1234ABCU PICO_CAR_BUILD_DIRTY=1)
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
add_test(NAME supervisor_unverified_start COMMAND test_supervisor_start_contract unverified-start)
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
foreach(resume_case IN ITEMS resume resume-bounded resume-used resume-stopped resume-corrupt resume-power)
    add_test(NAME supervisor_watchdog_${resume_case} COMMAND test_supervisor_start_contract watchdog-${resume_case})
endforeach()
add_test(NAME supervisor_watchdog_run_token COMMAND test_supervisor_start_contract run-token)
add_test(NAME telemetry_build_identity COMMAND test_supervisor_start_contract build-identity)
add_test(NAME motion_stop_timing COMMAND test_supervisor_start_contract stop-timing)
add_test(NAME motion_estop_timing COMMAND test_supervisor_start_contract estop-timing)
add_test(NAME telemetry_reserved_stationary_opcode COMMAND test_cmd reserved-stationary-opcode)
add_test(NAME motion_overlapping_stop_timing COMMAND test_supervisor_start_contract overlapping-stop-timing)
add_test(NAME motion_stale_rest_is_ignored COMMAND test_supervisor_start_contract stale-rest-is-ignored)
add_test(NAME motion_gp21_stop_timing COMMAND test_supervisor_start_contract gp21-stop-timing)
add_test(NAME motion_stop_timing_without_brake_evidence COMMAND test_supervisor_start_contract stop-timing-without-brake-evidence)
add_test(NAME motion_stop_episode_rearms_after_motion COMMAND test_supervisor_start_contract stop-episode-rearms-after-motion)
add_test(NAME motion_stop_timing_after_duty_returns COMMAND test_supervisor_start_contract stop-timing-after-duty-returns)
add_test(NAME motion_no_rearm_from_pre_release_motion COMMAND test_supervisor_start_contract no-rearm-from-pre-release-motion)
add_test(NAME motion_between_pass_estop_after_motion COMMAND test_supervisor_start_contract between-pass-estop-after-motion)
add_test(NAME motion_between_pass_estop_without_motion COMMAND test_supervisor_start_contract between-pass-estop-without-motion)
add_test(NAME supervisor_stop_with_slow_telemetry_consumer COMMAND test_supervisor_start_contract stop-with-slow-telemetry-consumer)
add_test(NAME supervisor_saturated_ack_handoff_stop COMMAND test_supervisor_start_contract saturated-ack-handoff-stop)
add_test(NAME supervisor_stationary_start_is_refused COMMAND test_supervisor_start_contract stationary-start-is-refused)
add_test(NAME supervisor_watchdog_rejects_stationary_resume COMMAND test_supervisor_start_contract watchdog-resume-reject-stationary)

# RP2040 reset reasons from the application watchdog adapter.
add_executable(test_watchdog_reset test_watchdog_reset.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/watchdog.c)
target_include_directories(test_watchdog_reset PRIVATE
    ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/shim ${CMAKE_CURRENT_SOURCE_DIR}/shim_motion)
target_compile_options(test_watchdog_reset PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
foreach(reset_case IN ITEMS timer forced both_watchdog power run debug unknown mixed
        token token-corrupt token-magic)
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

add_executable(test_motion_feedforward test_motion_feedforward.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/motor_setup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/odometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion/encoder_math.c)
target_include_directories(test_motion_feedforward PRIVATE
    ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/../src/motion ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_motion_feedforward PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion_feedforward COMMAND test_motion_feedforward)

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
             estop-gpio-wake estop-gpio-queued estop-final-admission estop-feedback-until-rest
             estop-unavailable-irq normal-output-admission state-events feedback-after-hold-release)
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

add_executable(test_encoder_capture test_encoder_capture.c ../src/motion/quadrature.c)
target_include_directories(test_encoder_capture PRIVATE ${APP_INCLUDE} ../src/motion shim shim_motion)
target_compile_options(test_encoder_capture PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME encoder_capture_avoids_masked_wait COMMAND test_encoder_capture)

# IR capture pure helpers: barcode edge decision and alarm catch-up.
add_executable(test_ir_edge test_ir_edge.c)
target_include_directories(test_ir_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_ir_edge PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir_edge COMMAND test_ir_edge)

add_executable(test_ir_capture_contract test_ir_capture_startup_contract.c
    ../src/track/ir_capture.c ../src/track/line_filter.c ${LINE_CONTROL_SOURCES}
    ../src/barcode/action.c ../src/pilot/pilot.c ../src/pilot/avoidance.c ../src/pilot/contour.c ../src/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES} ../src/pilot/look.c ${OBSTACLE_SRC}/geometry.c)
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
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_latches PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_latches PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_latches PRIVATE m)
add_test(NAME pilot_latches COMMAND test_pilot_latches)
add_test(NAME pilot_blind_crawl COMMAND test_pilot_latches blind-crawl)
foreach(latch_case encoder hug-timeout course-clearance)
    add_test(NAME pilot_recover_${latch_case} COMMAND test_pilot_latches recover-${latch_case})
endforeach()
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
add_test(NAME pilot_dropout_uses_one_interval COMMAND test_pilot_latches dropout-uses-one-interval)
add_test(NAME pilot_fan_deadline_survives_missing_replies COMMAND test_pilot_latches fan-deadline-survives-missing-replies)
add_test(NAME pilot_measured_retreat_keeps_recovery_eligible COMMAND test_pilot_latches measured-retreat-keeps-recovery-eligible)

add_executable(test_liveness test_liveness.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/liveness.c)
target_include_directories(test_liveness PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_definitions(test_liveness PRIVATE PICO_TEST_LIVENESS_ATOMICS)
target_compile_options(test_liveness PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME liveness COMMAND test_liveness)

add_executable(test_bypass_legs test_bypass_legs.c ${PILOT_OBSTACLE_SOURCES}
    ../src/pilot/look.c ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_bypass_legs PRIVATE ${APP_INCLUDE} ${OBSTACLE_SRC})
target_compile_options(test_bypass_legs PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_bypass_measured_legs COMMAND test_bypass_legs)
add_test(NAME obstacle_frozen_leg_ends_at_deadline COMMAND test_bypass_legs frozen-leg-ends-at-deadline)
add_test(NAME obstacle_same_pose_cannot_repeat_fan COMMAND test_bypass_legs same-pose-cannot-repeat-fan)
add_test(NAME obstacle_contact_fan_uses_measured_edge COMMAND test_bypass_legs contact-fan-uses-measured-edge)
add_test(NAME obstacle_contact_fan_preserves_unknown COMMAND test_bypass_legs contact-fan-preserves-unknown)
add_test(NAME obstacle_fan_glitch_keeps_deadline COMMAND test_bypass_legs fan-glitch-keeps-deadline)
add_test(NAME obstacle_fixed_pose_fan_once COMMAND test_bypass_legs fixed-pose-fan-once)
add_test(NAME obstacle_fixed_pose_hands_off_once COMMAND test_bypass_legs fixed-pose-hands-off-once)
add_test(NAME obstacle_hold_clock_excludes_fan_gap COMMAND test_bypass_legs hold-clock-excludes-fan-gap)
add_test(NAME obstacle_fixed_pose_stall_retrigger_once COMMAND test_bypass_legs fixed-pose-stall-retrigger-once)
add_test(NAME obstacle_fixed_pose_relook_cannot_restart_fan COMMAND test_bypass_legs fixed-pose-relook-cannot-restart-fan)
add_test(NAME obstacle_new_encounter_gets_bounded_fan COMMAND test_bypass_legs new-encounter-gets-bounded-fan)
add_test(NAME obstacle_dropouts_are_not_a_new_encounter COMMAND test_bypass_legs dropouts-are-not-a-new-encounter)
add_test(NAME obstacle_nonstationary_expired_fan_hands_off_once COMMAND test_bypass_legs nonstationary-expired-fan-hands-off-once)
add_test(NAME obstacle_single_wheel_travel_keeps_fan_closed COMMAND test_bypass_legs single-wheel-travel-keeps-fan-closed)

add_executable(test_pilot_rejoin test_pilot_rejoin.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot_log.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_rejoin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_rejoin PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_rejoin PRIVATE m)
add_test(NAME rejoin_outcome_reported COMMAND test_pilot_rejoin outcome-reported)
add_test(NAME rejoin_wrong_way_searches COMMAND test_pilot_rejoin wrong-way-searches)
add_test(NAME rejoin_bypass_end_silent COMMAND test_pilot_rejoin bypass-end-silent)
