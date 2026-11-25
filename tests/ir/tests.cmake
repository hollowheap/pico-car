add_executable(test_barcode_parser ir/barcode_parser_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/parser.c)
target_include_directories(test_barcode_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_barcode_parser PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir.barcode_parser.default COMMAND test_barcode_parser)

add_executable(test_barcode_width ir/barcode_width_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/barcode_width.c)
target_include_directories(test_barcode_width PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_barcode_width PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir.barcode_width.default COMMAND test_barcode_width)

add_executable(test_line_control ir/line_control_test.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_line_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_line_control PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_control PRIVATE m)
add_test(NAME ir.line_control.default COMMAND test_line_control)
add_test(NAME ir.line_control.line_controller_unit COMMAND test_line_control unit)
add_test(NAME ir.line_control.line_corner_wheel_demands COMMAND test_line_control corner-wheel-demands)

add_executable(test_line_task_contract ir/line_slope_contract_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line/line_task.c
    ${LINE_CONTROL_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line/line_filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/start_check.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/scan_algo.c
    ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_line_task_contract PRIVATE
    ${APP_INCLUDE} ${OBSTACLE_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes ../app_program/motion ../app_program/ir/line)
target_compile_options(test_line_task_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/ir/fakes/line_task_ports.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_task_contract PRIVATE m)
add_test(NAME ir.line_task_contract.line_task_slope_left COMMAND test_line_task_contract left)
foreach(startup_case grounded airborne no-imu late-imu no-wheel blocked clearance-lost noisy-baseline calibration calibration-blocked calibration-stuck)
    add_test(NAME ir.line_task_contract.startup_check_${startup_case} COMMAND test_line_task_contract startup-check-${startup_case})
endforeach()
add_test(NAME ir.line_task_contract.line_task_slope_right COMMAND test_line_task_contract right)
add_test(NAME ir.line_task_contract.line_task_retry_events COMMAND test_line_task_contract retry-events)
add_test(NAME ir.line_task_contract.bounded_course_hold COMMAND test_line_task_contract bounded-course-hold)
add_test(NAME ir.line_task_contract.bounded_course_box_removed COMMAND test_line_task_contract bounded-course-box-removed)
add_test(NAME ir.line_task_contract.bounded_course_box_invalid_rest COMMAND test_line_task_contract bounded-course-box-invalid-rest)
add_test(NAME ir.line_task_contract.bounded_course_probe_left COMMAND test_line_task_contract bounded-course-probe-left)
add_test(NAME ir.line_task_contract.bounded_course_probe_right COMMAND test_line_task_contract bounded-course-probe-right)
add_test(NAME ir.line_task_contract.bounded_course_clear_distance COMMAND test_line_task_contract bounded-course-clear-distance)
add_test(NAME ir.line_task_contract.bounded_course_cutoff COMMAND test_line_task_contract bounded-course-cutoff)
add_test(NAME ir.line_task_contract.bounded_course_ping_gap COMMAND test_line_task_contract bounded-course-ping-gap)
add_test(NAME ir.line_task_contract.bounded_course_slow_clear COMMAND test_line_task_contract bounded-course-slow-clear)
add_test(NAME ir.line_task_contract.pilot_coherent_motion_snapshot COMMAND test_line_task_contract coherent-motion-snapshot)
add_test(NAME ir.line_task_contract.pilot_motion_timing_fault_hold COMMAND test_line_task_contract motion-timing-fault)
add_test(NAME ir.line_task_contract.line_resume_clear COMMAND test_line_task_contract pose-clear)
add_test(NAME ir.line_task_contract.line_resume_near COMMAND test_line_task_contract pose-near)
add_test(NAME ir.line_task_contract.line_cancel_turn_after_pose_loss COMMAND test_line_task_contract pose-during-turn)
add_test(NAME ir.line_task_contract.barcode_route_ambiguous COMMAND test_line_task_contract probe-solid)
add_test(NAME ir.line_task_contract.barcode_route_placement_grid COMMAND ${CMAKE_COMMAND}
    -DROUTE_EXE=$<TARGET_FILE:test_line_task_contract>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/ir/check_barcode_route_grid.cmake)
add_test(NAME ir.line_task_contract.junction_left_wheel_pause COMMAND test_line_task_contract junction-left-pause)
add_test(NAME ir.line_task_contract.junction_right_wheel_pause COMMAND test_line_task_contract junction-right-pause)
add_test(NAME ir.line_task_contract.junction_probe_cancel_event COMMAND test_line_task_contract junction-cancel)
add_test(NAME ir.line_task_contract.junction_stripe_event COMMAND test_line_task_contract junction-stripe)
add_test(NAME ir.line_task_contract.turn_pivot_rises COMMAND test_line_task_contract turn-rise)
add_test(NAME ir.line_task_contract.pilot_turn_obstacle COMMAND test_line_task_contract turn-obstacle)
add_test(NAME ir.line_task_contract.pilot_turn_sonar_fault COMMAND test_line_task_contract turn-sonar-fault)
add_test(NAME ir.line_task_contract.pilot_straight_obstacle COMMAND test_line_task_contract straight-obstacle)
add_test(NAME ir.line_task_contract.invalid_launch COMMAND test_line_task_contract invalid-launch)
add_test(NAME ir.line_task_contract.pending_obstacle COMMAND test_line_task_contract pending-obstacle)
add_test(NAME ir.line_task_contract.pending_stale COMMAND test_line_task_contract pending-stale)
add_test(NAME ir.line_task_contract.pilot_stale_line_capture COMMAND test_line_task_contract stale-line-capture)
add_test(NAME ir.line_task_contract.pilot_turn_stale_line_capture COMMAND test_line_task_contract turn-stale-line-capture)
# These plain square corners exceed the track-width bound at the modeled
# barcode reader. They stay available in the replay binary.
set(CORNER_REPLAY_KNOWN_FAILING
    ir.line_task_contract.line_plain_square_left_matched_0_30
    ir.line_task_contract.line_plain_square_left_matched_5.5_30
    ir.line_task_contract.line_plain_square_left_nov16_-5.5_30
    ir.line_task_contract.line_plain_square_left_nov16_0_30
    ir.line_task_contract.line_plain_square_left_nov16_5.5_30
    ir.line_task_contract.line_plain_square_left_nov20_-5.5_30
    ir.line_task_contract.line_plain_square_left_nov20_0_30
    ir.line_task_contract.line_plain_square_left_nov20_5.5_30
    ir.line_task_contract.line_plain_square_right_matched_-5.5_30
    ir.line_task_contract.line_plain_square_right_matched_0_30
    ir.line_task_contract.line_plain_square_right_matched_5.5_30
    ir.line_task_contract.line_plain_square_right_nov16_-5.5_30
    ir.line_task_contract.line_plain_square_right_nov16_0_30
    ir.line_task_contract.line_plain_square_right_nov16_5.5_30
    ir.line_task_contract.line_plain_square_right_nov20_-5.5_30
    ir.line_task_contract.line_plain_square_right_nov20_0_30
    ir.line_task_contract.line_plain_square_right_nov20_5.5_30
)
foreach (shape IN ITEMS round square)
    foreach (direction IN ITEMS left right)
        foreach (wheel_model IN ITEMS matched nov16 nov20)
            foreach (lateral IN ITEMS -5.5 0 5.5)
                foreach (lead IN ITEMS -30 0 30)
                    set(case_name ir.line_task_contract.line_plain_${shape}_${direction}_${wheel_model}_${lateral}_${lead})
                    if (NOT case_name IN_LIST CORNER_REPLAY_KNOWN_FAILING)
                        add_test(NAME ${case_name} COMMAND test_line_task_contract
                            route-plain-${shape}-${direction} ${lateral} ${lead} ${wheel_model})
                        set_tests_properties(${case_name} PROPERTIES LABELS ordinary_corner)
                    endif ()
                    foreach (request IN ITEMS straight left right)
                        set(case_name ir.line_task_contract.line_pending_${request}_${shape}_${direction}_${wheel_model}_${lateral}_${lead})
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
    add_test(NAME ir.line_task_contract.line_route_${test_suffix} COMMAND test_line_task_contract route-${course})
endforeach ()
add_test(NAME ir.line_task_contract.line_route_right_nov20 COMMAND test_line_task_contract route-right -5.5 -30 nov20)
add_test(NAME ir.line_task_contract.line_route_t_left_matched COMMAND test_line_task_contract route-t-left 5.5 -30 matched)
add_test(NAME ir.line_task_contract.line_route_t_right_matched COMMAND test_line_task_contract route-t-right 5.5 -30 matched)
add_test(NAME ir.line_task_contract.line_route_t_straight_later_nov16 COMMAND test_line_task_contract route-t-straight-later 5.5 -30 nov16)
add_test(NAME ir.line_task_contract.line_route_t_straight_later_nov20 COMMAND test_line_task_contract route-t-straight-later 5.5 -30 nov20)
add_test(NAME ir.line_task_contract.line_route_single_left_matched_back COMMAND test_line_task_contract route-single-left 5.5 -30 matched)
add_test(NAME ir.line_task_contract.line_route_single_left_matched_inline COMMAND test_line_task_contract route-single-left 5.5 0 matched)

add_executable(test_line_filter ir/line_filter_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line/line_filter.c)
target_include_directories(test_line_filter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line)
target_compile_options(test_line_filter PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir.line_filter.default COMMAND test_line_filter)

# Barcode actions, the re-arm gate and the parser information.
add_executable(test_barcode_action ir/barcode_action_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/action.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/parser.c)
target_include_directories(test_barcode_action PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_barcode_action PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir.barcode_action.default COMMAND test_barcode_action)

add_executable(test_barcode_task_reject ir/barcode_task_reject_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/barcode_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/barcode_width.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/action.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/barcode/parser.c)
target_include_directories(test_barcode_task_reject PRIVATE ${APP_INCLUDE} platform/fakes)
target_compile_options(test_barcode_task_reject PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/supervisor/fakes/supervisor_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir.barcode_task_reject.default COMMAND test_barcode_task_reject)

# IR capture pure helpers: barcode edge decision and alarm catch-up.
add_executable(test_ir_edge ir/ir_edge_test.c)
target_include_directories(test_ir_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/ir/line
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_ir_edge PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME ir.ir_edge.default COMMAND test_ir_edge)

add_executable(test_ir_capture_contract ir/ir_capture_startup_contract_test.c
    ../app_program/ir/line/drv_ir_capture.c ../app_program/ir/line/line_filter.c ${LINE_CONTROL_SOURCES}
    ../app_program/ir/barcode/action.c ../app_program/motion/pilot/pilot.c ../app_program/motion/pilot/avoidance.c ../app_program/motion/pilot/contour.c ../app_program/motion/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES} ../app_program/motion/pilot/look.c ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_ir_capture_contract PRIVATE
    ${APP_INCLUDE} ${OBSTACLE_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/ir/fakes ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_definitions(test_ir_capture_contract PRIVATE TEST_CAPTURE_PILOT)
target_compile_options(test_ir_capture_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/ir/fakes/ir_capture_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_ir_capture_contract PRIVATE m)
add_test(NAME ir.ir_capture_contract.barcode_route_capture COMMAND test_ir_capture_contract route-capture)
add_test(NAME ir.ir_capture_contract.late_arm COMMAND test_ir_capture_contract late-arm)
add_test(NAME ir.ir_capture_contract.ir_capture_line_pins COMMAND test_ir_capture_contract line-capture)
add_test(NAME ir.ir_capture_contract.ir_capture_barcode_pulse_noise COMMAND test_ir_capture_contract barcode-pulse-noise)
add_test(NAME ir.ir_capture_contract.ir_capture_barcode_overflow_noise COMMAND test_ir_capture_contract barcode-overflow-noise)
add_test(NAME ir.ir_capture_contract.ir_capture_barcode_timer_wrap COMMAND test_ir_capture_contract barcode-timer-wrap)
add_test(NAME ir.ir_capture_contract.pilot_barcode_noise COMMAND test_ir_capture_contract pilot-barcode-noise)
