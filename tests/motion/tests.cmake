add_executable(test_pilot_observations motion/pilot_observations_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_observations PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${OBSTACLE_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(test_pilot_observations PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_observations PRIVATE m)
add_test(NAME motion.pilot_observations.default COMMAND test_pilot_observations)

add_executable(test_pilot_line motion/pilot_line_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/drive.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_line PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${OBSTACLE_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(test_pilot_line PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_line PRIVATE m)
add_test(NAME motion.pilot_line.default COMMAND test_pilot_line)
add_test(NAME motion.pilot_line.pilot_course_reply_deadline COMMAND test_pilot_line course-reply-deadline)
add_test(NAME motion.pilot_line.pilot_stationary_course_edge COMMAND test_pilot_line stationary-course-edge)
add_test(NAME motion.pilot_line.pilot_bounded_course_contact_boundary COMMAND test_pilot_line bounded-course-contact-boundary)
add_test(NAME motion.pilot_line.pilot_bounded_course_scan_budget COMMAND test_pilot_line bounded-course-scan-budget)
add_test(NAME motion.pilot_line.pilot_interrupted_course_scan_budget COMMAND test_pilot_line interrupted-course-scan-budget)

add_executable(test_pilot_drive motion/pilot_drive_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/contour.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_drive PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_drive PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.pilot_drive.default COMMAND test_pilot_drive)

add_executable(test_pilot_log motion/pilot_log_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot_log.c)
target_include_directories(test_pilot_log PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_options(test_pilot_log PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.pilot_log.default COMMAND test_pilot_log)

add_executable(test_pilot_look motion/pilot_look_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_pilot_look PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program ${OBSTACLE_SRC})
target_compile_options(test_pilot_look PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_look PRIVATE m)
add_test(NAME motion.pilot_look.default COMMAND test_pilot_look)

# Assemble the same quadrature program with the pinned SDK's host tool.
set(PICO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk")
include("${PICO_SDK_PATH}/pico_sdk_version.cmake")
set(PIOASM_VERSION_STRING "${PICO_SDK_VERSION_STRING}")
add_subdirectory("${PICO_SDK_PATH}/tools/pioasm" pioasm EXCLUDE_FROM_ALL)
set(QUADRATURE_PROGRAM "${APP_INCLUDE}/motion/quadrature.pio")
set(QUADRATURE_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/quadrature.pio.h")
add_custom_command(OUTPUT "${QUADRATURE_HEADER}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/generated"
    COMMAND pioasm -o c-sdk "${QUADRATURE_PROGRAM}" "${QUADRATURE_HEADER}"
    DEPENDS pioasm "${QUADRATURE_PROGRAM}"
    VERBATIM)

# The quadrature state machine program and its word decoder.
add_executable(test_quadrature motion/quadrature_test.c ${QUADRATURE_HEADER}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/quadrature.c)
target_include_directories(test_quadrature PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion
    ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_compile_definitions(test_quadrature PRIVATE PICO_NO_HARDWARE=1)
target_compile_options(test_quadrature PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.quadrature.default COMMAND test_quadrature)

add_executable(test_motion_controller motion/motion_controller_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/odometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/encoder_math.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/motion_inbox.c)
target_include_directories(test_motion_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion)
target_compile_options(test_motion_controller PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.motion_controller.default COMMAND test_motion_controller)

add_executable(test_motion_feedforward motion/motion_feedforward_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/motor_setup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/odometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/encoder_math.c)
target_include_directories(test_motion_feedforward PRIVATE
    ${APP_INCLUDE} ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_motion_feedforward PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.motion_feedforward.default COMMAND test_motion_feedforward)

add_executable(test_motion_feedback motion/motion_feedback_test.c
    ../app_program/motion/controller.c ../app_program/motion/pid.c ../app_program/motion/odometry.c
    ../app_program/motion/encoder_math.c ../app_program/motion/motion_inbox.c
    ../app_program/platform/faults.c ../app_program/platform/liveness.c
    ../app_program/comms/telemetry/assemble.c ../app_program/comms/telemetry/packet.c)
target_include_directories(test_motion_feedback PRIVATE
    ${APP_INCLUDE} ../app_program/motion platform/fakes motion/fakes)
target_compile_options(test_motion_feedback PRIVATE
    -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
target_link_options(test_motion_feedback PRIVATE -Wl,--gc-sections)
foreach(case signed-snapshot feedback-age-boundary reject-delayed-feedback
             estop-brake-failure boot-estop-brake-failure estop-remote-wake
             estop-gpio-wake estop-gpio-queued estop-final-admission estop-feedback-until-rest
             estop-unavailable-irq normal-output-admission state-events feedback-after-hold-release)
    string(REPLACE "-" "_" test_name "${case}")
    add_test(NAME motion.motion_feedback.motion_${test_name} COMMAND test_motion_feedback ${case})
endforeach()
add_test(NAME motion.motion_feedback.telemetry_estop_request_status COMMAND test_motion_feedback estop-request-status)

# The bounded wait on the encoder RX FIFO (header-only, no PIO driver).
add_executable(test_encoder_pio_wait motion/encoder_pio_wait_test.c)
target_include_directories(test_encoder_pio_wait PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion)
target_compile_options(test_encoder_pio_wait PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.encoder_pio_wait.default COMMAND test_encoder_pio_wait)

add_executable(test_encoder_capture motion/encoder_capture_test.c ../app_program/motion/quadrature.c)
target_include_directories(test_encoder_capture PRIVATE ${APP_INCLUDE} ../app_program/motion platform/fakes motion/fakes)
target_compile_options(test_encoder_capture PRIVATE -Wall -Wextra -Werror -Wpedantic)
add_test(NAME motion.encoder_capture.avoids_masked_wait COMMAND test_encoder_capture)

add_executable(test_pilot_latches motion/pilot_latches_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot_log.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_latches PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_latches PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_latches PRIVATE m)
add_test(NAME motion.pilot_latches.default COMMAND test_pilot_latches)
add_test(NAME motion.pilot_latches.pilot_blind_crawl COMMAND test_pilot_latches blind-crawl)
foreach(latch_case encoder hug-timeout course-clearance)
    add_test(NAME motion.pilot_latches.pilot_recover_${latch_case} COMMAND test_pilot_latches recover-${latch_case})
endforeach()
add_test(NAME motion.pilot_latches.steering_handoff COMMAND test_pilot_latches steering-handoff)
add_test(NAME motion.pilot_latches.inner_wheel_stop COMMAND test_pilot_latches inner-wheel-stop)
add_test(NAME motion.pilot_latches.unconfirmed_marker COMMAND test_pilot_latches unconfirmed-marker)
add_test(NAME motion.pilot_latches.pilot_marker_between_bends COMMAND test_pilot_latches marker-between-bends)
foreach(hug_case short-leg slow-scan frozen-pivot recorded-face adjacent-tape front-edge line-bend)
    add_test(NAME motion.pilot_latches.pilot_hug_${hug_case} COMMAND test_pilot_latches hug-${hug_case})
endforeach()
add_test(NAME motion.pilot_latches.pilot_startup_stall_scan COMMAND test_pilot_latches startup-stall-scan)
add_test(NAME motion.pilot_latches.pilot_course_stall_floor COMMAND test_pilot_latches course-stall-floor)
add_test(NAME motion.pilot_latches.pilot_course_held_contact COMMAND test_pilot_latches course-held-contact)
add_test(NAME motion.pilot_latches.pilot_course_slip_floor COMMAND test_pilot_latches course-slip-floor)
add_test(NAME motion.pilot_latches.pilot_course_movement_breaks_clear_streak COMMAND test_pilot_latches course-movement-breaks-clear-streak)
add_test(NAME motion.pilot_latches.pilot_forward_startup_deadline COMMAND test_pilot_latches never-seen-forward-startup-deadline)
add_test(NAME motion.pilot_latches.pilot_dropout_uses_one_interval COMMAND test_pilot_latches dropout-uses-one-interval)
add_test(NAME motion.pilot_latches.pilot_fan_deadline_survives_missing_replies COMMAND test_pilot_latches fan-deadline-survives-missing-replies)
add_test(NAME motion.pilot_latches.pilot_measured_retreat_keeps_recovery_eligible COMMAND test_pilot_latches measured-retreat-keeps-recovery-eligible)

add_executable(test_bypass_legs motion/bypass_legs_test.c ${PILOT_OBSTACLE_SOURCES}
    ../app_program/motion/pilot/look.c ${OBSTACLE_SRC}/geometry.c)
target_include_directories(test_bypass_legs PRIVATE ${APP_INCLUDE} ${OBSTACLE_SRC})
target_compile_options(test_bypass_legs PRIVATE -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME motion.bypass_legs.obstacle_bypass_measured_legs COMMAND test_bypass_legs)
add_test(NAME motion.bypass_legs.obstacle_frozen_leg_ends_at_deadline COMMAND test_bypass_legs frozen-leg-ends-at-deadline)
add_test(NAME motion.bypass_legs.obstacle_same_pose_cannot_repeat_fan COMMAND test_bypass_legs same-pose-cannot-repeat-fan)
add_test(NAME motion.bypass_legs.obstacle_contact_fan_uses_measured_edge COMMAND test_bypass_legs contact-fan-uses-measured-edge)
add_test(NAME motion.bypass_legs.obstacle_contact_fan_preserves_unknown COMMAND test_bypass_legs contact-fan-preserves-unknown)
add_test(NAME motion.bypass_legs.obstacle_fan_glitch_keeps_deadline COMMAND test_bypass_legs fan-glitch-keeps-deadline)
add_test(NAME motion.bypass_legs.obstacle_fixed_pose_fan_once COMMAND test_bypass_legs fixed-pose-fan-once)
add_test(NAME motion.bypass_legs.obstacle_fixed_pose_hands_off_once COMMAND test_bypass_legs fixed-pose-hands-off-once)
add_test(NAME motion.bypass_legs.obstacle_hold_clock_excludes_fan_gap COMMAND test_bypass_legs hold-clock-excludes-fan-gap)
add_test(NAME motion.bypass_legs.obstacle_fixed_pose_stall_retrigger_once COMMAND test_bypass_legs fixed-pose-stall-retrigger-once)
add_test(NAME motion.bypass_legs.obstacle_fixed_pose_relook_cannot_restart_fan COMMAND test_bypass_legs fixed-pose-relook-cannot-restart-fan)
add_test(NAME motion.bypass_legs.obstacle_new_encounter_gets_bounded_fan COMMAND test_bypass_legs new-encounter-gets-bounded-fan)
add_test(NAME motion.bypass_legs.obstacle_dropouts_are_not_a_new_encounter COMMAND test_bypass_legs dropouts-are-not-a-new-encounter)
add_test(NAME motion.bypass_legs.obstacle_nonstationary_expired_fan_hands_off_once COMMAND test_bypass_legs nonstationary-expired-fan-hands-off-once)
add_test(NAME motion.bypass_legs.obstacle_single_wheel_travel_keeps_fan_closed COMMAND test_bypass_legs single-wheel-travel-keeps-fan-closed)

add_executable(test_pilot_rejoin motion/pilot_rejoin_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/avoidance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/contour.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/pilot_log.c
    ${PILOT_OBSTACLE_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/motion/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${LINE_CONTROL_SOURCES})
target_include_directories(test_pilot_rejoin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${OBSTACLE_SRC})
target_compile_options(test_pilot_rejoin PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_rejoin PRIVATE m)
add_test(NAME motion.pilot_rejoin.rejoin_outcome_reported COMMAND test_pilot_rejoin outcome-reported)
add_test(NAME motion.pilot_rejoin.rejoin_wrong_way_searches COMMAND test_pilot_rejoin wrong-way-searches)
add_test(NAME motion.pilot_rejoin.rejoin_bypass_end_silent COMMAND test_pilot_rejoin bypass-end-silent)
