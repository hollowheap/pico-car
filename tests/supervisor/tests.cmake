# The command ring, the run state and the health checks.
set(SUPERVISOR_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/supervisor)
set(SUPERVISOR_TEST_CORE
    ${SUPERVISOR_SRC}/command_ring.c
    ${SUPERVISOR_SRC}/run_state.c
    ${SUPERVISOR_SRC}/health_core.c
    ${SUPERVISOR_SRC}/edge_events.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/liveness.c)
set(SUPERVISOR_TEST_INCLUDES
    ${SUPERVISOR_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
add_executable(test_supervisor supervisor/supervisor_test.c ${SUPERVISOR_TEST_CORE}
    ${PHYSICS_SRC}/attitude.c)
target_include_directories(test_supervisor PRIVATE ${SUPERVISOR_TEST_INCLUDES})
target_compile_options(test_supervisor PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_supervisor PRIVATE m)
add_test(NAME supervisor.supervisor.default COMMAND test_supervisor)
add_test(NAME supervisor.supervisor.set_down_requires_stationary_wheels
    COMMAND test_supervisor set_down_requires_stationary_wheels)

add_executable(test_supervisor_start_contract supervisor/supervisor_start_contract_test.c
    ${SUPERVISOR_TEST_CORE}
    ${SUPERVISOR_SRC}/supervisor/loop.c
    ${SUPERVISOR_SRC}/supervisor/admission.c
    ${SUPERVISOR_SRC}/supervisor/telem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/platform/faults.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/comms/telemetry/assemble.c)
target_include_directories(test_supervisor_start_contract PRIVATE ${SUPERVISOR_TEST_INCLUDES})
target_compile_definitions(test_supervisor_start_contract PRIVATE
    PICO_CAR_BUILD_HASH=0x1234ABCU PICO_CAR_BUILD_DIRTY=1)
target_compile_options(test_supervisor_start_contract PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/supervisor/fakes/supervisor_kernel.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME supervisor.supervisor_start_contract.barcode_route_supervisor_slot COMMAND test_supervisor_start_contract route-slot)
add_test(NAME supervisor.supervisor_start_contract.supervisor_estop_before_report COMMAND test_supervisor_start_contract estop-before-report)
add_test(NAME supervisor.supervisor_start_contract.supervisor_estop_full_stop_inbox COMMAND test_supervisor_start_contract estop-full-stop-inbox)
add_test(NAME supervisor.supervisor_start_contract.supervisor_stop_before_report COMMAND test_supervisor_start_contract stop-before-report)
add_test(NAME supervisor.supervisor_start_contract.supervisor_calibrating_floor COMMAND test_supervisor_start_contract calibrating-floor)
add_test(NAME supervisor.supervisor_start_contract.supervisor_continuous_floor COMMAND test_supervisor_start_contract continuous-floor)
add_test(NAME supervisor.supervisor_start_contract.supervisor_start_calibration_handoff COMMAND test_supervisor_start_contract start-calibration-handoff)
add_test(NAME supervisor.supervisor_start_contract.supervisor_delayed_upright_start COMMAND test_supervisor_start_contract delayed-upright-start)
foreach(pending_case stationary start stop timeout loss wheel fault settling tilted stale interrupted)
    add_test(NAME supervisor.supervisor_start_contract.supervisor_calibrating_${pending_case} COMMAND test_supervisor_start_contract calibrating-${pending_case})
endforeach()
add_test(NAME supervisor.supervisor_start_contract.supervisor_settling_pickup COMMAND test_supervisor_start_contract settling-pickup)
add_test(NAME supervisor.supervisor_start_contract.supervisor_settling_tilted COMMAND test_supervisor_start_contract settling-tilted)
add_test(NAME supervisor.supervisor_start_contract.supervisor_pickup_during_run COMMAND test_supervisor_start_contract running-pickup)
add_test(NAME supervisor.supervisor_start_contract.supervisor_unverified_start COMMAND test_supervisor_start_contract unverified-start)
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_gap COMMAND test_supervisor_start_contract watchdog-gap)
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_physics COMMAND test_supervisor_start_contract watchdog-physics)
add_test(NAME supervisor.supervisor_start_contract.supervisor_drive_feedback_validity COMMAND test_supervisor_start_contract drive-feedback-validity)
add_test(NAME supervisor.supervisor_start_contract.supervisor_lost_reference COMMAND test_supervisor_start_contract lost-reference)
add_test(NAME supervisor.supervisor_start_contract.supervisor_lost_reference_stop COMMAND test_supervisor_start_contract lost-reference-stop)
add_test(NAME supervisor.supervisor_start_contract.supervisor_lost_reference_near COMMAND test_supervisor_start_contract lost-reference-near)
add_test(NAME supervisor.supervisor_start_contract.supervisor_lost_reference_pickup COMMAND test_supervisor_start_contract lost-reference-pickup)
add_test(NAME supervisor.supervisor_start_contract.supervisor_lost_reference_overturn COMMAND test_supervisor_start_contract lost-reference-overturn)
add_test(NAME supervisor.supervisor_start_contract.supervisor_lost_reference_estop COMMAND test_supervisor_start_contract lost-reference-estop)
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_expired COMMAND test_supervisor_start_contract watchdog-expired)
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_gap_after_estop COMMAND test_supervisor_start_contract watchdog-gap-after-estop)
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_expired_after_estop COMMAND test_supervisor_start_contract watchdog-expired-after-estop)
add_test(NAME supervisor.supervisor_start_contract.supervisor_offline_power_up COMMAND test_supervisor_start_contract offline-start)
add_test(NAME supervisor.supervisor_start_contract.supervisor_offline_watchdog_reset COMMAND test_supervisor_start_contract watchdog-offline)
add_test(NAME supervisor.supervisor_start_contract.supervisor_offline_other_reset COMMAND test_supervisor_start_contract other-reset-offline)
add_test(NAME supervisor.supervisor_start_contract.supervisor_start_button COMMAND test_supervisor_start_contract start-button)
add_test(NAME supervisor.supervisor_start_contract.supervisor_start_button_gates COMMAND test_supervisor_start_contract start-button-gates)
foreach(resume_case IN ITEMS resume resume-bounded resume-used resume-stopped resume-corrupt resume-power)
    add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_${resume_case} COMMAND test_supervisor_start_contract watchdog-${resume_case})
endforeach()
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_run_token COMMAND test_supervisor_start_contract run-token)
add_test(NAME supervisor.supervisor_start_contract.telemetry_build_identity COMMAND test_supervisor_start_contract build-identity)
add_test(NAME supervisor.supervisor_start_contract.motion_stop_timing COMMAND test_supervisor_start_contract stop-timing)
add_test(NAME supervisor.supervisor_start_contract.motion_estop_timing COMMAND test_supervisor_start_contract estop-timing)
add_test(NAME supervisor.supervisor_start_contract.motion_overlapping_stop_timing COMMAND test_supervisor_start_contract overlapping-stop-timing)
add_test(NAME supervisor.supervisor_start_contract.motion_stale_rest_is_ignored COMMAND test_supervisor_start_contract stale-rest-is-ignored)
add_test(NAME supervisor.supervisor_start_contract.motion_gp21_stop_timing COMMAND test_supervisor_start_contract gp21-stop-timing)
add_test(NAME supervisor.supervisor_start_contract.motion_stop_timing_without_brake_evidence COMMAND test_supervisor_start_contract stop-timing-without-brake-evidence)
add_test(NAME supervisor.supervisor_start_contract.motion_stop_episode_rearms_after_motion COMMAND test_supervisor_start_contract stop-episode-rearms-after-motion)
add_test(NAME supervisor.supervisor_start_contract.motion_stop_timing_after_duty_returns COMMAND test_supervisor_start_contract stop-timing-after-duty-returns)
add_test(NAME supervisor.supervisor_start_contract.motion_no_rearm_from_pre_release_motion COMMAND test_supervisor_start_contract no-rearm-from-pre-release-motion)
add_test(NAME supervisor.supervisor_start_contract.motion_between_pass_estop_after_motion COMMAND test_supervisor_start_contract between-pass-estop-after-motion)
add_test(NAME supervisor.supervisor_start_contract.motion_between_pass_estop_without_motion COMMAND test_supervisor_start_contract between-pass-estop-without-motion)
add_test(NAME supervisor.supervisor_start_contract.supervisor_stop_with_slow_telemetry_consumer COMMAND test_supervisor_start_contract stop-with-slow-telemetry-consumer)
add_test(NAME supervisor.supervisor_start_contract.supervisor_saturated_ack_handoff_stop COMMAND test_supervisor_start_contract saturated-ack-handoff-stop)
add_test(NAME supervisor.supervisor_start_contract.supervisor_stationary_start_is_refused COMMAND test_supervisor_start_contract stationary-start-is-refused)
add_test(NAME supervisor.supervisor_start_contract.supervisor_watchdog_rejects_stationary_resume COMMAND test_supervisor_start_contract watchdog-resume-reject-stationary)
