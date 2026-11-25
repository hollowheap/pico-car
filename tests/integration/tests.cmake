# The firmware tasks, composed as usermain() composes them, on a simulated kernel and plant.
set(INTEGRATION_FW ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
set(INTEGRATION_SOURCES
    ${INTEGRATION_FW}/app_main.c
    ${INTEGRATION_FW}/platform/platform.c
    ${INTEGRATION_FW}/platform/stack.c
    ${INTEGRATION_FW}/platform/faults.c
    ${INTEGRATION_FW}/platform/liveness.c
    ${INTEGRATION_FW}/comms/telemetry/car_id.c
    ${INTEGRATION_FW}/comms/telemetry/cmd.c
    ${INTEGRATION_FW}/comms/telemetry/connection_task.c
    ${INTEGRATION_FW}/comms/telemetry/emit.c
    ${INTEGRATION_FW}/comms/telemetry/queue.c
    ${INTEGRATION_FW}/comms/telemetry/net_queue.c
    ${INTEGRATION_FW}/comms/telemetry/packet.c
    ${INTEGRATION_FW}/comms/telemetry/assemble.c
    ${INTEGRATION_FW}/comms/telemetry/worker.c
    ${INTEGRATION_FW}/comms/telemetry/telemetry_start.c
    ${INTEGRATION_FW}/supervisor/command_ring.c
    ${INTEGRATION_FW}/supervisor/run_state.c
    ${INTEGRATION_FW}/supervisor/health_core.c
    ${INTEGRATION_FW}/supervisor/edge_events.c
    ${INTEGRATION_FW}/supervisor/supervisor/loop.c
    ${INTEGRATION_FW}/supervisor/supervisor/admission.c
    ${INTEGRATION_FW}/supervisor/supervisor/telem.c
    ${INTEGRATION_FW}/supervisor/supervisor_start.c
    ${INTEGRATION_FW}/motion/controller.c
    ${INTEGRATION_FW}/motion/pid.c
    ${INTEGRATION_FW}/motion/odometry.c
    ${INTEGRATION_FW}/motion/encoder_math.c
    ${INTEGRATION_FW}/motion/motion_inbox.c
    ${INTEGRATION_FW}/motion/motion_request.c
    ${INTEGRATION_FW}/motion/motion_service.c
    ${INTEGRATION_FW}/motion/motion_start.c
    ${INTEGRATION_FW}/motion/motor_setup.c
    ${INTEGRATION_FW}/motion/start_check.c
    ${INTEGRATION_FW}/motion/pilot/pilot.c
    ${INTEGRATION_FW}/motion/pilot/avoidance.c
    ${INTEGRATION_FW}/motion/pilot/contour.c
    ${INTEGRATION_FW}/motion/pilot/drive.c
    ${INTEGRATION_FW}/motion/pilot/look.c
    ${INTEGRATION_FW}/motion/pilot/pilot_log.c
    ${PILOT_OBSTACLE_SOURCES}
    ${LINE_CONTROL_SOURCES}
    ${INTEGRATION_FW}/ir/line/line_task.c
    ${INTEGRATION_FW}/ir/line/line_filter.c
    ${INTEGRATION_FW}/ir/line/track_start.c
    ${INTEGRATION_FW}/ir/barcode/barcode_task.c
    ${INTEGRATION_FW}/ir/barcode/barcode_start.c
    ${INTEGRATION_FW}/ir/barcode/action.c
    ${INTEGRATION_FW}/ir/barcode/parser.c
    ${INTEGRATION_FW}/ir/barcode/barcode_width.c
    ${INTEGRATION_FW}/obstacle/obstacle.c
    ${INTEGRATION_FW}/obstacle/comms.c
    ${INTEGRATION_FW}/obstacle/scan_task.c
    ${INTEGRATION_FW}/obstacle/obstacle_start.c
    ${INTEGRATION_FW}/obstacle/sonar_health.c
    ${INTEGRATION_FW}/obstacle/geometry.c
    ${INTEGRATION_FW}/obstacle/scan_algo.c
    ${INTEGRATION_FW}/obstacle/motion_link.c
    ${INTEGRATION_FW}/imu/physics.c
    ${INTEGRATION_FW}/imu/physics_start.c
    ${INTEGRATION_FW}/imu/physics_task.c
    ${INTEGRATION_FW}/imu/pipeline.c
    ${INTEGRATION_FW}/imu/attitude.c
    ${INTEGRATION_FW}/imu/hump_detect.c
    ${INTEGRATION_FW}/imu/hump_msg.c
    ${INTEGRATION_FW}/imu/imu_motion.c
    ${INTEGRATION_FW}/imu/motion_adapter.c
    ${INTEGRATION_FW}/imu/motion_glue.c
    ${INTEGRATION_FW}/imu/drv_lsm303.c
    ${INTEGRATION_FW}/motion/quadrature.c)
add_executable(test_integration integration/firmware_test.c
    integration/fakes/world.c ${INTEGRATION_SOURCES})
target_include_directories(test_integration PRIVATE
    ${APP_INCLUDE} ${APP_INCLUDE}/supervisor ${APP_INCLUDE}/motion ${APP_INCLUDE}/motion/pilot
    ${OBSTACLE_SRC} ${APP_INCLUDE}/ir/line ${APP_INCLUDE}/imu ${APP_INCLUDE}/comms/telemetry
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes ${CMAKE_CURRENT_SOURCE_DIR}/integration/fakes)
target_compile_options(test_integration PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/integration/fakes/kernel.h"
    -Wall -Wextra -Werror)
target_link_libraries(test_integration PRIVATE m)

foreach(scenario IN ITEMS boot_parked mqtt_start_stop periodic_state_bundle button_start
        mqtt_rejects_stale_commands mqtt_estop_keeps_latch sonar_approach_stops_for_obstacle)
    add_test(NAME integration.firmware.${scenario} COMMAND test_integration ${scenario})
    set_tests_properties(integration.firmware.${scenario} PROPERTIES TIMEOUT 5)
endforeach()
