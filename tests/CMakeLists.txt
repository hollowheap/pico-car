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

add_executable(test_obstacle_bypass_sim test_obstacle_bypass_sim.c
    ${OBSTACLE_SRC}/geometry.c
    ${OBSTACLE_SRC}/follow.c
    ${OBSTACLE_SRC}/nav_core.c
    ${OBSTACLE_SRC}/statechart.c
    ${OBSTACLE_SRC}/planner.c
    ${OBSTACLE_SRC}/scan_algo.c)
target_include_directories(test_obstacle_bypass_sim PRIVATE ${OBSTACLE_SRC} ${APP_INCLUDE})
target_compile_options(test_obstacle_bypass_sim PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_obstacle_bypass_sim PRIVATE m)
add_test(NAME obstacle_bypass_sim COMMAND test_obstacle_bypass_sim)

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
target_compile_options(test_obstacle_comms PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/tk/flg_fake.h"
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME obstacle_comms COMMAND test_obstacle_comms)

# The STEER-word hug law and everything only it used are gone.
add_test(NAME obstacle_hug_law_removed
    COMMAND ${CMAKE_COMMAND}
        -DROOT=${CMAKE_CURRENT_SOURCE_DIR}/..
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_removed_symbols.cmake)

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

add_executable(test_line_control test_line_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_line_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_compile_options(test_line_control PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_line_control PRIVATE m)
add_test(NAME line_control COMMAND test_line_control)

add_executable(test_pilot_sim test_pilot_sim.c pilot_sim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/look.c
    ${OBSTACLE_SRC}/geometry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/track/line_control.c)
target_include_directories(test_pilot_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${OBSTACLE_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(test_pilot_sim PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pilot_sim PRIVATE m)
add_test(NAME pilot_sim COMMAND test_pilot_sim)

add_executable(test_pilot_line test_pilot_line.c pilot_sim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
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

add_executable(test_pilot_drive test_pilot_drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
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

# The command ring, the run state and the health checks.
set(SUPERVISOR_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src/supervisor)
add_executable(test_supervisor test_supervisor.c
    ${SUPERVISOR_SRC}/command_ring.c
    ${SUPERVISOR_SRC}/run_state.c
    ${SUPERVISOR_SRC}/health_core.c
    ${SUPERVISOR_SRC}/edge_events.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/platform/liveness.c)
target_include_directories(test_supervisor PRIVATE
    ${SUPERVISOR_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(test_supervisor PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME supervisor COMMAND test_supervisor)

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

add_executable(test_pilot_latches test_pilot_latches.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/drive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/pilot_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/pilot/hug.c
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
