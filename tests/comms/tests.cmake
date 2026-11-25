# Telemetry packet and command validation.
foreach(tname packet cmd car_id)
    add_executable(test_${tname} comms/${tname}_test.c
        ${TELEMETRY_SRC}/${tname}.c)
    target_include_directories(test_${tname} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_${tname} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME comms.${tname}.default COMMAND test_${tname})
endforeach()

# Telemetry queue, net queue and worker. The queues run
# unchanged against the kernel platform/fakes and the message buffer fake in
# platform/fakes/tk/mbf_fake.h.
set(TELEMETRY_TEST_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
set(TELEMETRY_TEST_FLAGS -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)

foreach(name IN ITEMS queue net_queue worker)
    add_executable(test_telemetry_${name} comms/telemetry_${name}_test.c
        ${TELEMETRY_SRC}/${name}.c)
    target_include_directories(test_telemetry_${name} PRIVATE
        ${TELEMETRY_TEST_INCLUDES})
    target_compile_options(test_telemetry_${name} PRIVATE ${TELEMETRY_TEST_FLAGS})
    add_test(NAME comms.${name}.default COMMAND test_telemetry_${name})
endforeach()

add_executable(test_telemetry_connection_cmd comms/telemetry_connection_cmd_test.c
    ${TELEMETRY_SRC}/cmd.c ${TELEMETRY_SRC}/connection_task.c ${TELEMETRY_SRC}/car_id.c
    ${TELEMETRY_SRC}/drv_wifi.c
    ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_telemetry_connection_cmd PRIVATE
    ${TELEMETRY_TEST_INCLUDES} ${TELEMETRY_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/comms/fakes/connection
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
target_compile_options(test_telemetry_connection_cmd PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/comms/fakes/connection/kernel.h"
    -ffunction-sections -fdata-sections ${TELEMETRY_TEST_FLAGS})
target_link_options(test_telemetry_connection_cmd PRIVATE
    -Wl,--gc-sections -Wl,--wrap=telemetry_cmd_receive)
add_test(NAME comms.telemetry_connection_cmd.default COMMAND test_telemetry_connection_cmd)
add_test(NAME comms.telemetry_connection_cmd.telemetry_connection_heartbeat_busy COMMAND test_telemetry_connection_cmd heartbeat-busy)
add_test(NAME comms.telemetry_connection_cmd.telemetry_connection_join_failure COMMAND test_telemetry_connection_cmd join-failure)
add_executable(test_assemble comms/assemble_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/comms/telemetry/assemble.c)
target_include_directories(test_assemble PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes)
target_compile_options(test_assemble PRIVATE
    -Wall -Wextra -Werror -Wpedantic -Wconversion -Wshadow)
add_test(NAME comms.assemble.telemetry_assemble COMMAND test_assemble)

add_executable(test_telemetry_emit comms/telemetry_emit_test.c
    ${TELEMETRY_SRC}/emit.c ${TELEMETRY_SRC}/queue.c ${TELEMETRY_SRC}/packet.c)
target_include_directories(test_telemetry_emit PRIVATE ${TELEMETRY_TEST_INCLUDES})
target_compile_options(test_telemetry_emit PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/comms/fakes/emit_kernel.h"
    ${TELEMETRY_TEST_FLAGS})
add_test(NAME comms.telemetry_emit.default COMMAND test_telemetry_emit)

# Build the pinned SDK MQTT source against the host's simulated TCP connection.
set(LWIP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk/lib/lwip/src)
set(LWIP_SRC ${LWIP_ROOT}/include)
set(SDK_MQTT ${LWIP_ROOT}/apps/mqtt/mqtt.c)
set_source_files_properties(${SDK_MQTT} PROPERTIES COMPILE_OPTIONS "-w")
add_executable(test_mqtt_session comms/mqtt_session_test.c ${SDK_MQTT}
    ${TELEMETRY_SRC}/mqtt_session.c ${TELEMETRY_SRC}/car_id.c ${TELEMETRY_SRC}/worker.c)
target_include_directories(test_mqtt_session BEFORE PRIVATE comms/fakes/lwip comms/fakes/connection)
target_include_directories(test_mqtt_session PRIVATE ${TELEMETRY_TEST_INCLUDES} ${TELEMETRY_SRC})
target_include_directories(test_mqtt_session SYSTEM PRIVATE ${LWIP_SRC})
target_compile_options(test_mqtt_session PRIVATE
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/comms/fakes/connection/kernel.h"
    ${TELEMETRY_TEST_FLAGS})
target_link_options(test_mqtt_session PRIVATE -Wl,--wrap=tk_snd_mbf)
foreach(scenario wire backpressure puback-space keepalive retry dns timeout no-car-id)
    add_test(NAME comms.mqtt_session.telemetry_mqtt_${scenario} COMMAND test_mqtt_session ${scenario})
endforeach()

# Run the kernel radio owner loop and the link timer with simulated CYW43 events and leases.
set(RADIO_PORT ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libwifi/sysdepend/pico_rp2040)
set(LINK_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../app_program/comms/telemetry/drv_wifi_link.c)
set_source_files_properties(${LINK_SRC} PROPERTIES INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/platform/fakes")
add_executable(test_wifi_recovery comms/wifi_recovery_test.c ${RADIO_PORT}/cyw43_utk.c ${LINK_SRC})
set_property(TARGET test_wifi_recovery PROPERTY C_STANDARD 11)
target_include_directories(test_wifi_recovery PRIVATE comms/fakes/wifi comms/fakes/lwip ${RADIO_PORT}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/pico-sdk/lib/cyw43-driver/src
    ${LWIP_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/../lib/mtk3smp-rp2040/lib/libnet/lwip/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../app_program)
target_compile_definitions(test_wifi_recovery PRIVATE TK_SUPPORT_SMP=1 TM_WIFI_JOIN=1
    TM_WIFI_NETIF=1 TM_WIFI_DHCP=1 TM_WIFI_DNS=0 TM_WIFI_UDP=0 TM_WIFI_TCP=0
    TM_WIFI_TCPBULK=0 TM_WIFI_STATIC=0 CYW43_LWIP=1 LWIP_DHCP=1
    CYW43_CONFIG_FILE="radio_port.h")
target_compile_options(test_wifi_recovery PRIVATE -Wall -Wextra -Werror)
foreach(reason async request timeout late-link renumber)
    add_test(NAME comms.wifi_recovery.wifi_join_${reason} COMMAND test_wifi_recovery ${reason})
endforeach()
add_test(NAME comms.wifi_recovery.wifi_rssi_refresh COMMAND test_wifi_recovery rssi-refresh)

add_test(NAME comms.cmd.telemetry_reserved_stationary_opcode COMMAND test_cmd reserved-stationary-opcode)
