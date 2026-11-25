add_library(telemetry STATIC
    app_program/comms/telemetry/assemble.c
    app_program/comms/telemetry/cmd.c
    app_program/comms/telemetry/connection_task.c
    app_program/comms/telemetry/drv_wifi.c
    app_program/comms/telemetry/emit.c
    app_program/comms/telemetry/mqtt_session.c
    app_program/comms/telemetry/net_queue.c
    app_program/comms/telemetry/packet.c
    app_program/comms/telemetry/queue.c
    app_program/comms/telemetry/drv_wifi_link.c
    app_program/comms/telemetry/worker.c
)
target_include_directories(telemetry PUBLIC app_program PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/generated)
# wifi_link.c calls the CYW43 driver. The kernel's cyw43_configport.h must win
# over the SDK's, so the SDK's own driver app_program directory stays out.
set(TELEMETRY_RADIO_INCLUDES ${PICO_SDK_INCLUDES})
list(FILTER TELEMETRY_RADIO_INCLUDES EXCLUDE REGEX "pico_cyw43_driver")
target_include_directories(telemetry SYSTEM PRIVATE ${TELEMETRY_RADIO_INCLUDES})
target_link_libraries(telemetry PRIVATE pico_base_headers)
target_link_libraries(telemetry PUBLIC platform mtk3smp)
target_compile_options(telemetry PRIVATE ${FIRMWARE_WARNINGS})
