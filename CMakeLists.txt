cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
set(ENV{PICO_SDK_PATH} "${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk")
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico-car C CXX ASM)

# Telemetry broker endpoint and credentials.  Override with -D on the
# configure line; the defaults target a LAN Mosquitto instance.
set(PICO_CAR_MQTT_HOST "mosquitto" CACHE STRING "MQTT broker host or IP")
set(PICO_CAR_MQTT_PORT "1883" CACHE STRING "MQTT broker TCP port")
set(PICO_CAR_MQTT_USERNAME "" CACHE STRING "MQTT username (empty for anonymous)")
set(PICO_CAR_MQTT_PASSWORD "" CACHE STRING "MQTT password (empty for anonymous)")
set(PICO_CAR_TOPIC_PREFIX "car/SE01" CACHE STRING "MQTT topic prefix")
set(PICO_CAR_CLIENT_ID "pico-car" CACHE STRING "MQTT client identifier")

if (NOT PICO_CAR_MQTT_PORT MATCHES "^[0-9]+$")
    message(FATAL_ERROR "PICO_CAR_MQTT_PORT must be numeric")
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/telemetry_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/telemetry_config.h
    @ONLY
)

# Wi-Fi join. Empty SSID or password keeps TM_WIFI_JOIN=0. The generated
# credentials header lives in build/ and is never committed.
set(PICO_CAR_WIFI_SSID "" CACHE STRING "Wi-Fi SSID (empty disables the join)")
set(PICO_CAR_WIFI_PASSWORD "" CACHE STRING "Wi-Fi password (empty disables the join)")
set(PICO_CAR_WIFI_AUTH "CYW43_AUTH_WPA2_AES_PSK" CACHE STRING
    "CYW43 auth constant for the join")

if (PICO_CAR_WIFI_SSID AND PICO_CAR_WIFI_PASSWORD)
    set(PICO_CAR_WIFI_JOIN 1)
    string(REPLACE "\\" "\\\\" PICO_CAR_WIFI_SSID_ESC "${PICO_CAR_WIFI_SSID}")
    string(REPLACE "\"" "\\\"" PICO_CAR_WIFI_SSID_ESC "${PICO_CAR_WIFI_SSID_ESC}")
    string(REPLACE "\\" "\\\\" PICO_CAR_WIFI_PASSWORD_ESC "${PICO_CAR_WIFI_PASSWORD}")
    string(REPLACE "\"" "\\\"" PICO_CAR_WIFI_PASSWORD_ESC "${PICO_CAR_WIFI_PASSWORD_ESC}")
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/wifi_credentials.h.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/wifi_credentials.h
        @ONLY
    )
else()
    set(PICO_CAR_WIFI_JOIN 0)
endif()

# mtk3smp-rp2040 (uT-Kernel 3.0 RP2040 SMP port) presence check
set(MTK3SMP_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/mtk3smp-rp2040")

if (NOT EXISTS "${MTK3SMP_PATH}/include/tk/tkernel.h")
    message(FATAL_ERROR "mtk3smp-rp2040 missing. Run 'git submodule update --init --recursive'.")
endif()

# Create compile_commands.json for Clangd to analyze the project and report errors in the editor
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Barr standard enforces C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This must come before we compile stuff
pico_sdk_init()

# mtk3smp-rp2040 (uT-Kernel 3.0 RP2040 SMP port) init.
# Configuration mirrors build_make/pico_rp2040.mk:
#   SMP=1 CONSOLE=usb_cdc WIFI=cyw43 (JOIN/NETIF/STATIC/DHCP/DNS/UDP/TCP/BULK=0)
add_library(mtk3smp STATIC)

# The kernel's own makefile builds with -std=gnu11; only the application is
# held to Barr-C C99.
set_target_properties(mtk3smp PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)

target_compile_definitions(mtk3smp PUBLIC
    _PICO_RP2040_
    CNF_SMP=1
    TM_CONSOLE_USB_CDC=1
    TM_WIFI_CYW43=1
    TM_WIFI_JOIN=${PICO_CAR_WIFI_JOIN}
    TM_WIFI_NETIF=1
    TM_WIFI_STATIC=0
    TM_WIFI_DHCP=1
    TM_WIFI_DNS=1
    TM_WIFI_UDP=0
    TM_WIFI_TCP=0
    TM_WIFI_TCPBULK=0
    CYW43_LWIP=1
    CYW43_ENABLE_BLUETOOTH=0
    CYW43_USE_OTP_MAC=1
    CYW43_DEFAULT_PIN_WL_REG_ON=23u
    CYW43_DEFAULT_PIN_WL_DATA_OUT=24u
    CYW43_DEFAULT_PIN_WL_DATA_IN=24u
    CYW43_DEFAULT_PIN_WL_HOST_WAKE=24u
    CYW43_DEFAULT_PIN_WL_CS=25u
    CYW43_DEFAULT_PIN_WL_CLOCK=29u
    PICO_CYW43_LOGGING_ENABLED=0
    PICO_BUILD=1
    PICO_ON_DEVICE=1
    PICO_NO_HARDWARE=0
    PICO_32BIT=1
    PICO_RP2040=1
    RASPBERRYPI_PICO_W=1
    NDEBUG
)

target_include_directories(mtk3smp PUBLIC
    ${MTK3SMP_PATH}/include
    ${CMAKE_CURRENT_BINARY_DIR}/generated
    ${MTK3SMP_PATH}/config
    ${MTK3SMP_PATH}/kernel/knlinc
    ${MTK3SMP_PATH}/kernel/tkernel
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/rp2040
    ${MTK3SMP_PATH}/kernel/sysdepend/pico_rp2040
    ${MTK3SMP_PATH}/device
    ${MTK3SMP_PATH}/lib/libwifi/sysdepend/pico_rp2040
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/include
    ${MTK3SMP_PATH}/lib/libnet/lwip/include
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/pico_rp2040/usb
  PRIVATE
    ${PICO_SDK_PATH}/src/boards/include
    ${PICO_SDK_PATH}/src/common/hardware_claim/include
    ${PICO_SDK_PATH}/src/common/pico_base_headers/include
    ${PICO_SDK_PATH}/src/common/pico_binary_info/include
    ${PICO_SDK_PATH}/src/common/pico_util/include
    ${PICO_SDK_PATH}/src/rp2040/hardware_regs/include
    ${PICO_SDK_PATH}/src/rp2040/hardware_structs/include
    ${PICO_SDK_PATH}/src/rp2040/pico_platform/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_base/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_clocks/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_dma/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_irq/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_pio/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_resets/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_sync_spin_lock/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_timer/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_common/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_compiler/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_panic/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_platform_sections/include
    ${PICO_SDK_PATH}/lib/cyw43-driver/src
    ${PICO_SDK_PATH}/lib/cyw43-driver/firmware
    ${PICO_SDK_PATH}/lib/tinyusb/src
)

target_sources(mtk3smp PRIVATE
    ${MTK3SMP_PATH}/device/adc/adc.c
    ${MTK3SMP_PATH}/device/adc/sysdepend/rp2040/adc_rp2040.c
    ${MTK3SMP_PATH}/device/common/drvif/msdrvif.c
    ${MTK3SMP_PATH}/device/i2c/i2c.c
    ${MTK3SMP_PATH}/device/i2c/sysdepend/rp2040/i2c_rp2040.c
    ${MTK3SMP_PATH}/device/ser/ser.c
    ${MTK3SMP_PATH}/device/ser/sysdepend/rp2040/ser_rp2040.c
    ${MTK3SMP_PATH}/kernel/inittask/inittask.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m/dispatch.S
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m/exc_hdr.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m/interrupt.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m/reset_hdl.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/rp2040/boot_2nd.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/rp2040/smp_atomic.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/rp2040/smp_rp2040.c
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/rp2040/vector_tbl.c
    ${MTK3SMP_PATH}/kernel/sysdepend/pico_rp2040/cpu_clock.c
    ${MTK3SMP_PATH}/kernel/sysdepend/pico_rp2040/devinit.c
    ${MTK3SMP_PATH}/kernel/sysdepend/pico_rp2040/hw_setting.c
    ${MTK3SMP_PATH}/kernel/sysdepend/pico_rp2040/power_save.c
    ${MTK3SMP_PATH}/kernel/sysinit/sysinit.c
    ${MTK3SMP_PATH}/kernel/tkernel/cpuctl.c
    ${MTK3SMP_PATH}/kernel/tkernel/device.c
    ${MTK3SMP_PATH}/kernel/tkernel/deviceio.c
    ${MTK3SMP_PATH}/kernel/tkernel/eventflag.c
    ${MTK3SMP_PATH}/kernel/tkernel/int.c
    ${MTK3SMP_PATH}/kernel/tkernel/klock.c
    ${MTK3SMP_PATH}/kernel/tkernel/mailbox.c
    ${MTK3SMP_PATH}/kernel/tkernel/memory.c
    ${MTK3SMP_PATH}/kernel/tkernel/mempfix.c
    ${MTK3SMP_PATH}/kernel/tkernel/mempool.c
    ${MTK3SMP_PATH}/kernel/tkernel/messagebuf.c
    ${MTK3SMP_PATH}/kernel/tkernel/misc_calls.c
    ${MTK3SMP_PATH}/kernel/tkernel/mutex.c
    ${MTK3SMP_PATH}/kernel/tkernel/objname.c
    ${MTK3SMP_PATH}/kernel/tkernel/power.c
    ${MTK3SMP_PATH}/kernel/tkernel/ready_queue_smp.c
    ${MTK3SMP_PATH}/kernel/tkernel/rendezvous.c
    ${MTK3SMP_PATH}/kernel/tkernel/semaphore.c
    ${MTK3SMP_PATH}/kernel/tkernel/smp_dispatch.c
    ${MTK3SMP_PATH}/kernel/tkernel/smp_lock.c
    ${MTK3SMP_PATH}/kernel/tkernel/task.c
    ${MTK3SMP_PATH}/kernel/tkernel/task_manage.c
    ${MTK3SMP_PATH}/kernel/tkernel/task_sync.c
    ${MTK3SMP_PATH}/kernel/tkernel/time_calls.c
    ${MTK3SMP_PATH}/kernel/tkernel/timer.c
    ${MTK3SMP_PATH}/kernel/tkernel/tkinit.c
    ${MTK3SMP_PATH}/kernel/tkernel/wait.c
    ${MTK3SMP_PATH}/kernel/tstdlib/bitop.c
    ${MTK3SMP_PATH}/kernel/tstdlib/string.c
    ${MTK3SMP_PATH}/kernel/usermain/usermain.c
    ${MTK3SMP_PATH}/lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c
    ${MTK3SMP_PATH}/lib/libtk/fastlock.c
    ${MTK3SMP_PATH}/lib/libtk/fastmlock.c
    ${MTK3SMP_PATH}/lib/libtk/kmalloc.c
    ${MTK3SMP_PATH}/lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c
    ${MTK3SMP_PATH}/lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.c
    ${MTK3SMP_PATH}/lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c
    ${MTK3SMP_PATH}/lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c
    ${MTK3SMP_PATH}/lib/libtm/libtm.c
    ${MTK3SMP_PATH}/lib/libtm/libtm_printf.c
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/no_device/tm_com.c
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/pico_rp2040/tm_com.c
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/pico_rp2040/usb/usb_descriptors.c
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c
    ${MTK3SMP_PATH}/lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c
    ${MTK3SMP_PATH}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c
    ${MTK3SMP_PATH}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c
    ${MTK3SMP_PATH}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c

    ${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_ctrl.c
    ${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_ll.c
    ${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_lwip.c
    ${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_stats.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/tusb.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/common/tusb_fifo.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/device/usbd.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/device/usbd_control.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/class/cdc/cdc_device.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c
    ${PICO_SDK_PATH}/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c
    ${PICO_SDK_PATH}/src/rp2_common/hardware_dma/dma.c
    ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/gpio.c
    ${PICO_SDK_PATH}/src/rp2_common/hardware_pio/pio.c
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c

    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/def.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/dns.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/inet_chksum.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ip.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/mem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/memp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/netif.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/pbuf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/raw.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/stats.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/sys.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/tcp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/tcp_in.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/tcp_out.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/timeouts.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/udp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/acd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/dhcp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/etharp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/icmp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/ip4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/ip4_addr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/core/ipv4/ip4_frag.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwip/src/netif/ethernet.c
    ${MTK3SMP_PATH}/lib/libnet/lwip/lwip_utk.c
    ${MTK3SMP_PATH}/lib/libnet/lwip/lwip_utk_ipv4.c
)

target_compile_options(mtk3smp PRIVATE -w -ffunction-sections -fdata-sections)

# Compile the executable
add_library(sys STATIC
    src/sys/gpio_irq.c
)
target_include_directories(sys PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(sys PUBLIC mtk3smp)

add_library(platform STATIC
    src/platform/faults.c
)
target_include_directories(platform PUBLIC include)
target_link_libraries(platform PUBLIC mtk3smp)

add_library(telemetry STATIC
    src/telemetry/cmd.c
    src/telemetry/connection_task.c
    src/telemetry/mqtt_client.c
    src/telemetry/net_queue.c
    src/telemetry/packet.c
    src/telemetry/publisher.c
    src/telemetry/queue.c
    src/telemetry/tcp_transport.c
    src/telemetry/worker.c
)
target_include_directories(telemetry PUBLIC include PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(telemetry PUBLIC platform mtk3smp)

add_library(motion_core STATIC
    src/motion/controller.c
    src/motion/pid.c
    src/motion/odometry.c
    src/motion/encoder_math.c
    src/motion/motion_request.c
    src/motion/motion_inbox.c
)
target_include_directories(motion_core PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(motion_core PUBLIC mtk3smp)

add_library(motion_service STATIC
    src/motion/motion_service.c
    src/motion/pwm.c
    src/motion/motor.c
    src/motion/encoder.c
    src/motion/quadrature.c
)
target_include_directories(motion_service PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(motion_service PUBLIC sys motion_core mtk3smp)

add_library(ir_service STATIC
    src/ir/ir_capture.c
)
target_include_directories(ir_service PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(ir_service PUBLIC sys mtk3smp)

add_executable(main
    src/main.c
    src/demo_tasks.c
)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

foreach(app_target IN ITEMS main)
    # Link the uT-Kernel. The USB CDC console is provided by libtm, not pico_stdio.
    target_link_libraries(${app_target} PRIVATE
        sys
        motion_service
        ir_service
        platform
        telemetry
        mtk3smp
    )
    # The kernel supplies its own startup code (vector table, boot stage 2, reset
    # handler) and linker script; the SDK C runtime must not be linked in.
    target_compile_options(${app_target} PRIVATE
        -ffreestanding
        -ffunction-sections
        -fdata-sections
    )
    target_link_options(${app_target} PRIVATE
        -nostartfiles
        -Wl,--gc-sections
        # The second-stage bootloader is an unreferenced data symbol in the kernel
        # archive; without this it is not pulled in and the linker script's
        # ".boot2 must be 256 bytes" assertion fails.
        -Wl,--undefined=boot2
        -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld"
    )
    # Create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(${app_target})
    # Extra compiler flags
    target_compile_options(${app_target} PRIVATE
        -Werror
        -Wall
        -Wextra

        -Wparentheses           # 8.6 suspicious assignment
        -Wfloat-equal           # 5.4b.iv no float equality tests
        -Wswitch-default        # 8.3b default required
        -Wimplicit-fallthrough  # 8.3c commented fall-throughs
        -Wshadow
        -Wvla
    )
endforeach()

# ---------------------------------------------------------------------------
# Buddy 5: obstacle detection and avoidance (src/navigation/obstacle)
#   obstacle        library: sensor, servo, scan, planner, statechart
#   obstacle_nav    Phase 2 glue used by the line task in main.c, plus
#                   the Buddy 5 flight recorder (flight_log.c)
#   obstacle_bench  standalone bench firmware -> build/obstacle_bench.uf2
#                   Buddy 5 + a pretend Navigation; the motors are never driven.
# ---------------------------------------------------------------------------
set(OBSTACLE_WARNINGS
    -Werror
    -Wall
    -Wextra
    -Wparentheses
    -Wfloat-equal
    -Wswitch-default
    -Wimplicit-fallthrough
    -Wshadow
    -Wvla
)

add_library(obstacle STATIC
    src/navigation/obstacle/scan_algo.c
    src/navigation/obstacle/planner.c
    src/navigation/obstacle/statechart.c
    src/navigation/obstacle/comms.c
    src/navigation/obstacle/scan_hw.c
    src/navigation/obstacle/scan_task.c
    src/navigation/obstacle/obstacle.c
)
target_include_directories(obstacle PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(obstacle PUBLIC sys mtk3smp)
target_compile_options(obstacle PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${OBSTACLE_WARNINGS})

add_executable(obstacle_bench
    src/navigation/obstacle/bench_main.c
)
target_include_directories(obstacle_bench PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(obstacle_bench PRIVATE obstacle sys mtk3smp)
target_compile_options(obstacle_bench PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${OBSTACLE_WARNINGS})
target_link_options(obstacle_bench PRIVATE
    -nostartfiles
    -Wl,--gc-sections
    -Wl,--undefined=boot2
    -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld"
)
pico_add_extra_outputs(obstacle_bench)

# Phase 2: main.uf2 runs Buddy 5 through the line task (src/main.c).
add_library(obstacle_nav STATIC
    src/navigation/obstacle/obstacle_nav.c
    src/navigation/obstacle/flight_log.c
)
target_include_directories(obstacle_nav PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(obstacle_nav PUBLIC obstacle motion_service sys mtk3smp)
target_compile_options(obstacle_nav PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${OBSTACLE_WARNINGS})
target_link_libraries(main PRIVATE obstacle_nav)

# Buddy 4: IMU, terrain detection, and bring-up targets
include(${CMAKE_CURRENT_LIST_DIR}/cmake/buddy4.cmake)

target_link_libraries(main PRIVATE buddy4_physics)
