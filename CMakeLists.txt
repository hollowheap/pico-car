cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
set(ENV{PICO_SDK_PATH} "${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk")
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico-car C CXX ASM)

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
    TM_WIFI_JOIN=0
    TM_WIFI_NETIF=0
    TM_WIFI_STATIC=0
    TM_WIFI_DHCP=0
    TM_WIFI_DNS=0
    TM_WIFI_UDP=0
    TM_WIFI_TCP=0
    TM_WIFI_TCPBULK=0
    CYW43_LWIP=0
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
    ${MTK3SMP_PATH}/config
    ${MTK3SMP_PATH}/kernel/knlinc
    ${MTK3SMP_PATH}/kernel/tkernel
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/core/armv6m
    ${MTK3SMP_PATH}/kernel/sysdepend/cpu/rp2040
    ${MTK3SMP_PATH}/kernel/sysdepend/pico_rp2040
    ${MTK3SMP_PATH}/device
    ${MTK3SMP_PATH}/lib/libwifi/sysdepend/pico_rp2040
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
)

target_compile_options(mtk3smp PRIVATE -w -ffunction-sections -fdata-sections)

# Compile the executable
add_library(platform STATIC
    src/platform/isr_dispatcher.c
)
target_include_directories(platform PUBLIC include)
target_link_libraries(platform PUBLIC mtk3smp)

add_library(motion_core STATIC
    src/subsystems/motion/controller.c
    src/subsystems/motion/pid.c
    src/subsystems/motion/odometry.c
    src/subsystems/motion/encoder_math.c
    src/subsystems/motion/guidance.c
    src/subsystems/motion/command_slot.c
)
target_include_directories(motion_core PUBLIC include PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/subsystems/motion)
target_link_libraries(motion_core PUBLIC mtk3smp)

add_library(motion STATIC
    src/subsystems/motion/config.c
    src/subsystems/motion/motor_task.c
    src/subsystems/motion/pwm.c
    src/subsystems/motion/bridge.c
    src/subsystems/motion/encoder_capture.c
    src/subsystems/motion/quadrature.c
)
target_include_directories(motion PUBLIC include PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/subsystems/motion)
target_link_libraries(motion PUBLIC platform motion_core mtk3smp)

add_library(track STATIC
    src/subsystems/track/ir_capture.c
    src/subsystems/track/line_task.c
    src/subsystems/track/line_filter.c
)
target_link_libraries(track PUBLIC platform motion mtk3smp)

add_library(navigation STATIC
    src/subsystems/navigation/mission_task.c
)
target_include_directories(navigation PUBLIC include)
target_link_libraries(navigation PUBLIC motion mtk3smp)

add_executable(main
    src/main.c
    src/demo_tasks.c
    src/system/startup_task.c
)

foreach(app_target IN ITEMS main)
    # Link the uT-Kernel. The USB CDC console is provided by libtm, not pico_stdio.
    target_link_libraries(${app_target} PRIVATE
        platform
        motion
        track
        navigation
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
