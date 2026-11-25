add_library(motion_core STATIC
    app_program/motion/controller.c
    app_program/motion/pid.c
    app_program/motion/odometry.c
    app_program/motion/encoder_math.c
    app_program/motion/motion_request.c
    app_program/motion/motion_inbox.c
)
target_include_directories(motion_core PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(motion_core PUBLIC platform mtk3smp)
target_compile_options(motion_core PRIVATE ${FIRMWARE_WARNINGS})

# The pinned SDK exposes the PIO headers to the C99 encoder adapter.
add_library(motion_encoder_pio STATIC app_program/motion/drv_encoder.c)
target_include_directories(motion_encoder_pio SYSTEM PRIVATE ${PICO_SDK_INCLUDES})
pico_generate_pio_header(motion_encoder_pio ${PROJECT_SOURCE_DIR}/app_program/motion/quadrature.pio)
target_link_libraries(motion_encoder_pio PUBLIC mtk3smp PRIVATE pico_base_headers)
target_compile_options(motion_encoder_pio PRIVATE ${FIRMWARE_WARNINGS})

add_library(motion_service STATIC
    app_program/motion/motion_service.c
    app_program/motion/motor_setup.c
    app_program/motion/drv_pwm.c
    app_program/motion/drv_motor.c
    app_program/motion/encoder.c
    app_program/motion/quadrature.c
)
target_include_directories(motion_service PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(motion_service PUBLIC platform motion_core motion_encoder_pio mtk3smp
    PRIVATE comms)
target_compile_options(motion_service PRIVATE ${FIRMWARE_WARNINGS})


# The default firmware runs this controller from the line task.
add_library(motion_pilot STATIC
    app_program/motion/pilot/pilot.c
    app_program/motion/pilot/avoidance.c
    app_program/motion/pilot/contour.c
    app_program/motion/pilot/drive.c
    app_program/motion/pilot/hug/hug.c
    app_program/motion/pilot/hug/motion.c
    app_program/motion/pilot/hug/scan.c
    app_program/motion/pilot/bypass/bypass.c
    app_program/motion/pilot/bypass/geometry.c
    app_program/motion/pilot/bypass/rejoin.c
    app_program/motion/pilot/look.c
    app_program/motion/pilot/pilot_log.c
)
target_include_directories(motion_pilot PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(motion_pilot PUBLIC ir_line_control platform obstacle)
target_compile_options(motion_pilot PRIVATE ${FIRMWARE_WARNINGS})
