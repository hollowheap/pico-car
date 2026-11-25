add_library(platform STATIC
    app_program/platform/faults.c
    app_program/platform/drv_gpio_irq.c
    app_program/platform/liveness.c
    app_program/platform/log.c
    app_program/platform/log_core.c
    app_program/platform/drv_watchdog.c
)
target_include_directories(platform PUBLIC app_program)
target_link_libraries(platform PUBLIC mtk3smp)
target_compile_options(platform PRIVATE ${FIRMWARE_WARNINGS})
