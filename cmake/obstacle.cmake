# Sonar service and wheel travel adapter.
add_library(obstacle STATIC
    app_program/obstacle/geometry.c
    app_program/obstacle/scan_algo.c
    app_program/obstacle/sonar_health.c
    app_program/obstacle/comms.c
    app_program/obstacle/drv_sonar.c
    app_program/obstacle/scan_task.c
    app_program/obstacle/obstacle.c
)
target_include_directories(obstacle PRIVATE
    ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(obstacle PUBLIC platform mtk3smp)
target_compile_options(obstacle PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})

add_library(obstacle_motion_input STATIC app_program/obstacle/motion_link.c)
target_include_directories(obstacle_motion_input PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(obstacle_motion_input PUBLIC obstacle motion_service platform mtk3smp)
target_compile_options(obstacle_motion_input PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})
