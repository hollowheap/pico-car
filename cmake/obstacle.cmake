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

add_library(obstacle_motion STATIC app_program/obstacle/motion_link.c)
target_include_directories(obstacle_motion PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(obstacle_motion PUBLIC obstacle motion_service platform mtk3smp)
target_compile_options(obstacle_motion PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})

# Original obstacle controller used by main and the scanner bench.
set(OBSTACLE_EXTRAS_SRC ${PROJECT_SOURCE_DIR}/app_program/obstacle/extras)

add_library(obstacle_extras_runtime STATIC
    ${OBSTACLE_EXTRAS_SRC}/comms.c
    ${OBSTACLE_EXTRAS_SRC}/drv_sonar.c
    ${OBSTACLE_EXTRAS_SRC}/extras_start.c
    ${OBSTACLE_EXTRAS_SRC}/flight_log.c
    ${OBSTACLE_EXTRAS_SRC}/planner.c
    ${OBSTACLE_EXTRAS_SRC}/scan.c
    ${OBSTACLE_EXTRAS_SRC}/scan_task.c
    ${OBSTACLE_EXTRAS_SRC}/service.c
    ${OBSTACLE_EXTRAS_SRC}/statechart.c)
target_link_libraries(obstacle_extras_runtime PUBLIC platform mtk3smp)
target_compile_options(obstacle_extras_runtime PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})
